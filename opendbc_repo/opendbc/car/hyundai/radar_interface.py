import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
import os   # SPRINT31AU_SONATA_RADAR_ACTIVE

# SPRINT31AU_SONATA_RADAR_ACTIVE: the 2025 Sonata broadcasts 32 radar objects on bus 1 (0x3A5-0x3C4, 24 bytes, 50 ms per id).
# Validated on this car 2026-09-08 (Sprint 19a). With the flag file present they are appended to the RadarData
# points the camera-SCC ext path already produces, so radard can fuse real tracks with the vision leads.
SONATA_RADAR_FLAG = "/data/sonata_radar_active"
SONATA_RADAR_DBC = "hyundai_radar_3a5_3c4"
SONATA_RADAR_START = 0x3A5
SONATA_RADAR_COUNT = 32
SONATA_RADAR_BUS = 1          # ACAN on this harness (CanBus(CP).ACAN == 1)
SONATA_RADAR_FREQ = 20        # Hz per track id, measured 2026-09-12 (median 50.1 ms)
SONATA_RADAR_PLATFORM = "HYUNDAI_SONATA_2024"
SONATA_RADAR_MIN_RANGE_M = 0.5   # SPRINT32P_ZERO_RANGE


def sonata_radar_active(CP) -> bool:
  try:
    return str(CP.carFingerprint) == SONATA_RADAR_PLATFORM and os.path.exists(SONATA_RADAR_FLAG)
  except Exception:
    return False


def get_sonata_radar_parser():
  messages = [(f"RADAR_TRACK_{addr:x}", SONATA_RADAR_FREQ) for addr in range(SONATA_RADAR_START, SONATA_RADAR_START + SONATA_RADAR_COUNT)]
  return CANParser(SONATA_RADAR_DBC, messages, SONATA_RADAR_BUS)

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)
    # SPRINT31AU_SONATA_RADAR_ACTIVE: flag absent -> None, and nothing below this line changes behaviour
    self.sonata_rcp = get_sonata_radar_parser() if sonata_radar_active(CP) else None
    self.sonata_pts = {}
    self.sonata_track_id = 100000   # well clear of the ext / stock ids
    # SPRINT35B_YREL_SIGN: radard scores yRel against -lead.y, so yRel must be LEFT-positive = +LAT_DIST.
    # /data/sonata_radar_yrel_legacy restores the 31au sign (-LAT_DIST); read once, at interface build.
    self.sonata_yrel_sign = -1.0 if os.path.exists("/data/sonata_radar_yrel_legacy") else 1.0

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    if self.sonata_rcp is not None:   # SPRINT31AU_SONATA_RADAR_ACTIVE
      self.sonata_rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      ret = self.update_ext(ret)
      if self.sonata_rcp is not None:   # SPRINT31AU_SONATA_RADAR_ACTIVE: SCC point kept as the fallback, tracks appended
        self.sonata_update(ret)
        # SPRINT32R_OWNED_POINTS: ONE assignment from the OWNED point objects. Reassigning a capnp list from elements that
        # alias its own storage reads them back as zeros (proven on the car 2026-09-14): 31au's
        # `list(ret.points) + sonata` zeroed the SCC point every cycle (the phantom 0 m lead behind FCW), and the 32p
        # filter over ret.points zeroed every point. The ext path's self.pts and our self.sonata_pts are stable.
        owned = list(self.pts.values()) + list(self.sonata_pts.values())
        ret.points = [p for p in owned if p.dRel >= SONATA_RADAR_MIN_RANGE_M]
      return ret

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def sonata_update(self, ret):
    """SPRINT31AU_SONATA_RADAR_ACTIVE: append every measured (3) / coasted (4) track of the 0x3A5-0x3C4 family.

    dRel = LONG_DIST (radard subtracts RADAR_TO_CAMERA itself), yRel = +LAT_DIST (SPRINT35B_YREL_SIGN: LAT is
    left-positive and so is radard's yRel - match_vision_to_track compares it with -lead.y), vRel = REL_SPEED. Track ids are
    stable per address while the track stays alive, as the stock Hyundai interface does. If our parser is not
    valid this cycle we contribute nothing - the SCC point path above is untouched."""
    if not self.sonata_rcp.can_valid:
      return
    for addr in range(SONATA_RADAR_START, SONATA_RADAR_START + SONATA_RADAR_COUNT):
      msg = self.sonata_rcp.vl[f"RADAR_TRACK_{addr:x}"]
      if msg["STATE"] in (3, 4):
        if addr not in self.sonata_pts:
          pt = structs.RadarData.RadarPoint()
          pt.trackId = self.sonata_track_id
          self.sonata_track_id += 1
          self.sonata_pts[addr] = pt
        pt = self.sonata_pts[addr]
        pt.dRel = float(msg["LONG_DIST"])
        pt.yRel = self.sonata_yrel_sign * float(msg["LAT_DIST"])   # SPRINT35B_YREL_SIGN (was -LAT_DIST)
        pt.vRel = float(msg["REL_SPEED"])
      else:
        self.sonata_pts.pop(addr, None)
    # SPRINT32R_OWNED_POINTS: publication happens once, in _update, from owned objects (see there)
