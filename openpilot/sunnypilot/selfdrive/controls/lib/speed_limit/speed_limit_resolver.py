"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE, LIMIT_ADAPT_ACC
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy, OffsetType
import json
import os

# SPRINT20H_LIMIT_BRACKETS: {"brackets": [[limitKphUpTo, offsetKph], ...]} sorted ascending; the first bracket whose
# upper bound is >= the posted limit applies. Example: [[40, 0], [60, 5], [80, 8], [200, 10]].
SONATA_LIMIT_OFFSETS = "/data/sonata_speed_limit_offsets.json"
_sonata_brackets = {"mtime": None, "check": 0.0, "rows": None}


def sonata_bracket_offset_kph(speed_limit_ms):
  now = time.monotonic()
  if now - _sonata_brackets["check"] >= 2.0:
    _sonata_brackets["check"] = now
    try:
      st = os.stat(SONATA_LIMIT_OFFSETS)
      if st.st_mtime != _sonata_brackets["mtime"]:
        _sonata_brackets["mtime"] = st.st_mtime
        with open(SONATA_LIMIT_OFFSETS) as f:
          obj = json.load(f)
        rows = obj.get("brackets") if isinstance(obj, dict) else None
        _sonata_brackets["rows"] = sorted(((float(a), float(b)) for a, b in rows), key=lambda r: r[0]) if isinstance(rows, list) and rows else None
    except Exception:
      _sonata_brackets["rows"] = None
  rows = _sonata_brackets["rows"]
  if not rows or not speed_limit_ms or speed_limit_ms <= 0:
    return None
  kph = float(speed_limit_ms) * CV.MS_TO_KPH
  for upper, offset in rows:
    if kph <= upper + 0.5:
      return max(-20.0, min(20.0, offset))
  return max(-20.0, min(20.0, rows[-1][1]))

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ALL_SOURCES = tuple(SpeedLimitSource.schema.enumerants.values())



import json  # SPRINT23E_ROAD_LIMIT
import os

# SPRINT23E_ROAD_LIMIT: guidance publishes the nearest OSM way's maxspeed (or a class default) in route_guidance_state.json
# ("roadLimit"); it stands in for the map source when mapd reports nothing (#190: five streets at 0).
SONATA_ROAD_LIMIT_STATE = "/data/sonata_drive10_lab/route_guidance_state.json"
SONATA_ROAD_LIMIT_STALE_S = 45.0


class SonataRoadLimit:
  def __init__(self, path=SONATA_ROAD_LIMIT_STATE):
    self.path = path
    self._mtime = None
    self._check = 0.0
    self.kph = None
    self.info = None

  def value_ms(self, road_name: str = "") -> float:   # SPRINT23B_ROAD_LIMIT_V2: prefer the candidate named like mapd's road
    now = time.monotonic()
    if now - self._check >= 1.0:
      self._check = now
      try:
        st = os.stat(self.path)
        if st.st_mtime != self._mtime:
          self._mtime = st.st_mtime
          with open(self.path) as f:
            s = json.load(f)
          rl = s.get("roadLimit") if isinstance(s, dict) else None
          self.info = rl if isinstance(rl, dict) else None
          self.kph = rl.get("kph") if isinstance(rl, dict) and isinstance(rl.get("kph"), (int, float)) else None
          self._cands = rl.get("candidates") if isinstance(rl, dict) and isinstance(rl.get("candidates"), list) else []
        if time.time() - st.st_mtime > SONATA_ROAD_LIMIT_STALE_S:
          self.kph = None
      except Exception:
        self.kph, self.info, self._cands = None, None, []
    kph = self.kph
    rn = str(road_name or "").strip().lower()
    if rn and self.kph is not None or rn and getattr(self, "_cands", None):
      for c in getattr(self, "_cands", []) or []:
        if isinstance(c, dict) and str(c.get("name") or "").strip().lower() == rn:
          kph = c.get("kph") if isinstance(c.get("kph"), (int, float)) else None
          break
    return float(kph) / 3.6 if kph else 0.0


sonata_road_limit = SonataRoadLimit()


class SpeedLimitResolver:
  limit_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  distance_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: custom.LongitudinalPlanSP.SpeedLimit.Source
  speed_limit_offset: float

  def __init__(self):
    self.params = Params()
    self.frame = -1

    self._gps_location_service = get_gps_location_service(self.params)
    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
    self.policy = get_sanitize_int_param(
      "SpeedLimitPolicy",
      Policy.min().value,
      Policy.max().value,
      self.params
    )
    self._policy_to_sources_map = {
      Policy.car_state_only: [SpeedLimitSource.car],
      Policy.map_data_only: [SpeedLimitSource.map],
      Policy.car_state_priority: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.map_data_priority: [SpeedLimitSource.map, SpeedLimitSource.car],
      Policy.combined: [SpeedLimitSource.car, SpeedLimitSource.map],
    }
    self.source = SpeedLimitSource.none
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)

    self.is_metric = self.params.get_bool("IsMetric")
    self.offset_type = get_sanitize_int_param(
      "SpeedLimitOffsetType",
      OffsetType.min().value,
      OffsetType.max().value,
      self.params
    )
    self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

    self.speed_limit = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    if self.speed_limit > 0.:
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final

  @property
  def speed_limit_valid(self) -> bool:
    return self.speed_limit > 0.

  @property
  def speed_limit_last_valid(self) -> bool:
    return self.speed_limit_last > 0.

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
      self.is_metric = self.params.get_bool("IsMetric")
      self.offset_type = self.params.get("SpeedLimitOffsetType", return_default=True)
      self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def _get_speed_limit_offset(self) -> float:
    if self.offset_type == OffsetType.off:
      return 0
    elif self.offset_type == OffsetType.fixed:
      # SPRINT20H_LIMIT_BRACKETS: per-bracket offsets from /data/sonata_speed_limit_offsets.json when present.
      bracket = sonata_bracket_offset_kph(self.speed_limit)
      if bracket is not None:
        return float(bracket * CV.KPH_TO_MS)
      return float(self.offset_value * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    elif self.offset_type == OffsetType.percentage:
      return float(self.offset_value * 0.01 * self.speed_limit)
    else:
      raise NotImplementedError("Offset not supported")

  def _reset_limit_sources(self, source: custom.LongitudinalPlanSP.SpeedLimit.Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    self.limit_solutions[SpeedLimitSource.car] = sm['carStateSP'].speedLimit
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_map_data(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._process_map_data(sm)

  def _process_map_data(self, sm: messaging.SubMaster) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    gps_fix_age = time.monotonic() - gps_data.unixTimestampMillis * 1e-3
    if gps_fix_age > LIMIT_MAX_MAP_DATA_AGE:
      return

    speed_limit = map_data.speedLimit if map_data.speedLimitValid else 0.
    next_speed_limit = map_data.speedLimitAhead if map_data.speedLimitAheadValid else 0.

    self._calculate_map_data_limits(sm, speed_limit, next_speed_limit)

  def _calculate_map_data_limits(self, sm: messaging.SubMaster, speed_limit: float, next_speed_limit: float) -> None:
    gps_data = sm[self._gps_location_service]
    map_data = sm['liveMapDataSP']

    distance_since_fix = self.v_ego * (time.monotonic() - gps_data.unixTimestampMillis * 1e-3)
    distance_to_speed_limit_ahead = max(0., map_data.speedLimitAheadDistance - distance_since_fix)

    self.limit_solutions[SpeedLimitSource.map] = speed_limit
    self.distance_solutions[SpeedLimitSource.map] = 0.

    # FIXME-SP: this is not working as expected
    if 0. < next_speed_limit < self.v_ego:
      adapt_time = (next_speed_limit - self.v_ego) / LIMIT_ADAPT_ACC
      adapt_distance = self.v_ego * adapt_time + 0.5 * LIMIT_ADAPT_ACC * adapt_time ** 2

      if distance_to_speed_limit_ahead <= adapt_distance:
        self.limit_solutions[SpeedLimitSource.map] = next_speed_limit
        self.distance_solutions[SpeedLimitSource.map] = distance_to_speed_limit_ahead

  def _get_source_solution_according_to_policy(self) -> custom.LongitudinalPlanSP.SpeedLimit.Source:
    sources_for_policy = self._policy_to_sources_map[Policy(self.policy)]

    if Policy(self.policy) != Policy.combined:
      # They are ordered in the order of preference, so we pick the first that's non-zero
      for source in sources_for_policy:
        if self.limit_solutions[source] > 0.:
          return source
      return SpeedLimitSource.none

    sources_with_limits = [(s, limit) for s, limit in [(s, self.limit_solutions[s]) for s in sources_for_policy] if limit > 0.]
    if sources_with_limits:
      return min(sources_with_limits, key=lambda x: x[1])[0]

    return SpeedLimitSource.none

  def _resolve_limit_sources(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Get limit solutions from each data source"""
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)
    if self.limit_solutions[SpeedLimitSource.map] <= 0.:   # SPRINT23E_ROAD_LIMIT: OSM way limit / class default as the map source
      _rl = sonata_road_limit.value_ms(sm['liveMapDataSP'].roadName if sm.valid['liveMapDataSP'] or sm.updated['liveMapDataSP'] else "")
      if _rl > 0.:
        self.limit_solutions[SpeedLimitSource.map] = _rl
        self.distance_solutions[SpeedLimitSource.map] = 0.

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    return speed_limit, distance, source

  def update(self, v_ego: float, sm: messaging.SubMaster) -> None:
    self.v_ego = v_ego
    self.update_params()

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    self.update_speed_limit_states()

    self.frame += 1
