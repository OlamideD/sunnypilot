#!/usr/bin/env python3
import json
import math
import os
import time
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc  # SPRINT20B_TRAFFIC_MODE
SONATA_STOP_COMMIT_SPEED = 1.20  # SPRINT16H_STOP_LATCH: #176 crept past the line at 0.82 m/s (was 0.70)
SONATA_STOP_COMMIT_TIME = 0.20
SONATA_STOP_COMPLETE_SPEED = 0.12
SONATA_STOP_HOLD_TIME = 1.0
SONATA_STOP_RELEASE_CLEAR_TIME = 0.80

# SPRINT15B_E2E_BRAKE_ADMISSION / SPRINT15B_STOP_SIGN_COMPLETION
# SPRINT16G_EARLIER_ADMISSION: the first firm model brake request is acted on ~1 s earlier (#153 red lights).
SONATA_E2E_ADMIT_ACCEL = -0.6          # m/s^2: model braking request strong enough to be admitted in ACC mode (was -0.8)
SONATA_E2E_ADMIT_ENDPOINT_MIN = 50.0   # m: model path endpoint closer than this is stop-like (was 40)
SONATA_E2E_ADMIT_ENDPOINT_TIME = 4.5   # s of travel: endpoint closer than v*this is stop-like (was 3.5)
SONATA_E2E_ADMIT_VMIN_RATIO = 0.5      # horizon speed dropping below this fraction of v_ego is stop-like
SONATA_STOP_SIGN_SCENE = '/data/sonata_telemetry/live_scene.json'
SONATA_STOP_SIGN_FRESH_S = 2.5         # s a source-qualified OEM Stop observation stays usable
SONATA_STOP_SIGN_ENDPOINT_MIN = 45.0   # m
SONATA_STOP_SIGN_ENDPOINT_TIME = 4.0   # s of travel
SONATA_STOP_SIGN_MIN_DECEL = -3.0      # m/s^2 hardest completion decel
SONATA_STOP_SIGN_MARGIN = 2.0          # m before the model stop point
SONATA_STOP_SIGN_COOLDOWN_S = 6.0
SONATA_PLANNER_LIVE = '/data/sonata_telemetry/planner_live.json'

# SPRINT15D_CURVE_PREP_FLOOR: deeper cruise deceleration while Vision Curve prepares for a bend
# whose target is far below the current speed (3 s of warning must be enough on unlit roads).
SONATA_CURVE_PREP_MIN_ACCEL = -2.2     # m/s^2 deepest floor
SONATA_CURVE_PREP_SHARP_ACCEL = -2.8   # SPRINT25B_SHARP_CURVE: deeper floor for a genuinely sharp bend (low target)
SONATA_CURVE_PREP_SHARP_V = 8.0        # m/s (~29 km/h): a curve target at/under this is a sharp bend (#193 t=92 was ~26)
M17_LANE_CONF = 0.25                   # SPRINT27A_M17: ego-lane line confidence below this = unmarked/low-confidence bend
SONATA_LANE_CONF = [1.0]               # SPRINT27A_M17: min ego-lane line probability, refreshed each frame
SONATA_CURVE_PREP_GAP_BP = [2.0, 6.0]  # m/s of (v_ego - curve target)
SONATA_CRUISE_MIN = [A_CRUISE_MIN]     # mutable floor read by get_cruise_accel

# SPRINT16A_STOP_HOLD: a committed stop is completed, not coasted. Near standstill the model asks
# for only -0.05..-0.15 m/s^2, which never armed the latch and left the car creeping at 0.2-0.6 m/s.
SONATA_STOP_HOLD_DECEL = -0.8          # m/s^2 commanded while latched and still rolling (SPRINT16H_STOP_LATCH, was -0.5)
SONATA_STOP_LATCH_ACCEL = -0.30        # model brake request that counts as stop intent near standstill
SONATA_STOP_LEAD_CLOSE_M = 15.0        # a lead this close owns the stop (mpc), not the latch

# SPRINT16C_ROUTE_PREP: route-aware speed preparation for the next turn maneuver (roadmap P3/P8).
# Reads the advisory guidance state written by /data/sonata-route-guidance.py; only ever lowers
# the cruise target, only on an ACTIVE route with fresh GPS, only within SONATA_ROUTE_MAX_DIST_M.
SONATA_ROUTE_STATE = '/data/sonata_drive10_lab/route_guidance_state.json'
SONATA_ROUTE_STALE_S = 4.0             # s since the guidance file was written
SONATA_ROUTE_GPS_MAX_AGE_S = 3.0
SONATA_ROUTE_MAX_DIST_M = 400.0   # SPRINT33S_TURN_ANTICIPATION: was 250, too short for a gentle approach
SONATA_ROUTE_OFFROUTE_M = 35.0
SONATA_ROUTE_NODE_MARGIN_M = 12.0      # reach the turn speed this far before the maneuver node
SONATA_ROUTE_PREP_DECEL = 0.9          # m/s^2 comfortable approach deceleration
SONATA_ROUTE_MIN_V = 5.0               # m/s: the planner never asks for less than this for a turn
# SPRINT18C_SIGNAL_PREP: the next traffic signal / stop sign AHEAD on the active route (guidance
# `nextTrafficControl`, from Mapbox intersection flags + OpenStreetMap nodes) caps the cruise target so
# the car reaches a speed the driver (or the E2E model) can complete a stop from. Same gates as the
# route-turn cap (ACTIVE route, fresh GPS, on route), lower-only, never a brake on its own.
SONATA_SIGNAL_PREP_OFF = '/data/sonata_signal_prep_off'   # existence disables the cap (badge stays)
SONATA_SIGNAL_MAX_DIST_M = 160.0   # SPRINT26D: begin the stop-sign slow-down as soon as the sign is known (was 100)
SONATA_STOP_PREP_DECEL = 0.7       # SPRINT26D: gentler single smooth profile to the sign (was the 0.9 shared decel)
SONATA_SIGNAL_NODE_MARGIN_M = 8.0     # reach the prep speed this far before the control
SONATA_SIGNAL_V = {'stop': 2.5}   # m/s at the control (9 km/h; SPRINT23D was 4.0). SPRINT20Q: no cap at signals (no colour perception)
# SPRINT23D_MAP_STOP_LATCH: the near-standstill latch also commits when the map stop sign is this close and slow.
SONATA_MAP_STOP_LATCH_M = 10.0
SONATA_MAP_STOP_LATCH_V = 3.5
SONATA_MAP_STOP_COOLDOWN_S = 8.0
# SPRINT23C_ARRIVAL: ease to walking pace at the destination (lower-only cruise cap from SONATA_ARRIVAL_DIST_M).
SONATA_ARRIVAL_DIST_M = 250.0   # SPRINT32M_ARRIVAL: was 150 - the 25a ramp was active but too shallow to feel (route 000000b6)
SONATA_ARRIVAL_DECEL = 0.5      # SPRINT32M_ARRIVAL: m/s^2 - 50 km/h at 200 m, 35 at 100 m, 17 at 30 m, walking pace at 12 m
SONATA_ARRIVAL_END_V = 2.0
SONATA_ARRIVAL_MARGIN_M = 12.0
SONATA_ARRIVAL_PASSED_M = 60.0
# SPRINT25A_ARRIVAL: monotonic creep-stop at the destination (fixes #193 off-road-pin release/30 km/h floor).
SONATA_ARRIVAL_STOP_V = 1.2          # m/s (~4 km/h) creep target once the destination stop is committed
SONATA_ARRIVAL_COMMIT_V = 6.5        # m/s (~23 km/h): commit the stop hold only when actually arriving this slow
SONATA_TURN_ANTICIPATE_DECEL = 0.45  # SPRINT33S_TURN_ANTICIPATION: the early, smooth envelope.
# The cap takes min(firm, anticipate), so it is never higher or later than before - only earlier.
SONATA_TURN_PREP_DECEL = 1.00        # SPRINT29B_TURN_PREP: was 0.75, which put the cap below 88 km/h a full
                                     # 250 m before a turn (owner #196: 'slowing way, way behind, still at 88').
                                     # Low grip still scales this by 0.75 -> 0.75 m/s^2, i.e. winter behaviour
                                     # is exactly today's dry-road behaviour; only dry roads start later.
SONATA_GRIP_PREP_DECEL_SCALE = 0.75  # low-grip: soften the arrival/turn approach decel (Canadian winter)
SONATA_GRIP_LIVE_FILE_RP = '/data/sonata_telemetry/grip_live.json'
SONATA_ROUTE_TURN_V = (('uturn', 4.5), ('sharp', 5.5), ('slight', 9.0), ('turn', 6.5),
                       ('end_of_road', 6.5), ('roundabout', 8.0), ('rotary', 8.0))   # SPRINT29B_TURN_PREP:
# roundabouts were 6.5 m/s = 23.4 km/h - the '23' the owner read off the dash in a 30 zone. 8.0 = 28.8 km/h.
# 'turn'/'end_of_road' stay at 6.5: those are 90-degree turns where 23 km/h is correct.


SONATA_LOW_GRIP_RP = [False]  # SPRINT25A: low-grip flag shared into the route/arrival prep decel


def sonata_route_turn_speed(maneuver):
  """Target speed (m/s) at the maneuver node for a route maneuver name, or None when not a turn."""
  name = str(maneuver or '').lower()
  if not name:
    return None
  for key, v in SONATA_ROUTE_TURN_V:
    if key in name:
      return v
  return None


def sonata_route_prep_target(maneuver, dist_m, v_ego):
  """Allowed speed now (m/s) so the car reaches the turn speed at the node, or None."""
  try:
    v_turn = sonata_route_turn_speed(maneuver)
    if v_turn is None or dist_m is None or v_ego is None:
      return None
    d = float(dist_m)
    if not (0.0 <= d <= SONATA_ROUTE_MAX_DIST_M) or float(v_ego) < 3.0:
      return None
    d_eff = max(d - SONATA_ROUTE_NODE_MARGIN_M, 0.0)
    _rp_decel = SONATA_TURN_PREP_DECEL * (SONATA_GRIP_PREP_DECEL_SCALE if SONATA_LOW_GRIP_RP[0] else 1.0)  # SPRINT25A
    # SPRINT33S_TURN_ANTICIPATION: the firm envelope stays as the floor-of-last-resort; the gentle
    # one is what the driver actually feels, and min() guarantees we never ask for MORE speed.
    _ant_decel = SONATA_TURN_ANTICIPATE_DECEL * (SONATA_GRIP_PREP_DECEL_SCALE if SONATA_LOW_GRIP_RP[0] else 1.0)
    v_allowed = min((v_turn * v_turn + 2.0 * _rp_decel * d_eff) ** 0.5,
                    (v_turn * v_turn + 2.0 * _ant_decel * d_eff) ** 0.5)
    return max(float(v_allowed), SONATA_ROUTE_MIN_V)
  except Exception:
    return None


def sonata_signal_prep_target(kind, dist_m, v_ego):
  """Allowed speed now (m/s) so the car reaches the control's prep speed at the control, or None."""
  try:
    v_min = SONATA_SIGNAL_V.get(str(kind or ''))
    if v_min is None or dist_m is None or v_ego is None:
      return None
    d = float(dist_m)
    if not (0.0 <= d <= SONATA_SIGNAL_MAX_DIST_M) or float(v_ego) < 3.0:
      return None
    d_eff = max(d - SONATA_SIGNAL_NODE_MARGIN_M, 0.0)
    _sp_decel = SONATA_STOP_PREP_DECEL * (SONATA_GRIP_PREP_DECEL_SCALE if SONATA_LOW_GRIP_RP[0] else 1.0)  # SPRINT26D
    return max(float((v_min * v_min + 2.0 * _sp_decel * d_eff) ** 0.5), v_min)
  except Exception:
    return None


# SPRINT35_STOP_CAP_CONTINUITY: keep the stop-sign envelope below 3 m/s until the stop is served.
# See tools/sonata/sprint35/stops/hotfix_35_stop_cap_continuity.py.
SONATA_STOP_CAP_CONT_OFF = '/data/sonata_stop_cap_continuity_off'
SONATA_STOP_SERVED_V = 0.3          # m/s: the car has come to rest for this stop
SONATA_STOP_SERVED_M = 30.0         # ... within this distance of the stop control
SONATA_STOP_NEW_CTRL_JUMP_M = 15.0  # the control distance jumping up this much = a different control
SONATA_STOP_GONE_S = 2.0            # no stop control for this long = forget the served flag


def sonata_stop_cap_low_speed(kind, dist_m, v_ego):
  """The stop envelope of sonata_signal_prep_target for v_ego < 3 m/s only, or None.

  Identical formula, so the cap is continuous across 3 m/s; at or above 3 m/s this returns None and the original
  function (unchanged) is the one that answers.
  """
  try:
    if str(kind or '') != 'stop' or dist_m is None or v_ego is None:
      return None
    d = float(dist_m)
    if not (0.0 <= d <= SONATA_SIGNAL_MAX_DIST_M) or float(v_ego) >= 3.0:
      return None
    v_min = SONATA_SIGNAL_V['stop']
    d_eff = max(d - SONATA_SIGNAL_NODE_MARGIN_M, 0.0)
    _sp_decel = SONATA_STOP_PREP_DECEL * (SONATA_GRIP_PREP_DECEL_SCALE if SONATA_LOW_GRIP_RP[0] else 1.0)
    return max(float((v_min * v_min + 2.0 * _sp_decel * d_eff) ** 0.5), v_min)
  except Exception:
    return None


# SPRINT19_LANE_CHANGE_LONG: while the runtime runs a lane change, the first car in the TARGET lane (passive radar,
# /data/sonata_telemetry/radar_live.json) becomes a lower-only cruise cap so the car does not accelerate into it, and
# the Sonata route/signal preparation caps are frozen so a lane change never gets a surprise deceleration of ours.
SONATA_RADAR_LIVE = '/data/sonata_telemetry/radar_live.json'
SONATA_LC_TARGET_GAP_T = 2.0          # s: target-lane lead closer than this (in travel time) sets the cap
SONATA_LC_MIN_V = 5.0                 # m/s floor for the cap


# SPRINT20B_TRAFFIC_MODE: personality file -> long_mpc.SONATA_PERSONALITY (same process) + acceleration profile.
# /data/sonata_personality.json also carries "accelProfile": "eco" | "normal" | "sport" and "trafficMode": {"auto": true,
# "tFollow": 1.0, "jerk": 0.6, "force": false}. Traffic Mode (auto) engages after 8 s below 30 km/h with a lead closer
# than 25 m and releases 10 s after the conditions clear.
SONATA_PERSONALITY_FILE = '/data/sonata_personality.json'
SONATA_ACCEL_PROFILE_SCALE = {'eco': 0.8, 'normal': 1.0, 'sport': 1.25}
SONATA_TRAFFIC_V_MAX = 8.3         # m/s (30 km/h)
SONATA_TRAFFIC_LEAD_MAX_M = 25.0
SONATA_TRAFFIC_ENGAGE_S = 8.0
SONATA_TRAFFIC_RELEASE_S = 10.0
SONATA_GRIP_LIVE_FILE = '/data/sonata_telemetry/grip_live.json'   # SPRINT20I_LOW_GRIP
SONATA_GRIP_ACCEL_SCALE = 0.7
SONATA_GRIP_T_FOLLOW_ADD = 0.5


class SonataPersonalityFile:
  def __init__(self, path=SONATA_PERSONALITY_FILE):
    self.path = path
    self._mtime = None
    self._check = 0.0
    self.cfg = {}
    self.traffic_since = None
    self.clear_since = None
    self.traffic_active = False
    self.info = {'accelProfile': 'normal', 'trafficActive': False}
    self._grip_mtime = None
    self._grip_check = 0.0
    self.low_grip = False

  def _poll_grip(self, now):
    if now - self._grip_check < 1.0:
      return
    self._grip_check = now
    try:
      st = os.stat(SONATA_GRIP_LIVE_FILE)
      if st.st_mtime != self._grip_mtime:
        self._grip_mtime = st.st_mtime
        with open(SONATA_GRIP_LIVE_FILE) as f:
          obj = json.load(f)
        self.low_grip = bool(isinstance(obj, dict) and obj.get('lowGrip'))
      if time.time() - st.st_mtime > 30.0:
        self.low_grip = False
    except Exception:
      self.low_grip = False

  def _poll(self, now):
    if now - self._check < 1.0:
      return
    self._check = now
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          obj = json.load(f)
        self.cfg = obj if isinstance(obj, dict) else {}
    except Exception:
      self.cfg = {}

  def update(self, now, v_ego, lead_d_rel, lead_present):
    self._poll(now)
    self._poll_grip(now)
    tm = self.cfg.get('trafficMode') if isinstance(self.cfg.get('trafficMode'), dict) else {}
    dense = bool(lead_present and v_ego < SONATA_TRAFFIC_V_MAX and lead_d_rel is not None and lead_d_rel < SONATA_TRAFFIC_LEAD_MAX_M)
    if tm.get('force'):
      self.traffic_active = True
    elif tm.get('auto', False):
      if dense:
        self.traffic_since = self.traffic_since or now
        self.clear_since = None
        if now - self.traffic_since >= SONATA_TRAFFIC_ENGAGE_S:
          self.traffic_active = True
      else:
        self.traffic_since = None
        self.clear_since = self.clear_since or now
        if now - self.clear_since >= SONATA_TRAFFIC_RELEASE_S:
          self.traffic_active = False
    else:
      self.traffic_active = False
    profile = str(self.cfg.get('accelProfile', 'normal')).lower()
    if profile not in SONATA_ACCEL_PROFILE_SCALE:
      profile = 'normal'
    try:
      long_mpc.SONATA_PERSONALITY.update({'tFollow': self.cfg.get('tFollow') or {}, 'jerk': self.cfg.get('jerk') or {},
                                          'trafficMode': tm, 'trafficActive': self.traffic_active,
                                          'tFollowAdd': SONATA_GRIP_T_FOLLOW_ADD if self.low_grip else 0.0})
    except Exception:
      pass
    scale = SONATA_ACCEL_PROFILE_SCALE[profile]
    if self.low_grip:
      scale = min(scale, SONATA_GRIP_ACCEL_SCALE)
    self.info = {'accelProfile': profile, 'trafficActive': self.traffic_active, 'tFollow': self.cfg.get('tFollow'),
                 'trafficMode': tm or None, 'lowGrip': self.low_grip, 'accelScale': scale}
    return scale


class SonataLaneChangeLong:
  def __init__(self, path=SONATA_RADAR_LIVE):
    self.path = path
    self._mtime = None
    self._check = 0.0
    self.radar = {}
    self.info = {'state': 'off', 'vTarget': None}

  def _poll(self, now):
    if now - self._check < 0.1:
      return
    self._check = now
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          obj = json.load(f)
        self.radar = obj if isinstance(obj, dict) else {}
      if time.time() - st.st_mtime > 1.0:
        self.radar = {}
    except Exception:
      self.radar = {}

  def update(self, now, meta, v_ego):
    """Returns (in_lane_change, v_cap or None). meta = modelV2.meta."""
    try:
      state = str(meta.laneChangeState).split('.')[-1].lower()
      direction = str(meta.laneChangeDirection).split('.')[-1].lower()
    except Exception:
      state, direction = 'off', 'none'
    active = state in ('prelanechange', 'lanechangestarting') and direction in ('left', 'right')
    v_cap = None
    lane = None
    if active:
      self._poll(now)
      lane = ((self.radar.get('lanes') or {}).get(direction) or {})
      gap = lane.get('frontGap')
      if isinstance(gap, (int, float)) and v_ego > 3.0 and gap < SONATA_LC_TARGET_GAP_T * v_ego:
        v_cap = max(float(v_ego + float(lane.get('frontVrel') or 0.0)), SONATA_LC_MIN_V)
    self.info = {'state': state, 'direction': direction, 'vTarget': v_cap,
                 'targetGap': lane.get('frontGap') if lane else None, 'targetVrel': lane.get('frontVrel') if lane else None,
                 'prepFrozen': state == 'lanechangestarting'}
    return state == 'lanechangestarting', v_cap


# SPRINT36M1_END_OF_ROAD: consumer of the guidance `sonataMapPrep` caps (Sprint 36m map/scene speed preparation:
# end of road, service roads / parking, ramps, downhill, amber, learned stops). A cap {distanceM, vEndMps, decel,
# marginM} allows v = sqrt(vEnd^2 + 2*decel*max(d - margin, 0)) now; the lowest joins the SonataRoutePrep MIN.
# LOWER-ONLY: it can never raise a target; vEnd >= SONATA_36M_MIN_V so it never asks for a stop; the caps never enter
# signal_info, so the map-stop latch cannot fire on them. Kill switches /data/sonata_36mN_off are honoured here too.
SONATA_36M_SCHEMA = 'sonata-map-prep-v1'
SONATA_36M_OFF_FMT = '/data/sonata_%s_off'
SONATA_36M_FEATURES = ('36m1', '36m2', '36m3', '36m4', '36m5', '36m6')
SONATA_36M_MAX_DIST_M = 600.0
SONATA_36M_MIN_V = 0.5
SONATA_36M_DECEL_MIN = 0.2
SONATA_36M_DECEL_MAX = 3.0


def sonata_36m_cap_target(cap, v_ego, age_s=0.0, low_grip=False):
  """PURE. Allowed speed now (m/s) for one map-prep cap, or None when the cap is malformed / out of range."""
  try:
    d = float(cap['distanceM'])
    v_end = float(cap['vEndMps'])
    a = float(cap['decel'])
    m = float(cap.get('marginM') or 0.0)
    v = max(float(v_ego), 0.0)
    age = min(max(float(age_s), 0.0), SONATA_ROUTE_STALE_S)
  except Exception:
    return None
  if not all(math.isfinite(x) for x in (d, v_end, a, m, v, age)) or a <= 0.0 or v_end <= 0.0:
    return None
  v_end = max(v_end, SONATA_36M_MIN_V)
  a = min(max(a, SONATA_36M_DECEL_MIN), SONATA_36M_DECEL_MAX) * (SONATA_GRIP_PREP_DECEL_SCALE if low_grip else 1.0)
  d = d - v * age                  # the guidance record is up to SONATA_ROUTE_STALE_S old: we are that much closer
  if d > SONATA_36M_MAX_DIST_M:
    return None
  d_eff = max(d - max(m, 0.0), 0.0)
  return max((v_end * v_end + 2.0 * a * d_eff) ** 0.5, v_end)


def sonata_36m_consume(rp, now, v_ego, s):
  """SPRINT36M1: the lowest map-prep cap for this frame (m/s) or None. Writes rp.info['mapPrep']. Never raises."""
  st = getattr(rp, '_s36m', None)
  if st is None:
    st = rp._s36m = {'off_t': -1e9, 'off': frozenset()}
  info = {'active': False, 'vTarget': None, 'reason': 'no map prep', 'feature': None, 'kind': None, 'distM': None, 'n': 0}
  best = None
  try:
    mp = s.get('sonataMapPrep') if isinstance(s, dict) else None
    if not isinstance(mp, dict) or mp.get('schema') != SONATA_36M_SCHEMA:
      pass
    elif rp._file_age > SONATA_ROUTE_STALE_S:
      info['reason'] = 'stale'
    elif not isinstance(s.get('gpsAgeS'), (int, float)) or s.get('gpsAgeS') > SONATA_ROUTE_GPS_MAX_AGE_S:
      info['reason'] = 'gps_age'
    else:
      if now - st['off_t'] >= 1.0:
        st['off_t'] = now
        st['off'] = frozenset(f for f in SONATA_36M_FEATURES if os.path.exists(SONATA_36M_OFF_FMT % f))
      caps = [c for c in (mp.get('caps') or []) if isinstance(c, dict)]
      info['n'] = len(caps)
      info['reason'] = 'no cap'
      for c in caps:
        if c.get('feature') not in SONATA_36M_FEATURES or c.get('feature') in st['off']:
          continue
        v = sonata_36m_cap_target(c, v_ego, rp._file_age, bool(getattr(rp, '_low_grip', False)))
        if v is not None and (best is None or v < best[0]):
          best = (v, c)
      if best is not None:
        info.update(active=True, vTarget=best[0], reason='ok', feature=best[1].get('feature'),
                    kind=best[1].get('kind'), distM=best[1].get('distanceM'))
  except Exception:
    best = None
    info['reason'] = 'error'
  try:
    rp.info['mapPrep'] = info
  except Exception:
    pass
  return best[0] if best is not None else None


class SonataRoutePrep:
  """Polls the guidance state (<= 4 Hz) and returns the route speed cap for this frame."""
  def __init__(self, path=SONATA_ROUTE_STATE):
    self.path = path
    self.state = {}
    self._mtime = None
    self._check = 0.0
    self._file_age = float('inf')
    self.info = {'active': False, 'reason': 'init'}
    self.signal_info = {'active': False, 'reason': 'init'}  # SPRINT18C_SIGNAL_PREP
    self.arrival_info = {'active': False, 'reason': 'init'}  # SPRINT23C_ARRIVAL
    self._signal_off = False
    self._arrival_committed = False   # SPRINT25A_ARRIVAL
    self._arrival_d_min = float('inf')
    self._low_grip = False
    self._stop_served = False      # SPRINT35_STOP_CAP_CONTINUITY
    self._stop_last_d = None
    self._stop_seen_t = -1e9
    self._stop_cont_off = False

  def _poll(self, now):
    if now - self._check < 0.25:
      return
    self._check = now
    self._signal_off = os.path.exists(SONATA_SIGNAL_PREP_OFF)  # SPRINT18C_SIGNAL_PREP
    self._stop_cont_off = os.path.exists(SONATA_STOP_CAP_CONT_OFF)  # SPRINT35_STOP_CAP_CONTINUITY
    try:  # SPRINT25A: low-grip softens the approach decel
      with open(SONATA_GRIP_LIVE_FILE_RP) as _gf:
        _g = json.load(_gf)
      self._low_grip = bool(isinstance(_g, dict) and _g.get('lowGrip'))
    except Exception:
      self._low_grip = False
    SONATA_LOW_GRIP_RP[0] = self._low_grip
    try:
      st = os.stat(self.path)
      self._file_age = max(0.0, time.time() - st.st_mtime)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          obj = json.load(f)
        self.state = obj if isinstance(obj, dict) else {}
    except Exception:
      self.state = {}
      self._file_age = float('inf')

  def update(self, now, v_ego):
    self._poll(now)
    s = self.state
    reason = 'ok'
    v_target = None
    if not s or self._file_age > SONATA_ROUTE_STALE_S:
      reason = 'stale'
    elif s.get('status') != 'ACTIVE':
      reason = str(s.get('status') or 'no_route').lower()
    elif not isinstance(s.get('gpsAgeS'), (int, float)) or s.get('gpsAgeS') > SONATA_ROUTE_GPS_MAX_AGE_S:
      reason = 'gps_age'
    elif isinstance(s.get('offRouteM'), (int, float)) and s.get('offRouteM') > SONATA_ROUTE_OFFROUTE_M:
      reason = 'off_route'
    else:
      v_target = sonata_route_prep_target(s.get('nextManeuver'), s.get('nextManeuverDistanceM'), v_ego)
      if v_target is None:
        reason = 'no_turn_ahead'
    # SPRINT25A_ARRIVAL: monotonic destination stop-ramp (replaces the 23c straight-line cap; see #193).
    # Ramp on the SMALLER of along-route remainingM and straight-line distance; once the driver is genuinely
    # arriving slow at a COMPLETE destination, COMMIT a monotonic creep-stop that never rises and holds through
    # standstill until the route is gone. Lower-only throughout: a driver gunning through is never committed.
    v_arrival, arrival_reason = None, 'no_route'
    _route_here = bool(s) and self._file_age <= SONATA_ROUTE_STALE_S and s.get('status') in ('ACTIVE', 'COMPLETE') \
        and isinstance(s.get('gpsAgeS'), (int, float)) and s.get('gpsAgeS') <= SONATA_ROUTE_GPS_MAX_AGE_S
    if not _route_here:
      self._arrival_committed = False
      self._arrival_d_min = float('inf')
    else:
      _d_rem = float(s['remainingM']) if isinstance(s.get('remainingM'), (int, float)) and 0.0 <= float(s['remainingM']) < 1e5 else None
      _d_str = float(s['destinationDistanceM']) if isinstance(s.get('destinationDistanceM'), (int, float)) else None
      _cands = [d for d in (_d_rem, _d_str) if d is not None]
      _d_arr = min(_cands) if _cands else None
      _completed = bool(s.get('completedLatched')) or s.get('status') == 'COMPLETE'
      _decel = SONATA_ARRIVAL_DECEL * (SONATA_GRIP_PREP_DECEL_SCALE if self._low_grip else 1.0)   # SPRINT32M_ARRIVAL
      if _completed and _d_arr is not None and _d_arr <= SONATA_ARRIVAL_DIST_M and float(v_ego) <= SONATA_ARRIVAL_COMMIT_V:
        self._arrival_committed = True
      # SPRINT32M_ARRIVAL: a standstill within SONATA_ARRIVAL_PASSED_M of the end IS the arrival, even when the pin sits
      # off the road and remainingM never reaches zero (leg 2: floored at 40 m). Gas still releases the hold.
      if _d_arr is not None and _d_arr <= SONATA_ARRIVAL_PASSED_M and float(v_ego) <= 0.3:
        self._arrival_committed = True
      if self._arrival_committed:
        if _d_arr is not None:
          self._arrival_d_min = min(self._arrival_d_min, _d_arr)
        _d_base = self._arrival_d_min if self._arrival_d_min < float('inf') else 0.0
        _d_eff = max(_d_base - SONATA_ARRIVAL_MARGIN_M, 0.0)
        v_arrival = max((SONATA_ARRIVAL_STOP_V ** 2 + 2.0 * _decel * _d_eff) ** 0.5, SONATA_ARRIVAL_STOP_V)
        arrival_reason = 'commit'
      elif _d_arr is not None and _d_arr <= SONATA_ARRIVAL_DIST_M and float(v_ego) >= 1.0:
        _d_eff = max(_d_arr - SONATA_ARRIVAL_MARGIN_M, 0.0)
        v_arrival = max((SONATA_ARRIVAL_END_V ** 2 + 2.0 * _decel * _d_eff) ** 0.5, SONATA_ARRIVAL_END_V)
        arrival_reason = 'ok'
      else:
        arrival_reason = 'far'
    self.arrival_info = {'active': v_arrival is not None, 'reason': arrival_reason,
                         'distM': (s.get('destinationDistanceM') if s else None),
                         'remainingM': (s.get('remainingM') if s else None),
                         'committed': self._arrival_committed, 'vTarget': v_arrival}
    if v_arrival is not None and (v_target is None or v_arrival < v_target):
      v_target, reason = v_arrival, 'arrival'
    self.info = {'active': v_target is not None, 'reason': reason,
                 'maneuver': s.get('nextManeuver'), 'distM': s.get('nextManeuverDistanceM'),
                 'vTarget': v_target, 'arrival': self.arrival_info}
    # SPRINT18C_SIGNAL_PREP: the same route gates apply; the control cap is combined lower-only.
    control = s.get('nextTrafficControl') if isinstance(s.get('nextTrafficControl'), dict) else {}
    v_signal = None
    route_ok = reason in ('ok', 'no_turn_ahead')
    # SPRINT21A_AHEAD_STOP: with no usable route, a stop sign on the road being driven (guidance aheadTrafficControl,
    # OSM around the car, heading-matched) caps exactly like a route stop sign; signals never cap (Sprint 20q).
    ahead = s.get('aheadTrafficControl') if isinstance(s.get('aheadTrafficControl'), dict) else {}
    if not (control and route_ok) and ahead.get('kind') == 'stop' and s and self._file_age <= SONATA_ROUTE_STALE_S \
        and isinstance(s.get('gpsAgeS'), (int, float)) and s.get('gpsAgeS') <= SONATA_ROUTE_GPS_MAX_AGE_S:
      control, route_ok = ahead, True
    signal_reason = reason if not route_ok else 'no_control_ahead'
    # SPRINT35_STOP_CAP_CONTINUITY: a stop control is served once the car has come to rest within 30 m of it.
    _sd = control.get('distanceM') if (control and route_ok and control.get('kind') == 'stop') else None
    if isinstance(_sd, (int, float)):
      if self._stop_last_d is not None and float(_sd) > self._stop_last_d + SONATA_STOP_NEW_CTRL_JUMP_M:
        self._stop_served = False
      if float(v_ego) <= SONATA_STOP_SERVED_V and float(_sd) <= SONATA_STOP_SERVED_M:
        self._stop_served = True
      self._stop_last_d = float(_sd)
      self._stop_seen_t = now
    elif now - self._stop_seen_t > SONATA_STOP_GONE_S:
      self._stop_served, self._stop_last_d = False, None
    if control and route_ok:
      if self._signal_off:
        signal_reason = 'disabled'
      else:
        v_signal = sonata_signal_prep_target(control.get('kind'), control.get('distanceM'), v_ego)
        signal_reason = 'ok' if v_signal is not None else 'far'
        # SPRINT35_STOP_CAP_CONTINUITY: below 3 m/s keep the stop envelope until this stop has been served.
        if v_signal is None and not self._stop_cont_off and not self._stop_served:
          v_signal = sonata_stop_cap_low_speed(control.get('kind'), control.get('distanceM'), v_ego)
          if v_signal is not None:
            signal_reason = 'ok_low_speed'
    self.signal_info = {'active': v_signal is not None, 'reason': signal_reason, 'kind': control.get('kind'),
                        'distM': control.get('distanceM'), 'source': control.get('source'), 'vTarget': v_signal,
                        'served': bool(self._stop_served)}   # SPRINT35_STOP_CAP_CONTINUITY
    # SPRINT36M1_END_OF_ROAD: the Sprint 36m map/scene caps (guidance sonataMapPrep), lower-only.
    _v36m = sonata_36m_consume(self, now, v_ego, s)
    if _v36m is not None and (v_signal is None or _v36m < v_signal):
      v_signal = _v36m
    if v_signal is not None and (v_target is None or v_signal < v_target):
      return v_signal
    return v_target


# SPRINT31B_CAPABILITY_CAPS: the ten capabilities publish decisions to capabilities.json. The three
# longitudinal ones become LOWER-ONLY cruise caps here - the same shape as every other Sonata cap, so
# they can slow the car and can never make it accelerate. Each is applied only when its own flag file
# exists under /data/sonata_caps/, which is what "enabled" means for that capability.
SONATA_CAPS_LIVE = '/data/sonata_telemetry/capabilities.json'
SONATA_ROUTE_EXEC_CAP = '/data/sonata_telemetry/route_exec_cap.json'   # SPRINT32D_ROUTE_EXEC: /data/sonata-route-exec.py
SONATA_ROUTE_EXEC_FRESH_S = 0.30      # older than this and the request is not admitted (the daemon writes at 20 Hz)
SONATA_RX_EMPTY = {'vTarget': None, 'shouldStop': False, 'aTarget': None, 'admitted': False, 'why': 'no file', 'state': None, 'requested': None}
try:
  with open('/proc/sys/kernel/random/boot_id') as _fh:
    SONATA_BOOT_ID = _fh.read().strip()
except Exception:
  SONATA_BOOT_ID = ''
SONATA_CAPS_DIR = '/data/sonata_caps'
SONATA_CAPS_STALE_S = 2.0        # a decision older than this is ignored; the daemon ticks at 0.2 s
SONATA_CAP_QUEUE_DECEL = 1.2     # m/s^2 comfortable approach to the back of a queue
SONATA_CAP_QUEUE_MARGIN_M = 6.0  # stop this far short of the last vehicle
SONATA_CAP_QUEUE_MIN_V = 0.0
SONATA_CAP_CUTIN_SCALE = 0.92    # a predicted cut-in trims the cap slightly - it must not brake hard
SONATA_CAP_CUTIN_MIN_V = 8.0     # never let a cut-in prediction cap below this
SONATA_CAP_YIELD_DECEL = 1.5
SONATA_CAP_YIELD_MIN_V = 0.0


def sonata_cap_enabled(name):
  """Actuation gate, mirroring sonata-capabilities.enabled(). Absent flag = this capability does not
  influence the car. Kept deliberately dumb so it is auditable at a glance."""
  try:
    return os.path.exists(os.path.join(SONATA_CAPS_DIR, name + '_on'))
  except Exception:
    return False


def sonata_route_exec_read(path, now_mono, boot_id):
  """SPRINT32D_ROUTE_EXEC: the route executor's lower-only request. PURE apart from one small file read.

  Admitted only when the file is from THIS boot, at most SONATA_ROUTE_EXEC_FRESH_S old (monotonic clock, shared
  with the daemon) and marked actuates by the daemon's own arming logic. Everything the daemon asked for is
  returned under 'requested' so planner_live shows requested vs admitted vs consumed."""
  out = dict(SONATA_RX_EMPTY)
  try:
    with open(path) as fh:
      d = json.load(fh)
  except Exception:
    return out
  if not isinstance(d, dict) or d.get('schema') != 'sonata-route-exec-cap-v1':
    out['why'] = 'bad schema'
    return out
  out['state'] = d.get('state')
  out['requested'] = {'vTarget': d.get('vTarget'), 'shouldStop': bool(d.get('shouldStop')), 'aTarget': d.get('aTarget'),
                      'mode': d.get('mode'), 'actuates': bool(d.get('actuates')), 'takeover': bool(d.get('takeover'))}
  if not boot_id or d.get('bootId') != boot_id:
    out['why'] = 'boot mismatch'
    return out
  mono = d.get('mono')
  if not isinstance(mono, (int, float)) or not math.isfinite(mono) or not 0.0 <= now_mono - mono <= SONATA_ROUTE_EXEC_FRESH_S:
    out['why'] = 'stale' if isinstance(mono, (int, float)) else 'no stamp'
    return out
  if d.get('actuates') is not True:
    out['why'] = 'not actuating: ' + str(d.get('why'))[:60]
    return out
  vt = d.get('vTarget')
  if isinstance(vt, (int, float)) and math.isfinite(vt) and vt >= 0.0:
    out['vTarget'] = float(vt)
  at = d.get('aTarget')
  if d.get('shouldStop') is True and isinstance(at, (int, float)) and math.isfinite(at) and -3.5 <= at <= 0.0:
    out['shouldStop'] = True
    out['aTarget'] = float(at)
  out['admitted'] = out['vTarget'] is not None or out['shouldStop']
  out['why'] = 'admitted' if out['admitted'] else 'no request'
  return out


# SPRINT36C1_ROUGH_ROAD: learned speed bump / pothole / rough-road cap published by the lane planner daemon
# (/data/sonata-lane-planner.py, Sprint 36c1). LOWER-ONLY like every other Sonata cap: admitted only when the record
# is from THIS boot, at most SONATA_ROUGH_FRESH_S old (monotonic clock shared with the daemon), marked actuates, and
# finite; floored at SONATA_ROUGH_MIN_V so it can slow the car but never stop it. /data/sonata_36c1_off ignores it.
SONATA_ROUGH_CAP = '/data/sonata_telemetry/rough_road_cap.json'
SONATA_ROUGH_OFF = '/data/sonata_36c1_off'
SONATA_ROUGH_FRESH_S = 0.6
SONATA_ROUGH_MIN_V = 4.0          # m/s (14 km/h)
_sonata_rough = {'off_t': -1e9, 'off': False}


def sonata_rough_road_cap(path, now_mono, boot_id):
  """SPRINT36C1_ROUGH_ROAD: (vTarget or None, why). PURE apart from one stat and one small file read."""
  if now_mono - _sonata_rough['off_t'] >= 1.0 or now_mono < _sonata_rough['off_t']:
    _sonata_rough['off_t'] = now_mono
    try:
      _sonata_rough['off'] = os.path.exists(SONATA_ROUGH_OFF)
    except Exception:
      _sonata_rough['off'] = False
  if _sonata_rough['off']:
    return None, 'kill switch'
  try:
    with open(path) as fh:
      d = json.load(fh)
  except Exception:
    return None, 'no file'
  if not isinstance(d, dict) or d.get('schema') != 'sonata-rough-road-cap-v1':
    return None, 'bad schema'
  if not boot_id or d.get('bootId') != boot_id:
    return None, 'boot mismatch'
  mono = d.get('mono')
  if not isinstance(mono, (int, float)) or not math.isfinite(mono) or not 0.0 <= now_mono - mono <= SONATA_ROUGH_FRESH_S:
    return None, 'stale'
  if d.get('actuates') is not True:
    return None, 'not actuating'
  vt = d.get('vTarget')
  if not isinstance(vt, (int, float)) or not math.isfinite(vt) or vt < 0.0:
    return None, 'no target'
  return max(float(vt), SONATA_ROUGH_MIN_V), str(d.get('why') or 'rough road')[:60]


def sonata_capability_cap(caps, v_ego):
  """PURE. Lowest lower-only cruise cap the enabled longitudinal capabilities ask for.

  Returns (v_target or None, reason). None means no capability wants to slow us."""
  if not isinstance(caps, dict):
    return None, 'no capabilities'
  best, why = None, 'no capability cap'

  q = caps.get('queue_join') or {}
  if q.get('actuates') and q.get('isQueue') and isinstance(q.get('backM'), (int, float)):
    d = max(0.0, float(q['backM']) - SONATA_CAP_QUEUE_MARGIN_M)
    v = max(SONATA_CAP_QUEUE_MIN_V, (2.0 * SONATA_CAP_QUEUE_DECEL * d) ** 0.5)
    if best is None or v < best:
      best, why = v, 'queue back at %.0f m -> %.1f m/s' % (q['backM'], v)

  c = caps.get('cut_in') or {}
  if c.get('actuates') and c.get('side') and isinstance(c.get('ttcS'), (int, float)):
    v = max(SONATA_CAP_CUTIN_MIN_V, float(v_ego) * SONATA_CAP_CUTIN_SCALE)
    if best is None or v < best:
      best, why = v, 'predicted %s cut-in in %.1f s -> hold %.1f m/s' % (c['side'], c['ttcS'], v)

  # SPRINT31E_ZONE_CAP: a posted limit materially BELOW the mapped limit is a school/construction/
  # temporary zone the map does not carry. One-directional by construction - restriction_zone() only
  # ever reports a LOWER limit - so this can only slow the car, never raise its speed.
  z = caps.get('restriction_zone') or {}
  if z.get('actuates') and z.get('active') and isinstance(z.get('limitMps'), (int, float)):
    v = max(0.0, float(z['limitMps']))
    if best is None or v < best:
      best, why = v, 'restriction zone: posted %.0f km/h' % (v * 3.6)

  # SPRINT33T_STRICT_LOW_ZONE: a 40-or-lower zone ahead or underneath us. Anticipated from 300 m on a
  # coast-rate ramp that meets the limit 50 m into the zone, then held AT the limit while inside - which
  # is how the owner's "overridden until we exit" is expressed in a lower-only cap.
  sz = caps.get('strict_low_zone') or {}
  if sz.get('actuates') and sz.get('active') and isinstance(sz.get('targetMps'), (int, float)):
    v = max(0.0, float(sz['targetMps']))
    if best is None or v < best:
      best, why = v, sz.get('why') or 'strict low-speed zone'

  # SPRINT33V_LIMIT_DROP: a materially lower posted limit ahead. Boundary-only - once we are in the
  # new section speed_limit governs, so this releases rather than double-governing the same road.
  ld = caps.get('limit_drop_prep') or {}
  if ld.get('actuates') and ld.get('active') and isinstance(ld.get('targetMps'), (int, float)):
    v = max(0.0, float(ld['targetMps']))
    if best is None or v < best:
      best, why = v, ld.get('why') or 'lower limit ahead'

  # SPRINT31H_YIELD_APPROACH_CAP: yield_control no longer claims to detect a conflict - it cannot.
  # It now reports an APPROACH speed for a give-way/roundabout entry, so the car arrives able to
  # yield. Lower-only, and it never commands a stop on the basis of a clearance we cannot establish;
  # the decision to enter stays the driver's.
  y = caps.get('yield_control') or {}
  if y.get('actuates') and y.get('action') == 'approach' and isinstance(y.get('targetV'), (int, float)):
    v = max(SONATA_CAP_YIELD_MIN_V, float(y['targetV']))
    if best is None or v < best:
      best, why = v, 'yield entry approach: %.0f km/h' % (v * 3.6)

  return best, why


# SPRINT31O_COAST_NOT_BRAKE: coast to shed speed; brake for objects.
# Owner, 2026-09-11: "The car should just ease off the acceleration but no braking unless collision
# impending or impact and bakes slowly and properly and then rapidly if needed."
# Measured that day: 312 of 312 harder-than-coast decelerations had NO lead vehicle, all from
# sccVision, worst -1.20 m/s^2; one recorded event shed 124.6 -> 89.7 km/h for a curve with nothing
# in front. This limits ANTICIPATORY braking only. Every object-driven path keeps full authority.
SONATA_COAST_DECEL = 0.45   # m/s^2 - about what lifting off the throttle gives at highway speed


# SPRINT32BH_LAUNCH: launch assist from a long hold. See tools/sonata/sprint32/longcontrol/hotfix_32bh_launch_assist.py.
SONATA_LAUNCH_OFF = '/data/sonata_launch_off'
SONATA_LAUNCH_MIN_HOLD_S = 3.0     # only after a real stop (a light, a queue), not a rolling hesitation
SONATA_LAUNCH_ACCEL = 0.8          # m/s^2 floor while launching (a normal driver's pull-away is 1.0-1.5)
SONATA_LAUNCH_S = 2.0              # at most this long from the model's first positive target
SONATA_LAUNCH_MODEL_GO = 0.15      # the model's own e2e target must want to go
SONATA_LAUNCH_MAX_V = 3.0          # m/s: past this the floor is moot
SONATA_LAUNCH_LEAD_MIN_M = 12.0    # a lead closer than this owns the launch (mpc)


# SPRINT33U_STOP_QUEUE: being SECOND in line at a stop sign. See hotfix_33u_stop_queue_creep.py.
SONATA_QUEUE_ARM_M = 30.0          # a map stop control this close is the one we are queuing for
SONATA_QUEUE_LEAD_GONE_M = 18.0    # the car in front has pulled away this far (or is gone). MUST sit
                                   # ABOVE SONATA_STOP_LEAD_CLOSE_M (15 m) or a lead in the overlap is
                                   # both 'close' and 'departed' and the state machine oscillates
                                   # queued<->creep within a single tick. 15-18 m is the hysteresis band:
                                   # neither condition holds there, so the current state simply persists.
SONATA_QUEUE_CREEP_V = 1.5         # m/s at the line - the owner's "sneaky little bit"
SONATA_QUEUE_MAX_V = 4.0           # m/s ceiling while rolling up to the line (~14 km/h)
SONATA_QUEUE_CREEP_DECEL = 0.45    # chosen so the ramp is <= SONATA_MAP_STOP_LATCH_V at
                                   # SONATA_MAP_STOP_LATCH_M: sqrt(1.5^2 + 2*0.45*10) = 3.35 <= 3.5
SONATA_QUEUE_TIMEOUT_S = 25.0      # never crawl forever
SONATA_QUEUE_STOPPED_V = 1.2       # we counted as stopped behind the lead below this


class SonataStopQueue:
  """We stopped behind another car at a stop sign, so we still owe the sign our own stop.

  Produces a lower-only cruise cap that rolls us up to the line slowly enough that the existing
  SPRINT23D map-stop latch commits there - it never brakes and never releases a stop itself.
  """

  def __init__(self):
    self.state = 'idle'
    self.since = 0.0
    self.info = {'state': 'idle', 'why': 'init', 'creepV': None, 'distM': None}

  @property
  def creeping(self):
    return self.state == 'creep'

  def _set(self, state, now, why, creep_v=None, dist=None):
    if state != self.state:
      self.state = state
      self.since = now
    self.info = {'state': state, 'why': why,
                 'creepV': (round(float(creep_v), 2) if creep_v is not None else None),
                 'distM': (round(float(dist), 1) if dist is not None else None)}

  def update(self, now, signal_info, v_ego, lead_present, lead_d, latched, gas, lead_close_m):
    """Returns the creep cap in m/s, or None. Lower-only; the caller takes a MIN."""
    si = signal_info if isinstance(signal_info, dict) else {}
    kind = str(si.get('kind') or '')
    dist = si.get('distM')
    have_stop = kind == 'stop' and isinstance(dist, (int, float)) and 0.0 <= float(dist) <= SONATA_QUEUE_ARM_M
    dist = float(dist) if have_stop else None

    if gas:
      self._set('idle', now, 'driver on the gas')
      return None
    if not have_stop:
      self._set('idle', now, 'no stop control within %.0f m' % SONATA_QUEUE_ARM_M)
      return None

    lead_is_close = bool(lead_present) and isinstance(lead_d, (int, float)) and float(lead_d) < lead_close_m
    lead_departed = (not lead_present) or (isinstance(lead_d, (int, float)) and float(lead_d) >= SONATA_QUEUE_LEAD_GONE_M)

    if self.state == 'idle':
      # Queue up only when we genuinely stopped behind someone at the sign.
      if lead_is_close and float(v_ego) <= SONATA_QUEUE_STOPPED_V:
        self._set('queued', now, 'stopped behind a lead at a stop sign', None, dist)
      else:
        self._set('idle', now, 'not queued at a stop sign', None, dist)
      return None

    if self.state == 'queued':
      if latched:
        # The latch took the stop while we waited - we owe the sign nothing more.
        self._set('served', now, 'stop latched while queued', None, dist)
        return None
      if lead_departed:
        self._set('creep', now, 'lead gone - rolling up to the line', None, dist)
      else:
        self.info['distM'] = round(dist, 1)
        return None

    if self.state == 'creep':
      if latched:
        self._set('served', now, 'stopped at the line - the latch holds the 1 s dwell', None, dist)
        return None
      if now - self.since > SONATA_QUEUE_TIMEOUT_S:
        self._set('idle', now, 'creep timed out', None, dist)
        return None
      if lead_is_close:
        # Someone else pulled in front; the mpc owns this again.
        self._set('queued', now, 'a lead is close again', None, dist)
        return None
      v = min(SONATA_QUEUE_MAX_V,
              (SONATA_QUEUE_CREEP_V * SONATA_QUEUE_CREEP_V + 2.0 * SONATA_QUEUE_CREEP_DECEL * max(dist, 0.0)) ** 0.5)
      v = max(v, SONATA_QUEUE_CREEP_V)
      self._set('creep', now, 'rolling up to the line at %.1f m/s' % v, v, dist)
      return float(v)

    if self.state == 'served':
      # Stay out of the way until the control is behind us or we are moving again.
      if not have_stop or float(v_ego) > 2.5:
        self._set('idle', now, 'stop served', None, dist)
      return None

    return None


# SPRINT34B_HOLD_UNTIL_EVIDENCE: who is allowed to end a stop. See hotfix_34b_hold_until_evidence.py.
SONATA_STOP_HOLD_EVIDENCE_OFF = '/data/sonata_stop_hold_evidence_off'
SONATA_SIGNAL_VISION_FILE = '/data/sonata_telemetry/signal_vision.json'
SONATA_SIGNAL_FRESH_S = 1.5        # older signal-vision output is not NOW
SONATA_GREEN_CONFIRM_S = 0.5       # consecutive fresh green before a red hold may end
SONATA_LEAD_DEPART_M = 3.0         # the lead we are held behind has pulled this far away
SONATA_LEAD_DEPART_V = 1.0         # ... or is moving at this speed
SONATA_DRIVER_GO_COOLDOWN_S = 4.0  # after the driver's go, do not re-grab the brake at the same control
SONATA_CREEP_ALERT_V = 0.12        # held, no gas, and rolling faster than this = the hold is failing


# SPRINT36P5_RED_LEAD_GUARD: at a red, a departing lead alone does not end the hold. See hotfix_36p5_red_lead_guard.py.
SONATA_RED_LEAD_GUARD_OFF = '/data/sonata_36p5_off'
_SONATA_RLG = {'t': -1e9, 'off': False}


def sonata_red_lead_guard_off(now):
  if now - _SONATA_RLG['t'] > 1.0:
    _SONATA_RLG['t'] = now
    _SONATA_RLG['off'] = os.path.exists(SONATA_RED_LEAD_GUARD_OFF)
  return _SONATA_RLG['off']


class SonataStopHoldAuthority:
  """Decides whether a held stop may end, from evidence rather than from the model going quiet."""

  def __init__(self):
    self._sig_check = -1e9
    self._sig_mtime = None
    self._sig_state = 'unknown'
    self._sig_age = 1e9
    self._off_check = -1e9
    self._off = False
    self.reset()

  def reset(self):
    self.red = False
    self.sign = False
    self.green_since = None
    self.lead_min_d = None
    self.lead_seen = False
    self.creep_events = 0
    self._creeping = False
    self.info = {'red': False, 'sign': False, 'green': None, 'leadDeparted': False, 'release': 'n/a',
                 'signal': 'unknown', 'creep': False, 'creepEvents': 0}

  def off(self, now):
    if now - self._off_check > 1.0:
      self._off_check = now
      self._off = os.path.exists(SONATA_STOP_HOLD_EVIDENCE_OFF)
    return self._off

  def _poll_signal(self, now):
    if now - self._sig_check < 0.1:
      return
    self._sig_check = now
    try:
      st = os.stat(SONATA_SIGNAL_VISION_FILE)
      self._sig_age = max(0.0, time.time() - st.st_mtime)
      if st.st_mtime != self._sig_mtime:
        self._sig_mtime = st.st_mtime
        with open(SONATA_SIGNAL_VISION_FILE) as f:
          self._sig_state = str((json.load(f) or {}).get('state') or 'unknown')
    except Exception:
      self._sig_state, self._sig_age = 'unknown', 1e9

  def signal(self):
    return self._sig_state if self._sig_age <= SONATA_SIGNAL_FRESH_S else 'unknown'

  def update(self, now, latched, map_stop, lead_present, lead_d, lead_v, v_ego, a_ego, gas):
    """Call every tick while latched. Returns True if the hold is allowed to end."""
    self._poll_signal(now)
    sig = self.signal()
    if not latched:
      self.reset()
      return True
    if sig == 'red':
      self.red = True
    if map_stop and not lead_present:
      self.sign = True
    # green must be CONSECUTIVE and fresh; unknown resets it, it never counts as green
    if sig == 'green':
      self.green_since = now if self.green_since is None else self.green_since
    else:
      self.green_since = None
    green_ok = self.green_since is not None and (now - self.green_since) >= SONATA_GREEN_CONFIRM_S
    # the lead we are held behind
    departed = False
    if lead_present and isinstance(lead_d, (int, float)):
      self.lead_seen = True
      self.lead_min_d = lead_d if self.lead_min_d is None else min(self.lead_min_d, lead_d)
      departed = (lead_d - self.lead_min_d) >= SONATA_LEAD_DEPART_M or (isinstance(lead_v, (int, float)) and lead_v >= SONATA_LEAD_DEPART_V)
    elif self.lead_seen:
      departed = True            # it was there and is gone
    # creep watch
    creeping = (not gas) and v_ego > SONATA_CREEP_ALERT_V and (a_ego is None or a_ego > 0.0)
    if creeping and not self._creeping:
      self.creep_events += 1
    self._creeping = creeping

    if self.off(now):
      allowed, why = True, 'evidence gate off'
    elif self.sign:
      allowed, why = False, 'stop sign: right of way is not perceivable - driver go only'
    elif self.red:
      # SPRINT36P5_RED_LEAD_GUARD: the lead leaving counts only while vision sees green NOW (a right-on-red lead must
      # not pull us through a red); red/unknown after a red -> green confirmation or the driver.
      _dep_ok = departed and (sig == 'green' or sonata_red_lead_guard_off(now))
      allowed = green_ok or _dep_ok
      why = 'green confirmed' if green_ok else ('lead departed' if _dep_ok else
            ('red seen: lead left but no green - green or driver only' if departed else 'red seen: hold until green, lead, or driver'))
    else:
      allowed, why = True, 'no red, no sign: model release'
    self.info = {'red': self.red, 'sign': self.sign, 'green': (round(now - self.green_since, 2) if self.green_since else None),
                 'leadDeparted': departed, 'release': why, 'signal': sig, 'creep': creeping,
                 'creepEvents': self.creep_events}
    self.info['redLeadGuard'] = bool(self.red and not self.sign and departed and not allowed)   # SPRINT36P5_RED_LEAD_GUARD
    return allowed


class SonataLaunch:
  def __init__(self):
    self.standstill_since = None
    self.go_since = None
    self.info = {'state': 'idle', 'held': None, 'sinceGo': None, 'floored': False}
    self._off_t = -1e9
    self._off = False

  def off(self, now):
    if now - self._off_t > 1.0:
      self._off_t = now
      self._off = os.path.exists(SONATA_LAUNCH_OFF)
    return self._off

  def update(self, now, v_ego, a_target, standstill, gas, brake, latched, lead_d, a_e2e, a_max):
    """Returns the (possibly floored) a_target."""
    if standstill:
      if self.standstill_since is None:
        self.standstill_since = now
      held = now - self.standstill_since
    else:
      held = (now - self.standstill_since) if self.standstill_since is not None else 0.0
    if self.off(now) or gas or brake or latched or v_ego > SONATA_LAUNCH_MAX_V or (lead_d is not None and lead_d < SONATA_LAUNCH_LEAD_MIN_M):
      self.go_since = None
      if not standstill and v_ego > SONATA_LAUNCH_MAX_V:
        self.standstill_since = None
      self.info = {'state': 'blocked' if (gas or brake or latched or (lead_d is not None and lead_d < SONATA_LAUNCH_LEAD_MIN_M)) else 'idle',
                   'held': round(held, 1), 'sinceGo': None, 'floored': False}
      return a_target
    if self.go_since is None:
      if standstill and held >= SONATA_LAUNCH_MIN_HOLD_S and a_e2e >= SONATA_LAUNCH_MODEL_GO and a_target > 0.0:
        self.go_since = now
      else:
        self.info = {'state': 'armed' if (standstill and held >= SONATA_LAUNCH_MIN_HOLD_S) else 'idle', 'held': round(held, 1), 'sinceGo': None, 'floored': False}
        if not standstill and v_ego > 0.5:
          self.standstill_since = None
        return a_target
    since = now - self.go_since
    if since > SONATA_LAUNCH_S or a_e2e < 0.0 or a_target <= 0.0:
      self.go_since = None
      self.standstill_since = None if not standstill else self.standstill_since
      self.info = {'state': 'done', 'held': round(held, 1), 'sinceGo': round(since, 2), 'floored': False}
      return a_target
    floored = float(min(max(a_target, SONATA_LAUNCH_ACCEL), a_max))
    self.info = {'state': 'launching', 'held': round(held, 1), 'sinceGo': round(since, 2), 'floored': floored > a_target}
    return floored


# SPRINT36P1_STOP_GAP: stop behind a stationary lead at the owner's gap. See hotfix_36p1_stop_gap.py.
SONATA_STOP_GAP_OFF = '/data/sonata_36p1_off'
SONATA_STOP_GAP_TARGET_M = 5.5       # final gap behind a stopped lead (owner median 5.9 m; openpilot today 3.9 m)
SONATA_STOP_GAP_TAIL_M = 0.4         # LongControl's stopping ramp rolls ~0.3-0.5 m after 0.3 m/s: aim this much further back
SONATA_STOP_GAP_LOOKAHEAD_S = 0.3    # plan on where the car will be once the brakes respond
SONATA_STOP_GAP_LEAD_STILL_V = 0.5   # |vLead| at or below this = standing still
SONATA_STOP_GAP_LEAD_MOVE_V = 1.0    # |vLead| above this = moving: 36p1 lets go at once
SONATA_STOP_GAP_ARM_S = 0.5          # the lead must stand still this long first
SONATA_STOP_GAP_MAX_D = 40.0         # m
SONATA_STOP_GAP_MAX_V = 12.0         # m/s (43 km/h): the final approach of a queue stop, never a highway ghost
SONATA_STOP_GAP_MIN_V = 0.3          # m/s: below this LongControl's stopping state owns the car
SONATA_STOP_GAP_DECEL_MAX = 1.5      # m/s^2: the most this feature ever asks for (op median peak decel 1.57)
SONATA_STOP_GAP_JERK = 1.5           # m/s^3: onset rate, measured from the previous planner output
SONATA_STOP_GAP_PROB = 0.5           # vision model probability that the lead is real
SONATA_STOP_GAP_PROB_HOLD_S = 1.0    # ... seen within this long
SONATA_STOP_GAP_MIN_ROOM = 0.25      # m: floor of the remaining distance in the kinematic formula


class SonataStopGap:
  """Lower-only planner candidate: the even deceleration to rest SONATA_STOP_GAP_TARGET_M behind a stopped lead."""

  def __init__(self):
    self.still_since = None
    self.prob_t = -1e9
    self._off_t = -1e9
    self._off = False
    self.info = {'state': 'idle', 'why': 'init', 'aCand': None, 'gapTarget': SONATA_STOP_GAP_TARGET_M}

  def off(self, now):
    if now - self._off_t > 1.0:
      self._off_t = now
      self._off = os.path.exists(SONATA_STOP_GAP_OFF)
    return self._off

  def _idle(self, why, reset=False):
    if reset:
      self.still_since = None
    self.info = {'state': 'idle', 'why': why, 'aCand': None, 'gapTarget': SONATA_STOP_GAP_TARGET_M}
    return None

  def update(self, now, dt, v_ego, lead_present, d_rel, v_lead, prob, a_prev, blocked):
    """Returns a candidate acceleration (m/s^2) or None. The caller min()s it with the other candidates."""
    try:
      if blocked:
        return self._idle('driver / disengaged', True)
      if self.off(now):
        return self._idle('kill switch', True)
      if not lead_present or d_rel is None or v_lead is None or not (math.isfinite(d_rel) and math.isfinite(v_lead)):
        return self._idle('no lead', True)
      if abs(v_lead) > SONATA_STOP_GAP_LEAD_MOVE_V:
        return self._idle('lead moving', True)
      if prob is not None and math.isfinite(prob) and prob >= SONATA_STOP_GAP_PROB:
        self.prob_t = now
      if abs(v_lead) <= SONATA_STOP_GAP_LEAD_STILL_V:
        self.still_since = now if self.still_since is None else self.still_since
      if self.still_since is None or now - self.still_since < SONATA_STOP_GAP_ARM_S:
        return self._idle('lead not yet standing still')
      if now - self.prob_t > SONATA_STOP_GAP_PROB_HOLD_S:
        return self._idle('vision does not confirm the lead')
      if not (SONATA_STOP_GAP_MIN_V <= v_ego <= SONATA_STOP_GAP_MAX_V) or not (0.0 < d_rel <= SONATA_STOP_GAP_MAX_D):
        return self._idle('outside the final-approach window')
      room = d_rel - (SONATA_STOP_GAP_TARGET_M + SONATA_STOP_GAP_TAIL_M) - v_ego * SONATA_STOP_GAP_LOOKAHEAD_S
      a_req = -(v_ego * v_ego) / (2.0 * max(room, SONATA_STOP_GAP_MIN_ROOM))
      a = max(a_req, -SONATA_STOP_GAP_DECEL_MAX)
      a = max(a, float(a_prev) - SONATA_STOP_GAP_JERK * dt)
      self.info = {'state': 'active', 'why': 'stopped lead at %.1f m' % d_rel, 'aCand': round(a, 3),
                   'aReq': round(a_req, 3), 'gapTarget': SONATA_STOP_GAP_TARGET_M}
      return float(a)
    except Exception:
      return self._idle('error', True)


def sonata_coast_limit(a_target, v_ego, has_object, stop_active, fcw, driver_braking, curve_v,
                       plan_source=None, cruise_source=None):
  """Lower-bound the braking authority when nothing is actually in front.

  Returns a_target unchanged unless ALL of these are true:
    - the plan wants to brake harder than a coast
    - there is no lead, no latched/planned stop, no FCW and the driver is not braking
    - the car is NOT already above the speed the curve geometry physically allows
  In that case the deceleration is limited to coast authority. This can only ever brake LESS; it
  never accelerates, never raises a target and never weakens a stop.
  """
  try:
    a = float(a_target)
  except (TypeError, ValueError):
    return a_target
  if a >= -SONATA_COAST_DECEL:
    return a_target                      # already gentler than a coast
  # Only braking chosen to meet a SPEED TARGET is limited. If the winning candidate was the e2e
  # model or an MPC lead track, something was actually seen - a stopped car, a pedestrian, a red
  # light - and radarState.leadOne may well be empty for it. Those keep full authority.
  if cruise_source is not None and plan_source != cruise_source:
    return a_target
  if has_object or stop_active or fcw or driver_braking:
    return a_target                      # an object is involved - untouched, full authority
  # "as physics demands": if we are genuinely faster than the corner allows, let the brakes work.
  try:
    if curve_v is not None and float(curve_v) > 0.0 and float(v_ego) > float(curve_v):
      return a_target
  except (TypeError, ValueError):
    return a_target
  return -SONATA_COAST_DECEL


# SPRINT36P2_CUTIN_COAST: a car converging on our lane from the next lane -> coast. See hotfix_36p2_cutin_coast.py.
# Shared by 36p3/36p4: the radar_live.json reader, our lane edges from the model, and the converging-vehicle watch.
SONATA_RADAR36_STALE_S = 0.5         # radar_live.json older than this (file mtime) is ignored
SONATA_RADAR36_POLL_S = 0.04
SONATA_RADAR_TO_CAMERA_M = 1.52      # radar range -> model frame x
SONATA_LANE_PROB36 = 0.30            # a lane line below this probability is not used
SONATA_LANE_WIDTH36 = 3.6            # m: lane width used when only one line of our lane is confident
SONATA_CONV_WINDOW_S = 0.5           # least-squares window for the gap-closing speed
SONATA_CONV_SETTLE_V = 0.10          # m/s: closing slower than this ...
SONATA_CONV_SETTLE_S = 0.6           # ... for this long = settled, let go
SONATA_CONV_LOST_S = 0.5             # track missing this long = let go
SONATA_CONV_COOLDOWN_S = 2.0         # a released track cannot re-arm for this long (5 s after a timeout)
SONATA_CONV_RAMP_J = 1.0             # m/s^3: positive acceleration comes back at this rate after a release
SONATA_CUTIN_OFF = '/data/sonata_36p2_off'
SONATA_CUTIN_PARAMS = {'sides': ('left', 'right'), 'x_min': 4.0, 'x_max': 40.0, 'vrel_min': -40.0, 'vrel_max': 0.5,
                       'toward_min': 0.30, 'vy_min': 0.20, 'hold_s': 0.5, 'v_min': 3.0, 'v_hold': 2.0,
                       'gap_max': 2.5, 'timeout_s': 5.0}


class SonataRadarLive:
  """mtime-cached reader of the passive radar daemon's /data/sonata_telemetry/radar_live.json. Never raises."""

  def __init__(self, path=SONATA_RADAR_LIVE):
    self.path = path
    self._mtime = None
    self._check = -1e9
    self.snap = {}
    self.fresh = False

  def poll(self, now):
    if now - self._check < SONATA_RADAR36_POLL_S:
      return
    self._check = now
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          obj = json.load(f)
        self.snap = obj if isinstance(obj, dict) else {}
      self.fresh = (time.time() - st.st_mtime) <= SONATA_RADAR36_STALE_S and not self.snap.get('idle')
    except Exception:
      self.snap, self.fresh = {}, False

  def current(self):
    return self.snap if self.fresh else None


def sonata_ego_lane_edges(model):
  """(left, right) ego-lane lines as (xs, ys) arrays in the model frame (right positive), None when not confident."""
  out = []
  try:
    lines, probs = model.laneLines, model.laneLineProbs
    for i in (1, 2):
      if len(lines) > i and len(probs) > i and float(probs[i]) >= SONATA_LANE_PROB36 and len(lines[i].x) >= 2:
        out.append((np.asarray(lines[i].x, dtype=float), np.asarray(lines[i].y, dtype=float)))
      else:
        out.append(None)
  except Exception:
    return None, None
  return out[0], out[1]


def sonata_lane_gap(edges, lane, x, y):
  """Metres from a track in the left/right lane to OUR lane line on that side (positive = still outside), or None."""
  try:
    if lane not in ('left', 'right') or edges is None:
      return None
    xm = float(x) + SONATA_RADAR_TO_CAMERA_M
    e = edges[0] if lane == 'left' else edges[1]
    o = edges[1] if lane == 'left' else edges[0]
    if e is not None:
      ye = float(np.interp(xm, e[0], e[1]))
    elif o is not None:        # only the far line of our lane is confident: a standard lane width from it
      ye = float(np.interp(xm, o[0], o[1])) + (-SONATA_LANE_WIDTH36 if lane == 'left' else SONATA_LANE_WIDTH36)
    else:
      return None
    return (ye - float(y)) if lane == 'left' else (float(y) - ye)
  except Exception:
    return None


class SonataConvergeWatch:
  """Arms when a radar track in an adjacent lane closes the gap to our lane line steadily. Output: clip(a) <= 0."""

  def __init__(self, name, kill, params):
    self.name, self.kill = name, kill
    self.p = dict(params)
    self.hist = {}          # track id -> [(mono, x, gap)]
    self.cond_since = {}    # track id -> mono the arming condition started
    self.cool = {}          # track id -> mono until which it cannot re-arm
    self.rows = {}
    self.active_id = None
    self.active_since = None
    self.settle_since = None
    self.lost_since = None
    self.release_t = None
    self.last_mono = None
    self.onsets = 0
    self._off_t, self._off = -1e9, False
    self.info = {'state': 'idle', 'why': 'init', 'track': None, 'onsets': 0}

  def off(self, now):
    if now - self._off_t > 1.0:
      self._off_t = now
      self._off = os.path.exists(self.kill)
    return self._off

  @property
  def active(self):
    return self.active_id is not None

  @staticmethod
  def _closing(h, mono):
    pts = [q for q in h if mono - q[0] <= SONATA_CONV_WINDOW_S + 1e-6]
    if len(pts) < 4 or pts[-1][0] - pts[0][0] < 0.3:
      return None
    n = float(len(pts))
    mt = sum(q[0] for q in pts) / n
    mg = sum(q[2] for q in pts) / n
    den = sum((q[0] - mt) ** 2 for q in pts)
    if den <= 1e-9:
      return None
    return -sum((q[0] - mt) * (q[2] - mg) for q in pts) / den     # positive = the gap to our lane is closing

  def _release(self, why, now, mono, cooldown):
    if self.active_id is not None:
      self.cool[self.active_id] = mono + cooldown
      self.release_t = now
    self.active_id, self.active_since, self.settle_since, self.lost_since = None, None, None, None
    self.info = {'state': 'idle', 'why': why, 'track': None, 'onsets': self.onsets}

  def update(self, now, snap, edges, v_ego, lc_active):
    """now: time.monotonic(); snap: radar_live dict or None when stale; edges: sonata_ego_lane_edges(model).
    Never raises into plannerd: any error lets go (no clip) and clears the watch."""
    try:
      return self._update(now, snap, edges, v_ego, lc_active)
    except Exception:
      self.active_id, self.active_since, self.settle_since, self.lost_since = None, None, None, None
      self.cond_since, self.hist, self.rows = {}, {}, {}
      self.info = {'state': 'idle', 'why': 'error', 'track': None, 'onsets': self.onsets}
      return False

  def _update(self, now, snap, edges, v_ego, lc_active):
    p = self.p
    gate = None
    if self.off(now):
      gate = 'kill switch'
    elif lc_active:
      gate = 'our lane change'
    elif snap is None:
      gate = 'radar stale'
    elif v_ego < p['v_hold']:
      gate = 'slow'
    if gate is not None:
      if self.active_id is not None:
        self._release(gate, now, self.last_mono or 0.0, SONATA_CONV_COOLDOWN_S)
      if gate != 'slow':
        self.cond_since.clear()
      self.info = {'state': 'idle', 'why': gate, 'track': None, 'onsets': self.onsets}
      return False
    try:
      mono = float(snap.get('mono'))
    except Exception:
      return self.active
    if mono != self.last_mono:
      self._ingest(snap, edges, v_ego, mono)
      self.last_mono = mono
    was_active = self.active_id is not None
    if self.active_id is not None:
      r = self.rows.get(self.active_id)
      why, cool = None, SONATA_CONV_COOLDOWN_S
      if r is None:
        self.lost_since = mono if self.lost_since is None else self.lost_since
        if mono - self.lost_since > SONATA_CONV_LOST_S:
          why = 'track lost'
      else:
        self.lost_since = None
        x, lane, gap, closing, vrel = r
        if lane == 'ego' or (gap is not None and gap <= 0.0):
          why = 'in our lane - the planner lead now'
        elif x < 0.5 or x > p['x_max'] + 10.0:
          why = 'out of range'
        elif vrel > p['vrel_max'] + 1.5:
          why = 'pulling away'
        elif closing is not None and closing < SONATA_CONV_SETTLE_V:
          self.settle_since = mono if self.settle_since is None else self.settle_since
          if mono - self.settle_since >= SONATA_CONV_SETTLE_S:
            why = 'settled'
        else:
          self.settle_since = None
      if why is None and now - self.active_since > p['timeout_s']:
        why, cool = 'timeout', 5.0
      if why is not None:
        self._release(why, now, mono, cool)
      else:
        self.info = {'state': 'active', 'why': 'converging', 'onsets': self.onsets,
                     'track': self._track_info(self.active_id)}
    if self.active_id is None and v_ego >= p['v_min']:
      best, bx = None, 1e9
      for tid, since in self.cond_since.items():
        r = self.rows.get(tid)
        if r is not None and mono - since >= p['hold_s'] and self.cool.get(tid, -1e9) <= mono and r[0] < bx:
          best, bx = tid, r[0]
      if best is not None:
        self.active_id, self.active_since, self.settle_since, self.lost_since = best, now, None, None
        self.release_t = None
        self.onsets += 1
        self.info = {'state': 'active', 'why': 'converging', 'onsets': self.onsets, 'track': self._track_info(best)}
      elif not was_active:
        self.info = {'state': 'idle', 'why': 'watching %d' % len(self.cond_since), 'track': None, 'onsets': self.onsets}
    return self.active

  def _track_info(self, tid):
    r = self.rows.get(tid)
    if r is None:
      return {'id': tid}
    return {'id': tid, 'x': round(r[0], 1), 'lane': r[1], 'gap': (round(r[2], 2) if r[2] is not None else None),
            'closing': (round(r[3], 2) if r[3] is not None else None), 'vRel': round(r[4], 2)}

  def _ingest(self, snap, edges, v_ego, mono):
    p = self.p
    rows = {}
    for r in (snap.get('tracks') or []):
      try:
        tid, x, y = int(r['id']), float(r['x']), float(r['y'])
        vrel, vy = float(r.get('vRel') or 0.0), float(r.get('vy') or 0.0)
        lane, motion = str(r.get('lane')), int(r.get('motion') or 0)
      except Exception:
        continue
      h = self.hist.get(tid)
      if h and (mono - h[-1][0] > 0.35 or abs(x - h[-1][1]) > 10.0):
        h = None                              # a different object on a reused track id
        self.cond_since.pop(tid, None)
      h = h or []
      gap = sonata_lane_gap(edges, lane, x, y)
      if gap is not None:
        h.append((mono, x, gap))
        del h[:-12]
      self.hist[tid] = h
      closing = self._closing(h, mono) if gap is not None else None
      rows[tid] = (x, lane, gap, closing, vrel)
      toward_vy = vy if lane == 'left' else -vy
      vabs = v_ego + vrel
      cond = (lane in p['sides'] and motion != 1 and gap is not None and 0.0 < gap <= p['gap_max']
              and p['x_min'] <= x <= p['x_max'] and p['vrel_min'] <= vrel <= p['vrel_max']
              and vabs >= max(2.0, 0.3 * v_ego) and closing is not None and closing >= p['toward_min']
              and toward_vy >= p['vy_min']
              # it can reach our lane before we draw level with it (a car we are passing cannot cut in front)
              and (vrel >= -0.1 or gap / max(closing, 0.05) <= (x - 1.0) / -vrel))
      if cond:
        self.cond_since.setdefault(tid, mono)
      else:
        self.cond_since.pop(tid, None)
    for tid in list(self.hist):
      if tid not in rows and (not self.hist[tid] or mono - self.hist[tid][-1][0] > 1.0):
        del self.hist[tid]
        self.cond_since.pop(tid, None)
    for tid in list(self.cool):
      if self.cool[tid] < mono - 10.0:
        del self.cool[tid]
    self.rows = rows

  def clip(self, a, now):
    """Lower-only: <= 0 while armed, then positive acceleration ramps back in at SONATA_CONV_RAMP_J."""
    if not (a == a):          # NaN in: hand it back untouched (the planner's own np.clip deals with it as before)
      return a
    if self.active_id is not None:
      return min(a, 0.0)
    if self.release_t is not None:
      cap = (now - self.release_t) * SONATA_CONV_RAMP_J
      if cap >= ACCEL_MAX:
        self.release_t = None
        return a
      return min(a, cap)
    return a


def sonata_vision_physical_target(vision):
  """SPRINT16H_PHYSICAL_CURVE_FLOOR: the curve speed the geometry allows (sqrt(a_lat_max / kappa)),
  not output_v_target, which already subtracts 4 s of deceleration and inflates the gap."""
  try:
    if vision is None or not bool(getattr(vision, 'is_active', False)):
      return None
    v = float(getattr(vision, 'v_target', float('nan')))
    if not math.isfinite(v) or v <= 0.0:
      return None
    return v
  except Exception:
    return None


# SPRINT36P3_NO_PASS_RIGHT: do not overtake a slower car on its right. See hotfix_36p3_no_pass_right.py.
SONATA_NPR_OFF = '/data/sonata_36p3_off'
SONATA_NPR_V_ARM = 60.0 / 3.6        # m/s: only above 60 km/h
SONATA_NPR_V_HOLD = 57.0 / 3.6       # ... held down to 57 km/h once armed
SONATA_NPR_MARGIN = 5.0 / 3.6        # cap = the left car's speed + 5 km/h
SONATA_NPR_LEFT_V_MIN = SONATA_NPR_V_ARM - SONATA_NPR_MARGIN   # left cars slower than 55 km/h are not "traffic we pass"
SONATA_NPR_X_MIN = 3.0               # m ahead (front radar)
SONATA_NPR_X_MAX = 60.0
SONATA_NPR_ARM_S = 1.0               # a left car must be seen this long before it caps us
SONATA_NPR_RELEASE_S = 1.0           # ... and gone this long before the cap lets go
SONATA_NPR_COAST = 0.45              # m/s of cruise error = m/s^2: the cap is approached at coast rate at most


class SonataNoPassRight:
  """Lower-only cruise cap: the slowest same-direction car in the left lane ahead + 5 km/h."""

  def __init__(self):
    self.since = {}          # track id -> mono first seen qualifying (continuously)
    self.cap = None
    self.last_ok_t = None
    self.target = None
    self.onsets = 0
    self._off_t, self._off = -1e9, False
    self.info = {'state': 'idle', 'why': 'init', 'vCap': None, 'left': None, 'onsets': 0}

  def off(self, now):
    if now - self._off_t > 1.0:
      self._off_t = now
      self._off = os.path.exists(SONATA_NPR_OFF)
    return self._off

  def _idle(self, why):
    self.cap, self.last_ok_t, self.target = None, None, None
    self.info = {'state': 'idle', 'why': why, 'vCap': None, 'left': None, 'onsets': self.onsets}
    return None

  def update(self, now, snap, v_ego, lc_active):
    """Returns the cruise cap (m/s) or None. The caller applies min(v_cruise, cap) and the coast-rate floor."""
    try:
      if self.off(now):
        self.since.clear()
        return self._idle('kill switch')
      if lc_active:
        self.since.clear()
        return self._idle('our lane change')
      if snap is None:
        self.since.clear()
        return self._idle('radar stale')
      if v_ego < (SONATA_NPR_V_HOLD if self.cap is not None else SONATA_NPR_V_ARM):
        self.since.clear()
        return self._idle('below 60 km/h')
      left_lane = ((snap.get('lanes') or {}).get('left') or {})
      if not left_lane.get('exists'):
        self.since.clear()
        return self._idle('no left lane (leftmost lane)')
      mono = float(snap.get('mono'))
      best = None
      seen = set()
      for r in (snap.get('tracks') or []):
        try:
          tid, x, vrel, lane, motion = int(r['id']), float(r['x']), float(r.get('vRel') or 0.0), str(r.get('lane')), int(r.get('motion') or 0)
        except Exception:
          continue
        vabs = v_ego + vrel
        ok = (lane == 'left' and motion != 1 and SONATA_NPR_X_MIN <= x <= SONATA_NPR_X_MAX
              and vabs >= SONATA_NPR_LEFT_V_MIN and vabs >= 0.5 * v_ego)
        if not ok:
          continue
        seen.add(tid)
        since = self.since.setdefault(tid, mono)
        if mono - since >= SONATA_NPR_ARM_S and (best is None or vabs < best[1]):
          best = (tid, vabs, x)
      for tid in list(self.since):
        if tid not in seen:
          del self.since[tid]
      if best is not None:
        cap = best[1] + SONATA_NPR_MARGIN
        if self.cap is None:
          self.onsets += 1
        self.cap, self.last_ok_t, self.target = cap, now, best
        self.info = {'state': 'capped', 'why': 'car in the left lane', 'vCap': round(cap, 2), 'onsets': self.onsets,
                     'left': {'id': best[0], 'v': round(best[1], 2), 'x': round(best[2], 1)}}
        return cap
      if self.cap is not None and self.last_ok_t is not None and now - self.last_ok_t < SONATA_NPR_RELEASE_S:
        self.info['why'] = 'left car gone - holding %.1f s' % SONATA_NPR_RELEASE_S
        return self.cap
      return self._idle('no slower car in the left lane')
    except Exception:
      return self._idle('error')


def sonata_npr_apply(v_cruise, cap, v_ego):
  """min(v_cruise, cap), but never more than SONATA_NPR_COAST below the current speed: shed speed by coasting."""
  if cap is None:
    return v_cruise
  return min(v_cruise, max(cap, v_ego - SONATA_NPR_COAST))


def sonata_curve_prep_floor(vision_active, v_target, v_ego):
  """Cruise deceleration floor for this frame; A_CRUISE_MIN unless a curve target is far below v_ego."""
  try:
    if not vision_active or v_target is None or v_target >= V_CRUISE_UNSET or v_ego < 3.0:
      return A_CRUISE_MIN
    gap = float(v_ego) - float(v_target)
    if gap <= SONATA_CURVE_PREP_GAP_BP[0]:
      return A_CRUISE_MIN
    # SPRINT25B_SHARP_CURVE: a sharp bend (low target) earns a deeper floor so a late-detected tight turn is still caught.
    # SPRINT27A_M17: an unmarked / low-confidence bend also earns the deeper floor - the model predicts curvature
    # late there (#194 t=662 s: llprob 0.04/0.08, torque at the ceiling). A limit, not a command.
    _sharp = float(v_target) <= SONATA_CURVE_PREP_SHARP_V or SONATA_LANE_CONF[0] < M17_LANE_CONF
    _deepest = SONATA_CURVE_PREP_SHARP_ACCEL if _sharp else SONATA_CURVE_PREP_MIN_ACCEL
    return float(np.interp(gap, SONATA_CURVE_PREP_GAP_BP, [A_CRUISE_MIN, _deepest]))
  except Exception:
    return A_CRUISE_MIN


# SPRINT36P4_MERGE_COURTESY: a car merging from the right on the highway -> coast. See hotfix_36p4_merge_courtesy.py.
SONATA_MERGE_OFF = '/data/sonata_36p4_off'
SONATA_MERGE_PARAMS = {'sides': ('right',), 'x_min': 1.0, 'x_max': 30.0, 'vrel_min': -5.0, 'vrel_max': 3.0,
                       'toward_min': 0.20, 'vy_min': 0.10, 'hold_s': 0.6, 'v_min': 60.0 / 3.6, 'v_hold': 55.0 / 3.6,
                       'gap_max': 2.5, 'timeout_s': 6.0}


def sonata_model_horizon(model):
  """(path endpoint distance, minimum horizon speed) from modelV2; (inf, inf) if unavailable."""
  try:
    px = model.position.x
    vx = model.velocity.x
    endpoint = float(px[len(px) - 1]) if len(px) else float('inf')
    vmin = min(float(v) for v in vx) if len(vx) else float('inf')
    return endpoint, vmin
  except Exception:
    return float('inf'), float('inf')


def sonata_e2e_brake_admission(is_e2e, a_e2e, v_ego, endpoint, vmin):
  """Admit the model's braking request as a planner candidate while DEC keeps ACC.

  Only a braking request (<= SONATA_E2E_ADMIT_ACCEL) with a stop-like horizon is admitted, and a
  candidate can only lower the chosen acceleration, so weak positive E2E accel (the Drive 4
  clear-road underspeed) is never admitted.
  """
  if is_e2e:
    return False, 'blended'
  if v_ego < 1.0:
    return False, 'standstill'
  if not a_e2e <= SONATA_E2E_ADMIT_ACCEL:
    return False, 'weak'
  stop_like = (endpoint < max(SONATA_E2E_ADMIT_ENDPOINT_MIN, SONATA_E2E_ADMIT_ENDPOINT_TIME * v_ego)
               or vmin < SONATA_E2E_ADMIT_VMIN_RATIO * v_ego)
  return (True, 'admitted') if stop_like else (False, 'horizon_open')


class SonataStopSignCompletion:
  """Fresh source-qualified OEM Stop + stop-like model horizon -> complete the stop.

  The sign alone never brakes; the model must already show a stop-like horizon or stop intent.
  The stop point is the model path endpoint at arming, anchored to the road and re-tightened
  when the model shortens it. Gas aborts. The hold is delegated to the near-standstill latch.
  """
  def __init__(self, scene_path=SONATA_STOP_SIGN_SCENE):
    self.scene_path = scene_path
    self.state = 'idle'
    self.sign_fresh_until = 0.0
    self.cooldown_until = 0.0
    self.dist = float('inf')
    self.a_target = 0.0
    self._scene_mtime = None
    self._scene_check = 0.0

  def _poll_sign(self, now):
    if now - self._scene_check < 0.2:
      return
    self._scene_check = now
    try:
      st = os.stat(self.scene_path)
      if st.st_mtime == self._scene_mtime:
        return
      self._scene_mtime = st.st_mtime
      with open(self.scene_path) as f:
        scene = json.load(f)
      if now - float(scene.get('mono', -1e9)) > 1.5:
        return
      for sign in scene.get('roadSigns') or []:
        if (isinstance(sign, dict) and sign.get('observed') and str(sign.get('additionalSignName')) == 'stop'
            and float(sign.get('ageS', 9.0)) < 1.0):
          self.sign_fresh_until = now + SONATA_STOP_SIGN_FRESH_S
    except Exception:
      pass

  def sign_fresh(self, now):
    return now < self.sign_fresh_until

  def update(self, now, dt, v_ego, endpoint, vmin, a_e2e, should_stop_e2e, gas_pressed, standstill):
    """Returns True while a stop-completion deceleration target is active (state 'stopping')."""
    self._poll_sign(now)
    fresh = self.sign_fresh(now)
    if gas_pressed:
      if self.state != 'idle':
        self.cooldown_until = now + SONATA_STOP_SIGN_COOLDOWN_S
      self.state = 'idle'
      return False
    if self.state == 'idle':
      stop_like = (endpoint < max(SONATA_STOP_SIGN_ENDPOINT_MIN, SONATA_STOP_SIGN_ENDPOINT_TIME * v_ego)
                   or vmin < 0.6 * v_ego or bool(should_stop_e2e) or a_e2e <= -0.5)
      if fresh and now >= self.cooldown_until and v_ego > 1.0 and endpoint < 120.0 and stop_like:
        self.state = 'stopping'
        self.dist = endpoint
      else:
        return False
    if self.state == 'stopping':
      self.dist = min(self.dist - v_ego * dt, endpoint)
      if standstill or v_ego <= SONATA_STOP_COMMIT_SPEED:
        self.state = 'hold'
        self.a_target = 0.0
        return False
      if self.dist <= 0.0 and v_ego > 2.0:
        # The anchored stop point is behind us and the car is still moving: do not brake late.
        self.state = 'idle'
        self.cooldown_until = now + SONATA_STOP_SIGN_COOLDOWN_S
        return False
      d = max(self.dist - SONATA_STOP_SIGN_MARGIN, 1.0)
      self.a_target = max(SONATA_STOP_SIGN_MIN_DECEL, min(0.0, -(v_ego * v_ego) / (2.0 * d)))
      return True
    if self.state == 'hold':
      if v_ego > 1.5:
        self.state = 'idle'
        self.cooldown_until = now + SONATA_STOP_SIGN_COOLDOWN_S
      return False
    return False


def sonata_write_planner_live(path, payload):
  try:
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(payload, f, separators=(',', ':'))
    os.replace(tmp, path)
  except Exception:
    pass


# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

SONATA_MAX_ACCEL_SCALE = [1.0]  # SPRINT20B_TRAFFIC_MODE: accel profile scale (eco 0.8 / normal 1.0 / sport 1.25), never above ACCEL_MAX


def get_max_accel(v_ego):
  return min(float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)) * SONATA_MAX_ACCEL_SCALE[0], ACCEL_MAX)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

SONATA_CRUISE_DEADBAND = 0.35   # m/s (1.26 km/h) - SPRINT31AB_CRUISE_DEADBAND


def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  # SPRINT31AB_CRUISE_DEADBAND: target_accel IS the raw speed error - gain 1.0, no tolerance - so the car
  # squirted throttle for deviations no driver would correct. Measured on the 2026-09-11 trip:
  # 29.1% of all engaged cycles were small throttle (0.02 < a < 0.15) and only 17.0% were coasting.
  # Inside the deadband the cruise term asks for nothing; outside it, behaviour is unchanged. This
  # only moves target_accel TOWARD ZERO, and a_cruise is just one candidate in min(candidates), so
  # lead/e2e/stop/FCW braking are untouched.
  _err = v_cruise - v_ego
  if abs(_err) < SONATA_CRUISE_DEADBAND:
    _err = 0.0
  target_accel = np.clip(_err, SONATA_CRUISE_MIN[0], max_accel)  # SPRINT15D_CURVE_PREP_FLOOR
  j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False
    self.sonata_stop_latched = False
    self.sonata_stop_commit_time = 0.0
    self.sonata_rx = dict(SONATA_RX_EMPTY)   # SPRINT32D_ROUTE_EXEC
    self.sonata_launch = SonataLaunch()   # SPRINT32BH_LAUNCH
    self.sonata_rx_cap_used = False
    self.sonata_rx_stop_used = False
    self.sonata_stop_hold_time = 0.0
    self.sonata_stop_clear_time = 0.0
    self.sonata_stop_sign = SonataStopSignCompletion()
    self.sonata_map_stop_pending = False   # SPRINT23D_MAP_STOP_LATCH
    self.sonata_stop_queue = SonataStopQueue()   # SPRINT33U_STOP_QUEUE
    self.sonata_hold_auth = SonataStopHoldAuthority()   # SPRINT34B_HOLD_UNTIL_EVIDENCE
    self.sonata_stop_gap = SonataStopGap()   # SPRINT36P1_STOP_GAP
    self.sonata_driver_go_until = 0.0
    self.sonata_map_stop_cooldown_until = 0.0
    self.sonata_admit = (False, 'init')
    self.sonata_planner_live_t = 0.0
    self.sonata_route_prep = SonataRoutePrep()
    self.sonata_merge = SonataConvergeWatch('merge', SONATA_MERGE_OFF, SONATA_MERGE_PARAMS)   # SPRINT36P4_MERGE_COURTESY
    self.sonata_cap_v = None            # SPRINT31B_CAPABILITY_CAPS
    self.sonata_cap_why = 'not evaluated'
    self.sonata_lane_long = SonataLaneChangeLong()  # SPRINT19_LANE_CHANGE_LONG
    self.sonata_radar36 = SonataRadarLive()   # SPRINT36P2_CUTIN_COAST (shared with 36p3/36p4)
    self.sonata_cutin = SonataConvergeWatch('cutin', SONATA_CUTIN_OFF, SONATA_CUTIN_PARAMS)   # SPRINT36P2_CUTIN_COAST
    self.sonata_personality = SonataPersonalityFile()  # SPRINT20B_TRAFFIC_MODE
    self.sonata_npr = SonataNoPassRight()   # SPRINT36P3_NO_PASS_RIGHT
    self.sonata_max_accel_scale = 1.0
    self.sonata_route_v = None
    self.sonata_curve_v = None

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)
    _vision = getattr(getattr(self, 'scc', None), 'vision', None)
    self.sonata_curve_v = sonata_vision_physical_target(_vision)
    try:   # SPRINT27A_M17: ego-lane line confidence (indices 1,2 are the lines either side of us)
      _llp = sm['modelV2'].laneLineProbs
      SONATA_LANE_CONF[0] = float(min(_llp[1], _llp[2])) if len(_llp) >= 3 else 1.0
    except Exception:
      SONATA_LANE_CONF[0] = 1.0
    SONATA_CRUISE_MIN[0] = sonata_curve_prep_floor(self.sonata_curve_v is not None, self.sonata_curve_v, v_ego)  # SPRINT16H_PHYSICAL_CURVE_FLOOR
    # SPRINT16C_ROUTE_PREP: the next route turn caps the cruise target on the approach (lower only).
    self.sonata_route_v = self.sonata_route_prep.update(time.monotonic(), v_ego)
    # SPRINT19_LANE_CHANGE_LONG: target-lane lead cap during a lane change; our prep caps are frozen while it runs.
    sonata_lc_running, sonata_lc_v = self.sonata_lane_long.update(time.monotonic(), sm['modelV2'].meta, v_ego)
    if sonata_lc_running:
      self.sonata_route_v = None
    if sonata_lc_v is not None and sonata_lc_v < v_cruise:
      v_cruise = sonata_lc_v
    # SPRINT36P2_CUTIN_COAST: passive radar tracks -> converging-vehicle watch (the clip is applied at the output).
    _r36_now = time.monotonic()
    self.sonata_radar36.poll(_r36_now)
    _r36_snap = self.sonata_radar36.current()
    _r36_edges = sonata_ego_lane_edges(sm['modelV2'])
    _r36_lc = str(self.sonata_lane_long.info.get('state', 'off')) != 'off'
    self.sonata_cutin.update(_r36_now, _r36_snap, _r36_edges, v_ego, _r36_lc)
    self.sonata_merge.update(_r36_now, _r36_snap, _r36_edges, v_ego, _r36_lc)   # SPRINT36P4_MERGE_COURTESY
    # SPRINT33U_STOP_QUEUE: second in line at a stop sign - roll up to the line, do not launch at it.
    _q_lead = sm['radarState'].leadOne
    _q_v = self.sonata_stop_queue.update(
      time.monotonic(), self.sonata_route_prep.signal_info, v_ego,
      bool(_q_lead.present), (float(_q_lead.dRel) if _q_lead.present else None),
      bool(self.sonata_stop_latched), bool(sm['carState'].gasPressed), SONATA_STOP_LEAD_CLOSE_M)
    if _q_v is not None and _q_v < v_cruise:
      v_cruise = _q_v

    # SPRINT31B_CAPABILITY_CAPS: lower-only, and only for capabilities whose flag is set.
    try:
      _caps_raw = {}
      if os.path.exists(SONATA_CAPS_LIVE) and (time.time() - os.path.getmtime(SONATA_CAPS_LIVE)) <= SONATA_CAPS_STALE_S:
        with open(SONATA_CAPS_LIVE) as _fh:
          _caps_raw = (json.load(_fh) or {}).get('capabilities') or {}
      self.sonata_cap_v, self.sonata_cap_why = sonata_capability_cap(_caps_raw, v_ego)
    except Exception:
      self.sonata_cap_v, self.sonata_cap_why = None, 'capabilities unreadable'
    if self.sonata_cap_v is not None and self.sonata_cap_v < v_cruise:
      v_cruise = self.sonata_cap_v
    # SPRINT36C1_ROUGH_ROAD: learned bump / pothole / rough-road cap from the lane planner daemon (lower-only)
    self.sonata_rough_v, self.sonata_rough_why = sonata_rough_road_cap(SONATA_ROUGH_CAP, time.monotonic(), SONATA_BOOT_ID)
    if self.sonata_rough_v is not None and self.sonata_rough_v < v_cruise:
      v_cruise = self.sonata_rough_v
    # SPRINT36P3_NO_PASS_RIGHT: a slower car in the left lane caps us at its speed + 5 km/h (lower-only, coast rate).
    _npr_cap = self.sonata_npr.update(_r36_now, _r36_snap, v_ego, _r36_lc)
    v_cruise = sonata_npr_apply(v_cruise, _npr_cap, v_ego)
    # SPRINT32D_ROUTE_EXEC_CAP: the surveyed-course route executor (/data/sonata-route-exec.py). Lower-only cruise cap,
    # admitted only when boot-bound, <=0.3 s old and marked actuates. Requested/admitted/consumed all go to
    # planner_live.routeExec (DEVICE_HANDOFF S4: a heartbeat is not proof of control use; this is).
    self.sonata_rx = sonata_route_exec_read(SONATA_ROUTE_EXEC_CAP, time.monotonic(), SONATA_BOOT_ID)
    self.sonata_rx_cap_used = False
    if self.sonata_rx['vTarget'] is not None and self.sonata_rx['vTarget'] < v_cruise:
      v_cruise = self.sonata_rx['vTarget']
      self.sonata_rx_cap_used = True

    if self.sonata_route_v is not None and self.sonata_route_v < v_cruise:
      v_cruise = self.sonata_route_v
      SONATA_CRUISE_MIN[0] = min(SONATA_CRUISE_MIN[0], sonata_curve_prep_floor(True, self.sonata_route_v, v_ego))

    # SPRINT20B_TRAFFIC_MODE: custom personalities / Traffic Mode feed long_mpc before the weights are set.
    SONATA_MAX_ACCEL_SCALE[0] = self.sonata_max_accel_scale = self.sonata_personality.update(time.monotonic(), v_ego, sm['radarState'].leadOne.dRel if sm['radarState'].leadOne.present else None, bool(sm['radarState'].leadOne.present))
    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    # Complete a stop that E2E already committed to; this does not classify light colour or signs.
    if reset_state or sm['carState'].gasPressed:
      # SPRINT34B_HOLD_UNTIL_EVIDENCE: the driver's go ends the hold, and the same control must not re-grab the brake.
      if self.sonata_stop_latched and sm['carState'].gasPressed:
        self.sonata_driver_go_until = time.monotonic() + SONATA_DRIVER_GO_COOLDOWN_S
        if self.sonata_map_stop_pending:
          self.sonata_map_stop_cooldown_until = time.monotonic() + SONATA_MAP_STOP_COOLDOWN_S
          self.sonata_map_stop_pending = False
      self.sonata_hold_auth.reset()
      self.sonata_stop_latched = False
      self.sonata_stop_commit_time = 0.0
      self.sonata_stop_hold_time = 0.0
      self.sonata_stop_clear_time = 0.0
    else:
      # SPRINT16A_STOP_HOLD: model stop intent alone commits the stop near standstill; a strong brake
      # request with no close lead counts too. A close lead's stop belongs to the mpc.
      _lead = sm['radarState'].leadOne
      sonata_lead_close = bool(_lead.present) and float(_lead.dRel) < SONATA_STOP_LEAD_CLOSE_M
      commit_candidate = ((bool(output_should_stop_e2e)
                           or (output_a_target_e2e <= SONATA_STOP_LATCH_ACCEL and not sonata_lead_close)
                           or self.sonata_stop_sign.state in ('stopping', 'hold'))
                          and v_ego <= SONATA_STOP_COMMIT_SPEED)
      # SPRINT23D_MAP_STOP_LATCH: a map stop sign within reach commits the stop from 3.5 m/s, once per sign.
      _si = self.sonata_route_prep.signal_info
      _now_m = time.monotonic()
      map_stop_candidate = (str(_si.get('kind')) == 'stop' and isinstance(_si.get('distM'), (int, float))
                            and 0.0 <= float(_si['distM']) <= SONATA_MAP_STOP_LATCH_M and v_ego <= SONATA_MAP_STOP_LATCH_V
                            and not sonata_lead_close and _now_m >= self.sonata_map_stop_cooldown_until)
      if map_stop_candidate and not self.sonata_stop_latched:
        self.sonata_map_stop_pending = True
      # SPRINT32D_ROUTE_EXEC_STOP: the executor's stop request commits the latch exactly like a model/map stop.
      commit_candidate = commit_candidate or map_stop_candidate or (self.sonata_rx['shouldStop'] and v_ego <= SONATA_STOP_COMMIT_SPEED)
      self.sonata_stop_commit_time = (self.sonata_stop_commit_time + self.dt) if commit_candidate else 0.0
      if (not self.sonata_stop_latched and self.sonata_stop_commit_time >= SONATA_STOP_COMMIT_TIME
          and time.monotonic() >= self.sonata_driver_go_until):   # SPRINT34B_HOLD_UNTIL_EVIDENCE
        self.sonata_stop_latched = True
        self.sonata_hold_auth.reset()   # SPRINT34B_HOLD_UNTIL_EVIDENCE: no state from the previous stop
        self.sonata_stop_hold_time = 0.0
        self.sonata_stop_clear_time = 0.0

      if self.sonata_stop_latched:
        stopped = bool(sm['carState'].standstill) or v_ego <= SONATA_STOP_COMPLETE_SPEED
        self.sonata_stop_hold_time = self.sonata_stop_hold_time + self.dt if stopped else 0.0
        # SPRINT32D_ROUTE_EXEC_STOP: while the executor still asks for the stop the latch cannot count clear time -> one hold state machine
        self.sonata_stop_clear_time = self.sonata_stop_clear_time + self.dt if (stopped and not output_should_stop_e2e and not self.sonata_rx['shouldStop']) else 0.0
        # SPRINT34B_HOLD_UNTIL_EVIDENCE: the model going quiet is necessary but no longer sufficient.
        _hl = sm['radarState'].leadOne
        _release_ok = self.sonata_hold_auth.update(
          time.monotonic(), True, bool(self.sonata_map_stop_pending), bool(_hl.present),
          (float(_hl.dRel) if _hl.present else None), (float(_hl.vLead) if _hl.present else None),
          v_ego, float(sm['carState'].aEgo), bool(sm['carState'].gasPressed))
        if (self.sonata_stop_hold_time >= SONATA_STOP_HOLD_TIME
            and self.sonata_stop_clear_time >= SONATA_STOP_RELEASE_CLEAR_TIME
            and _release_ok):
          if self.sonata_map_stop_pending:   # SPRINT23D_MAP_STOP_LATCH: do not re-latch on the same sign
            self.sonata_map_stop_cooldown_until = time.monotonic() + SONATA_MAP_STOP_COOLDOWN_S
            self.sonata_map_stop_pending = False
          self.sonata_stop_latched = False
          self.sonata_stop_commit_time = 0.0
          self.sonata_stop_hold_time = 0.0
          self.sonata_stop_clear_time = 0.0

    is_e2e = self.is_e2e(sm)

    self.a_cruise = get_cruise_accel(is_e2e, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if is_e2e:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    # SPRINT15B_E2E_BRAKE_ADMISSION: a strong model braking request with a stop-like horizon is a
    # candidate in every DEC mode. min() below means it can only lower the chosen acceleration.
    sonata_endpoint, sonata_vmin = sonata_model_horizon(sm['modelV2'])
    self.sonata_admit = sonata_e2e_brake_admission(is_e2e, float(output_a_target_e2e), v_ego, sonata_endpoint, sonata_vmin)
    if self.sonata_admit[0]:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    # SPRINT15B_STOP_SIGN_COMPLETION: fresh OEM Stop + stop-like model horizon completes the stop.
    sonata_now = time.monotonic()
    sonata_stopping = self.sonata_stop_sign.update(sonata_now, self.dt, v_ego, sonata_endpoint, sonata_vmin,
                                                   float(output_a_target_e2e), bool(output_should_stop_e2e),
                                                   bool(sm['carState'].gasPressed) or reset_state,
                                                   bool(sm['carState'].standstill))
    if sonata_stopping:
      candidates.append((self.sonata_stop_sign.a_target, LongitudinalPlanSource.e2e, v_ego <= SONATA_STOP_COMMIT_SPEED))
    # SPRINT32D_ROUTE_EXEC_STOP: the executor's stop request is one more candidate; the min() below keeps it lower-only.
    self.sonata_rx_stop_used = False
    if self.sonata_rx['shouldStop'] and self.sonata_rx['aTarget'] is not None:
      candidates.append((float(self.sonata_rx['aTarget']), LongitudinalPlanSource.e2e, v_ego <= SONATA_STOP_COMMIT_SPEED))

    # SPRINT36P1_STOP_GAP: a stopped lead -> the even deceleration to rest at the owner's gap (lower-only candidate).
    _sg_lead = sm['radarState'].leadOne
    _sg_a = self.sonata_stop_gap.update(sonata_now, self.dt, v_ego, bool(_sg_lead.present),
                                        (float(_sg_lead.dRel) if _sg_lead.present else None),
                                        (float(_sg_lead.vLead) if _sg_lead.present else None),
                                        (float(_sg_lead.modelProb) if _sg_lead.present else None),
                                        float(a_prev), bool(sm['carState'].gasPressed) or bool(reset_state))
    if _sg_a is not None:
      candidates.append((_sg_a, LongitudinalPlanSource.lead0, False))
    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.sonata_rx_stop_used = bool(self.sonata_rx['shouldStop'] and self.sonata_rx['aTarget'] is not None
                                    and output_a_target == float(self.sonata_rx['aTarget']))   # SPRINT32D_ROUTE_EXEC
    if sonata_now - self.sonata_planner_live_t >= 0.2:
      self.sonata_planner_live_t = sonata_now
      sonata_write_planner_live(SONATA_PLANNER_LIVE, {
        'mono': sonata_now, 'isE2e': bool(is_e2e), 'aE2e': float(output_a_target_e2e), 'aMpc': float(output_a_target_mpc),
        'aCruise': float(self.a_cruise), 'aOut': float(output_a_target), 'source': str(self.mpc.source),
        'admitted': bool(self.sonata_admit[0]), 'admitReason': self.sonata_admit[1],
        'endpoint': sonata_endpoint if sonata_endpoint != float('inf') else None,
        'vMin': sonata_vmin if sonata_vmin != float('inf') else None,
        'stopSign': {'state': self.sonata_stop_sign.state, 'fresh': self.sonata_stop_sign.sign_fresh(sonata_now),
                     'dist': self.sonata_stop_sign.dist if self.sonata_stop_sign.dist != float('inf') else None,
                     'aTarget': self.sonata_stop_sign.a_target if sonata_stopping else None},
        'stopLatched': bool(self.sonata_stop_latched),
        'stopQueue': self.sonata_stop_queue.info,   # SPRINT33U_STOP_QUEUE
        'stopHold': self.sonata_hold_auth.info,   # SPRINT34B_HOLD_UNTIL_EVIDENCE
        'curvePrepFloor': float(SONATA_CRUISE_MIN[0]),
        'routePrep': self.sonata_route_prep.info,
        'capabilityCap': {'vTarget': self.sonata_cap_v, 'why': self.sonata_cap_why},   # SPRINT31B
        'roughRoad': {'vTarget': getattr(self, 'sonata_rough_v', None), 'why': getattr(self, 'sonata_rough_why', None)},   # SPRINT36C1_ROUGH_ROAD
        'routeExec': {'requested': self.sonata_rx.get('requested'), 'admitted': bool(self.sonata_rx.get('admitted')),   # SPRINT32D_ROUTE_EXEC
                      'why': self.sonata_rx.get('why'), 'state': self.sonata_rx.get('state'),
                      'capConsumed': bool(self.sonata_rx_cap_used), 'stopConsumed': bool(self.sonata_rx_stop_used)},
        'signalPrep': self.sonata_route_prep.signal_info,
        'laneChangeLong': self.sonata_lane_long.info,
        'cutInCoast': self.sonata_cutin.info,   # SPRINT36P2_CUTIN_COAST
        'personality': self.sonata_personality.info,
        'noPassRight': self.sonata_npr.info,   # SPRINT36P3_NO_PASS_RIGHT
        'curveVTarget': self.sonata_curve_v,
        'mergeCourtesy': self.sonata_merge.info,   # SPRINT36P4_MERGE_COURTESY
        'launch': self.sonata_launch.info,   # SPRINT32BH_LAUNCH
        'stopGap': self.sonata_stop_gap.info,   # SPRINT36P1_STOP_GAP
      })
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates) or self.sonata_stop_latched
    if (self.sonata_stop_commit_time > 0.0 and not self.sonata_stop_latched
        and not os.path.exists('/data/sonata_stop_entry_off')):   # SPRINT35G_STOP_ENTRY: never accelerate into a stop being committed
      output_a_target = min(output_a_target, 0.0)
    if self.sonata_stop_latched:   # SPRINT29A_STANDSTILL_HOLD: hold THROUGH standstill, not only while still rolling
      # Owner, #198: "it kills the speed to zero but doesn't hold it... the car just rolls forward".
      # The old condition (v_ego > SONATA_STOP_COMPLETE_SPEED) dropped this clamp the instant the car
      # actually stopped, so a latched stop reverted to the base plan (~0.00 m/s^2) and crept on idle
      # torque. min() is lower-only - this can only brake harder, never accelerate - and the release
      # state machine above is unchanged, so the hold stays bounded by it.
      output_a_target = min(output_a_target, SONATA_STOP_HOLD_DECEL)  # SPRINT16A_STOP_HOLD: finish the stop
    # SPRINT32BH_LAUNCH: after a long hold, floor a launch the model already wants (never past the accel limits)
    _launch_lead = sm['radarState'].leadOne
    output_a_target = self.sonata_launch.update(sonata_now, v_ego, float(output_a_target), bool(sm['carState'].standstill),
                                                bool(sm['carState'].gasPressed), bool(sm['carState'].brakePressed),
                                                bool(self.sonata_stop_latched) or bool(self.sonata_stop_queue.creeping),   # SPRINT33U_STOP_QUEUE
                                                (float(_launch_lead.dRel) if _launch_lead.present else None), float(output_a_target_e2e), float(ACCEL_MAX))
    output_a_target = self.sonata_cutin.clip(output_a_target, time.monotonic())   # SPRINT36P2_CUTIN_COAST: coast, never brake
    # SPRINT31O_COAST_NOT_BRAKE: anticipatory braking with nothing in front is limited to coast authority.
    # Objects, stops, FCW, the driver's own brake and a genuine over-speed for the curve geometry
    # all keep full authority - see sonata_coast_limit.
    _lead1 = sm['radarState'].leadOne
    _lead2 = sm['radarState'].leadTwo
    output_a_target = sonata_coast_limit(
      output_a_target,
      v_ego,
      bool(_lead1.present) or bool(_lead2.present),
      bool(self.output_should_stop) or bool(self.sonata_stop_latched),
      bool(self.fcw),
      bool(sm['carState'].brakePressed),
      self.sonata_curve_v,
      self.mpc.source,
      LongitudinalPlanSource.cruise,
    )
    output_a_target = self.sonata_merge.clip(output_a_target, time.monotonic())   # SPRINT36P4_MERGE_COURTESY: coast, never brake
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
