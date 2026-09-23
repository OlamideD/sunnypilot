"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_ENTERING_PRED_LAT_ACC_TH = 1.0  # Sprint13 earlier preparation
_ABORT_ENTERING_PRED_LAT_ACC_TH = 0.85  # Sprint13 hysteresis floor

# SPRINT15B_AUTHORITY_AWARE: the 270-unit CAN-FD steering command on this car physically delivers
# ~1.0-1.4 m/s^2 of lateral acceleration at full torque (torqued raw latAccelFactor 1.25-1.33; native
# rlogs #146-#148: 58-80 % of desired curvature at the clip). Curve speed is planned for the authority
# the car has, not the 384-unit tune it inherited. STEER_MAX and Panda limits are unchanged.
_TURNING_LAT_ACC_TH = 1.1  # Lat Acc threshold to trigger turning state (was 1.6).

_LEAVING_LAT_ACC_TH = 0.95  # Lat Acc threshold to trigger leaving turn state (was 1.3).
_FINISH_LAT_ACC_TH = 0.85  # Lat Acc threshold to trigger the end of the turn cycle (was 1.1).

_A_LAT_REG_MAX = 1.35  # SPRINT29C_CURVE_ENTRY: was 1.15, which made every curve sqrt(1.15/1.6) = 15% slower
                       # than stock 1.6 - the owner's 'entered that bend very, very slowly'. 1.35 is still
                       # 16% below stock, so margin is kept. Low grip is held at today's value below.
                       # (was 1.6 stock)  # SONATA_DRIVE11_CURVE_OEM_SIGNS_V1

# SPRINT31AP_CURVE_NEED: a bend is only worth slowing for if it is slower than we are. Measured
# 2026-09-12: at 124 km/h the entering threshold (1.0 m/s^2) is reached by a 1183 m radius sweeper,
# whose own geometry permits 144 km/h. Hysteresis band, m/s, so the state does not chatter while
# v_ego falls to meet the target.
SONATA_CURVE_NEED_MARGIN = 0.8


_NO_OVERSHOOT_TIME_HORIZON = 4.  # s. Time to use for velocity desired based on a_target when not overshooting.

# Lookup table for the minimum smooth deceleration during the ENTERING state
# depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
_ENTERING_SMOOTH_DECEL_V = [-0.35, -1.]  # Sprint13 modest early bleed
_ENTERING_SMOOTH_DECEL_BP = [1.0, 3.]  # Sprint13 earlier lookup

# Lookup table for the acceleration for the TURNING state
# depending on the current lateral acceleration of the vehicle.
_TURNING_ACC_V = [0.2, 0., -0.4]  # acc value (SPRINT15B: no speed recovery while lateral demand is near authority)
_TURNING_ACC_BP = [0.9, 1.1, 1.3]  # absolute value of current lat acc (SPRINT15B: scaled to measured authority)

_LEAVING_ACC = 0.0  # Sprint14: hold speed until lateral exit-settle completes.

# SPRINT16B_ROBUST_CURVE: predicted curvature from the model plan, evaluated at the current speed.
# |yawRate| * v_plan over the full 10 s horizon produced a 4.2 m/s target at 2 AM (#153 seg-010) on
# a bend whose real curvature needed 8.7 m/s: far-horizon yaw-rate noise at v_plan ~0. Curvature is
# now |yawRate| / v_plan over the first ~6 s, capped by the geometric curvature of the planned path
# (Menger), clipped to a physical maximum and low-pass filtered.
_SONATA_CURVE_HORIZON_N = 26        # plan indices 0..25 -> T_IDXS <= 6.1 s
_SONATA_KAPPA_MAX = 0.08            # 1/m (R >= 12.5 m)
_SONATA_KAPPA_GEOM_GAIN = 1.3       # yaw-rate curvature may exceed path geometry by this factor ...
_SONATA_KAPPA_GEOM_BIAS = 0.0015    # ... plus this (1/m)
_SONATA_KAPPA_FILTER_ALPHA = 0.35   # per 50 ms frame (tau ~0.1 s)
_SONATA_KAPPA_MIN_V = 2.0           # m/s floor for v_plan in the yaw-rate division
# SPRINT20Q_CURVE_PLAUSIBILITY (drive #187 t=38 s: a 4 s model-path bend at an intersection sent the target to 6 km/h)
_SONATA_KAPPA_ATTACK_ALPHA = 0.10   # rising curvature filters slower (tau ~0.5 s) than falling (0.35)
_SONATA_CURVE_PREP_DECEL = 0.45     # m/s^2: SPRINT31O_COAST_NOT_BRAKE (was 0.9). At 0.9 the prep waited and then
# braked; at 0.45 it begins about twice as far out and eases off instead. Owner 2026-09-11: "the car
# should just ease off the acceleration but no braking unless collision impending".
_SONATA_CURVE_PREP_MARGIN_M = 8.0
_SONATA_ENTERING_FLOOR = 0.5        # while ENTERING the target never drops below this fraction of v_ego ...
_SONATA_ENTERING_RATE = 1.2         # ... nor faster than this m/s per second

# SPRINT18B_AUTHORITY_SCALE: Sprint 18a raised the CAN-FD steering torque ceiling to 384 below 14 m/s,
# tapering to 270 by 21 m/s (Panda + controller). The lateral-acceleration budget and the turn-state
# thresholds above (Sprint 15b, measured at the 270 ceiling) scale with that speed-dependent authority:
# x1.35 at <= 14 m/s (270 delivered 1.0-1.4 m/s^2; 384 is 1.42x the torque) down to x1.0 at >= 21 m/s.
_SONATA_AUTH_BP = [14.0, 21.0]
_SONATA_AUTH_V = [1.35, 1.35]   # SPRINT31AQ_STEER384_ALL: the 384->270 taper this mirrored is gone; flat.
# Effective curve budget = _A_LAT_REG_MAX * 1.35 = 1.82 m/s^2 at every speed (73% of the measured 2.50
# full-scale authority), i.e. highway now gets exactly what <= 50 km/h already had. Turn-state
# thresholds scale by the same factor, so every transition stays coherent with the budget.


def sonata_authority_scale(v):
  try:
    return float(np.interp(float(v), _SONATA_AUTH_BP, _SONATA_AUTH_V))
  except Exception:
    return 1.0


# SPRINT20A_LEARNED_CURVE: the observer learns, per curvature bucket, the lateral acceleration the driver accepts when
# driving manually or pushing the gas in a curve (/data/sonata_curve_learned.json). The budget used here is that value
# (x0.9 safety margin), never below _A_LAT_REG_MAX and never above the Sprint 18b authority ceiling for the speed.
# Disable: /data/sonata_curve_learning_off. Buckets need >= 20 samples before they count.
SONATA_CURVE_LEARNED = '/data/sonata_curve_learned.json'
SONATA_CURVE_LEARNING_OFF = '/data/sonata_curve_learning_off'
SONATA_LEARNED_MARGIN = 0.9
SONATA_LEARNED_MIN_SAMPLES = 20
SONATA_LEARNED_MAX_FACTOR = 1.35          # at most 35 % above the authority-scaled plan ...
SONATA_LEARNED_ABS_BP = [14.0, 21.0]      # ... and never above what the steering can physically hold:
SONATA_LEARNED_ABS_V = [1.85, 1.85]       # SPRINT31AQ_STEER384_ALL: 384 units at every speed (was 270 above 21 m/s)


class SonataLearnedCurve:
  def __init__(self, path=SONATA_CURVE_LEARNED):
    self.path = path
    self._mtime = None
    self._check = 0.0
    self.buckets = []      # [(kappa_upper, a_lat_p85, samples)]
    self.enabled = False

  def poll(self):
    now = time.monotonic()
    if now - self._check < 1.0:
      return
    self._check = now
    try:
      import os
      self.enabled = not os.path.exists(SONATA_CURVE_LEARNING_OFF)
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        import json
        with open(self.path) as f:
          obj = json.load(f)
        rows = obj.get('buckets') if isinstance(obj, dict) else None
        self.buckets = sorted(((float(r['kappaMax']), float(r['aLatP85']), int(r['samples'])) for r in rows), key=lambda r: r[0]) if rows else []
    except Exception:
      self.buckets = []

  def budget(self, kappa, v):
    """Lateral budget (m/s^2) for this curvature at speed v, or None when nothing learned applies."""
    if not self.enabled or not self.buckets:
      return None
    for k_max, a_p85, n in self.buckets:
      if kappa <= k_max:
        if n < SONATA_LEARNED_MIN_SAMPLES:
          return None
        ceiling = min(_A_LAT_REG_MAX * sonata_authority_scale(v) * SONATA_LEARNED_MAX_FACTOR,
                      float(np.interp(v, SONATA_LEARNED_ABS_BP, SONATA_LEARNED_ABS_V)))
        return float(min(max(a_p85 * SONATA_LEARNED_MARGIN, _A_LAT_REG_MAX), ceiling))
    return None


_SONATA_LEARNED = SonataLearnedCurve()

# SPRINT20I_LOW_GRIP: /data/sonata_telemetry/grip_live.json (lane planner daemon) -> lateral budget x0.75, learning ignored.
SONATA_GRIP_LIVE = '/data/sonata_telemetry/grip_live.json'
SONATA_GRIP_LAT_SCALE = 0.639   # SPRINT29C_CURVE_ENTRY: chosen so the low-grip product is UNCHANGED -
# 1.35 * 0.639 = 0.8627 vs the previous 1.15 * 0.75 = 0.8625. Canadian winter cornering is untouched;
# only dry roads get the higher budget.
_sonata_grip = {"check": 0.0, "mtime": None, "low": False}


def sonata_grip_low():
  now = time.monotonic()
  if now - _sonata_grip["check"] >= 1.0:
    _sonata_grip["check"] = now
    try:
      import os, json
      st = os.stat(SONATA_GRIP_LIVE)
      if st.st_mtime != _sonata_grip["mtime"]:
        _sonata_grip["mtime"] = st.st_mtime
        with open(SONATA_GRIP_LIVE) as f:
          obj = json.load(f)
        _sonata_grip["low"] = bool(isinstance(obj, dict) and obj.get("lowGrip"))
      if time.time() - st.st_mtime > 30.0:
        _sonata_grip["low"] = False
    except Exception:
      _sonata_grip["low"] = False
  return _sonata_grip["low"]


def sonata_learned_budget(kappa, v):
  if sonata_grip_low():
    return None
  _SONATA_LEARNED.poll()
  return _SONATA_LEARNED.budget(kappa, v)


# SPRINT35A_CURVE_AUTHORITY: plan bend speed for the lateral accel the car can HOLD at that speed.
# Measured 20 % peg-onset of desired lateral accel (engaged, hands off, road bends, 988 segments):
# 0.8 @ 20 km/h, 1.2 @ 30, 1.6 @ 40, 2.0 @ 50, never above 55. See tools/sonata/sprint35/curves/.
# Lower-only: the target is min(previous, ceiling); >= 50 km/h the ceiling (1.85) is above the 1.82 budget.
SONATA_AUTH_LAT_BP = [7.0, 8.3, 11.1, 13.9]     # m/s (25, 30, 40, 50 km/h)
SONATA_AUTH_LAT_V = [1.10, 1.25, 1.55, 1.85]    # m/s^2 holdable without pegging
SONATA_AUTH_CEIL_MIN_V = 7.0                    # m/s: tighter bends are steering-ANGLE limited; no crawl
SONATA_AUTH_CEIL_OFF = '/data/sonata_curve_authority_off'
_SONATA_AUTH_V_GRID = np.arange(1.0, 45.0, 0.1)
_sonata_auth_ceil = {"check": -1e9, "off": False}


def sonata_authority_ceiling_off():
  now = time.monotonic()
  if now - _sonata_auth_ceil["check"] >= 1.0:
    _sonata_auth_ceil["check"] = now
    try:
      import os
      _sonata_auth_ceil["off"] = os.path.exists(SONATA_AUTH_CEIL_OFF)
    except Exception:
      _sonata_auth_ceil["off"] = False
  return _sonata_auth_ceil["off"]


def sonata_authority_ceiling_speed(kappa):
  """Highest speed (m/s) at which kappa * v^2 stays within SONATA_AUTH_LAT(v), floored at
  SONATA_AUTH_CEIL_MIN_V. Returns +inf when the curvature is unusable, so min() leaves the target alone."""
  try:
    k = float(kappa)
    if not np.isfinite(k) or k <= 0.0:
      return float('inf')
    a = np.interp(_SONATA_AUTH_V_GRID, SONATA_AUTH_LAT_BP, SONATA_AUTH_LAT_V)
    ok = k * _SONATA_AUTH_V_GRID * _SONATA_AUTH_V_GRID <= a
    v = float(_SONATA_AUTH_V_GRID[ok].max()) if ok.any() else 0.0
    # At and above the last breakpoint the ceiling (1.85) is >= every existing budget (1.82, learned <= 1.85),
    # so the existing target already satisfies it: return +inf and leave fast bends bit-for-bit unchanged.
    if v >= SONATA_AUTH_LAT_BP[-1] or bool(ok[-1]):
      return float('inf')
    return max(v, SONATA_AUTH_CEIL_MIN_V)
  except Exception:
    return float('inf')


def sonata_curve_target_speed(kappa):
  """Curve speed for the lateral budget available AT that speed.

  v = sqrt(A * scale(v) / kappa): starting from scale 1 the iterates alternate around the fixed point
  (below, above, below); two passes end on a lower bound, so the plan never assumes more authority
  than the car will have at the planned speed.
  """
  k = max(float(kappa), 1e-4)
  a_base = _A_LAT_REG_MAX * (SONATA_GRIP_LAT_SCALE if sonata_grip_low() else 1.0)  # SPRINT20I_LOW_GRIP
  v = (a_base / k) ** 0.5
  for _ in range(2):
    v = (a_base * sonata_authority_scale(v) / k) ** 0.5
  learned = sonata_learned_budget(k, v)  # SPRINT20A_LEARNED_CURVE
  if learned is not None and learned > a_base * sonata_authority_scale(v):
    v = (learned / k) ** 0.5
  if not sonata_authority_ceiling_off():   # SPRINT35A_CURVE_AUTHORITY: lower-only
    v = min(v, sonata_authority_ceiling_speed(k))
  return float(v)


def sonata_path_curvature_max(pos_x, pos_y, n):
  """Maximum Menger curvature over consecutive triplets of the first n planned path points."""
  m = min(len(pos_x), len(pos_y), n)
  if m < 3:
    return 0.0
  x = np.asarray(pos_x, dtype=float)[:m]
  y = np.asarray(pos_y, dtype=float)[:m]
  ax, ay = x[1:-1] - x[:-2], y[1:-1] - y[:-2]
  bx, by = x[2:] - x[1:-1], y[2:] - y[1:-1]
  cx, cy = x[2:] - x[:-2], y[2:] - y[:-2]
  cross = np.abs(ax * by - ay * bx)
  denom = np.sqrt(ax * ax + ay * ay) * np.sqrt(bx * bx + by * by) * np.sqrt(cx * cx + cy * cy)
  with np.errstate(divide='ignore', invalid='ignore'):
    k = np.where(denom > 1e-6, 2.0 * cross / denom, 0.0)
  k = k[np.isfinite(k)]
  return float(np.max(k)) if k.size else 0.0


def sonata_predicted_curvature(rate_plan, vel_plan, pos_x, pos_y):
  """Robust maximum curvature (1/m) of the planned path within the preparation horizon (SPRINT20Q: with its distance)."""
  k, _d = sonata_predicted_curvature_dist(rate_plan, vel_plan, pos_x, pos_y)
  return k


def sonata_predicted_curvature_dist(rate_plan, vel_plan, pos_x, pos_y):
  """(max curvature 1/m, distance m along the plan to that point)."""
  n = min(len(rate_plan), len(vel_plan), _SONATA_CURVE_HORIZON_N)
  if n < 3:
    return 0.0, 0.0
  rate = np.abs(np.asarray(rate_plan, dtype=float))[:n]
  vel = np.asarray(vel_plan, dtype=float)[:n]
  k_rate = rate / np.maximum(vel, _SONATA_KAPPA_MIN_V)
  cap = _SONATA_KAPPA_GEOM_GAIN * sonata_path_curvature_max(pos_x, pos_y, n) + _SONATA_KAPPA_GEOM_BIAS
  k = np.nan_to_num(np.minimum(k_rate, cap), nan=0.0, posinf=0.0, neginf=0.0)
  if not k.size:
    return 0.0, 0.0
  i = int(np.argmax(k))
  px = np.asarray(pos_x, dtype=float)
  dist = float(px[i]) if i < len(px) and np.isfinite(px[i]) else 0.0
  return float(min(k[i], _SONATA_KAPPA_MAX)), max(dist, 0.0)


class SmartCruiseControlVision:
  v_target: float = 0
  a_target: float = 0.
  v_ego: float = 0.
  a_ego: float = 0.
  output_v_target: float = V_CRUISE_UNSET
  output_a_target: float = 0.

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.sonata_kappa_filt = 0.0
    self.sonata_auth = 1.0  # SPRINT18B_AUTHORITY_SCALE
    self.sonata_kappa_dist = 0.0   # SPRINT20Q
    self.sonata_v_target_prev = 0.0
    self.sonata_entering_low_time = 0.0
    self.sonata_exit_settle_time = 0.0
    self.sonata_v_allowed_now = 0.0   # SPRINT31AP_CURVE_NEED: 0 = unknown -> behave as before

  def sonata_curve_needs_slowdown(self, margin=0.0):
    # SPRINT31AP_CURVE_NEED: does the bend ahead require a speed below the one we are doing?
    # Unknown returns True, so a missing or invalid target leaves today's behaviour alone.
    try:
      v_allowed = float(self.sonata_v_allowed_now)
      v_ego = float(self.v_ego)
      # v_ego must be validated too: NaN makes every comparison False, which would read
      # as 'no slowdown needed' - the one answer this must never give on bad data.
      if not np.isfinite(v_allowed) or v_allowed <= 0.0 or not np.isfinite(v_ego):
        return True
      return bool(v_allowed < v_ego - float(margin))
    except Exception:
      return True

  def get_a_target_from_control(self) -> float:
    return self.a_target

  def get_v_target_from_control(self) -> float:
    if self.is_active:
      v_target = max(self.v_target, MIN_V) + self.a_target * _NO_OVERSHOOT_TIME_HORIZON
      # Cap the Vision target in ENTERING/TURNING. This is not a
      # guarantee about downstream acceleration. LEAVING remains unchanged.
      if self.state in (VisionState.entering, VisionState.turning):
        v_target = min(v_target, self.v_ego)
        if self.v_ego_measured is not None:
          v_target = min(v_target, self.v_ego_measured)
      return v_target

    return V_CRUISE_UNSET

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      self.sonata_v_allowed_now = 0.0   # SPRINT31AP_CURVE_NEED: unknown, not 'no slowdown needed'
      return
    else:
      rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
      vel_plan = np.array(sm['modelV2'].velocity.x)

      self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

      # SPRINT16B_ROBUST_CURVE: curvature of the planned path, filtered; lateral demand at the
      # current speed is v_ego^2 * kappa, which is what the steering authority must hold.
      kappa, self.sonata_kappa_dist = sonata_predicted_curvature_dist(rate_plan, vel_plan, sm['modelV2'].position.x, sm['modelV2'].position.y)
      alpha = _SONATA_KAPPA_ATTACK_ALPHA if kappa > self.sonata_kappa_filt else _SONATA_KAPPA_FILTER_ALPHA   # SPRINT20Q
      self.sonata_kappa_filt += alpha * (kappa - self.sonata_kappa_filt)
      v_ego = max(self.v_ego, 0.1)  # ensure a value greater than 0 for calculations
      self.max_pred_lat_acc = v_ego ** 2 * self.sonata_kappa_filt
      max_curve = max(self.sonata_kappa_filt, 1e-4)

      # Get the target velocity for the maximum curve
      self.sonata_auth = sonata_authority_scale(self.v_ego)  # SPRINT18B_AUTHORITY_SCALE
      v_curve = sonata_curve_target_speed(max_curve)
      # SPRINT20Q_CURVE_PLAUSIBILITY: the target is the speed allowed NOW so that v_curve is reached at the curve;
      # while ENTERING it is floored and rate-limited (a wrong model path costs a gentle slowdown, not a crawl).
      d_eff = max(float(self.sonata_kappa_dist) - _SONATA_CURVE_PREP_MARGIN_M, 0.0)
      v_now = float((v_curve * v_curve + 2.0 * _SONATA_CURVE_PREP_DECEL * d_eff) ** 0.5)
      # SPRINT31AP_CURVE_NEED: keep the RAW distance-aware target, before the ENTERING floor and
      # rate limit below reshape it. This is the speed permitted right now to reach v_curve
      # at the bend under the existing prep deceleration - the only honest test of whether
      # any slowdown is required at all.
      self.sonata_v_allowed_now = v_now
      if self.state not in ACTIVE_STATES:
        self.sonata_v_target_prev = float(self.v_ego)
      elif self.state == VisionState.entering:
        v_now = max(v_now, _SONATA_ENTERING_FLOOR * float(self.v_ego), self.sonata_v_target_prev - _SONATA_ENTERING_RATE * DT_MDL)
      self.v_target = v_now
      self.sonata_v_target_prev = float(self.v_target)

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # Do not enter a turn control cycle if the speed is low.
          if self.v_ego <= MIN_V:
            pass
          # If significant lateral acceleration is predicted ahead, then move to Entering turn state.
          # SPRINT31AP_CURVE_NEED: curvature alone is not a reason to slow. The bend must also be
          # slower than we are - AND, never OR, so this can only ever enter less often.
          elif (self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH * self.sonata_auth
                and self.sonata_curve_needs_slowdown(SONATA_CURVE_NEED_MARGIN)):
            self.sonata_entering_low_time = 0.0
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          # Transition to Turning if current lateral acceleration is over the threshold.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH * self.sonata_auth:
            self.state = VisionState.turning
          # Abort if the predicted lateral acceleration drops
          # SPRINT31AP_CURVE_NEED: also stop once the bend is no longer slower than we are - we have
          # either shed enough speed or the path straightened. Routed through the SAME 1.2 s
          # debounce below, and released a margin LATER than it engages, so it cannot chatter.
          elif (self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH * self.sonata_auth
                or not self.sonata_curve_needs_slowdown(-SONATA_CURVE_NEED_MARGIN)):
            self.sonata_entering_low_time += DT_MDL
            if self.sonata_entering_low_time >= 1.2:
              self.sonata_entering_low_time = 0.0
              self.state = VisionState.enabled
          else:
            self.sonata_entering_low_time = 0.0

        # TURNING
        elif self.state == VisionState.turning:
          # Transition to Leaving if current lateral acceleration drops below a threshold.
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH * self.sonata_auth:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          # Sprint14: do not declare the curve finished while the car is still
          # unwinding. Require both current and predicted lateral demand to stay
          # low for a short settle window. This does not change steering limits.
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH * self.sonata_auth:
            self.sonata_exit_settle_time = 0.0
            self.state = VisionState.turning
          elif self.current_lat_acc < 0.90 * self.sonata_auth and self.max_pred_lat_acc < 0.90 * self.sonata_auth:
            self.sonata_exit_settle_time += DT_MDL
            if self.sonata_exit_settle_time >= 0.80:
              self.sonata_exit_settle_time = 0.0
              self.state = VisionState.enabled
          else:
            self.sonata_exit_settle_time = 0.0

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  def _update_solution(self) -> float:
    # DISABLED, ENABLED, OVERRIDING
    if self.state not in ACTIVE_STATES:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
      # the smooth deceleration.
      a_target = self.a_ego
    # ENTERING
    elif self.state == VisionState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      a_target = np.interp(self.max_pred_lat_acc / self.sonata_auth, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V)
    # TURNING
    elif self.state == VisionState.turning:
      # When turning, we provide a target acceleration that is comfortable for the lateral acceleration felt.
      a_target = np.interp(self.current_lat_acc / self.sonata_auth, _TURNING_ACC_BP, _TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionState.leaving:
      # When leaving, we provide a comfortable acceleration to regain speed.
      a_target = _LEAVING_ACC
    else:
      raise NotImplementedError(f"SCC-V state not supported: {self.state}")

    return a_target

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    # SONATA_MEASURED_CURVE_CAP_V2: keep filtered planner speed separate.
    # Missing/stale/invalid inputs retain the existing target; existing service
    # safety checks still decide engagement. Never carry an old measurement.
    self.v_ego_measured = None
    if sm.seen["carState"] and sm.valid["carState"] and sm.alive["carState"]:
      age = time.monotonic() - sm.logMonoTime["carState"] / 1e9
      measured = float(sm["carState"].vEgo)
      if 0.0 <= age <= 0.2 and np.isfinite(measured):
        self.v_ego_measured = max(0.0, measured)
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    self.output_v_target = self.get_v_target_from_control()
    self.output_a_target = self.get_a_target_from_control()

    self.frame += 1
