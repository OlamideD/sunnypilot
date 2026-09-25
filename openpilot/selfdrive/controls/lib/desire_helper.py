import json as _sonata_json, os as _sonata_os, time as _sonata_time   # SPRINT31AR_ONE_TOUCH
from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# SPRINT31AR_ONE_TOUCH: the automatic lane change needs the stalk to have been TAPPED, not held.
# stalk.json is written by /data/sonata-stalk-monitor.py from bus-0 0x417 (stalk position, 5 Hz).
SONATA_STALK_FILE = "/data/sonata_telemetry/stalk.json"
SONATA_STALK_MAX_AGE_S = 1.5     # older than this -> unknown -> automatic path withheld (nudge required)
SONATA_TAP_RECENT_S = 3.0        # the release/pulse must be this recent to count as "the tap that started this"
_sonata_stalk_cache = {"mtime": None, "data": None}


def sonata_stalk_state():
  """Latest stalk.json, re-read only when its mtime changes. None if missing or unreadable."""
  try:
    st = _sonata_os.stat(SONATA_STALK_FILE)
    if st.st_mtime != _sonata_stalk_cache["mtime"]:
      with open(SONATA_STALK_FILE) as f:
        _sonata_stalk_cache["data"] = _sonata_json.load(f)
      _sonata_stalk_cache["mtime"] = st.st_mtime
    return _sonata_stalk_cache["data"]
  except Exception:
    return None


def sonata_one_touch_ok(direction, now=None, state=None):
  """True only when the stalk for this side was tapped and has sprung back. PURE given state/now."""
  try:
    now = _sonata_time.monotonic() if now is None else now
    data = sonata_stalk_state() if state is None else state
    if not isinstance(data, dict) or not isinstance(data.get("mono"), (int, float)):
      return False
    if now - float(data["mono"]) > SONATA_STALK_MAX_AGE_S:
      return False                                   # stale: unknown is not permission
    side = data.get("left" if direction == LaneChangeDirection.left else "right")
    if not isinstance(side, dict) or side.get("held"):
      return False                                   # still deflected -> a held stalk -> nudge required
    recent = [side.get("lastReleaseMono"), side.get("lastPulseMono")]
    return any(isinstance(t, (int, float)) and 0.0 <= now - float(t) <= SONATA_TAP_RECENT_S for t in recent)
  except Exception:
    return False


# SPRINT32BT_YELLOW_LINE_LANE_CHANGE: the camera's stable lane-line colour, written by the telemetry master's perception loop.
# DesireHelper already folds `left/right_boundary_crossable` into `semantic_blocked` -> `blindspot_detected`, which
# gates the torque-nudge path AND the automatic path; production never supplied them (modeld.py passes 5 positional
# args), so a yellow centre line never blocked anything. Owner test 2026-09-17: left blinker beside a solid yellow
# line started the lane change four times while laneLineColour.left was "yellow".
SONATA_LANE_COLOUR_FILE = "/data/sonata_telemetry/perception_v2.json"
SONATA_LANE_COLOUR_MAX_AGE_S = 2.0   # the perception loop republishes at ~4 Hz onroad (258 ms median, 435 p90)
_sonata_colour_cache = {"mtime": None, "data": None}


def sonata_lane_colour(side, now=None):
  """'yellow' / 'white' / None for the ego lane's boundary on this side. None whenever the evidence is missing,
  stale or unreadable - unknown must never block a lane change. Re-reads only when the file's mtime changes."""
  try:
    st = _sonata_os.stat(SONATA_LANE_COLOUR_FILE)
    now = _sonata_time.time() if now is None else now
    if now - st.st_mtime > SONATA_LANE_COLOUR_MAX_AGE_S:
      return None                                    # stale: the perception loop is not publishing
    if st.st_mtime != _sonata_colour_cache["mtime"]:
      with open(SONATA_LANE_COLOUR_FILE) as f:
        _sonata_colour_cache["data"] = _sonata_json.load(f)
      _sonata_colour_cache["mtime"] = st.st_mtime
    lc = (_sonata_colour_cache["data"] or {}).get("laneLineColour") or {}
    c = (lc.get(side) or {}).get("colour")
    return c if c in ("yellow", "white") else None
  except Exception:
    return None


def sonata_crossable(side, given, now=None):
  """False only on a fresh, stable yellow line; otherwise whatever the caller passed (None = unknown = allowed)."""
  if given is not None:
    return given
  # SPRINT35C_RIGHT_YELLOW_NEVER_BLOCKS: a yellow line is only ever on our LEFT in Ontario marking. A camera "yellow" on
  # the right is street-light cast on white paint (14.9 % of known night rows) - or we are in the oncoming lane, where
  # moving right is the move that must never be refused. Left side unchanged. Kill switch restores both sides.
  if side == "right" and not _sonata_os.path.exists("/data/sonata_right_yellow_blocks_on"):
    return None
  return False if sonata_lane_colour(side, now) == "yellow" else None


# SPRINT36L1_WAIT_FOR_GAP: a request made while the blind spot on that side was lit is kept PENDING (driver's blinker
# still on, <= 8 s) and starts the lane change once the blind spot has been clear for 0.3 s and every existing gate
# still passes. Torque the other way / brake / blinker off / 8 s cancel it. Kill switch /data/sonata_36l1_off.
SONATA_GAP_OFF_FILE = "/data/sonata_36l1_off"
SONATA_GAP_LOG = "/data/sonata_telemetry/wait_for_gap.jsonl"
SONATA_GAP_LOG_MAX = 2 * 1024 * 1024
SONATA_GAP_MAX_S = 8.0          # the request is kept this long at most
SONATA_GAP_CLEAR_S = 0.3        # the blind spot must stay clear this long before the pending request starts


class SonataWaitForGap:
  """SPRINT36L1_WAIT_FOR_GAP: the pending-request memory. step() is PURE apart from the evidence log."""
  def __init__(self):
    self.pending = None           # {"dir", "age", "clear", "via"}
    self.fired = 0
    self._flag_t = -1e9
    self._off = False

  def off(self):
    now = _sonata_time.monotonic()
    if now - self._flag_t >= 1.0:
      self._flag_t = now
      self._off = _sonata_os.path.exists(SONATA_GAP_OFF_FILE)
    return self._off

  def _log(self, event, **data):
    try:
      row = dict(utc=_sonata_time.strftime("%Y-%m-%dT%H:%M:%SZ", _sonata_time.gmtime()), event=event, **data)
      if _sonata_os.path.exists(SONATA_GAP_LOG) and _sonata_os.path.getsize(SONATA_GAP_LOG) > SONATA_GAP_LOG_MAX:
        _sonata_os.replace(SONATA_GAP_LOG, SONATA_GAP_LOG + ".1")
      with open(SONATA_GAP_LOG, "a") as f:
        f.write(_sonata_json.dumps(row) + "\n")
    except Exception:
      pass

  def cancel(self, why):
    p, self.pending = self.pending, None
    if p is not None:
      self._log("cancelled", why=why, direction=p["dir"], via=p["via"], waitedS=round(p["age"], 2))

  def step(self, direction, requested, via, bsm, opposite_torque, brake, driver_signal, dt):
    """One preLaneChange frame. True on a frame the pending request may start (the caller still requires
    `not blindspot_detected`)."""
    try:
      if self.off():
        self.pending = None
        return False
      if self.pending is not None and self.pending["dir"] != direction:
        self.cancel("direction_changed")
      if self.pending is None:
        if requested and bsm and driver_signal and not brake:
          self.pending = {"dir": direction, "age": 0.0, "clear": 0.0, "via": via}
          self._log("pending", direction=direction, via=via)
        return False
      p = self.pending
      p["age"] += dt
      if opposite_torque:
        self.cancel("opposite_torque")
      elif brake:
        self.cancel("brake")
      elif not driver_signal:
        self.cancel("driver_signal_off")
      elif p["age"] > SONATA_GAP_MAX_S + 1e-6:
        self.cancel("timeout")
      else:
        p["clear"] = 0.0 if bsm else p["clear"] + dt
        return p["clear"] >= SONATA_GAP_CLEAR_S - 1e-6
      return False
    except Exception:
      self.pending = None
      return False

  def started(self, by_gap):
    """The helper just entered laneChangeStarting; a pending request is consumed either way."""
    p, self.pending = self.pending, None
    if p is not None:
      self.fired += 1 if by_gap else 0
      self._log("started" if by_gap else "started_by_driver", direction=p["dir"], via=p["via"], waitedS=round(p["age"], 2))

  def leave(self, why):
    if self.pending is not None:
      self.cancel(why)


def sonata_gap_inputs(dh, carstate, torque_applied):
  """SPRINT36L1_WAIT_FOR_GAP: (requested, via, bsm, opposite_torque, driver_signal) for the current direction."""
  left = dh.lane_change_direction == LaneChangeDirection.left
  right = dh.lane_change_direction == LaneChangeDirection.right
  auto_mode = dh.alc.lane_change_set_timer not in (AutoLaneChangeMode.OFF, AutoLaneChangeMode.NUDGE)
  tapped = auto_mode and not dh.alc.prev_brake_pressed and not dh.alc.prev_lane_change and \
           sonata_one_touch_ok(dh.lane_change_direction)
  via = "nudge" if torque_applied else ("tap" if tapped else None)
  bsm = bool((carstate.leftBlindspot and left) or (carstate.rightBlindspot and right))
  opposite = bool(carstate.steeringPressed and ((carstate.steeringTorque < 0 and left) or (carstate.steeringTorque > 0 and right)))
  drv_l = bool(getattr(carstate, "sonataTurnLeftBlinker", carstate.leftBlinker))
  drv_r = bool(getattr(carstate, "sonataTurnRightBlinker", carstate.rightBlinker))
  signal = bool((left and carstate.leftBlinker and drv_l and not drv_r) or (right and carstate.rightBlinker and drv_r and not drv_l))
  return via is not None, via, bsm, opposite, signal


# SPRINT36L2_BSM_ABORT: the blind spot on the TARGET side lighting up during laneChangeStarting, before the ego centre
# has crossed the target line, cancels the lane change (desire back to none -> the model returns to the original lane).
# Kill switch /data/sonata_36l2_off.
import sys as _sonata_sys   # SPRINT36L2_BSM_ABORT
SONATA_ABORT_OFF_FILE = "/data/sonata_36l2_off"
SONATA_ABORT_LOG = "/data/sonata_telemetry/bsm_abort.jsonl"
SONATA_ABORT_LOG_MAX = 2 * 1024 * 1024
SONATA_ABORT_FRAMES = 2               # 0.1 s of a lit target-side blind spot
SONATA_ABORT_BLIND_WINDOW_S = 2.0     # without lane-line evidence: only this early in the change (earliest crossing 1.26 s)
SONATA_ABORT_CROSS_M = 0.2            # ego centre this close to the target line counts as crossed
SONATA_ABORT_RELABEL_M = 1.0          # target-line distance jumping up this much = the model re-labelled the lanes
SONATA_ABORT_MIN_PROB = 0.3           # target line probability needed to trust the geometry
SONATA_RELC_MODULE = "openpilot.sunnypilot.selfdrive.controls.lib.relc"


class SonataBsmAbort:
  """SPRINT36L2_BSM_ABORT: decide, frame by frame in laneChangeStarting, whether to cancel for a blind-spot hit."""
  def __init__(self):
    self.lit = 0
    self.crossed = False
    self.min_dist = None
    self.last_seq = None
    self.noted_late = False
    self.aborts = 0
    self._flag_t = -1e9
    self._off = False

  def off(self):
    now = _sonata_time.monotonic()
    if now - self._flag_t >= 1.0:
      self._flag_t = now
      self._off = _sonata_os.path.exists(SONATA_ABORT_OFF_FILE)
    return self._off

  def _log(self, event, **data):
    try:
      row = dict(utc=_sonata_time.strftime("%Y-%m-%dT%H:%M:%SZ", _sonata_time.gmtime()), event=event, **data)
      if _sonata_os.path.exists(SONATA_ABORT_LOG) and _sonata_os.path.getsize(SONATA_ABORT_LOG) > SONATA_ABORT_LOG_MAX:
        _sonata_os.replace(SONATA_ABORT_LOG, SONATA_ABORT_LOG + ".1")
      with open(SONATA_ABORT_LOG, "a") as f:
        f.write(_sonata_json.dumps(row) + "\n")
    except Exception:
      pass

  def target_distance(self, direction):
    """(distance from the ego centre to the target line in m, fresh) from relc's same-frame hand-off."""
    try:
      ego = getattr(_sonata_sys.modules.get(SONATA_RELC_MODULE), "SONATA_EGO_LINES", None)
      if not isinstance(ego, dict) or ego.get("seq") == self.last_seq:
        return None, False
      self.last_seq = ego.get("seq")
      left = direction == LaneChangeDirection.left
      y, p = (ego.get("left"), ego.get("lpLeft")) if left else (ego.get("right"), ego.get("lpRight"))
      if not isinstance(y, float) or not isinstance(p, float) or p < SONATA_ABORT_MIN_PROB:
        return None, True
      return (-y if left else y), True
    except Exception:
      return None, False

  def check(self, direction, carstate, timer):
    """True when the lane change must be cancelled on this frame. Never raises."""
    try:
      if timer <= DT_MDL + 1e-6:                        # first frame of a new lane change
        self.lit, self.crossed, self.min_dist, self.noted_late = 0, False, None, False
      dist, fresh = self.target_distance(direction)
      if dist is not None:
        if dist <= SONATA_ABORT_CROSS_M or (self.min_dist is not None and dist - self.min_dist >= SONATA_ABORT_RELABEL_M):
          self.crossed = True
        self.min_dist = dist if self.min_dist is None else min(self.min_dist, dist)
      if self.off():
        self.lit = 0
        return False
      left = direction == LaneChangeDirection.left
      right = direction == LaneChangeDirection.right
      lit = bool((carstate.leftBlindspot and left) or (carstate.rightBlindspot and right))
      self.lit = self.lit + 1 if lit else 0
      if self.lit < SONATA_ABORT_FRAMES:
        return False
      evidence = "lines" if dist is not None else "time"
      before = (not self.crossed) if dist is not None else (not self.crossed and timer <= SONATA_ABORT_BLIND_WINDOW_S + 1e-6)
      d = "left" if left else "right"
      if not before:
        if not self.noted_late:
          self.noted_late = True
          self._log("bsm_after_crossing", direction=d, atS=round(timer, 2), evidence=evidence,
                    distM=(round(dist, 2) if dist is not None else None), crossed=self.crossed)
        return False
      self.aborts += 1
      self.lit = 0
      self._log("aborted", direction=d, atS=round(timer, 2), evidence=evidence,
                distM=(round(dist, 2) if dist is not None else None), minDistM=(round(self.min_dist, 2) if self.min_dist is not None else None))
      return True
    except Exception:
      return False


TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.sonata_abort = SonataBsmAbort()   # SPRINT36L2_BSM_ABORT
    self.sonata_gap = SonataWaitForGap()   # SPRINT36L1_WAIT_FOR_GAP

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False,
             left_boundary_crossable=None, right_boundary_crossable=None):
    # SPRINT32BT_YELLOW_LINE_LANE_CHANGE: production passes neither crossable argument, so fill them from the camera's own verdict.
    left_boundary_crossable = sonata_crossable("left", left_boundary_crossable)
    right_boundary_crossable = sonata_crossable("right", right_boundary_crossable)
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    # SPRINT32A_ROUTE_TURN_REACHES_MODEL: a Sonata view may hide a ROUTE-TURN blinker from the lane-change path (23b); the turn
    # controller must still see the real signal, else the model never gets the turn desire. Plain carState: unchanged.
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=getattr(carstate, "sonataTurnLeftBlinker", carstate.leftBlinker),
                                               right_blinker=getattr(carstate, "sonataTurnRightBlinker", carstate.rightBlinker), v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        semantic_blocked = ((left_boundary_crossable is False and self.lane_change_direction == LaneChangeDirection.left) or
                            (right_boundary_crossable is False and self.lane_change_direction == LaneChangeDirection.right))
        blindspot_detected = (semantic_blocked or
                              ((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)
        # SPRINT36L1_WAIT_FOR_GAP: remember a request the blind spot refused; release it when the gap has opened
        _g_req, _g_via, _g_bsm, _g_opp, _g_sig = sonata_gap_inputs(self, carstate, torque_applied)
        _sonata_gap_go = self.sonata_gap.step(self.lane_change_direction, _g_req, _g_via, _g_bsm, _g_opp,
                                              carstate.brakePressed, _g_sig, DT_MDL)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        # SPRINT31AR_ONE_TOUCH: the AUTOMATIC path needs a tapped-and-released stalk; the nudge path is unchanged.
        elif (torque_applied or (self.alc.auto_lane_change_allowed and sonata_one_touch_ok(self.lane_change_direction))
              or _sonata_gap_go) and not blindspot_detected:   # SPRINT36L1_WAIT_FOR_GAP: or the pending request
          self.sonata_gap.started(_sonata_gap_go and not torque_applied)   # SPRINT36L1_WAIT_FOR_GAP
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL
        # SPRINT36L2_BSM_ABORT: target-side blind spot lit before the crossing -> the existing cancel path
        if self.sonata_abort.check(self.lane_change_direction, carstate, self.lane_change_timer):
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0

          # SONATA_PROJECT_ONE_SIGNAL_ONE_LANE_CHANGE
          # A completed lane change consumes the current blinker activation.
          # prev_one_blinker remains True while the same signal stays on, so the
          # normal off-state edge detector cannot arm another lane change until
          # the blinker first returns OFF and is then activated again.
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none

    if self.lane_change_state != LaneChangeState.preLaneChange:   # SPRINT36L1_WAIT_FOR_GAP
      self.sonata_gap.leave("left_pre_lane_change" if lateral_active else "lateral_inactive")
    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
