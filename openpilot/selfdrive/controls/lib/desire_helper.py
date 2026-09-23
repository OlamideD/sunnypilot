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
  return False if sonata_lane_colour(side, now) == "yellow" else None


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

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        # SPRINT31AR_ONE_TOUCH: the AUTOMATIC path needs a tapped-and-released stalk; the nudge path is unchanged.
        elif (torque_applied or (self.alc.auto_lane_change_allowed and sonata_one_touch_ok(self.lane_change_direction))) \
             and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0

          # SONATA_PROJECT_ONE_SIGNAL_ONE_LANE_CHANGE
          # A completed lane change consumes the current blinker activation.
          # prev_one_blinker remains True while the same signal stays on, so the
          # normal off-state edge detector cannot arm another lane change until
          # the blinker first returns OFF and is then activated again.
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none

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
