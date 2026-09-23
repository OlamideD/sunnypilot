"""Sprint 19f: lane-change request bridge for the desire helper (runs inside modeld).

The Sonata lane planner (/data/sonata-lane-planner.py) writes /data/sonata_telemetry/lane_planner.json. When it
carries an active `request` (auto mode only), this proxy presents the requested direction to DesireHelper as a
blinker input, exactly as if the driver had touched the stalk. Everything else in the lane-change path is
unchanged: the runtime's own timer (AutoLaneChangeTimer), BSM delay, road-edge/lane gating and the driver torque /
brake handling still decide whether and when the lane change happens. A stale, missing or malformed file is the
same as no request. Physical stalk input always passes through unchanged.
"""
from __future__ import annotations

import json
import os
import time

LANE_PLANNER = "/data/sonata_telemetry/lane_planner.json"
POLL_S = 0.10
MAX_AGE_S = 1.5


class SonataLaneRequest:
  def __init__(self, path: str = LANE_PLANNER):
    self.path = path
    self._mtime = None
    self._next_poll = 0.0
    self.direction = None
    self.until = 0.0
    self.mode = "off"

  def poll(self, now: float | None = None):
    now = time.monotonic() if now is None else now
    if now < self._next_poll:
      return
    self._next_poll = now + POLL_S
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          obj = json.load(f)
        req = obj.get("request") if isinstance(obj, dict) else None
        self.mode = str(obj.get("mode", "off")) if isinstance(obj, dict) else "off"
        if isinstance(req, dict) and req.get("direction") in ("left", "right") and self.mode == "auto":
          self.direction = req["direction"]
          self.until = float(req.get("untilMono", 0.0))
        else:
          self.direction = None
          self.until = 0.0
      if self._mtime is not None and time.time() - self._mtime > MAX_AGE_S:
        self.direction = None
    except Exception:
      self.direction = None
      self.until = 0.0

  def active(self, now: float | None = None) -> str | None:
    now = time.monotonic() if now is None else now
    self.poll(now)
    return self.direction if self.direction and now < self.until else None


class CarStateWithRequest:
  """Read-only view of a carState reader with the requested direction OR-ed into the blinker fields."""
  __slots__ = ("_cs", "leftBlinker", "rightBlinker", "sonataTurnLeftBlinker", "sonataTurnRightBlinker")   # SPRINT32A_ROUTE_TURN_REACHES_MODEL

  def __init__(self, cs, direction: str | None):
    self._cs = cs
    self.leftBlinker = bool(cs.leftBlinker) or direction == "left"
    self.rightBlinker = bool(cs.rightBlinker) or direction == "right"
    # SPRINT32A_ROUTE_TURN_REACHES_MODEL: the turn controller sees the DRIVER's signal only - a planner request is never a turn
    self.sonataTurnLeftBlinker = bool(cs.leftBlinker)
    self.sonataTurnRightBlinker = bool(cs.rightBlinker)

  def __getattr__(self, name):
    return getattr(self._cs, name)


_REQUEST = SonataLaneRequest()


# SPRINT23B_TURN_INTENT: a blinker is a TURN, not a lane change, when the route's next maneuver is a turn that way within
# TURN_ROUTE_DIST_M (ACTIVE route, fresh GPS), or, with no usable route, when the driver is braking with it on below
# TURN_MAX_V. The blinker is hidden from DesireHelper above TURN_MASK_MIN_V (LaneTurnDesire below 19 km/h keeps it);
# once classified the mask holds until the blinker goes off. #190: lane changes started at 80 and 55 km/h from turn blinkers.
TURN_ROUTE_DIST_M = 250.0
TURN_MAX_V = 16.7        # m/s (60 km/h) for the braking rule
TURN_BRAKE_A = -0.6      # m/s^2
TURN_MASK_MIN_V = 6.0    # m/s
TURN_MANEUVERS = ("turn", "sharp", "slight", "uturn", "end_of_road", "exit_roundabout")
TURN_GUIDANCE = "/data/sonata_drive10_lab/route_guidance_state.json"


class SonataTurnIntent:
  def __init__(self, path: str = TURN_GUIDANCE):
    self.path = path
    self._mtime = None
    self._next_poll = 0.0
    self.route_side = None
    self.route_ok = False
    self.masked = None

  def poll(self, now: float | None = None):
    now = time.monotonic() if now is None else now
    if now < self._next_poll:
      return
    self._next_poll = now + 0.25
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          s = json.load(f)
        self.route_side, self.route_ok = None, False
        if isinstance(s, dict) and isinstance(s.get("gpsAgeS"), (int, float)) and s["gpsAgeS"] <= 3.0:
          self.route_ok = s.get("status") == "ACTIVE"
          m = str(s.get("nextManeuver") or "").lower()
          d = s.get("nextManeuverDistanceM")
          if self.route_ok and any(k in m for k in TURN_MANEUVERS) and isinstance(d, (int, float)) and 0.0 <= d <= TURN_ROUTE_DIST_M:
            self.route_side = "left" if "left" in m else "right" if "right" in m else None
      if self._mtime is not None and time.time() - self._mtime > 4.0:
        self.route_side, self.route_ok = None, False
    except Exception:
      self.route_side, self.route_ok = None, False

  def side(self, cs, now: float | None = None) -> str | None:
    """The blinker side to hide from DesireHelper this frame, or None."""
    now = time.monotonic() if now is None else now
    self.poll(now)
    left, right = bool(cs.leftBlinker), bool(cs.rightBlinker)
    if not (left or right) or (left and right):
      self.masked = None
      return None
    blink = "left" if left else "right"
    if self.masked == blink:
      return blink
    if self.masked is not None and self.masked != blink:
      self.masked = None
    v = float(cs.vEgo)
    if v < TURN_MASK_MIN_V:
      return None
    turn = (self.route_side == blink) or (not self.route_ok and v <= TURN_MAX_V and float(cs.aEgo) <= TURN_BRAKE_A)
    if turn:
      self.masked = blink
      return blink
    return None


class CarStateMasked:
  """Read-only carState view with one blinker hidden."""
  __slots__ = ("_cs", "leftBlinker", "rightBlinker", "sonataTurnLeftBlinker", "sonataTurnRightBlinker")   # SPRINT32A_ROUTE_TURN_REACHES_MODEL

  def __init__(self, cs, side: str):
    self._cs = cs
    self.leftBlinker = bool(cs.leftBlinker) and side != "left"
    self.rightBlinker = bool(cs.rightBlinker) and side != "right"
    # SPRINT32A_ROUTE_TURN_REACHES_MODEL: hidden from the lane-change path only; the turn controller still gets the real signal
    self.sonataTurnLeftBlinker = bool(cs.leftBlinker)
    self.sonataTurnRightBlinker = bool(cs.rightBlinker)

  def __getattr__(self, name):
    return getattr(self._cs, name)


_TURN = SonataTurnIntent()


ROUTE_TURN_SHADOW_LOG = "/data/sonata_telemetry/route_turn_shadow.jsonl"   # SPRINT32A_ROUTE_TURN_REACHES_MODEL
ROUTE_TURN_SHADOW_MAX = 20 * 1024 * 1024
SONATA_TURN_CEILING_V = 19.0 * 0.44704   # sunnypilot LaneTurnValue default, mph -> m/s; graded, not raised, here
_route_turn_shadow_next = [0.0]


def _route_turn_shadow(cs, side):
  """SPRINT32A_ROUTE_TURN_REACHES_MODEL: 1 Hz evidence row while a route turn is in range - did the driver's signal become a turn desire?"""
  now = time.monotonic()
  if now < _route_turn_shadow_next[0] or _TURN.route_side is None:
    return
  _route_turn_shadow_next[0] = now + 1.0
  try:
    left, right = bool(cs.leftBlinker), bool(cs.rightBlinker)
    blink = "left" if (left and not right) else "right" if (right and not left) else None
    v = float(cs.vEgo)
    bsm = bool(cs.leftBlindspot) if blink == "left" else bool(cs.rightBlindspot) if blink == "right" else False
    if blink is None:
      why = "no_signal"
    elif blink != _TURN.route_side:
      why = "signal_opposes_route"
    elif v >= SONATA_TURN_CEILING_V:
      why = "above_turn_ceiling"
    elif bsm:
      why = "blind_spot"
    else:
      why = "turn_desire_sent"
    row = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "routeSide": _TURN.route_side, "signal": blink,
           "masked": side, "vEgo": round(v, 2), "ceilingV": round(SONATA_TURN_CEILING_V, 2), "bsm": bsm, "outcome": why}
    if os.path.exists(ROUTE_TURN_SHADOW_LOG) and os.path.getsize(ROUTE_TURN_SHADOW_LOG) > ROUTE_TURN_SHADOW_MAX:
      os.replace(ROUTE_TURN_SHADOW_LOG, ROUTE_TURN_SHADOW_LOG + ".1")
    with open(ROUTE_TURN_SHADOW_LOG, "a") as f:
      f.write(json.dumps(row) + "\n")
  except Exception:
    pass


def sonata_carstate_for_desire(cs):
  """Wrap carState for DesireHelper.update; a no-op view when the planner has no active request."""
  direction = _REQUEST.active()
  if direction is None:
    side = _TURN.side(cs)   # SPRINT23B_TURN_INTENT
    _route_turn_shadow(cs, side)   # SPRINT32A_ROUTE_TURN_REACHES_MODEL
    return CarStateMasked(cs, side) if side else cs
  return CarStateWithRequest(cs, direction)


# SPRINT21C_P6_KEEP_DESIRE: opt-in (flag file). keepLeft / keepRight to the model for an off-ramp / fork / merge on the
# ACTIVE route, only while the desire helper is idle. Guidance state older than 4 s, no GPS, OFF_ROUTE = no desire.
# SPRINT26B_TURN_DESIRE: adds lane POSITIONING before a route TURN (keep toward the turn side within P6_TURN_DIST_M,
# P6_TURN_MIN_V..P6_TURN_MAX_V), fed to the model ONLY with /data/sonata_p6_turn_desire; otherwise SHADOW-logged.
P6_FLAG = "/data/sonata_p6_keep_desire"
P6_TURN_FLAG = "/data/sonata_p6_turn_desire"          # SPRINT26B_TURN_DESIRE
P6_SHADOW_FILE = "/data/sonata_telemetry/route_desire_shadow.json"   # SPRINT26B_TURN_DESIRE
P6_SHADOW_LOG = "/data/sonata_telemetry/route_desire_shadow.jsonl"   # SPRINT26C_SHADOW_LOG: append-only history
P6_SHADOW_LOG_MAX = 20 * 1024 * 1024
P6_GUIDANCE = "/data/sonata_drive10_lab/route_guidance_state.json"
P6_KEEP_DIST_M = 400.0
P6_KEEP_MIN_V = 16.0          # m/s (58 km/h): motorway / arterial exits only
P6_TURN_DIST_M = 300.0        # SPRINT26B_TURN_DESIRE
P6_TURN_MIN_V = 8.0           # m/s (29 km/h)
P6_TURN_MAX_V = 22.0          # m/s (79 km/h)
P6_DESIRE = {"left": 5, "right": 6}   # log.Desire.keepLeft / keepRight
P6_MANEUVERS = ("off_ramp", "fork", "merge", "on_ramp", "keep")
P6_TURN_MANEUVERS = ("turn",)         # SPRINT26B_TURN_DESIRE
# SPRINT35C_KEEP_REPULSE: keep the keep desire inside the model's 5 s pulse history all the way to the fork.
P6_REPULSE_FLAG = "/data/sonata_p6_keep_repulse_on"   # owner switch, OFF unless the file exists
P6_REPULSE_S = 2.5            # one `none` frame this often -> a fresh rising edge (history is 5 s)
P6_KEEP_HOLD_MIN_V = 11.0     # m/s: once sent for this maneuver, keep sending down to this (arming stays 16 m/s)


class SonataRouteDesire:
  def __init__(self, path: str = P6_GUIDANCE, flag: str = P6_FLAG, turn_flag: str = P6_TURN_FLAG):
    self.path, self.flag, self.turn_flag = path, flag, turn_flag
    self._mtime = None
    self._next_poll = 0.0
    self._next_flag = 0.0
    self._next_shadow = 0.0
    self.enabled = False
    self.turn_enabled = False
    self.side = None
    self.dist = None
    self.kind = None          # "keep" (21c) or "turn" (26b)
    self.repulse_on = False   # SPRINT35C_KEEP_REPULSE
    self._keep_code, self._keep_edge, self._pulses = None, 0.0, 0   # SPRINT35C_KEEP_REPULSE

  def poll(self, now: float | None = None):
    now = time.monotonic() if now is None else now
    if now >= self._next_flag:
      self._next_flag = now + 2.0
      self.enabled = os.path.exists(self.flag)
      self.turn_enabled = os.path.exists(self.turn_flag)
      self.repulse_on = os.path.exists(P6_REPULSE_FLAG)   # SPRINT35C_KEEP_REPULSE
    if now < self._next_poll:
      return
    self._next_poll = now + 0.25
    try:
      st = os.stat(self.path)
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        with open(self.path) as f:
          s = json.load(f)
        self.side, self.dist, self.kind = None, None, None
        if isinstance(s, dict) and s.get("status") == "ACTIVE" and isinstance(s.get("gpsAgeS"), (int, float)) and s["gpsAgeS"] <= 3.0:
          m = str(s.get("nextManeuver") or "").lower()
          d = s.get("nextManeuverDistanceM")
          if any(m.startswith(k) for k in P6_MANEUVERS) and isinstance(d, (int, float)) and 0.0 <= d <= P6_KEEP_DIST_M:
            if m.endswith("left") or m.endswith("_slight_left") or "left" in m:
              self.side = "left"
            elif "right" in m:
              self.side = "right"
            self.dist = float(d)
            self.kind = "keep" if self.side else None
          elif any(m.startswith(k) for k in P6_TURN_MANEUVERS) and isinstance(d, (int, float)) and 0.0 <= d <= P6_TURN_DIST_M:
            if "left" in m:
              self.side = "left"
            elif "right" in m:
              self.side = "right"
            self.dist = float(d)
            self.kind = "turn" if self.side else None
      if self._mtime is not None and time.time() - self._mtime > 4.0:
        self.side = None; self.kind = None
    except Exception:
      self.side = None; self.kind = None

  def _would(self, dh_desire: int, v_ego: float):
    """(desire to send, gate name) if this hint were enabled; None when nothing applies."""
    if self.side is None or int(dh_desire) != 0:
      return None
    if self.kind == "keep" and float(v_ego) >= P6_KEEP_MIN_V:
      return P6_DESIRE[self.side], "keep"
    if self.kind == "turn" and P6_TURN_MIN_V <= float(v_ego) <= P6_TURN_MAX_V:
      return P6_DESIRE[self.side], "turn"
    return None

  def _shadow(self, now: float, dh_desire: int, v_ego: float, would, sent: int):
    if now < self._next_shadow:
      return
    self._next_shadow = now + 1.0
    try:
      row = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "kind": self.kind, "side": self.side,
             "distM": self.dist, "vEgo": round(float(v_ego), 2), "dhDesire": int(dh_desire),
             "wouldSend": (would[0] if would else 0), "wouldKind": (would[1] if would else None),
             "sent": int(sent), "keepEnabled": self.enabled, "turnEnabled": self.turn_enabled,
             "repulse": bool(self.repulse_on), "pulses": int(self._pulses)}   # SPRINT35C_KEEP_REPULSE
      tmp = P6_SHADOW_FILE + ".tmp"
      with open(tmp, "w") as f:
        json.dump(row, f)
      os.replace(tmp, P6_SHADOW_FILE)
      # SPRINT26C_SHADOW_LOG: append history only when gradeable (maneuver in range) or the state changed
      sig = (self.kind, self.side, int(row['wouldSend']), int(sent), self.turn_enabled, self.enabled)
      if self.kind is not None or sig != getattr(self, '_last_sig', None):
        self._last_sig = sig
        try:
          if os.path.exists(P6_SHADOW_LOG) and os.path.getsize(P6_SHADOW_LOG) > P6_SHADOW_LOG_MAX:
            os.replace(P6_SHADOW_LOG, P6_SHADOW_LOG + '.1')
        except OSError:
          pass
        with open(P6_SHADOW_LOG, 'a') as lf:
          lf.write(json.dumps(row) + '\n')
    except Exception:
      pass

  def _sonata_repulse(self, now: float, sent: int, dh_desire: int, v_ego: float) -> int:
    """SPRINT35C_KEEP_REPULSE: re-pulse / hold an active keep. Identity when the switch is off."""
    if not (self.repulse_on and self.enabled):
      self._keep_code, self._keep_edge, self._pulses = None, 0.0, 0
      return sent
    code = P6_DESIRE.get(self.side) if (self.kind == "keep" and self.side) else None
    if (sent not in (5, 6) and code is not None and int(dh_desire) == 0 and self._keep_code == code
        and float(v_ego) >= P6_KEEP_HOLD_MIN_V):
      sent = code                                   # hold below the 16 m/s arming speed for the same maneuver
    if sent in (5, 6):
      if self._keep_code != sent:
        self._keep_code, self._keep_edge, self._pulses = sent, now, 0   # this frame is the first edge
      elif now - self._keep_edge >= P6_REPULSE_S:
        self._keep_edge = now
        self._pulses += 1
        return 0                                    # one none frame; the next frame is a fresh rising edge
      return sent
    self._keep_code = None
    return sent

  def desire(self, dh_desire: int, v_ego: float, now: float | None = None) -> int:
    """The desire to feed the model: the helper's own when it is busy, else keepLeft/keepRight when the route asks."""
    now = time.monotonic() if now is None else now
    self.poll(now)
    would = self._would(dh_desire, v_ego)
    sent = int(dh_desire)
    if would is not None:
      if would[1] == "keep" and self.enabled:
        sent = would[0]
      elif would[1] == "turn" and self.turn_enabled:
        sent = would[0]
    sent = self._sonata_repulse(now, sent, dh_desire, v_ego)   # SPRINT35C_KEEP_REPULSE
    self._shadow(now, dh_desire, v_ego, would, sent)
    return sent


_ROUTE_DESIRE = SonataRouteDesire()


def sonata_route_desire(dh_desire, cs):
  """Wrap DH.desire for the model input (SPRINT21C keep, SPRINT26B turn); no-op unless the matching flag exists."""
  try:
    return _ROUTE_DESIRE.desire(int(dh_desire), float(cs.vEgo))
  except Exception:
    return int(dh_desire)
