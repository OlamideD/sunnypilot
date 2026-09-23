"""Sonata HUD overlay for the comma four ("mici") onroad view, v2 "Tesla-style" (SONATA_MICI_HUD, Sprint 22b, 2026-09-09).

Drawn by the mici ModelRenderer after lane lines and the path (sunnypilot's rainbow road stays exactly as it is; this
draws on top of it). Owner review 2026-09-09: no rectangles, no black-and-white ground; car silhouettes, lights, signs.

What is drawn
  vehicles   radar tracks and the runtime's leads as perspective car silhouettes (body + roof + wheels), sized by distance:
             ego-lane lead = solid Tesla blue when radar-confirmed, amber outline when vision-only; adjacent-lane tracks
             translucent grey; a coasted (predicted) track is dimmer; a closing vehicle shows a small red tail bar
  ego        a silhouette at the bottom centre of the road; red silhouettes beside it while the blind spot is occupied,
             and the path is tinted red at the same time
  controls   the next traffic signal (3-lamp post, lamps unlit: no colour perception) or stop sign (red octagon "STOP")
             drawn ON the road at its projected distance, from the route (`nextTrafficControl`) or, with no route, from
             the road being driven (`aheadTrafficControl`, SPRINT21A_AHEAD_STOP); distance text under it
  badges     compact top-left column: OEM sign, route / maneuver, lane recommendation, LOW GRIP, weather
  pedals     BRAKE / GAS pads bottom-right (FrogPilot-style)
  limit      speed-limit roundel top-right from the runtime resolver (car sign camera first, map fallback), with the
             set target underneath when the +offset applies
Sources are the same Sonata daemon files as before (read at 2-10 Hz, parsed only on change). Nothing here feeds control.
"""
import json
import time
from pathlib import Path

import pyray as rl
from openpilot.system.ui.lib.shader_polygon import draw_polygon

SCENE = Path('/data/sonata_telemetry/live_scene.json')
ROUTE = Path('/data/sonata_drive10_lab/route_guidance_state.json')
RADAR = Path('/data/sonata_telemetry/radar_live.json')
LANE = Path('/data/sonata_telemetry/lane_planner.json')
BSM_TINT = rl.Color(255, 70, 60, 70)
BADGE_H = 40
BADGE_FONT = 24
BADGE_STEP = 46
BADGE_X = 12
BADGE_Y = 178          # below the 162 px set-speed circle
CONTROL_MAX_M = 120.0

C_BLUE = rl.Color(56, 150, 255, 235)
C_BLUE_EDGE = rl.Color(170, 215, 255, 255)
C_AMBER = rl.Color(255, 196, 60, 255)
C_GREY = rl.Color(200, 205, 215, 110)
C_GREY_EDGE = rl.Color(230, 235, 240, 170)
C_EGO = rl.Color(235, 238, 242, 230)
C_EGO_EDGE = rl.Color(120, 130, 140, 255)
C_RED = rl.Color(230, 60, 60, 230)
C_RED_EDGE = rl.Color(255, 170, 170, 255)
C_GLASS = rl.Color(40, 48, 60, 200)
C_WHEEL = rl.Color(20, 22, 26, 255)
C_TEXT = rl.Color(255, 255, 255, 255)


class _File:
  """Small change-driven JSON reader: stat at `period` s, parse only on mtime change, empty when older than `max_age`."""
  def __init__(self, path: Path, period: float, max_age: float):
    self.path, self.period, self.max_age = path, period, max_age
    self.obj = {}
    self._read = 0.0
    self._mtime = None

  def get(self, now: float):
    if now - self._read < self.period:
      return self.obj
    self._read = now
    try:
      st = self.path.stat()
      if st.st_mtime != self._mtime:
        self._mtime = st.st_mtime
        obj = json.loads(self.path.read_text())
        self.obj = obj if isinstance(obj, dict) else {}
      if time.time() - st.st_mtime > self.max_age:
        self.obj = {}
    except Exception:
      self.obj = {}
    return self.obj


def car_size(dist_m: float) -> float:
  """On-screen width in px of a car at `dist_m` (perspective-ish, clamped)."""
  return max(14.0, min(96.0, 1500.0 / max(dist_m, 4.0)))


def draw_car(x: float, y: float, w: float, fill, edge, solid=True, tail=None):
  """Rear-view car silhouette centred at (x, y) = ground contact point. w = width px."""
  h = w * 0.62
  bx, by = x - w / 2, y - h
  body = rl.Rectangle(bx, by + h * 0.28, w, h * 0.72)
  roof = rl.Rectangle(bx + w * 0.16, by, w * 0.68, h * 0.42)
  if solid:
    rl.draw_rectangle_rounded(body, 0.35, 6, fill)
    rl.draw_rectangle_rounded(roof, 0.5, 6, fill)
    rl.draw_rectangle_rounded(rl.Rectangle(bx + w * 0.22, by + h * 0.06, w * 0.56, h * 0.3), 0.5, 6, C_GLASS)  # rear window
  rl.draw_rectangle_rounded_lines_ex(body, 0.35, 6, 2, edge)
  rl.draw_rectangle_rounded_lines_ex(roof, 0.5, 6, 2, edge)
  ww, wh = max(3.0, w * 0.14), max(3.0, h * 0.14)
  rl.draw_rectangle(int(bx + w * 0.08), int(y - wh), int(ww), int(wh), C_WHEEL)
  rl.draw_rectangle(int(bx + w * 0.92 - ww), int(y - wh), int(ww), int(wh), C_WHEEL)
  if tail is not None:   # tail-light bar (red = closing on us)
    rl.draw_rectangle(int(bx + w * 0.12), int(y - h * 0.3), int(w * 0.76), max(2, int(h * 0.08)), tail)


def draw_traffic_light(x: float, y: float, s: float):
  """A 3-lamp signal on a post, lamps unlit (no colour perception). (x, y) = ground point, s = scale px."""
  post_h = s * 2.2
  rl.draw_rectangle(int(x - s * 0.08), int(y - post_h), max(2, int(s * 0.16)), int(post_h), rl.Color(70, 70, 75, 255))
  hw, hh = s * 0.55, s * 1.5
  rl.draw_rectangle_rounded(rl.Rectangle(x - hw / 2, y - post_h - hh, hw, hh), 0.3, 6, rl.Color(35, 35, 40, 245))
  for i, c in enumerate((rl.Color(120, 40, 40, 255), rl.Color(120, 100, 30, 255), rl.Color(30, 100, 50, 255))):
    rl.draw_circle(int(x), int(y - post_h - hh + hh * (0.2 + 0.3 * i)), max(2.0, s * 0.17), c)


def draw_stop_sign(x: float, y: float, s: float):
  """Red octagon with STOP on a post. (x, y) = ground point, s = scale px."""
  post_h = s * 1.6
  rl.draw_rectangle(int(x - s * 0.06), int(y - post_h), max(2, int(s * 0.12)), int(post_h), rl.Color(120, 120, 125, 255))
  cy = y - post_h - s * 0.55
  rl.draw_poly(rl.Vector2(x, cy), 8, s * 0.6, 22.5, rl.Color(200, 30, 40, 245))
  rl.draw_poly_lines_ex(rl.Vector2(x, cy), 8, s * 0.6, 22.5, 2, rl.Color(255, 255, 255, 230))
  if s >= 26:
    fs = int(s * 0.32)
    rl.draw_text("STOP", int(x - fs * 1.15), int(cy - fs * 0.5), fs, C_TEXT)


class SonataOverlay:
  def __init__(self):
    self.scene = _File(SCENE, 0.2, 1.5)
    self.route = _File(ROUTE, 0.5, 4.0)
    self.radar = _File(RADAR, 0.1, 1.0)
    self.lane = _File(LANE, 0.2, 4.0)

  # ------------------------------------------------------------------ labels
  @staticmethod
  def next_control(route):
    """(kind, distanceM) of the control to draw on the road: route first, then the road being driven."""
    if not isinstance(route, dict):
      return None, None
    c = route.get("nextTrafficControl") if route.get("status") == "ACTIVE" else None
    if not (isinstance(c, dict) and isinstance(c.get("distanceM"), (int, float))):
      c = route.get("aheadTrafficControl")
    if isinstance(c, dict) and isinstance(c.get("distanceM"), (int, float)) and 0.0 <= c["distanceM"] <= CONTROL_MAX_M:
      return str(c.get("kind") or ""), float(c["distanceM"])
    return None, None

  @staticmethod
  def route_label(route):
    status = str(route.get("status") or "")
    if status == "OFF_ROUTE":
      return "OFF ROUTE", rl.Color(220, 80, 60, 255)
    if status != "ACTIVE":
      return None, None
    maneuver = route.get("nextManeuver")
    dist = route.get("nextManeuverDistanceM")
    if not maneuver or not isinstance(dist, (int, float)) or dist > 600:
      name = str(route.get("routeName") or "").strip().upper()   # SPRINT16G_ROUTE_ARMED
      return ("ROUTE " + name[:22]) if name else "ROUTE ARMED", rl.Color(120, 200, 140, 255)
    label = str(maneuver).replace("_", " ").upper() + " " + str(int(dist)) + " m"
    road = str(route.get("nextRoad") or "").strip().upper()
    if road:
      label += " " + road[:18]
    return label, rl.Color(90, 170, 255, 255)

  @staticmethod
  def lane_label(lane):
    if not lane or lane.get("mode") == "off":
      return None, None
    req = lane.get("request")
    rec = lane.get("recommendation")
    if isinstance(req, dict) and req.get("direction") in ("left", "right"):
      arrow = "<" if req["direction"] == "left" else ">"
      return f"AUTO LANE {arrow} {req['direction'].upper()}", rl.Color(120, 220, 255, 255)
    if isinstance(rec, dict) and rec.get("direction") in ("left", "right"):
      arrow = "<" if rec["direction"] == "left" else ">"
      reason = str(rec.get("reason") or "").split(" ")[0].upper()[:10]
      cd = rec.get("countdownS")
      text = f"LANE {arrow} {rec['direction'].upper()} {reason}"
      if isinstance(cd, (int, float)) and cd > 0:
        text += f" {cd:.0f}s"
      return text, rl.Color(200, 200, 255, 255)
    return None, None

  @staticmethod
  def weather_label(route):
    w = route.get("weather") if isinstance(route, dict) else None
    if not isinstance(w, dict) or w.get("code") is None:
      return None, None
    code = int(w.get("code") or 0)
    temp = w.get("tempC")
    if code >= 95:
      text, colour = "STORM", rl.Color(255, 120, 60, 255)
    elif 71 <= code <= 77 or code in (85, 86):
      text, colour = "SNOW", rl.Color(200, 230, 255, 255)
    elif code >= 51:
      text, colour = "RAIN", rl.Color(120, 180, 255, 255)
    elif code >= 45:
      text, colour = "FOG", rl.Color(200, 200, 200, 255)
    elif isinstance(temp, (int, float)) and temp <= 2.0:
      text, colour = "ICE RISK", rl.Color(180, 220, 255, 255)
    else:
      return None, None
    if isinstance(temp, (int, float)):
      text += f" {temp:.0f}C"
    return text, colour

  # ------------------------------------------------------------------ drawing helpers
  @staticmethod
  def _badge(x, y, label, colour):
    w = max(150, int(BADGE_FONT * 0.56 * len(label)) + 20)
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, BADGE_H), 0.3, 6, rl.Color(18, 18, 18, 190))
    rl.draw_rectangle(x, y, 5, BADGE_H, colour)
    rl.draw_text(label, x + 14, y + 8, BADGE_FONT, C_TEXT)
    return y + BADGE_STEP

  def _draw_recommendation_banner(self, rect, lane):
    """SPRINT31V_BANNER: the recommendation as a full-width prompt, not a 40 px badge.

    Drawn in the upper third so it can never overlap openpilot's own alert band at the bottom -
    a real safety alert must always win. Advisory only: the driver accepts with the blinker.
    """
    if not isinstance(lane, dict) or lane.get("mode") == "off":
      return
    # SPRINT33F_TURN_PROMPT: a route turn outranks a lane-change suggestion - missing the turn ends
    # the route, while missing a lane change costs nothing. Same banner, same upper third, so it can
    # never cover openpilot's own alert band.
    tp = lane.get("turnPrompt")
    if isinstance(tp, dict) and tp.get("show") and tp.get("head"):
      bw = int(rect.width * 0.82)
      bx = int(rect.x + (rect.width - bw) / 2)
      by = int(rect.y + 96)
      bh = 104
      edge = rl.Color(255, 209, 102, 255)
      rl.draw_rectangle_rounded(rl.Rectangle(bx, by, bw, bh), 0.18, 8, rl.Color(16, 20, 28, 232))
      rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(bx, by, bw, bh), 0.18, 8, 4, edge)
      rl.draw_rectangle(bx, by, 10, bh, edge)
      head = str(tp["head"])[:34]
      sub = str(tp.get("sub") or "")[:40]
      hw = rl.measure_text(head, 44)
      rl.draw_text(head, bx + int((bw - hw) / 2), by + 14, 44, C_TEXT)
      sw = rl.measure_text(sub, 24)
      rl.draw_text(sub, bx + int((bw - sw) / 2), by + 66, 24, edge)
      return
    req = lane.get("request")
    rec = lane.get("recommendation")
    src = req if isinstance(req, dict) and req.get("direction") in ("left", "right") else None
    if src is None:
      src = rec if isinstance(rec, dict) and rec.get("direction") in ("left", "right") else None
    if src is None:
      return
    direction = src["direction"]
    arrow = "<" if direction == "left" else ">"
    auto = src is req
    head = ("AUTO LANE %s %s" if auto else "LANE %s %s") % (arrow, direction.upper())
    cd = src.get("countdownS")
    if isinstance(cd, (int, float)) and cd > 0:
      head += "  %.0fs" % cd
    sub = "SIGNAL %s TO ACCEPT" % direction.upper()
    reason = str(src.get("reason") or "").strip()
    if reason:
      sub = reason.upper()[:26] + "  -  " + sub

    bw = int(rect.width * 0.82)
    bx = int(rect.x + (rect.width - bw) / 2)
    by = int(rect.y + 96)
    bh = 104
    edge = C_BLUE_EDGE if auto else rl.Color(210, 226, 255, 255)
    rl.draw_rectangle_rounded(rl.Rectangle(bx, by, bw, bh), 0.18, 8, rl.Color(16, 20, 28, 225))
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(bx, by, bw, bh), 0.18, 8, 4, edge)
    rl.draw_rectangle(bx, by, 10, bh, C_BLUE if auto else rl.Color(120, 170, 255, 255))
    hw = rl.measure_text(head, 44)
    rl.draw_text(head, bx + int((bw - hw) / 2), by + 14, 44, C_TEXT)
    sw = rl.measure_text(sub, 24)
    rl.draw_text(sub, bx + int((bw - sw) / 2), by + 66, 24, edge)

  def _draw_badges(self, rect, scene, route, lane):
    x = int(rect.x + BADGE_X)
    y = int(rect.y + BADGE_Y)
    signs = []
    for item in scene.get("roadSigns", []) if scene else []:   # SPRINT15A_VISIBLE_BADGES
      if isinstance(item, dict) and item.get("observed"):
        name = str(item.get("additionalSignName") or item.get("name") or "SIGN").replace("_", " ").upper()
        if name not in signs:
          signs.append(name)
        if len(signs) >= 2:
          break
    for name in signs:
      y = self._badge(x, y, name[:22], rl.Color(255, 200, 60, 255))
    # SPRINT33G2_NO_DESTINATION: with no route the whole navigation layer is inert - turn-by-turn,
    # reroute, early lane choice, route turn preparation and the turn prompt. Captures #267-#275 were
    # all graded that way and nobody noticed. Say so where the driver can see it.
    if isinstance(route, dict) and str(route.get("status") or "") in ("NO_ROUTE", ""):
      y = self._badge(x, y, "NO DESTINATION", rl.Color(255, 123, 123, 255))
    for label, colour in (self.route_label(route), self.lane_label(lane)):
      if label:
        y = self._badge(x, y, label, colour)
    grip = lane.get("grip") if isinstance(lane, dict) else None   # SPRINT20I_LOW_GRIP
    if isinstance(grip, dict) and grip.get("lowGrip"):
      y = self._badge(x, y, "LOW GRIP " + str(grip.get("reason") or "").upper()[:16], rl.Color(120, 200, 255, 255))
    label, colour = self.weather_label(route)
    if label:
      y = self._badge(x, y, label, colour)

  @staticmethod
  def _draw_pedals(rect, cs):
    gas, brake = bool(cs.gasPressed), bool(cs.brakePressed)
    x0 = int(rect.x + rect.width - 16 - 2 * 66)
    y0 = int(rect.y + rect.height - 14 - 60 - 46)
    for i, (label, on, colour) in enumerate((("BRAKE", brake, rl.Color(230, 60, 60, 235)), ("GAS", gas, rl.Color(70, 200, 110, 235)))):
      x = x0 + i * 66
      rl.draw_rectangle_rounded(rl.Rectangle(x, y0, 58, 46), 0.3, 6, colour if on else rl.Color(30, 30, 30, 140))
      rl.draw_text(label, x + (6 if label == "BRAKE" else 14), y0 + 15, 17, rl.Color(255, 255, 255, 255 if on else 110))

  @staticmethod
  def _ground_point(r, x_m, y_m):
    """Screen point for a car-frame ground position (x ahead, y left-positive in model frame)."""
    path_x = r._path.raw_points[:, 0]
    idx = r._get_path_length_idx(path_x, x_m)
    z = r._path.raw_points[idx, 2] if idx < len(r._path.raw_points) else 0.0
    return r._map_to_screen(x_m, y_m + r._camera_offset, z + r._path_offset_z)

  def _draw_vehicles(self, r, radar, radar_state):
    """Radar tracks + runtime leads as silhouettes. Far first so near ones paint over them."""
    items = []
    lead_x = set()
    if radar_state is not None:
      for lead in (radar_state.leadOne, radar_state.leadTwo):
        if lead.present and lead.dRel > 0:
          items.append((float(lead.dRel), -float(lead.yRel), "lead", bool(lead.radar), True, float(lead.vRel)))
          lead_x.add(round(float(lead.dRel)))
    # SPRINT30D_NO_RAW_RADAR: the owner's "they track anything, even a shoe" - these are RAW bus-1 radar returns with no
    # vehicle validation, unlike the runtime leads above. Display only: sonata-radar-tracks.py keeps
    # running and keeps logging, so radar analysis is unaffected. Restore with /data/sonata_radar_boxes_on.
    _raw_boxes = Path("/data/sonata_radar_boxes_on").exists()
    for t in ((radar.get("tracks") or []) if (radar and _raw_boxes) else []):
      try:
        x, y = float(t["x"]), float(t["y"])
      except Exception:
        continue
      if x <= 1.0 or x > 150.0 or round(x) in lead_x:
        continue
      items.append((x, y, str(t.get("lane") or ""), t.get("state") == 3, t.get("motion") == 2, float(t.get("vy", 0.0) or 0.0)))
    for x, y, kind, confirmed, moving, vrel in sorted(items, key=lambda i: -i[0]):
      pt = self._ground_point(r, x, y)
      if not pt:
        continue
      w = car_size(x)
      tail = C_RED if vrel < -1.5 else None
      if kind == "lead":
        if confirmed:
          draw_car(pt[0], pt[1], w, C_BLUE, C_BLUE_EDGE, True, tail)
        else:
          draw_car(pt[0], pt[1], w, rl.Color(0, 0, 0, 0), C_AMBER, False, tail)
        if x < 60:
          rl.draw_text(f"{x:.0f} m", int(pt[0] - w * 0.3), int(pt[1] + 4), 16, C_TEXT)
      elif kind == "ego":
        draw_car(pt[0], pt[1], w, C_BLUE if confirmed else rl.Color(56, 150, 255, 120), C_BLUE_EDGE, True, tail)
      else:
        fill = C_GREY if confirmed else rl.Color(200, 205, 215, 60)
        draw_car(pt[0], pt[1], w, fill, C_GREY_EDGE, True, tail)

  def _draw_controls(self, r, route):
    kind, dist = self.next_control(route)
    if kind not in ("signal", "stop"):
      return
    # right-hand side of the road at that distance: 3.5 m right of the path centre (model y is left-positive)
    pt = self._ground_point(r, max(dist, 6.0), -3.5)
    if not pt:
      return
    s = max(16.0, min(70.0, 1600.0 / max(dist, 8.0)))
    if kind == "signal":
      draw_traffic_light(pt[0], pt[1], s)
    else:
      draw_stop_sign(pt[0], pt[1], s)
    rl.draw_text(f"{dist:.0f} m", int(pt[0] - 16), int(pt[1] + 4), 16, C_TEXT)

  @staticmethod
  def _draw_speed_limit(rect, sm):
    """Speed-limit roundel (white disc, red ring, km/h) from the runtime's speed-limit resolver, top-right under the
    set-speed area. Guarded: the UI may not subscribe to longitudinalPlanSP, and the value may be 0 (no limit known)."""
    try:
      sl = sm['longitudinalPlanSP'].speedLimit.resolver
      kph = float(sl.speedLimit) * 3.6
      final = float(sl.speedLimitFinal) * 3.6
    except Exception:
      return
    if kph < 5.0:
      return
    cx, cy, rad = int(rect.x + rect.width - 90), int(rect.y + 110), 44
    rl.draw_circle(cx, cy, rad, rl.Color(255, 255, 255, 240))
    rl.draw_ring(rl.Vector2(cx, cy), rad - 8, rad, 0, 360, 48, rl.Color(215, 30, 40, 255))
    txt = f"{kph:.0f}"
    rl.draw_text(txt, cx - (11 * len(txt)), cy - 14, 30, rl.Color(20, 20, 20, 255))
    if final >= 5.0 and abs(final - kph) >= 1.0:
      rl.draw_text(f"set {final:.0f}", cx - 30, cy + rad + 4, 16, C_TEXT)

  def _draw_ego(self, r, cs):
    rect = r._rect
    cx = rect.x + rect.width / 2
    gy = rect.y + rect.height - 30
    draw_car(cx, gy, 78, C_EGO, C_EGO_EDGE, True)
    if cs.leftBlindspot:
      draw_car(cx - 118, gy - 6, 62, C_RED, C_RED_EDGE, True)
    if cs.rightBlindspot:
      draw_car(cx + 118, gy - 6, 62, C_RED, C_RED_EDGE, True)
    if str(cs.gearShifter).split('.')[-1].lower() == 'reverse' and (cs.stockAeb or cs.stockFcw):
      rl.draw_text("REAR!", int(cx - 30), int(gy - 100), 26, C_RED)

  # ------------------------------------------------------------------ entry point
  def render(self, r, sm, radar_state, rect):
    """Called by the mici ModelRenderer after lane lines and path. `r` is the renderer (projection helpers)."""
    try:
      now = time.monotonic()
      scene, route, radar, lane = self.scene.get(now), self.route.get(now), self.radar.get(now), self.lane.get(now)
      cs = sm['carState']
      if r._path.projected_points.size and (cs.leftBlindspot or cs.rightBlindspot):
        draw_polygon(r._rect, r._path.projected_points, BSM_TINT)
      # SPRINT31Q_STRIP: the owner asked twice for these gone - "the radar boxes were still on
      # screen", then after the 31l badge restore brought them back: "the old UI seems to have
      # returned. The one with the stop signs, brake and gas pedal and radar boxes".
      # _draw_controls draws the stop-sign / traffic-light markers, _draw_vehicles the radar
      # boxes, _draw_pedals the brake+gas bars. The badges and the speed-limit roundel stay -
      # the badges are how a capability proposal reaches the driver at all.
      self._draw_ego(r, cs)
      self._draw_badges(rect, scene, route, lane)
      # self._draw_speed_limit(rect, sm)   # SPRINT32BJ_NO_SPEED_ROUNDEL: owner 2026-09-16 "remove the red speed icon from the UI"
      self._draw_recommendation_banner(rect, lane)   # SPRINT31V_BANNER: unmissable, upper third
    except Exception:
      pass   # the HUD must never take the UI down
