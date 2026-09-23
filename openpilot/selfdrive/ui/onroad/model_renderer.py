import colorsys
import json
import time
from pathlib import Path
import numpy as np
import pyray as rl
from openpilot.cereal import messaging
from opendbc.car.structs import car
from dataclasses import dataclass, field
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

from openpilot.selfdrive.ui.sunnypilot.onroad.model_renderer import ChevronMetrics, ModelRendererSP

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[tuple[float, float]] = field(default_factory=list)
  chevron: list[tuple[float, float]] = field(default_factory=list)
  fill_alpha: int = 0


class ModelRenderer(Widget, ChevronMetrics, ModelRendererSP):
  def __init__(self):
    Widget.__init__(self)
    ChevronMetrics.__init__(self)
    ModelRendererSP.__init__(self)
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]
    self._counter = -1
    self._camera_offset = ui_state.params.get("CameraOffset", return_default=True) if ui_state.active_bundle else 0.0
    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)
    self._sonata_scene = {}
    self._sonata_scene_read = 0.0
    self._sonata_scene_path = Path('/data/sonata_telemetry/live_scene.json')
    # SPRINT16D_MANEUVER_HUD: advisory route guidance state (next maneuver + distance).
    self._sonata_route = {}
    self._sonata_route_read = 0.0
    self._sonata_route_path = Path('/data/sonata_drive10_lab/route_guidance_state.json')
    # SPRINT19B_RADAR_HUD: passive radar tracks (10 Hz file) and the lane planner (change-driven file).
    self._sonata_radar = {}
    self._sonata_radar_read = 0.0
    self._sonata_radar_mtime = None
    self._sonata_radar_path = Path('/data/sonata_telemetry/radar_live.json')
    self._sonata_lane = {}
    self._sonata_lane_read = 0.0
    self._sonata_lane_mtime = None
    self._sonata_lane_path = Path('/data/sonata_telemetry/lane_planner.json')
    self._sonata_bsm_tint = rl.Color(255, 70, 60, 70)  # SPRINT20E_HUD
    self._sonata_rx = {}   # SPRINT32D_ROUTE_EXEC_HUD: route executor live state (5 Hz, gone 1.5 s after the daemon stops)
    self._sonata_rx_read = 0.0
    self._sonata_rx_mtime = None
    self._sonata_rx_path = Path('/data/sonata_telemetry/route_exec_live.json')

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := Params().get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    # Check if data is up-to-date
    if (sm.recv_frame["extrinsicsCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode

    extrinsics_calibration = sm['extrinsicsCalibration']
    self._path_offset_z = extrinsics_calibration.height[0] if extrinsics_calibration.height else HEIGHT_INIT[0]

    if self._counter % 60 == 0:
      self._camera_offset = ui_state.params.get("CameraOffset", return_default=True) if ui_state.active_bundle else 0.0
    self._counter += 1

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    render_lead_indicator = self._longitudinal_control and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array)
      self._transform_dirty = False

    self._update_sonata_scene()
    # Draw elements
    self._draw_lane_lines()
    self._draw_path(sm)

    if render_lead_indicator and radar_state:
      self._draw_lead_indicator()
      self.chevron_metrics.draw_lead_status(sm, radar_state, self._rect, self._lead_vehicles)

    self._draw_sonata_scene(rect)
    self._draw_sonata_pedals(sm)  # SPRINT20E_HUD

    # SONATA_SOURCE_AWARE_SCENE_V1
    self._draw_sonata_source_scene(sm, radar_state)

  def _update_sonata_scene(self):
    now = time.monotonic()
    if now - self._sonata_scene_read < 0.20:
      return
    self._sonata_scene_read = now
    try:
      obj = json.loads(self._sonata_scene_path.read_text())
      if not isinstance(obj, dict) or now - float(obj.get("mono", -1e9)) > 1.5:
        self._sonata_scene = {}
      else:
        self._sonata_scene = obj
    except Exception:
      self._sonata_scene = {}
    self._update_sonata_route(now)
    self._update_sonata_radar(now)

  def _update_sonata_route(self, now):
    # SPRINT16D_MANEUVER_HUD: <= 2 Hz; the guidance file must be younger than 4 s.
    if now - self._sonata_route_read < 0.5:
      return
    self._sonata_route_read = now
    try:
      age = time.time() - self._sonata_route_path.stat().st_mtime
      obj = json.loads(self._sonata_route_path.read_text())
      self._sonata_route = obj if (isinstance(obj, dict) and 0.0 <= age <= 4.0) else {}
    except Exception:
      self._sonata_route = {}

  def _update_sonata_radar(self, now):
    # SPRINT19B_RADAR_HUD: parse only when the files changed; radar <= 10 Hz, lane planner <= 5 Hz.
    if now - self._sonata_radar_read >= 0.1:
      self._sonata_radar_read = now
      try:
        st = self._sonata_radar_path.stat()
        if st.st_mtime != self._sonata_radar_mtime:
          self._sonata_radar_mtime = st.st_mtime
          obj = json.loads(self._sonata_radar_path.read_text())
          self._sonata_radar = obj if isinstance(obj, dict) else {}
        if time.time() - st.st_mtime > 1.0:
          self._sonata_radar = {}
      except Exception:
        self._sonata_radar = {}
    if now - self._sonata_rx_read >= 0.2:   # SPRINT32D_ROUTE_EXEC_HUD
      self._sonata_rx_read = now
      try:
        st = self._sonata_rx_path.stat()
        if st.st_mtime != self._sonata_rx_mtime:
          self._sonata_rx_mtime = st.st_mtime
          obj = json.loads(self._sonata_rx_path.read_text())
          self._sonata_rx = obj if isinstance(obj, dict) else {}
        if time.time() - st.st_mtime > 1.5:
          self._sonata_rx = {}
      except Exception:
        self._sonata_rx = {}
    if now - self._sonata_lane_read >= 0.2:
      self._sonata_lane_read = now
      try:
        st = self._sonata_lane_path.stat()
        if st.st_mtime != self._sonata_lane_mtime:
          self._sonata_lane_mtime = st.st_mtime
          obj = json.loads(self._sonata_lane_path.read_text())
          self._sonata_lane = obj if isinstance(obj, dict) else {}
        if time.time() - st.st_mtime > 4.0:
          self._sonata_lane = {}
      except Exception:
        self._sonata_lane = {}

  def _sonata_lane_label(self):
    lane = self._sonata_lane
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

  def _draw_sonata_radar_tracks(self, path_x):
    # SPRINT19B_RADAR_HUD: white boxes for measured tracks, grey for coasted; nothing here feeds control.
    tracks = self._sonata_radar.get("tracks") if self._sonata_radar else None
    if not tracks:
      return
    for t in tracks:
      try:
        x = float(t["x"]); y = float(t["y"])
      except Exception:
        continue
      if x <= 1.0 or x > 150.0:
        continue
      idx = self._get_path_length_idx(path_x, x)
      z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
      point = self._map_to_screen(x, y + self._camera_offset, z + self._path_offset_z)
      if not point:
        continue
      size = max(10, min(46, int(1100.0 / x)))
      measured = t.get("state") == 3
      moving = t.get("motion") == 2
      colour = rl.Color(255, 255, 255, 235) if measured else rl.Color(160, 160, 170, 170)
      rx, ry = int(point[0] - size / 2), int(point[1] - size * 0.8)
      rl.draw_rectangle_lines_ex(rl.Rectangle(rx, ry, size, int(size * 1.4)), 2 if measured else 1, colour)
      if moving and measured:
        rl.draw_rectangle(rx + 2, ry + 2, max(size - 4, 2), 4, rl.Color(120, 220, 255, 230))
      if t.get("lane") == "ego" and measured:
        rl.draw_text(f"{x:.0f}", rx, ry - 18, 16, colour)

  def _sonata_route_label(self):
    route = self._sonata_route
    status = str(route.get("status") or "")
    if status == "OFF_ROUTE":
      return "OFF ROUTE", rl.Color(220, 80, 60, 255)
    if status != "ACTIVE":
      return None, None
    maneuver = route.get("nextManeuver")
    dist = route.get("nextManeuverDistanceM")
    # SPRINT18C_SIGNAL_HUD: the next traffic control ahead takes the slot when it is the closer thing.
    control = route.get("nextTrafficControl")
    if isinstance(control, dict):
      c_dist = control.get("distanceM")
      if isinstance(c_dist, (int, float)) and c_dist <= 120 and (not isinstance(dist, (int, float)) or c_dist < dist):
        kind = str(control.get("kind") or "")
        if kind == "stop":
          return "STOP SIGN " + str(int(c_dist)) + " m", rl.Color(235, 70, 60, 255)
        if kind == "signal":
          return "SIGNAL " + str(int(c_dist)) + " m", rl.Color(255, 190, 40, 255)
    if not maneuver or not isinstance(dist, (int, float)) or dist > 600:
      # SPRINT16G_ROUTE_ARMED: the route is loaded and armed; show its name until a maneuver is near.
      name = str(route.get("routeName") or "").strip().upper()
      return ("ROUTE " + name[:22]) if name else "ROUTE ARMED", rl.Color(120, 200, 140, 255)
    label = str(maneuver).replace("_", " ").upper() + " " + str(int(dist)) + " m"
    road = str(route.get("nextRoad") or "").strip().upper()
    if road:
      label += " " + road[:18]
    return label, rl.Color(90, 170, 255, 255)

  def _draw_sonata_scene(self, rect):
    scene = self._sonata_scene
    if not scene and not self._sonata_route and not self._sonata_lane and not self._sonata_rx:   # SPRINT32D_ROUTE_EXEC_HUD
      return
    signs = []
    for item in scene.get("roadSigns", []):
      if not isinstance(item, dict) or not item.get("observed"):
        continue
      name = str(item.get("additionalSignName") or item.get("name") or "SIGN").replace("_", " ").upper()
      if name not in signs:
        signs.append(name)
      if len(signs) >= 4:
        break
    # SPRINT15A_VISIBLE_BADGES: below the set-speed HUD box (x+60, y+45, h 204), which is
    # painted after this renderer and previously hid the badges completely.
    x = int(rect.x + 60)
    y = int(rect.y + 45 + 204 + 18)
    for name in signs:
      label = "OEM " + name[:22]
      w = max(200, 17 * len(label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, rl.Color(255, 200, 60, 255))
      rl.draw_text(label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58

    # SPRINT16D_MANEUVER_HUD: next route maneuver below the OEM sign badges.
    route_label, route_color = self._sonata_route_label()
    if route_label:
      w = max(200, 17 * len(route_label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, route_color)
      rl.draw_text(route_label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58
    # SPRINT19B_RADAR_HUD: lane planner recommendation / auto request badge.
    lane_label, lane_color = self._sonata_lane_label()
    if lane_label:
      w = max(200, 17 * len(lane_label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, lane_color)
      rl.draw_text(lane_label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58
    # SPRINT32D_ROUTE_EXEC_HUD: route executor state; red TAKEOVER is the visible takeover event (DEVICE_HANDOFF S4).
    rx_label, rx_color = self._sonata_route_exec_label()
    if rx_label:
      w = max(200, 17 * len(rx_label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, rx_color)
      rl.draw_text(rx_label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58
    # SPRINT20I_LOW_GRIP: badge while the grip monitor reports low grip.
    grip = (self._sonata_lane or {}).get("grip") if isinstance(self._sonata_lane, dict) else None
    if isinstance(grip, dict) and grip.get("lowGrip"):
      grip_label = "LOW GRIP " + str(grip.get("reason") or "").upper()[:16]
      w = max(200, 17 * len(grip_label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, rl.Color(120, 200, 255, 255))
      rl.draw_text(grip_label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58
    # SPRINT20E_HUD: weather badge.
    weather_label, weather_color = self._sonata_weather_label()
    if weather_label:
      w = max(160, 17 * len(weather_label) + 24)
      rl.draw_rectangle(x, y, w, 50, rl.Color(18, 18, 18, 205))
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y, w, 50), 3, weather_color)
      rl.draw_text(weather_label, x + 12, y + 10, 30, rl.Color(255, 255, 255, 255))
      y += 58

    vehicles = scene.get("frontCameraVehicles", [])
    object_count = 0
    for group in vehicles if isinstance(vehicles, list) else []:
      if isinstance(group, dict):
        objects = group.get("objects") or []
        if isinstance(objects, list):
          object_count += len(objects)
    if object_count:
      label = "OEM VEHICLES " + str(object_count)
      # Below the top-right buttons (border 30 + button 192) rather than underneath them.
      rl.draw_rectangle(int(rect.x + rect.width - 30 - 260), int(rect.y + 45 + 204 + 18), 260, 50, rl.Color(30, 80, 150, 205))
      rl.draw_text(label, int(rect.x + rect.width - 30 - 248), int(rect.y + 45 + 204 + 28), 30, rl.Color(255, 255, 255, 255))

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, np.array(model.position.y) + self._camera_offset, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, np.array(lane_line.y) + self._camera_offset, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, np.array(road_edge.y) + self._camera_offset, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.present:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel + self._camera_offset, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points
    for i, lane_line in enumerate(self._lane_lines):
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, 0.025 * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    # Update road edges using raw points
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, 0.025, 0.0, max_idx, max_distance)

    # Update path using raw points
    if lead and lead.present:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, self._get_path_half_width(), self._path_offset_z, max_idx, max_distance, allow_invert=False
    )

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=segment_colors,
      stops=gradient_stops,
    )

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect):
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _draw_lane_lines(self):
    """Draw lane lines and road edges"""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      alpha = np.clip(self._lane_line_probs[i], 0.0, 0.7)
      color = rl.Color(255, 255, 255, int(alpha * 255))
      draw_polygon(self._rect, lane_line.projected_points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      color = rl.Color(255, 0, 0, int(alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if ui_state.rainbow_path and self._lateral_active:
      self.rainbow_path.draw_rainbow_path(self._rect, self._path)
      return

    if self._experimental_mode:
      # Draw with acceleration coloring
      if len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )
      draw_polygon(self._rect, self._path.projected_points, gradient=gradient)
    # SPRINT20E_HUD: blind-spot tint on the path while the OEM BSM reports a vehicle on either side.
    try:
      cs = sm['carState']
      if cs.leftBlindspot or cs.rightBlindspot:
        draw_polygon(self._rect, self._path.projected_points, self._sonata_bsm_tint)
    except Exception:
      pass

  def _draw_sonata_pedals(self, sm):
    # SPRINT20E_HUD: gas / brake indicators (FrogPilot "pedals") bottom-right, only while pressed or engaged.
    try:
      cs = sm['carState']
      gas, brake = bool(cs.gasPressed), bool(cs.brakePressed)
    except Exception:
      return
    rect = self._rect
    x0 = int(rect.x + rect.width - 30 - 2 * 74)
    y0 = int(rect.y + rect.height - 30 - 54)
    for i, (label, on, colour) in enumerate((("BRAKE", brake, rl.Color(230, 60, 60, 235)), ("GAS", gas, rl.Color(70, 200, 110, 235)))):
      x = x0 + i * 74
      fill = colour if on else rl.Color(30, 30, 30, 150)
      rl.draw_rectangle(x, y0, 64, 54, fill)
      rl.draw_rectangle_lines_ex(rl.Rectangle(x, y0, 64, 54), 2, rl.Color(200, 200, 200, 180 if on else 90))
      rl.draw_text(label, x + 8 if label == "BRAKE" else x + 16, y0 + 17, 18, rl.Color(255, 255, 255, 255 if on else 120))

  def _sonata_route_exec_label(self):
    # SPRINT32D_ROUTE_EXEC_HUD: ROUTE <STATE> <km/h> [STOP] green while actuating, grey ROUTE SHADOW otherwise,
    # red ROUTE TAKEOVER <reason> when the executor asks the driver to take over.
    rx = self._sonata_rx
    if not isinstance(rx, dict) or not rx:
      return None, None
    state = str(rx.get("state") or "").upper()[:14]
    if not state:
      return None, None
    if rx.get("takeover"):
      why = str(rx.get("why") or "").replace("_", " ").upper()[:22]
      return ("ROUTE TAKEOVER " + why).strip(), rl.Color(255, 80, 80, 255)
    vt = rx.get("vTarget")
    v = " %d" % int(round(float(vt) * 3.6)) if isinstance(vt, (int, float)) else ""
    if rx.get("actuates"):
      return "ROUTE " + state + v + (" STOP" if rx.get("shouldStop") else ""), rl.Color(120, 255, 160, 255)
    if rx.get("armed") and rx.get("configured") and str(rx.get("mode") or "shadow") != "shadow":   # SPRINT32K_ARMED_LABEL
      return "ROUTE ARMED " + state, rl.Color(255, 200, 60, 255)
    return "ROUTE SHADOW " + state, rl.Color(170, 170, 170, 255)

  def _sonata_weather_label(self):
    # SPRINT20E_HUD: weather from the guidance daemon (open-meteo, Sprint 20f).
    w = (self._sonata_route or {}).get("weather") if isinstance(self._sonata_route, dict) else None
    if not isinstance(w, dict) or w.get("code") is None:
      return None, None
    code = int(w.get("code") or 0)
    temp = w.get("tempC")
    if code >= 95:
      text, colour = "STORM", rl.Color(255, 120, 60, 255)
    elif code >= 71 and code <= 77 or code in (85, 86):
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

  def _draw_sonata_car_glyph(self, x, y, fill, outline, solid=True):
    w, h = 30, 54
    rx, ry = int(x - w / 2), int(y - h / 2)
    if solid:
      rl.draw_rectangle(rx, ry, w, h, fill)
    rl.draw_rectangle_lines_ex(rl.Rectangle(rx, ry, w, h), 3, outline)
    rl.draw_rectangle(rx + 5, ry + 8, w - 10, 10, rl.Color(outline.r, outline.g, outline.b, 180))

  def _draw_sonata_source_scene(self, sm, radar_state):
    # Visualization only. Never feeds planning or controls.
    car_state = sm['carState']
    rect = self._rect
    cx = rect.x + rect.width / 2
    ego_y = rect.y + rect.height - 100

    # Ego reference car.
    self._draw_sonata_car_glyph(cx, ego_y, rl.Color(38, 45, 56, 220), rl.Color(235, 240, 245, 235), True)

    # OEM blind-spot indicators are occupancy warnings, not object coordinates.
    bsm_fill = rl.Color(255, 154, 31, 210)
    bsm_outline = rl.Color(255, 220, 120, 255)
    if car_state.leftBlindspot:
      self._draw_sonata_car_glyph(cx - 78, ego_y - 8, bsm_fill, bsm_outline, True)
    if car_state.rightBlindspot:
      self._draw_sonata_car_glyph(cx + 78, ego_y - 8, bsm_fill, bsm_outline, True)

    # When reversing, only show a generic rear hazard marker if the OEM itself
    # reports stock AEB/FCW. We do not manufacture a rear object position.
    gear_text = str(car_state.gearShifter).split('.')[-1].lower()
    if gear_text == 'reverse' and (car_state.stockAeb or car_state.stockFcw):
      self._draw_sonata_car_glyph(cx, ego_y + 66, rl.Color(210, 35, 45, 230), rl.Color(255, 175, 175, 255), True)

    if radar_state is None or self._path.raw_points.size == 0:
      return
    path_x = self._path.raw_points[:, 0]
    self._draw_sonata_radar_tracks(path_x)  # SPRINT19B_RADAR_HUD
    for lead in (radar_state.leadOne, radar_state.leadTwo):
      if not lead.present or lead.dRel <= 0:
        continue
      idx = self._get_path_length_idx(path_x, lead.dRel)
      z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
      point = self._map_to_screen(lead.dRel, -lead.yRel + self._camera_offset, z + self._path_offset_z)
      if not point:
        continue
      if lead.radar:
        # radar=True means the selected lead is matched to a radar track.
        fill, outline, solid = rl.Color(15, 205, 225, 220), rl.Color(165, 250, 255, 255), True
      else:
        # Vision/model-only lead: outline, deliberately not presented as radar.
        fill, outline, solid = rl.Color(0, 0, 0, 0), rl.Color(255, 210, 70, 255), False
      self._draw_sonata_car_glyph(point[0], point[1], fill, outline, solid)

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      rl.draw_triangle_fan(lead.glow, len(lead.glow), rl.Color(218, 202, 37, 255))
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), rl.Color(201, 34, 49, lead.fill_alpha))

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_distance: float) -> int:
    """Get the index corresponding to the given path distance"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_distance)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, max_distance: float, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]

    # Interpolate around max_idx so path end is smooth (max_distance is always >= p0.x)
    if 0 < max_idx < line.shape[0] - 1:
      p0 = line[max_idx]
      p1 = line[max_idx + 1]
      x0, x1 = p0[0], p1[0]
      interp_y = np.interp(max_distance, [x0, x1], [p0[1], p1[1]])
      interp_z = np.interp(max_distance, [x0, x1], [p0[2], p1[2]])
      interp_point = np.array([max_distance, interp_y, interp_z], dtype=points.dtype)
      points = np.concatenate((points, interp_point[None, :]), axis=0)

    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  @staticmethod
  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]
