import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR

# SPRINT18A_DYNAMIC_STEER_MAX: one m/s more conservative than the Panda table, which the safety code
# evaluates at (min vehicle speed - 1 m/s).
# SPRINT31AQ_STEER384_ALL: the table is now flat - 384 at every speed, matching the Panda table
# {8,14,21}->{384,384,384}. The function and breakpoints are kept so the Panda/controller consistency
# test and any future taper need no structural change.
SONATA_STEER_MAX_BP = [9.0, 14.0, 21.0]
SONATA_STEER_MAX_V = [384.0, 384.0, 384.0]


def sonata_dynamic_steer_max(v_ego: float, steer_max: int) -> int:
  if steer_max <= 270:
    return steer_max
  return int(np.interp(float(v_ego), SONATA_STEER_MAX_BP, SONATA_STEER_MAX_V))


# SPRINT35A_OVERRIDE_RAMPIN: soften the torque hand-back after a driver override. See hotfix_35a_override_rampin.py.
import os as _sonata_ri_os  # SPRINT35A_OVERRIDE_RAMPIN
import time as _sonata_ri_time  # SPRINT35A_OVERRIDE_RAMPIN
SONATA_RAMPIN_ON_FILE = '/data/sonata_override_rampin_on'  # opt-in, off by default
SONATA_RAMPIN_GAP_MIN = 115        # CAN units: pent-up |request - applied| at release that arms the ramp (0.30 of 384)
SONATA_RAMPIN_UP = 1               # CAN units per 10 ms frame while armed (stock STEER_DELTA_UP is 2)
SONATA_RAMPIN_MAX_FRAMES = 100     # 1.0 s at 100 Hz
_sonata_ri_flag = {'t': -1e9, 'v': False}


def _sonata_rampin_enabled():
  now = _sonata_ri_time.monotonic()
  if now - _sonata_ri_flag['t'] > 1.0:
    _sonata_ri_flag['t'] = now
    _sonata_ri_flag['v'] = _sonata_ri_os.path.exists(SONATA_RAMPIN_ON_FILE)
  return _sonata_ri_flag['v']


class SonataOverrideRampIn:
  """SPRINT35A_OVERRIDE_RAMPIN: after the driver lets go with a large pent-up request, let the applied torque grow
  at half the stock rate for up to 1 s. Returns a value between apply_torque_last and the stock-limited
  apply_torque, so it can only ever be gentler than stock."""

  def __init__(self):
    self.prev_pressed = False
    self.frames_left = 0
    self.armed_count = 0

  def update(self, apply_torque, apply_torque_last, new_torque, steering_pressed, lat_active, flag_on=None):
    try:
      on = _sonata_rampin_enabled() if flag_on is None else flag_on
      if not on or not lat_active or steering_pressed:
        self.frames_left = 0
        self.prev_pressed = bool(steering_pressed) and bool(lat_active)
        return apply_torque
      if self.prev_pressed:
        self.prev_pressed = False
        if abs(new_torque - apply_torque_last) >= SONATA_RAMPIN_GAP_MIN:
          self.frames_left = SONATA_RAMPIN_MAX_FRAMES
          self.armed_count += 1
      if self.frames_left <= 0:
        return apply_torque
      self.frames_left -= 1
      if abs(new_torque - apply_torque) <= SONATA_RAMPIN_UP:
        self.frames_left = 0             # caught up with the request: back to stock
        return apply_torque
      if abs(apply_torque) > abs(apply_torque_last):
        step = apply_torque - apply_torque_last
        if abs(step) > SONATA_RAMPIN_UP:
          return int(apply_torque_last + (SONATA_RAMPIN_UP if step > 0 else -SONATA_RAMPIN_UP))
      return apply_torque
    except Exception:
      self.frames_left = 0
      return apply_torque              # never let this path break the steering message


from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import LeadDataCarController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData as _SonataCanFdLeadData  # SPRINT34D_VIRTUAL_TARGET
import os as _sonata_vt_os  # SPRINT34D_VIRTUAL_TARGET
import time as _sonata_vt_time  # SPRINT34D_VIRTUAL_TARGET
SONATA_VT_ON_FILE = '/data/sonata_virtual_target_on'  # SPRINT34D_VIRTUAL_TARGET: opt-in, off by default
SONATA_VT_DIST_M = 3.0      # stock reported 2.8-3.8 m for a real stopped lead at the stops that held
SONATA_VT_MAX_V = 0.3       # m/s: a controlled standstill, including idle-creep speeds
_sonata_vt_flag = {'t': -1e9, 'v': False}


def _sonata_vt_enabled():
  now = _sonata_vt_time.monotonic()
  if now - _sonata_vt_flag['t'] > 1.0:
    _sonata_vt_flag['t'] = now
    _sonata_vt_flag['v'] = _sonata_vt_os.path.exists(SONATA_VT_ON_FILE)
  return _sonata_vt_flag['v']


def sonata_virtual_target(enabled, long_active, gas_override, stopping, v_ego, lead_data, cruise_info, flag_on=None):
  """SPRINT34D_VIRTUAL_TARGET: (lead_data, cruise_info) to send. Stock values unless openpilot is holding a
  lead-less standstill, when a stationary object SONATA_VT_DIST_M ahead is reported to the ESC."""
  try:
    on = _sonata_vt_enabled() if flag_on is None else flag_on
    if not (on and enabled and long_active and not gas_override and stopping and v_ego < SONATA_VT_MAX_V
            and not lead_data.lead_visible):
      return lead_data, cruise_info
    vt = _SonataCanFdLeadData(2, SONATA_VT_DIST_M, 0.0, True)
    if cruise_info is not None:
      cruise_info = dict(cruise_info)
      cruise_info['ACC_ObjDist'] = SONATA_VT_DIST_M
      cruise_info['ACC_ObjRelSpd'] = 0.0
    return vt, cruise_info
  except Exception:
    return lead_data, cruise_info   # never let this path break the control message
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# On some HKG CAN and CAN FD non-CANFD_ALT_BUTTONS, the cancel button (CF_Clu_CruiseSwState / CRUISE_BUTTONS = 4) is
# a pause/resume toggle, not a dedicated cancel. Firing it mid-brake inadvertently can cause a re-enable attempt
# and triggers the "SCC Conditions Not Met" alert. Delaying the button send lets factory SCC disengage
# naturally on brake press. We send ~100 ms later if it fails to do so, or if we want to cancel for another reason.
CANCEL_BUTTON_DELAY_FRAMES = 10


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase, EsccCarController, LeadDataCarController, LongitudinalController, MadsCarController,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    EsccCarController.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    LeadDataCarController.__init__(self, CP)
    LongitudinalController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.sonata_rampin = SonataOverrideRampIn()  # SPRINT35A_OVERRIDE_RAMPIN
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.cancel_counter = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      LongitudinalController.update(self, CC, CS)

    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    sonata_steer_max = sonata_dynamic_steer_max(CS.out.vEgo, self.params.STEER_MAX)
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params,
                                                    steer_max=sonata_steer_max)
    apply_torque = self.sonata_rampin.update(apply_torque, self.apply_torque_last, new_torque,  # SPRINT35A_OVERRIDE_RAMPIN
                                             CS.out.steeringPressed, CC.latActive)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # Delay the cancel button send so the brake can disengage factory SCC first.
    # Reset whenever openpilot is no longer requesting cancel.
    self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      # Hold torque with induced temporary fault when cutting the actuation bit
      # FIXME: we don't use this with CAN FD?
      torque_fault = CC.latActive and not apply_steer_req

      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = self.tuning.actual_accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning,
                                              self.lkas_icon))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      self.lead_data, hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.ESCC))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, self.lfa_icon))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP, self.ESCC))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not self.ESCC.enabled:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl
    ccnc_non_hda2 = self.CP.flags & HyundaiFlags.CCNC and not lka_steering

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, self.lkas_icon))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      if ccnc_non_hda2:
        can_sends.extend(hyundaicanfd.create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, CC.hudControl, CC.leftBlinker,
                                                  CC.rightBlinker, CS.msg_161, CS.msg_162, CS.msg_1b5, CS.is_metric, CS.out, CS.main_cruise_enabled,
                                                  self.lfa_icon))
      else:
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      elif not ccnc_non_hda2:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        _vt_lead, _vt_cruise = sonata_virtual_target(CC.enabled, CC.longActive, CC.cruiseControl.override, stopping,  # SPRINT34D_VIRTUAL_TARGET
                                                     CS.out.vEgo, self.lead_data, CS.cruise_info if ccnc_non_hda2 else None)
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control, _vt_lead, CS.main_cruise_enabled, self.tuning,
                                                         _vt_cruise))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          # Here we send ACC message to cancel, not buttons. Don't delay
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          elif self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
