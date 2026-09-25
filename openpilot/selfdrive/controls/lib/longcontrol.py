import numpy as np
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# SPRINT32BG_SMOOTH_STOP: final-stop smoothing (trial of sunnypilot PR #2034 with a causation term). See tools/sonata/sprint32/longcontrol.
import os as _sonata_os
SONATA_STOP_SMOOTH_OFF = '/data/sonata_stop_smooth_off'
SONATA_STOPPING_TIME_S = 2.5
SONATA_STOPPING_DISTANCE_M = 1.5
SONATA_STOPPED_SPEED = 0.02
SONATA_MIN_HOLD_ACCEL = -0.2
SONATA_CAUSE_TOL = 0.4          # hold only a command at least as strong as the measured deceleration (minus this)
SONATA_ROLLING_RATE = 0.3       # m/s^2/s ramp while still rolling (stock: 1.0)
SONATA_STOPPED_RATE = 2.0       # m/s^2/s once at rest: build the holding brake as before
# SPRINT34A_ARRIVE_HELD: idle creep settles the car at 0.03-0.12 m/s, so it never reaches SONATA_STOPPED_SPEED
# and the firm rate above never triggered. Below this speed a car that is not clearly decelerating is AT the
# creep equilibrium, and the hold is built fast. After 32BG, 2 of 52 lead stops crept with a -0.04 command.
SONATA_HOLD_ZONE_V = 0.30       # m/s
SONATA_HOLD_ZONE_DECEL = -0.10  # m/s^2: decelerating less than this in the hold zone = not stopping on its own
SONATA_CREEP_RATE = 4.0         # m/s^2/s: inside the 5.0 JerkLowerLimit sent to the car; not felt, the car is ~stopped


def sonata_in_creep_zone(v_ego, a_ego):
  return v_ego <= SONATA_HOLD_ZONE_V and (a_ego is None or a_ego > SONATA_HOLD_ZONE_DECEL)
_sonata_off = {'t': -1e9, 'v': False}


def _sonata_smooth_off():
  import time as _t
  now = _t.monotonic()
  if now - _sonata_off['t'] > 1.0:
    _sonata_off['t'] = now
    _sonata_off['v'] = _sonata_os.path.exists(SONATA_STOP_SMOOTH_OFF)
  return _sonata_off['v']


def sonata_hold_stopping(last_output_accel, v_ego, a_ego, a_target):
  """True while the current brake command is already stopping the car well enough to stop within 1.5 m / 2.5 s
  and is at least as strong as that deceleration (so it is its cause): hold it instead of ramping toward stopAccel."""
  if _sonata_smooth_off():
    return False
  cmd = float(last_output_accel)
  if sonata_in_creep_zone(v_ego, a_ego):
    return False   # SPRINT34A_ARRIVE_HELD: never freeze a command that is not stopping the car
  return (cmd <= SONATA_MIN_HOLD_ACCEL and a_target >= cmd and v_ego > SONATA_STOPPED_SPEED and a_ego < 0.0
          and v_ego <= -a_ego * SONATA_STOPPING_TIME_S and v_ego * v_ego <= -2.0 * a_ego * SONATA_STOPPING_DISTANCE_M
          and cmd <= a_ego + SONATA_CAUSE_TOL)   # a command WEAKER than the deceleration is not its cause: never hold it


SONATA_NOT_BITING_ACCEL = -0.3     # rolling with less deceleration than this under a real brake command: the brake is not biting
SONATA_REAL_BRAKE_CMD = -0.5


SONATA_STOP_ENTRY_OFF = '/data/sonata_stop_entry_off'   # SPRINT35G_STOP_ENTRY
SONATA_STOP_ENTRY_FLOOR = -2.0
_sonata_entry_off = {'t': -1e9, 'v': False}


def sonata_stop_entry_accel(last_output_accel, a_target, v_ego, a_ego):
  """SPRINT35G_STOP_ENTRY: the command on the tick LongControl enters `stopping`. While still rolling, continue the
  deceleration already in progress - never restart the ramp from 0 (#311: +1.557 -> 0 -> car accelerated)."""
  import time as _t
  now = _t.monotonic()
  if now - _sonata_entry_off['t'] > 1.0:
    _sonata_entry_off['t'] = now
    _sonata_entry_off['v'] = _sonata_os.path.exists(SONATA_STOP_ENTRY_OFF)
  if _sonata_entry_off['v'] or v_ego <= 0.05:
    return last_output_accel
  try:
    start = min(float(last_output_accel), float(a_target), float(a_ego), 0.0)
  except Exception:
    return last_output_accel
  return max(start, SONATA_STOP_ENTRY_FLOOR)


def sonata_stopping_rate(v_ego, a_ego=None, cmd=None):
  if _sonata_smooth_off():
    return 1.0
  if v_ego <= SONATA_STOPPED_SPEED:
    return SONATA_STOPPED_RATE
  if sonata_in_creep_zone(v_ego, a_ego):
    return SONATA_CREEP_RATE   # SPRINT34A_ARRIVE_HELD
  # never a rolling stop: a real brake command that is not producing deceleration ramps at the stock rate
  if a_ego is not None and cmd is not None and cmd <= SONATA_REAL_BRAKE_CMD and a_ego > SONATA_NOT_BITING_ACCEL:
    return 1.0
  return SONATA_ROLLING_RATE


def long_control_state_trans(CP_SP, active, long_control_state,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    _sonata_prev_state = self.long_control_state   # SPRINT35G_STOP_ENTRY
    self.long_control_state = long_control_state_trans(self.CP_SP, active, self.long_control_state,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if _sonata_prev_state != LongCtrlState.stopping:   # SPRINT35G_STOP_ENTRY: continue the deceleration in progress
        output_accel = sonata_stop_entry_accel(output_accel, a_target, CS.vEgo, CS.aEgo)
      if output_accel > self.CP.stopAccel and not sonata_hold_stopping(output_accel, CS.vEgo, CS.aEgo, a_target):   # SPRINT32BG_SMOOTH_STOP
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= sonata_stopping_rate(CS.vEgo, CS.aEgo, output_accel) * DT_CTRL   # SPRINT32BG_SMOOTH_STOP (stock: 1.0 m/s^2/s)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
