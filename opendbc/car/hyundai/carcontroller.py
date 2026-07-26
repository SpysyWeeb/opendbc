import math
from enum import IntEnum

import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

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

# BLaT: reduce low-speed release jerk only after the EPS and wheel confirm that
# the requested torque has broken static friction. Damping fades out from 12 to
# 15 mph and is never allowed to hold torque above the controller's demand.
TORQUE_DAMPING_VERSION = 2
TORQUE_DAMPING_FULL_SPEED = 12.0 * CV.MPH_TO_MS
TORQUE_DAMPING_ZERO_SPEED = 15.0 * CV.MPH_TO_MS
TORQUE_DAMPING_GAIN = 0.002  # normalized torque per steering-wheel deg/s
TORQUE_DAMPING_MAX = 0.20  # normalized torque
TORQUE_DAMPING_SUSTAIN_FRACTION = 0.90
TORQUE_DAMPING_RATE_FILTER_TAU = 0.08
TORQUE_DAMPING_REQUEST_SIGN_ENTER = 8  # CAN torque counts
TORQUE_DAMPING_REQUEST_SIGN_EXIT = 4
TORQUE_DAMPING_EPS_SIGN_ENTER = 1.0  # CR_Mdps_OutTq native units
TORQUE_DAMPING_EPS_SIGN_EXIT = 0.5
TORQUE_DAMPING_RATE_SIGN_ENTER = 12.0  # steering-wheel deg/s
TORQUE_DAMPING_RATE_SIGN_EXIT = 6.0
TORQUE_DAMPING_ANGLE_DELTA_DEADBAND = 0.05  # steering-wheel degrees per control frame
TORQUE_DAMPING_LATCH_HOLD_FRAMES = int(0.5 / DT_CTRL)


class TorqueDampingState(IntEnum):
  INACTIVE = 0
  DRIVER_OVERRIDE = 1
  SPEED_INACTIVE = 2
  REQUEST_DEADBAND = 3
  EPS_DEADBAND = 4
  WHEEL_STATIONARY = 5
  REQUEST_EPS_MISMATCH = 6
  EPS_MOTION_MISMATCH = 7
  DAMPING = 8
  SUSTAIN_FLOOR = 9
  TURN_IN_AUTHORITY = 10


class SignedHysteresis:
  """Stable sign detection with separate enter and exit thresholds."""

  def __init__(self, enter: float, exit_threshold: float):
    assert 0 <= exit_threshold < enter
    self.enter = enter
    self.exit = exit_threshold
    self.sign = 0

  def reset(self) -> None:
    self.sign = 0

  def update(self, value: float) -> int:
    if self.sign > 0:
      if value <= -self.enter:
        self.sign = -1
      elif value < self.exit:
        self.sign = 0
    elif self.sign < 0:
      if value >= self.enter:
        self.sign = 1
      elif value > -self.exit:
        self.sign = 0
    elif value >= self.enter:
      self.sign = 1
    elif value <= -self.enter:
      self.sign = -1
    return self.sign


class HyundaiLowSpeedTorqueDamping:
  """EPS-gated damping with an adaptive floor that only caps damping subtraction."""

  def __init__(self, steer_max: int):
    self.steer_max = steer_max
    self.request_sign = SignedHysteresis(TORQUE_DAMPING_REQUEST_SIGN_ENTER, TORQUE_DAMPING_REQUEST_SIGN_EXIT)
    self.eps_sign = SignedHysteresis(TORQUE_DAMPING_EPS_SIGN_ENTER, TORQUE_DAMPING_EPS_SIGN_EXIT)
    self.motion_sign = SignedHysteresis(TORQUE_DAMPING_RATE_SIGN_ENTER, TORQUE_DAMPING_RATE_SIGN_EXIT)
    self.rate_filter = FirstOrderFilter(0.0, TORQUE_DAMPING_RATE_FILTER_TAU, DT_CTRL)
    self.previous_angle: float | None = None
    self.angle_direction = 0
    self.was_moving = False
    self.stationary_frames = 0
    self.latch_direction = 0
    self.breakaway_latch = 0.0

    self.state = TorqueDampingState.INACTIVE
    self.signed_steering_rate = 0.0
    self.damping_requested = 0.0
    self.damping_applied = 0.0
    self.sustain_floor = 0.0

  def _clear_latch(self) -> None:
    self.stationary_frames = 0
    self.latch_direction = 0
    self.breakaway_latch = 0.0

  def _reset(self, steering_angle: float) -> None:
    self.request_sign.reset()
    self.eps_sign.reset()
    self.motion_sign.reset()
    self.rate_filter.x = 0.0
    self.signed_steering_rate = 0.0
    self.previous_angle = steering_angle
    self.angle_direction = 0
    self.was_moving = False
    self._clear_latch()

  def _update_signed_rate(self, steering_angle: float, steering_rate: float) -> float:
    if self.previous_angle is None:
      self.previous_angle = steering_angle
      return 0.0

    angle_delta = steering_angle - self.previous_angle
    self.previous_angle = steering_angle
    if angle_delta >= TORQUE_DAMPING_ANGLE_DELTA_DEADBAND:
      self.angle_direction = 1
    elif angle_delta <= -TORQUE_DAMPING_ANGLE_DELTA_DEADBAND:
      self.angle_direction = -1

    if self.angle_direction == 0:
      raw_rate = 0.0
    elif abs(steering_rate) > 1e-3:
      # CR_Mdps_WHLSpd is unsigned on affected Hyundai platforms.
      raw_rate = self.angle_direction * abs(steering_rate)
    else:
      raw_rate = angle_delta / DT_CTRL
    self.signed_steering_rate = float(self.rate_filter.update(raw_rate))
    return self.signed_steering_rate

  @staticmethod
  def _speed_scale(v_ego: float) -> float:
    fraction = float(np.clip((v_ego - TORQUE_DAMPING_FULL_SPEED) / (TORQUE_DAMPING_ZERO_SPEED - TORQUE_DAMPING_FULL_SPEED), 0.0, 1.0))
    return 1.0 - fraction * fraction * (3.0 - 2.0 * fraction)

  def update(
    self, demand: int, applied_last: int, eps_torque: float, steering_angle: float, steering_rate: float, v_ego: float, lat_active: bool,
    steering_pressed: bool, damping_blocked: bool = False,
  ) -> int:
    self.damping_requested = 0.0
    self.damping_applied = 0.0
    self.sustain_floor = 0.0

    if not lat_active:
      self._reset(steering_angle)
      self.state = TorqueDampingState.INACTIVE
      return demand
    if steering_pressed:
      self._reset(steering_angle)
      self.state = TorqueDampingState.DRIVER_OVERRIDE
      return demand
    if v_ego >= TORQUE_DAMPING_ZERO_SPEED:
      self._reset(steering_angle)
      self.state = TorqueDampingState.SPEED_INACTIVE
      return demand
    if damping_blocked:
      self._reset(steering_angle)
      self.state = TorqueDampingState.TURN_IN_AUTHORITY
      return demand

    signed_rate = self._update_signed_rate(steering_angle, steering_rate)
    request_sign = self.request_sign.update(demand)
    eps_sign = self.eps_sign.update(eps_torque)
    motion_sign = self.motion_sign.update(signed_rate)

    if self.latch_direction != 0 and request_sign != 0 and request_sign != self.latch_direction:
      self._clear_latch()

    motion_started = motion_sign != 0 and not self.was_moving
    self.was_moving = motion_sign != 0
    if motion_sign == 0:
      self.stationary_frames += 1
      if self.stationary_frames >= TORQUE_DAMPING_LATCH_HOLD_FRAMES:
        self._clear_latch()
    else:
      self.stationary_frames = 0
      if self.latch_direction != 0 and motion_sign != self.latch_direction:
        self._clear_latch()

    if request_sign == 0:
      self._clear_latch()
      self.state = TorqueDampingState.REQUEST_DEADBAND
      return demand
    if eps_sign == 0:
      self.state = TorqueDampingState.EPS_DEADBAND
      return demand
    if motion_sign == 0:
      self.state = TorqueDampingState.WHEEL_STATIONARY
      return demand

    if request_sign != eps_sign:
      self.state = TorqueDampingState.REQUEST_EPS_MISMATCH
      return demand
    if eps_sign != motion_sign:
      self.state = TorqueDampingState.EPS_MOTION_MISMATCH
      return demand

    if self.latch_direction != motion_sign:
      self.latch_direction = motion_sign
      self.breakaway_latch = abs(applied_last)
    elif motion_started:
      # Keep a conservative latch through a brief re-stick/release cycle.
      self.breakaway_latch = max(self.breakaway_latch, abs(applied_last))

    speed_scale = self._speed_scale(v_ego)
    self.damping_requested = min(abs(signed_rate) * TORQUE_DAMPING_GAIN * self.steer_max * speed_scale, TORQUE_DAMPING_MAX * self.steer_max)
    demand_magnitude = abs(demand)
    damped_magnitude = max(demand_magnitude - self.damping_requested, 0.0)
    self.sustain_floor = TORQUE_DAMPING_SUSTAIN_FRACTION * self.breakaway_latch

    # The floor protects only against our subtraction. It must never hold the
    # target above the undamped controller demand during overshoot correction.
    target_magnitude = min(demand_magnitude, max(damped_magnitude, self.sustain_floor))
    target = int(round(math.copysign(target_magnitude, demand)))
    self.damping_applied = max(demand_magnitude - abs(target), 0.0)
    self.state = TorqueDampingState.SUSTAIN_FLOOR if target_magnitude > damped_magnitude else TorqueDampingState.DAMPING
    return target


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


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.low_speed_torque_damping = HyundaiLowSpeedTorqueDamping(self.params.STEER_MAX)
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.cancel_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    damping_target = self.low_speed_torque_damping.update(
      new_torque,
      self.apply_torque_last,
      CS.out.steeringTorqueEps,
      CS.out.steeringAngleDeg,
      CS.out.steeringRateDeg,
      CS.out.vEgo,
      CC.latActive,
      CS.out.steeringPressed,
      actuators.torqueDampingBlocked,
    )
    apply_torque = apply_driver_steer_torque_limits(damping_target, self.apply_torque_last, CS.out.steeringTorque, self.params)

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
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl:
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

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.torqueBeforeDamping = new_torque / self.params.STEER_MAX
    new_actuators.torqueDampingRequested = self.low_speed_torque_damping.damping_requested / self.params.STEER_MAX
    new_actuators.torqueDampingApplied = self.low_speed_torque_damping.damping_applied / self.params.STEER_MAX
    new_actuators.torqueDampingState = int(self.low_speed_torque_damping.state)
    new_actuators.torqueDampingVersion = TORQUE_DAMPING_VERSION
    new_actuators.signedSteeringRateDeg = self.low_speed_torque_damping.signed_steering_rate
    new_actuators.torqueDampingFloor = self.low_speed_torque_damping.sustain_floor / self.params.STEER_MAX
    new_actuators.torqueBreakawayLatch = self.low_speed_torque_damping.breakaway_latch / self.params.STEER_MAX
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # AOL: present LKAS/LFA as active whenever we're steering, not only when fully engaged.
    # Sending torque while the LKAS HUD state reads "off" causes the MDPS to raise
    # CF_Mdps_ToiUnavail (steerTempUnavailable) at ~1 Hz, flashing the dash.
    # Longitudinal / SCC messages stay gated on CC.enabled.
    lkas_active = CC.latActive

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(lkas_active, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, lkas_active,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning))

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
      # Spysypilot: JerkUpperLimit gates the standstill exit -- at the stock 1.0 the car
      # takes ~1.4s to physically deliver startAccel against brake bleed (field-measured,
      # routes 5d/61: accCmd pinned while vEgo stays 0). 5.0 is the platform's own
      # factory-launch floor; stopping/off keep the gentle 1.0 for brake feathering
      if actuators.longControlState == LongCtrlState.pid:
        jerk = 3.0
      elif actuators.longControlState == LongCtrlState.starting:
        jerk = 5.0
      else:
        jerk = 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, lkas_active))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
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
