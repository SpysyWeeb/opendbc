import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarControl
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carcontroller import (
  CarController,
  TORQUE_DAMPING_MAX,
  TORQUE_DAMPING_SUSTAIN_FRACTION,
  HyundaiLowSpeedTorqueDamping,
  SignedHysteresis,
  TorqueDampingState,
)
from opendbc.car.hyundai.values import HyundaiFlags


STEER_MAX = 409


def update(damping, demand=300, applied_last=200, eps_torque=8.0, steering_angle=1.0, steering_rate=500.0, speed=5.0, lat_active=True,
           steering_pressed=False, damping_blocked=False):
  return damping.update(demand, applied_last, eps_torque, steering_angle, steering_rate, speed, lat_active, steering_pressed, damping_blocked)


class TestSignedHysteresis(unittest.TestCase):
  def test_deadband_and_exit_hysteresis(self):
    sign = SignedHysteresis(8.0, 4.0)
    assert sign.update(7.9) == 0
    assert sign.update(8.0) == 1
    assert sign.update(4.0) == 1
    assert sign.update(3.9) == 0
    assert sign.update(-7.9) == 0
    assert sign.update(-8.0) == -1
    assert sign.update(-4.0) == -1
    assert sign.update(-3.9) == 0

  def test_strong_reversal_changes_sign_immediately(self):
    sign = SignedHysteresis(8.0, 4.0)
    assert sign.update(20.0) == 1
    assert sign.update(-20.0) == -1


class TestHyundaiLowSpeedTorqueDamping(unittest.TestCase):
  def setUp(self):
    self.damping = HyundaiLowSpeedTorqueDamping(STEER_MAX)
    update(self.damping, demand=0, applied_last=0, eps_torque=0.0, steering_angle=0.0, steering_rate=0.0, lat_active=False)

  def test_stationary_wheel_preserves_demand(self):
    assert update(self.damping, steering_angle=0.0, steering_rate=0.0) == 300
    assert self.damping.state == TorqueDampingState.WHEEL_STATIONARY
    assert self.damping.damping_applied == 0.0

  def test_aligned_motion_is_damped(self):
    target = update(self.damping)
    assert 0 < target < 300
    assert self.damping.state == TorqueDampingState.DAMPING
    assert self.damping.breakaway_latch == 200
    assert self.damping.damping_applied == 300 - target

  def test_sustain_floor_only_caps_damping_subtraction(self):
    target = update(self.damping, demand=220, applied_last=200)
    expected_floor = TORQUE_DAMPING_SUSTAIN_FRACTION * 200
    assert target == round(expected_floor)
    assert self.damping.state == TorqueDampingState.SUSTAIN_FLOOR

    # A controller-demand collapse remains authoritative; the floor never
    # turns into a torque hold-up command.
    target = update(self.damping, demand=150, applied_last=target, steering_angle=2.0)
    assert target == 150
    assert target < expected_floor

  def test_eps_lag_disables_damping_during_request_reversal(self):
    for demand, eps_torque in ((-300, 8.0), (300, -8.0)):
      with self.subTest(demand=demand, eps_torque=eps_torque):
        self.setUp()
        angle = 1.0 if demand > 0 else -1.0
        assert update(self.damping, demand=demand, eps_torque=eps_torque, steering_angle=angle) == demand
        assert self.damping.state == TorqueDampingState.REQUEST_EPS_MISMATCH

  def test_eps_motion_mismatch_preserves_demand(self):
    # Feed enough negative angle motion for the filtered signed rate to enter
    # its negative hysteresis state while EPS and demand remain positive.
    target = 300
    for frame in range(1, 20):
      target = update(self.damping, steering_angle=-float(frame), steering_rate=100.0)
    assert target == 300
    assert self.damping.state == TorqueDampingState.EPS_MOTION_MISMATCH

  def test_angle_delta_deadband_prevents_direction_chatter(self):
    assert update(self.damping, steering_angle=0.02, steering_rate=100.0) == 300
    assert self.damping.signed_steering_rate == 0.0
    assert self.damping.state == TorqueDampingState.WHEEL_STATIONARY

    update(self.damping, steering_angle=0.10, steering_rate=100.0)
    assert self.damping.signed_steering_rate > 0.0

  def test_speed_fade(self):
    full = HyundaiLowSpeedTorqueDamping(STEER_MAX)
    mid = HyundaiLowSpeedTorqueDamping(STEER_MAX)
    off = HyundaiLowSpeedTorqueDamping(STEER_MAX)
    for damping in (full, mid, off):
      update(damping, demand=0, applied_last=0, eps_torque=0.0, steering_angle=0.0, steering_rate=0.0, lat_active=False)

    full_target = update(full, speed=12.0 * CV.MPH_TO_MS)
    mid_target = update(mid, speed=13.5 * CV.MPH_TO_MS)
    off_target = update(off, speed=15.0 * CV.MPH_TO_MS)
    assert full_target < mid_target < off_target
    assert off_target == 300
    assert off.state == TorqueDampingState.SPEED_INACTIVE
    self.assertAlmostEqual(mid.damping_requested, full.damping_requested / 2.0)

  def test_damping_cap(self):
    update(self.damping, demand=300, applied_last=0, steering_rate=5000.0)
    self.assertAlmostEqual(self.damping.damping_requested, TORQUE_DAMPING_MAX * STEER_MAX)

  def test_symmetry(self):
    positive = update(self.damping)
    negative_damping = HyundaiLowSpeedTorqueDamping(STEER_MAX)
    update(negative_damping, demand=0, applied_last=0, eps_torque=0.0, steering_angle=0.0, steering_rate=0.0, lat_active=False)
    negative = update(negative_damping, demand=-300, applied_last=-200, eps_torque=-8.0, steering_angle=-1.0)
    assert positive == -negative

  def test_driver_override_resets_latch_and_preserves_demand(self):
    update(self.damping)
    assert self.damping.breakaway_latch > 0
    assert update(self.damping, steering_angle=2.0, steering_pressed=True) == 300
    assert self.damping.state == TorqueDampingState.DRIVER_OVERRIDE
    assert self.damping.breakaway_latch == 0.0
    assert self.damping.signed_steering_rate == 0.0

  def test_undertracked_turn_in_guard_preserves_full_demand(self):
    update(self.damping)
    assert self.damping.breakaway_latch > 0
    assert update(self.damping, steering_angle=2.0, damping_blocked=True) == 300
    assert self.damping.state == TorqueDampingState.TURN_IN_AUTHORITY
    assert self.damping.damping_applied == 0.0
    assert self.damping.breakaway_latch == 0.0

  def test_brief_stall_keeps_and_updates_latch(self):
    update(self.damping, applied_last=200)
    assert self.damping.breakaway_latch == 200

    # Let the filtered rate fall through its exit threshold, but remain well
    # inside the 0.5-second latch hold.
    for _ in range(1, 30):
      update(self.damping, applied_last=200, steering_angle=1.0, steering_rate=0.0)
    assert self.damping.state == TorqueDampingState.WHEEL_STATIONARY
    assert self.damping.breakaway_latch == 200

    update(self.damping, applied_last=220, steering_angle=2.0)
    assert self.damping.breakaway_latch == 220

  def test_diagnostic_schema_exposes_named_gate_state(self):
    actuators = CarControl.Actuators.new_message()
    actuators.torqueDampingState = int(TorqueDampingState.SUSTAIN_FLOOR)
    actuators.torqueDampingVersion = 2
    actuators.torqueDampingBlocked = True
    actuators.signedSteeringRateDeg = -42.0
    assert str(actuators.torqueDampingState) == "sustainFloor"
    assert actuators.torqueDampingVersion == 2
    assert actuators.torqueDampingBlocked
    assert actuators.signedSteeringRateDeg == -42.0


class TestHyundaiAolCanfdMessages(unittest.TestCase):
  @patch.object(hyundaicanfd, "create_lfahda_cluster", return_value=("cluster",))
  @patch.object(hyundaicanfd, "create_steering_messages", return_value=[])
  def test_aol_uses_lateral_active_for_steering_and_icons(self, create_steering_messages, create_lfahda_cluster):
    controller = SimpleNamespace(
      CP=SimpleNamespace(flags=HyundaiFlags.CANFD, openpilotLongitudinalControl=False),
      CAN=object(),
      packer=object(),
      frame=0,
      last_button_frame=0,
    )
    controls = SimpleNamespace(
      enabled=False,
      latActive=True,
      leftBlinker=False,
      rightBlinker=False,
      cruiseControl=SimpleNamespace(cancel=False, resume=False),
    )

    CarController.create_canfd_msgs(controller, True, 100, 0.0, 0.0, False, SimpleNamespace(), SimpleNamespace(), controls)

    self.assertIs(create_steering_messages.call_args.args[3], controls.latActive)
    self.assertIs(create_lfahda_cluster.call_args.args[2], controls.latActive)
