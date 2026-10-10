"""Independent lateral permission; exercised through freshly compiled safety hooks."""
import unittest

import pytest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

from opendbc.safety.tests import test_hyundai


@pytest.mark.parametrize("alt_buttons", [False, True])
@pytest.mark.parametrize("lka", [False, True])
@pytest.mark.parametrize("button", ["ADAPTIVE_CRUISE_MAIN_BTN", "LDA_BTN"])
def test_canfd_lateral_button(alt_buttons, lka, button):
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, (32 if alt_buttons else 0) | (16 if lka else 0))
  safety.init_tests()
  safety.set_alternative_experience(1024)
  packer = CANPackerSafety("hyundai_canfd_generated")
  msg = "CRUISE_BUTTONS_ALT" if alt_buttons else "CRUISE_BUTTONS"
  assert safety.safety_rx_hook(packer.make_can_msg_safety(msg, int(lka), {button: 1}))
  if alt_buttons and lka:
    # Upstream LKA configurations do not whitelist ALT buttons; stay fail-closed.
    assert not safety.get_lateral_allowed()
    return
  assert safety.get_lateral_allowed()
  assert not safety.get_controls_allowed()
  steer = packer.make_can_msg_safety("LKAS" if lka else "LFA", 0, {"StrTqReqVal": 1, "ActToiSta": 1})
  assert safety.safety_tx_hook(steer)


@pytest.mark.parametrize("model", [CarParams.SafetyModel.hondaNidec, CarParams.SafetyModel.toyota,
  CarParams.SafetyModel.ford, CarParams.SafetyModel.gm, CarParams.SafetyModel.noOutput])
def test_non_hyundai_cannot_arm_aol(model):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(model, 0) == 0
  safety.init_tests()
  safety.set_alternative_experience(1024)
  safety.set_controls_allowed(True)
  safety.set_aol_inputs(True, False)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(0x123, 0, bytes(8)))
  assert not safety.get_lateral_allowed()


class TestHyundaiAol(unittest.TestCase):
  def setUp(self):
    car = test_hyundai.TestHyundaiLongitudinalSafety()
    car.setUp()
    self.safety = car.safety
    for method in ("_rx", "_tx", "_button_msg", "_speed_msg", "_torque_cmd_msg", "_accel_msg"):
      setattr(self, method, getattr(car, method))
    self.safety.set_alternative_experience(1024)

  def test_lateral_only_button_engagement(self):
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self._rx(self._button_msg(0, main_button=1)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self._tx(self._accel_msg(1)))

  def _engage_aol(self):
    self._rx(self._button_msg(0, main_button=1))
    self._rx(self._button_msg(0, main_button=0))
    self.assertTrue(self.safety.get_lateral_allowed())

  def test_feature_disabled_and_mode_reset(self):
    self._engage_aol()
    self.safety.set_alternative_experience(0)
    self._rx(self._button_msg(0))
    self.assertFalse(self.safety.get_lateral_allowed())
    self.safety.set_alternative_experience(1024)
    self._engage_aol()
    self.safety.set_safety_hooks(0, 0)
    self.assertFalse(self.safety.get_lateral_allowed())

  def test_heartbeat_mismatch(self):
    self._engage_aol()
    self.safety.set_aol_heartbeat(True)
    for _ in range(4):
      self.safety.aol_heartbeat_tick()
    self.assertTrue(self.safety.get_lateral_allowed())
    self.safety.set_aol_heartbeat(False)
    for _ in range(3):
      self.safety.aol_heartbeat_tick()
    self.assertFalse(self.safety.get_lateral_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_rx_lag_revoke_and_no_rearm(self):
    self._engage_aol()
    self.safety.set_timer(2_000_000)
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_lateral_allowed())
    self._rx(self._button_msg(0, main_button=1))
    self.assertFalse(self.safety.get_lateral_allowed())

  def test_invalid_rx_immediately_revokes(self):
    self._engage_aol()
    msg = self._speed_msg(10)
    msg[0].data[0] ^= 1
    self.assertFalse(self._rx(msg))
    self.assertFalse(self.safety.get_lateral_allowed())

  def test_persistent_steering_fault_blocks_rearm(self):
    self._engage_aol()
    self.safety.set_aol_inputs(False, True)
    self._rx(self._button_msg(0))
    self.assertFalse(self.safety.get_lateral_allowed())
    self._rx(self._button_msg(0, main_button=1))
    self.assertFalse(self.safety.get_lateral_allowed())

  def test_acc_main_off_revokes(self):
    self._engage_aol()
    self.safety.set_aol_inputs(True, False)
    self._rx(self._button_msg(0))
    self.safety.set_aol_inputs(False, False)
    self._rx(self._button_msg(0))
    self.assertFalse(self.safety.get_lateral_allowed())

  def test_aol_still_checks_torque_driver_and_rates(self):
    self._engage_aol()
    self.assertTrue(self._tx(self._torque_cmd_msg(1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(100)))
    self.safety.set_torque_driver(-1000, -1000)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_speed_mismatch_revokes_lateral(self):
    self._engage_aol()
    self.safety.speed_mismatch_check(10.0)
    self.assertFalse(self.safety.get_lateral_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))

  def test_classic_lfa_button(self):
    from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket
    self._rx(make_CANPacket(0x391, 0, bytes([16]) + bytes(7)))
    self.assertTrue(self.safety.get_lateral_allowed())
