import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.steering_request import MAX_ANGLE, MAX_ANGLE_CONSECUTIVE_FRAMES, MAX_ANGLE_FRAMES
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags


class TestHyundaiSteeringRequestState(unittest.TestCase):
  PLATFORMS = (CAR.HYUNDAI_PALISADE, CAR.KIA_EV6)

  @staticmethod
  def make_interface(platform):
    CP = CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP)
    CI.update([])
    return CI

  @staticmethod
  def make_control(lat_active):
    CC = structs.CarControl(enabled=lat_active, latActive=lat_active)
    CC.actuators.torque = 1.0
    return CC.as_reader()

  @staticmethod
  def get_steering_command(CP, can_sends):
    if CP.flags & HyundaiFlags.CANFD:
      can_bus = CanBus(CP)
      if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
        msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
        bus = can_bus.ACAN
      else:
        msg = "LFA"
        bus = can_bus.ECAN
      request_signal = "ActToiSta"
      torque_signal = "StrTqReqVal"
    else:
      msg = "LKAS11"
      request_signal = "CF_Lkas_ActToi"
      torque_signal = "CR_Lkas_StrToqReq"
      bus = 0

    parser = CANParser(DBC[CP.carFingerprint][Bus.pt], [(msg, 0)], bus)
    assert parser.update([0, can_sends]) == parser.addresses
    return bool(parser.vl[msg][request_signal]), int(parser.vl[msg][torque_signal])

  def test_schema_defaults_and_round_trip(self):
    actuators = structs.CarControl.Actuators()
    assert not actuators.steeringRequestActive
    assert not actuators.steeringRequestActiveValid
    assert actuators.steeringRequestFaultAvoidanceCounter == 0

    actuators.steeringRequestActive = True
    actuators.steeringRequestActiveValid = True
    actuators.steeringRequestFaultAvoidanceCounter = 42
    with structs.CarControl.Actuators.from_bytes(actuators.to_bytes()) as decoded:
      assert decoded.steeringRequestActive
      assert decoded.steeringRequestActiveValid
      assert decoded.steeringRequestFaultAvoidanceCounter == 42

  def test_active_and_inactive_request_state(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform, lat_active=True):
        CI = self.make_interface(platform)
        actuators, can_sends = CI.apply(self.make_control(True), 0)
        assert actuators.steeringRequestActiveValid
        assert actuators.steeringRequestActive
        assert self.get_steering_command(CI.CP, can_sends)[0]

      with self.subTest(platform=platform, lat_active=False):
        CI = self.make_interface(platform)
        actuators, can_sends = CI.apply(self.make_control(False), 0)
        assert actuators.steeringRequestActiveValid
        assert not actuators.steeringRequestActive
        assert not self.get_steering_command(CI.CP, can_sends)[0]

  def test_high_angle_request_cut_preserves_torque_count(self):
    for platform in self.PLATFORMS:
      with self.subTest(platform=platform):
        CI = self.make_interface(platform)
        CC = self.make_control(True)

        for _ in range(CI.CC.params.STEER_MAX):
          CI.apply(CC, 0)
          if CI.CC.apply_torque_last == CI.CC.params.STEER_MAX:
            break
        assert CI.CC.apply_torque_last == CI.CC.params.STEER_MAX

        CI.CS.out.steeringAngleDeg = MAX_ANGLE + 1
        for _ in range(MAX_ANGLE_FRAMES):
          actuators, _ = CI.apply(CC, 0)
          assert actuators.steeringRequestActive

        first_cut, can_sends = CI.apply(CC, 0)
        assert first_cut.steeringRequestActiveValid
        assert not first_cut.steeringRequestActive
        request_active, torque_output_can = self.get_steering_command(CI.CP, can_sends)
        assert not request_active
        assert first_cut.torqueOutputCan != 0
        assert first_cut.torqueOutputCan == torque_output_can
        assert first_cut.steeringRequestFaultAvoidanceCounter == MAX_ANGLE_FRAMES + 1

        final_cut = first_cut
        for _ in range(MAX_ANGLE_CONSECUTIVE_FRAMES - 1):
          final_cut, can_sends = CI.apply(CC, 0)
          request_active, torque_output_can = self.get_steering_command(CI.CP, can_sends)
          assert not final_cut.steeringRequestActive
          assert not request_active
          assert final_cut.torqueOutputCan != 0
          assert final_cut.torqueOutputCan == torque_output_can
          assert final_cut.torqueOutputCan == first_cut.torqueOutputCan
        assert final_cut.steeringRequestFaultAvoidanceCounter == 0

        resumed, can_sends = CI.apply(CC, 0)
        assert resumed.steeringRequestActive
        assert self.get_steering_command(CI.CP, can_sends)[0]
        assert resumed.steeringRequestFaultAvoidanceCounter == 1


if __name__ == "__main__":
  unittest.main()
