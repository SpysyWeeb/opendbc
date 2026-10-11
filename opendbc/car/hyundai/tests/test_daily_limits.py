"""Daily's default classic envelope; every narrower upstream exception stays intact."""
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from opendbc.car.interfaces import ACCEL_MAX, CarInterfaceBase


def envelope(candidate, flags=0, firmware=()):
  cp = SimpleNamespace(carFingerprint=candidate, flags=flags, carFw=firmware)
  p = CarControllerParams(cp)
  return p.STEER_MAX, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN


@pytest.mark.parametrize("candidate", [CAR.HYUNDAI_PALISADE, CAR.HYUNDAI_SONATA, "unknown"])
@pytest.mark.parametrize("firmware", [(), (b"LX",), (b"ON",), (b"LX", b"ON")])
def test_default_classic_envelope(candidate, firmware):
  # Palisade/Telluride share a platform. No firmware selector is requested.
  assert envelope(candidate, firmware=firmware) == (409, 4, 7)


@pytest.mark.parametrize("candidate", [CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA, CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.HYUNDAI_IONIQ, CAR.HYUNDAI_IONIQ_EV_LTD, CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.HYUNDAI_SONATA_LF,
  CAR.KIA_FORTE, CAR.KIA_NIRO_PHEV, CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL, CAR.KIA_SORENTO])
def test_255_exceptions_unchanged(candidate):
  assert envelope(candidate) == (255, 3, 7)


@pytest.mark.parametrize("flag,expected", [(HyundaiFlags.CANFD, (270, 2, 3)),
  (HyundaiFlags.ALT_LIMITS, (270, 2, 3)), (HyundaiFlags.ALT_LIMITS_2, (170, 2, 3))])
def test_narrower_limits_unchanged(flag, expected):
  assert envelope(CAR.HYUNDAI_PALISADE, flag) == expected


def test_bsm_does_not_select_an_envelope():
  for candidate in CAR:
    assert envelope(candidate, HyundaiFlags.HAS_BSM) == envelope(candidate)


def test_accel_ceiling():
  assert ACCEL_MAX == CarControllerParams.ACCEL_MAX == 4.0
  assert CarInterfaceBase.get_pid_accel_limits(None, 0, 30) == (-3.5, 4.0)
  assert CarInterface.get_pid_accel_limits(None, 0, 30) == (-3.5, 4.0)


@pytest.mark.parametrize("candidate", [CAR.HYUNDAI_SONATA, CAR.KIA_EV6])
@pytest.mark.parametrize("requested,expected", [(-3.51, -3.5), (-3.5, -3.5), (0.0, 0.0), (2.01, 2.01), (4.0, 4.0), (4.01, 4.0)])
def test_accel_controller_and_can_ceiling(candidate, requested, expected):
  cp = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long=True, is_release=False, docs=False)
  assert cp.openpilotLongitudinalControl
  interface = CarInterface(cp)
  interface.update([])
  canfd = bool(cp.flags & HyundaiFlags.CANFD)
  message = "SCC_CONTROL" if canfd else "SCC12"
  parser = CANParser("hyundai_canfd_generated" if canfd else "hyundai_can_generated", [(message, 0)],
                     interface.CC.CAN.ECAN if canfd else 0)
  cc = structs.CarControl()
  cc.enabled = cc.longActive = True
  cc.actuators.accel = requested
  cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
  # Exercise the real controller and CAN-FD slew, not just the class constant.
  for frame in range(100):
    applied, messages = interface.apply(cc.as_reader(), frame * 10_000_000)
    assert applied.accel == pytest.approx(expected)
    parser.update([(frame * 10_000_000, messages)])
  assert parser.vl[message]["aReqRaw"] == pytest.approx(expected)
  assert parser.vl[message]["aReqValue"] == pytest.approx(expected)
