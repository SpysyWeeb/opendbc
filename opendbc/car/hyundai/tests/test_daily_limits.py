"""Daily's default classic envelope; every narrower upstream exception stays intact."""
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags


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
  assert CarControllerParams.ACCEL_MAX == 2.0
