from collections import defaultdict
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.values import CAR, HyundaiFlags


@pytest.mark.parametrize("canfd", [False, True])
@pytest.mark.parametrize("lat_active", [False, True])
def test_lateral_only_hud_without_longitudinal_permission(canfd, lat_active):
  cp = structs.CarParams()
  cp.carFingerprint = CAR.HYUNDAI_PALISADE
  cp.flags = int(HyundaiFlags.CANFD if canfd else HyundaiFlags.SEND_LFA)
  cp.openpilotLongitudinalControl = True
  cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
  dbc = "hyundai_canfd_generated" if canfd else "hyundai_can_generated"
  controller = CarController({Bus.pt: dbc}, cp)
  cc = structs.CarControl()
  cc.enabled = False
  cc.latActive = lat_active
  cc.hudControl.leftLaneVisible = True
  cc.hudControl.rightLaneVisible = True
  cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.starting
  cs = SimpleNamespace(lkas11=defaultdict(int))
  if canfd:
    msgs = controller.create_canfd_msgs(lat_active, 0, 0, 0, False, cc.hudControl, cs, cc)
    names = ["LFA", "SCC_CONTROL"]
  else:
    msgs = controller.create_can_msgs(lat_active, 0, False, 0, 0, False, cc.hudControl, cc.actuators, cs, cc)
    names = ["LKAS11", "LFAHDA_MFC", "SCC12", "SCC14"]
  parser = CANParser(dbc, [(name, 0) for name in names], 0)
  parser.update([(0, msgs)])
  if canfd:
    assert parser.vl["LFA"]["LKA_SysIndReq"] == (2 if lat_active else 1)
    assert parser.vl["SCC_CONTROL"]["ACCMode"] == 0
  else:
    assert parser.vl["LKAS11"]["CF_Lkas_LdwsSysState"] == (3 if lat_active else 4)
    assert parser.vl["LKAS11"]["CF_Lkas_FcwOpt_USM"] == (2 if lat_active else 1)
    assert parser.vl["LFAHDA_MFC"]["LFA_Icon_State"] == (2 if lat_active else 0)
    assert parser.vl["SCC12"]["ACCMode"] == 0
    assert parser.vl["SCC14"]["JerkUpperLimit"] == 1.0
