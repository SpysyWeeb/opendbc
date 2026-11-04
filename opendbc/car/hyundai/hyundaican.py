import math
from opendbc.car import DT_CTRL
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import CAR, HyundaiFlags

hyundai_checksum = mk_crc8_fun(CRC8J1850, init_crc=0xFD, xor_out=0xDF)


def create_lkas11(packer, frame, CP, apply_torque, steer_req,
                  torque_fault, lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart):
  values = {s: lkas11[s] for s in [
    "CF_Lkas_LdwsActivemode",
    "CF_Lkas_LdwsSysState",
    "CF_Lkas_SysWarning",
    "CF_Lkas_LdwsLHWarning",
    "CF_Lkas_LdwsRHWarning",
    "CF_Lkas_HbaLamp",
    "CF_Lkas_FcwBasReq",
    "CF_Lkas_HbaSysState",
    "CF_Lkas_FcwOpt",
    "CF_Lkas_HbaOpt",
    "CF_Lkas_FcwSysState",
    "CF_Lkas_FcwCollisionWarning",
    "CF_Lkas_FusionState",
    "CF_Lkas_FcwOpt_USM",
    "CF_Lkas_LdwsOpt_USM",
  ]}
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_torque
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = torque_fault  # seems to allow actuation on CR_Lkas_StrToqReq
  values["CF_Lkas_MsgCount"] = frame % 0x10

  if CP.carFingerprint in (CAR.HYUNDAI_SONATA, CAR.HYUNDAI_PALISADE, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021, CAR.KIA_NIRO_PHEV_2022, CAR.HYUNDAI_SANTA_FE,
                           CAR.HYUNDAI_IONIQ_EV_2020, CAR.HYUNDAI_IONIQ_PHEV, CAR.KIA_SELTOS, CAR.HYUNDAI_ELANTRA_2021, CAR.GENESIS_G70_2020,
                           CAR.HYUNDAI_ELANTRA_HEV_2021, CAR.HYUNDAI_SONATA_HYBRID, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_KONA_HEV, CAR.HYUNDAI_KONA_EV_2022,
                           CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_IONIQ_HEV_2022, CAR.HYUNDAI_SANTA_FE_HEV_2022,
                           CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.KIA_STINGER_2022, CAR.KIA_K5_HEV_2020, CAR.KIA_CEED, CAR.KIA_CEED_PHEV,
                           CAR.HYUNDAI_AZERA_6TH_GEN, CAR.HYUNDAI_AZERA_HEV_6TH_GEN, CAR.HYUNDAI_CUSTIN_1ST_GEN, CAR.HYUNDAI_KONA_2022):
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  # Likely cars lacking the ability to show individual lane lines in the dash
  elif CP.carFingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL):
    # SysWarning 4 = keep hands on wheel + beep
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

    # SysState 0 = no icons
    # SysState 1-2 = white car + lanes
    # SysState 3 = green car + lanes, green steering wheel
    # SysState 4 = green car + lanes
    values["CF_Lkas_LdwsSysState"] = 3 if enabled else 1
    values["CF_Lkas_LdwsOpt_USM"] = 2  # non-2 changes above SysState definition

    # these have no effect
    values["CF_Lkas_LdwsActivemode"] = 0
    values["CF_Lkas_FcwOpt_USM"] = 0

  elif CP.carFingerprint == CAR.HYUNDAI_GENESIS:
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2

  dat = packer.make_can_msg("LKAS11", 0, values)[1]

  if CP.flags & HyundaiFlags.CHECKSUM_CRC8:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif CP.flags & HyundaiFlags.CHECKSUM_6B:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", 0, values)


def create_clu11(packer, frame, clu11, button, CP):
  values = {s: clu11[s] for s in [
    "CF_Clu_CruiseSwState",
    "CF_Clu_CruiseSwMain",
    "CF_Clu_SldMainSW",
    "CF_Clu_ParityBit1",
    "CF_Clu_VanzDecimal",
    "CF_Clu_Vanz",
    "CF_Clu_SPEED_UNIT",
    "CF_Clu_DetentOut",
    "CF_Clu_RheostatLevel",
    "CF_Clu_CluInfo",
    "CF_Clu_AmpInfo",
    "CF_Clu_AliveCnt1",
  ]}
  values["CF_Clu_CruiseSwState"] = button
  values["CF_Clu_AliveCnt1"] = frame % 0x10
  # send buttons to camera on camera-scc based cars
  bus = 2 if CP.flags & HyundaiFlags.CAMERA_SCC else 0
  return packer.make_can_msg("CLU11", bus, values)


def create_lfahda_mfc(packer, enabled):
  values = {
    "LFA_Icon_State": 2 if enabled else 0,
  }
  return packer.make_can_msg("LFAHDA_MFC", 0, values)


class LeadDisplay:
  # Mirrors stock SCC's cluster lead: a lead is shown after 1.5 s of continuous presence and dropped 0.5 s after it
  # is lost, and its gap level has 0.15 s of hysteresis, as stock's radar does. The input is still openpilot's raw
  # lead, only its presentation follows stock. Constants are fit to stock cruise logs on a 2021 Palisade.
  # When openpilot sends the distance it holds at this speed, the level reads the held distance against it instead.
  # At the target the icon sits on the mark just past the gap bars: 2 for 1 bar, 3 for 2 and 4 for 3 (3 when no bars
  # are sent). It moves one mark per band of held distance over target: closer from 0.8 and again from 0.6, farther
  # from 1.1, 1.5 and 2.0, each band kept until 0.05 under its start, and the level stays within 2 to 5. Without a
  # target the time gap below is used
  SHOW_FRAMES = round(1.5 / DT_CTRL)
  HIDE_FRAMES = round(0.5 / DT_CTRL)
  GAP_OFFSET = 5.  # m, fit offset from openpilot's lead distance to stock's gap
  MIN_SPEED = 1.5  # m/s, so a lead still gets a gap level at a stop
  GAPS = {3: 1.2, 4: 1.5, 5: 1.8}  # s, time gap at which each ObjGap level starts
  GAP_HYSTERESIS = 0.15  # s
  JUMP = 5.  # m
  JUMP_RATIO = 0.3
  # Stock places a stopped car at the farthest level from about 7 m, and so does the time gap above. The owner would
  # rather see a close car as close, so a level also needs the held distance to reach its floor: 9 m for level 3,
  # 12 m for 4 and 15 m for 5, each kept until the distance is 1 m under its floor. From 6.1 m/s (22 km/h) up the
  # time-gap thresholds are past the floors and the display is unchanged
  FLOORS = {3: 9., 4: 12., 5: 15.}  # m, held distance at which each level can start
  FLOOR_HYSTERESIS = 1.  # m
  # held distance over target distance at which each band starts; band 0 is at the target, -2 is under 0.6
  BANDS = {-1: 0.6, 0: 0.8, 1: 1.1, 2: 1.5, 3: 2.0}
  RATIO_HYSTERESIS = 0.05

  def __init__(self):
    self.shown = False
    self.frames = 0
    self.jump_frames = 0
    self.distance = 0.
    self.level = 2
    self.band = -2  # ratio band against the target, kept apart from the time-gap level
    self.held = 2
    self.cap = 2

  def update(self, lead_distance, v_ego, target_distance=0., lead_distance_bars=0):
    present = lead_distance > 0  # a NaN distance counts as no lead, a NaN speed or a NaN or infinite target holds the level
    self.frames = self.frames + 1 if present != self.shown else 0
    if self.frames >= (self.HIDE_FRAMES if self.shown else self.SHOW_FRAMES):
      self.shown = not self.shown
      self.frames = 0
      self.jump_frames = 0
      self.distance = lead_distance
    if not self.shown:
      self.level = 2
      self.band = -2
      self.held = 2
      self.cap = 2
      return 0

    if present:
      # a distance jump is a different object: hold the level until it has been seen for SHOW_FRAMES, like a new lead
      jump = abs(lead_distance - self.distance) > max(self.JUMP, self.JUMP_RATIO * self.distance)
      self.jump_frames = self.jump_frames + 1 if jump else 0
      if not jump or self.jump_frames >= self.SHOW_FRAMES:
        self.distance = lead_distance
        self.jump_frames = 0
    if 0 < target_distance and math.isfinite(target_distance):
      ratio = self.distance / target_distance
      while self.band < 3 and ratio >= self.BANDS[self.band + 1]:
        self.band += 1
      while self.band > -2 and ratio < self.BANDS[self.band] - self.RATIO_HYSTERESIS:
        self.band -= 1
      anchor = min(max(lead_distance_bars + 1, 2), 4) if lead_distance_bars > 0 else 3
      level = min(max(anchor + self.band, 2), 5)
    elif target_distance <= 0:  # a NaN or infinite target is neither and holds the level
      self._step((self.distance - self.GAP_OFFSET) / max(v_ego, self.MIN_SPEED), self.GAPS, self.GAP_HYSTERESIS)
      level = self.level
    else:
      level = self.held  # a NaN or infinite target holds the last level of either rule
    while self.cap < 5 and self.distance >= self.FLOORS[self.cap + 1]:
      self.cap += 1
    while self.cap > 2 and self.distance < self.FLOORS[self.cap] - self.FLOOR_HYSTERESIS:
      self.cap -= 1
    self.held = level
    return min(level, self.cap)

  def _step(self, value, starts, hysteresis):
    while self.level < 5 and value >= starts[self.level + 1]:
      self.level += 1
    while self.level > 2 and value < starts[self.level] - hysteresis:
      self.level -= 1


def create_acc_commands(packer, enabled, accel, upper_jerk, idx, hud_control, obj_gap, set_speed, stopping, long_override, use_fca, CP):
  commands = []

  scc11_values = {
    "MainMode_ACC": 1,
    "TauGapSet": hud_control.leadDistanceBars,
    "VSetDis": set_speed if enabled else 0,
    "AliveCounterACC": idx % 0x10,
    "ObjValid": 1, # close lead makes controls tighter
    "ACC_ObjStatus": 1, # close lead makes controls tighter
    "ACC_ObjLatPos": 0,
    "ACC_ObjRelSpd": 0,
    "ACC_ObjDist": 1, # close lead makes controls tighter
    }
  commands.append(packer.make_can_msg("SCC11", 0, scc11_values))

  scc12_values = {
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 0,
    "StopReq": 1 if stopping else 0,
    "aReqRaw": accel,
    "aReqValue": accel,  # stock ramps up and down respecting jerk limit until it reaches aReqRaw
    "CR_VSM_Alive": idx % 0xF,
  }

  # show AEB disabled indicator on dash with SCC12 if not sending FCA messages.
  # these signals also prevent a TCS fault on non-FCA cars with alpha longitudinal
  if not use_fca:
    scc12_values["CF_VSM_ConfMode"] = 1
    scc12_values["AEB_Status"] = 1  # AEB disabled

  scc12_dat = packer.make_can_msg("SCC12", 0, scc12_values)[1]
  scc12_values["CR_VSM_ChkSum"] = 0x10 - sum(sum(divmod(i, 16)) for i in scc12_dat) % 0x10

  commands.append(packer.make_can_msg("SCC12", 0, scc12_values))

  scc14_values = {
    "ComfortBandUpper": 0.0, # stock usually is 0 but sometimes uses higher values
    "ComfortBandLower": 0.0, # stock usually is 0 but sometimes uses higher values
    "JerkUpperLimit": upper_jerk, # stock usually is 1.0 but sometimes uses higher values
    "JerkLowerLimit": 5.0, # stock usually is 0.5 but sometimes uses higher values
    "ACCMode": 2 if enabled and long_override else 1 if enabled else 4, # stock will always be 4 instead of 0 after first disengage
    "ObjGap": obj_gap, # 5: farthest, 2: closest, 0: no lead
  }
  commands.append(packer.make_can_msg("SCC14", 0, scc14_values))

  # Only send FCA11 on cars where it exists on the bus
  # On Camera SCC cars, FCA11 is not disabled, so we forward stock FCA11 back to the car forward hooks
  if use_fca and not (CP.flags & HyundaiFlags.CAMERA_SCC):
    # note that some vehicles most likely have an alternate checksum/counter definition
    # https://github.com/commaai/opendbc/commit/9ddcdb22c4929baf310295e832668e6e7fcfa602
    fca11_values = {
      "CR_FCA_Alive": idx % 0xF,
      "PAINT1_Status": 1,
      "FCA_DrvSetStatus": 1,
      "FCA_Status": 1,  # AEB disabled
    }
    fca11_dat = packer.make_can_msg("FCA11", 0, fca11_values)[1]
    fca11_values["CR_FCA_ChkSum"] = hyundai_checksum(fca11_dat[:7])
    commands.append(packer.make_can_msg("FCA11", 0, fca11_values))

  return commands


def create_acc_opt(packer, CP):
  commands = []

  scc13_values = {
    "SCCDrvModeRValue": 2,
    "SCC_Equip": 1,
    "Lead_Veh_Dep_Alert_USM": 2,
  }
  commands.append(packer.make_can_msg("SCC13", 0, scc13_values))

  # TODO: this needs to be detected and conditionally sent on unsupported long cars
  # On Camera SCC cars, FCA12 is not disabled, so we forward stock FCA12 back to the car forward hooks
  if not (CP.flags & HyundaiFlags.CAMERA_SCC):
    fca12_values = {
      "FCA_DrvSetState": 2,
      "FCA_USM": 1, # AEB disabled
    }
    commands.append(packer.make_can_msg("FCA12", 0, fca12_values))

  return commands


def create_frt_radar_opt(packer):
  frt_radar11_values = {
    "CF_FCA_Equip_Front_Radar": 1,
  }
  return packer.make_can_msg("FRT_RADAR11", 0, frt_radar11_values)
