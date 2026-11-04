import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.hyundai.carcontroller import (LAUNCH_JERK_SPEED, LAUNCH_JERK_UPPER, PID_JERK_UPPER,
                                               STOPPING_JERK_UPPER, acc_jerk_upper)
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaican import LeadDisplay
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import CAR, DATE_FW_ECUS, FW_QUERY_CONFIG, CANFD_FUZZY_WHITELIST, \
                                         PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         HyundaiFlags, get_platform_codes, HyundaiSafetyFlags
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_CEED_PHEV,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}

# CAN-only feature flags that should not appear on CAN FD platforms
CAN_FEATURE_FLAGS = (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS | HyundaiFlags.CHECKSUM_CRC8 |
                     HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL |
                     HyundaiFlags.CAMERA_SCC)


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


class TestHyundaiFingerprint(unittest.TestCase):
  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_can_features(self):
    for car_model in CAR:
      flags = car_model.config.flags
      # Test no EV/HEV with cluster/TCU gears (should all use ELECT_GEAR)
      if flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
        assert not (flags & (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS))

      # Test CAN FD car not in CAN feature lists
      if flags & HyundaiFlags.CANFD:
        assert not (flags & CAN_FEATURE_FLAGS), "CAN FD car unexpectedly has a CAN feature flag"

  def test_hybrid_ev_flags(self):
    for car_model in CAR:
      flags = car_model.config.flags
      assert not (flags & HyundaiFlags.HYBRID and flags & HyundaiFlags.EV), "Shared cars between hybrid and EV"
      assert not (flags & HyundaiFlags.CANFD and flags & HyundaiFlags.HYBRID), \
        "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CAR:
      if not (car_model.config.flags & HyundaiFlags.CANFD):
        continue
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = cars_with(HyundaiFlags.CANFD) | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= cars_with(HyundaiFlags.CANFD) - cars_with(HyundaiFlags.EV) - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms


class TestHyundaiLaunchJerk(unittest.TestCase):
  # the standstill-exit window the deleted `starting` LongCtrlState used to own. The bracket on the Palisade
  # (command -> wheel roll) was 1.0 = 1390 ms, 3.0 = 960, 5.0 = 790, 7.0 = 1200, so 5.0 is a measured optimum
  def test_the_launch_window_is_the_pid_state_while_the_wheels_are_still(self):
    from opendbc.car.structs import CarControl
    state = CarControl.Actuators.LongControlState
    assert acc_jerk_upper(state.pid, 0.0) == LAUNCH_JERK_UPPER
    assert acc_jerk_upper(state.pid, LAUNCH_JERK_SPEED - 1e-6) == LAUNCH_JERK_UPPER
    assert acc_jerk_upper(state.pid, LAUNCH_JERK_SPEED) == PID_JERK_UPPER
    assert acc_jerk_upper(state.pid, 20.0) == PID_JERK_UPPER

  def test_stopping_and_off_keep_the_gentle_limit_at_any_speed(self):
    from opendbc.car.structs import CarControl
    state = CarControl.Actuators.LongControlState
    for v in (0.0, 0.05, 1.0, 20.0):
      assert acc_jerk_upper(state.stopping, v) == STOPPING_JERK_UPPER
      assert acc_jerk_upper(state.off, v) == STOPPING_JERK_UPPER

  def test_every_value_encodes_exactly_on_scc14(self):
    # JerkUpperLimit is 7 bits at 0.1 m/s^3 (0..12.7); a value that does not land on the quantum would be silently
    # truncated on the bus, which is how a "5.0" could reach the car as something else
    for value in (LAUNCH_JERK_UPPER, PID_JERK_UPPER, STOPPING_JERK_UPPER):
      raw = round(value / 0.1)
      assert 0 <= raw <= 127
      assert abs(raw * 0.1 - value) < 1e-9


class TestHyundaiLeadDisplay(unittest.TestCase):
  def setUp(self):
    self.lead_display = LeadDisplay()

  def update(self, frames, dist, v_ego=20., target=None, bars=None):
    # None means not sent: the defaults, 0. (no target, time gap) and 0 (no bars, anchor 3)
    target = 0. if target is None else target
    bars = 0 if bars is None else bars
    return {self.lead_display.update(dist, v_ego, target, bars) for _ in range(frames)}

  def test_debounce(self):
    assert self.update(100, 45.) == {0}
    assert self.update(1, 0.) == {0}  # a dropout restarts the count
    assert self.update(149, 45.) == {0}  # shown after 1.5 s
    assert self.update(1, 45.) == {5}
    assert self.update(10, 0.) == {5}  # the last distance is held while the lead is missing
    assert self.update(1, 45.) == {5}  # and the hide count restarts
    assert self.update(49, float('nan')) == {5}
    assert self.update(1, 0.) == {0}  # hidden after 0.5 s

  def test_hysteresis(self):
    self.update(LeadDisplay.SHOW_FRAMES, 25.)
    for dist, obj_gap in [(31., 3), (37., 4), (45., 5), (39., 5), (37., 4), (33., 4), (31., 3)]:  # 1.3, 1.6, 2.0, 1.7, 1.6, 1.4, 1.3 s
      assert self.update(1, dist) == {obj_gap}, dist

  def test_reshow_at_stop(self):
    self.update(LeadDisplay.SHOW_FRAMES, 45.)
    self.update(LeadDisplay.HIDE_FRAMES, 0.)
    assert self.update(LeadDisplay.SHOW_FRAMES - 1, 6.6, 0.) == {0}
    assert self.update(1, 6.6, 0.) == {2}  # starts over from 2, a kept 5 would step down to 3
    assert self.update(1, 8., 0.) == {2}  # the time gap alone says 5 here, as stock would show; the floors keep it close
    assert self.update(1, 11., 0.) == {3}
    assert self.update(1, 15., 0.) == {5}
    assert self.update(1, 14.5, float('nan')) == {5}  # a NaN speed holds the level

  def test_distance_floors(self):
    # at a stop the time gap alone would put a car 8 m ahead at the farthest level; the floors keep it close
    self.update(LeadDisplay.SHOW_FRAMES, 6., 0.)
    for dist, obj_gap in [(8.9, 2), (9., 3), (11.9, 3), (12., 4), (14.9, 4), (15., 5), (14.1, 5), (13.9, 4), (11.1, 4), (10.9, 3), (8.1, 3), (7.9, 2)]:
      assert self.update(1, dist, 0.) == {obj_gap}, dist
    # the floors apply to the held distance, so a distance jump does not lift the cap before it is taken
    self.update(1, 7.9, 0.)
    assert self.update(LeadDisplay.SHOW_FRAMES - 1, 20., 0.) == {2}
    assert self.update(1, 20., 0.) == {5}

  def test_distance_floors_only_at_low_speed(self):
    class NoFloors(LeadDisplay):
      FLOORS = {3: 0., 4: 0., 5: 0.}

    def levels(v_ego, display):
      dists = [6. + 10. * abs(((i / 300.) % 2.) - 1.) for i in range(3000)]  # 16..6..16 m, a 6 s triangle
      return [display.update(d, v_ego) for d in dists]

    for v_ego in (6.1, 8., 12., 20., 30.):
      assert levels(v_ego, LeadDisplay()) == levels(v_ego, NoFloors()), v_ego  # unchanged from 6.1 m/s up
    assert levels(5.5, LeadDisplay()) != levels(5.5, NoFloors())  # under it level 5 arrives at 14.9 m, inside the floor
    assert levels(3., LeadDisplay()) != levels(3., NoFloors())
    assert max(levels(3., LeadDisplay())) == 5  # the floors cap, they do not hide a far lead

  def test_distance_jump(self):
    self.update(LeadDisplay.SHOW_FRAMES, 20.)
    assert self.update(1, 45.) == {2}  # a jump is a different object, the level holds
    assert self.update(1, 21.) == {2}  # until the lead is back
    assert self.update(LeadDisplay.SHOW_FRAMES - 1, 45.) == {2}  # and the count restarts
    assert self.update(1, 45.) == {5}  # taken after 1.5 s, like a new lead
    assert self.update(1, 37.) == {4}  # a small step is taken at once

  def test_target_ratio(self):
    # aggressive at 29 m/s holds 36 m: levels start at 0.8, 1.1 and 1.5 of it, steps under 5 m dodge the jump guard
    self.update(LeadDisplay.SHOW_FRAMES, 25., 29., 36.)
    for dist, obj_gap in [(28.7, 2), (28.8, 3), (33., 3), (37., 3), (39.5, 3), (39.6, 4), (44., 4), (48., 4), (52., 4), (53.9, 4),
                          (54., 5), (52.2, 5), (52.1, 4), (48., 4), (44., 4), (40., 4), (37.9, 4), (37.7, 3), (33., 3), (29., 3),
                          (27., 3), (26.9, 2)]:
      assert self.update(1, dist, 29., 36.) == {obj_gap}, dist
    # 4 is kept to 37.8 m (1.05 x 36) in exact terms; 37.8 / 36 rounds to just under 1.05, so 37.9 and 37.7 bracket it

  def test_target_ratio_relaxed(self):
    # relaxed at 29 m/s holds 58 m, the same ratios apply; the exact boundaries are float ties, so each is bracketed by 0.1 m
    self.update(LeadDisplay.SHOW_FRAMES, 40., 29., 58.)
    for dist, obj_gap in [(46.3, 2), (46.5, 3), (52., 3), (58., 3), (63.7, 3), (63.9, 4), (69., 4), (74., 4), (79., 4), (84., 4),
                          (86.9, 4), (87., 5), (84.1, 5), (84., 4), (79., 4), (74., 4), (69., 4), (64., 4), (61., 4), (60.8, 3),
                          (56., 3), (51., 3), (46., 3), (43.5, 3), (43.4, 2)]:
      assert self.update(1, dist, 29., 58.) == {obj_gap}, dist

  def test_target_reads_gap_not_personality(self):
    # at its own target each personality reads 3; the time gap alone reads aggressive's 36 m as 2 and relaxed's 58 m as 5
    for target, time_gap_level in [(36., 2), (58., 5)]:
      self.lead_display = LeadDisplay()
      assert self.update(LeadDisplay.SHOW_FRAMES, target, 29., target) == {0, 3}, target
      self.lead_display = LeadDisplay()
      assert self.update(LeadDisplay.SHOW_FRAMES, target, 29.) == {0, time_gap_level}, target

  def test_target_at_stop(self):
    # an engaged stop sits about 5.9 m behind a 7 m target, ratio 0.84; the 9 m floor keeps it at 2
    self.update(LeadDisplay.SHOW_FRAMES - 1, 5.9, 0., 7.)
    assert self.update(1, 5.9, 0., 7.) == {2}
    assert self.update(1, 10., 0., 7.) == {3}  # ratio 1.43 says 4, the floor 3
    assert self.update(1, 12., 0., 7.) == {4}  # ratio 1.71 says 5, the floor 4

  def test_target_nan_holds(self):
    self.update(LeadDisplay.SHOW_FRAMES, 45., 29., 36.)
    assert self.update(1, 45., 29., 36.) == {4}  # 1.25
    for dist in (41., 37., 33., 29.):  # the ratio would step to 3 by 29 m and the time gap to 2
      assert self.update(1, dist, 29., float('nan')) == {4}, dist
    for dist in (33., 37., 41.):  # an infinite target holds too, it would read every distance as 2
      assert self.update(1, dist, 29., float('inf')) == {4}, dist
    assert self.update(1, 41., 29., 36.) == {4}  # a target again is taken at once, 1.14
    assert self.update(1, 37., 29., 36.) == {3}  # 1.03

  def test_target_zero_is_time_gap(self):
    dists = [6. + 60. * abs(((i / 300.) % 2.) - 1.) for i in range(3000)]  # 66..6..66 m, a 6 s triangle
    for v_ego in (0., 3., 12., 29.):
      no_target, zero, negative = LeadDisplay(), LeadDisplay(), LeadDisplay()
      levels = [no_target.update(d, v_ego) for d in dists]
      assert levels == [zero.update(d, v_ego, 0.) for d in dists], v_ego
      assert levels == [negative.update(d, v_ego, -1.) for d in dists], v_ego

  def test_target_mode_switch(self):
    # the target may come and go between frames: the time-gap level and the ratio band are kept apart, each frame
    # steps the one its rule uses
    self.update(LeadDisplay.SHOW_FRAMES, 40., 29., 36.)
    for dist, target, obj_gap in [(40., 36., 4),  # 1.11
                                  (40., 0., 3),  # 1.21 s
                                  (40., 36., 4),
                                  (36., 0., 3),  # 1.07 s
                                  (36., 36., 3),  # 1.0
                                  (32., 0., 2),  # 0.93 s
                                  (32., 36., 3),  # 0.89
                                  (28., 36., 3),  # 0.78, kept from 3
                                  (28., 0., 2),  # 0.79 s
                                  (28., 36., 3)]:  # 0.78, the band is still kept from 3
      assert self.update(1, dist, 29., target) == {obj_gap}, (dist, target)
    for dist, target, bars in [(28., 36., 1), (28., 0., 3), (28., 36., 3), (40., float('nan'), 1), (40., 0., 1), (40., 36., 3)]:
      assert self.update(1, dist, 29., target, bars) <= {2, 3, 4, 5}, (dist, target, bars)

  def test_target_anchor_aggressive(self):
    # 1 bar puts the icon on 2 at the target, then one mark per band: 3 from 1.1, 4 from 1.5 and 5 from 2.0 of 36 m
    self.update(LeadDisplay.SHOW_FRAMES, 30., 29., 36., 1)
    for dist, obj_gap in [(30., 2), (33., 2), (36., 2), (39.5, 2), (39.6, 3), (44., 3), (49., 3), (53.9, 3), (54., 4), (58., 4),
                          (62., 4), (66., 4), (70., 4), (71.9, 4), (72., 5), (70.2, 5), (70.1, 4), (66., 4), (62., 4), (58., 4),
                          (54., 4), (52.2, 4), (52.1, 3), (48., 3), (44., 3), (40., 3), (37.9, 3), (37.7, 2), (33., 2), (29., 2)]:
      assert self.update(1, dist, 29., 36., 1) == {obj_gap}, dist
    # 37.8 / 36 rounds to just under 1.05, so 37.9 and 37.7 bracket it

  def test_target_anchor_standard(self):
    self.update(LeadDisplay.SHOW_FRAMES, 25., 29., 36., 2)
    for dist, obj_gap in [(28.7, 2), (28.8, 3), (33., 3), (36., 3), (39.5, 3), (39.6, 4), (44., 4), (49., 4), (53.9, 4), (54., 5)]:
      assert self.update(1, dist, 29., 36., 2) == {obj_gap}, dist

  def test_target_anchor_relaxed(self):
    # 3 bars put the icon on 4 at the target (from 0.8 of 58 m), 5 from 1.1, 3 under 0.8 and 2 under 0.6
    self.update(LeadDisplay.SHOW_FRAMES, 30., 29., 58., 3)
    for dist, obj_gap in [(30., 2), (34.7, 2), (34.8, 3), (39., 3), (44., 3), (46.4, 3), (46.5, 4), (51., 4), (56., 4), (58., 4),
                          (63.8, 4), (63.9, 5), (68., 5), (63.9, 5), (61., 5), (60.8, 4), (56., 4), (51., 4), (46., 4), (43.6, 4), (43.4, 3),
                          (39., 3), (35., 3), (32., 3), (31.8, 2)]:
      assert self.update(1, dist, 29., 58., 3) == {obj_gap}, dist
    # 5 is kept to 60.9 m and 4 to 43.5 m (1.05 and 0.75 x 58) in exact terms; float ties, so each is bracketed

  def test_target_anchor_bars_clamp(self):
    # no bars (0, or a caller that sends none) is anchor 3, as 2 bars; bars past 3 clamp to anchor 4
    dists = [20. + 60. * abs(((i / 300.) % 2.) - 1.) for i in range(3000)]  # 80..20..80 m, a 6 s triangle

    def levels(target, bars):
      display = LeadDisplay()
      if bars is None:
        return [display.update(d, 29., target) for d in dists]
      return [display.update(d, 29., target, bars) for d in dists]

    for target in (36., 58.):
      assert levels(target, None) == levels(target, 0) == levels(target, 2) == levels(target, -1), target
      assert levels(target, 4) == levels(target, 3) != levels(target, 2), target
      assert levels(target, 1) != levels(target, 2), target

  def test_target_anchor_personality_change(self):
    # at a fixed 36 m against a 36 m target, a personality change moves the icon by one mark in the same frame
    self.update(LeadDisplay.SHOW_FRAMES, 36., 29., 36., 1)
    for bars, obj_gap in [(1, 2), (2, 3), (3, 4), (2, 3), (1, 2)]:
      assert self.update(1, 36., 29., 36., bars) == {obj_gap}, bars

  def test_target_anchor_floors(self):
    # the floors still cap: an engaged stop 5.9 m behind a 7 m target reads 2, and 3 bars do not lift a close car
    self.update(LeadDisplay.SHOW_FRAMES - 1, 5.9, 0., 7., 1)
    assert self.update(1, 5.9, 0., 7., 1) == {2}
    assert self.update(1, 10., 0., 7., 1) == {3}  # ratio 1.43, band 1, the floor 3
    assert self.update(1, 12., 0., 7., 1) == {4}  # ratio 1.71, band 2 says 4, the floor 4
    assert self.update(1, 12., 0., 7., 3) == {4}  # anchor 4 says 5, the floor 4
    assert self.update(1, 10., 0., 7., 3) == {3}  # band 1 says 5, the floor 3

  def test_target_distance_jump(self):
    self.update(LeadDisplay.SHOW_FRAMES, 30., 29., 36.)
    assert self.update(1, 30., 29., 36.) == {3}  # 0.83
    assert self.update(LeadDisplay.SHOW_FRAMES - 1, 60., 29., 36.) == {3}  # twice the distance is a different object, the level holds
    assert self.update(1, 60., 29., 36.) == {5}  # taken after 1.5 s, 1.67
