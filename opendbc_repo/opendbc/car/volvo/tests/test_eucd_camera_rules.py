import math
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CarControllerParams, LongMode
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.volvocan import eucd_next_long_mode

CCP = CarControllerParams
HARD = CCP.EUCD_CAMERA_BRAKE_HONOR_BELOW - 0.5


def controller(cam_brake=False):
  cc = CarController.__new__(CarController)
  cc.eucd_cam_brake = cam_brake
  cc.eucd_cam_brake_honor = False
  return cc


def test_hard_camera_brake_followed_at_once_and_latched():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.1, HARD) == HARD
  assert cc._eucd_camera_brake(0.1, -1.0) == -1.0  # latched while the camera brakes
  cc.eucd_cam_brake = False
  assert cc._eucd_camera_brake(0.1, -1.0) == 0.1   # released with the camera's brake


def test_mild_camera_brake_ignored():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.1, -1.0) == 0.1
  assert not cc.eucd_cam_brake_honor


def test_op_braking_more_is_never_limited():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(HARD - 0.5, HARD) == HARD - 0.5


def fsm3(mode):
  return SimpleNamespace(eucd_stock_FSM3={"ACC_Mode": mode})


@pytest.mark.parametrize("op_mode", [LongMode.BRAKE, LongMode.COAST, LongMode.DRIVE])
def test_fsm3_mirrors_camera_mode_0(op_mode):
  assert CarController._eucd_fsm3_mode(fsm3(LongMode.BRAKE), op_mode) == LongMode.BRAKE


@pytest.mark.parametrize("cam_mode", [LongMode.COAST, LongMode.DRIVE])
@pytest.mark.parametrize("op_mode", [LongMode.BRAKE, LongMode.COAST, LongMode.DRIVE])
def test_fsm3_keeps_op_mode_otherwise(cam_mode, op_mode):
  assert CarController._eucd_fsm3_mode(fsm3(cam_mode), op_mode) == op_mode


def test_traction_continuous_at_drive_switch():
  # Coast and drive traction meet at the DRIVE threshold, like the camera
  a = CCP.EUCD_DRIVE_ENTER
  coast = volvocan.eucd_fsm3_traction(a, LongMode.COAST) - 256
  drive = volvocan.eucd_fsm3_traction(a, LongMode.DRIVE)
  assert drive - coast <= 6


def test_fsm3_untouched_without_op_mode():
  assert CarController._eucd_fsm3_mode(fsm3(LongMode.BRAKE), None) is None


def offset(pitch_deg):
  return CarController._eucd_slope_brake_offset(SimpleNamespace(orientationNED=[0., math.radians(pitch_deg), 0.]))


def test_uphill_coast_is_not_braking():
  # Coast on a 5.4 deg climb is not braking
  assert eucd_next_long_mode(LongMode.COAST, -0.84) == LongMode.BRAKE
  assert eucd_next_long_mode(LongMode.COAST, -0.84, offset(5.4)) == LongMode.COAST
  assert eucd_next_long_mode(LongMode.COAST, -1.5, offset(5.4)) == LongMode.BRAKE


def test_uphill_coast_overshoot_is_not_braking():
  # Coast-cap overshoot on a 5 deg climb
  assert eucd_next_long_mode(LongMode.COAST, -1.15, offset(5.0)) == LongMode.COAST
  assert eucd_next_long_mode(LongMode.COAST, -1.6, offset(5.0)) == LongMode.BRAKE
  # Holds down to the usual exit
  assert eucd_next_long_mode(LongMode.BRAKE, -0.8, offset(5.0)) == LongMode.BRAKE


def test_downhill_brakes_earlier():
  # Downhill brakes earlier
  assert eucd_next_long_mode(LongMode.COAST, -0.4) == LongMode.COAST
  assert eucd_next_long_mode(LongMode.COAST, -0.4, offset(-1.5)) == LongMode.BRAKE
  assert offset(-10.) == CCP.EUCD_DOWNHILL_BRAKE_OFFSET_MAX
  # BRAKE exit stays at or below 0
  assert eucd_next_long_mode(LongMode.BRAKE, 0.01, offset(-10.)) == LongMode.COAST


def test_flat_unchanged():
  assert offset(0.) == 0.
  assert CarController._eucd_slope_brake_offset(SimpleNamespace(orientationNED=[])) == 0.


def downhill_after(pitches_deg, start=False):
  """Feeds one pitch per frame, returns the downhill state after each."""
  cc = CarController.__new__(CarController)
  cc.eucd_downhill, cc.eucd_downhill_frames = start, 0
  CS = SimpleNamespace()
  out = []
  for p in pitches_deg:
    cc._eucd_update_downhill(SimpleNamespace(orientationNED=[0., math.radians(p), 0.]), CS)
    out.append(CS.eucd_downhill)
  return out


def test_downhill_needs_2s_below_minus_2_deg():
  n = int(2.0 / DT_CTRL)
  assert not any(downhill_after([-3.] * (n - 1)))
  assert downhill_after([-3.] * n)[-1]
  # a short dip (hard braking) does not count
  assert not any(downhill_after(([-3.] * (n // 2) + [0.]) * 3))
  assert not any(downhill_after([-1.5] * (3 * n)))  # between the thresholds


def test_downhill_ends_after_1s_above_minus_1_deg():
  n = int(1.0 / DT_CTRL)
  assert all(downhill_after([-1.5] * (5 * n), start=True))  # still downhill between the thresholds
  states = downhill_after([0.] * n, start=True)
  assert all(states[:-1]) and not states[-1]


def test_no_pitch_is_flat():
  cc = CarController.__new__(CarController)
  cc.eucd_downhill, cc.eucd_downhill_frames = False, 0
  CS = SimpleNamespace()
  for _ in range(500):
    cc._eucd_update_downhill(SimpleNamespace(orientationNED=[]), CS)
  assert not CS.eucd_downhill


def limited(accel, last):
  cc = CarController.__new__(CarController)
  cc.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
  return cc._eucd_jerk_limit(accel, last)


def test_jerk_limit_ramps():
  dt = 0.02
  assert abs(limited(1.3, 0.0) - CCP.EUCD_JERK_UP * dt) < 1e-9             # rising
  assert abs(limited(-0.4, 1.3) - (1.3 - CCP.EUCD_JERK_DOWN_LIFT * dt)) < 1e-9  # lifting off
  assert abs(limited(-3.5, -1.0) - (-1.0 - CCP.EUCD_JERK_DOWN_BRAKE * dt)) < 1e-9  # braking deeper
  assert limited(0.31, 0.3) == 0.31                                        # small steps pass


def test_jerk_limit_reaches_target():
  a = 1.3
  for _ in range(100):
    a = limited(-0.4, a)
  assert a == -0.4


def cam_target(dist, v=24.):
  return SimpleNamespace(eucd_acc_distance=dist, out=SimpleNamespace(vEgo=v))


def test_accel_capped_by_camera_with_target():
  # Motorcycle case: OP wants +2, camera holds back
  assert CarController._eucd_camera_accel_cap(cam_target(22), 2.0, -0.5) == CCP.EUCD_CAMERA_ACCEL_CAP_FLOOR
  assert CarController._eucd_camera_accel_cap(cam_target(22), 2.0, 0.6) == 0.6


def test_accel_cap_never_adds_braking():
  assert CarController._eucd_camera_accel_cap(cam_target(22), -1.5, -0.5) == -1.5
  assert CarController._eucd_camera_accel_cap(cam_target(22), -0.2, -2.0) == -0.2


def test_accel_cap_off_without_close_target():
  assert CarController._eucd_camera_accel_cap(cam_target(255), 2.0, -0.5) == 2.0
  # Same far lead for OP and camera: OP keeps its accel
  assert CarController._eucd_camera_accel_cap(cam_target(70), 2.0, 1.0) == 2.0
  assert CarController._eucd_camera_accel_cap(cam_target(9, v=0.), 1.0, 0.2) == 0.2


def test_fsm3_template_never_keeps_the_camera_ack():
  """The camera's ACC_Check (engaging at standstill) left in the template made the ECM brake in the drive-off (8a3)."""
  from opendbc.car.volvo.eucd_full_stop import EucdFullStop
  fs = EucdFullStop()
  cam_fsm3 = {"ACC_NoTargetCancel": 0, "Byte_6": 0, "FrameKind": 0x81, "ACC_Check": 1}
  CS = SimpleNamespace(eucd_ecm_acc_engaged=True, eucd_stock_FSM3=cam_fsm3,
                       eucd_stock_FSM0_frames=[{"Byte_0": 0, "ACC_AbortCode": 0, "ACC_Enabled": 1}],
                       eucd_stock_FSM4_frames=[{"Byte_2": 0}], out=SimpleNamespace(vEgo=0.))
  fs.update(SimpleNamespace(longActive=False), CS)
  assert fs.tpl_fsm3["data"]["ACC_Check"] == 0
  assert cam_fsm3["ACC_Check"] == 1  # the camera's frame untouched


def stopped_full_stop():
  """OP's stop at standstill, the car holding."""
  from opendbc.car.volvo.eucd_full_stop import EucdFullStop
  fs = EucdFullStop()
  fs.active = fs.cam_standby = True
  fs.tpl_fsm4 = {"Byte_0": 0x55, "ACC_BrakeRequest": 1}
  CS = SimpleNamespace(eucd_ecm_acc_engaged=True, eucd_car_hold=True,
                       eucd_stock_FSM3={"ACC_NoTargetCancel": 0, "Byte_6": 0, "FrameKind": 0x81, "ACC_Check": 0},
                       eucd_stock_FSM0_frames=[{"Byte_0": 0, "ACC_AbortCode": 0, "ACC_Enabled": 0}],
                       eucd_stock_FSM4_frames=[{"Byte_2": 0}],
                       out=SimpleNamespace(vEgo=0., standstill=True, gasPressed=False))
  CC = SimpleNamespace(enabled=True, longActive=True, actuators=SimpleNamespace(accel=-1.))
  for _ in range(int(3. / DT_CTRL)):
    fs.update(CC, CS)
  assert fs.standstill and fs.holding
  return fs, CC, CS


def test_driver_gas_in_the_stop_drives_off_engaged():
  fs, CC, CS = stopped_full_stop()
  CS.out.gasPressed, CC.longActive = True, False
  fs.update(CC, CS)
  assert fs.active and fs.driver_gas and fs.driving_off
  assert not fs.standstill and not fs.holding
  fsm4, op_mode = fs.fsm4({"Byte_0": 0xAA})
  assert fsm4["ACC_BrakeRequest"] == 0 and not op_mode  # no brake from OP on the gas


def test_disengaged_on_the_gas_ends_the_stop():
  fs, CC, CS = stopped_full_stop()
  CS.out.gasPressed, CC.longActive, CC.enabled = True, False, False
  fs.update(CC, CS)
  assert not fs.active and not fs.driver_gas


def test_gas_release_keeps_the_stop_until_op_long_is_back():
  fs, CC, CS = stopped_full_stop()
  CS.out.gasPressed, CC.longActive = True, False
  fs.update(CC, CS)
  CS.out.gasPressed = False  # pedal released, OP's long back a frame later
  fs.update(CC, CS)
  assert fs.active and fs.driver_gas
  CC.longActive = True
  fs.update(CC, CS)
  assert fs.active and not fs.driver_gas
