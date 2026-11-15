import math
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL
from opendbc.car.volvo.eucd_carcontroller import CarControllerEUCD as CarController
from opendbc.car.volvo.eucd_params import CarControllerEUCDParams as CarControllerParams, LongMode
from opendbc.car.volvo import eucd_can as volvocan
from opendbc.car.volvo.eucd_can import eucd_next_long_mode

CCP = CarControllerParams
HARD = CCP.EUCD_CAMERA_BRAKE_HONOR_BELOW - 0.5


def controller(cam_brake=False):
  cc = CarController.__new__(CarController)
  cc.eucd_cam_brake = cam_brake
  cc.eucd_cam_brake_honor = False
  cc.eucd_cam_delta_frames = 0
  cc.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
  return cc


HOLD_FRAMES = round(CCP.EUCD_CAMERA_BRAKE_HONOR_DELTA_TIME / 0.02)


def test_hard_camera_brake_followed_at_once_and_latched():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.1, HARD) == HARD
  assert cc._eucd_camera_brake(0.1, -1.0) == -1.0  # latched while the camera brakes
  cc.eucd_cam_brake = False
  assert cc._eucd_camera_brake(0.1, -1.0) == 0.1   # released with the camera's brake


def test_mild_camera_brake_ignored():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.1, -1.0) == 0.1
  assert cc._eucd_camera_brake(-0.75, -2.2) == -0.75  # 1.45 harder: still OP's
  assert not cc.eucd_cam_brake_honor


def test_camera_brake_well_past_ops_for_2s_is_followed_and_latched():
  # Route 1f: cut-in, camera -2.56 vs OP +0.09 (665.4 s); camera -2.00 vs OP -0.48 before its abort 0x10 (721.7 s)
  for op, cam in ((0.09, -2.56), (-0.48, -2.00)):
    cc = controller(cam_brake=True)
    for _ in range(HOLD_FRAMES - 1):
      assert cc._eucd_camera_brake(op, cam) == op  # OP's own until the delta has lasted 2 s
    assert cc._eucd_camera_brake(op, cam) == cam
    assert cc._eucd_camera_brake(op, -0.5) == -0.5  # latched while the camera brakes, also when it eases
    cc.eucd_cam_brake = False
    assert cc._eucd_camera_brake(op, -0.5) == op


def test_short_camera_delta_is_left_to_op():
  # Leads braking in routes 10 and 1a: under 2 s past OP, OP handled them
  cc = controller(cam_brake=True)
  for _ in range(3):
    for _ in range(HOLD_FRAMES - 1):
      assert cc._eucd_camera_brake(0.0, -2.0) == 0.0
    assert cc._eucd_camera_brake(0.0, -1.0) == 0.0  # back within the delta: the hold starts over
  assert not cc.eucd_cam_brake_honor


def test_hard_camera_brake_needs_no_hold():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.0, HARD) == HARD


def test_camera_delta_needs_its_brake_request():
  cc = controller(cam_brake=False)
  for _ in range(HOLD_FRAMES + 1):
    assert cc._eucd_camera_brake(0.5, -1.5) == 0.5
  assert not cc.eucd_cam_brake_honor


def test_camera_brake_released_when_only_its_leftover_is_left():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(0.1, HARD) == HARD
  assert cc._eucd_camera_brake(1.9, 0.96, leftover=True) == 1.9  # target gone, camera asks no brake (route 8bd)
  assert not cc.eucd_cam_brake_honor


def test_op_braking_more_is_never_limited():
  cc = controller(cam_brake=True)
  assert cc._eucd_camera_brake(HARD - 0.5, HARD) == HARD - 0.5


def fsm3(mode, distance=50, cam_accel=-1.0):
  return SimpleNamespace(eucd_stock_FSM3={"ACC_Mode": mode, "ACC_AccelerationRequest": cam_accel}, eucd_acc_distance=distance)


@pytest.mark.parametrize("op_mode", [LongMode.COAST, LongMode.DRIVE])
def test_fsm3_leaves_the_camera_brake_leftover(op_mode):
  assert CarController._eucd_fsm3_mode(fsm3(LongMode.BRAKE, distance=255, cam_accel=0.96), op_mode) == op_mode
  assert CarController._eucd_fsm3_mode(fsm3(LongMode.BRAKE, distance=255, cam_accel=-0.5), op_mode) == LongMode.BRAKE


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


def limited(accel, last):
  cc = CarController.__new__(CarController)
  cc.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
  return cc._eucd_jerk_limit(accel, last)


def test_jerk_limit_only_cuts_jumps():
  assert abs(limited(1.3, 0.0) - CCP.EUCD_JERK_UP * 0.02) < 1e-9               # rising moderate
  assert abs(limited(-3.5, -1.0) - (-1.0 - CCP.EUCD_JERK_DOWN * 0.02)) < 1e-9  # braking only cut on jumps
  assert limited(-0.4, -0.2) == -0.4                   # the planner's own steps pass
  # 0 to -3 m/s^2 in 0.25 s, not 0.75 s as before (route 1f: braking started late)
  a, frames = 0., 0
  while a > -3.:
    a, frames = limited(-3., a), frames + 1
  assert frames * 0.02 <= 0.26


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


def stopped_full_stop(seconds=3.):
  """OP's stop at standstill, the car holding."""
  from opendbc.car.volvo.eucd_full_stop import EucdFullStop
  fs = EucdFullStop()
  fs.active = fs.cam_standby = True
  fs.tpl_fsm4 = {"Byte_0": 0x55, "ACC_BrakeRequest": 1}
  CS = SimpleNamespace(eucd_ecm_acc_engaged=True, eucd_car_hold=True, eucd_ecm_standstill_ready=False,
                       eucd_stock_FSM3={"ACC_NoTargetCancel": 0, "Byte_6": 0, "FrameKind": 0x81, "ACC_Check": 0},
                       eucd_stock_FSM0_frames=[{"Byte_0": 0, "ACC_AbortCode": 0, "ACC_Enabled": 0}],
                       eucd_stock_FSM4_frames=[{"Byte_2": 0}],
                       out=SimpleNamespace(vEgo=0., standstill=True, gasPressed=False))
  CC = SimpleNamespace(enabled=True, longActive=True, actuators=SimpleNamespace(accel=-1.))
  for _ in range(int(seconds / DT_CTRL)):
    fs.update(CC, CS)
  assert fs.holding and fs.standstill == (seconds >= CCP.EUCD_STANDSTILL_DELAY)
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


def test_short_stop_drives_off_on_traction():
  fs, CC, CS = stopped_full_stop(seconds=1.)
  CC.actuators.accel = 0.5
  fs.update(CC, CS)
  assert fs.active and not fs.stopping and not fs.holding and not fs.standstill


def test_long_stop_waits_for_the_resume():
  fs, CC, CS = stopped_full_stop()
  CC.actuators.accel = 0.5
  fs.update(CC, CS)
  assert fs.stopping and fs.holding and fs.standstill


def test_ecm_in_standstill_waits_for_the_resume():
  fs, CC, CS = stopped_full_stop(seconds=1.)
  CS.eucd_ecm_standstill_ready, CC.actuators.accel = True, 0.5
  fs.update(CC, CS)
  assert fs.stopping and fs.holding


def resume_buttons_after(driver_resume, active):
  """One frame stopped in standstill: the Resume hold frames left and the drive-off request."""
  from opendbc.car.volvo.eucd_full_stop import EucdFullStop
  cc = CarController.__new__(CarController)
  cc.frame, cc.eucd_last_resume_frame = 1000, 0
  cc.eucd_op_resume_frames = cc.eucd_resume_hold_frames = cc.eucd_sng_ack_frames = cc.eucd_sng_count = 0
  cc.eucd_waiting, cc.eucd_distance, cc.eucd_last_ccbuttons = False, 0, None
  cc.eucd_full_stop = EucdFullStop()
  cc.eucd_full_stop.active = active
  CS = SimpleNamespace(out=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True, standstill=True), vEgo=0.),
                       eucd_acc_distance=0, eucd_stock_CCButtons_frames=[], eucd_driver_resume=driver_resume,
                       eucd_ecm_standstill_ready=True)
  cc._eucd_resume_buttons(SimpleNamespace(cruiseControl=SimpleNamespace(resume=False)), CS, [])
  return cc.eucd_resume_hold_frames, cc.eucd_full_stop.drive_off_request


def test_driver_resume_drives_off_ops_stop():
  assert resume_buttons_after(True, True) == (CCP.EUCD_RESUME_HOLD_FRAMES - 1, True)
  assert resume_buttons_after(False, True) == (0, False)
  assert resume_buttons_after(True, False) == (0, False)  # the camera's own stop takes it by itself
