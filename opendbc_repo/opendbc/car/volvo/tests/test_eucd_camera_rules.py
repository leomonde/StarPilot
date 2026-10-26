import math
from types import SimpleNamespace

import pytest

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
  assert cc._eucd_camera_brake(-3.5, HARD) == -3.5


def fsm3(mode):
  return SimpleNamespace(eucd_stock_FSM3={"Byte_2": 0xD4 | mode})


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


KPH = 1 / 3.6


def tolerance(v_kph, set_kph=110., accel=-0.5, pitch_deg=-2., lead_m=255):
  CC = SimpleNamespace(hudControl=SimpleNamespace(setSpeed=set_kph * KPH))
  CS = SimpleNamespace(out=SimpleNamespace(vEgo=v_kph * KPH), eucd_acc_distance=lead_m)
  return CarController._eucd_coast_tolerance(CC, CS, accel, offset(pitch_deg))


def test_coast_tolerance_downhill_over_set():
  assert tolerance(110.5)
  assert tolerance(114.)
  assert not tolerance(115.5)          # above set + 5: brakes
  assert not tolerance(90.)  # held below the set: no tolerance
  assert not tolerance(112., pitch_deg=0.)   # flat
  assert not tolerance(112., accel=-1.2)     # strong request
  assert not tolerance(112., lead_m=40)  # camera target closer than 2.5 s
  assert tolerance(112., lead_m=120)


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
