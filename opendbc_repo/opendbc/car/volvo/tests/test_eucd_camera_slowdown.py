from types import SimpleNamespace

from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CarControllerParams

CCP = CarControllerParams
DT = 0.02


def controller():
  cc = CarController.__new__(CarController)
  cc.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
  cc.eucd_oppose_time = 0.
  cc.eucd_slowdown_w = 0.
  cc.eucd_cam_brake = False
  cc.eucd_cam_brake_honor = False
  return cc


def run(cc, seconds, accel, cam_accel, distance=20):
  CS = SimpleNamespace(eucd_acc_distance=distance)
  out = accel
  for _ in range(round(seconds / DT)):
    out = cc._eucd_camera_slowdown(CS, accel, cam_accel)
  return out


def test_op_free_during_grace():
  cc = controller()
  assert run(cc, CCP.EUCD_CAMERA_SLOWDOWN_GRACE - 0.1, 0.1, -0.4) == 0.1


def test_op_follows_camera_after_grace_and_ramp():
  cc = controller()
  run(cc, CCP.EUCD_CAMERA_SLOWDOWN_GRACE, 0.1, -0.4)
  mid = run(cc, CCP.EUCD_CAMERA_SLOWDOWN_RAMP / 2, 0.1, -0.4)
  assert -0.4 < mid < 0.1
  assert abs(run(cc, CCP.EUCD_CAMERA_SLOWDOWN_RAMP, 0.1, -0.4) + 0.4) < 1e-6


def test_releases_when_camera_stops_slowing():
  cc = controller()
  run(cc, CCP.EUCD_CAMERA_SLOWDOWN_GRACE + CCP.EUCD_CAMERA_SLOWDOWN_RAMP, 0.1, -0.4)
  assert run(cc, CCP.EUCD_CAMERA_SLOWDOWN_RAMP, 0.1, 0.0) == 0.1
  assert cc.eucd_slowdown_w == 0.


def test_op_braking_more_is_never_limited():
  cc = controller()
  assert run(cc, 5., -1.0, -0.4) == -1.0


def test_no_target_no_limit():
  cc = controller()
  assert run(cc, 5., 0.1, -0.4, distance=255) == 0.1


def test_hard_camera_brake_followed_at_once_and_latched():
  cc = controller()
  cc.eucd_cam_brake = True
  assert run(cc, DT, 0.1, CCP.EUCD_CAMERA_BRAKE_HONOR_BELOW - 0.5) == CCP.EUCD_CAMERA_BRAKE_HONOR_BELOW - 0.5
  assert run(cc, DT, 0.1, -1.0) == -1.0  # latched while the camera brakes
  cc.eucd_cam_brake = False
  assert run(cc, DT, 0.1, -1.0) == 0.1   # released with the camera's brake, mild grace starts over


def test_mild_camera_brake_ignored_during_grace():
  cc = controller()
  cc.eucd_cam_brake = True
  assert run(cc, 1.0, 0.1, -1.0) == 0.1
  assert not cc.eucd_cam_brake_honor
