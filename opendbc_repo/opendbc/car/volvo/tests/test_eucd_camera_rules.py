import math
from types import SimpleNamespace

import pytest

from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CarControllerParams, LongMode
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


def test_fsm3_untouched_without_op_mode():
  assert CarController._eucd_fsm3_mode(fsm3(LongMode.BRAKE), None) is None


def offset(pitch_deg):
  return CarController._eucd_uphill_brake_offset(SimpleNamespace(orientationNED=[0., math.radians(pitch_deg), 0.]))


def test_uphill_coast_is_not_braking():
  # route 858: -0.84 requested on a 5.4 deg slope is just coasting
  assert eucd_next_long_mode(LongMode.COAST, -0.84) == LongMode.BRAKE
  assert eucd_next_long_mode(LongMode.COAST, -0.84, offset(5.4)) == LongMode.COAST
  assert eucd_next_long_mode(LongMode.COAST, -1.2, offset(5.4)) == LongMode.BRAKE


def test_flat_and_downhill_unchanged():
  assert offset(0.) == 0.
  assert offset(-5.) == 0.
  assert CarController._eucd_uphill_brake_offset(SimpleNamespace(orientationNED=[])) == 0.
