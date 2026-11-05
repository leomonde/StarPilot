#!/usr/bin/env python3
import unittest

import numpy as np

from opendbc.car.structs import CarParams
from opendbc.car.volvo.values import VolvoSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

VOLVO_MAIN_BUS = 0
VOLVO_PARTY_BUS = 2


class TestVolvoEUCD(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  """Volvo EUCD: V60 2010-18, lateral over the stock ACC."""
  TX_MSGS = [[0x127, VOLVO_MAIN_BUS], [0x262, VOLVO_MAIN_BUS], [0x246, VOLVO_PARTY_BUS]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {VOLVO_MAIN_BUS: [0x262], VOLVO_PARTY_BUS: [0x246]}
  FWD_BLACKLISTED_ADDRS = {VOLVO_PARTY_BUS: [0x262], VOLVO_MAIN_BUS: [0x246]}

  # Angle control limits
  STEER_ANGLE_MAX = 90  # deg, matches CarControllerEUCDParams.EUCD_ANGLE_LIMITS
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def setUp(self):
    self.packer = CANPackerSafety("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, VolvoSafetyFlags.EUCD.value)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    # LKAAngleReq carries the signed angle the safety rate-limits. LKASteerDirection is the car's per-frame actuation
    # gate, but not used by the safety: it drops to NONE during the anti-windup pause while OP stays engaged.
    direction = (2 if angle > 0 else 1) if enabled else 0
    values = {"LKAAngleReq": angle, "LKASteerDirection": direction}
    return self.packer.make_can_msg_safety("FSM2", VOLVO_MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    return self.packer.make_can_msg_safety("PSCM1", VOLVO_MAIN_BUS, {"SteeringAngleServo": angle})

  def _pcm_status_msg(self, enable):
    # engagement comes from the ECM (EngineData), not from the camera's FSM0
    values = {"ACC_State_ECM": 3 if enable else 2}  # engaged / standby
    return self.packer.make_can_msg_safety("EngineData", VOLVO_MAIN_BUS, values)

  def _speed_msg(self, speed: float):
    return self.packer.make_can_msg_safety("VehicleSpeed1", VOLVO_MAIN_BUS, {"VehicleSpeed": speed * 3.6})

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("Brake_Info", VOLVO_MAIN_BUS, {"BrakePedal": 2 if brake else 0})

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("AccPedal", VOLVO_MAIN_BUS, {"AccPedal": 10 if gas else 0})

  def test_ecm_acc_off_is_not_engaged(self):
    # EngineData ACC_State_ECM: 1 off, 2 standby, 3 engaged. Turning ACC off (On/Off in standby) gives 1,
    # which has bit 4 set like engaged (route 823); only 3 allows controls
    for state, engaged in ((1, False), (2, False), (3, True)):
      self.safety.set_controls_allowed(False)
      self._rx(self._pcm_status_msg(False))
      self._rx(self.packer.make_can_msg_safety("EngineData", VOLVO_MAIN_BUS, {"ACC_State_ECM": state}))
      self.assertEqual(engaged, self.safety.get_controls_allowed(), f"{state=}")

  def test_camera_fsm0_does_not_engage(self):
    # the camera's FSM0 is not an engagement source
    self.safety.set_controls_allowed(False)
    self._rx(self.packer.make_can_msg_safety("FSM0", VOLVO_PARTY_BUS, {"ACC_Enabled": 1}))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_rules(self):
    # Resume needs controls allowed, cancel needs cruise engaged
    resume = {"ACCResumeBtn": 1, "ACCResumeBtnInv": 0, "ACCOnOffBtnInv": 1}
    cancel = {"ACCOnOffBtn": 1, "ACCOnOffBtnInv": 0, "ACCResumeBtnInv": 1}
    self._rx(self._pcm_status_msg(False))
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self.packer.make_can_msg_safety("CCButtons", VOLVO_MAIN_BUS, resume)))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("CCButtons", VOLVO_MAIN_BUS, cancel)))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("CCButtons", VOLVO_MAIN_BUS, cancel)))

  # Volvo gates the angle command on engagement, not on the LKASteerDirection flag, and only a zero
  # angle passes while disengaged. These mirror AngleSteeringSafetyTest with those expectations.
  def test_angle_cmd_when_disabled(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      for angle_meas in np.arange(-90, 91, 10):
        self._reset_angle_measurement(angle_meas)

        for angle_cmd in np.arange(-90, 91, 10):
          self._set_prev_desired_angle(angle_cmd)

          # engaged: angle (held at prev) is allowed; disengaged: only a zero angle passes
          should_tx = controls_allowed or (angle_cmd == 0)
          self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_cmd, True)))

  def test_angle_cmd_when_enabled(self):
    # when controls are allowed, angle cmd rate limit is enforced
    speeds = [0., 1., 5., 10., 15., 50.]
    angles = np.concatenate((np.arange(-self.STEER_ANGLE_MAX * 2, self.STEER_ANGLE_MAX * 2, 5), [0]))
    for a in angles:
      for s in speeds:
        max_delta_up = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP)
        max_delta_down = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN)

        self._reset_angle_measurement(a)
        self._reset_speed_measurement(s)
        self._set_prev_desired_angle(a)
        self.safety.set_controls_allowed(1)

        # Stay within limits
        self.assertTrue(self._tx(self._angle_cmd_msg(a + common.sign_of(a) * max_delta_up, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(a - common.sign_of(a) * max_delta_down, True)))

        # Inject too high rates
        self.assertFalse(self._tx(self._angle_cmd_msg(a + common.sign_of(a) * (max_delta_up + 1.1), True)))
        self.safety.set_controls_allowed(1)
        self._set_prev_desired_angle(a)
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertFalse(self._tx(self._angle_cmd_msg(a - common.sign_of(a) * (max_delta_down + 1.1), True)))

        # When disabled, only a zero angle is allowed
        self.safety.set_controls_allowed(0)
        self.assertEqual(a == 0, self._tx(self._angle_cmd_msg(a, False)))


if __name__ == "__main__":
  unittest.main()
