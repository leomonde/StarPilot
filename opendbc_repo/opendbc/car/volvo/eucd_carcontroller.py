import math

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.volvo.eucd_can import (create_eucd_button_msg, create_eucd_lka_msg, create_eucd_lkas_state_msg, create_eucd_longitudinal,
                                        create_eucd_fsm0, create_eucd_fsm4, create_eucd_radar, create_eucd_cam_brake_level,
                                        create_eucd_cam_brake_status, create_eucd_cam_wheel_status, create_eucd_cam_brake_request_ack,
                                        create_eucd_cam_brake_hold, create_eucd_cam_button, create_eucd_cam_engine_data, create_eucd_resume_button,
                                        create_eucd_cam_acc_hold, eucd_next_long_mode)
from opendbc.car.volvo.eucd_full_stop import EucdFullStop
from opendbc.car.volvo.eucd_params import CarControllerEUCDParams, LongMode, SteerDirection


def eucd_cam_brake_leftover(CS) -> bool:
  """Camera still in brake mode with no target and no braking asked: its brake kept 6 s after a false target (route 8bd, 193.6 s)."""
  return CS.eucd_acc_distance >= 255 and float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"]) >= 0.


class CarControllerEUCD(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.eucd_apply_steer_prev = 0
    self.eucd_apply_steer_dir_prev = SteerDirection.NONE
    self.eucd_latActive_prev = False
    self.eucd_steer_blocked = False
    self.eucd_steer_blocked_cnt = 0
    self.eucd_steer_dir_bf_block = SteerDirection.NONE
    self.eucd_last_resume_frame = 0
    self.eucd_distance = 0
    self.eucd_waiting = False
    self.eucd_sng_count = 0
    self.eucd_op_resume_frames = 0
    self.eucd_sng_ack_frames = 0
    self.eucd_resume_hold_frames = 0  # Resume held to the ECM, frames left
    self.eucd_last_ccbuttons = None  # CEM's last CCButtons, copied for the Resume
    self.eucd_downhill = False
    self.eucd_downhill_frames = 0  # frames the pitch has been past the enter/exit threshold
    self.eucd_next_long_tx_nanos = 0
    self.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
    self.eucd_long_mode = LongMode.COAST
    self.eucd_accel = 0.0
    self.eucd_cam_brake = False
    self.eucd_cam_brake_honor = False
    self.eucd_cam_hard_brake = False  # the camera brakes harder than OP's limit: its FSM3/FSM4 go out unchanged
    self.eucd_spoof_cam_ack = False
    self.eucd_cam_brake_target = 0
    self.eucd_cam_mask_frames = 0
    self.eucd_brake_level = 0
    self.eucd_brake_hold_b0: int | None = None
    self.eucd_full_stop = EucdFullStop()  # OP's stop to 0 (lead, red light, stop sign)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > CarControllerEUCDParams.EUCD_MIN_STEER_SPEED and not CS.out.brakePressed:
      can_sends.append(create_eucd_button_msg(self.packer, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self._eucd_lateral(CC, CS)

    self.eucd_full_stop.update(CC, CS)
    self._eucd_update_downhill(CC, CS)
    # Carstate takes the ACC standstill from OP during the stop
    CS.eucd_full_stop_active = self.eucd_full_stop.active
    CS.eucd_full_stop_standstill = self.eucd_full_stop.standstill
    self._eucd_resume_buttons(CC, CS, can_sends)
    self._eucd_relay_fsm0(CS, can_sends, CS.eucd_ecm_acc_engaged)

    if now_nanos >= self.eucd_next_long_tx_nanos:
      next_tx = self.eucd_next_long_tx_nanos + self.EUCD_LONG_TX_PERIOD_NANOS
      if self.eucd_next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.EUCD_LONG_TX_PERIOD_NANOS
      self.eucd_next_long_tx_nanos = next_tx
      can_sends += self._eucd_longitudinal(CC, CS)

    can_sends += self._eucd_relay_fsm4(CC, CS)
    can_sends += self._eucd_relay_brake_acks(CC, CS)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.eucd_apply_steer_prev
    if CC.longActive:
      new_actuators.accel = self.eucd_accel

    self.frame += 1
    return new_actuators, can_sends

  def _eucd_lateral(self, CC, CS):
    """FSM2 steering request and PSCM1 servo state to the camera, 50 Hz."""
    CCP = CarControllerEUCDParams
    actuators = CC.actuators
    if CC.latActive and CS.out.vEgo > CCP.EUCD_MIN_STEER_SPEED:
      apply_steer = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.eucd_apply_steer_prev, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive, CCP.EUCD_ANGLE_LIMITS
      )
      apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

      error = CS.out.steeringAngleDeg - apply_steer
      error_with_deadzone = 0 if abs(error) < CCP.EUCD_DEADZONE else error

      if not self.eucd_latActive_prev:
        self.eucd_apply_steer_dir_prev = apply_steer_dir

      # Block direction flips until the servo catches up
      if self.eucd_steer_blocked:
        if (apply_steer_dir == self.eucd_steer_dir_bf_block) or (self.eucd_steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
          self.eucd_steer_blocked = False
      else:
        if apply_steer_dir != self.eucd_apply_steer_dir_prev and error_with_deadzone != 0:
          self.eucd_steer_blocked = True
          self.eucd_steer_blocked_cnt = CCP.EUCD_BLOCK_LEN
          self.eucd_steer_dir_bf_block = self.eucd_apply_steer_dir_prev

      if self.eucd_steer_blocked:
        self.eucd_steer_blocked_cnt -= 1
        apply_steer_dir = SteerDirection.NONE
      elif error_with_deadzone == 0:
        apply_steer_dir = self.eucd_apply_steer_dir_prev
    else:
      apply_steer = 0
      apply_steer_dir = SteerDirection.NONE

    self.eucd_apply_steer_prev = apply_steer
    self.eucd_apply_steer_dir_prev = apply_steer_dir
    self.eucd_latActive_prev = CC.latActive

    return [
      create_eucd_lka_msg(self.packer, apply_steer, int(apply_steer_dir)),
      # Servo state to the camera
      create_eucd_lkas_state_msg(self.packer, CS.out.steeringAngleDeg, CS.eucd_pscm_stock_values),
    ]

  def _eucd_resume_buttons(self, CC, CS, can_sends):
    """Resume burst when the lead moves away."""
    CCP = CarControllerEUCDParams
    at_standstill = CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01

    self.eucd_op_resume_frames = self.eucd_op_resume_frames + 1 if CC.cruiseControl.resume else 0
    op_resume = self.eucd_op_resume_frames * DT_CTRL >= CCP.EUCD_SNG_OP_RESUME_TIME

    if (self.frame - self.eucd_last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.eucd_waiting:
        self.eucd_distance = CS.eucd_acc_distance
        self.eucd_waiting = True
        self.eucd_sng_count = 0
      # In OP's stop only the planner's resume drives off (as Ford/Toyota), not the camera's distance jitter (route 884)
      lead_moved = CS.eucd_acc_distance > self.eucd_distance and not self.eucd_full_stop.active
      # The driver's Resume also drives off OP's stop: the ECM takes it, but held in standstill it stays (route 8ab, 436 s)
      driver_resume = CS.eucd_driver_resume and self.eucd_full_stop.active
      drive_off = lead_moved or op_resume or driver_resume or self.eucd_full_stop.driver_gas
      if at_standstill and self.eucd_waiting and drive_off and self.eucd_full_stop.drive_off_ready(CS):
        self.eucd_resume_hold_frames = CCP.EUCD_RESUME_HOLD_FRAMES
        if self.eucd_sng_count == 0:
          self.eucd_sng_ack_frames = 25
        self.eucd_sng_count += 1
        self.eucd_full_stop.drive_off_request = self.eucd_full_stop.active
      if self.eucd_waiting and (self.eucd_sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.eucd_waiting = False
        self.eucd_last_resume_frame = self.frame

    # Resume held as the driver's: the CEM's frame with Resume pressed, every frame
    if CS.eucd_stock_CCButtons_frames:
      self.eucd_last_ccbuttons = CS.eucd_stock_CCButtons_frames[-1]
    if self.eucd_resume_hold_frames > 0 and self.eucd_last_ccbuttons is not None:
      can_sends.append(create_eucd_resume_button(self.packer, self.eucd_last_ccbuttons))
    self.eucd_resume_hold_frames = max(self.eucd_resume_hold_frames - 1, 0)

  def _eucd_update_downhill(self, CC, CS):
    """Downhill from the calibrated pitch, with hysteresis: carstate reports the set speed + offset then."""
    CCP = CarControllerEUCDParams
    pitch = math.degrees(CC.orientationNED[1]) if len(CC.orientationNED) == 3 else 0.
    past = pitch > CCP.EUCD_DOWNHILL_EXIT_PITCH if self.eucd_downhill else pitch < CCP.EUCD_DOWNHILL_ENTER_PITCH
    self.eucd_downhill_frames = self.eucd_downhill_frames + 1 if past else 0
    hold_time = CCP.EUCD_DOWNHILL_EXIT_TIME if self.eucd_downhill else CCP.EUCD_DOWNHILL_ENTER_TIME
    if self.eucd_downhill_frames * DT_CTRL >= hold_time:
      self.eucd_downhill = not self.eucd_downhill
      self.eucd_downhill_frames = 0
    CS.eucd_downhill = self.eucd_downhill

  def _eucd_relay_fsm0(self, CS, can_sends, ecm_engaged):
    """FSM0 relayed, never ACC enabled without the ECM engaged."""
    for stock_fsm0 in CS.eucd_stock_FSM0_frames:
      can_sends.append(create_eucd_fsm0(self.packer, self.eucd_full_stop.fsm0(stock_fsm0), ecm_engaged))

  def _eucd_relay_fsm4(self, CC, CS):
    """FSM4 relayed; OP sets brake flag and target."""
    sends = []
    fsm4_mode = self.eucd_long_mode if (CC.longActive and not self.eucd_cam_hard_brake) else None
    for stock_fsm4 in CS.eucd_stock_FSM4_frames:
      self.eucd_cam_brake = bool(stock_fsm4["ACC_BrakeRequest"])
      mode = fsm4_mode
      if self.eucd_full_stop.active:
        stock_fsm4, op_mode = self.eucd_full_stop.fsm4(stock_fsm4)
        mode = mode if op_mode else None
      sends.append(create_eucd_fsm4(self.packer, stock_fsm4, self.eucd_accel, mode))
    return sends

  def _eucd_longitudinal(self, CC, CS):
    """Accel and mode, FSM1/FSM3 at 50 Hz."""
    if CS.eucd_stock_FSM4_frames:
      self.eucd_cam_brake = bool(CS.eucd_stock_FSM4_frames[-1]["ACC_BrakeRequest"])
    if self.eucd_full_stop.driver_gas:
      # Driver's gas: no accel nor brake from OP, from the first frame the panda sees the pedal
      accel, long_mode = 0., LongMode.COAST
      self.eucd_long_mode = LongMode.COAST
      self.eucd_cam_brake_honor = False
    elif CC.longActive:
      accel, long_mode = self._eucd_long_decision(CC, CS)
    else:
      accel = float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"])
      self.eucd_long_mode = LongMode.COAST
      self.eucd_cam_brake_honor = False
      long_mode = None
    if self.eucd_full_stop.holding:
      # The car holds the stop: the camera's standstill request (route 807)
      accel = CarControllerEUCDParams.EUCD_HOLD_ACCEL
      long_mode = self.eucd_long_mode = LongMode.BRAKE
    self.eucd_accel = accel
    # The panda only lets the camera's own frames past OP's limit, so they go out unchanged
    cam_accel = float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"])
    self.eucd_cam_hard_brake = (CC.longActive and self.eucd_cam_brake_honor and not self.eucd_full_stop.active and
                                cam_accel < CarControllerEUCDParams.EUCD_ACCEL_MIN)

    self.eucd_spoof_cam_ack = (CC.longActive and self.eucd_cam_brake and not self.eucd_cam_brake_honor and
                               self.eucd_long_mode != LongMode.BRAKE and not self.eucd_full_stop.active)

    stock_fsm3 = self.eucd_full_stop.fsm3(CS)
    # OP's own frames within the panda's limit, also when following the camera's brake (route 895, 384.3 s)
    accel = max(accel, CarControllerEUCDParams.EUCD_ACCEL_MIN)
    if self.eucd_sng_ack_frames > 0:
      acc_check = 1
      self.eucd_sng_ack_frames -= 1
    else:
      acc_check = int(stock_fsm3["ACC_Check"])

    if self.eucd_full_stop.active:
      # OP's mode as is (the template's mode is stale), standstill from OP, camera cancel masked
      return [
        create_eucd_radar(self.packer, CS.eucd_stock_FSM1, long_mode),
        create_eucd_longitudinal(self.packer, stock_fsm3, accel, acc_check, long_mode, standstill=self.eucd_full_stop.standstill),
      ]
    if self.eucd_cam_hard_brake:
      return [
        create_eucd_radar(self.packer, CS.eucd_stock_FSM1),
        create_eucd_longitudinal(self.packer, stock_fsm3, cam_accel, int(stock_fsm3["ACC_Check"])),
      ]
    return [
      create_eucd_radar(self.packer, CS.eucd_stock_FSM1),
      create_eucd_longitudinal(self.packer, stock_fsm3, accel, acc_check, self._eucd_fsm3_mode(CS, long_mode)),
    ]

  def _eucd_long_decision(self, CC, CS):
    """Accel and mode with OP long active."""
    CCP = CarControllerEUCDParams
    actuators = CC.actuators
    accel = min(max(actuators.accel, CCP.EUCD_ACCEL_MIN), CCP.EUCD_ACCEL_MAX)
    accel = self._eucd_jerk_limit(accel, self.eucd_accel)
    cam_accel = float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"])
    accel = self._eucd_camera_accel_cap(CS, accel, cam_accel)
    accel = self._eucd_camera_brake(accel, cam_accel, eucd_cam_brake_leftover(CS))
    brake_offset = self._eucd_slope_brake_offset(CC)

    if self.eucd_cam_brake_honor:
      self.eucd_long_mode = LongMode.BRAKE
    else:
      self.eucd_long_mode = eucd_next_long_mode(self.eucd_long_mode, accel, brake_offset)
    return accel, self.eucd_long_mode

  def _eucd_jerk_limit(self, accel, last_accel):
    """Ramp OP's accel request (jerk limits)."""
    CCP = CarControllerEUCDParams
    dt = self.EUCD_LONG_TX_PERIOD_NANOS * 1e-9
    down = CCP.EUCD_JERK_DOWN_LIFT if last_accel > 0. else CCP.EUCD_JERK_DOWN_BRAKE
    return min(max(accel, last_accel - down * dt), last_accel + CCP.EUCD_JERK_UP * dt)

  # ---- Camera rules ----

  @staticmethod
  def _eucd_camera_accel_cap(CS, accel, cam_accel):
    """No more accel than the camera's while its target is close."""
    CCP = CarControllerEUCDParams
    close = max(CS.out.vEgo * CCP.EUCD_CAMERA_ACCEL_CAP_GAP, CCP.EUCD_CAMERA_ACCEL_CAP_MIN_DIST)
    if CS.eucd_acc_distance >= min(close, 255):
      return accel
    return min(accel, max(cam_accel, CCP.EUCD_CAMERA_ACCEL_CAP_FLOOR))

  def _eucd_camera_brake(self, accel, cam_accel, leftover=False):
    """Follow the camera's hard brake, latched until it releases or only its leftover brake state is left."""
    if not self.eucd_cam_brake or leftover:
      self.eucd_cam_brake_honor = False
    elif cam_accel < CarControllerEUCDParams.EUCD_CAMERA_BRAKE_HONOR_BELOW:
      self.eucd_cam_brake_honor = True
    return min(accel, cam_accel) if self.eucd_cam_brake_honor else accel

  @staticmethod
  def _eucd_fsm3_mode(CS, long_mode):
    """FSM3 mode: 0 when the camera is in it, else OP's."""
    if long_mode is None:
      return None
    if int(CS.eucd_stock_FSM3["ACC_Mode"]) == LongMode.BRAKE and not eucd_cam_brake_leftover(CS):
      return LongMode.BRAKE
    return long_mode

  @staticmethod
  def _eucd_slope_brake_offset(CC):
    """Slope part of the coast accel from pitch."""
    if len(CC.orientationNED) != 3:
      return 0.
    CCP = CarControllerEUCDParams
    return min(-CCP.EUCD_COAST_PITCH_GAIN * math.sin(CC.orientationNED[1]), CCP.EUCD_DOWNHILL_BRAKE_OFFSET_MAX)

  def _eucd_relay_brake_acks(self, CC, CS):
    """Brake acks to the camera: masked or spoofed."""
    CCP = CarControllerEUCDParams
    full_stop = self.eucd_full_stop
    if self.eucd_cam_brake and not full_stop.active:
      self.eucd_cam_mask_frames = 0
    elif (CC.longActive and self.eucd_long_mode == LongMode.BRAKE) or full_stop.active:
      # OP's stop: the camera never asked, so it never sees the car brake or hold
      self.eucd_cam_mask_frames = CCP.EUCD_CAM_BRAKE_MASK_HOLD_FRAMES
    elif self.eucd_cam_mask_frames > 0:
      self.eucd_cam_mask_frames -= 1
    mask = self.eucd_cam_mask_frames > 0
    masked = CCP.EUCD_CAM_BRAKE_MASK_MSGS
    mask_brake_level, mask_brake_status, mask_wheelspeed1, mask_brake_req_ack = (
      mask and msg in masked for msg in ("BrakeLevel", "BrakeStatus", "wheelspeed1", "BrakeReqAck"))
    spoof_wheelspeed1, spoof_brake_req_ack = (self.eucd_spoof_cam_ack and msg in masked for msg in ("wheelspeed1", "BrakeReqAck"))
    if CS.eucd_stock_FSM4_frames:
      last_fsm4 = CS.eucd_stock_FSM4_frames[-1]
      self.eucd_cam_brake_target = int(last_fsm4["ACC_BrakeTarget"])
    spoof_brake_req_ack = spoof_brake_req_ack and self.eucd_cam_brake_target < CCP.EUCD_CAM_ACK_RELEASE_VALUE

    cam_standby = full_stop.show_cam_standby

    sends = []
    for stock_brake_level in CS.eucd_stock_BrakeLevel_frames:
      if not mask_brake_level:
        self.eucd_brake_level = int(stock_brake_level["BrakeLevel"])
      sends.append(create_eucd_cam_brake_level(self.packer, stock_brake_level, self.eucd_brake_level if mask_brake_level else None))
    for stock_brake_status in CS.eucd_stock_BrakeStatus_frames:
      sends.append(create_eucd_cam_brake_status(self.packer, stock_brake_status, mask_brake_status))
    for stock_wheelspeed1 in CS.eucd_stock_wheelspeed1_frames:
      sends.append(create_eucd_cam_wheel_status(self.packer, stock_wheelspeed1, mask_wheelspeed1, spoof_wheelspeed1, hide_hold=mask,
                                                 show_standby=cam_standby))
    for stock_brake_req_ack in CS.eucd_stock_BrakeReqAck_frames:
      sends.append(create_eucd_cam_brake_request_ack(self.packer, stock_brake_req_ack, mask_brake_req_ack, spoof_brake_req_ack, hide_hold=mask))
    # EngineData and 0x325 always relayed (the panda blocks them); ACC standby while the camera is shown it
    for stock_engine_data in CS.eucd_stock_EngineData_frames:
      sends.append(create_eucd_cam_engine_data(self.packer, stock_engine_data, cam_standby))
    for stock_acc_hold in CS.eucd_stock_AccHold_frames:
      sends.append(create_eucd_cam_acc_hold(self.packer, stock_acc_hold, cam_standby))
    # CCButtons always relayed (the panda blocks them), with Set held while re-engaging the camera
    set_plus = full_stop.take_cam_set()
    for stock_ccbuttons in CS.eucd_stock_CCButtons_frames:
      sends.append(create_eucd_cam_button(self.packer, stock_ccbuttons, set_plus))
    # 0x1FF b0 held from the stop on, while stopped in OP's stop
    hold_mask = full_stop.active and CS.out.standstill
    for stock_brake_hold in CS.eucd_stock_BrakeHold_frames:
      if not hold_mask or self.eucd_brake_hold_b0 is None:
        self.eucd_brake_hold_b0 = int(stock_brake_hold["Byte_0"])
      sends.append(create_eucd_cam_brake_hold(self.packer, stock_brake_hold, self.eucd_brake_hold_b0 if hold_mask else None))
    return sends
