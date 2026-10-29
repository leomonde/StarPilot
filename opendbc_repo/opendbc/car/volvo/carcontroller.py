import math
from collections import deque

import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.volvo.helpers import LCA3CounterSync
from opendbc.car.volvo.volvocan import (create_c1_cancel, create_c1_pscm_message, create_c1_steering_control, create_lca_message,
                                        create_pscm_message, create_lca_3_message, create_lca_2_message, create_lca_4_message, create_lca_5_message,
                                        create_lca_6_message, create_lca_7_message, create_pscm_related_message,
                                        create_eucd_button_msg, create_eucd_lka_msg, create_eucd_lkas_state_msg, create_eucd_longitudinal,
                                        create_eucd_fsm0, create_eucd_fsm4, create_eucd_radar, create_eucd_cam_brake_level,
                                        create_eucd_cam_brake_status, create_eucd_cam_wheel_status, create_eucd_cam_brake_request_ack,
                                        create_eucd_cam_brake_hold, create_eucd_cam_brake_info, create_eucd_cam_button, create_eucd_cam_engine_data,
                                        create_eucd_cam_acc_hold, create_eucd_cam_acc_speed,
                                        eucd_fsm4_brake_value, eucd_next_long_mode)
from opendbc.car.volvo.values import CAR, CarControllerParams, LongMode, SteerDirection, VolvoC1PlatformConfig, VolvoEUCDPlatformConfig

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.is_c1 = isinstance(CAR(CP.carFingerprint).config, VolvoC1PlatformConfig)
    self.is_eucd = isinstance(CAR(CP.carFingerprint).config, VolvoEUCDPlatformConfig)
    self.packer = CANPacker(dbc_names[Bus.pt] if (self.is_c1 or self.is_eucd) else dbc_names[Bus.party])
    self.apply_angle_last = 0.0  # last applied steering angle
    self.c1_torque_samples = deque(maxlen=CarControllerParams.C1_N_ZERO_TORQUE)
    self.c1_recovery_until = -1

    # EUCD (V60) state
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
    self.eucd_next_long_tx_nanos = 0
    self.EUCD_LONG_TX_PERIOD_NANOS = 20_000_000
    self.eucd_long_mode = LongMode.COAST
    self.eucd_accel = 0.0
    self.eucd_cam_brake = False
    self.eucd_cam_brake_honor = False
    self.eucd_cam_stop = False
    self.eucd_cam_stop_w = 0.
    self.eucd_cam_relay = False
    self.eucd_cam_weight = 0.
    self.eucd_spoof_cam_ack = False
    self.eucd_cam_brake_target = 0
    self.eucd_cam_mask_frames = 0
    self.eucd_brake_level = 0

    # No-target stop (red light): OP stops to 0 like the camera does behind a lead
    self.eucd_nt_active = False
    self.eucd_nt_cam_aborted = False  # camera cancelled: FSM0/FSM3/FSM4 from the templates
    self.eucd_nt_still_frames = 0
    self.eucd_nt_hold_frames = 0
    self.eucd_nt_going = False  # resume sent, standstill released
    self.eucd_nt_go_frames = 0
    self.eucd_nt_go_moved = False
    self.eucd_nt_go_request = False
    self.eucd_nt_standstill = False
    self.eucd_nt_stopping = False
    self.eucd_nt_holding = False
    self.eucd_nt_driver_gas = False  # driver on the gas: OP's long off, the stop kept as a drive-off (routes 896/897)
    self.eucd_nt_cam_standby = False  # camera shown ACC standby (EngineData, 0x325)
    self.eucd_nt_exit_standby_frames = 0  # stop ended (driver's brake, cancel) with the ECM still engaged: still standby
    self.eucd_nt_resync_frames = 0  # frames re-engaging the camera after the drive-off
    self.eucd_nt_resync_attempts = 0
    self.eucd_nt_clean_frames = 0
    self.eucd_cam_set_frames = 0  # Set press to the camera, frames left
    self.eucd_brake_hold_b0: int | None = None
    # Camera's last clean frames: FSM0 by Byte_0, FSM3 data/type, FSM4
    self.eucd_tpl_fsm0: dict[int, dict] = {}
    self.eucd_tpl_fsm3: dict[str, dict] = {}
    self.eucd_tpl_fsm4: dict | None = None

    self.gear_acc = 60
    self.lca_4_acc = 0  # Bresenham accumulator, 29 Hz

    # LCA_2 counters
    self.lca_2_counter_1 = None  # seeded from CarState
    self.lca_2_counter_2 = None

    # PSCM_RELATED counter
    self.pscm_related_counter = None  # seeded from CarState

    # LCA_3 counter
    self.lca_3_counter_sync = LCA3CounterSync()

    # LCA_5 counter
    self.lca_5_counter = None  # seeded from CarState

    self.last_lat_active = False  # last lat_active

    self.lca_7_acc = 0  # Bresenham accumulator, 29 Hz
    self.lca_7_last_steer = 0  # last LCA_7 steer

    # LCA torque-authority envelope state
    self.lca_auth_pos = 0.0
    self.lca_auth_neg = 0.0
    # Light-contact rising-edge detector
    self.lca_auth_drv_prev = 0.0
    self.lca_auth_light_frames = 0
    # Frames since the last real override
    self.lca_auth_real_off_frames = 1000  # light contact can fire at once
    # Override latch with hysteresis
    self.lca_auth_override_active = False
    # Low-passed |drv| for the yield arm
    self.lca_auth_drv_mag_filt = 0.0

  def update(self, CC, CS, now_nanos, starpilot_toggles):
    if self.is_c1:
      return self._update_c1(CC, CS)
    if self.is_eucd:
      return self._update_eucd(CC, CS, now_nanos)

    can_sends = []
    actuators = CC.actuators

    # Disengage
    if not CC.latActive and self.last_lat_active:
      #self.lca_commands.reset()  # Clear state ← IMPORTANT!
      pass

    lat_active = CC.latActive

    # Lateral: LCA sent every frame, stock LCA blocked by panda safety
    if self.frame % CarControllerParams.STEER_STEP == 0:  # 100 Hz
      # Desired angle from controlsd
      apply_angle = actuators.steeringAngleDeg  # degrees

      # Clamp the command to actual angle +- ANGLE_ERROR
      apply_angle = float(np.clip(
        apply_angle,
        CS.out.steeringAngleDeg - CarControllerParams.ANGLE_ERROR,
        CS.out.steeringAngleDeg + CarControllerParams.ANGLE_ERROR,
      ))

      # Rate limit; passthrough when inactive
      apply_angle = apply_std_steer_angle_limits(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                 CS.out.steeringAngleDeg, lat_active, CarControllerParams.ANGLE_LIMITS)

      # Update the LCA torque-authority envelope
      P = CarControllerParams
      DT = 0.01  # 100 Hz
      # Override from an explicit |steeringTorque| threshold
      drv_mag = abs(CS.out.steeringTorque)
      drv_rate = drv_mag - self.lca_auth_drv_prev
      self.lca_auth_drv_prev = drv_mag
      # Low-pass |drv| for the yield arm
      self.lca_auth_drv_mag_filt = ((1.0 - P.LCA_AUTH_YIELD_LP_ALPHA) * self.lca_auth_drv_mag_filt
                                    + P.LCA_AUTH_YIELD_LP_ALPHA * drv_mag)
      # Override latch: enter at ENTER, exit at EXIT
      if not self.lca_auth_override_active and drv_mag > P.LCA_AUTH_OVERRIDE_ENTER:
        self.lca_auth_override_active = True
      elif self.lca_auth_override_active and drv_mag < P.LCA_AUTH_OVERRIDE_EXIT:
        self.lca_auth_override_active = False
      real_override = self.lca_auth_override_active
      # Frames since the last real override
      if real_override:
        self.lca_auth_real_off_frames = 0
      else:
        self.lca_auth_real_off_frames += 1
      # Light-contact rising edge arms a brief yield
      if (drv_mag > P.LCA_AUTH_LIGHT_THRESH and
          drv_rate > P.LCA_AUTH_LIGHT_RISE_DELTA and
          self.lca_auth_light_frames == 0 and
          self.lca_auth_real_off_frames > P.LCA_AUTH_LIGHT_COOLDOWN_FRAMES):
        self.lca_auth_light_frames = P.LCA_AUTH_LIGHT_HOLD_FRAMES
      else:
        self.lca_auth_light_frames = max(0, self.lca_auth_light_frames - 1)
      light_collapse = self.lca_auth_light_frames > 0
      overriding = real_override or light_collapse
      # Collapse rate scales with driver torque
      collapse_rate = P.LCA_AUTH_COLLAPSE_RATE * max(1.0, drv_mag / float(P.LCA_AUTH_OVERRIDE_ENTER))
      step = collapse_rate * DT
      if not lat_active:
        self.lca_auth_pos = 0.0
        self.lca_auth_neg = 0.0
        self.lca_auth_drv_prev = 0.0
        self.lca_auth_light_frames = 0
        self.lca_auth_override_active = False
        self.lca_auth_drv_mag_filt = 0.0
        self.lca_auth_real_off_frames = 1000
      elif overriding:
        if self.lca_auth_pos > P.LCA_AUTH_SPLIT or -self.lca_auth_neg > P.LCA_AUTH_SPLIT:
          # Symmetric collapse toward +-SPLIT
          self.lca_auth_pos = max(float(P.LCA_AUTH_SPLIT), self.lca_auth_pos - step)
          self.lca_auth_neg = min(-float(P.LCA_AUTH_SPLIT), self.lca_auth_neg + step)
        else:
          # Asymmetric plateau: yield on the driver's side
          excess = max(0.0, self.lca_auth_drv_mag_filt - float(P.LCA_AUTH_OVERRIDE_ENTER))
          yield_signed = float(P.LCA_AUTH_YIELD_BASE) - P.LCA_AUTH_YIELD_SLOPE * excess
          yield_signed = max(float(P.LCA_AUTH_YIELD_MIN), min(yield_signed, float(P.LCA_AUTH_YIELD_BASE)))
          if CS.out.steeringTorque > 0:  # driver pushing right
            target_pos = yield_signed  # yield arm
            target_neg = -float(P.LCA_AUTH_PLATEAU_COUNTER)  # counter arm
          else:  # driver pushing left or zero
            target_pos = float(P.LCA_AUTH_PLATEAU_COUNTER)  # counter arm
            target_neg = -yield_signed  # yield arm
          # Arms toward their plateau at COLLAPSE_RATE
          self.lca_auth_pos = max(target_pos, self.lca_auth_pos - step) \
                              if self.lca_auth_pos > target_pos \
                              else min(target_pos, self.lca_auth_pos + step)
          self.lca_auth_neg = min(target_neg, self.lca_auth_neg + step) \
                              if self.lca_auth_neg < target_neg \
                              else max(target_neg, self.lca_auth_neg - step)
      else:
        # No override: rebuild to saturation
        rebuild_step = P.LCA_AUTH_REBUILD_RATE * DT
        self.lca_auth_pos = min(float(P.LCA_AUTH_MAX), self.lca_auth_pos + rebuild_step)
        self.lca_auth_neg = max(-float(P.LCA_AUTH_MAX), self.lca_auth_neg - rebuild_step)

      # LCA - 0x58 - 100 Hz (angle-based)
      can_sends.append(create_lca_message(self.packer, lat_active, apply_angle, CS.msg_lca,
                                          authority_pos=int(round(self.lca_auth_pos)),
                                          authority_neg=int(round(self.lca_auth_neg))))
      self.apply_angle_last = apply_angle

      # PSCM (bus 2 -> 0) - 0x16 - 100 Hz
      can_sends.append(create_pscm_message(self.packer, lat_active, CS.msg_pscm, self.frame))
      # EGSM - 0x45 - 100 Hz
      #can_sends.append(create_egsm_message(self.packer, CS.msg_egsm))

      # PSCM_RELATED (0x17), 100 Hz; counter seeded from CarState
      if self.pscm_related_counter is None:
        self.pscm_related_counter = CS.msg_pscm_related['SIG1_BYTE_1_HI_NIBBLE']

      # Counter +1, mod 15
      self.pscm_related_counter = (self.pscm_related_counter + 1) % 15

      can_sends.append(create_pscm_related_message(self.packer, lat_active, CS.pilot_assist_engaged,
                                                     CS.msg_pscm_related, self.pscm_related_counter))

    # LCA_3 - 0x57 - avg 66.66 Hz
    #if (self.frame * 67) % 100 < 67: # if (self.frame % 3) < 2:
    # 0x57 at ~66.67 Hz, skip frame % 3 == 1
    if self.frame % 3 != 1:  # 66.67 Hz
      # Counter from the observed value
      counter, is_synced = self.lca_3_counter_sync.update(CS.msg_lca_3['COUNTER_1'])
      can_sends.append(create_lca_3_message(self.packer, lat_active, apply_angle, CS.msg_lca_3, counter))
      #can_sends.append(create_0x1a_message(self.packer, CS.msg_0x1a))

    # SPEED messages - 0x60, 0x68 - 50 Hz
    if self.frame % 2 == 0: # 50 Hz
      #can_sends.append(create_speed_message(self.packer, CS.msg_speed))
      #can_sends.append(create_speed_2_message(self.packer, CS.msg_speed_2))
      pass

    # LCA_2 (0x69), 50 Hz: spoof PILOT_ASSIST_ENGAGED
    if self.frame % 2 == 0: # 50 Hz
      # Counters seeded from CarState
      if self.lca_2_counter_1 is None:
        self.lca_2_counter_1 = CS.msg_lca_2['COUNTER_1']
        self.lca_2_counter_2 = CS.msg_lca_2['COUNTER_2']

      # COUNTER_1 +2, COUNTER_2 +4, mod 16
      self.lca_2_counter_1 = (self.lca_2_counter_1 + 2) % 16
      self.lca_2_counter_2 = (self.lca_2_counter_2 + 4) % 16

      can_sends.append(create_lca_2_message(self.packer, lat_active, CS.msg_lca_2,
                                            self.lca_2_counter_1, self.lca_2_counter_2))

    # LCA_5 (0x67), 50 Hz: wheel speeds + LCA signals
    if self.frame % 2 == 0: # 50 Hz
      if not lat_active or self.lca_5_counter is None:
        self.lca_5_counter = CS.msg_lca_5['COUNTER']

      # Counter +4, wrap at 15
      if lat_active:
        self.lca_5_counter = (self.lca_5_counter + 4) % 15

      can_sends.append(create_lca_5_message(self.packer, lat_active, apply_angle,
                                            CS.msg_lca_5, self.lca_5_counter))

    # LCA_4 (0x90), 29 Hz: hold LCA_ENABLE so PA stays on
    self.lca_4_acc += 29
    if self.lca_4_acc >= 100:
      self.lca_4_acc -= 100
      can_sends.append(create_lca_4_message(self.packer, lat_active, CS.msg_lca_4, apply_angle))

    # LCA_6 - 0X97 - 25 Hz
    if self.frame % 4 == 0: # 25 Hz
        can_sends.append(create_lca_6_message(self.packer, lat_active, CS.msg_lca_6, apply_angle))

    # LCA_7 (0x92), 29 Hz (Bresenham)
    self.lca_7_acc += 29
    if self.lca_7_acc >= 100:
      self.lca_7_acc -= 100
      delta_steer = apply_angle - self.lca_7_last_steer
      can_sends.append(create_lca_7_message(self.packer, lat_active, CS.msg_lca_7, apply_angle, delta_steer))
      self.lca_7_last_steer = apply_angle

    # GEAR_POSITION - 0x80 - 40 Hz
    #self.gear_acc += 40 # Bresenham-style approach
    #if self.gear_acc >= 100:
    #    self.gear_acc -= 100
    if self.frame % 5 == 0 or self.frame % 5 == 2:  # 40 Hz
      #can_sends.append(create_gear_position_message(self.packer, CS.msg_gear_position))
      pass

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    self.last_lat_active = CC.latActive
    return new_actuators, can_sends

  # ---- V60/EUCD (from sunnypilot volvo_oplong) ----

  def _update_eucd(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > self.CP.minSteerSpeed and not CS.out.brakePressed:
      can_sends.append(create_eucd_button_msg(self.packer, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self._eucd_lateral(CC, CS)

    self._eucd_update_no_target_stop(CC, CS, can_sends)
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
    CCP = CarControllerParams
    actuators = CC.actuators
    if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
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
    CCP = CarControllerParams
    at_standstill = CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01

    self.eucd_op_resume_frames = self.eucd_op_resume_frames + 1 if CC.cruiseControl.resume else 0
    op_resume = self.eucd_op_resume_frames * DT_CTRL >= CCP.EUCD_SNG_OP_RESUME_TIME

    if (self.frame - self.eucd_last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.eucd_waiting:
        self.eucd_distance = CS.eucd_acc_distance
        self.eucd_waiting = True
        self.eucd_sng_count = 0
      # In the no-target stop only OP's resume (green) drives off: the camera's distance has no lead to follow
      # and its jitter drove off while OP still asked to stop (route 884, 30.9 s)
      lead_moved = CS.eucd_acc_distance > self.eucd_distance and not self.eucd_nt_active
      if at_standstill and self.eucd_waiting and (lead_moved or op_resume or self.eucd_nt_driver_gas):
        can_sends.extend(self._eucd_resume_burst())
        if self.eucd_sng_count == 0:
          self.eucd_sng_ack_frames = 25
        self.eucd_sng_count += 1
        self.eucd_nt_go_request = self.eucd_nt_active
      if self.eucd_waiting and (self.eucd_sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.eucd_waiting = False
        self.eucd_last_resume_frame = self.frame

  def _eucd_resume_burst(self):
    return [create_eucd_button_msg(self.packer, resume=True)] * CarControllerParams.EUCD_BUTTON_BURST

  def _eucd_relay_fsm0(self, CS, can_sends, ecm_engaged):
    """FSM0 relayed, never ACC enabled without the ECM engaged."""
    for stock_fsm0 in CS.eucd_stock_FSM0_frames:
      if self.eucd_nt_cam_aborted:
        # Camera's abort (ACC off, code 0x10) kept from the ECM
        stock_fsm0 = self.eucd_tpl_fsm0.get(int(stock_fsm0["Byte_0"]), next(iter(self.eucd_tpl_fsm0.values())))
      can_sends.append(create_eucd_fsm0(self.packer, stock_fsm0, ecm_engaged))

  def _eucd_relay_fsm4(self, CC, CS):
    """FSM4 relayed; OP sets brake flag and target."""
    sends = []
    fsm4_mode = self.eucd_long_mode if (CC.longActive and not self.eucd_cam_relay) else None
    for stock_fsm4 in CS.eucd_stock_FSM4_frames:
      self.eucd_cam_brake = bool(stock_fsm4["ACC_BrakeRequest"])
      mode = fsm4_mode
      if self.eucd_nt_active:
        if self.eucd_nt_cam_aborted:
          # Camera's abort (b2 FC) kept from the ECM; b0 alternates 55/AA as the camera's
          stock_fsm4 = {**self.eucd_tpl_fsm4, "Byte_0": stock_fsm4["Byte_0"], "ACC_BrakeRequest": 0}
        else:
          stock_fsm4 = dict(stock_fsm4)
        # b1 F9: stopping
        byte_1 = int(stock_fsm4["Byte_1"])
        stock_fsm4["Byte_1"] = (byte_1 | 0x08) if self.eucd_nt_stopping else (byte_1 & ~0x08)
        if self.eucd_nt_holding or self.eucd_nt_driver_gas:
          # The car holds (or the driver's gas): brake request released, like the camera 50 ms after the hold
          stock_fsm4 |= {"Byte_5_b7": 1, "ACC_BrakeRequest": 0, "ACC_BrakeTarget": eucd_fsm4_brake_value(0., False) & 0x3FFF}
          mode = None
      sends.append(create_eucd_fsm4(self.packer, stock_fsm4, self.eucd_accel, mode, cam_weight=self.eucd_cam_weight))
    return sends

  # ---- No-target stop (red light) ----

  @staticmethod
  def _eucd_cam_cancelled(CS):
    """Camera cancelling or aborted: slow with no target, ACC off, abort code, FSM4 FC."""
    fsm3 = CS.eucd_stock_FSM3
    if fsm3["ACC_NoTargetCancel"] or int(fsm3["Byte_6"]):
      return True
    if any(f["ACC_AbortCode"] or not f["ACC_Enabled"] for f in CS.eucd_stock_FSM0_frames):
      return True
    return any(int(f["Byte_2"]) == 0xFC for f in CS.eucd_stock_FSM4_frames)

  def _eucd_update_no_target_stop(self, CC, CS, can_sends):
    """OP stop to 0 without a camera target: state, templates, what the ECM and camera see."""
    CCP = CarControllerParams
    cancelled = self._eucd_cam_cancelled(CS)

    # Templates from the camera while it's clean and the ECM engaged
    if CS.eucd_ecm_acc_engaged and not cancelled and not self.eucd_nt_cam_aborted:
      for f in CS.eucd_stock_FSM0_frames:
        self.eucd_tpl_fsm0[int(f["Byte_0"])] = f
      self.eucd_tpl_fsm3["data" if int(CS.eucd_stock_FSM3["Byte_4"]) == 0x81 else "type"] = dict(CS.eucd_stock_FSM3)
      if CS.eucd_stock_FSM4_frames:
        self.eucd_tpl_fsm4 = CS.eucd_stock_FSM4_frames[-1]

    v = CS.out.vEgo
    allowed = CCP.EUCD_NO_TARGET_STOP and (CS.eucd_sim_red_light or not CCP.EUCD_NO_TARGET_STOP_SIM_ONLY)
    have_templates = self.eucd_tpl_fsm0 and len(self.eucd_tpl_fsm3) == 2 and self.eucd_tpl_fsm4 is not None
    # The driver's gas turns OP's long off but not the stop: ending it there gave the ECM the camera's frames
    # (not engaged) and the camera the ECM's (engaged), and the camera aborted for good (routes 896/897)
    self.eucd_nt_driver_gas = self.eucd_nt_active and CC.enabled and CS.out.gasPressed and not CC.longActive
    # Ended with the ECM still engaged (OP drops on the driver's brake first): the camera keeps seeing standby until the
    # ECM is out of it, never the ECM engaged from standby, which aborted it 0x10 for good (route 896)
    if self.eucd_nt_exit_standby_frames > 0:
      done = not CS.eucd_ecm_acc_engaged or self.eucd_nt_exit_standby_frames * DT_CTRL >= CCP.EUCD_NT_EXIT_STANDBY_MAX
      self.eucd_nt_exit_standby_frames = 0 if (done or self.eucd_nt_active) else self.eucd_nt_exit_standby_frames + 1
    if not (CC.longActive or self.eucd_nt_driver_gas) or (v > CCP.EUCD_NO_TARGET_STOP_EXIT_SPEED and not self.eucd_nt_cam_aborted):
      if self.eucd_nt_cam_standby and CS.eucd_ecm_acc_engaged:
        self.eucd_nt_exit_standby_frames = 1
      self._eucd_reset_no_target_stop()
    elif self.eucd_nt_cam_aborted and (self.eucd_nt_going or not allowed) and v > CCP.EUCD_CAM_RESYNC_SPEED:
      # Driven off after the stop, or green before stopping
      self._eucd_resync_camera(cancelled)
    elif (not self.eucd_nt_active and CC.longActive and allowed and have_templates and not cancelled and
          CS.eucd_acc_distance >= 255 and v < CCP.EUCD_NO_TARGET_STOP_ENTER_SPEED):
      # From now on the ECM gets the camera's last clean frames from OP, and the camera sees ACC standby
      self.eucd_nt_active = self.eucd_nt_cam_aborted = self.eucd_nt_cam_standby = True

    if self.eucd_nt_active:
      self.eucd_nt_cam_aborted |= cancelled
      self.eucd_nt_still_frames = self.eucd_nt_still_frames + 1 if CS.out.standstill else 0

      # The driver's gas drives off as OP's resume does
      self.eucd_nt_go_request |= self.eucd_nt_driver_gas and not self.eucd_nt_going
      if self.eucd_nt_go_request:
        self.eucd_nt_going, self.eucd_nt_go_frames, self.eucd_nt_go_moved = True, 0, False
      if self.eucd_nt_going:
        # Stopped again (or never moved): standstill sequence again, resume retried
        self.eucd_nt_go_frames += 1
        self.eucd_nt_go_moved |= not CS.out.standstill
        if CS.out.standstill and self.eucd_nt_go_moved:
          self.eucd_nt_going = False
          self.eucd_nt_still_frames = 1
        elif CS.out.standstill and self.eucd_nt_go_frames * DT_CTRL > CCP.EUCD_GO_RETRY_TIME:
          # Never moved: straight back to standstill, so the resume burst goes again
          self.eucd_nt_going = False
          self.eucd_nt_stopping = True
          self.eucd_nt_still_frames = int(CCP.EUCD_STANDSTILL_DELAY / DT_CTRL)

      # Stopping latched at the stop until the resume burst, like the camera's
      at_stop = not self.eucd_nt_going and CS.out.standstill
      stopping = not self.eucd_nt_going and ((v < CCP.EUCD_STOPPING_FLAG_SPEED and CC.actuators.accel <= 0.) or
                                             (at_stop and self.eucd_nt_stopping))
      self.eucd_nt_hold_frames = self.eucd_nt_hold_frames + 1 if (stopping and CS.out.standstill and CS.eucd_car_hold) else 0
      self.eucd_nt_stopping = stopping
      # Held until OP asks to drive after the resume, so no brake pulse at the start
      self.eucd_nt_holding = not self.eucd_nt_driver_gas and (
        self.eucd_nt_hold_frames > CCP.EUCD_HOLD_RELEASE_FRAMES or
        (self.eucd_nt_holding and self.eucd_nt_going and CS.out.standstill and CC.actuators.accel <= 0.))
      self.eucd_nt_standstill = stopping and self.eucd_nt_still_frames * DT_CTRL >= CCP.EUCD_STANDSTILL_DELAY
    self.eucd_nt_go_request = False

    # Carstate takes the ACC standstill from OP during the stop
    CS.eucd_no_target_stop = self.eucd_nt_active
    CS.eucd_op_standstill = self.eucd_nt_standstill

  def _eucd_resync_camera(self, cancelled):
    """After the drive-off: the camera re-engaged like on the driver's Set, then its frames back to the ECM."""
    CCP = CarControllerParams
    if self.eucd_nt_resync_frames == 0:
      self.eucd_cam_set_frames = CCP.EUCD_CAM_SET_FRAMES
    elif self.eucd_nt_resync_frames == CCP.EUCD_CAM_SPOOF_RELEASE_FRAMES:
      self.eucd_nt_cam_standby = False  # the camera sees the ECM go engaged
    self.eucd_nt_resync_frames += 1

    self.eucd_nt_clean_frames = 0 if (cancelled or self.eucd_nt_cam_standby) else self.eucd_nt_clean_frames + 1
    if self.eucd_nt_clean_frames >= CCP.EUCD_CAM_CLEAN_FRAMES:
      self._eucd_reset_no_target_stop()
    elif self.eucd_nt_resync_frames * DT_CTRL > CCP.EUCD_CAM_RESYNC_RETRY:
      self.eucd_nt_resync_attempts += 1
      if self.eucd_nt_resync_attempts >= CCP.EUCD_CAM_RESYNC_ATTEMPTS:
        self._eucd_reset_no_target_stop()
      else:
        # Standby again for 1 s, then Set again
        self.eucd_nt_cam_standby = True
        self.eucd_nt_resync_frames = -int(1.0 / DT_CTRL)

  def _eucd_reset_no_target_stop(self):
    self.eucd_nt_active = self.eucd_nt_cam_aborted = self.eucd_nt_going = False
    self.eucd_nt_standstill = self.eucd_nt_stopping = self.eucd_nt_holding = self.eucd_nt_driver_gas = False
    self.eucd_nt_still_frames = self.eucd_nt_hold_frames = 0
    self.eucd_nt_cam_standby = False
    self.eucd_nt_resync_frames = self.eucd_nt_resync_attempts = self.eucd_nt_clean_frames = 0

  def _eucd_nt_fsm3(self, CS):
    """FSM3 base: the camera's, or its clean template (same data/type cadence) once it cancelled."""
    if not self.eucd_nt_cam_aborted:
      return CS.eucd_stock_FSM3
    kind = "data" if int(CS.eucd_stock_FSM3["Byte_4"]) == 0x81 else "type"
    return self.eucd_tpl_fsm3[kind]

  def _eucd_longitudinal(self, CC, CS):
    """Accel and mode, FSM1/FSM3 at 50 Hz."""
    if CS.eucd_stock_FSM4_frames:
      self.eucd_cam_brake = bool(CS.eucd_stock_FSM4_frames[-1]["ACC_BrakeRequest"])
    if CC.longActive:
      accel, long_mode = self._eucd_long_decision(CC, CS)
    else:
      accel = float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"])
      self.eucd_long_mode = LongMode.COAST
      self.eucd_cam_brake_honor = False
      self._eucd_update_cam_stop(CC, CS)
      self.eucd_cam_relay = False
      self.eucd_cam_weight = 0.
      long_mode = None
      if self.eucd_nt_driver_gas:
        # The ECM drives on the driver's gas: no brake nor traction from OP (the template's mode is stale)
        accel, long_mode = 0., LongMode.COAST
    if self.eucd_nt_holding:
      # The car holds the stop: the camera's standstill request (route 807)
      accel = CarControllerParams.EUCD_HOLD_ACCEL
      long_mode = self.eucd_long_mode = LongMode.BRAKE
    self.eucd_accel = accel

    self.eucd_spoof_cam_ack = (CC.longActive and not self.eucd_cam_relay and self.eucd_cam_brake and not self.eucd_cam_brake_honor and
                               self.eucd_long_mode != LongMode.BRAKE and not self.eucd_nt_active)

    stock_fsm3 = self._eucd_nt_fsm3(CS)
    if self.eucd_sng_ack_frames > 0:
      acc_check = 1
      self.eucd_sng_ack_frames -= 1
    else:
      acc_check = int(stock_fsm3["ACC_Check"])

    if self.eucd_nt_active:
      # OP's mode as is (the template's mode is stale), standstill from OP, camera cancel masked
      return [
        create_eucd_radar(self.packer, CS.eucd_stock_FSM1, long_mode),
        create_eucd_longitudinal(self.packer, stock_fsm3, accel, acc_check, long_mode, standstill=self.eucd_nt_standstill),
      ]
    return [
      create_eucd_radar(self.packer, CS.eucd_stock_FSM1),
      create_eucd_longitudinal(self.packer, stock_fsm3, accel, acc_check, self._eucd_fsm3_mode(CS, long_mode)),
    ]

  def _eucd_long_decision(self, CC, CS):
    """Accel and mode with OP long active."""
    CCP = CarControllerParams
    actuators = CC.actuators
    accel = min(max(actuators.accel, CCP.EUCD_ACCEL_MIN), CCP.EUCD_ACCEL_MAX)
    accel = self._eucd_jerk_limit(accel, self.eucd_accel)
    cam_accel = float(CS.eucd_stock_FSM3["ACC_AccelerationRequest"])
    accel = self._eucd_camera_accel_cap(CS, accel, cam_accel)
    accel = self._eucd_camera_brake(accel, cam_accel)
    brake_offset = self._eucd_slope_brake_offset(CC)

    self._eucd_update_cam_stop(CC, CS)
    op_brakes_more = (actuators.longControlState == LongCtrlState.pid and
                      ((not self.eucd_cam_brake and cam_accel >= 0. and accel < CCP.EUCD_OP_BRAKE_OVERRIDE_ACCEL) or
                       accel < cam_accel - CCP.EUCD_OP_BRAKE_OVERRIDE_MARGIN))
    w = 0. if op_brakes_more else self.eucd_cam_stop_w
    self.eucd_cam_relay = w >= 1.
    self.eucd_cam_weight = w

    if self.eucd_cam_relay:
      cam_mode = int(CS.eucd_stock_FSM3["ACC_Mode"])
      self.eucd_long_mode = LongMode(cam_mode) if cam_mode <= LongMode.DRIVE else LongMode.COAST
      self.eucd_cam_brake_honor = self.eucd_cam_brake
      return cam_accel, None

    if w > 0.:
      accel = accel + w * (cam_accel - accel)
      self.eucd_long_mode = LongMode.BRAKE if self.eucd_cam_brake else eucd_next_long_mode(self.eucd_long_mode, accel, brake_offset)
      self.eucd_cam_brake_honor = self.eucd_cam_brake
      return accel, self.eucd_long_mode

    if self.eucd_cam_brake_honor:
      self.eucd_long_mode = LongMode.BRAKE
    else:
      mode = eucd_next_long_mode(self.eucd_long_mode, accel, brake_offset)
      if mode == LongMode.BRAKE and self._eucd_coast_tolerance(CC, CS, accel, brake_offset, self.eucd_long_mode == LongMode.BRAKE):
        mode = LongMode.COAST
      self.eucd_long_mode = mode
    return accel, self.eucd_long_mode

  def _eucd_jerk_limit(self, accel, last_accel):
    """Ramp OP's accel request (jerk limits)."""
    CCP = CarControllerParams
    dt = self.EUCD_LONG_TX_PERIOD_NANOS * 1e-9
    down = CCP.EUCD_JERK_DOWN_LIFT if last_accel > 0. else CCP.EUCD_JERK_DOWN_BRAKE
    return min(max(accel, last_accel - down * dt), last_accel + CCP.EUCD_JERK_UP * dt)

  # ---- Camera rules ----

  @staticmethod
  def _eucd_camera_accel_cap(CS, accel, cam_accel):
    """No more accel than the camera's while its target is close."""
    CCP = CarControllerParams
    close = max(CS.out.vEgo * CCP.EUCD_CAMERA_ACCEL_CAP_GAP, CCP.EUCD_CAMERA_ACCEL_CAP_MIN_DIST)
    if CS.eucd_acc_distance >= min(close, 255):
      return accel
    return min(accel, max(cam_accel, CCP.EUCD_CAMERA_ACCEL_CAP_FLOOR))

  def _eucd_camera_brake(self, accel, cam_accel):
    """Follow the camera's hard brake, latched until it releases."""
    if not self.eucd_cam_brake:
      self.eucd_cam_brake_honor = False
    elif cam_accel < CarControllerParams.EUCD_CAMERA_BRAKE_HONOR_BELOW:
      self.eucd_cam_brake_honor = True
    return min(accel, cam_accel) if self.eucd_cam_brake_honor else accel

  @staticmethod
  def _eucd_fsm3_mode(CS, long_mode):
    """FSM3 mode: 0 when the camera is in it, else OP's."""
    if long_mode is None:
      return None
    if int(CS.eucd_stock_FSM3["ACC_Mode"]) == LongMode.BRAKE:
      return LongMode.BRAKE
    return long_mode

  @staticmethod
  def _eucd_coast_tolerance(CC, CS, accel, brake_offset, braking=False):
    """Downhill band over the ACC set speed: coast up to set + tolerance, brake back to set."""
    CCP = CarControllerParams
    v, set_speed = CS.out.vEgo, CC.hudControl.setSpeed
    lead_close = CS.eucd_acc_distance < 255 and CS.eucd_acc_distance < v * CCP.EUCD_COAST_TOLERANCE_LEAD_GAP
    if brake_offset <= 0. or lead_close or accel <= CCP.EUCD_COAST_TOLERANCE_MAX_DECEL:
      return False
    top = set_speed if braking else set_speed + CCP.EUCD_COAST_TOLERANCE
    return set_speed - CCP.EUCD_COAST_TOLERANCE_AT_SET < v < top

  @staticmethod
  def _eucd_slope_brake_offset(CC):
    """Slope part of the coast accel from pitch."""
    if len(CC.orientationNED) != 3:
      return 0.
    CCP = CarControllerParams
    return min(-CCP.EUCD_COAST_PITCH_GAIN * math.sin(CC.orientationNED[1]), CCP.EUCD_DOWNHILL_BRAKE_OFFSET_MAX)

  def _eucd_update_cam_stop(self, CC, CS):
    """Camera final stop: state and blend weight."""
    CCP = CarControllerParams
    lead = CS.eucd_acc_distance < 255
    v = CS.out.vEgo
    if not (CCP.EUCD_CAMERA_STOPS_BEHIND_LEAD and CC.longActive) or self.eucd_nt_active:
      self.eucd_cam_stop = False
      if not CC.longActive:
        self.eucd_cam_stop_w = 0.
    elif not self.eucd_cam_stop and lead and v < CCP.EUCD_CAM_STOP_HANDOVER_SPEED:
      self.eucd_cam_stop = True
    elif self.eucd_cam_stop and (v > CCP.EUCD_CAM_STOP_RETURN_SPEED or (not lead and v > 1.)):
      self.eucd_cam_stop = False
    step = (self.EUCD_LONG_TX_PERIOD_NANOS * 1e-9) / CCP.EUCD_CAM_STOP_TRANSITION_TIME
    self.eucd_cam_stop_w = min(max(self.eucd_cam_stop_w + (step if self.eucd_cam_stop else -step), 0.), 1.)

  def _eucd_relay_brake_acks(self, CC, CS):
    """Brake acks to the camera: masked or spoofed."""
    CCP = CarControllerParams
    if self.eucd_cam_brake and not self.eucd_nt_active:
      self.eucd_cam_mask_frames = 0
    elif (CC.longActive and self.eucd_long_mode == LongMode.BRAKE and not self.eucd_cam_relay) or self.eucd_nt_active:
      # No-target stop: the camera never asked, so it never sees the car brake or hold
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

    cam_standby = self.eucd_nt_cam_standby or self.eucd_nt_exit_standby_frames > 0

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
    # EngineData, 0x325, Brake_Info and ACC_Speed always relayed (the panda blocks them); ACC standby while the camera is shown it
    for stock_engine_data in CS.eucd_stock_EngineData_frames:
      sends.append(create_eucd_cam_engine_data(self.packer, stock_engine_data, cam_standby))
    for stock_acc_hold in CS.eucd_stock_AccHold_frames:
      sends.append(create_eucd_cam_acc_hold(self.packer, stock_acc_hold, cam_standby))
    for stock_brake_info in CS.eucd_stock_Brake_Info_frames:
      sends.append(create_eucd_cam_brake_info(self.packer, stock_brake_info, cam_standby))
    for stock_acc_speed in CS.eucd_stock_ACC_Speed_frames:
      sends.append(create_eucd_cam_acc_speed(self.packer, stock_acc_speed, cam_standby))
    # CCButtons always relayed (the panda blocks them), with Set held while re-engaging the camera
    set_plus = self.eucd_cam_set_frames > 0
    self.eucd_cam_set_frames = max(self.eucd_cam_set_frames - 1, 0)
    for stock_ccbuttons in CS.eucd_stock_CCButtons_frames:
      sends.append(create_eucd_cam_button(self.packer, stock_ccbuttons, set_plus))
    # 0x1FF b0 held from the stop on, while stopped in the no-target stop
    hold_mask = self.eucd_nt_active and CS.out.standstill
    for stock_brake_hold in CS.eucd_stock_BrakeHold_frames:
      if not hold_mask or self.eucd_brake_hold_b0 is None:
        self.eucd_brake_hold_b0 = int(stock_brake_hold["Byte_0"])
      sends.append(create_eucd_cam_brake_hold(self.packer, stock_brake_hold, self.eucd_brake_hold_b0 if hold_mask else None))
    return sends

  def _update_c1(self, CC, CS):
    can_sends = []
    actuators = CC.actuators

    if self.frame % 2 == 0:  # stock FSM1 and PSCM1 are 50 Hz
      requested_active = CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed
      recovering = requested_active and self.frame < self.c1_recovery_until

      if not requested_active:
        self.c1_torque_samples.clear()
        self.c1_recovery_until = -1
      elif recovering:
        self.c1_torque_samples.clear()
      else:
        if self.c1_recovery_until >= 0:
          self.c1_recovery_until = -1
          self.c1_torque_samples.clear()
        self.c1_torque_samples.append(CS.c1_lka_torque)
        if (len(self.c1_torque_samples) == CarControllerParams.C1_N_ZERO_TORQUE and
            all(torque == 0 for torque in self.c1_torque_samples)):
          self.c1_recovery_until = self.frame + 100
          self.c1_torque_samples.clear()
          recovering = True

      lat_active = requested_active and not recovering
      desired_angle = float(np.clip(
        actuators.steeringAngleDeg,
        CS.out.steeringAngleDeg - CarControllerParams.C1_ANGLE_ERROR,
        CS.out.steeringAngleDeg + CarControllerParams.C1_ANGLE_ERROR,
      ))
      apply_angle = apply_std_steer_angle_limits(
        desired_angle, self.apply_angle_last, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, lat_active, CarControllerParams.C1_ANGLE_LIMITS,
      )

      can_sends.append(create_c1_pscm_message(self.packer, CS.c1_msg_pscm))
      can_sends.append(create_c1_steering_control(self.packer, apply_angle, lat_active))
      self.apply_angle_last = apply_angle

    if CC.cruiseControl.cancel and self.frame % 10 == 0:
      can_sends.append(create_c1_cancel(self.packer))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
