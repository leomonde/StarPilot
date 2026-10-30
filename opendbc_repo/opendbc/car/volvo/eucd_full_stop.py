from opendbc.car import DT_CTRL
from opendbc.car.volvo.values import CarControllerParams
from opendbc.car.volvo.volvocan import eucd_fsm4_brake_value


def eucd_cam_cancelled(CS) -> bool:
  """Camera cancelling or aborted: slow with no target, ACC off, abort code, FSM4 FC."""
  fsm3 = CS.eucd_stock_FSM3
  if fsm3["ACC_NoTargetCancel"] or int(fsm3["Byte_6"]):
    return True
  if any(f["ACC_AbortCode"] or not f["ACC_Enabled"] for f in CS.eucd_stock_FSM0_frames):
    return True
  return any(int(f["Byte_2"]) == 0xFC for f in CS.eucd_stock_FSM4_frames)


class EucdFullStop:
  """OP's stop to 0 on the EUCD, replaying the camera's own stop (route 807).

  As the Ford/Toyota ports, the planner decides (lead, red light, stop sign, obstacle) and this follows its accel and
  cruiseControl.resume. Below the enter speed, braking, OP owns the ECM with the camera's last clean FSM0/FSM3/FSM4,
  and the camera is shown the driver's side of it: ACC standby, so it never cancels slow with no target (routes
  827/895) nor aborts. After the drive-off it is re-engaged like on the driver's Set (route 885).
  """

  def __init__(self):
    self.active = False  # OP owns the ECM: FSM0/FSM3/FSM4 from the camera's templates
    self.cam_standby = False  # camera shown ACC standby (EngineData, 0x325, 0x220)
    self.exit_standby_frames = 0  # ended (driver's brake, gas, cancel) with the ECM still engaged: still standby
    self.stopped_frames = 0
    self.car_hold_frames = 0
    self.stopping = False  # FSM4 b1 F9
    self.holding = False  # the car holds the stop: brake request released
    self.standstill = False  # FSM3 ACC_Standstill to the ECM, and the cruise standstill carstate reports
    self.driving_off = False  # resume sent, standstill released
    self.drive_off_request = False  # set with the resume burst, taken on the next frame
    self.drive_off_frames = 0
    self.drive_off_moved = False
    self.resync_frames = 0  # frames re-engaging the camera after the drive-off
    self.resync_attempts = 0
    self.cam_clean_frames = 0
    self.cam_set_frames = 0  # Set press to the camera, frames left
    # Camera's last clean frames: FSM0 by Byte_0, FSM3 data/type, FSM4
    self.tpl_fsm0: dict[int, dict] = {}
    self.tpl_fsm3: dict[str, dict] = {}
    self.tpl_fsm4: dict | None = None

  @property
  def show_cam_standby(self) -> bool:
    return self.cam_standby or self.exit_standby_frames > 0

  def update(self, CC, CS):
    """State, templates, and when OP takes and gives back the ECM."""
    CCP = CarControllerParams
    cancelled = eucd_cam_cancelled(CS)

    # Templates from the camera while it's clean and the ECM engaged
    if CS.eucd_ecm_acc_engaged and not cancelled and not self.active:
      for f in CS.eucd_stock_FSM0_frames:
        self.tpl_fsm0[int(f["Byte_0"])] = f
      self.tpl_fsm3["data" if int(CS.eucd_stock_FSM3["FrameKind"]) == 0x81 else "type"] = dict(CS.eucd_stock_FSM3)
      if CS.eucd_stock_FSM4_frames:
        self.tpl_fsm4 = CS.eucd_stock_FSM4_frames[-1]

    v = CS.out.vEgo
    have_templates = self.tpl_fsm0 and len(self.tpl_fsm3) == 2 and self.tpl_fsm4 is not None
    # Ended by the driver (brake, gas, cancel) with the ECM still engaged: the ECM gets the camera's frames (standby)
    # and drops, the camera keeps seeing standby until then, never the ECM engaged from standby, which aborted it 0x10
    # for good (route 896). The gas ends it on purpose: the panda gives the ECM the camera's FSM3 while it is pressed
    # (gas has priority), so the ACC goes to standby and the driver re-engages it later (route 898, 357.7 s)
    if self.exit_standby_frames > 0:
      done = not CS.eucd_ecm_acc_engaged or self.exit_standby_frames * DT_CTRL >= CCP.EUCD_FULL_STOP_EXIT_STANDBY_MAX
      self.exit_standby_frames = 0 if (done or self.active) else self.exit_standby_frames + 1
    if not CC.longActive:
      if self.cam_standby and CS.eucd_ecm_acc_engaged:
        self.exit_standby_frames = 1
      self.reset()
    elif self.active and v > CCP.EUCD_CAM_RESYNC_SPEED:
      # Driven off after the stop, or the planner did not stop after all
      self._resync_camera(cancelled)
    elif (not self.active and CCP.EUCD_FULL_STOP and have_templates and not cancelled and
          v < CCP.EUCD_FULL_STOP_ENTER_SPEED and CC.actuators.accel < 0.):
      # OP braking slow, lead or not: from now on the ECM gets the camera's last clean frames from OP, and the
      # camera sees ACC standby
      self.active = self.cam_standby = True

    if self.active:
      self._update_stop(CC, CS)
    self.drive_off_request = False

  def _update_stop(self, CC, CS):
    """Stopping, the car's hold, standstill and the drive-off, as the camera does them."""
    CCP = CarControllerParams
    standstill = CS.out.standstill
    self.stopped_frames = self.stopped_frames + 1 if standstill else 0

    if self.drive_off_request:
      self.driving_off, self.drive_off_frames, self.drive_off_moved = True, 0, False
    if self.driving_off:
      # Stopped again (or never moved): standstill sequence again, resume retried
      self.drive_off_frames += 1
      self.drive_off_moved |= not standstill
      if standstill and self.drive_off_moved:
        self.driving_off = False
        self.stopped_frames = 1
      elif standstill and self.drive_off_frames * DT_CTRL > CCP.EUCD_GO_RETRY_TIME:
        # Never moved: straight back to standstill, so the resume burst goes again
        self.driving_off = False
        self.stopping = True
        self.stopped_frames = int(CCP.EUCD_STANDSTILL_DELAY / DT_CTRL)

    # Stopping latched at the stop until the resume burst, like the camera's
    at_stop = not self.driving_off and standstill
    stopping = not self.driving_off and ((CS.out.vEgo < CCP.EUCD_STOPPING_FLAG_SPEED and CC.actuators.accel <= 0.) or
                                         (at_stop and self.stopping))
    self.car_hold_frames = self.car_hold_frames + 1 if (stopping and standstill and CS.eucd_car_hold) else 0
    self.stopping = stopping
    # Held until OP asks to drive after the resume, so no brake pulse at the start
    self.holding = (self.car_hold_frames > CCP.EUCD_HOLD_RELEASE_FRAMES or
                    (self.holding and self.driving_off and standstill and CC.actuators.accel <= 0.))
    self.standstill = stopping and self.stopped_frames * DT_CTRL >= CCP.EUCD_STANDSTILL_DELAY

  def _resync_camera(self, cancelled):
    """After the drive-off: the camera re-engaged like on the driver's Set, then its frames back to the ECM."""
    CCP = CarControllerParams
    if self.resync_frames == 0:
      self.cam_set_frames = CCP.EUCD_CAM_SET_FRAMES
    elif self.resync_frames == CCP.EUCD_CAM_SPOOF_RELEASE_FRAMES:
      self.cam_standby = False  # the camera sees the ECM go engaged
    self.resync_frames += 1

    self.cam_clean_frames = 0 if (cancelled or self.cam_standby) else self.cam_clean_frames + 1
    if self.cam_clean_frames >= CCP.EUCD_CAM_CLEAN_FRAMES:
      self.reset()
    elif self.resync_frames * DT_CTRL > CCP.EUCD_CAM_RESYNC_RETRY:
      self.resync_attempts += 1
      if self.resync_attempts >= CCP.EUCD_CAM_RESYNC_ATTEMPTS:
        self.reset()
      else:
        # Standby again for 1 s, then Set again
        self.cam_standby = True
        self.resync_frames = -int(1.0 / DT_CTRL)

  def reset(self):
    self.active = self.driving_off = False
    self.standstill = self.stopping = self.holding = False
    self.stopped_frames = self.car_hold_frames = 0
    self.cam_standby = False
    self.resync_frames = self.resync_attempts = self.cam_clean_frames = 0

  def drive_off_ready(self, CS) -> bool:
    """The resume burst may go: the ECM ready to drive off from its standstill (or it never said so in time)."""
    if not self.active or CS.eucd_ecm_standstill_ready:
      return True
    CCP = CarControllerParams
    return self.stopped_frames * DT_CTRL >= CCP.EUCD_STANDSTILL_DELAY + CCP.EUCD_ECM_READY_TIMEOUT

  def take_cam_set(self) -> bool:
    """Set held on the camera's CCButtons this frame."""
    set_plus = self.cam_set_frames > 0
    self.cam_set_frames = max(self.cam_set_frames - 1, 0)
    return set_plus

  # ---- What the ECM gets while OP owns it ----

  def fsm0(self, stock_fsm0):
    """FSM0 to the ECM: the camera's abort (ACC off, code 0x10) kept from it."""
    if not self.active:
      return stock_fsm0
    return self.tpl_fsm0.get(int(stock_fsm0["Byte_0"]), next(iter(self.tpl_fsm0.values())))

  def fsm3(self, CS):
    """FSM3 base: the camera's, or its clean template (same data/type cadence)."""
    if not self.active:
      return CS.eucd_stock_FSM3
    kind = "data" if int(CS.eucd_stock_FSM3["FrameKind"]) == 0x81 else "type"
    return self.tpl_fsm3[kind]

  def fsm4(self, stock_fsm4):
    """FSM4 to the ECM, and whether OP's mode applies: the camera's abort (b2 FC) kept from it, stopping flag, brake
    request released while the car holds."""
    # b0 alternates 55/AA as the camera's
    fsm4 = {**self.tpl_fsm4, "Byte_0": stock_fsm4["Byte_0"], "ACC_BrakeRequest": 0}
    # b1 F1 -> F9: stopping
    fsm4["ACC_Stopping"] = int(self.stopping)
    if self.holding:
      # The car holds: brake request released, like the camera 50 ms after the hold
      fsm4 |= {"Byte_5_b7": 1, "ACC_BrakeRequest": 0, "ACC_BrakeTarget": eucd_fsm4_brake_value(0., False) & 0x3FFF}
      return fsm4, False
    return fsm4, True
