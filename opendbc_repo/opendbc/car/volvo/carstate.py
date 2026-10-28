import copy

from cereal import custom
from opendbc.car import Bus, ButtonType, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CAR, CANBUS, CarControllerParams, DBC, EcmAccState, VolvoC1PlatformConfig, VolvoEUCDPlatformConfig, VolvoSPAPlatformConfig

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

# Main-bus SPEED (0x60) raw counts to m/s, matches VOLVO_SPEED_TO_MS in volvo.h
SPEED_TO_MS = 0.003977
STEERING_PRESSED_THRESHOLD = 2


def eucd_all_frames(cp: CANParser, name: str) -> list[dict[str, float]]:
  sigs = cp.vl_all[name]
  return [dict(zip(sigs.keys(), vals, strict=True)) for vals in zip(*sigs.values(), strict=True)]


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    self.is_c1 = isinstance(CAR(CP.carFingerprint).config, VolvoC1PlatformConfig)
    self.is_spa = isinstance(CAR(CP.carFingerprint).config, VolvoSPAPlatformConfig)
    self.is_eucd = isinstance(CAR(CP.carFingerprint).config, VolvoEUCDPlatformConfig)
    self.gas_pressed_prev = False
    self.dispatch_lca_2_msg = False
    self.msg_pscm = {}
    self.msg_lca = {}
    self.msg_lca_2 = {}
    self.msg_lca_3 = {}
    self.msg_gear_position = {}
    self.pilot_assist_engaged = False
    self.msg_lca_5 = {}  # formerly msg_speed_1
    self.msg_speed = {}
    self.msg_speed_2 = {}
    self.msg_0x1a = {}
    self.msg_egsm = {}
    self.msg_pscm_related = {}
    self.msg_lca_4 = {}
    self.msg_lca_6 = {}
    self.msg_lca_7 = {}
    self.c1_msg_pscm = {}
    self.c1_lka_torque = 0
    self.c1_button_states = {
      "ACCOnOffBtn": False,
      "ACCStopBtn": False,
      "ACCSetBtn": False,
      "ACCResumeBtn": False,
      "ACCMinusBtn": False,
      "TimeGapIncreaseBtn": False,
      "TimeGapDecreaseBtn": False,
    }
    # EUCD (V60) state
    self.eucd_cruise_enabled_prev = False
    self.eucd_eps_torque_timer = 0
    self._eucd_cruise_speed_prev_kph = 0
    self._eucd_pending_delta = 0
    self._eucd_custom_acc_step = 1
    self.eucd_gap_button_states = {"TimeGapIncreaseBtn": False, "TimeGapDecreaseBtn": False}
    self.eucd_sim_red_light = False
    self.eucd_start_stop_on: bool | None = None  # debounced start/stop state
    self.eucd_start_stop_frames = 0  # frames the start/stop differs from it
    # Set by the carcontroller: OP's no-target stop owns the ACC standstill
    self.eucd_no_target_stop = False
    self.eucd_op_standstill = False

  def update(self, can_parsers, starpilot_toggles) -> structs.CarState:
    if self.is_c1:
      return self._update_c1(can_parsers)
    if self.is_eucd:
      return self._update_eucd(can_parsers)

    cp_main = can_parsers[Bus.main]
    cp_pt = can_parsers[Bus.pt]
    cp_party = can_parsers[Bus.party]
    ret = structs.CarState()

    # Car speed from the main-bus SPEED
    ret.vEgoRaw = cp_main.vl["SPEED"]["SPEED"] * SPEED_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw <= 0.1  # m/s

    # Gas: CMA raw 0-255, SPA percent; thresholds match volvo.h
    if self.is_spa:
      ret.gasPressed = cp_pt.vl["ECM_1"]["ACCELERATOR_PEDAL_POS"] > 1.0  # percent
    else:
      ret.gasPressed = cp_pt.vl["ECM_1"]["ACCELERATOR_PEDAL_POS"] > 20+1  # raw counts, 20 baseline + 1

    # brake
    #ret.brakePressed = bool(cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_A"] or cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_B"])
    # BRAKE_PEDAL_PRESSED_A at first touch, _B with the brake light
    ret.brakePressed = cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_B"] == 1
    ret.parkingBrake = False  # TODO parking brake

    # ESC active
    ret.espActive = cp_main.vl["LCA_2"]["ESC_ACTUATING"] == 1 and cp_main.vl["LCA_2"]["ESC_ELIGIBLE"] == 1

    # steering wheel
    ret.steeringAngleDeg = cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR']  # negative for a right turn
    #ret.steeringAngleDeg = cp_party.vl['SAS']['SAS_ANGLE_SENSOR']

    ret.steeringTorque = -cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_INPUT']  # openpilot right turn is positive
    driver_input = abs(cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_INPUT'])
    ret.steeringPressed = driver_input > STEERING_PRESSED_THRESHOLD

    # EPS status placeholder
    self.eps_active = True  # assumed active

    if self.is_spa:
      # SPA: byte 0 bit 1, inverted
      cruise_raw = cp_pt.vl["BUS1_CRUISE_CONTROL"]["CRUISE_CONTROL_SPA_ENABLED"] == 1
    else:
      # CMA: two booleans
      cruise_raw = cp_pt.vl["BUS1_CRUISE_CONTROL"]["CRUISE_CONTROL_ENABLED"] == 1 or cp_pt.vl["BUS1_CRUISE_CONTROL"]["CRUISE_CONTROL_ENABLED_IDLE_TRAFFIC"] == 1

    ret.cruiseState.enabled = cruise_raw

    self.gas_pressed_prev = ret.gasPressed
    ret.cruiseState.available = True  # TODO real availability
    ret.cruiseState.speed = 0  # TODO set speed
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = ret.standstill  # TODO cruise standstill

    # gear
    gearPosition = cp_main.vl['GEAR_POSITION']['GEAR_POSITION']  # 0 P, 1 R, 2 N, 3 D, 4 B
    if gearPosition == 0:
      ret.gearShifter = GearShifter.park
    elif gearPosition == 1:
      ret.gearShifter = GearShifter.reverse
    elif gearPosition == 2:
      ret.gearShifter = GearShifter.neutral
    elif gearPosition == 3:
      ret.gearShifter = GearShifter.drive
    elif gearPosition == 4:
      ret.gearShifter = GearShifter.drive

    # Blinkers TODO (FlexRay)
    ret.leftBlinker = False
    ret.rightBlinker = False

    # Lock info TODO (FlexRay)
    ret.doorOpen = False  # TODO door open
    ret.seatbeltUnlatched = False  # TODO seatbelt

    # Store the message dicts
    self.msg_pscm = cp_party.vl['PSCM']
    self.msg_lca = cp_main.vl['LCA']
    self.msg_lca_2 = cp_main.vl['LCA_2']
    self.msg_lca_3 = cp_main.vl['LCA_3']
    self.msg_lca_4 = cp_main.vl['LCA_4']
    self.msg_lca_5 = cp_main.vl['LCA_5']
    self.msg_lca_6 = cp_main.vl['LCA_6']
    self.msg_lca_7 = cp_main.vl['LCA_7']
    self.msg_speed = cp_main.vl['SPEED']
    self.msg_speed_2 = cp_main.vl['SPEED_2']
    self.msg_gear_position = cp_main.vl['GEAR_POSITION']
    self.msg_egsm = cp_party.vl['EGSM']
    self.msg_pscm_related = cp_party.vl['PSCM_RELATED']

    self.pilot_assist_engaged = cp_main.vl['LCA_2']['PILOT_ASSIST_ENGAGED'] == 1

    fp_ret = custom.StarPilotCarState.new_message()
    return ret, fp_ret

  def _update_c1(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    ret.vEgoRaw = cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringTorque = cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = False

    ret.gasPressed = cp.vl["PedalandBrake"]["AccPedal"] > 5.0
    ret.brakePressed = bool(cp.vl["PedalandBrake"]["BrakePedalActive2"] or
                            cp.vl["PedalandBrake"]["BrakePedalActive"])

    ret.gearShifter = {
      0: GearShifter.park,
      1: GearShifter.reverse,
      2: GearShifter.neutral,
      3: GearShifter.drive,
    }.get(int(cp.vl["TCM0"]["GearShifter"]), GearShifter.unknown)

    ret.cruiseState.available = bool(cp_cam.vl["FSM0"]["ACCStatusOnOff"])
    ret.cruiseState.enabled = bool(cp_cam.vl["FSM0"]["ACCStatusActive"])
    ret.cruiseState.speed = cp.vl["ACC"]["SpeedTargetACC"] * CV.KPH_TO_MS
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = ret.standstill

    turn_signal = int(cp.vl["MiscCarInfo"]["TurnSignal"])
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
      50, turn_signal == 1, turn_signal == 3)
    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    button_types = {
      "ACCOnOffBtn": ButtonType.mainCruise,
      "ACCStopBtn": ButtonType.cancel,
      "ACCSetBtn": ButtonType.setCruise,
      "ACCResumeBtn": ButtonType.resumeCruise,
      "ACCMinusBtn": ButtonType.decelCruise,
      "TimeGapIncreaseBtn": ButtonType.gapAdjustCruise,
      "TimeGapDecreaseBtn": ButtonType.gapAdjustCruise,
    }
    button_events = []
    for signal, button_type in button_types.items():
      pressed = bool(cp.vl["CCButtons"][signal])
      button_events.extend(create_button_events(pressed, self.c1_button_states[signal], {True: button_type}))
      self.c1_button_states[signal] = pressed
    ret.buttonEvents = button_events

    self.c1_msg_pscm = cp.vl["PSCM1"]
    self.c1_lka_torque = int(cp.vl["PSCM1"]["LKATorque"])

    fp_ret = custom.StarPilotCarState.new_message()
    return ret, fp_ret

  def _update_eucd(self, can_parsers):
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]

    ret = structs.CarState()

    # car speed
    ret.vEgoRaw = pt_cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gasPressed = pt_cp.vl["AccPedal"]["AccPedal"] >= 10

    # brake pedal
    ret.brakePressed = pt_cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = pt_cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringRateDeg = pt_cp.vl["SAS0"]["SteeringRateOfChange"]
    eucd_steering_direction = pt_cp.vl["SAS0"]["SteeringDirection"]
    ret.steeringTorque = pt_cp.vl["PSCM1"]["EPSTorque"]
    if eucd_steering_direction:
      ret.steeringTorque = -abs(ret.steeringTorque)
    ret.steeringTorqueEps = pt_cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    # cruise state
    ret.cruiseState.speed = pt_cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.cruiseState.available = bool(cam_cp.vl["FSM0"]["ACC_Available"])
    # ECM is the ACC master
    self.eucd_ecm_acc_engaged = int(pt_cp.vl["EngineData"]["ACC_State_ECM"]) == EcmAccState.ENGAGED
    ret.cruiseState.enabled = self.eucd_ecm_acc_engaged
    # Camera standstill hold, gated by the car's standstill; OP's own in a no-target stop
    if self.eucd_no_target_stop:
      ret.cruiseState.standstill = self.eucd_op_standstill and ret.standstill
    else:
      ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"]) and ret.standstill
    # The car holds the stop (BrakeReqAck b0 bit7, route 807)
    self.eucd_car_hold = bool(int(pt_cp.vl["BrakeReqAck"]["Byte_0"]) & 0x80)
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False
    self.eucd_acc_distance = cam_cp.vl["FSM1"]["ACC_Distance"]

    # Servo timeout while ACC is active
    if ret.cruiseState.enabled and ret.vEgo > self.CP.minSteerSpeed:
      if not self.eucd_cruise_enabled_prev:
        self.eucd_eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eucd_eps_torque_timer += 1
      else:
        self.eucd_eps_torque_timer = 0

      ret.steerFaultTemporary = self.eucd_eps_torque_timer >= CarControllerParams.EUCD_STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.eucd_cruise_enabled_prev = ret.cruiseState.enabled

    # Gear: reverse decoded, else drive
    if pt_cp.vl["Gear_Info"]["Gear"] == 44 and ret.vEgo < 3:
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # One accelCruise/decelCruise per frame from the pending delta
    cruise_kph_now = round(ret.cruiseState.speed * CV.MS_TO_KPH)
    if ret.cruiseState.enabled and self.eucd_cruise_enabled_prev:
      self._eucd_pending_delta += cruise_kph_now - self._eucd_cruise_speed_prev_kph
    self._eucd_cruise_speed_prev_kph = cruise_kph_now

    # One event per ACC step
    step = max(1, self._eucd_custom_acc_step)
    button_events = []
    if self._eucd_pending_delta >= 1:
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.accelCruise))
      self._eucd_pending_delta = max(0, self._eucd_pending_delta - step)
    elif self._eucd_pending_delta <= -1:
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise))
      self._eucd_pending_delta = min(0, self._eucd_pending_delta + step)

    # Gap buttons cycle the personality (bit set, inverted copy cleared)
    for signal in self.eucd_gap_button_states:
      pressed = bool(pt_cp.vl["CCButtons"][signal]) and not pt_cp.vl["CCButtons"][signal + "Inv"]
      button_events.extend(create_button_events(pressed, self.eucd_gap_button_states[signal], {True: ButtonType.gapAdjustCruise}))
      self.eucd_gap_button_states[signal] = pressed
    ret.buttonEvents = button_events

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    self.eucd_pscm_stock_values = pt_cp.vl["PSCM1"]
    self.eucd_stock_FSM1 = copy.copy(cam_cp.vl["FSM1"])
    self.eucd_stock_FSM3 = copy.copy(cam_cp.vl["FSM3"])
    # Frames carcontroller relays 1:1
    self.eucd_stock_FSM0_frames = eucd_all_frames(cam_cp, "FSM0")
    self.eucd_stock_FSM4_frames = eucd_all_frames(cam_cp, "FSM4")
    self.eucd_stock_BrakeLevel_frames = eucd_all_frames(pt_cp, "BrakeLevel")
    self.eucd_stock_BrakeStatus_frames = eucd_all_frames(pt_cp, "BrakeStatus")
    self.eucd_stock_wheelspeed1_frames = eucd_all_frames(pt_cp, "wheelspeed1")
    self.eucd_stock_BrakeReqAck_frames = eucd_all_frames(pt_cp, "BrakeReqAck")
    self.eucd_stock_BrakeHold_frames = eucd_all_frames(pt_cp, "BrakeHold")
    self.eucd_stock_EngineData_frames = eucd_all_frames(pt_cp, "EngineData")
    self.eucd_stock_AccHold_frames = eucd_all_frames(pt_cp, "AccHold")
    self.eucd_stock_CCButtons = copy.copy(pt_cp.vl["CCButtons"])

    fp_ret = custom.StarPilotCarState.new_message()
    # TSR speed limit, 0 = no sign
    tsr_kph = cam_cp.vl["FSM5"]["TSR_Speed"]
    fp_ret.dashboardSpeedLimit = tsr_kph * CV.KPH_TO_MS if tsr_kph > 0 else 0.0
    start_stop = pt_cp.vl["StartStop"]["StartStop"], pt_cp.vl["Accessories_02"]["StartStop_Inv"]
    fp_ret.simRedLight = self._eucd_update_sim_red_light(start_stop == (1, 0), start_stop == (0, 1))
    return ret, fp_ret

  def _eucd_update_sim_red_light(self, start_stop_on, start_stop_off):
    """Simulated red light from the start/stop button, on its edges: turned off = red, back on = green."""
    CCP = CarControllerParams
    if not CCP.EUCD_SIM_RED_LIGHT:
      return False
    # Both copies must agree (0x322 and the inverted 0x180)
    if start_stop_on or start_stop_off:
      if self.eucd_start_stop_on is None:
        self.eucd_start_stop_on = start_stop_on  # state at start-up is not an edge
      self.eucd_start_stop_frames = self.eucd_start_stop_frames + 1 if start_stop_on != self.eucd_start_stop_on else 0
      if self.eucd_start_stop_frames >= CCP.EUCD_SIM_RED_LIGHT_DEBOUNCE_FRAMES:
        self.eucd_start_stop_on = start_stop_on
        self.eucd_start_stop_frames = 0
        self.eucd_sim_red_light = not start_stop_on
    return self.eucd_sim_red_light

  @staticmethod
  def get_can_parsers(CP):
    if isinstance(CAR(CP.carFingerprint).config, VolvoC1PlatformConfig):
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [
          ("VehicleSpeed1", 50),
          ("CCButtons", 100),
          ("PSCM1", 50),
          ("PedalandBrake", 100),
          ("TCM0", 10),
          ("ACC", 17),
          ("MiscCarInfo", 25),
        ], 0),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], [
          ("FSM0", 100),
          ("FSM1", 50),
        ], 2),
      }

    if isinstance(CAR(CP.carFingerprint).config, VolvoEUCDPlatformConfig):
      pt_messages = [
        ("VehicleSpeed1", 50),
        ("AccPedal", 100),
        ("BrakePedal", 50),
        ("Brake_Info", 50),
        ("PSCM1", 50),
        ("ACC_Speed", 50),
        ("MiscCarInfo", 25),
        ("Doors", 20),
        ("SAS0", 100),
        ("EngineData", 66),
        ("CCButtons", 66),
        ("Gear_Info", 50),
        ("BrakeLevel", 50),
        ("BrakeStatus", 25),
        ("wheelspeed1", 50),
        ("BrakeReqAck", 50),
        ("StartStop", 33),
        ("Accessories_02", 50),
        ("BrakeHold", 50),
        ("AccHold", 25),
      ]
      cam_messages = [
        ("FSM0", 100),
        ("FSM1", 50),
        ("FSM3", 50),
        ("FSM4", 33),
        ("FSM5", 10),
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
      }

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], 2),
    }
