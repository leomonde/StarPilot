from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.eucd_params import CANBUS, CarControllerEUCDParams, EcmAccState
from opendbc.car.volvo.values import DBC

GearShifter = structs.CarState.GearShifter


class CarStateEUCD(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.eucd_cruise_enabled_prev = False
    self.eucd_eps_torque_timer = 0
    self.eucd_ecm_acc_engaged = False
    self.eucd_pscm_stock_values = {}

  def update(self, can_parsers) -> structs.CarState:
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
    # Camera standstill hold, gated by the car's standstill
    ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"]) and ret.standstill
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False

    # Servo timeout while ACC is active
    if ret.cruiseState.enabled and ret.vEgo > CarControllerEUCDParams.EUCD_MIN_STEER_SPEED:
      if not self.eucd_cruise_enabled_prev:
        self.eucd_eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eucd_eps_torque_timer += 1
      else:
        self.eucd_eps_torque_timer = 0

      ret.steerFaultTemporary = self.eucd_eps_torque_timer >= CarControllerEUCDParams.EUCD_STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.eucd_cruise_enabled_prev = ret.cruiseState.enabled

    # Gear lever position
    ret.gearShifter = {
      0: GearShifter.park,
      1: GearShifter.reverse,
      2: GearShifter.neutral,
      3: GearShifter.drive,
    }[int(pt_cp.vl["GearSelector"]["GearLever"])]

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    self.eucd_pscm_stock_values = pt_cp.vl["PSCM1"]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("MiscCarInfo", 25),
      ("Doors", 20),
      ("SAS0", 100),
      ("EngineData", 66),
      ("CCButtons", 66),
      ("GearSelector", 66),
    ]
    cam_messages = [
      ("FSM0", 100),
      ("FSM3", 50),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
    }
