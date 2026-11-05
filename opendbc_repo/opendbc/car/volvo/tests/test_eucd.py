import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo.eucd_can import calculate_eucd_lka_checksum, create_eucd_lka_msg
from opendbc.car.volvo.eucd_carcontroller import CarControllerEUCD
from opendbc.car.volvo.eucd_carstate import CarStateEUCD
from opendbc.car.volvo.eucd_params import EcmAccState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC, VolvoSafetyFlags

GearShifter = structs.CarState.GearShifter
DBC_NAME = DBC[CAR.VOLVO_V60][Bus.pt]


def params(alpha=False, release=False):
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V60)
  return CarInterface._get_params(cp, CAR.VOLVO_V60, {0: {}, 1: {}, 2: {}}, [], alpha, release, False)


def required_frames(**overrides):
  packer = CANPacker(DBC_NAME)
  values = {
    "VehicleSpeed1": {"VehicleSpeed": 72},
    "AccPedal": {"AccPedal": 0},
    "Brake_Info": {"BrakePedal": 0},
    "PSCM1": {"SteeringAngleServo": 12.5, "EPSTorque": 10, "LKATorque": 5},
    "ACC_Speed": {"ACC_Speed": 100},
    "MiscCarInfo": {"TurnSignal": 1},
    "Doors": {"DriverDoorClosed": 1, "PassengerDoorClosed": 1},
    "SAS0": {"SteeringRateOfChange": 2, "SteeringDirection": 0},
    "EngineData": {"ACC_State_ECM": EcmAccState.ENGAGED},
    "CCButtons": {},
    "GearSelector": {"GearLever": 3},
  }
  cam_values = {"FSM0": {"ACC_Available": 1}, "FSM3": {"ACC_Standstill": 0}}
  for name, signals in overrides.items():
    (cam_values if name in cam_values else values)[name] = signals
  return [CanData(*packer.make_can_msg(name, 0, signals)) for name, signals in values.items()] + \
         [CanData(*packer.make_can_msg(name, 2, signals)) for name, signals in cam_values.items()]


def car_state(**overrides):
  cp = params()
  cs = CarStateEUCD(cp)
  parsers = cs.get_can_parsers(cp)
  frames = required_frames(**overrides)
  for parser in parsers.values():
    parser.update([(1_000_000_000, frames)])
  return cs, parsers, cp


def test_params_are_lateral_only_with_stock_acc():
  for alpha in (False, True):
    for release in (False, True):
      cp = params(alpha, release)
      config = cp.safetyConfigs[0]
      assert (config.safetyModel, config.safetyParam) == (structs.CarParams.SafetyModel.volvo, VolvoSafetyFlags.EUCD.value)
      assert cp.pcmCruise and not cp.openpilotLongitudinalControl and not cp.alphaLongitudinalAvailable
      assert cp.radarUnavailable
      assert cp.steerControlType == structs.CarParams.SteerControlType.angle
      assert cp.minSteerSpeed == 0.
      assert cp.steerActuatorDelay == pytest.approx(0.2)
      assert cp.steerLimitTimer == pytest.approx(0.8)
      assert cp.steerAtStandstill


def test_interface_selects_the_eucd_implementation():
  cp = params()
  interface = CarInterface(cp)
  assert isinstance(interface.CS, CarStateEUCD)
  assert isinstance(interface.CC, CarControllerEUCD)


def test_required_can_and_state_contract():
  cs, parsers, _ = car_state()
  assert all(parser.can_valid for parser in parsers.values())
  ret = cs.update(parsers)
  assert ret.vEgoRaw == pytest.approx(72 * CV.KPH_TO_MS)
  assert ret.steeringAngleDeg == pytest.approx(12.5, abs=0.05)
  assert ret.gearShifter == GearShifter.drive
  assert ret.cruiseState.available and ret.cruiseState.enabled
  assert ret.cruiseState.speed == pytest.approx(100 * CV.KPH_TO_MS)
  assert ret.leftBlinker and not ret.rightBlinker
  assert not ret.doorOpen


@pytest.mark.parametrize("state, engaged", [(EcmAccState.OFF, False), (EcmAccState.STANDBY, False), (EcmAccState.ENGAGED, True)])
def test_cruise_follows_the_ecm(state, engaged):
  cs, parsers, _ = car_state(EngineData={"ACC_State_ECM": state})
  assert cs.update(parsers).cruiseState.enabled == engaged


@pytest.mark.parametrize("gear, expected", [(0, GearShifter.park), (1, GearShifter.reverse), (2, GearShifter.neutral), (3, GearShifter.drive)])
def test_gear_lever(gear, expected):
  cs, parsers, _ = car_state(GearSelector={"GearLever": gear})
  assert cs.update(parsers).gearShifter == expected


def control(lat_active, angle=0., cancel=False):
  cc = structs.CarControl()
  cc.latActive = lat_active
  cc.actuators.steeringAngleDeg = angle
  cc.cruiseControl.cancel = cancel
  return cc.as_reader()


def controller_state(speed=20., angle=0., brake=False):
  cs = type("CS", (), {})()
  cs.out = structs.CarState(vEgo=speed, vEgoRaw=speed, steeringAngleDeg=angle, brakePressed=brake)
  cs.eucd_pscm_stock_values = {"byte0": 0, "byte4": 0, "byte7": 0, "LKAActive": 0xFF, "EPSTorque": 0}
  return cs


def controller():
  return CarControllerEUCD({Bus.pt: DBC_NAME}, params())


def test_inactive_sends_zero_angle_and_no_direction():
  sends = controller().update(control(False, 10.), controller_state(), 0)[1]
  assert [s[0] for s in sends] == [0x262, 0x246]
  assert sends[0][1][5] & 0x3 == 0
  assert ((sends[0][1][3] & 0x3F) << 8) + sends[0][1][4] == 8192  # 0 deg


def test_active_steering_is_rate_limited_and_50hz():
  cc = controller()
  new_actuators, sends = cc.update(control(True, 10.), controller_state(), 0)
  assert [s[0] for s in sends] == [0x262, 0x246]
  assert 0 < new_actuators.steeringAngleDeg <= 0.15 + 1e-3
  assert cc.update(control(True, 10.), controller_state(), 0)[1] == []
  assert sends[0][1][5] & 0x3 == 2  # left


def test_direction_flip_pauses_on_none():
  cc = controller()
  cc.eucd_apply_steer_prev = 0.05
  cc.eucd_apply_steer_dir_prev = 2
  cc.eucd_latActive_prev = True
  sends = cc.update(control(True, -10.), controller_state(angle=5.), 0)[1]
  assert sends[0][1][5] & 0x3 == 0
  assert cc.eucd_steer_blocked and cc.eucd_steer_blocked_cnt == 7


def test_pscm_relay_clears_lka_state():
  sends = controller().update(control(False), controller_state(angle=4.), 0)[1]
  assert sends[1][2] == 2
  parser_values = {"LKAActive": 0xFF & 0xF5}
  packer = CANPacker(DBC_NAME)
  expected = packer.make_can_msg("PSCM1", 2, {**parser_values, "SteeringAngleServo": 4., "LKATorque": 0, "EPSTorque": 0})
  assert sends[1][1] == expected[1]


@pytest.mark.parametrize("speed, brake, sent", [(20., False, True), (0., False, False), (20., True, False)])
def test_cancel_only_while_moving_and_off_the_brake(speed, brake, sent):
  sends = controller().update(control(False, cancel=True), controller_state(speed, brake=brake), 0)[1]
  assert (0x127 in [s[0] for s in sends]) == sent


def test_lka_checksum_matches_the_frame():
  packer = CANPacker(DBC_NAME)
  _, dat, _ = create_eucd_lka_msg(packer, -7.5, 1)
  assert dat[6] == calculate_eucd_lka_checksum(dat)


def test_engine_firmware_identifies_the_v60():
  from opendbc.car.fw_versions import match_fw_to_car
  fw = structs.CarParams.CarFw(ecu=structs.CarParams.Ecu.engine, address=0x7e0, subAddress=0, brand="volvo",
                               fwVersion=b"31361041 AJ" + bytes(13))
  assert match_fw_to_car([fw], "")[1] == {CAR.VOLVO_V60}
