import pytest

from cereal import custom
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, ButtonType, CanData, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC

GearShifter = structs.CarState.GearShifter
CCBUTTONS = 0x127
# CCButtons frames from V60 routes: idle, time gap increase, time gap decrease
IDLE = bytes.fromhex("88108761a7ff0000")
GAP_INCREASE = bytes.fromhex("88108761a5ff0200")
GAP_DECREASE = bytes.fromhex("88108761a3ff0400")  # LKA_State (b2 bit2) as in IDLE, else it is an LKA press
# increase bit set without its inverted copy cleared: not a press
GAP_INCREASE_NO_INV = bytes.fromhex("88108761a7ff0200")


@pytest.fixture
def car():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V60)
  return CarState(cp, custom.StarPilotCarParams.new_message()), CarState.get_can_parsers(cp)


def step(car, t, dat):
  cs, parsers = car
  for parser in parsers.values():
    parser.update([(t, [CanData(CCBUTTONS, dat, 0)])])
  ret, _ = cs.update(parsers, None)
  return [(be.type, be.pressed) for be in ret.buttonEvents]


@pytest.mark.parametrize("pressed", [GAP_INCREASE, GAP_DECREASE])
def test_gap_button_press_and_release(car, pressed):
  assert step(car, 1_000_000, IDLE) == []
  assert step(car, 2_000_000, pressed) == [(ButtonType.gapAdjustCruise, True)]
  assert step(car, 3_000_000, pressed) == []
  assert step(car, 4_000_000, IDLE) == [(ButtonType.gapAdjustCruise, False)]


def test_gap_button_needs_inverted_signal(car):
  assert step(car, 1_000_000, IDLE) == []
  assert step(car, 2_000_000, GAP_INCREASE_NO_INV) == []


@pytest.mark.parametrize("tsr_kph", [0, 30, 80, 120])
def test_tsr_speed_limit(car, tsr_kph):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  addr, dat, bus = packer.make_can_msg("FSM5", 2, {"TSR_Speed": tsr_kph})
  for parser in parsers.values():
    parser.update([(1_000_000, [CanData(addr, dat, bus)])])
  _, fp_ret = cs.update(parsers, None)
  assert fp_ret.dashboardSpeedLimit == pytest.approx(tsr_kph * CV.KPH_TO_MS)


@pytest.mark.parametrize("gear, speed_kph, expected", [
  (0, 0, GearShifter.park),
  (1, 0, GearShifter.reverse),
  (1, 8, GearShifter.reverse),
  (2, 0, GearShifter.neutral),
  (3, 40, GearShifter.drive),
])
def test_gear_lever(car, gear, speed_kph, expected):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  msgs = [packer.make_can_msg("GearSelector", 0, {"GearLever": gear}),
          packer.make_can_msg("VehicleSpeed1", 0, {"VehicleSpeed": speed_kph})]
  for parser in parsers.values():
    parser.update([(1_000_000, [CanData(*m) for m in msgs])])
  ret, _ = cs.update(parsers, None)
  assert ret.gearShifter == expected


def test_lka_press_events(car):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  presses = []
  for t, lka in enumerate((1, 1, 0, 0, 1)):
    msgs = [packer.make_can_msg("CCButtons", 0, {"LKA_State": lka})]
    for parser in parsers.values():
      parser.update([(1_000_000 * (t + 1), [CanData(*m) for m in msgs])])
    ret, fp_ret = cs.update(parsers, None)
    presses.append(sum(be.type == ButtonType.lkas and be.pressed for be in ret.buttonEvents))
    assert fp_ret.alwaysOnLateralAllowed == bool(lka)
  assert presses == [0, 0, 1, 0, 1]  # one press per LKA_State flip, none at start


def test_driver_resume_events(car):
  # Resume needs the bit set and its inverted copy cleared, frames from route 8ab
  resume = bytes.fromhex("88108761a7df0020")
  no_inv = bytes.fromhex("88108761a7ff0020")
  assert step(car, 1_000_000, IDLE) == []
  assert step(car, 2_000_000, no_inv) == []
  assert step(car, 3_000_000, resume) == [(ButtonType.resumeCruise, True)]
  assert step(car, 4_000_000, resume) == []
  assert step(car, 5_000_000, IDLE) == [(ButtonType.resumeCruise, False)]
