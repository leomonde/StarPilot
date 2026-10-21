import pytest

from cereal import custom
from opendbc.car import ButtonType, CanData
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR

CCBUTTONS = 0x127
# CCButtons frames from V60 routes: idle, time gap increase, time gap decrease
IDLE = bytes.fromhex("88108761a7ff0000")
GAP_INCREASE = bytes.fromhex("88108761a5ff0200")
GAP_DECREASE = bytes.fromhex("88108361a3ff0400")
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
