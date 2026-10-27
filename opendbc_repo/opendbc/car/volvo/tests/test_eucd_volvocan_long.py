import copy
import math

import pytest

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, EcmAccState, LongMode

DBC = "volvo_v60_2015_pt"
FSM3 = 0x270

# stock FSM3 frames from the camera, route 00000808 (ACC engaged); b4=0x81 data, 0x00 type
STOCK = {
  ("data", LongMode.BRAKE): ["aa7ad40081000000", "aa7ed40081000000"],
  ("data", LongMode.COAST): ["aa6ed5b881000000", "aa78d5dd81000000"],
  ("data", LongMode.DRIVE): ["aa98d65f81000000", "aa88d61d81000000"],
  ("type", LongMode.BRAKE): ["0082000000000000", "0076000000000000"],
  ("type", LongMode.COAST): ["007a01ec00000000", "007c01f300000000"],
  ("type", LongMode.DRIVE): ["0088022100000000", "008a022600000000"],
}
ALL_FRAMES = [(kind, mode, bytes.fromhex(h)) for (kind, mode), hs in STOCK.items() for h in hs]


def parse(name: str, addr: int, freq: int, dat: bytes) -> dict:
  cp = CANParser(DBC, [(name, freq)], 0)
  cp.update([(0, [(addr, dat, 0)])])
  return copy.copy(cp.vl[name])


def parse_fsm3(dat: bytes) -> dict:
  return parse("FSM3", FSM3, 50, dat)


def build(stock: bytes, accel=None, mode=None) -> bytes:
  fsm3 = parse_fsm3(stock)
  if accel is None:
    accel = fsm3["ACC_AccelerationRequest"]
  return volvocan.create_eucd_longitudinal(CANPacker(DBC), fsm3, accel, int(fsm3["ACC_Check"]), mode)[1]


@pytest.mark.parametrize("kind,mode,stock", ALL_FRAMES)
def test_relay_is_byte_identical(kind, mode, stock):
  assert build(stock) == stock


@pytest.mark.parametrize("kind,mode,stock", ALL_FRAMES)
def test_stock_mode_and_traction_reproduce_frame(kind, mode, stock):
  # Camera's own mode leaves Byte_2 untouched
  out = build(stock, mode=mode)
  assert out[2] == stock[2]
  assert out[:2] == stock[:2] and out[4:] == stock[4:]
  if mode == LongMode.BRAKE:
    assert out[3] == 0


@pytest.mark.parametrize("kind,stock_mode,stock", ALL_FRAMES)
@pytest.mark.parametrize("mode", list(LongMode))
def test_op_mode_keeps_cadence(kind, stock_mode, stock, mode):
  accel = {LongMode.BRAKE: -1.5, LongMode.COAST: -0.1, LongMode.DRIVE: 0.8}[mode]
  out = build(stock, accel=accel, mode=mode)
  assert out[2] & 0x03 == mode
  assert out[2] & 0xFC == stock[2] & 0xFC  # 0xD4 data / 0x00 type kept
  assert out[4] == stock[4]                # data/type marker kept
  assert out[0] == stock[0]                # standstill and status bits kept
  assert out[1] == math.floor((accel + 5.04) / 0.04 + 0.5)  # packer rounds half up
  assert out[3] == volvocan.eucd_fsm3_traction(accel, mode)


def test_traction():
  assert volvocan.eucd_fsm3_traction(-2.0, LongMode.BRAKE) == 0
  assert volvocan.eucd_fsm3_traction(1.0, LongMode.DRIVE) == 91
  assert volvocan.eucd_fsm3_traction(2.0, LongMode.DRIVE) == 188
  assert volvocan.eucd_fsm3_traction(-0.5, LongMode.DRIVE) == 0
  assert volvocan.eucd_fsm3_traction(-0.1, LongMode.COAST) == 256 - 18
  assert volvocan.eucd_fsm3_traction(0.5, LongMode.COAST) == 0xFF  # never non-negative in COAST


def test_mode_hysteresis():
  CCP = CarControllerParams
  m = LongMode.COAST
  m = volvocan.eucd_next_long_mode(m, -0.4)
  assert m == LongMode.COAST
  m = volvocan.eucd_next_long_mode(m, CCP.EUCD_BRAKE_ENTER - 0.01)
  assert m == LongMode.BRAKE
  m = volvocan.eucd_next_long_mode(m, CCP.EUCD_BRAKE_EXIT - 0.05)  # stays braking until BRAKE_EXIT
  assert m == LongMode.BRAKE
  m = volvocan.eucd_next_long_mode(m, CCP.EUCD_BRAKE_EXIT + 0.05)
  assert m == LongMode.COAST
  m = volvocan.eucd_next_long_mode(m, 0.06)
  assert m == LongMode.COAST
  m = volvocan.eucd_next_long_mode(m, 0.1)
  assert m == LongMode.DRIVE
  m = volvocan.eucd_next_long_mode(m, 0.05)
  assert m == LongMode.DRIVE
  m = volvocan.eucd_next_long_mode(m, -1.0)  # hard brake: DRIVE straight to BRAKE
  assert m == LongMode.BRAKE
  m = volvocan.eucd_next_long_mode(m, 1.0)
  assert m == LongMode.DRIVE


FSM4 = 0x31A
FSM1 = 0x260
# stock camera frames, route 00000808 (ACC engaged)
STOCK_FSM4 = ["aaf1343a8ef24600", "aaf15e5e8af3cb00", "aaf173748ab40b00", "aaf173738ab3f800"]  # Byte_5 0xF2/0xF3 brake, 0xB4/0xB3 not
STOCK_FSM1 = ["ffffb80049e37408", "5ad4bc0049e37408", "46ff000000e31408", "23f4040000e31408"]  # Byte_2 data 0xB8/0xBC, type 0x00/0x04


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM4])
def test_fsm4_relay_is_byte_identical(stock):
  assert volvocan.create_eucd_fsm4(CANPacker(DBC), parse("FSM4", FSM4, 33, stock), 0.0)[1] == stock


def fsm4(stock: bytes, accel: float, mode) -> bytes:
  return volvocan.create_eucd_fsm4(CANPacker(DBC), parse("FSM4", FSM4, 33, stock), accel, mode)[1]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM4])
@pytest.mark.parametrize("accel", [-3.5, -2.0, -1.0, -0.6, -0.1])
def test_fsm4_op_brake_sets_value_from_accel(stock, accel):
  out = fsm4(stock, accel, LongMode.BRAKE)
  value = (out[5] << 8) | out[6]
  assert value == volvocan.eucd_fsm4_brake_value(accel, True)
  assert value & 0xC000 == 0xC000  # bit15 always, bit14 = hydraulic brake request
  assert out[:5] == stock[:5] and out[7:] == stock[7:]


def test_fsm4_brake_value_matches_stock_levels():
  # Matches the stock Byte_5 levels
  assert volvocan.eucd_fsm4_brake_value(-3.5, True) >> 8 == 0xF0
  assert volvocan.eucd_fsm4_brake_value(0.0, True) >> 8 == 0xF4
  values = [volvocan.eucd_fsm4_brake_value(a / 10, True) for a in range(-40, 5)]
  assert values == sorted(values)  # stronger decel, lower value
  # Smooth: a small step moves a few counts
  assert all(abs(volvocan.eucd_fsm4_brake_value(a / 50, True) - volvocan.eucd_fsm4_brake_value((a + 1) / 50, True)) < 16 for a in range(-200, 10))


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM4[2:]])
@pytest.mark.parametrize("mode", [LongMode.COAST, LongMode.DRIVE])
def test_fsm4_no_brake_keeps_camera_value(stock, mode):
  assert fsm4(stock, 0.3, mode) == stock


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM4[:2]])
@pytest.mark.parametrize("mode", [LongMode.COAST, LongMode.DRIVE])
def test_fsm4_camera_brake_cleared_when_op_does_not_brake(stock, mode):
  out = fsm4(stock, 0.3, mode)
  value = (out[5] << 8) | out[6]
  assert value == volvocan.eucd_fsm4_brake_value(0.3, False)
  assert value & 0xC000 == 0x8000
  assert out[:5] == stock[:5] and out[7:] == stock[7:]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM1])
def test_fsm1_relay_is_byte_identical(stock):
  assert volvocan.create_eucd_radar(CANPacker(DBC), parse("FSM1", FSM1, 50, stock))[1] == stock


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_FSM1])
@pytest.mark.parametrize("mode", list(LongMode))
def test_fsm1_brake_bit_follows_op(stock, mode):
  out = volvocan.create_eucd_radar(CANPacker(DBC), parse("FSM1", FSM1, 50, stock), mode)[1]
  assert out[2] == (stock[2] & ~0x04) | (0x04 if mode == LongMode.BRAKE else 0)
  assert out[:2] == stock[:2] and out[3:] == stock[3:]


BRAKE_STATUS_ADDR = 0x323
BRAKE_LEVEL_ADDR = 0x133
# Stock BrakeStatus and BrakeLevel frames, route 807
STOCK_BRAKE_STATUS = ["0000fc04c5800400", "0000fc04d5800400", "0000fc04e5800400"]
STOCK_BRAKE_LEVEL = ["0b146e3cce370719", "0b2d6df6cdf60732"]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_STATUS])
def test_brake_status_relay_to_camera(stock):
  addr, dat, bus = volvocan.create_eucd_cam_brake_status(CANPacker(DBC), parse("BrakeStatus", BRAKE_STATUS_ADDR, 25, stock), False)
  assert (addr, dat, bus) == (BRAKE_STATUS_ADDR, stock, 2)


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_STATUS])
def test_brake_status_mask_clears_brake_ack(stock):
  out = volvocan.create_eucd_cam_brake_status(CANPacker(DBC), parse("BrakeStatus", BRAKE_STATUS_ADDR, 25, stock), True)[1]
  assert out[4] == stock[4] & ~0x10  # 0xD5 -> 0xC5, others unchanged
  assert out[:4] == stock[:4] and out[5:] == stock[5:]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_LEVEL])
def test_brake_level_relay_to_camera(stock):
  addr, dat, bus = volvocan.create_eucd_cam_brake_level(CANPacker(DBC), parse("BrakeLevel", BRAKE_LEVEL_ADDR, 50, stock), None)
  assert (addr, dat, bus) == (BRAKE_LEVEL_ADDR, stock, 2)


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_LEVEL])
def test_brake_level_mask_holds_brake_level(stock):
  out = volvocan.create_eucd_cam_brake_level(CANPacker(DBC), parse("BrakeLevel", BRAKE_LEVEL_ADDR, 50, stock), 0x14)[1]
  assert out[1] == (stock[1] & 0x80) | 0x14
  assert out[0] == stock[0] and out[2:] == stock[2:]


WHEELSPEED1_ADDR = 0x220
BRAKE_REQ_ACK_ADDR = 0x299
# Stock wheelspeed1 and BrakeReqAck frames, route 816
STOCK_WHEELSPEED1 = ["8b11e000a2577a30", "8b11e400a245a216", "8b11e500a1f6a1d3"]
STOCK_BRAKE_REQ_ACK = ["40054c020dff2235", "4005480201f72239"]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_WHEELSPEED1])
@pytest.mark.parametrize("mask", [False, True])
def test_wheelspeed1_relay_to_camera(stock, mask):
  addr, dat, bus = volvocan.create_eucd_cam_wheel_status(CANPacker(DBC), parse("wheelspeed1", WHEELSPEED1_ADDR, 50, stock), mask)
  assert (addr, bus) == (WHEELSPEED1_ADDR, 2)
  assert dat[2] == (stock[2] & ~0x04 if mask else stock[2])  # 0xE4 -> 0xE0, 0xE5 -> 0xE1
  assert dat[:2] == stock[:2] and dat[3:] == stock[3:]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_REQ_ACK])
@pytest.mark.parametrize("mask", [False, True])
def test_brake_req_ack_relay_to_camera(stock, mask):
  addr, dat, bus = volvocan.create_eucd_cam_brake_request_ack(CANPacker(DBC), parse("BrakeReqAck", BRAKE_REQ_ACK_ADDR, 50, stock), mask)
  assert (addr, bus) == (BRAKE_REQ_ACK_ADDR, 2)
  assert dat[5] == (stock[5] | 0x08 if mask else stock[5])  # 0xF7 -> 0xFF
  assert dat[:5] == stock[:5] and dat[6:] == stock[6:]


FSM0 = 0x51
# Camera FSM0 before and after an abort, route 816
FSM0_OK = bytes.fromhex("15e3860091955f7a")
FSM0_ABORT = bytes.fromhex("15e3821091955f7e")


def fsm0(stock: bytes, ecm_engaged: bool) -> bytes:
  return volvocan.create_eucd_fsm0(CANPacker(DBC), parse("FSM0", FSM0, 100, stock), ecm_engaged)[1]


@pytest.mark.parametrize("stock", [FSM0_OK, FSM0_ABORT])
def test_fsm0_relay_is_byte_identical(stock):
  assert fsm0(stock, True) == stock


def test_fsm0_never_enabled_without_ecm():
  out = fsm0(FSM0_OK, False)
  assert out[2] == FSM0_OK[2] & ~0x04
  assert out[:2] == FSM0_OK[:2] and out[3:] == FSM0_OK[3:]


ENGINE_DATA = 0x12A


# Stock 0x12A Byte_4: 0xA8 standby, 0x98 off, 0xB8 engaged
@pytest.mark.parametrize("b4,state", [(0xA8, EcmAccState.STANDBY), (0x98, EcmAccState.OFF), (0xB8, EcmAccState.ENGAGED)])
def test_ecm_acc_state_decoding(b4, state):
  dat = bytes([0xEA, 0x00, 0xC7, 0xF8, b4, 0xF8, 0xE5, 0x7A])
  assert int(parse("EngineData", ENGINE_DATA, 66, dat)["ACC_State_ECM"]) == state


def test_cam_brake_mask_msgs_are_relayed_messages():
  # Only relayed messages can be masked
  assert CarControllerParams.EUCD_CAM_BRAKE_MASK_MSGS <= {"BrakeLevel", "BrakeStatus", "wheelspeed1", "BrakeReqAck"}


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_WHEELSPEED1])
def test_wheelspeed1_spoofs_brake_request(stock):
  dat = volvocan.create_eucd_cam_wheel_status(CANPacker(DBC), parse("wheelspeed1", WHEELSPEED1_ADDR, 50, stock), False, True)[1]
  assert dat[2] == stock[2] | 0x04
  assert dat[:2] == stock[:2] and dat[3:] == stock[3:]


@pytest.mark.parametrize("stock", [bytes.fromhex(h) for h in STOCK_BRAKE_REQ_ACK])
def test_brake_req_ack_spoofs_ack(stock):
  dat = volvocan.create_eucd_cam_brake_request_ack(CANPacker(DBC), parse("BrakeReqAck", BRAKE_REQ_ACK_ADDR, 50, stock), False, True)[1]
  assert dat[5] == stock[5] & ~0x08  # 0xFF -> 0xF7
  assert dat[:5] == stock[:5] and dat[6:] == stock[6:]


def test_fsm4_blends_brake_target_into_camera():
  stock = bytes.fromhex(STOCK_FSM4[0])  # camera braking (Byte_5 bit6)
  fsm4_vl = parse("FSM4", FSM4, 33, stock)
  cam_value = ((stock[5] << 8) | stock[6]) & 0x3FFF
  op = volvocan.create_eucd_fsm4(CANPacker(DBC), fsm4_vl, -1.0, LongMode.BRAKE)[1]
  op_value = ((op[5] << 8) | op[6]) & 0x3FFF
  half = volvocan.create_eucd_fsm4(CANPacker(DBC), fsm4_vl, -1.0, LongMode.BRAKE, cam_weight=0.5)[1]
  assert ((half[5] << 8) | half[6]) & 0x3FFF == round(op_value + 0.5 * (cam_value - op_value))
  full = volvocan.create_eucd_fsm4(CANPacker(DBC), fsm4_vl, -1.0, LongMode.BRAKE, cam_weight=1.0)[1]
  assert ((full[5] << 8) | full[6]) & 0x3FFF == cam_value and full[5] & 0x40
