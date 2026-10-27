import numpy as np

from opendbc.car.volvo.helpers import (checksum_lca_2_message, checksum_2_0x69_message, checksum_1_pscm_related_message,
                                       checksum_2_pscm_related_message, checksum_lca_5_message)
from opendbc.car.carlog import carlog
from opendbc.car.volvo.values import CANBUS, CarControllerParams, LongMode


def create_c1_pscm_message(packer, msg_pscm: dict):
  values = {
    "LKATorque": 0,
    "SteeringAngleServo": msg_pscm["SteeringAngleServo"],
    "byte0": msg_pscm["byte0"],
    "byte3": msg_pscm["byte3"],
    "byte4": msg_pscm["byte4"],
    "byte7": msg_pscm["byte7"],
    "LKAActive": int(msg_pscm["LKAActive"]) & 0xD,
  }
  return packer.make_can_msg("PSCM1", 2, values)


def create_c1_checksum(data: bytes) -> int:
  angle_raw = ((data[4] & 0x3F) << 8) | data[5]
  direction = data[7] & 0x3
  checksum_sum = (data[3] + direction + angle_raw + (angle_raw >> 8)) & 0xFF
  return checksum_sum ^ 0xFF


def create_c1_steering_control(packer, apply_angle: float, lat_active: bool):
  values = {
    "SET_X_E3": 0xE3,
    "SET_X_B4": 0xB4,
    "SET_X_08": 0x08,
    "TrqLim": 0,
    "LKAAngleReq": apply_angle,
    "LKASteerDirection": 3 if lat_active else 0,
    "SET_X_25": 0x25,
    "SET_X_02": 0x02,
  }
  data = packer.make_can_msg("FSM1", 0, values)[1]
  values["Checksum"] = create_c1_checksum(data)
  return packer.make_can_msg("FSM1", 0, values)


def create_c1_cancel(packer):
  return packer.make_can_msg("CCButtons", 0, {"ACCStopBtn": 1})


def create_lca_message(packer, lat_active: bool, apply_angle: float, msg_lca: dict,
                       authority_pos: int = 614, authority_neg: int = -614,
                       overrides: dict | None = None):
  """LCA (0x58) angle steering command, sent every frame."""
  if not lat_active:
    return packer.make_can_msg('LCA', 2, msg_lca)

  # Positive angle is a left turn in both openpilot and Volvo

  values = {
    'NEW_SIGNAL_1': 3,
    'LCA_ENABLE_INV': 0 if lat_active else 1,
    'LANE_KEEP_ACTIVE_INV': 3,
    'LCA_STEER_LOOSELY': int(authority_pos) if lat_active else 0,
    'NEW_SIGNAL_7': 7,
    'LCA_STEER_LOOSELY_INV': int(authority_neg) if lat_active else 0,
    # Steering rate
    'LCA_RATE_OF_CHANGE': 80 if lat_active else 251,
    'LCA_STEER': msg_lca['LCA_STEER'],
    'NEW_SIGNAL_6': 15,
  }

  # Live-tuning overrides
  if overrides:
    for key, val in overrides.items():
      values[key] = val

  return packer.make_can_msg('LCA', 2, values)

def create_pscm_message(packer, lat_active: bool, msg_pscm: dict, frame: int):
  values = {
    'PSCM_ANGLE_SENSOR': msg_pscm['PSCM_ANGLE_SENSOR'],
    'BIT_0': msg_pscm['BIT_0'],
    'HANDS_ON_STEERING_WHEEL_A': msg_pscm['HANDS_ON_STEERING_WHEEL_A'],
    'HANDS_ON_STEERING_WHEEL_B': msg_pscm['HANDS_ON_STEERING_WHEEL_B'],
    'BYTE_4': msg_pscm['BYTE_4'],
    'DRIVER_INPUT_DEVIATION': msg_pscm['DRIVER_INPUT_DEVIATION'],
    'BYTE_6': msg_pscm['BYTE_6'],
    'BYTE_7': msg_pscm['BYTE_7'],
  }

  # Spoof hands on wheel while openpilot steers
  if lat_active:
    values['HANDS_ON_STEERING_WHEEL_B'] = 186 if frame % 2 == 0 else 154  # stock HANDS_ON_STEERING_WHEEL_B
    values['HANDS_ON_STEERING_WHEEL_A'] = 195 if frame % 2 == 0 else 249  # stock HANDS_ON_STEERING_WHEEL_A

  return packer.make_can_msg('PSCM', 0, values)

def create_lca_3_message(packer, lat_active: bool, apply_angle: float, msg_lca_3: dict, counter_value: int):
  """LCA_3 message, lets the PSCM accept LCA commands."""
  values = {
    'NEW_SIGNAL_3': 0 if lat_active else msg_lca_3['NEW_SIGNAL_3'],
    'LCA_ACCEPT_COMMANDS_RELATED': 15 if lat_active else msg_lca_3['LCA_ACCEPT_COMMANDS_RELATED'],
    'NEW_SIGNAL_2': 0 if lat_active else msg_lca_3['NEW_SIGNAL_2'],
    'NEW_SIGNAL_5': 30 if lat_active else msg_lca_3['NEW_SIGNAL_5'],
    'LCA_ACCEPT_COMMANDS_INV': 0 if lat_active else msg_lca_3['LCA_ACCEPT_COMMANDS_INV'],
    'NEW_SIGNAL_4': 3 if lat_active else msg_lca_3['NEW_SIGNAL_4'],
    'SPEED_A': msg_lca_3['SPEED_A'],
    'SPEED_B': msg_lca_3['SPEED_B'],
    'NEW_SIGNAL_8': 1 if lat_active else msg_lca_3['NEW_SIGNAL_8'],
    'NEW_SIGNAL_7': 3 if lat_active else msg_lca_3['NEW_SIGNAL_7'],
    'NEW_SIGNAL_9': msg_lca_3['NEW_SIGNAL_9'],
    'COUNTER_1': counter_value,
  }

  return packer.make_can_msg('LCA_3', 2, values)

def diff_dicts(a, b):
  only_in_a = a.keys() - b.keys()
  only_in_b = b.keys() - a.keys()
  in_both = a.keys() & b.keys()

  changed = {k: (a[k], b[k]) for k in in_both if a[k] != b[k]}

  return {
    "only_in_a": {k: a[k] for k in only_in_a},
    "only_in_b": {k: b[k] for k in only_in_b},
    "changed": changed,
  }

def create_lca_2_message(packer, lat_active: bool, msg_lca_2: dict, counter_1: int, counter_2: int):
  """LCA_2 message, spoofs PILOT_ASSIST_ENGAGED while active."""
  #return packer.make_can_msg('LCA_2', 2, msg_lca_2)
  #if not lat_active:
  #  return packer.make_can_msg('LCA_2', 2, msg_lca_2)

  #values = dict(msg_lca_2)

  values = {
    'BYTE_0': 24 if lat_active else msg_lca_2['BYTE_0'],  # always 24
    'COUNTER_1': msg_lca_2['COUNTER_1'],  # +2 mod 16
    'PILOT_ASSIST_ENGAGED': 1 if lat_active else msg_lca_2['PILOT_ASSIST_ENGAGED'],  # byte 1 bit 4
    'BYTE_1_BITFIELD_0': msg_lca_2['BYTE_1_BITFIELD_0'],
    'ESC_ACTUATING': msg_lca_2['ESC_ACTUATING'],
    'ESC_ELIGIBLE': msg_lca_2['ESC_ELIGIBLE'],
    'CHECKSUM_2': msg_lca_2['CHECKSUM_2'],  # bytes 0 and 1
    'NEW_SIGNAL_2': 0 if lat_active else msg_lca_2['NEW_SIGNAL_2'],
    'COUNTER_2': msg_lca_2['COUNTER_2'],  # +4 mod 16
    'NEW_SIGNAL_3': 3 if lat_active else msg_lca_2['NEW_SIGNAL_3'],
    'BRAKE_PEDAL_PRESSED_B': msg_lca_2['BRAKE_PEDAL_PRESSED_B'],
    'BRAKE_PEDAL_PRESSED_A': msg_lca_2['BRAKE_PEDAL_PRESSED_A'],
    'CHECKSUM_1': msg_lca_2['CHECKSUM_1'],  # bytes 1, 2 and 5
    'BYTE_7': 0 if lat_active else msg_lca_2['BYTE_7'],
  }

  dat = packer.make_can_msg('LCA_2', 2, values)

  built_bytes = dat[1]  # (addr, bytes, bus)
  b0 = built_bytes[0]
  b1 = built_bytes[1]
  b2 = built_bytes[2]
  b3 = built_bytes[3]
  b4 = built_bytes[4]
  b5 = built_bytes[5]

  values['CHECKSUM_1'] = checksum_lca_2_message(b0, b5)

  # Validate only when inactive and valid
  if not lat_active:
    #assert values['CHECKSUM_1'] == msg_lca_2['CHECKSUM_1']
    if values['CHECKSUM_1'] != msg_lca_2['CHECKSUM_1']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM mismatch")
      print(f"b0={b0}, b1={b1}, b2={b2}, b5={b5}, calculated={values['CHECKSUM_1']}, expected={msg_lca_2['CHECKSUM_1']}")
      #assert False

  # Checksum 2: bytes 0, 1, 3, 4
  checksum_2 = checksum_2_0x69_message(b0, b1, b3, b4)
  values['CHECKSUM_2'] = checksum_2
  if not lat_active:
    if values['CHECKSUM_2'] != msg_lca_2['CHECKSUM_2']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM_2 mismatch")
      print(f"b0={b0}, b1={b1}, b3={b3}, b4={b4}, calculated={values['CHECKSUM_2']}, expected={msg_lca_2['CHECKSUM_2']}")
      #assert False
  values['COUNTER_1'] = counter_1
  values['COUNTER_2'] = counter_2
  # Re-pack with the new counters
  dat = packer.make_can_msg('LCA_2', 2, values)
  built_bytes = dat[1]  # (addr, bytes, bus)
  b0 = built_bytes[0]
  b1 = built_bytes[1]
  b3 = built_bytes[3]
  b4 = built_bytes[4]
  b5 = built_bytes[5]
  values['CHECKSUM_1'] = checksum_lca_2_message(b0, b5)
  values['CHECKSUM_2'] = checksum_2_0x69_message(b0, b1, b3, b4)
  return packer.make_can_msg('LCA_2', 2, values)

def create_lca_5_message(packer, lat_active: bool, target_angle_deg: float, msg_lca_5: dict, counter: int,
                         overrides: dict | None = None):
  """LCA_5 (0x67) angle steering message."""

  if not lat_active:
    return packer.make_can_msg('LCA_5', 2, msg_lca_5)

  # LCA_5_STEER in degrees, the packer encodes it

  # Wheel speeds and counter unchanged
  values = {
    'WHEEL_SPEED_1': msg_lca_5['WHEEL_SPEED_1'],
    'NEW_SIGNAL_4': msg_lca_5['NEW_SIGNAL_4'],
    'NEW_SIGNAL_1': msg_lca_5['NEW_SIGNAL_1'],
    'WHEEL_SPEED_2': msg_lca_5['WHEEL_SPEED_2'],
    'NEW_SIGNAL_5': msg_lca_5['NEW_SIGNAL_5'],
    'NEW_SIGNAL_2': msg_lca_5['NEW_SIGNAL_2'],
    'LCA_5_STEER': target_angle_deg,
    'COUNTER': counter,
  }

  # Live-tuning overrides
  if overrides:
    for key, val in overrides.items():
      values[key] = val

  dat = packer.make_can_msg('LCA_5', 2, values)
  built_bytes = dat[1]  # (addr, bytes, bus)
  values['CHECKSUM'] = checksum_lca_5_message(built_bytes[0], built_bytes[1], built_bytes[3], built_bytes[4], built_bytes[5])
  return packer.make_can_msg('LCA_5', 2, values)

def create_speed_message(packer, msg_speed: dict):
  """Forward SPEED (0x60)."""
  values = {
    'SPEED': msg_speed['SPEED'],
    'NEW_SIGNAL_1': msg_speed['NEW_SIGNAL_1'],
    'NEW_SIGNAL_2': msg_speed['NEW_SIGNAL_2'],
    'NEW_SIGNAL_3': msg_speed['NEW_SIGNAL_3'],
    'NEW_SIGNAL_4': msg_speed['NEW_SIGNAL_4'],
    'NEW_SIGNAL_5': msg_speed['NEW_SIGNAL_5'],
    'NEW_SIGNAL_6': msg_speed['NEW_SIGNAL_6'],
  }

  return packer.make_can_msg('SPEED', 2, values)

def create_speed_2_message(packer, msg_speed_2: dict):
  """Forward SPEED_2 (0x68)."""
  values = {
    'WHEEL_SPEED_LEFT': msg_speed_2['WHEEL_SPEED_LEFT'],
    'WHEEL_SPEED_RIGHT': msg_speed_2['WHEEL_SPEED_RIGHT'],
    'NEW_SIGNAL_1': msg_speed_2['NEW_SIGNAL_1'],
    'NEW_SIGNAL_2': msg_speed_2['NEW_SIGNAL_2'],
    'NEW_SIGNAL_3': msg_speed_2['NEW_SIGNAL_3'],
    'NEW_SIGNAL_4': msg_speed_2['NEW_SIGNAL_4'],
    'COUNTER_1': msg_speed_2['COUNTER_1'],
    'COUNTER_2': msg_speed_2['COUNTER_2'],
  }

  return packer.make_can_msg('SPEED_2', 2, values)

def create_speed_3_message(packer, msg_speed_3: dict):
  """Forward SPEED_3."""
  values = {
    'ALL_BYTES': msg_speed_3['ALL_BYTES'],
  }

  return packer.make_can_msg('SPEED_3', 2, values)

def create_0x1a_message(packer, msg_0x1a: dict):
  """Forward 0x1A."""
  values = {
    'ALL_BYTES': msg_0x1a['ALL_BYTES'],
  }
  return packer.make_can_msg('NEW_MSG_1A', 2, values)

def create_gear_position_message(packer, msg_gear_position: dict):
  """Forward GEAR_POSITION."""
  values = dict(msg_gear_position)
  values['GEAR_POSITION'] = msg_gear_position['GEAR_POSITION']  # 3
  return packer.make_can_msg('GEAR_POSITION', 2, values)

def create_egsm_message(packer, msg_egsm: dict):
  """Forward EGSM."""
  values = {
    'ALL_BYTES': msg_egsm['ALL_BYTES'],
  }
  return packer.make_can_msg('EGSM', 0, values)

def create_pscm_related_message(packer, lat_active: bool, stock_lca_engaged: bool, msg_pscm_related: dict, sig1_counter: int):
  # BO_ 23 PSCM_RELATED: 8 XXX
  # SG_ CHECKSUM : 7|8@0+ (1,0) [0|255] "" XXX
  # SG_ LCA_ENABLED_ECHO : 11|4@0+ (1,0) [0|15] "" XXX
  # SG_ SIG1_BYTE_1_HI_NIBBLE : 15|4@0+ (1,0) [0|15] "" XXX
  # SG_ SIG1_REPLICA_BYTE_2_LO_NIBLE : 19|4@0+ (1,0) [0|15] "" XXX
  # SG_ NEW_SIGNAL_2 : 23|4@0+ (1,0) [0|15] "" XXX
  # SG_ BYTE_3 : 31|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_4_5 : 39|16@0+ (1,0) [0|65535] "" XXX
  # SG_ BYTE_6 : 55|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_7 : 63|8@0+ (1,0) [0|255] "" XXX
  values = dict(msg_pscm_related)

  # SIG1 counter, same value in both places
  values['SIG1_BYTE_1_HI_NIBBLE'] = sig1_counter
  values['SIG1_REPLICA_BYTE_2_LO_NIBLE'] = sig1_counter

  dat = packer.make_can_msg('PSCM_RELATED', 0, values)
  built_bytes = dat[1]  # (addr, bytes, bus)
  b1 = built_bytes[1]
  b2 = built_bytes[2]
  values['CHECKSUM_1'] = checksum_1_pscm_related_message(b1, b2)
  values['CHECKSUM_2'] = checksum_2_pscm_related_message(b2)
  #assert values['CHECKSUM_1'] == msg_pscm_related['CHECKSUM_1']
  #assert values['CHECKSUM_2'] == msg_pscm_related['CHECKSUM_2']
  if lat_active and not stock_lca_engaged:
    values['LCA_ENABLED_ECHO'] = 0
    b1 = packer.make_can_msg('PSCM_RELATED', 0, values)[1][1]
    values['CHECKSUM_1'] = checksum_1_pscm_related_message(b1, b2)
  return packer.make_can_msg('PSCM_RELATED', 0, values)

def create_lca_4_message(packer, lat_active: bool, msg_lca_4: dict, lca_4_steer: int,
                         overrides: dict | None = None):
  """LCA_4 (0x90), holds LCA_ENABLE at 3 so the PSCM sees PA on."""
  if not lat_active:
    # Inactive: relay stock
    return packer.make_can_msg('LCA_4', 2, msg_lca_4)

  # Active: LCA_ENABLE = 3 (PA on)
  values = {
    'BYTE_0': msg_lca_4['BYTE_0'],
    'LCA_ENABLE': 3,  # both bits set
    'BYTE_1_FLAGS': msg_lca_4['BYTE_1_FLAGS'],
    'BYTE_1_NIBBLE_HI': msg_lca_4['BYTE_1_NIBBLE_HI'],
    'BYTE_2_3': msg_lca_4['BYTE_2_3'],
    'YAW_RATE': msg_lca_4['YAW_RATE'],
    'BYTE_6': msg_lca_4['BYTE_6'],
    'BYTE_7_NIBBLE_LO': msg_lca_4['BYTE_7_NIBBLE_LO'],
    'BYTE_7_NIBBLE_HI': msg_lca_4['BYTE_7_NIBBLE_HI'],
  }

  # Live-tuning overrides
  if overrides:
    for key, val in overrides.items():
      values[key] = val

  # TODO checksum, e.g.:
  # values['CHECKSUM'] = checksum_lca_4_message(...)

  # TODO checksum check when inactive
  # if not lat_active and 'CHECKSUM' in msg_lca_4:
  #   if values['CHECKSUM'] != msg_lca_4['CHECKSUM']:
  #     carlog.warning("[volvocan.py] LCA_4 CHECKSUM mismatch")

  return packer.make_can_msg('LCA_4', 2, values)

def create_lca_6_message(packer, lat_active: bool, msg_lca_6: dict, lca_6_steer: int,
                         overrides: dict | None = None):

  if not lat_active:
    # Inactive: relay stock
    return packer.make_can_msg('LCA_6', 2, msg_lca_6)

  values = {
    'LCA_6_STEER': msg_lca_6['LCA_6_STEER'],
    'LCA_6_STEER_2': msg_lca_6['LCA_6_STEER_2'],
    'NEW_SIGNAL_1': msg_lca_6['NEW_SIGNAL_1'],
    'NEW_SIGNAL_2': msg_lca_6['NEW_SIGNAL_2'],
    'NEW_SIGNAL_3': msg_lca_6['NEW_SIGNAL_3'],
    'NEW_SIGNAL_4': msg_lca_6['NEW_SIGNAL_4'],
    'NEW_SIGNAL_5': msg_lca_6['NEW_SIGNAL_5'],
  }

  # Live-tuning overrides
  if overrides:
    for key, val in overrides.items():
      values[key] = val

  return packer.make_can_msg('LCA_6', 2, values)

def create_lca_7_message(packer, lat_active: bool, msg_lca_7: dict, lca_7_steer: int, lca_7_delta_steer: int,
                         overrides: dict | None = None, steer_active: bool = False):

  if not lat_active:
    # Inactive: relay stock
    return packer.make_can_msg('LCA_7', 2, msg_lca_7)

  values = {
    'LCA_7_STEER': msg_lca_7['LCA_7_STEER'],
    'LCA_7_DELTA_STEER': msg_lca_7['LCA_7_DELTA_STEER'],
    'NEW_SIGNAL_1': msg_lca_7['NEW_SIGNAL_1'],
    'NEW_SIGNAL_2': msg_lca_7['NEW_SIGNAL_2'],
    'NEW_SIGNAL_3': msg_lca_7['NEW_SIGNAL_3'],
    'NEW_SIGNAL_4': msg_lca_7['NEW_SIGNAL_4'],
  }

  # Live-tuning overrides
  if overrides:
    for key, val in overrides.items():
      values[key] = val

  return packer.make_can_msg('LCA_7', 2, values)


# ---- V60/EUCD (FSM0-4, PSCM1), from sunnypilot volvo_oplong ----

def create_eucd_button_msg(packer, resume=False, cancel=False, set_plus=False, minus=False, bus=0):
  msg = {
    "ACCOnOffBtn": cancel,
    "ACCOnOffBtnInv": not cancel,
    "ACCResumeBtn": resume,
    "ACCResumeBtnInv": not resume,
    "ACCSetBtn": set_plus,
    "ACCSetBtnInv": not set_plus,
    "ACCMinusBtn": minus,
    "ACCMinusBtnInv": not minus,
  }
  return packer.make_can_msg("CCButtons", bus, msg)


def create_eucd_lkas_state_msg(packer, steering_angle: float, stock_values: dict):
  # Servo state to the camera, LKA torque/active cleared
  msg = {
    "LKATorque": 0,
    "SteeringAngleServo": steering_angle,
    "byte0": stock_values["byte0"],
    "byte4": stock_values["byte4"],
    "byte7": stock_values["byte7"],
    "LKAActive": int(stock_values["LKAActive"]) & 0xF5,
    "EPSTorque": stock_values["EPSTorque"],
  }
  return packer.make_can_msg("PSCM1", 2, msg)


def calculate_eucd_lka_checksum(dat: bytearray) -> int:
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03
  trqlim = dat[2]

  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  return s ^ 0xFF


def create_eucd_lka_msg(packer, apply_steer: float, steer_direction: int):
  values = {
    "LKAAngleReq": apply_steer,
    "LKASteerDirection": steer_direction,
    "TrqLim": 0,

    "SET_X_22": 0x25,
    "SET_X_02": 0,
    "SET_X_10": 0x10,
    "SET_X_A4": 0xa7,
  }

  # Checksum
  dat = packer.make_can_msg("FSM2", 0, values)[1]
  values["Checksum"] = calculate_eucd_lka_checksum(dat)

  return packer.make_can_msg("FSM2", 0, values)


def eucd_next_long_mode(mode: LongMode, accel: float, brake_offset: float = 0.) -> LongMode:
  # brake_offset shifts the BRAKE thresholds by the slope
  CCP = CarControllerParams
  if mode == LongMode.BRAKE:
    if accel <= CCP.EUCD_BRAKE_EXIT + brake_offset:
      return LongMode.BRAKE
  elif mode == LongMode.DRIVE:
    if accel >= CCP.EUCD_DRIVE_EXIT:
      return LongMode.DRIVE
  enter_offset = brake_offset * (1. + CCP.EUCD_UPHILL_BRAKE_ENTER_EXTRA) if brake_offset < 0. else brake_offset
  if accel < CCP.EUCD_BRAKE_ENTER + enter_offset:
    return LongMode.BRAKE
  if accel > CCP.EUCD_DRIVE_ENTER:
    return LongMode.DRIVE
  return LongMode.COAST


def eucd_fsm3_traction(accel: float, mode: LongMode) -> int:
  # FSM3 ACC_TractionRequest
  if mode == LongMode.BRAKE:
    return 0
  if mode == LongMode.DRIVE:
    gain, offset = CarControllerParams.EUCD_TRACTION_DRIVE
    return min(max(round(gain * accel + offset), 0), 255)
  gain, offset = CarControllerParams.EUCD_TRACTION_COAST
  return min(max(round(gain * accel + offset), -127), -1) & 0xFF


def _eucd_copy(stock, skip=()):
  # Stock frame signals, without the ones overlapping others
  return {k: v for k, v in stock.items() if k not in skip}


def create_eucd_longitudinal(packer, stock_fsm3, accel, acc_check, mode: LongMode | None = None):
  # Camera FSM3 with OP's accel, ACC_Check, mode and traction
  values = _eucd_copy(stock_fsm3)
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  if mode is not None:
    values["ACC_Mode"] = int(mode)
    values["ACC_TractionRequest"] = eucd_fsm3_traction(accel, mode)
  return packer.make_can_msg("FSM3", 0, values)


def eucd_fsm4_brake_value(accel: float, brake: bool) -> int:
  # FSM4 brake word: 0x8000 | brake flag 0x4000 | 14-bit target
  CCP = CarControllerParams
  bp, v = (CCP.EUCD_FSM4_BRAKE_ACCEL_BP, CCP.EUCD_FSM4_BRAKE_VALUE_V) if brake else (CCP.EUCD_FSM4_NO_BRAKE_ACCEL_BP, CCP.EUCD_FSM4_NO_BRAKE_VALUE_V)
  return 0x8000 | (0x4000 if brake else 0) | int(round(float(np.interp(accel, bp, v))))


def create_eucd_fsm4(packer, stock_fsm4, accel: float, mode: LongMode | None = None, cam_weight: float = 0.):
  # Camera FSM4; brake flag and target follow OP's accel when either side brakes
  values = _eucd_copy(stock_fsm4)
  op_brake = mode == LongMode.BRAKE
  cam_brake = bool(stock_fsm4["ACC_BrakeRequest"])
  if mode is not None and (op_brake or cam_brake):
    value = eucd_fsm4_brake_value(accel, op_brake)
    if cam_weight > 0. and op_brake and cam_brake:
      # Blend into the camera's target during its stop
      cam_value = int(stock_fsm4["ACC_BrakeTarget"])
      blended = int(round((value & 0x3FFF) + cam_weight * (cam_value - (value & 0x3FFF))))
      value = (value & 0xC000) | blended
    values["Byte_5_b7"] = (value >> 15) & 1
    values["ACC_BrakeRequest"] = (value >> 14) & 1
    values["ACC_BrakeTarget"] = value & 0x3FFF
  return packer.make_can_msg("FSM4", 0, values)


def create_eucd_fsm0(packer, stock_fsm0, ecm_engaged: bool):
  # Camera FSM0, never ACC enabled without the ECM engaged
  values = _eucd_copy(stock_fsm0, skip=("ACC_BrakeAlert",))
  if not ecm_engaged:
    values["ACC_Enabled"] = 0
  return packer.make_can_msg("FSM0", 0, values)


def create_eucd_cam_brake_status(packer, stock_323, mask: bool):
  # 0x323 to the camera; masked clears brake applied
  values = _eucd_copy(stock_323)
  if mask:
    values["BrakeApplied"] = 0
  return packer.make_can_msg("BrakeStatus", CANBUS.cam, values)


def create_eucd_cam_brake_level(packer, stock_133, frozen_level: int | None):
  # 0x133 to the camera; masked holds the brake level
  values = _eucd_copy(stock_133)
  if frozen_level is not None:
    values["BrakeLevel"] = frozen_level & 0x7F
  return packer.make_can_msg("BrakeLevel", CANBUS.cam, values)


def create_eucd_cam_wheel_status(packer, stock_220, mask: bool, spoof: bool = False):
  # 0x220 to the camera; brake request bit masked or spoofed
  values = _eucd_copy(stock_220, skip=("WhlSpdRR", "WhlSpdLR"))
  if mask:
    values["BrakeReqActive"] = 0
  elif spoof:
    values["BrakeReqActive"] = 1
  return packer.make_can_msg("wheelspeed1", CANBUS.cam, values)


def create_eucd_cam_brake_request_ack(packer, stock_299, mask: bool, spoof: bool = False):
  # 0x299 to the camera; brake ack (active low) masked or spoofed
  values = _eucd_copy(stock_299)
  if mask:
    values["BrakeReqNotAck"] = 1
  elif spoof:
    values["BrakeReqNotAck"] = 0
  return packer.make_can_msg("BrakeReqAck", CANBUS.cam, values)


def create_eucd_radar(packer, stock_fsm1, mode: LongMode | None = None):
  # Camera FSM1; ACC_Braking follows OP braking
  values = _eucd_copy(stock_fsm1)
  if mode is not None:
    values["ACC_Braking"] = 1 if mode == LongMode.BRAKE else 0
  return packer.make_can_msg("FSM1", 0, values)
