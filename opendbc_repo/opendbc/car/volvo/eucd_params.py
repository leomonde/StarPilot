from enum import IntEnum

from opendbc.car import DT_CTRL
from opendbc.car.lateral import AngleSteeringLimits


class SteerDirection(IntEnum):
  """LKA steer direction; EUCD waits 8 frames on a direction change."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class EcmAccState(IntEnum):
  """ECM ACC state, EngineData Byte_4 bits 5:4."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class CarControllerEUCDParams:
  EUCD_ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )
  EUCD_STEER_TIMEOUT = 30 / DT_CTRL  # frames of 0 torque before the EPS faults
  EUCD_BLOCK_LEN = 8   # frames to block steering on a direction change
  EUCD_DEADZONE = 0.2  # deg, keep the previous direction inside it
  EUCD_MIN_STEER_SPEED = 0.1  # m/s, no steering nor cancel button below this


class CANBUS:
  pt = 0
  body = 1  # aux bus: Delphi ESR radar, CEM dynamics
  cam = 2
