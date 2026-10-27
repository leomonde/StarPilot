from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu

# C1 adapted from dragonpilot's V40 port (commit 773dce5)

# V60/EUCD adapted from sunnypilot volvo_oplong


class VolvoSafetyFlags(IntFlag):
  SPA = 1
  C1 = 2
  EUCD = 4


class SteerDirection(IntEnum):
  """LKA steer direction; EUCD waits 8 frames on a direction change."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class LongMode(IntEnum):
  """FSM3 Byte_2 mode bits; the ECM brakes hydraulically only in BRAKE."""
  BRAKE = 0
  COAST = 1
  DRIVE = 2


class EcmAccState(IntEnum):
  """ECM ACC state, EngineData Byte_4 bits 5:4."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class CarControllerParams:
  STEER_STEP = 1  # 100 Hz LCA command

  # Max commanded vs actual steering angle error (deg)
  ANGLE_ERROR = 3.0

  # LCA torque-authority envelope, like stock Pilot Assist
  LCA_AUTH_MAX = 614  # saturation
  LCA_AUTH_PLATEAU_COUNTER = 130  # counter-arm magnitude in override
  # Override enter/exit thresholds on |steeringTorque|
  LCA_AUTH_OVERRIDE_ENTER = 5
  LCA_AUTH_OVERRIDE_EXIT = 3
  # Light-contact haptic yield
  LCA_AUTH_LIGHT_THRESH = 3  # min |drv| for contact
  LCA_AUTH_LIGHT_RISE_DELTA = 1.0  # min |drv| rise per frame
  LCA_AUTH_LIGHT_HOLD_FRAMES = 15  # yield frames (~150 ms)
  LCA_AUTH_LIGHT_COOLDOWN_FRAMES = 30  # quiet frames before re-arm (~300 ms)
  # Yield arm scaled by filtered driver torque
  LCA_AUTH_YIELD_BASE = 60  # yield at the override threshold
  LCA_AUTH_YIELD_SLOPE = 8  # yield drop per |drv| unit
  LCA_AUTH_YIELD_MIN = 0  # min yield
  LCA_AUTH_YIELD_LP_ALPHA = 0.04  # |drv| low-pass alpha
  LCA_AUTH_SPLIT = 200  # symmetric to asymmetric handover
  LCA_AUTH_REBUILD_RATE = 230  # counts/s
  LCA_AUTH_COLLAPSE_RATE = 2500  # counts/s, scales with |drv|

  # Angle rate limits
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    540,  # deg, 1.5 turns to lock
    ([0., 5., 25.], [2.5, 1.5, .2]),  # rate up vs speed
    ([0., 5., 25.], [5., 2., .3]),  # rate down vs speed
  )

  C1_STEER_NO = 0
  C1_STEER = 3
  C1_N_ZERO_TORQUE = 12
  C1_ANGLE_ERROR = 20.0
  C1_ANGLE_DELTA_BP = [0., 8.33, 13.89, 19.44, 25., 30.55, 36.1]
  C1_ANGLE_DELTA_UP = [2., 1.2, .25, .20, .15, .10, .10]
  C1_ANGLE_DELTA_DOWN = [2., 1.2, .25, .20, .15, .10, .10]
  C1_ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    359.9,
    (C1_ANGLE_DELTA_BP, C1_ANGLE_DELTA_UP),
    (C1_ANGLE_DELTA_BP, C1_ANGLE_DELTA_DOWN),
  )

  # ---- V60/EUCD ----
  EUCD_ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )
  EUCD_STEER_TIMEOUT = 30 / DT_CTRL  # frames of 0 torque before the EPS faults
  EUCD_BLOCK_LEN = 8   # frames to block steering on a direction change
  EUCD_DEADZONE = 0.2  # deg, keep the previous direction inside it
  EUCD_BUTTON_BURST = 15  # frames per button press
  EUCD_SNG_OP_RESUME_TIME = 0.5  # s of openpilot resume before SNG

  # Longitudinal limits
  EUCD_ACCEL_MIN = -4.0  # m/s^2
  EUCD_ACCEL_MAX = 2.0   # m/s^2

  # FSM3 mode hysteresis on OP accel
  EUCD_BRAKE_ENTER = -0.50  # m/s^2
  EUCD_BRAKE_EXIT = -0.15
  EUCD_DRIVE_ENTER = 0.08
  EUCD_DRIVE_EXIT = 0.04

  # FSM3 Byte_3 traction = gain * accel + offset, as the camera sends it
  EUCD_TRACTION_COAST = (94., -9.)
  EUCD_TRACTION_DRIVE = (97., -6.)

  # FSM4 14-bit brake target vs accel, with and without brake flag
  EUCD_FSM4_BRAKE_ACCEL_BP = [-3.5, -2.5, -1.75, -1.25, -0.8, -0.45, -0.2, 0.0, 0.2]
  EUCD_FSM4_BRAKE_VALUE_V = [12519, 12850, 12964, 13073, 13143, 13227, 13279, 13328, 13378]
  EUCD_FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  EUCD_FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

  # Camera brake acks, masked while OP brakes alone
  EUCD_CAM_BRAKE_MASK_MSGS = frozenset({"wheelspeed1", "BrakeReqAck"})
  EUCD_CAM_BRAKE_MASK_HOLD_FRAMES = int(1.0 / DT_CTRL)

  EUCD_CAM_ACK_RELEASE_VALUE = 13280  # spoofed BrakeReqAck released at this FSM4 target

  # Camera final stop behind a lead
  EUCD_CAMERA_STOPS_BEHIND_LEAD = True
  EUCD_CAM_STOP_HANDOVER_SPEED = 20. * CV.KPH_TO_MS  # camera stops below this, with a target
  EUCD_CAM_STOP_RETURN_SPEED = 30. * CV.KPH_TO_MS  # OP takes back above this
  EUCD_CAM_STOP_TRANSITION_TIME = 1.0  # s, OP/camera blend
  EUCD_OP_BRAKE_OVERRIDE_ACCEL = -0.3  # OP brakes below this if the camera doesn't
  EUCD_OP_BRAKE_OVERRIDE_MARGIN = 1.0  # OP brakes if this far below the camera

  # FSM3 mirrors camera mode 0; camera hydraulic brake followed below this
  EUCD_CAMERA_BRAKE_HONOR_BELOW = -3.0  # m/s^2
  # With a close camera target, OP accel capped at the camera's, not below the floor
  EUCD_CAMERA_ACCEL_CAP_FLOOR = 0.0  # m/s^2
  EUCD_CAMERA_ACCEL_CAP_GAP = 1.5  # s, camera target closer than this
  EUCD_CAMERA_ACCEL_CAP_MIN_DIST = 10.  # m, at low speed
  # BRAKE thresholds shifted by the slope's coast accel
  EUCD_COAST_PITCH_GAIN = 5.65  # m/s^2 per sin(pitch)
  EUCD_DOWNHILL_BRAKE_OFFSET_MAX = 0.15  # m/s^2, downhill shift cap
  # Extra uphill margin before BRAKE
  EUCD_UPHILL_BRAKE_ENTER_EXTRA = 0.6  # fraction of the uphill offset
  # Downhill coast tolerance over the ACC set speed
  EUCD_COAST_TOLERANCE = 5. * CV.KPH_TO_MS  # brakes from set + this, releases at set
  EUCD_COAST_TOLERANCE_MAX_DECEL = -1.5  # m/s^2, stronger requests brake
  EUCD_COAST_TOLERANCE_LEAD_GAP = 2.5  # s, no tolerance with a closer camera target
  EUCD_COAST_TOLERANCE_AT_SET = 1. * CV.KPH_TO_MS  # below the set, still counted as at set

  # Accel request ramps (jerk limits)
  EUCD_JERK_UP = 2.0  # m/s^3, rising
  EUCD_JERK_DOWN_LIFT = 2.5  # m/s^3, falling while positive
  EUCD_JERK_DOWN_BRAKE = 4.0  # m/s^3, falling at or below 0


class CANBUS:
  pt = 0
  body = 1  # aux bus: Delphi ESR radar, CEM dynamics
  cam = 2


# Delphi ESR 2.5 on the aux bus, read directly
RADAR_ESR = "ESR"


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Pilot Assist & Adaptive Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class VolvoCMAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.main: 'volvo_mid_1',
    Bus.party: 'volvo_mid_1',
    Bus.pt: 'volvo_front_1_cma',
  })

@dataclass
class VolvoSPAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.main: 'volvo_mid_1',
    Bus.party: 'volvo_mid_1',
    Bus.pt: 'volvo_front_1_spa',
  })


@dataclass
class VolvoC1PlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_v40_2017_pt',
    Bus.cam: 'volvo_v40_2017_pt',
  })


@dataclass
class VolvoEUCDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_v60_2015_pt',
    Bus.radar: RADAR_ESR,
  })


@dataclass(frozen=True)
class VolvoEUCDCarSpecs(CarSpecs):
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.44
  minSteerSpeed: float = 0.1


class CAR(Platforms):
  VOLVO_V40 = VolvoC1PlatformConfig(
    [VolvoCarDocs("Volvo V40 2013-19")],
    CarSpecs(
      mass=1610,
      wheelbase=2.647,
      steerRatio=14.7,
      centerToFrontRatio=0.44,
      minSteerSpeed=1.0 * CV.KPH_TO_MS,
    ),
  )

  VOLVO_XC40_RECHARGE = VolvoCMAPlatformConfig(
    [VolvoCarDocs("Volvo XC40 Recharge 2021-23")],
    CarSpecs(
      mass=2170,
      wheelbase=2.702,
      steerRatio=15.8,
      centerToFrontRatio=0.52,
    ),
  )

  VOLVO_S60_RECHARGE = VolvoSPAPlatformConfig(
    [VolvoCarDocs("Volvo S60 Recharge 2024")],
    CarSpecs(
      mass=2020,
      wheelbase=2.872,
      steerRatio=16.2,
      centerToFrontRatio=0.516,
    ),
  )

  # Polestar 2 (CMA) uses the SPA DBC on CAN 1
  POLESTAR_2 = VolvoSPAPlatformConfig(
    [VolvoCarDocs("Polestar 2 2020-25")],
    CarSpecs(
      mass=2123,
      wheelbase=2.735,
      steerRatio=15.8,
      centerToFrontRatio=0.52,
    ),
  )

  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [VolvoCarDocs("Volvo V60 2010-18")],
    VolvoEUCDCarSpecs(mass=1750, wheelbase=2.776),
  )

# FW query for CMA (unused)
# FW_QUERY_CONFIG = FwQueryConfig(
#   requests=[
#     Request(
#       [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
#       [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
#       bus=0,
#     ),
#   ],
# )
VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

# V60 identified by engine FW, V40 by CAN fingerprint
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
