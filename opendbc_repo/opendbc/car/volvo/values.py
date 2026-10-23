from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu

# C1 support is adapted from the original dragonpilot V40 port:
# https://github.com/dragonpilot/dragonpilot/commit/773dce507082d931236b64dca8024dce9625446f

# V60/EUCD support (lateral + longitudinal) is adapted from the sunnypilot volvo_oplong port.


class VolvoSafetyFlags(IntFlag):
  SPA = 1
  C1 = 2
  EUCD = 4


class SteerDirection(IntEnum):
  """LKASteerDirection: EUCD needs an 8-frame wait on direction change."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class LongMode(IntEnum):
  """FSM3 Byte_2 bits 1:0. The ECM uses the hydraulic brake only in BRAKE."""
  BRAKE = 0
  COAST = 1
  DRIVE = 2


class EcmAccState(IntEnum):
  """EngineData Byte_4 bits 5:4, the ECM's ACC state."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class CarControllerParams:
  STEER_STEP = 1  # 100 Hz LCA command frequency (controlsd runs at 100 Hz)

  # Max commanded-vs-actual steering angle error (deg). Stock Volvo Pilot Assist holds
  # commanded within ~1.7° of actual even under sustained driver override; bounding the
  # command to actual ± this error prevents the stale-command snap-back that causes
  # aggressive post-release overcorrection.
  ANGLE_ERROR = 3.0

  # LCA torque-authority envelope, modeled after stock Pilot Assist behavior.
  # LCA_STEER_LOOSELY (positive arm) and LCA_STEER_LOOSELY_INV (negative arm)
  # form a directional envelope that PSCM applies to its EPS torque. Stock PA:
  #  - holds both arms at saturation (±LCA_AUTH_MAX) when no driver torque
  #  - on driver override, collapses both arms symmetrically at COLLAPSE_RATE
  #    until envelope reaches ~±LCA_AUTH_SPLIT, then splits asymmetrically:
  #    the arm matching driver direction (yielding) settles at ±PLATEAU_YIELD,
  #    the counter arm holds at ±PLATEAU_COUNTER (yield is shallower than counter)
  #  - rebuilds at REBUILD_RATE after release (~3 s back to saturation)
  # See route_analysis/lca_override_mechanism.md for the data behind these.
  LCA_AUTH_MAX = 614              # signal saturation
  LCA_AUTH_PLATEAU_COUNTER = 130  # counter-arm magnitude during sustained override
  # Override trigger thresholds on |CS.out.steeringTorque| (op-convention raw
  # units, mirror of DRIVER_INPUT). Must be ABOVE the resting-hand noise floor
  # (CS.steeringPressed uses |raw|>2 as a sensitive DM-fallback floor and does
  # NOT indicate override intent — don't use it for envelope triggering).
  # Hysteresis: enter override at ENTER, exit at EXIT (< ENTER) to prevent the
  # envelope flapping between collapse and rebuild when driver torque hovers
  # near a single threshold (was causing ~10 Hz EPS-torque ripple in lane
  # changes when driver applied 6-8 raw to "ride along" with op).
  LCA_AUTH_OVERRIDE_ENTER = 5
  LCA_AUTH_OVERRIDE_EXIT = 3
  # "Light contact" / haptic-acknowledgment region. When |drv| crosses into
  # [LIGHT_THRESH, OVERRIDE_THRESH] from below, briefly collapse the envelope
  # for LIGHT_HOLD_FRAMES (a haptic confirmation of hand-on-wheel detection),
  # then rebuild even while the contact persists. Prevents the driver from
  # needing to sustain force just to feel that the system noticed them — helps
  # with hand-fatigue / RSI.
  # Rising edge detected via per-frame derivative; the brief-yield window does
  # NOT re-arm while still active, so a steady elevated torque only triggers
  # one yield and then the envelope rebuilds.
  # Cooldown: light_collapse only fires when real_override has been off for
  # LIGHT_COOLDOWN_FRAMES — suppresses repeated firings during active
  # co-steering (lane changes), where |drv| oscillates and would otherwise
  # re-arm the haptic-ack window each time, causing felt ripple.
  LCA_AUTH_LIGHT_THRESH = 3            # min |drv| to consider as contact
  LCA_AUTH_LIGHT_RISE_DELTA = 1.0      # min per-frame increase in |drv| to count as rising contact
  LCA_AUTH_LIGHT_HOLD_FRAMES = 15      # ~150 ms of yield on fresh light contact
  LCA_AUTH_LIGHT_COOLDOWN_FRAMES = 30  # ~300 ms quiet-time on real_override before light contact re-arms
  # Yield-arm plateau scales with driver-torque magnitude so brief strong presses
  # (potholes, lane corrections) get full yield while light sustained pressure
  # only gets a soft yield. yield_signed = YIELD_BASE − YIELD_SLOPE *
  # max(0, drv_mag_filt − OVERRIDE_ENTER), clamped to [YIELD_MIN, YIELD_BASE].
  # drv_mag_filt is a low-pass of |drv| (alpha=0.04, ~250 ms time constant) —
  # without it, 1-2 unit driver-torque jitter became ~10 unit yield-arm jitter
  # which PSCM converted to felt ripple at sustained co-steering pressure.
  LCA_AUTH_YIELD_BASE = 60        # yield-arm magnitude at the override threshold
  LCA_AUTH_YIELD_SLOPE = 8        # counts of yield reduction per unit |drv torque| above threshold
  LCA_AUTH_YIELD_MIN = 0          # yield authority without crossing the safety sign boundary
  LCA_AUTH_YIELD_LP_ALPHA = 0.04  # LP-filter coefficient on |drv| for yield calc (~250 ms tau)
  LCA_AUTH_SPLIT = 200            # symmetric → asymmetric handover
  LCA_AUTH_REBUILD_RATE = 230     # counts/s (≈ 2.7 s rebuild from 0 to 614)
  LCA_AUTH_COLLAPSE_RATE = 2500   # counts/s base (scales with |drv|/THRESH for sharper pothole jolts)

  # Angle limits for rate limiting
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    540, # deg - 1.5 turns to lock
    ([0., 5., 25.], [2.5, 1.5, .2]),  # rate up limits at different speeds
    ([0., 5., 25.], [5., 2., .3]),    # rate down limits at different speeds
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
  EUCD_SNG_OP_RESUME_TIME = 0.5  # s of openpilot's resume request before it triggers the SNG resume

  # Longitudinal limits
  EUCD_ACCEL_MIN = -4.0  # m/s^2
  EUCD_ACCEL_MAX = 2.0   # m/s^2

  # FSM3 mode hysteresis on OP accel (fit to the stock camera)
  EUCD_BRAKE_ENTER = -0.50  # m/s^2
  EUCD_BRAKE_EXIT = -0.15
  EUCD_DRIVE_ENTER = 0.08
  EUCD_DRIVE_EXIT = 0.04

  # FSM4 14-bit brake target vs accel (stock camera means), with and without the brake flag
  EUCD_FSM4_BRAKE_ACCEL_BP = [-3.5, -2.5, -1.75, -1.25, -0.8, -0.45, -0.2, 0.0, 0.2]
  EUCD_FSM4_BRAKE_VALUE_V = [12519, 12850, 12964, 13073, 13143, 13227, 13279, 13328, 13378]
  EUCD_FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  EUCD_FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

  # Brake acks to the camera (0x220 Byte_2 bit2, 0x299 Byte_5 bit3): masked while OP brakes alone, and this long after
  EUCD_CAM_BRAKE_MASK_MSGS = frozenset({0x220, 0x299})
  EUCD_CAM_BRAKE_MASK_HOLD_FRAMES = int(1.0 / DT_CTRL)

  # Camera brake requests: OP ignores mild ones (spoofed ack), honors them below CAMERA_BRAKE_HONOR_BELOW
  EUCD_OP_IGNORES_MILD_CAMERA_BRAKE = True
  EUCD_CAMERA_BRAKE_HONOR_BELOW = -2.0  # m/s^2
  EUCD_CAM_ACK_RELEASE_VALUE = 13280  # spoofed 0x299 ack drops when the camera's FSM4 target reaches this (no braking)

  # Camera final stop behind a lead
  EUCD_CAMERA_STOPS_BEHIND_LEAD = True
  EUCD_CAM_STOP_HANDOVER_SPEED = 10. * CV.KPH_TO_MS  # camera takes over below this, with a target
  EUCD_CAM_STOP_RETURN_SPEED = 15. * CV.KPH_TO_MS    # OP takes back above this
  EUCD_CAM_STOP_TRANSITION_TIME = 1.0  # s, blend between OP and camera
  EUCD_OP_BRAKE_OVERRIDE_ACCEL = -0.3  # m/s^2, OP brakes if it wants less than this and the camera does not brake
  EUCD_OP_BRAKE_OVERRIDE_MARGIN = 1.0  # m/s^2, OP brakes if it wants this much more than the camera

  # FSM3 shows the camera's BRAKE mode while both decelerate and OP coasts (routes 849/84b: the camera
  # aborted after ~4 s of its BRAKE vs OP's COAST). FSM3 only: FSM4 and the brake-ack mask stay on COAST.
  EUCD_FSM3_FOLLOWS_CAMERA_BRAKE_MODE = True

  # OP may speed up against the camera only briefly: with a target, camera slowing down and OP speeding up for
  # the grace time, OP's accel is brought to the camera's (route 852: the camera aborted after 3 s of this)
  EUCD_CAMERA_SLOWDOWN_LIMIT = True
  EUCD_CAMERA_SLOWDOWN_ACCEL = -0.15  # m/s^2, camera "slowing down" below this
  EUCD_CAMERA_SLOWDOWN_GRACE = 1.5    # s of disagreement before following the camera
  EUCD_CAMERA_SLOWDOWN_RAMP = 0.5     # s, blend in and out

  # Keep the ECM engaged on the camera's last good frames after a camera abort (latched a fault in testing)
  EUCD_OP_OWNS_ACC_AFTER_CAMERA_ABORT = False


class CANBUS:
  pt = 0
  body = 1  # aux bus — carries Delphi ESR 2.5 radar (0x500..0x53F) and CEM dynamics
  cam = 2


# Delphi ESR 2.5 on aux bus (0x500-0x53F, 20Hz, 64 tracks); read directly so planner sees leads independently of stock FSM
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
  minSteerSpeed: float = 1.0


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

  # Polestar 2 is technically CMA, but appears to use SPA DBC for CAN 1 bus
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

# FW Query configuration for Volvo CMA platform
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

# V60/EUCD is identified by the engine FW (its CAN fingerprint varies with equipment); V40/C1 by CAN fingerprint
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
