from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import CAR, DBC, VolvoC1PlatformConfig, VolvoEUCDPlatformConfig, VolvoSafetyFlags, VolvoSPAPlatformConfig

TransmissionType = structs.CarParams.TransmissionType

SAFETY_VOLVO = structs.CarParams.SafetyModel.volvo


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  # Binds the ESR radar interface
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'volvo'

    platform = CAR(candidate).config
    is_eucd = isinstance(platform, VolvoEUCDPlatformConfig)
    safety_param = 0
    if isinstance(platform, VolvoSPAPlatformConfig):
      safety_param = VolvoSafetyFlags.SPA.value
    elif isinstance(platform, VolvoC1PlatformConfig):
      safety_param = VolvoSafetyFlags.C1.value
    elif is_eucd:
      safety_param = VolvoSafetyFlags.EUCD.value
    ret.safetyConfigs = [get_safety_config(SAFETY_VOLVO, safety_param)]
    #ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.2 if isinstance(platform, (VolvoC1PlatformConfig, VolvoEUCDPlatformConfig)) else 0.3
    ret.steerLimitTimer = 0.8 if is_eucd else 0.1
    ret.steerAtStandstill = not isinstance(platform, VolvoC1PlatformConfig)

    # Angle-based steering
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # V60/EUCD: Delphi ESR 2.5 decoded directly
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = is_eucd
    if is_eucd:
      # OP long: stock ACC engages and owns the set speed; OP replaces FSM3
      ret.openpilotLongitudinalControl = alpha_long
      if ret.openpilotLongitudinalControl:
        ret.longitudinalActuatorDelay = 0.5  # FSM3 accel to aEgo delay
        # Integral-only accel loop
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [0.5]

    ret.pcmCruise = True

    if isinstance(platform, VolvoC1PlatformConfig):
      ret.transmissionType = TransmissionType.automatic

    return ret
