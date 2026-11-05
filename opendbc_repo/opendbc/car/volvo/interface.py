from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import CAR, DBC, VolvoC1PlatformConfig, VolvoEUCDPlatformConfig, VolvoSPAPlatformConfig, VolvoSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface  # Delphi ESR 2.5 on the V60; the others have no radar DBC

  def __init__(self, CP):
    config = CAR(CP.carFingerprint).config
    if isinstance(config, VolvoEUCDPlatformConfig):
      from opendbc.car.volvo.eucd_carstate import CarStateEUCD
      from opendbc.car.volvo.eucd_carcontroller import CarControllerEUCD
      self.CarState = CarStateEUCD
      self.CarController = CarControllerEUCD
    elif not isinstance(config, VolvoC1PlatformConfig):
      from opendbc.car.volvo.cma_carstate import CarStateCMA
      from opendbc.car.volvo.cma_carcontroller import CarControllerCMA
      self.CarState = CarStateCMA
      self.CarController = CarControllerCMA
    super().__init__(CP)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = 'volvo'
    platform = CAR(candidate).config
    c1 = isinstance(platform, VolvoC1PlatformConfig)
    eucd = isinstance(platform, VolvoEUCDPlatformConfig)
    safety_param = VolvoSafetyFlags.C1.value if c1 else VolvoSafetyFlags.SPA.value if isinstance(platform, VolvoSPAPlatformConfig) else 0
    if eucd:
      safety_param = VolvoSafetyFlags.EUCD.value
    if candidate == CAR.POLESTAR_2:
      safety_param |= VolvoSafetyFlags.LCA2_MOD15.value
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo, safety_param)]
    ret.steerActuatorDelay = 0.2 if (c1 or eucd) else 0.3
    ret.steerLimitTimer = 0.8 if eucd else 0.1
    ret.steerAtStandstill = not c1
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = Bus.radar not in DBC[candidate]
    ret.alphaLongitudinalAvailable = eucd
    if eucd:
      # OP long: stock ACC engages and owns the set speed; OP replaces FSM3
      ret.openpilotLongitudinalControl = alpha_long
      if ret.openpilotLongitudinalControl:
        ret.longitudinalActuatorDelay = 0.5  # FSM3 accel to aEgo delay
        # Integral-only accel loop, the car brakes short of OP's request (route 89d)
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [0.2]
    ret.pcmCruise = True
    if c1:
      ret.transmissionType = structs.CarParams.TransmissionType.automatic
    return ret
