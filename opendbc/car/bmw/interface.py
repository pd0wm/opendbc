from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "bmw"

    # Minimal BMW safety mode: arms controls_allowed off the stock ACC and allowlists
    # the steering inject (see opendbc/safety/modes/bmw.h). It's behind ALLOW_DEBUG,
    # so it requires a debug panda build for now.
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.bmw)]

    # angle-based lateral; longitudinal stays on the stock ACC
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    ret.radarUnavailable = True

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = True

    return ret
