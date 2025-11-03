from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarDocs

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL


@dataclass(frozen=True, kw_only=True)
class BMWCarSpecs(CarSpecs):
  mass: float = 2000.
  wheelbase: float = 3.105
  steerRatio: float = 16.3
  centerToFrontRatio: float = 0.5


@dataclass
class BMWPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'bmw_sp2018'})

@dataclass
class BMWCarDocs(CarDocs):
  name: str = "BMW SP2018"
  package: str = "5AU"
class CAR(Platforms):
  BMW_SP2018 = BMWPlatformConfig(
    [BMWCarDocs()],
    BMWCarSpecs(),
  )


DBC = CAR.create_dbc_map()


# Lateral limits and controller parameters for BMW angle control
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # Assume EPAS faults above this angle; tune with testing
    360,  # deg
    # BMW uses vehicle-model limiting; rate tables unused here
    ([], []),
    ([], []),

    # Vehicle model-based limits (start conservative; adjust after road test)
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.04),  # ~3.4-3.5 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.04),                 # ~3.4-3.5 m/s^3

    # prevent EPS faults and improve low-speed comfort
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  # Angle command is sent every other frame (~50 Hz when DT_CTRL=100 Hz)
  STEER_STEP = 2

