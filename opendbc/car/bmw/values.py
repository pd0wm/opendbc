from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarDocs

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms


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


