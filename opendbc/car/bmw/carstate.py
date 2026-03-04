from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

    can_define = CANDefine("bmw_sp2018")
    self.shifter_values = can_define.dv["GEARBOX_2"]["GEAR"]

    # Use CarStateBase.out / out_sp as rolling previous-state buffers
  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # External panda is index 1 -> buses 4-7. Use bus 4 for main traffic.
    cp_main = CANParser("bmw_sp2018", [
       ("WHEEL_SPEEDS", float("nan")),
       ("GEARBOX_2", float("nan")),
       ("STEERING_WHEEL_3", float("nan")),
       ("BRAKE_PEDAL_3", float("nan")),
      ], bus=4)

    # One-time DBC config; avoid doing this in the control loop
    cp_main.dbc.name_to_msg["WHEEL_SPEEDS"].ignore_checksum = True
    cp_main.dbc.name_to_msg["WHEEL_SPEEDS"].ignore_counter = True

    cp_main.dbc.name_to_msg["GEARBOX_2"].ignore_checksum = True
    cp_main.dbc.name_to_msg["GEARBOX_2"].ignore_counter = True

    cp_main.dbc.name_to_msg["STEERING_WHEEL_3"].ignore_checksum = True
    cp_main.dbc.name_to_msg["STEERING_WHEEL_3"].ignore_counter = True


    cp_main.dbc.name_to_msg["BRAKE_PEDAL_3"].ignore_checksum = True
    cp_main.dbc.name_to_msg["BRAKE_PEDAL_3"].ignore_counter = True

    cp_sas = CANParser("bmw_sp2018", [("CRUISE_STATE", float("nan"))], bus=5)

    cp_sas.dbc.name_to_msg["CRUISE_STATE"].ignore_checksum = True
    cp_sas.dbc.name_to_msg["CRUISE_STATE"].ignore_counter = True

    return {
      Bus.main: cp_main,
      Bus.adas: cp_sas,
    }

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    cp_sas = can_parsers[Bus.adas]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret, cp.vl["WHEEL_SPEEDS"]["FL"], cp.vl["WHEEL_SPEEDS"]["FR"], cp.vl["WHEEL_SPEEDS"]["RL"], cp.vl["WHEEL_SPEEDS"]["RR"], unit=1)
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = float(cp.vl["STEERING_WHEEL_3"]["STEERING_ANGLE"])

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEARBOX_2"]["GEAR"], None))
    if (ret.gearShifter == GearShifter.neutral) and bool(cp.vl["GEARBOX_2"]["PARK_LOCKED"]):
      ret.gearShifter = GearShifter.park

    ret.cruiseState.enabled = bool(cp_sas.vl["CRUISE_STATE"]["CRUISE_ENGAGED_1"])
    ret.cruiseState.available = True

    ret.brakePressed = bool(cp.vl["BRAKE_PEDAL_3"]["BRAKE_RESSED_1"]) # TODO fix typo in signal name

    return ret, ret_sp
