from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw.values import DBC, BMW_BUS, CarControllerParams

GearShifter = structs.CarState.GearShifter

# Every FlexRay frame carries the live cycle in CYCLE_COUNT. We read it from the
# freshest main-bus frame so the CarController can time its steering injects.
CYCLE_MSGS = ("WHEEL_SPEEDS", "STEERING_WHEEL_2", "GEARBOX_2", "BRAKE_PEDAL_3")


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEARBOX_2"]["GEAR"]
    # latest FlexRay cycle observed on the main bus
    self.cycle = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    # Wheel Speeds
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"], cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"], cp.vl["WHEEL_SPEEDS"]["RR"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    # Steering
    ret.steeringAngleDeg = float(cp.vl["STEERING_WHEEL_2"]["STEERING_ANGLE_1"])
    ret.steeringTorque = float(cp.vl["STEERING_WHEEL_5"]["STEERING_TORQUE_1"])

    # STEERING_WHEEL2.HANDS_OFF_WHEEL is measured using a capacitive sensor, cannot be used for detecting user override
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)

    # Gear
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEARBOX_2"]["GEAR"], None))
    if ret.gearShifter == GearShifter.neutral and cp.vl["GEARBOX_2"]["PARK_LOCKED"]:
      ret.gearShifter = GearShifter.park

    # Brake (BRAKE_PRESSED_1 is the most sensitive brake-pressed signal)
    ret.brakePressed = bool(cp.vl["BRAKE_PEDAL_3"]["BRAKE_PRESSED_1"])

    # Cruise State is handled by the stock ACC
    ret.cruiseState.enabled = bool(cp.vl["CRUISE_STATE"]["CRUISE_ENGAGED_1"])
    ret.cruiseState.available = True

    ret.leftBlinker, ret.rightBlinker = bool(cp.vl["BLINKER_STATE"]["LEFT"]), bool(cp.vl["BLINKER_STATE"]["RIGHT"])

    # Track the live FlexRay cycle from the freshest frame that carries it
    newest_ts = -1
    for msg in CYCLE_MSGS:
      ts = cp.ts_nanos[msg]["CYCLE_COUNT"]
      if ts > newest_ts:
        newest_ts = ts
        self.cycle = int(cp.vl[msg]["CYCLE_COUNT"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # TODO: add checksum
    messages = [
      ("WHEEL_SPEEDS", 50),
      ("STEERING_WHEEL_2", 50),
      ("STEERING_WHEEL_5", 100),
      ("GEARBOX_2", 50),
      ("BRAKE_PEDAL_3", 50),
      ("CRUISE_STATE", 50),
      ("BLINKER_STATE", 1),
    ]
    return {Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, BMW_BUS)}
