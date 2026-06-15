from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import (CarControllerParams, BMW_FLEXRAY_CYCLES, BMW_STEER_CYCLE_MOD,
                                    BMW_STEER_CYCLE_REM, BMW_FLEXRAY_WORDS, BMW_STEER_CRC_INIT,
                                    BMW_STEER_LEN, BMW_BUS)

# Constant filler that makes the EPS treat the frame as an active steering request.
# Everything except the angle, rolling counter, FlexRay cycle and checksum is fixed.
STEER_DEFAULTS = {
  "DIRECTION": 0,
  "LENGTH": BMW_FLEXRAY_WORDS,
  "CHECKSUM": 0,
  "COUNTER": 0,
  "SET_ME_0x9": 0x9,
  "STEER_ANGLE_REQUEST": 0.0,
  "STEER_TORQUE_REQUEST": 0.0,
  "REVERSING_ASSIST": 0,
  "ASSIST_TORQUE": 0xA0,
  "ASSIST_MODE": 0x1,
  "SET_ME_0xFE": 0xFE,
  "SET_ME_0x17": 0x17,
  "SET_ME_0xFF": 0xFF,
  "SET_ME_0x3": 0x3,
  "ACTIVE": 2,
  "SET_ME_0xA2": 0xA2,
  "SET_ME_0xFA": 0xFA,
}


def next_trigger_cycles(cur: int, n: int) -> list[int]:
  """The next `n` future FlexRay cycles (wrapping) where cycle % 4 == 1."""
  out = []
  for i in range(1, BMW_FLEXRAY_CYCLES + 1):
    c = (cur + i) % BMW_FLEXRAY_CYCLES
    if c % BMW_STEER_CYCLE_MOD == BMW_STEER_CYCLE_REM:
      out.append(c)
      if len(out) == n:
        break
  return out


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.crc = mk_crc8_fun(CRC8J1850, init_crc=BMW_STEER_CRC_INIT)
    self.apply_angle_last = 0.0
    self.VM = VehicleModel(CP)

  def create_steer_request(self, apply_angle: float, cycle: int):
    values = dict(STEER_DEFAULTS)
    values["STEER_ANGLE_REQUEST"] = apply_angle
    values["CYCLE_COUNT"] = cycle & 0x3F
    # 4-bit rolling counter, one step per valid (cycle % 4 == 1) instance
    values["COUNTER"] = (cycle >> 2) & 0xF

    addr, dat, bus = self.packer.make_can_msg("STEER_REQUEST", BMW_BUS, values)
    dat = bytearray(dat)
    # CHECKSUM (FlexRay payload byte 0, at byte index 2) = crc8 over the bytes after it
    dat[2] = self.crc(bytes(dat[3:]))
    # the 18-byte frame is sent as CAN-FD; pad up to the next valid DLC length (20)
    dat += bytes(BMW_STEER_LEN - len(dat))
    return [addr, bytes(dat), bus]

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_angle = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                              CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
    self.apply_angle_last = apply_angle

    # Queue the next few valid FlexRay cycles with the freshest angle. The bridge
    # injects each frame on its tagged cycle and overwrites still-pending payloads.
    if CC.latActive:
      for cycle in next_trigger_cycles(CS.cycle, CarControllerParams.STEER_LOOKAHEAD):
        can_sends.append(self.create_steer_request(apply_angle, cycle))

    self.frame += 1
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    return new_actuators, can_sends
