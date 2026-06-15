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

# Overrides applied when lateral isn't active. We keep streaming the frame every
# valid cycle (like the stock steering module, which sends slot 0x48 at 50 Hz
# regardless of engagement) but disable the request: ACTIVE=INACTIVE and the whole
# assist command zeroed. Matches the stock inactive frame observed on the FlexRay
# bridge RX side (CAN 0x481) in route 0fb664817c575e13/00000007--4b23439cf5.
STEER_DISABLED = {
  "ACTIVE": 1,  # INACTIVE
  "ASSIST_MODE": 0,
  "ASSIST_TORQUE": 0,
  "SET_ME_0xA2": 0,
  "SET_ME_0xFA": 0,
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

  def create_steer_request(self, apply_angle: float, cycle: int, lat_active: bool):
    values = dict(STEER_DEFAULTS)
    if not lat_active:
      values.update(STEER_DISABLED)
    # While disabled, send a zero angle to match the stock inactive frame exactly.
    # TODO: once panda-side angle safety is in place, send the measured angle here
    # instead (smooth handoff) and test that the EPS does not fault when sending a
    # non-zero angle when inactive.
    values["STEER_ANGLE_REQUEST"] = apply_angle if lat_active else 0.0
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
    # We always stream the frame (like the stock module); when lateral isn't active
    # the request is disabled inside the message (ACTIVE=INACTIVE, assist zeroed).
    for cycle in next_trigger_cycles(CS.cycle, CarControllerParams.STEER_LOOKAHEAD):
      can_sends.append(self.create_steer_request(apply_angle, cycle, CC.latActive))

    self.frame += 1
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    return new_actuators, can_sends
