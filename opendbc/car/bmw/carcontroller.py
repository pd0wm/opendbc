from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.bmw.values import (CarControllerParams, BMW_FLEXRAY_CYCLES, BMW_STEER_CYCLE_MOD,
                                    BMW_STEER_CYCLE_REM, BMW_FLEXRAY_WORDS, BMW_STEER_CRC_INIT,
                                    BMW_STEER_LEN, BMW_BUS, BMW_STEER_COUNTER_MOD, BMW_STEER_CRC_RANGE,
                                    bmw_inject_slot)

# Constant filler that makes the EPS treat the frame as an active steering request.
# Everything except the angle, rolling counter, FlexRay cycle and checksum is fixed.
STEER_DEFAULTS = {
  "DIRECTION": 0,
  "LENGTH": BMW_FLEXRAY_WORDS,
  "CHECKSUM": 0,
  "COUNTER": 0,
  "SET_ME_0x9": 0x9,
  "STEER_ANGLE_REQUEST": 0.0,
  "STEER_ANGLE_RATE_REQUEST": 0.0,
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


def steer_cycles_between(prev: int, cur: int) -> int:
  """Number of steering cycles (cycle % 4 == 1) in (prev, cur], wrapping mod 64.

  Used to advance the free-running rolling counter by exactly one per steering
  cycle the bus has passed, independent of how many control frames ran.
  """
  delta = (cur - prev) % BMW_FLEXRAY_CYCLES
  return sum((prev + k) % BMW_STEER_CYCLE_MOD == BMW_STEER_CYCLE_REM for k in range(1, delta + 1))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.crc = mk_crc8_fun(CRC8J1850, init_crc=BMW_STEER_CRC_INIT)
    self.apply_angle_last = 0.0
    self.VM = VehicleModel(CP)
    self.steer_counter = 0
    self.last_cycle: int | None = None

  def create_steer_request(self, apply_angle: float, apply_rate: float, cycle: int, counter: int, lat_active: bool):
    values = dict(STEER_DEFAULTS)
    if not lat_active:
      values.update(STEER_DISABLED)

    # Track the measured angle while disabled (apply_steer_angle_limits_vm returns the
    # measured angle when inactive) so engaging doesn't step away from it and trip the
    # panda angle rate limit. The panda safety requires STEER_ANGLE_REQUEST to stay close
    # to the measured angle whenever ACTIVE != 2 (see opendbc/safety/modes/bmw.h).
    # TODO: bench test that the EPS does not fault on a non-zero inactive angle request.
    values["STEER_ANGLE_REQUEST"] = apply_angle
    # Angle-rate feedforward (deg/s): d(angle)/dt. Zeroed when inactive to match the
    # stock inactive frame (the panda safety does not check this signal).
    values["STEER_ANGLE_RATE_REQUEST"] = apply_rate if lat_active else 0.0
    values["CYCLE_COUNT"] = cycle & 0x3F
    values["COUNTER"] = counter

    addr, dat, bus = self.packer.make_can_msg("STEER_REQUEST", BMW_BUS, values)
    dat = bytearray(dat)

    # TODO: have CANPacker compute the CRC
    ci, start, end = BMW_STEER_CRC_RANGE
    dat[ci] = self.crc(bytes(dat[start:end]))

    # sent as CAN-FD; pad up to the valid DLC length (already 20 from the packer)
    dat += bytes(BMW_STEER_LEN - len(dat))

    return [bmw_inject_slot(addr), bytes(dat), bus]

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_angle = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                              CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
    # STEER_ANGLE_RATE_REQUEST feedforward: derivative of the (already rate-limited)
    # commanded angle. Control runs every frame at 100 Hz (STEER_STEP=1) so dt == DT_CTRL.
    apply_rate = (apply_angle - self.apply_angle_last) / DT_CTRL
    self.apply_angle_last = apply_angle

    # Advance the free-running rolling counter to the current bus cycle: +1 per
    # steering cycle passed since we last ran (usually 0-1 at 100 Hz vs the 50 Hz
    # steering cadence). steer_counter is now the value for the most recent
    # steering cycle <= CS.cycle; the i-th look-ahead cycle is i+1 steps later.
    if self.last_cycle is not None:
      self.steer_counter = (self.steer_counter + steer_cycles_between(self.last_cycle, CS.cycle)) % BMW_STEER_COUNTER_MOD
    self.last_cycle = CS.cycle

    # Queue the next few valid FlexRay cycles with the freshest angle. The bridge
    # injects each frame on its tagged cycle and overwrites still-pending payloads.
    # Each target cycle gets its own counter (consecutive, mod 15) so the injected
    # stream the EPS sees increments by exactly one every steering cycle. Re-queues
    # of the same target carry the same counter: as CS.cycle advances one step,
    # steer_counter +1 and the target's look-ahead index -1 cancel out.
    # We always stream the frame (like the stock module); when lateral isn't active
    # the request is disabled inside the message (ACTIVE=INACTIVE, assist zeroed).
    for i, cycle in enumerate(next_trigger_cycles(CS.cycle, CarControllerParams.STEER_LOOKAHEAD)):
      counter = (self.steer_counter + i + 1) % BMW_STEER_COUNTER_MOD
      can_sends.append(self.create_steer_request(apply_angle, apply_rate, cycle, counter, CC.latActive))

    self.frame += 1
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    return new_actuators, can_sends
