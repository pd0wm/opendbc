#!/usr/bin/env python3
import math
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.car.bmw.interface import CarInterface
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_SAMPLE_VALS

# addresses / constants mirrored from opendbc/safety/modes/bmw.h
MSG_CRUISE_STATE = 931
MSG_BRAKE_PEDAL_3 = 753
MSG_WHEEL_SPEEDS = 736
MSG_STEERING_WHEEL_2 = 945
MSG_STEER_REQUEST = 72  # bare FlexRay slot 0x48 the inject is re-addressed to

BMW_BUS = 0
BMW_STEER_ACTIVE = 2
BMW_WHEEL_STANDSTILL = 4
BMW_FLEXRAY_CYCLES = 64
BMW_STEER_CYCLE_MOD = 4
BMW_STEER_CYCLE_REM = 1
BMW_STEER_TX_FUTURE_WINDOW = 8
BMW_CYCLE_HZ = 200

# STEER_ANGLE_REQUEST / STEERING_ANGLE_1 DBC scale (0.04395 deg/LSB)
DEG_TO_CAN = 1.0 / 0.04395
STEER_ANGLE_MAX = 360


class TestBmwSafety(common.CarSafetyTest):
  TX_MSGS = [[MSG_STEER_REQUEST, BMW_BUS]]
  # the bridge runs single-bus with no camera: no relay malfunction detection, no forwarding
  RELAY_MALFUNCTION_ADDRS: dict = {}
  FWD_BLACKLISTED_ADDRS: dict = {}
  FWD_BUS_LOOKUP: dict = {}

  STANDSTILL_THRESHOLD = 0.0

  def setUp(self):
    self.packer = CANPackerSafety("bmw_sp2018")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.bmw, 0)
    self.safety.init_tests()
    self.VM = VehicleModel(CarInterface.get_non_essential_params("BMW_SP2018"))

  # *** message builders ***

  def _angle_cmd_msg(self, angle: float, active: bool, cycle: int = BMW_STEER_CYCLE_REM,
                     reversing: int = 0, rate: float = 0.):
    # packed from the report-address DBC message, then re-addressed to the bare slot (like the carcontroller)
    values = {
      "STEER_ANGLE_REQUEST": angle,
      "STEER_ANGLE_RATE_REQUEST": rate,
      "ACTIVE": BMW_STEER_ACTIVE if active else 1,
      "CYCLE_COUNT": cycle,
      "REVERSING_ASSIST": reversing,
    }
    _, dat, _ = self.packer.make_can_msg("STEER_REQUEST", BMW_BUS, values)
    return libsafety_py.make_CANPacket(MSG_STEER_REQUEST, BMW_BUS, dat)

  def _angle_meas_msg(self, angle: float, cycle: int = 0):
    return self.packer.make_can_msg_safety("STEERING_WHEEL_2", BMW_BUS, {"STEERING_ANGLE_1": angle, "CYCLE_COUNT": cycle})

  def _speed_msg(self, speed: float, cycle: int = 0):
    v = speed * 3.6  # km/h
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", BMW_BUS, {"FL": v, "FR": v, "RL": v, "RR": v, "CYCLE_COUNT": cycle})

  def _speed_msg_2(self, speed: float):
    return None  # single speed source

  def _vehicle_moving_msg(self, speed: float, cycle: int = 0):
    v = speed * 3.6  # km/h
    state = 1 if speed > self.STANDSTILL_THRESHOLD else BMW_WHEEL_STANDSTILL
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", BMW_BUS,
      {"FL": v, "FR": v, "RL": v, "RR": v, "FL_STATE": state, "FR_STATE": state, "RL_STATE": state, "RR_STATE": state,
       "CYCLE_COUNT": cycle})

  def _pcm_status_msg(self, enable: bool, cycle: int = 0):
    return self.packer.make_can_msg_safety("CRUISE_STATE", BMW_BUS, {"CRUISE_ENGAGED_1": enable, "CYCLE_COUNT": cycle})

  def _user_brake_msg(self, brake: bool, cycle: int = 0):
    return self.packer.make_can_msg_safety("BRAKE_PEDAL_3", BMW_BUS, {"BRAKE_PRESSED_1": brake, "CYCLE_COUNT": cycle})

  def _user_gas_msg(self, gas):
    # BMW does not monitor gas (stock ACC handles gas override); see skipped gas tests below
    return None

  # *** helpers ***

  def _reset_angle_measurement(self, angle: float):
    for _ in range(MAX_SAMPLE_VALS):
      self._rx(self._angle_meas_msg(angle))

  def _reset_speed_measurement(self, speed: float):
    for _ in range(MAX_SAMPLE_VALS):
      self._rx(self._speed_msg(speed))

  def _set_rx_cycle(self, cycle: int, speed: float = 11.):
    self._rx(self._speed_msg(speed, cycle=cycle))

  def _reset_state(self):
    # set_safety_hooks re-runs bmw_init, which resets the per-cycle angle history / rate window
    self._reset_safety_hooks()
    self.safety.init_tests()

  def _max_angle_delta(self, speed: float) -> float:
    # safety jerk allowance over one steering cycle (BMW_STEER_CYCLE_MOD / 200 Hz = 20 ms)
    return self._max_angle_rate(speed) * (BMW_STEER_CYCLE_MOD / BMW_CYCLE_HZ)

  def _max_angle_rate(self, speed: float) -> float:
    max_curvature_rate = MAX_LATERAL_JERK / (speed ** 2)
    return math.degrees(self.VM.get_steer_from_curvature(max_curvature_rate, speed, 0))

  def _next_steer_cycles(self, cur: int, n: int):
    cycles = []
    for i in range(1, BMW_FLEXRAY_CYCLES + 1):
      cycle = (cur + i) % BMW_FLEXRAY_CYCLES
      if cycle % BMW_STEER_CYCLE_MOD == BMW_STEER_CYCLE_REM:
        cycles.append(cycle)
        if len(cycles) == n:
          break
    return cycles

  def _prev_steer_cycles(self, cur: int, n: int):
    cycles = []
    for i in range(1, BMW_FLEXRAY_CYCLES + 1):
      cycle = (cur - i) % BMW_FLEXRAY_CYCLES
      if cycle % BMW_STEER_CYCLE_MOD == BMW_STEER_CYCLE_REM:
        cycles.append(cycle)
        if len(cycles) == n:
          break
    return cycles

  def test_prev_gas(self):
    raise unittest.SkipTest("TODO")

  def test_allow_engage_with_gas_pressed(self):
    raise unittest.SkipTest("TODO")

  def test_no_disengage_on_gas(self):
    raise unittest.SkipTest("TODO")

  # *** angle safety tests ***

  def test_steering_angle_measurements(self):
    self._common_measurement_test(self._angle_meas_msg, -STEER_ANGLE_MAX, STEER_ANGLE_MAX, DEG_TO_CAN,
                                  self.safety.get_angle_meas_min, self.safety.get_angle_meas_max)

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 80, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_angle_cmd_rate_limit(self):
    # The command for each steering cycle is rate limited against the angle committed for the
    # previous steering cycle (per-cycle history), at the ISO lateral jerk limit.
    for speed in (5., 11.):
      for sign in (1, -1):
        max_delta = self._max_angle_delta(speed)
        self._reset_state()
        self.safety.set_controls_allowed(True)
        self._reset_angle_measurement(0)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed down by 1 m/s

        # first active frame rate limits from the measured angle (history fallback)
        angle = sign * max_delta * 0.5
        self.assertTrue(self._tx(self._angle_cmd_msg(angle, True, cycle=1)))

        # subsequent steering cycles rate limit from the previous cycle's committed angle
        for i in range(2, 5):
          angle += sign * max_delta * 0.5
          self.assertTrue(self._tx(self._angle_cmd_msg(angle, True, cycle=1 + (i - 1) * BMW_STEER_CYCLE_MOD)))
          self.assertTrue(self.safety.get_controls_allowed())

        # a step beyond the per-cycle jerk limit is blocked
        angle += sign * max_delta * 2.0
        self.assertFalse(self._tx(self._angle_cmd_msg(angle, True, cycle=1 + 4 * BMW_STEER_CYCLE_MOD)))

  def test_angle_rate_request_limit(self):
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(12)  # safety fudges the speed down by 1 m/s
    max_rate = self._max_angle_rate(11)

    for sign in (1, -1):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, active=True, rate=sign * max_rate)))
      self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, rate=sign * (max_rate + 10))))
      self.assertFalse(self._tx(self._angle_cmd_msg(0, active=False, rate=sign)))

  def test_angle_cmd_when_disabled(self):
    # While not actively steering, the command must track the measured angle, regardless of controls.
    for controls_allowed in (True, False):
      for meas in (-45., 0., 30.):
        self._reset_state()
        self.safety.set_controls_allowed(controls_allowed)
        self._reset_angle_measurement(meas)

        # matching the measured angle is allowed
        self.assertTrue(self._tx(self._angle_cmd_msg(meas, active=False, cycle=1)))
        # an angle far from measured is blocked
        self.assertFalse(self._tx(self._angle_cmd_msg(meas + 30, active=False, cycle=1)))

  def test_no_steer_when_controls_not_allowed(self):
    self._reset_state()
    self.safety.set_controls_allowed(False)
    self._reset_angle_measurement(0)

    # active steering request is blocked when controls are not allowed
    self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, cycle=1)))
    # an inactive (measured-tracking) request is still allowed
    self.assertTrue(self._tx(self._angle_cmd_msg(0, active=False, cycle=1)))

  def test_invalid_cycle_blocked(self):
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(11)

    # steering frames only exist on cycles where cycle % 4 == 1
    for cycle in (0, 2, 3):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, cycle=cycle)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, active=True, cycle=BMW_STEER_CYCLE_REM)))

  def test_cycle_tx_window(self):
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(11)

    live_cycle = 10
    self._set_rx_cycle(live_cycle)
    for cycle in self._next_steer_cycles(live_cycle, BMW_STEER_TX_FUTURE_WINDOW):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, active=True, cycle=cycle)))

    # The safety window is intentionally wider than the controller's 4-cycle lookahead,
    # but still narrower than a full 16-slot FlexRay steering round.
    outside_future_cycle = self._next_steer_cycles(live_cycle, BMW_STEER_TX_FUTURE_WINDOW + 1)[-1]
    outside_past_cycle = self._prev_steer_cycles(live_cycle, 1)[0]
    self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, cycle=outside_future_cycle)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, cycle=outside_past_cycle)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cycle_tx_window_blocks_full_history_replay(self):
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(12)
    self._set_rx_cycle(0, speed=12)

    max_delta = self._max_angle_delta(11)
    angle = 0.

    # An adversarial sender can update only the accepted slots while RX cycle time is
    # parked. It cannot walk all 16 history slots and then wrap the rate reference.
    accepted_cycles = self._next_steer_cycles(0, BMW_STEER_TX_FUTURE_WINDOW)
    for cycle in accepted_cycles:
      angle += max_delta * 0.5
      self.assertTrue(self._tx(self._angle_cmd_msg(angle, active=True, cycle=cycle)))

    angle += max_delta * 0.5
    outside_future_cycle = self._next_steer_cycles(0, BMW_STEER_TX_FUTURE_WINDOW + 1)[-1]
    first_accepted_cycle = accepted_cycles[0]
    self.assertFalse(self._tx(self._angle_cmd_msg(angle, active=True, cycle=outside_future_cycle)))
    self.assertFalse(self._tx(self._angle_cmd_msg(angle, active=True, cycle=first_accepted_cycle)))

  def test_reversing_assist_always_zero(self):
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(11)

    self.assertTrue(self._tx(self._angle_cmd_msg(0, active=True, cycle=1, reversing=0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, active=True, cycle=1, reversing=1)))

  def test_angle_cmd_across_cycle_wrap(self):
    # The per-cycle history must rate limit correctly across the 63->0 FlexRay round wrap:
    # the last steering cycle of a round (61) is the reference for the first of the next (1).
    self._reset_state()
    self.safety.set_controls_allowed(True)
    self._reset_angle_measurement(0)
    self._reset_speed_measurement(12)
    self._set_rx_cycle(57, speed=12)
    max_delta = self._max_angle_delta(11)
    ref = max_delta * 0.5

    # commit an angle on the last steering cycle of the round
    self.assertTrue(self._tx(self._angle_cmd_msg(ref, True, cycle=61)))
    self._set_rx_cycle(61, speed=12)
    # first steering cycle of the next round is limited against cycle 61's committed angle
    self.assertTrue(self._tx(self._angle_cmd_msg(ref + max_delta * 0.5, True, cycle=1)))
    self.assertFalse(self._tx(self._angle_cmd_msg(ref + max_delta * 3.0, True, cycle=1)))


if __name__ == "__main__":
  unittest.main()
