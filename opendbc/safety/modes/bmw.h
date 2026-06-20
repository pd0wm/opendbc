#pragma once

#include "opendbc/safety/declarations.h"

// BMW SP2018 over a CAN<->FlexRay bridge. All traffic (RX and steering TX) is on bus 0.
//
// controls_allowed mirrors openpilot: it arms on the rising edge of the stock ACC
// (CRUISE_STATE.CRUISE_ENGAGED_1) and disarms when it drops out or the driver brakes
// (BRAKE_PEDAL_3.BRAKE_PRESSED_1, via the core generic_rx_checks()). vehicle_moving
// comes from the per-wheel motion flags so a held brake keeps disengaging while moving.
//
// The steering inject (STEER_REQUEST, packed at report addr 0x481, re-addressed to the
// bare FlexRay slot 0x48 on the wire) is angle controlled and gated: REVERSING_ASSIST
// must always be zero, STEER_ANGLE_REQUEST is rate/accel limited, and
// STEER_ANGLE_RATE_REQUEST is bounded (bmw_steer_angle_checks).
//
// *** Cycle-tagged steering ***
// Each FlexRay frame carries CYCLE_COUNT, a 0..63 counter that ticks at 200 Hz. Steering
// frames only exist on cycles where cycle % 4 == 1, so the EPS acts on one commanded angle
// per steering cycle (every 4 cycles = 20 ms). openpilot tags each injected frame with the
// FlexRay cycle it is destined for and queues a few upcoming cycles every control frame,
// re-queuing the freshest angle (see car/bmw/carcontroller.py). As a result the same cycle
// can be sent multiple times with an evolving angle, and the cycle values are NOT monotonic
// on the wire. The latest received FlexRay cycle gates TX to a bounded lookahead window, so
// a sender cannot synthesize a full round of steering cycles without bus time advancing. The
// commanded-angle delta limit is computed per steering cycle: each command is limited against
// the angle committed for the PREVIOUS steering cycle, kept in a small per-cycle history that
// survives the 64-cycle wrap. No wall-clock timer is used.

#define BMW_CRUISE_STATE   931U  // RX, stock ACC engaged state
#define BMW_BRAKE_PEDAL    753U  // RX, brake pedal state
#define BMW_WHEEL_SPEEDS   736U  // RX, per-wheel speeds and motion state
#define BMW_STEERING_WHEEL 945U  // RX, measured steering angle (STEERING_WHEEL_2)
#define BMW_STEER_REQUEST   72U  // TX, FlexRay steering inject (slot 0x48)

#define BMW_BUS 0U

// WHEEL_SPEEDS.XX_STATE per-wheel motion flag: 1 = moving, 4 = standstill (others unknown).
#define BMW_WHEEL_STANDSTILL 4U

// The 18-byte FlexRay frames are carried as CAN-FD; 18 rounds up to the next valid
// DLC length (20). Adjust if the bridge emits a different on-wire length.
#define BMW_CAN_FD_LEN 20U

// STEER_REQUEST.ACTIVE value that requests an active steering command (1 == INACTIVE).
#define BMW_STEER_ACTIVE 2U

// STEER_ANGLE_REQUEST / STEERING_ANGLE_1 share the DBC scale 0.04395 deg/LSB with the zero
// point at raw 25000 (offset -1098.75 = -25000 * 0.04395). So the signed CAN-scale angle is
// (raw - 25000) and 1 deg == 1/0.04395 CAN units.
#define BMW_STEER_ANGLE_OFFSET 25000U
#define BMW_ANGLE_DEG_TO_CAN (1.0 / 0.04395)
// STEER_ANGLE_RATE_REQUEST has 0.4395 deg/s per LSB and zero at raw 32766.
#define BMW_STEER_RATE_OFFSET 32766U
#define BMW_STEER_RATE_DEG_TO_CAN (1.0f / 0.4395f)

// FlexRay cycle geometry: 64 cycles per round, ticking at 200 Hz. Steering frames live on
// cycles where cycle % 4 == 1, giving 16 steering cycles per round (one every 20 ms).
#define BMW_FLEXRAY_CYCLES 64
#define BMW_STEER_CYCLE_MOD 4
#define BMW_STEER_CYCLE_REM 1
#define BMW_STEER_CYCLES 16
#define BMW_CYCLE_HZ 200U
#define BMW_STEER_TX_FUTURE_WINDOW 4  // accepted future steering cycles from the latest RX cycle

// Per-steering-cycle angle history: the last accepted CAN-scale angle for each of the 16
// steering cycles in a FlexRay round. Persists across the round wrap so a command can always
// be rate limited against the previous steering cycle, even across the 63->0 boundary.
// openpilot sends a frame every steering cycle (tracking the measured angle while inactive),
// so this stays current without any measured-angle fallback or disengage reset.
static int bmw_desired_angle_last[BMW_STEER_CYCLES];
static int bmw_rx_cycle_last = 0;

static void bmw_reset_steer_state(void) {
  for (int i = 0; i < BMW_STEER_CYCLES; i++) {
    bmw_desired_angle_last[i] = 0;
  }
  bmw_rx_cycle_last = 0;
}

static int bmw_next_steer_cycle(int cycle) {
  int delta = (BMW_STEER_CYCLE_REM - (cycle % BMW_STEER_CYCLE_MOD) + BMW_STEER_CYCLE_MOD) % BMW_STEER_CYCLE_MOD;
  if (delta == 0) {
    delta = BMW_STEER_CYCLE_MOD;
  }

  return (cycle + delta) % BMW_FLEXRAY_CYCLES;
}

// Allow only a bounded future window from the latest RX CYCLE_COUNT.
static bool bmw_cycle_in_tx_window(int cycle) {
  bool in_window = false;

  if ((cycle % BMW_STEER_CYCLE_MOD) == BMW_STEER_CYCLE_REM) {
    int future_cycle = bmw_rx_cycle_last;
    for (int i = 0; (i < BMW_STEER_TX_FUTURE_WINDOW) && !in_window; i++) {
      future_cycle = bmw_next_steer_cycle(future_cycle);
      in_window = cycle == future_cycle;
    }
  }

  return in_window;
}

// Angle command and requested-rate safety, mirroring steer_angle_cmd_checks_vm but with a
// cycle-based delta reference (the previous steering cycle's committed angle) instead of a
// single monotonic last value, since BMW frames are cycle-tagged and re-queued out of order.
// The cycle counter is the clock (200 Hz), so no wall-clock timer is used.
static bool bmw_steer_angle_checks(int desired_angle, int desired_rate, bool steer_control_enabled, int cycle) {
  static const AngleSteeringLimits limits = {
    .max_angle = 8191,  // 360 deg, matches openpilot STEER_ANGLE_MAX (assumed EPS fault above)
    .angle_deg_to_can = BMW_ANGLE_DEG_TO_CAN,
  };
  // Vehicle model used for the lateral accel/jerk limits, matching car/bmw (BMW_SP2018 specs).
  static const AngleSteeringParams params = {
    .slip_factor = -0.0005401875442447107,  // calc_slip_factor(VM)
    .steer_ratio = 12.5,
    .wheelbase = 3.105,
  };

  // This check uses a simple vehicle model to allow for constant lateral acceleration and jerk limits across all speeds.

  // Highway curves are rolled in the direction of the turn, add tolerance to compensate
  const float MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^2
  // Lower than ISO 11270 lateral jerk limit, which is 5.0 m/s^3
  const float MAX_LATERAL_JERK = 3.0 + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^3

  const float fudged_speed = SAFETY_MAX((vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0, 1.0);
  const float curvature_factor = get_curvature_factor(fudged_speed, params);

  bool violation = false;

  // Steering frames only exist on cycles where cycle % 4 == 1 (cycle is 0..63 from a 6-bit field).
  bool valid_cycle = (cycle % BMW_STEER_CYCLE_MOD) == BMW_STEER_CYCLE_REM;
  if (!valid_cycle) {
    violation = true;
  }

  // Bound TX to the near future so one control step cannot walk a full round of history slots.
  if (valid_cycle && !bmw_cycle_in_tx_window(cycle)) {
    violation = true;
  }
  int slot = valid_cycle ? (cycle / BMW_STEER_CYCLE_MOD) : 0;  // 0..15

  if (controls_allowed && steer_control_enabled && valid_cycle) {
    // Rate limit reference: the angle committed for the previous steering cycle.
    // One steering cycle is BMW_STEER_CYCLE_MOD raw cycles, or 20 ms at 200 Hz.
    // NOTE: each slot is rate limited only against its previous slot. An adversarial carcontroller could,
    // within one 50 Hz cycle, write slot N high, write slot N+1 higher, then overwrite slot N low again,
    // leaving adjacent committed angles up to (2*window - 1) deltas apart: a bounded back-and-forth
    // vibration above the per-cycle rate. Absolute angle still can't exceed the accel limit below.
    int prev_slot = (slot + BMW_STEER_CYCLES - 1) % BMW_STEER_CYCLES;
    int desired_angle_last_for_cycle = bmw_desired_angle_last[prev_slot];

    // *** ISO lateral jerk limit ***
    // calculate maximum angle rate per second
    const float max_curvature_rate_sec = MAX_LATERAL_JERK / (fudged_speed * fudged_speed);
    const float max_angle_rate_sec = get_angle_from_curvature(max_curvature_rate_sec, curvature_factor, params);

    // finally get max angle delta per steering cycle
    const float steer_cycle_sec = (float)BMW_STEER_CYCLE_MOD / (float)BMW_CYCLE_HZ;  // 20 ms
    const float max_angle_delta = max_angle_rate_sec * steer_cycle_sec;
    const int max_angle_delta_can = (max_angle_delta * limits.angle_deg_to_can) + 1.;

    // NOTE: symmetric up and down limits
    const int highest_desired_angle = desired_angle_last_for_cycle + max_angle_delta_can;
    const int lowest_desired_angle = desired_angle_last_for_cycle - max_angle_delta_can;

    violation |= safety_max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);

    // *** ISO lateral accel limit ***
    const float max_curvature = MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed);
    const float max_angle = get_angle_from_curvature(max_curvature, curvature_factor, params);
    const int max_angle_can = (max_angle * limits.angle_deg_to_can) + 1.;

    violation |= safety_max_limit_check(desired_angle, max_angle_can, -max_angle_can);

    // *** requested angle rate limit ***
    const float max_desired_rate_float = (max_angle_rate_sec * BMW_STEER_RATE_DEG_TO_CAN) + 1.0f;
    const int max_desired_rate = (int)max_desired_rate_float;
    violation |= safety_max_limit_check(desired_rate, max_desired_rate, -max_desired_rate);
  }

  // Inactive steering should track the measured angle and carry zero requested rate.
  if (!steer_control_enabled) {
    const int max_inactive_angle = SAFETY_CLAMP(angle_meas.max, -limits.max_angle, limits.max_angle) + 1;
    const int min_inactive_angle = SAFETY_CLAMP(angle_meas.min, -limits.max_angle, limits.max_angle) - 1;
    violation |= safety_max_limit_check(desired_angle, max_inactive_angle, min_inactive_angle);
    violation |= desired_rate != 0;
  }

  // No angle control allowed when controls are not allowed
  if (!controls_allowed) {
    violation |= steer_control_enabled;
  }

  // commit accepted commands to the per-cycle history. A rejected command is not stored, so
  // the reference stays at the last good angle and a violation can't ratchet the limit up.
  if (valid_cycle && !violation) {
    bmw_desired_angle_last[slot] = desired_angle;
  }

  return violation;
}

static void bmw_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == BMW_BUS) {
    // CYCLE_COUNT (bits 0-5) is present on all BMW FlexRay report frames we monitor.
    bmw_rx_cycle_last = msg->data[0] & 0x3FU;

    if (msg->addr == BMW_CRUISE_STATE) {
      // CRUISE_ENGAGED_1 (bit 67): stock ACC active. Arms controls on the rising edge.
      pcm_cruise_check(GET_BIT(msg, 67U));
    }

    if (msg->addr == BMW_BRAKE_PEDAL) {
      // BRAKE_PRESSED_1 (bit 122): most sensitive brake signal. generic_rx_checks()
      // disengages controls on the rising edge, matching openpilot disengage-on-brake.
      brake_pressed = GET_BIT(msg, 122U);
    }

    if (msg->addr == BMW_STEERING_WHEEL) {
      // STEERING_ANGLE_1 (bits 24-39, little-endian): measured steering angle, same DBC scale
      // as the command, so the CAN-scale angle is (raw - 25000).
      int angle_meas_new = ((msg->data[4] << 8) | msg->data[3]) - BMW_STEER_ANGLE_OFFSET;
      update_sample(&angle_meas, angle_meas_new);
    }

    if (msg->addr == BMW_WHEEL_SPEEDS) {
      // Per-wheel speeds (km/h, 0.0198863636 per LSB, -652 offset), 16-bit little-endian:
      // RL bytes 4-5, RR bytes 6-7, FL bytes 8-9, FR bytes 10-11.
      int rl = (msg->data[5] << 8) | msg->data[4];
      int rr = (msg->data[7] << 8) | msg->data[6];
      int fl = (msg->data[9] << 8) | msg->data[8];
      int fr = (msg->data[11] << 8) | msg->data[10];
      int total_raw = rl + rr + fl + fr;
      float speed_kph = (((float)total_raw / 4.0f) * 0.0198863636f) - 652.0f;

      // Speeds can be negative when in reverse.
      if (speed_kph < 0.0f) {
        speed_kph = -speed_kph;
      }

      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);

      // Per-wheel motion flags (4 bits each): FL/RR in byte 12, RL/FR in byte 13.
      // Treat the car as moving unless all four wheels explicitly report standstill,
      // so a held brake keeps disengaging while moving while still allowing engagement
      // at a confirmed standstill (where vehicle_moving stays false).
      uint8_t fl_state = (msg->data[12] >> 4) & 0xFU;
      uint8_t rr_state = msg->data[12] & 0xFU;
      uint8_t rl_state = (msg->data[13] >> 4) & 0xFU;
      uint8_t fr_state = msg->data[13] & 0xFU;
      vehicle_moving = !((fl_state == BMW_WHEEL_STANDSTILL) && (fr_state == BMW_WHEEL_STANDSTILL) &&
                         (rl_state == BMW_WHEEL_STANDSTILL) && (rr_state == BMW_WHEEL_STANDSTILL));
    }
  }
}

static bool bmw_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  if (msg->addr == BMW_STEER_REQUEST) {
    // ACTIVE (bits 124-127): 2 requests active steering.
    bool steer_active = ((msg->data[15] >> 4) & 0xFU) == BMW_STEER_ACTIVE;

    // STEER_ANGLE_REQUEST (bits 32-47, little-endian), CAN-scale angle == raw - 25000.
    int desired_angle = ((msg->data[5] << 8) | msg->data[4]) - BMW_STEER_ANGLE_OFFSET;

    // STEER_ANGLE_RATE_REQUEST (bits 48-63, little-endian), CAN-scale rate == raw - 32766.
    int desired_rate = ((msg->data[7] << 8) | msg->data[6]) - BMW_STEER_RATE_OFFSET;

    // CYCLE_COUNT (bits 0-5): the FlexRay cycle this frame is tagged for.
    int cycle = msg->data[0] & 0x3FU;

    if (bmw_steer_angle_checks(desired_angle, desired_rate, steer_active, cycle)) {
      tx = false;
    }

    // REVERSING_ASSIST (bits 74-78) must always be zero.
    if (((msg->data[9] >> 2) & 0x1FU) != 0U) {
      tx = false;
    }
  }

  return tx;
}

static safety_config bmw_init(uint16_t param) {
  SAFETY_UNUSED(param);
  static const CanMsg BMW_TX_MSGS[] = {
    {BMW_STEER_REQUEST, BMW_BUS, BMW_CAN_FD_LEN, .check_relay = false},
  };

  static RxCheck bmw_rx_checks[] = {
    {.msg = {{BMW_CRUISE_STATE, BMW_BUS, BMW_CAN_FD_LEN, 50U, .ignore_checksum = true,
              .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_BRAKE_PEDAL, BMW_BUS, BMW_CAN_FD_LEN, 100U, .ignore_checksum = true,
              .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_WHEEL_SPEEDS, BMW_BUS, BMW_CAN_FD_LEN, 200U, .ignore_checksum = true,
              .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BMW_STEERING_WHEEL, BMW_BUS, BMW_CAN_FD_LEN, 50U, .ignore_checksum = true,
              .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  bmw_reset_steer_state();

  safety_config ret = BUILD_SAFETY_CFG(bmw_rx_checks, BMW_TX_MSGS);
  // Single-bus bridge setup with no camera: disable the default bus 0<->2 relay forwarding.
  ret.disable_forwarding = true;
  return ret;
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
};
