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
// bare FlexRay slot 0x48 on the wire) is gated: ACTIVE may only request active steering
// (== 2) while controls are allowed, and REVERSING_ASSIST must always be zero.
// TODO: add angle and rate limits to bmw_tx_hook (see psa.h steer_angle_cmd_checks).

#define BMW_CRUISE_STATE  931U  // RX, stock ACC engaged state
#define BMW_BRAKE_PEDAL   753U  // RX, brake pedal state
#define BMW_WHEEL_SPEEDS  736U  // RX, per-wheel speeds and motion state
#define BMW_STEER_REQUEST  72U  // TX, FlexRay steering inject (slot 0x48)

#define BMW_BUS 0U

// WHEEL_SPEEDS.XX_STATE per-wheel motion flag: 1 = moving, 4 = standstill (others unknown).
#define BMW_WHEEL_STANDSTILL 4U

// The 18-byte FlexRay frames are carried as CAN-FD; 18 rounds up to the next valid
// DLC length (20). Adjust if the bridge emits a different on-wire length.
#define BMW_CAN_FD_LEN 20U

// STEER_REQUEST.ACTIVE value that requests an active steering command (1 == INACTIVE).
#define BMW_STEER_ACTIVE 2U

static void bmw_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == BMW_BUS) {
    if (msg->addr == BMW_CRUISE_STATE) {
      // CRUISE_ENGAGED_1 (bit 67): stock ACC active. Arms controls on the rising edge.
      pcm_cruise_check(GET_BIT(msg, 67U));
    }

    if (msg->addr == BMW_BRAKE_PEDAL) {
      // BRAKE_PRESSED_1 (bit 122): most sensitive brake signal. generic_rx_checks()
      // disengages controls on the rising edge, matching openpilot disengage-on-brake.
      brake_pressed = GET_BIT(msg, 122U);
    }

    if (msg->addr == BMW_WHEEL_SPEEDS) {
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
    // ACTIVE (bits 124-127): only request active steering while controls are allowed.
    bool steer_active = ((msg->data[15] >> 4) & 0xFU) == BMW_STEER_ACTIVE;
    if (steer_active && !controls_allowed) {
      tx = false;
    }

    // REVERSING_ASSIST (bits 74-78) must always be zero.
    if (((msg->data[9] >> 2) & 0x1FU) != 0U) {
      tx = false;
    }

    // No angle/rate limits yet (see TODO above).
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
  };

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
