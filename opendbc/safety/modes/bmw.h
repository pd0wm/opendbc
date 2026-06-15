#pragma once

#include "opendbc/safety/declarations.h"

// BMW SP2018 over a CAN<->FlexRay bridge. All traffic (RX and steering TX) is on bus 0.
//
// Minimal safety mode: arm controls_allowed from the stock ACC (CRUISE_STATE) so the
// platform is treated as a real car safety mode (controls_allowed tracks cruise instead
// of being a permanently-open allOutput). The steering inject is allowlisted but its
// payload is not yet range/rate checked.
// TODO: add angle and rate limits to bmw_tx_hook (see psa.h steer_angle_cmd_checks).

#define BMW_CRUISE_STATE  931U  // RX, stock ACC engaged state
#define BMW_STEER_REQUEST  72U  // TX, FlexRay steering inject (slot 0x48)

#define BMW_BUS 0U

// The 18-byte FlexRay frames are carried as CAN-FD; 18 rounds up to the next valid
// DLC length (20). Adjust if the bridge emits a different on-wire length.
#define BMW_CAN_FD_LEN 20U

static void bmw_rx_hook(const CANPacket_t *msg) {
  if ((msg->bus == BMW_BUS) && (msg->addr == BMW_CRUISE_STATE)) {
    // CRUISE_ENGAGED_1 (byte 8, bit 3): stock ACC active
    pcm_cruise_check((msg->data[8] >> 3) & 1U);
  }
}

static bool bmw_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  // Allow all STEER_REQUEST payloads for now (no angle/rate limits yet).
  return true;
}

static safety_config bmw_init(uint16_t param) {
  SAFETY_UNUSED(param);
  static const CanMsg BMW_TX_MSGS[] = {
    {BMW_STEER_REQUEST, BMW_BUS, BMW_CAN_FD_LEN, .check_relay = false},
  };

  static RxCheck bmw_rx_checks[] = {
    {.msg = {{BMW_CRUISE_STATE, BMW_BUS, BMW_CAN_FD_LEN, 20U, .ignore_checksum = true,
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
