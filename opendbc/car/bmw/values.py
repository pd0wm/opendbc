from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM

# *** FlexRay steering inject (slot 0x48 == CAN id 72) ***
# The car's FlexRay network runs a 64-cycle counter. The steering frame is only
# present on cycles where cycle % 4 == 1, so we tag each frame with the FlexRay
# cycle it should be injected on and let the CAN<->FlexRay bridge place it (see
# CarController).
BMW_FLEXRAY_CYCLES = 64
BMW_STEER_CYCLE_MOD = 4
BMW_STEER_CYCLE_REM = 1
# FlexRay payload length in 16-bit words for slot 0x48 (the LENGTH header byte).
BMW_FLEXRAY_WORDS = 8
# BMW EPS payload checksum seed for slot 0x48 (CRC-8/J1850 == opendbc CRC8J1850).
BMW_STEER_CRC_INIT = 0xD6
# The STEER_REQUEST DBC message is 18 bytes; sent as CAN-FD it's padded up to the
# next valid DLC length (20). Must match the safety mode's tx allowlist length.
BMW_STEER_LEN = 20

# The CAN<->FlexRay bridge is on bus 0 for both RX and TX.
BMW_BUS = 0

# COUNTER on slot 0x48 is a free-running rolling counter, +1 per steering cycle,
# modulo 15 (values 0..14 — the stock module never emits 15). It is NOT derived
# from the FlexRay cycle: there are 16 steering cycles (cycle % 4 == 1) per
# 64-cycle round but only 15 counter values, so a cycle-locked counter would
# repeat a value once per round (a stuck step the EPS rejects). The host keeps
# its own monotonic steering-cycle index and sends index % 15; absolute phase
# vs the stock counter does not matter because we inject every cycle, so the EPS
# only ever sees our (consistently incrementing) stream.
BMW_STEER_COUNTER_MOD = 15


@dataclass(frozen=True, kw_only=True)
class BMWCarSpecs(CarSpecs):
  mass: float = 2000.
  wheelbase: float = 3.105
  steerRatio: float = 16.3


@dataclass
class BMWPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'bmw_sp2018'})


class CAR(Platforms):
  # Experimental FlexRay-bridge port; hidden from the docs (empty CarDocs list).
  BMW_SP2018 = BMWPlatformConfig(
    [],
    BMWCarSpecs(),
  )


class CarControllerParams:
  # the steering angle is recomputed and re-queued every control frame (100 Hz)
  STEER_STEP = 1

  # How many upcoming valid (cycle % 4 == 1) FlexRay cycles to queue each frame.
  # We re-queue with the freshest angle every frame; the bridge overwrites any
  # still-pending payload for the same target cycle.
  STEER_LOOKAHEAD = 4

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    # max angle accepted by the EPS; assume a fault above this, tune with testing
    360,  # deg
    # MAX_LATERAL_ACCEL / MAX_LATERAL_JERK default to the common ISO-based limits.
    # Cap the per-frame angle change for low-speed comfort and to prevent EPS faults.
    MAX_ANGLE_RATE=5.0,  # deg per control frame (10 ms)
  )


DBC = CAR.create_dbc_map()
