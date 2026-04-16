from __future__ import annotations

from cereal import car
from openpilot.common.params import Params
from openpilot.selfdrive.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarDocs, CarHarness, CarParts


class CarControllerParams:
  ACCEL_MIN = -3.5  # m/s
  ACCEL_MAX = 2.0   # m/s

  def __init__(self, CP):
    self.STEER_DELTA_UP = 5
    self.STEER_DELTA_DOWN = 10
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150
    self.STEER_MAX = 384


class I30SteeringCommandMode:
  LEGACY_SSC = "legacy_ssc"
  TRQI_TQ_DELTA = "trqi_tq_delta"


TRQI_STEERING_PARAM = "TRQISteeringToggle"
_params = Params()

# Read the steering backend selection once at startup so carstate, controller,
# and the Panda safety expectations all stay aligned for the life of the process.
# The UI toggle is therefore intended to take effect on the next reboot.
I30_STEERING_COMMAND_MODE = (
  I30SteeringCommandMode.TRQI_TQ_DELTA
  if _params.get_bool(TRQI_STEERING_PARAM)
  else I30SteeringCommandMode.LEGACY_SSC
)


def i30_uses_trqi_steering() -> bool:
  return I30_STEERING_COMMAND_MODE == I30SteeringCommandMode.TRQI_TQ_DELTA


class SteerLimitParams:  # controls running @ 100hz
  MAX_STEERING_TQ = 4             # Nm (original 12), this is for NEMA23, effective max steer torque
  STEER_DELTA_UP = 5 / 100        # 5 Nm/s (10Nm/s original) start quite low value with i30 because the steering is quite light
  STEER_DELTA_DOWN = 1000 / 100   # 10Nm/sample - no limit
  STEER_ERROR_MAX = 999           # max delta between torque cmd and torque motor
  STEER_STEP = 1                  # 100Hz
  STEER_MAX = 4                   # Nm, this is basically steer actuator scaling factor
  STEER_DRIVER_ALLOWANCE = 0      # We use these only for apply_driver_steer_torque_limits() compliance
  STEER_DRIVER_MULTIPLIER = 0     # We use these only for apply_driver_steer_torque_limits() compliance
  STEER_DRIVER_FACTOR = 0         # We use these only for apply_driver_steer_torque_limits() compliance


class TrqiSteerLimitParams:  # controls running @ 100hz
  # TRQI uses the sender's "TQ" domain, not the physical Nm signal from the old
  # standalone servo message. Full-scale 400 TQ is about 0.66 V of delta, which
  # narrows the command window to roughly 1.7..3.0 V around a typical mid-point
  # without touching the board-side absolute clamp.
  MAX_STEERING_TQ = 400.0
  STEER_DELTA_UP = 7.5
  STEER_DELTA_DOWN = 60.0
  STEER_STEP = 1
  STEER_MAX = 400.0

  # Positive TQ is intentionally defined as a right-turn command for TRQI mode.
  # openpilot's internal steering sign is the opposite on this platform, so the
  # controller multiplies by this sign before encoding the TRQI frame.
  OPENPILOT_TO_TRQI_TQ_SIGN = -1.0

  # Copy the debug sender's TQ-mode conversion:
  #   100 TQ -> 165 legacy input units -> -0.165 V -> DAC delta counts.
  TORQUE_REFERENCE = 100.0
  LEGACY_INPUT_AT_TORQUE_REFERENCE = 165.0
  INPUT_SCALE = 1000.0
  DAC_FULL_SCALE_VOLTS = 5.0
  DAC_BITS = 12
  MAX_DELTA = 2047
  RELAY_ENABLED = 1
  RELAYE_ENABLED = 1


class CAR(Platforms):
  HYUNDAI_I30_GD_2014 = PlatformConfig(
    [CarDocs("Hyundai i30 2014", "Custom", car_parts=CarParts.common([CarHarness.hyundai_b]))],
    CarSpecs(mass=1193, wheelbase=2.650, steerRatio=15.3, centerToFrontRatio=0.4, tireStiffnessFactor=0.385),
    dbc_dict("hyundai_i30_2014", None),
  )


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


DBC = CAR.create_dbc_map()

# Fingerprint/FW versions are not populated for this standalone make yet.
FINGERPRINTS: dict[str, list[dict[int, int]]] = {}
FW_VERSIONS: dict[str, dict[str | car.CarParams.Ecu, list[bytes]]] = {}
