from __future__ import annotations

from cereal import car
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
