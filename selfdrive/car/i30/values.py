from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Union

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarInfo, Harness


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
  STEER_DRIVER_ALLOWANCE = 0      # We use these only for apply_std_steer_torque_limits() compliance
  STEER_DRIVER_MULTIPLIER = 0     # We use these only for apply_std_steer_torque_limits() compliance
  STEER_DRIVER_FACTOR = 0         # We use these only for apply_std_steer_torque_limits() compliance


class CAR:
  I30 = "HYUNDAI I30 GD 2014"


@dataclass
class I30CarInfo(CarInfo):
  package: str = "Custom"


CAR_INFO: Dict[str, Optional[Union[I30CarInfo, List[I30CarInfo]]]] = {
  CAR.I30: I30CarInfo("Hyundai i30 2014", harness=Harness.hyundai_b),
}


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


DBC = {
  CAR.I30: dbc_dict('hyundai_i30_2014', None),
}

# Fingerprint/FW versions are not populated for this standalone make yet.
FINGERPRINTS: Dict[str, List[Dict[int, int]]] = {}
FW_VERSIONS: Dict[str, Dict[Union[str, car.CarParams.Ecu], List[bytes]]] = {}
