from __future__ import annotations

from enum import IntFlag

from cereal import car
from openpilot.selfdrive.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarDocs


class CAR(Platforms):
  MOTORHOME_J1939_SSC = PlatformConfig(
    [CarDocs("Motorhome J1939 SSC", "Custom")],
    # Baseline values for a 2017 45 ft diesel pusher coach. The tire stiffness
    # factor is intentionally conservative: openpilot already scales stiffness
    # by mass, and this effective model also includes RV suspension/tire compliance.
    CarSpecs(mass=19142, wheelbase=8.28, steerRatio=20.0, centerToFrontRatio=0.63, tireStiffnessFactor=0.35),
    dbc_dict("j1939_standard", None),
  )


DBC = CAR.create_dbc_map()


class MotorhomeFlags(IntFlag):
  SSC_ENABLED = 1


class CarControllerParams:
  # Initial SSC commissioning limits, in the actuator protocol's Nm units.
  # Installation-specific gearing, signs and feedback must be calibrated first.
  STEER_MAX = 4.0
  STEER_RATE_UP = 5.0
  STEER_RATE_DOWN = 10.0
  SSC_FEEDBACK_ENABLED = 2
  SSC_SOFT_OFF = 4

# Fingerprint/FW versions are not populated for this standalone J1939 platform yet.
FINGERPRINTS: dict[str, list[dict[int, int]]] = {}
FW_VERSIONS: dict[str, dict[str | car.CarParams.Ecu, list[bytes]]] = {}
