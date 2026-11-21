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


# Old-long pedal command from the fitted engine/vehicle map (longitudinal_map.py) instead of the
# legacy linear accel/1.6 conversion. longcontrol computes the pedal and controlsd forwards it in
# actuators.gas; the carcontroller still sends exactly zero whenever longitudinal is inactive.
I30_PEDAL_MAP_ENABLED = True
I30_LEGACY_PEDAL_ACCEL = 1.6  # m/s^2 per full pedal in the legacy linear conversion
# Planner throttle veto from fitted coast capability (coast_lift.py); only ever removes throttle.
I30_EARLY_LIFT_ENABLED = True


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
  # 0x232 uses signed desired EPS output torque in Ncm. Keep the openpilot-side
  # nominal scale fixed at 10 Nm per unit, with 20% extra command headroom.
  MAX_STEERING_TQ = 1200.0
  STEER_DELTA_UP = 6.0
  STEER_DELTA_DOWN = 6.0
  STEER_STEP = 1
  STEER_MAX = 1000.0

  # Catch-up slew: when the applied command lags the request by more than
  # CATCHUP_GAP, both windup and unwind may move CATCHUP_DELTA per frame instead
  # of the nominal 6. Targets curve entry/exit lag; near-straight driving never
  # opens a 3 Nm gap, so it keeps the nominal slew. TRQI firmware accepts
  # 100 + 40 Ncm per 10 ms frame, so 15 stays well inside its step check.
  CATCHUP_DELTA = 15.0
  CATCHUP_GAP = 300.0
  # Catch-up is disabled while the driver touches the wheel and for
  # CATCHUP_HOLDOFF_FRAMES afterwards, so openpilot never winds up faster against
  # the driver. CATCHUP_DRIVER_MV mirrors the firmware SNR1-SNR2 difference
  # (about 342 mV/Nm), so 170 mV is about 0.5 Nm, below the 250 mV passthrough
  # threshold. CATCHUP_ENGAGE_FRAMES keeps the engage ramp at the nominal slew.
  CATCHUP_DRIVER_MV = 170.0
  CATCHUP_HOLDOFF_FRAMES = 100
  # A driver gate held only by the analog SNR estimate (no pressed or passthrough
  # requested/pending/active flag in the hold-off) still allows catch-up unwind toward
  # zero below CATCHUP_ANALOG_UNWIND_MAX_SPEED (m/s, 60 km/h). The estimate rises about
  # 0.04 Nm per Nm of EPS output, so on tight low-speed turn exits it trips the gate
  # while openpilot holds 8-12 Nm. On the 2026-11-20 drive that blocked the unwind and
  # the car kept turning (00000640 at 128 s: 0.040 1/m vs 0.014 requested). Windup
  # never uses this exception.
  CATCHUP_ANALOG_UNWIND_MAX_SPEED = 16.67
  CATCHUP_ENGAGE_FRAMES = 100
  # Lane changes: catch-up is off while a blinker is on or the model lane change is active
  # (controlsd sets CC blinkers while laneChangeState is not off, which can outlast the
  # blinker), and for CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES afterwards. On the 2026-11-20
  # highway drive the boost fired in 45 of 46 lane changes and p90 lane-change lateral jerk
  # was 5.4 m/s^3 vs 2.9 before catch-up. The gate applies once speed has exceeded
  # CATCHUP_LANE_CHANGE_MIN_SPEED (m/s, 50 km/h) during the episode, so a blinker at a
  # junction or roundabout approached slowly keeps catch-up, and a lane change never
  # switches policy mid-manoeuvre by crossing the threshold.
  CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES = 100
  CATCHUP_LANE_CHANGE_MIN_SPEED = 13.9
  # Windup catch-up fades to the nominal slew between these speeds (m/s, 70 -> 90 km/h);
  # unwind keeps CATCHUP_DELTA. A precaution against fast highway torque build-up: on the
  # 2026-11-20 drive it changed 1 of 132 ordinary highway curve entries, by at most 0.18 Nm.
  CATCHUP_WINDUP_FADE_BP = [19.44, 25.0]
  # Firmware CANCTR_SNR_PASSTHROUGH_ZERO_DIFF_MV.
  SNR_ZERO_DIFF_MV = -95.0

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
