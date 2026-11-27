"""LS600h HRR lateral tuning.

The LS600h steers through the external HRR controller (it emulates the column torque sensor), so the
OEM EPS assist map and rack friction sit between openpilot and the rack. These schedules compensate for
that plant and belong to the HRR car only. Evaluate changes offline with ~/OPstuff/ls600h_lat_sim first
(see LATERAL_SIM.md).
"""
from openpilot.common.numpy_fast import interp

# Steady curves lagged the setpoint by 22-26% with Ki 0.3. Ki averages measurement noise, unlike
# friction or Kp, which raised jitter most at motorway speed. 0.45 shipped first. In the LS600h sim
# (2026-11-27, refit on 28 drives with the deadband map) 0.6 cut tracking error 2-4% at every speed
# for 1-4% more torque rate, a better trade than Kp 1.0. 0.75 doubled both.
TORQUE_KI = 0.6


def torque_feedforward_scale(v_ego: float) -> float:
  # Temporary motion-response compensation for HRR v0.1. 1.45 at motorway speed rang at about 1.3 Hz
  # in high-torque curves (routes 000001f8/f9, 96-123 km/h), which 1.25 rarely did, so it tapers back
  # to 1.25 from 70 to 90 km/h. Below 40 km/h there is almost no engaged data, so it tapers to 1.0 at
  # 30 km/h. Remove when validated HRR firmware owns this.
  return float(interp(v_ego * 3.6, [30.0, 40.0, 50.0, 70.0, 90.0], [1.0, 1.15, 1.45, 1.45, 1.25]))


def torque_deadband_comp(v_ego: float) -> float:
  # Below about 0.1 command the EPS gives almost no assist, and rack friction adds about 45 EPS units
  # more, so the wheel does not move until the command reaches about 0.2. At 30-70 km/h the car sat
  # 0.10-0.15 m right of where the model asked and steered too little of its small corrections.
  # In the LS600h sim (2026-11-22, 21 drives) 0.10 at 50-70 km/h cut slow tracking error 15% for 12%
  # more torque rate. It starts at 35 km/h (no engaged data below, steep low-speed Kp) and is gone by
  # 90 km/h, so the motorway tuning, where jitter matters most, is unchanged. The torque learner only
  # supplies latAccelOffset here (lateral accel factor and friction are fixed), which the map's odd
  # shape barely biases.
  return float(interp(v_ego * 3.6, [35.0, 50.0, 70.0, 90.0], [0.0, 0.10, 0.10, 0.0]))
