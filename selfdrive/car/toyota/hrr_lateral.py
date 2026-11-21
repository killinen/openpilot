"""LS600h HRR lateral tuning.

The LS600h steers through the external HRR controller (it emulates the column torque sensor), so the
OEM EPS assist map and rack friction sit between openpilot and the rack. These schedules compensate for
that plant and belong to the HRR car only. Evaluate changes offline with ~/OPstuff/ls600h_lat_sim first
(see LATERAL_SIM.md).
"""
from openpilot.common.numpy_fast import interp

# Steady curves lagged the setpoint by 22-26% with Ki 0.3. Ki averages measurement noise, unlike
# friction or Kp, which raised jitter most at motorway speed.
TORQUE_KI = 0.45


def torque_feedforward_scale(v_ego: float) -> float:
  # Temporary motion-response compensation for HRR v0.1. 1.45 at motorway speed rang at about 1.3 Hz
  # in high-torque curves (routes 000001f8/f9, 96-123 km/h), which 1.25 rarely did, so it tapers back
  # to 1.25 from 70 to 90 km/h. Below 40 km/h there is almost no engaged data, so it tapers to 1.0 at
  # 30 km/h. Remove when validated HRR firmware owns this.
  return float(interp(v_ego * 3.6, [30.0, 40.0, 50.0, 70.0, 90.0], [1.0, 1.15, 1.45, 1.45, 1.25]))
