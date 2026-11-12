"""i30 pedal/coast map: engine torque table + vehicle power balance (fitted offline).

Model (see tools/car_porting/i30_longitudinal/README.md):
  net torque  T [% of 230 Nm] = table(ECU pedal %, engine rpm)        (gear independent)
  accel       a = k_gear * T * rpm / v - c0 - c2 v^2 - (gain * g sin(pitch) + offset)

Gear is never assumed: k is applied only when the measured rpm/speed ratio lies inside a
calibrated gear window. Everything here is scalar arithmetic over small tables; there is
no fitting or file I/O per call. Unsupported inputs return None so the caller
must choose an explicit fallback.
"""
import json
from bisect import bisect_right
from math import isfinite, sin
from pathlib import Path

CALIBRATION_PATH = Path(__file__).with_name('pedal_map.json')
GRAVITY = 9.81


def _bracket(points, value):
  i = max(0, min(len(points) - 2, bisect_right(points, value) - 1))
  f = (value - points[i]) / (points[i + 1] - points[i])
  return i, min(max(f, 0.0), 1.0)


class I30LongitudinalMap:
  def __init__(self, calibration=None):
    cal = calibration if calibration is not None else json.loads(CALIBRATION_PATH.read_text())
    self.pedal_bp = [float(x) for x in cal['pedal_bp']]
    self.rpm_bp = [float(x) for x in cal['rpm_bp']]
    self.torque = [[float(x) for x in row] for row in cal['torque']]
    self.max_pedal = [float(x) for x in cal['max_pedal']]
    self.supported = [[bool(x) for x in row] for row in cal['supported']]
    # True where data show torque flattening at max_pedal (cap); else max_pedal is a coverage edge (fallback).
    self.saturates = [bool(x) for x in cal['saturates']]
    self.gears = {int(g): (float(v['ratio']), float(v['tol']), float(v['k'])) for g, v in cal['gears'].items() if v.get('controllable', False)}
    self.c0, self.c2 = float(cal['drag']['c0']), float(cal['drag']['c2'])
    self.grade_gain, self.grade_offset = float(cal['grade']['gain']), float(cal['grade']['offset'])
    self.max_abs_pitch = float(cal['grade']['max_abs_pitch'])
    self.cmd_scale, self.cmd_offset = float(cal['pedal_command']['scale']), float(cal['pedal_command']['offset'])
    self.v_min, self.v_max = float(cal['limits']['v_min']), float(cal['limits']['v_max'])
    self.last_limited = False  # set by command(): True when the result is the saturation knee
    for row in self.torque:
      if any(b <= a for a, b in zip(row, row[1:], strict=False)):
        raise ValueError('torque rows must be strictly increasing in pedal')

  # --- helpers -------------------------------------------------------------
  def gear_gain(self, v_ego, rpm):
    if not (isfinite(v_ego) and isfinite(rpm)) or v_ego < self.v_min or v_ego > self.v_max:
      return None
    ratio = rpm / v_ego
    for ratio_c, tol, k in self.gears.values():
      if abs(ratio - ratio_c) < tol:
        return k * ratio
    return None

  def _row(self, rpm):
    """Torque row at this rpm, the usable pedal limit, and per-cell support of both bracketing rows."""
    if not (self.rpm_bp[0] <= rpm <= self.rpm_bp[-1]):
      return None
    i, f = _bracket(self.rpm_bp, rpm)
    lo, hi = self.torque[i], self.torque[i + 1]
    row = [a + f * (b - a) for a, b in zip(lo, hi, strict=True)]
    max_pedal = min(self.max_pedal[i], self.max_pedal[i + 1])
    support = [a and b for a, b in zip(self.supported[i], self.supported[i + 1], strict=True)]
    return row, max_pedal, support, self.saturates[i] and self.saturates[i + 1]

  def resistance(self, v_ego, pitch):
    grade = 0.0
    if isfinite(pitch):
      grade = self.grade_gain * GRAVITY * sin(max(-self.max_abs_pitch, min(self.max_abs_pitch, pitch))) + self.grade_offset
    else:
      grade = self.grade_offset
    return self.c0 + self.c2 * v_ego * v_ego + grade

  def ecu_to_command(self, ecu_pedal_pct):
    return min(max((ecu_pedal_pct - self.cmd_offset) / (100.0 * self.cmd_scale), 0.0), 1.0)

  def command_to_ecu(self, command):
    return command * 100.0 * self.cmd_scale + self.cmd_offset if command > 0.0 else 0.0

  # --- public --------------------------------------------------------------
  def coast_accel(self, v_ego, rpm, pitch):
    """Acceleration with zero pedal in the current gear, or None if unknown."""
    gain = self.gear_gain(v_ego, rpm)
    row = self._row(rpm) if gain is not None else None
    if row is None or not row[2][0]:
      return None
    return gain * row[0][0] - self.resistance(v_ego, pitch)

  def accel(self, command, v_ego, rpm, pitch):
    """Steady-state forward prediction for an interceptor command (0..1)."""
    gain = self.gear_gain(v_ego, rpm)
    row = self._row(rpm) if gain is not None else None
    if row is None or not isfinite(command):
      return None
    torque, max_pedal, support, _ = row
    pedal = min(self.command_to_ecu(min(max(command, 0.0), 1.0)), max_pedal)
    i, f = _bracket(self.pedal_bp, pedal)
    if not support[i]:
      return None
    return gain * (torque[i] + f * (torque[i + 1] - torque[i])) - self.resistance(v_ego, pitch)

  def command(self, accel, v_ego, rpm, pitch):
    """Interceptor command (0..1) for a desired acceleration, or None outside coverage.

    Requests at or below coast capability give exactly 0. Requests above the usable
    torque are held at the knee where data show saturation, otherwise return None.
    """
    self.last_limited = False
    gain = self.gear_gain(v_ego, rpm)
    row = self._row(rpm) if gain is not None else None
    if row is None or not isfinite(accel) or gain <= 0.0:
      return None
    torque, max_pedal, support, saturates = row
    if max_pedal <= self.pedal_bp[0] or not support[0]:
      return None
    needed = (accel + self.resistance(v_ego, pitch)) / gain
    if needed <= torque[0]:
      return 0.0
    i_max, f_max = _bracket(self.pedal_bp, max_pedal)
    torque_max = torque[i_max] + f_max * (torque[i_max + 1] - torque[i_max])
    if needed >= torque_max:
      # Genuine saturation: more pedal adds no torque, so hold the knee. A coverage edge
      # is unknown territory: let the caller fall back instead of capping.
      self.last_limited = saturates
      return self.ecu_to_command(max_pedal) if saturates else None
    i, f = _bracket(torque, needed)
    if not support[i]:
      return None
    return self.ecu_to_command(self.pedal_bp[i] + f * (self.pedal_bp[i + 1] - self.pedal_bp[i]))
