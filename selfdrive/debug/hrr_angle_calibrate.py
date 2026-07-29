#!/usr/bin/env python3
"""Guided 0x25-referenced calibration for the STM32G474 HRR resolvers."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
import math
import queue
import secrets
import select
import struct
import sys
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from panda import Panda


CONFIG_ADDR = 0x603
STEER_ANGLE_ADDR = 0x25
STATE_STATUS_ADDR = 0x634
CAL_STATUS_ADDR = 0x635
VECTOR_STATUS_ADDR = 0x637
CONFIG_KEY = 0xA5

CMD_CAL_START = 4
CMD_CAL_FINISH_SAVE = 5
CMD_CAL_ABORT = 6
CMD_CAL_MODE = 7
CMD_MATRIX_FIRST = 11
CMD_MATRIX_LAST = 18
CMD_PHASE_PER_STEER = 19
CMD_FIT_METRICS = 20

CAL_PROTOCOL_VERSION = 2
MATRIX_Q = 20
MIN_SAMPLES = 100
MIN_REFERENCE_SPAN_DEG = 360.0
MIN_PHASE_SPAN_DEG = 270.0
MIN_DIRECTION_TRAVEL_DEG = 120.0
MAX_REFERENCE_RATE_DEG_S = 50.0
RESOLVER_REVOLUTION_DEG = 360.0
SHAFT_DEG_PER_RESOLVER_REVOLUTION = 22.5
# RMS/covariance vectors identify phase modulo 180 degrees, so one physical
# 360-degree resolver revolution produces 180 degrees of unwrapped phase.
NOMINAL_PHASE_PER_STEER = (RESOLVER_REVOLUTION_DEG / 2.0) / SHAFT_DEG_PER_RESOLVER_REVOLUTION
MIN_PHASE_PER_STEER = 6.0
MAX_PHASE_PER_STEER = 10.0
MAX_RESOLVER_PHASE_RATE_DEG_S = MAX_REFERENCE_RATE_DEG_S * NOMINAL_PHASE_PER_STEER
MAX_FIT_RMS_DEG = 1.5
MAX_FIT_ERROR_DEG = 5.0
# IN and OU observe opposite sides of the steering torsion bar.  Fit only
# samples where their corrected angles are close to the unloaded relationship;
# otherwise steering effort is incorrectly learned as resolver non-linearity.
MAX_TORSION_DEVIATION_DEG = 1.5
REFERENCE_TIMEOUT_S = 0.10
STATUS_TIMEOUT_S = 0.75
VECTOR_MIN_MAGNITUDE = 4.0
VECTOR_MATRIX_INPUT_SCALE = 1024.0
REFERENCE_DELAY_MIN_S = -0.20
REFERENCE_DELAY_MAX_S = 0.40
REFERENCE_DELAY_STEP_S = 0.01
STAGE_ACK_TIMEOUT_S = 0.5
STAGE_MAX_ATTEMPTS = 8
START_MAX_ATTEMPTS = 5
CAL2_RESUME_MAX_ATTEMPTS = 6
COMMIT_FAILED = 0
COMMIT_COMPLETE = 1
COMMIT_RESUME = 2
FLASH_VALIDATE_RESET_REASON = 25

FAILURE_REASONS = {
  0: "none",
  1: "unsafe state",
  2: "bad session",
  3: "incomplete parameters",
  4: "invalid coefficient",
  5: "fit quality rejected",
  6: "flash save failed",
  7: "session timed out",
  8: "CANCTR disabled",
  9: "CANCTR test mode active",
  10: "resolver mirror stopped",
  11: "SVEC delta nonzero",
  12: "relay command active",
  13: "relay feedback active",
  14: "torque interlock inactive",
  19: "IN resolver vector invalid",
  20: "OU resolver vector invalid",
  21: "firmware reset during physical SRAM flash erase",
  22: "firmware reset during flash body programming",
  23: "firmware reset during flash verification",
  24: "firmware reset during flash commit-marker programming",
  25: "firmware reset during final flash validation",
  26: "firmware reset while scanning the destination flash page",
  27: "brownout/low-power reset during calibration flash operation",
  28: "non-watchdog/non-power reset during calibration flash operation",
  70: "NMI during calibration flash operation; watchdog recovery reset",
  71: "HardFault during calibration flash operation; watchdog recovery reset",
  72: "MemManage fault during calibration flash operation; watchdog recovery reset",
  73: "BusFault during calibration flash operation; watchdog recovery reset",
  74: "UsageFault during calibration flash operation; watchdog recovery reset",
  75: "CAL2 body partially saved; replay required to continue atomic commit",
}
for phase_base, phase_name in ((31, "issuing flash data"),
                               (41, "waiting for flash busy"),
                               (51, "after flash busy cleared"),
                               (61, "before entering SRAM writer")):
  for index in range(7):
    FAILURE_REASONS[phase_base + index] = (
      f"watchdog reset {phase_name} at snapshot offset 0x{8 * (index + 1):02X}"
    )


def crc8_poly07(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def sign_extend(value: int, bits: int) -> int:
  sign = 1 << (bits - 1)
  return (value ^ sign) - sign


def build_config_frame(command: int, value: int = 0) -> bytes:
  if not 0 <= command <= 0xFF:
    raise ValueError("command must fit in one byte")
  value &= 0xFFFFFFFF
  body = bytes((command,)) + value.to_bytes(4, "little") + bytes((CONFIG_KEY, 0))
  checksum = crc8_poly07(bytes((CONFIG_ADDR & 0xFF, CONFIG_ADDR >> 8)) + body)
  return body + bytes((checksum,))


def shortest_mod180_delta(value: float, previous: float) -> float:
  delta = (value - previous + 90.0) % 180.0 - 90.0
  if math.isclose(abs(delta), 90.0, abs_tol=1e-9):
    raise ValueError("ambiguous 90 degree resolver step")
  return delta


@dataclass(frozen=True)
class SteerReference:
  angle_deg: float

  @classmethod
  def decode(cls, payload: bytes) -> SteerReference:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{STEER_ANGLE_ADDR:03X}, got {len(payload)}")
    coarse = sign_extend(((payload[0] & 0x0F) << 8) | payload[1], 12)
    return cls(coarse * 1.5)


@dataclass(frozen=True)
class SafetyStatus:
  rel: bool
  rele: bool
  ou_angle_valid: bool
  in_angle_valid: bool
  brake_seen: bool
  brake_fresh: bool
  brake_pressed: bool
  brake_interlock: bool
  torque_seen: bool
  centers_valid: bool
  mirror_running: bool
  guard_enabled: bool
  guard_angles_valid: bool
  canctr_enabled: bool
  torque_fresh: bool
  torque_interlock: bool
  brake_age_ms: int
  torque_age_ms: int

  @classmethod
  def decode(cls, payload: bytes) -> SafetyStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{STATE_STATUS_ADDR:03X}, got {len(payload)}")
    flags = int.from_bytes(payload[:2], "little")
    return cls(bool(flags & (1 << 0)), bool(flags & (1 << 1)),
               bool(flags & (1 << 2)), bool(flags & (1 << 3)),
               bool(flags & (1 << 4)), bool(flags & (1 << 5)),
               bool(flags & (1 << 6)), bool(flags & (1 << 7)),
               bool(flags & (1 << 8)), bool(flags & (1 << 9)),
               bool(flags & (1 << 10)), bool(flags & (1 << 11)),
               bool(flags & (1 << 12)), bool(flags & (1 << 13)),
               bool(flags & (1 << 14)), bool(flags & (1 << 15)),
               int.from_bytes(payload[4:6], "little"),
               int.from_bytes(payload[6:8], "little"))

  @property
  def calibration_safe(self) -> bool:
    return (not self.rel and not self.rele and self.ou_angle_valid and self.in_angle_valid and
            self.mirror_running and self.canctr_enabled and self.torque_interlock)

  def format(self) -> str:
    return (f"relay={int(self.rel)}/{int(self.rele)} raw={int(self.in_angle_valid)}/{int(self.ou_angle_valid)} " +
            f"mirror={int(self.mirror_running)} canctr={int(self.canctr_enabled)} " +
            f"brake=seen:{int(self.brake_seen)},fresh:{int(self.brake_fresh)},pressed:{int(self.brake_pressed)}," +
            f"interlock:{int(self.brake_interlock)},age:{self.brake_age_ms}ms " +
            f"torque=seen:{int(self.torque_seen)},fresh:{int(self.torque_fresh)}," +
            f"interlock:{int(self.torque_interlock)},age:{self.torque_age_ms}ms")


@dataclass(frozen=True)
class CalibrationStatus:
  running: bool
  valid: bool
  enabled: bool
  failed: bool
  parameters_complete: bool
  in_raw_valid: bool
  ou_raw_valid: bool
  legacy_active: bool
  version: int
  samples: int
  staged_mask: int
  flash_size_kib: int
  dual_bank: bool
  failure_reason: int
  rms_error_deg: float

  @classmethod
  def decode(cls, payload: bytes) -> CalibrationStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{CAL_STATUS_ADDR:03X}, got {len(payload)}")
    flags, version, samples, raw_staged, reason, rms = struct.unpack("<BBHHBB", payload)
    staged_mask = raw_staged & 0x03FF
    flash_size_kib = ((raw_staged >> 10) & 0x1F) * 32
    return cls(bool(flags & (1 << 0)), bool(flags & (1 << 1)), bool(flags & (1 << 2)),
               bool(flags & (1 << 3)), bool(flags & (1 << 4)), bool(flags & (1 << 5)),
               bool(flags & (1 << 6)), bool(flags & (1 << 7)), version, samples,
               staged_mask, flash_size_kib, bool(raw_staged & (1 << 15)), reason, rms * 0.1)

  def format(self) -> str:
    mode = "LEGACY" if self.legacy_active else "CALIBRATED"
    reason = FAILURE_REASONS.get(self.failure_reason, f"unknown({self.failure_reason})")
    flash = f" flash={self.flash_size_kib}KiB,dbank={int(self.dual_bank)}" if self.flash_size_kib else ""
    return (f"v{self.version}{flash} state={'RUNNING' if self.running else 'IDLE'} mode={mode} " +
            f"valid={int(self.valid)} raw={int(self.in_raw_valid)}/{int(self.ou_raw_valid)} " +
            f"staged=0x{self.staged_mask:03x} samples={self.samples} " +
            f"rms={self.rms_error_deg:.1f}deg reason={reason}")


@dataclass(frozen=True)
class ResolverVector:
  in_cos: int
  in_sin: int
  ou_cos: int
  ou_sin: int

  @classmethod
  def decode(cls, payload: bytes) -> ResolverVector:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{VECTOR_STATUS_ADDR:03X}, got {len(payload)}")
    return cls(*struct.unpack("<hhhh", payload))


@dataclass(frozen=True)
class CalibrationSample:
  timestamp: float
  reference_deg: float
  raw_phase_deg: float
  unwrapped_phase_deg: float
  in_cos: int
  in_sin: int
  ou_cos: int
  ou_sin: int


@dataclass(frozen=True)
class TimedReference:
  timestamp: float
  angle_deg: float


@dataclass(frozen=True)
class CalibrationFit:
  matrices: tuple[float, ...]
  phase_per_steer: float
  samples: int
  total_samples: int
  rms_error_deg: float
  max_error_deg: float
  ou_rms_error_deg: float
  ou_max_error_deg: float
  torsion_rms_deg: float
  torsion_max_deg: float
  reference_span_deg: float
  phase_span_deg: float
  positive_travel_deg: float
  negative_travel_deg: float
  occupied_bins: int
  reference_delay_s: float

  @property
  def ready(self) -> bool:
    return (self.samples >= MIN_SAMPLES and self.reference_span_deg >= MIN_REFERENCE_SPAN_DEG and
            self.phase_span_deg >= MIN_PHASE_SPAN_DEG and
            self.positive_travel_deg >= MIN_DIRECTION_TRAVEL_DEG and
            self.negative_travel_deg >= MIN_DIRECTION_TRAVEL_DEG and self.occupied_bins >= 15 and
            self.rms_error_deg <= MAX_FIT_RMS_DEG and self.max_error_deg <= MAX_FIT_ERROR_DEG)

  def format(self) -> str:
    return (f"low_torsion={self.samples}/{self.total_samples} " +
            f"ref_span={self.reference_span_deg:.0f}/{MIN_REFERENCE_SPAN_DEG:.0f}deg " +
            f"phase_span={self.phase_span_deg:.0f}/{MIN_PHASE_SPAN_DEG:.0f}deg " +
            f"travel=+{self.positive_travel_deg:.0f}/-{self.negative_travel_deg:.0f}deg " +
            f"bins={self.occupied_bins}/18 ratio={self.phase_per_steer:+.6f} " +
            f"lag={self.reference_delay_s:+.2f}s " +
            f"IN={self.rms_error_deg:.2f}/{self.max_error_deg:.2f}deg " +
            f"OU={self.ou_rms_error_deg:.2f}/{self.ou_max_error_deg:.2f}deg " +
            f"torsion={self.torsion_rms_deg:.2f}/{self.torsion_max_deg:.2f}deg")


def vector_phase_deg(cos_value: int | float, sin_value: int | float) -> float:
  if math.hypot(cos_value, sin_value) < VECTOR_MIN_MAGNITUDE:
    raise ValueError("resolver vector too small")
  return math.degrees(math.atan2(sin_value, cos_value)) % 180.0


def linear_fit(x: list[float], y: list[float]) -> tuple[float, float]:
  x_mean = sum(x) / len(x)
  y_mean = sum(y) / len(y)
  denominator = sum((item - x_mean) ** 2 for item in x)
  if denominator <= 1e-9:
    raise ValueError("insufficient steering-angle variation")
  slope = sum((xv - x_mean) * (yv - y_mean) for xv, yv in zip(x, y, strict=True)) / denominator
  return slope, y_mean - slope * x_mean


def robust_phase_ratio(samples: list[CalibrationSample]) -> tuple[float, list[int]]:
  indices = list(range(len(samples)))
  for _ in range(4):
    x = [samples[i].reference_deg for i in indices]
    y = [samples[i].unwrapped_phase_deg for i in indices]
    slope, intercept = linear_fit(x, y)
    residuals = [samples[i].unwrapped_phase_deg - (slope * samples[i].reference_deg + intercept) for i in indices]
    median = sorted(residuals)[len(residuals) // 2]
    deviations = sorted(abs(value - median) for value in residuals)
    mad = deviations[len(deviations) // 2]
    limit = max(2.0, 4.5 * 1.4826 * mad)
    filtered = [i for i, residual in zip(indices, residuals, strict=True) if abs(residual - median) <= limit]
    if len(filtered) == len(indices) or len(filtered) < MIN_SAMPLES:
      break
    indices = filtered
  slope, _ = linear_fit([samples[i].reference_deg for i in indices],
                        [samples[i].unwrapped_phase_deg for i in indices])
  if not MIN_PHASE_PER_STEER <= abs(slope) <= MAX_PHASE_PER_STEER:
    raise ValueError(f"implausible resolver/steering ratio {slope:+.6f}")
  return slope, indices


def solve_matrix(samples: list[CalibrationSample], indices: list[int], ratio: float,
                 pair: str) -> tuple[tuple[float, float, float, float], list[int]]:
  current = [index for index in indices
             if math.hypot(samples[index].in_cos if pair == "in" else samples[index].ou_cos,
                           samples[index].in_sin if pair == "in" else samples[index].ou_sin) >=
             VECTOR_MIN_MAGNITUDE]
  if len(current) < MIN_SAMPLES:
    raise ValueError(f"need at least {MIN_SAMPLES} valid {pair.upper()} vectors")
  matrix = (1.0, 0.0, 0.0, 1.0)
  for _ in range(4):
    sxx = sxy = syy = 0.0
    h00 = h01 = h10 = h11 = 0.0
    for index in current:
      sample = samples[index]
      cos_raw = float(sample.in_cos if pair == "in" else sample.ou_cos)
      sin_raw = float(sample.in_sin if pair == "in" else sample.ou_sin)
      magnitude = math.hypot(cos_raw, sin_raw)
      if magnitude < VECTOR_MIN_MAGNITUDE:
        continue
      # Keep one fixed scale for all samples. Per-sample normalization makes
      # the relationship non-linear and biases a least-squares ellipse fit.
      x, y = cos_raw / VECTOR_MATRIX_INPUT_SCALE, sin_raw / VECTOR_MATRIX_INPUT_SCALE
      target = math.radians((ratio * sample.reference_deg) % 180.0)
      target_cos, target_sin = math.cos(target), math.sin(target)
      # Resolver vectors are projective: z and -z describe the same modulo-180
      # phase. Choose the target representative closest to the measured vector
      # so the least-squares fit stays continuous across the 180-degree seam.
      if target_cos * cos_raw + target_sin * sin_raw < 0.0:
        target_cos, target_sin = -target_cos, -target_sin
      sxx += x * x
      sxy += x * y
      syy += y * y
      h00 += target_cos * x
      h01 += target_cos * y
      h10 += target_sin * x
      h11 += target_sin * y
    determinant = sxx * syy - sxy * sxy
    if determinant < 1e-4:
      raise ValueError(f"poorly conditioned {pair.upper()} resolver sweep")
    inv00, inv01, inv11 = syy / determinant, -sxy / determinant, sxx / determinant
    matrix = (h00 * inv00 + h01 * inv01, h00 * inv01 + h01 * inv11,
              h10 * inv00 + h11 * inv01, h10 * inv01 + h11 * inv11)
    if abs(matrix[0] * matrix[3] - matrix[1] * matrix[2]) < 0.05 or max(map(abs, matrix)) > 4.0:
      raise ValueError(f"invalid {pair.upper()} resolver correction matrix")
    residuals: list[tuple[int, float]] = []
    for index in current:
      sample = samples[index]
      cos_raw = float(sample.in_cos if pair == "in" else sample.ou_cos)
      sin_raw = float(sample.in_sin if pair == "in" else sample.ou_sin)
      out_cos = matrix[0] * cos_raw + matrix[1] * sin_raw
      out_sin = matrix[2] * cos_raw + matrix[3] * sin_raw
      measured = vector_phase_deg(out_cos, out_sin)
      target = (ratio * sample.reference_deg) % 180.0
      residuals.append((index, abs(shortest_mod180_delta(measured, target))))
    errors = sorted(error for _, error in residuals)
    median = errors[len(errors) // 2]
    deviations = sorted(abs(error - median) for error in errors)
    mad = deviations[len(deviations) // 2]
    limit = max(2.0, median + 4.5 * 1.4826 * mad)
    filtered = [index for index, error in residuals if error <= limit]
    if len(filtered) == len(current) or len(filtered) < MIN_SAMPLES:
      break
    current = filtered
  return matrix, current


def corrected_phase(sample: CalibrationSample, matrix: tuple[float, float, float, float],
                    pair: str) -> float:
  cos_raw = sample.in_cos if pair == "in" else sample.ou_cos
  sin_raw = sample.in_sin if pair == "in" else sample.ou_sin
  return vector_phase_deg(matrix[0] * cos_raw + matrix[1] * sin_raw,
                          matrix[2] * cos_raw + matrix[3] * sin_raw)


def median(values: list[float]) -> float:
  ordered = sorted(values)
  middle = len(ordered) // 2
  if len(ordered) % 2:
    return ordered[middle]
  return 0.5 * (ordered[middle - 1] + ordered[middle])


def fit_calibration_aligned(samples: list[CalibrationSample], reference_delay_s: float = 0.0) -> CalibrationFit:
  if len(samples) < MIN_SAMPLES:
    raise ValueError(f"need at least {MIN_SAMPLES} samples")

  # First obtain approximate electrical corrections from all robust samples.
  # Their relative corrected phase reveals steering torque without assuming
  # that the operator can maintain a constant sweep speed.
  preliminary_ratio, preliminary_indices = robust_phase_ratio(samples)
  preliminary_in, preliminary_indices = solve_matrix(samples, preliminary_indices,
                                                      preliminary_ratio, "in")
  preliminary_ou, preliminary_indices = solve_matrix(samples, preliminary_indices,
                                                      preliminary_ratio, "ou")
  torsions = [
    shortest_mod180_delta(corrected_phase(samples[index], preliminary_ou, "ou"),
                          corrected_phase(samples[index], preliminary_in, "in")) /
    abs(preliminary_ratio)
    for index in preliminary_indices
  ]
  torsion_baseline = median(torsions)
  low_torsion_indices = [
    index for index, torsion in zip(preliminary_indices, torsions, strict=True)
    if abs(torsion - torsion_baseline) <= MAX_TORSION_DEVIATION_DEG
  ]
  if len(low_torsion_indices) < MIN_SAMPLES:
    raise ValueError(
      f"need at least {MIN_SAMPLES} low-torsion samples " +
      f"({len(low_torsion_indices)}/{len(samples)} within " +
      f"{MAX_TORSION_DEVIATION_DEG:.1f}deg); pause and relax hand torque at more wheel angles"
    )

  # Refit everything using only the unloaded relationship. Coverage and fit
  # quality below are deliberately evaluated on this retained subset.
  filtered_samples = [samples[index] for index in low_torsion_indices]
  ratio, indices = robust_phase_ratio(filtered_samples)
  in_matrix, indices = solve_matrix(filtered_samples, indices, ratio, "in")
  ou_matrix, indices = solve_matrix(filtered_samples, indices, ratio, "ou")
  in_errors: list[float] = []
  ou_errors: list[float] = []
  final_torsions: list[float] = []
  bins = set()
  for index in indices:
    sample = filtered_samples[index]
    target = (ratio * sample.reference_deg) % 180.0
    in_phase = corrected_phase(sample, in_matrix, "in")
    ou_phase = corrected_phase(sample, ou_matrix, "ou")
    in_errors.append(abs(shortest_mod180_delta(in_phase, target)) / abs(ratio))
    ou_errors.append(abs(shortest_mod180_delta(ou_phase, target)) / abs(ratio))
    final_torsions.append(shortest_mod180_delta(ou_phase, in_phase) / abs(ratio))
    bins.add(int(target // 10.0) % 18)
  references = [filtered_samples[i].reference_deg for i in indices]
  phases = [filtered_samples[i].unwrapped_phase_deg for i in indices]
  positive = negative = 0.0
  for previous, current in zip(references, references[1:], strict=False):
    delta = current - previous
    if delta > 0:
      positive += delta
    else:
      negative -= delta
  torsion_center = median(final_torsions)
  torsion_deviations = [abs(value - torsion_center) for value in final_torsions]
  return CalibrationFit(in_matrix + ou_matrix, ratio, len(indices), len(samples),
                        math.sqrt(sum(error * error for error in in_errors) / len(in_errors)), max(in_errors),
                        math.sqrt(sum(error * error for error in ou_errors) / len(ou_errors)), max(ou_errors),
                        math.sqrt(sum(error * error for error in torsion_deviations) /
                                  len(torsion_deviations)), max(torsion_deviations),
                        max(references) - min(references), max(phases) - min(phases),
                        positive, negative, len(bins), reference_delay_s)


def align_samples(samples: list[CalibrationSample], references: list[TimedReference],
                  delay_s: float) -> list[CalibrationSample]:
  """Interpolate 0x025 at the time represented by each windowed 0x637 vector."""
  if len(references) < 2:
    return []
  aligned = []
  reference_index = 0
  for sample in samples:
    target_time = sample.timestamp - delay_s
    while (reference_index + 1 < len(references) and
           references[reference_index + 1].timestamp <= target_time):
      reference_index += 1
    if reference_index + 1 >= len(references):
      break
    before = references[reference_index]
    after = references[reference_index + 1]
    if target_time < before.timestamp:
      continue
    elapsed = after.timestamp - before.timestamp
    if elapsed <= 0.0:
      continue
    fraction = (target_time - before.timestamp) / elapsed
    reference_deg = before.angle_deg + fraction * (after.angle_deg - before.angle_deg)
    aligned.append(CalibrationSample(sample.timestamp, reference_deg,
                                     sample.raw_phase_deg, sample.unwrapped_phase_deg,
                                     sample.in_cos, sample.in_sin, sample.ou_cos, sample.ou_sin))
  return aligned


def fit_calibration(samples: list[CalibrationSample],
                    references: list[TimedReference] | None = None) -> CalibrationFit:
  if references is None:
    return fit_calibration_aligned(samples)
  if len(samples) < MIN_SAMPLES:
    raise ValueError(f"need at least {MIN_SAMPLES} samples")

  scored_delays: list[tuple[float, float]] = []
  delay_steps = round((REFERENCE_DELAY_MAX_S - REFERENCE_DELAY_MIN_S) / REFERENCE_DELAY_STEP_S)
  for step in range(delay_steps + 1):
    delay_s = REFERENCE_DELAY_MIN_S + step * REFERENCE_DELAY_STEP_S
    aligned = align_samples(samples, references, delay_s)
    if len(aligned) < MIN_SAMPLES:
      continue
    try:
      ratio, indices = robust_phase_ratio(aligned)
      slope, intercept = linear_fit([aligned[index].reference_deg for index in indices],
                                    [aligned[index].unwrapped_phase_deg for index in indices])
    except ValueError:
      continue
    residual_rms = math.sqrt(sum(
      (aligned[index].unwrapped_phase_deg -
       (slope * aligned[index].reference_deg + intercept)) ** 2
      for index in indices) / len(indices)) / abs(ratio)
    scored_delays.append((residual_rms, delay_s))
  if not scored_delays:
    # Preserve the most useful error message when no lag produces a plausible ratio.
    return fit_calibration_aligned(samples)

  # The raw phase score cheaply locates the timing valley. Run the more
  # expensive two-matrix fit only around that valley so CAN polling remains
  # responsive on the comma.
  _, coarse_delay = min(scored_delays)
  best: CalibrationFit | None = None
  best_score = math.inf
  for _, delay_s in scored_delays:
    if abs(delay_s - coarse_delay) > 3.0 * REFERENCE_DELAY_STEP_S:
      continue
    aligned = align_samples(samples, references, delay_s)
    try:
      candidate = fit_calibration_aligned(aligned, delay_s)
    except ValueError:
      continue
    candidate_score = max(candidate.rms_error_deg / MAX_FIT_RMS_DEG,
                          candidate.max_error_deg / MAX_FIT_ERROR_DEG)
    if candidate_score < best_score:
      best = candidate
      best_score = candidate_score
  if best is None:
    return fit_calibration_aligned(samples)
  return best


@dataclass(frozen=True)
class FitWorkerResult:
  fit: CalibrationFit | None
  error: str | None
  collected_samples: int


class CalibrationFitWorker:
  """Fit immutable snapshots without interrupting live CAN reception."""

  def __init__(self) -> None:
    self.thread: threading.Thread | None = None
    self.results: queue.SimpleQueue[FitWorkerResult] = queue.SimpleQueue()

  def busy(self) -> bool:
    return self.thread is not None and self.thread.is_alive()

  def start(self, samples: list[CalibrationSample], references: list[TimedReference]) -> None:
    if self.busy():
      return

    def run() -> None:
      try:
        fit = fit_calibration(samples, references)
        self.results.put(FitWorkerResult(fit, None, len(samples)))
      except ValueError as error:
        self.results.put(FitWorkerResult(None, str(error), len(samples)))

    self.thread = threading.Thread(target=run, name="hrr-calibration-fit", daemon=True)
    self.thread.start()

  def take_latest(self) -> FitWorkerResult | None:
    latest = None
    while True:
      try:
        latest = self.results.get_nowait()
      except queue.Empty:
        return latest


class HrrCalibrationSession:
  def __init__(self, panda: Panda | None, bus: int, reference_buses: tuple[int, ...], dry_run: bool) -> None:
    self.panda = panda
    self.bus = bus
    self.reference_buses = reference_buses
    self.dry_run = dry_run
    self.calibration: CalibrationStatus | None = None
    self.safety: SafetyStatus | None = None
    self.reference: SteerReference | None = None
    self.vector: ResolverVector | None = None
    self.reference_at: float | None = None
    self.status_at: float | None = None
    self.safety_at: float | None = None
    self.samples: list[CalibrationSample] = []
    self.reference_history: list[TimedReference] = []
    self.previous_phase: float | None = None
    self.unwrapped_phase = 0.0
    self.previous_reference: float | None = None
    self.previous_sample_at: float | None = None

  def send_command(self, command: int, value: int = 0) -> bytes:
    payload = build_config_frame(command, value)
    print(f"TX 0x{CONFIG_ADDR:03X}: {payload.hex(' ')}")
    if not self.dry_run:
      assert self.panda is not None
      self.panda.can_send(CONFIG_ADDR, payload, self.bus)
    return payload

  def _collect_vector(self, vector: ResolverVector, now: float) -> None:
    if (self.calibration is None or not self.calibration.in_raw_valid or not self.calibration.ou_raw_valid or
        self.reference is None or self.reference_at is None or now - self.reference_at > REFERENCE_TIMEOUT_S):
      return
    try:
      phase = vector_phase_deg(vector.in_cos, vector.in_sin)
    except ValueError:
      return
    candidate_unwrapped = phase
    phase_delta = 0.0
    if self.previous_phase is not None:
      try:
        phase_delta = shortest_mod180_delta(phase, self.previous_phase)
      except ValueError:
        return
      candidate_unwrapped = self.unwrapped_phase + phase_delta
    if self.previous_reference is not None and self.previous_sample_at is not None:
      elapsed = now - self.previous_sample_at
      if (elapsed <= 0 or
          abs(self.reference.angle_deg - self.previous_reference) / elapsed > MAX_REFERENCE_RATE_DEG_S or
          abs(phase_delta) / elapsed > MAX_RESOLVER_PHASE_RATE_DEG_S):
        return
    self.unwrapped_phase = candidate_unwrapped
    self.samples.append(CalibrationSample(now, self.reference.angle_deg, phase, candidate_unwrapped,
                                          vector.in_cos, vector.in_sin, vector.ou_cos, vector.ou_sin))
    self.previous_phase = phase
    self.previous_reference = self.reference.angle_deg
    self.previous_sample_at = now

  def poll(self, collect: bool = False) -> None:
    if self.dry_run:
      return
    assert self.panda is not None
    for address, _, payload, rx_bus in self.panda.can_recv():
      now = time.monotonic()
      try:
        # Panda reports the source bus to the host, even when the frame is
        # being forwarded physically to CAN2. 0x025 must therefore be read
        # from its vehicle-side source bus rather than the HRR bus.
        if address == STEER_ANGLE_ADDR and rx_bus in self.reference_buses:
          self.reference = SteerReference.decode(payload)
          self.reference_at = now
          self.reference_history.append(TimedReference(now, self.reference.angle_deg))
        elif rx_bus != self.bus:
          continue
        elif address == STATE_STATUS_ADDR:
          self.safety = SafetyStatus.decode(payload)
          self.safety_at = now
        elif address == CAL_STATUS_ADDR:
          self.calibration = CalibrationStatus.decode(payload)
          self.status_at = now
        elif address == VECTOR_STATUS_ADDR:
          self.vector = ResolverVector.decode(payload)
          if collect:
            self._collect_vector(self.vector, now)
      except ValueError:
        continue

  def wait_for(self, predicate, timeout: float, collect: bool = False) -> CalibrationStatus | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.poll(collect)
      if self.calibration is not None and predicate(self.calibration):
        return self.calibration
      time.sleep(0.01)
    return None

  def live_ready(self) -> bool:
    now = time.monotonic()
    return (self.calibration is not None and self.calibration.version == CAL_PROTOCOL_VERSION and
            self.calibration.in_raw_valid and self.calibration.ou_raw_valid and
            self.status_at is not None and now - self.status_at <= STATUS_TIMEOUT_S and
            self.safety is not None and self.safety.calibration_safe and self.safety_at is not None and
            now - self.safety_at <= STATUS_TIMEOUT_S and self.reference is not None and
            self.reference_at is not None and now - self.reference_at <= REFERENCE_TIMEOUT_S and
            self.vector is not None)

  def readiness_errors(self) -> list[str]:
    """Describe every condition that currently prevents a calibration start."""
    now = time.monotonic()
    errors = []
    if self.calibration is None:
      errors.append("no valid 8-byte HRR calibration status (0x635)")
    else:
      if self.calibration.version != CAL_PROTOCOL_VERSION:
        errors.append(f"0x635 protocol v{self.calibration.version}, expected v{CAL_PROTOCOL_VERSION}")
      if self.status_at is None or now - self.status_at > STATUS_TIMEOUT_S:
        errors.append(f"0x635 stale (> {STATUS_TIMEOUT_S:.2f}s)")
      if not self.calibration.in_raw_valid:
        errors.append("0x635 IN resolver vector invalid")
      if not self.calibration.ou_raw_valid:
        errors.append("0x635 OU resolver vector invalid")

    if self.safety is None:
      errors.append("no valid 8-byte HRR safety status (0x634)")
    else:
      if self.safety_at is None or now - self.safety_at > STATUS_TIMEOUT_S:
        errors.append(f"0x634 stale (> {STATUS_TIMEOUT_S:.2f}s)")
      unsafe = []
      if self.safety.rel:
        unsafe.append("REL closed")
      if self.safety.rele:
        unsafe.append("RELE closed")
      if not self.safety.in_angle_valid:
        unsafe.append("IN angle invalid")
      if not self.safety.ou_angle_valid:
        unsafe.append("OU angle invalid")
      if not self.safety.mirror_running:
        unsafe.append("resolver mirror stopped")
      if not self.safety.canctr_enabled:
        unsafe.append("CANCTR disabled")
      if not self.safety.torque_interlock:
        unsafe.append(f"torque interlock off (fresh={int(self.safety.torque_fresh)}, age={self.safety.torque_age_ms}ms)")
      if unsafe:
        errors.append("0x634 unsafe: " + ", ".join(unsafe))

    if self.reference is None:
      buses = "/".join(str(bus) for bus in self.reference_buses)
      errors.append(f"no valid 8-byte steering reference (0x025) on bus {buses}")
    elif self.reference_at is None or now - self.reference_at > REFERENCE_TIMEOUT_S:
      errors.append(f"0x025 stale (> {REFERENCE_TIMEOUT_S:.2f}s)")

    if self.vector is None:
      errors.append("no valid 8-byte resolver vector (0x637)")
    return errors

  def wait_for_live_ready(self, timeout: float) -> bool:
    """Receive fresh CAN after an interactive pause before judging readiness."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.poll()
      if self.live_ready():
        return True
      time.sleep(0.01)
    return self.live_ready()


def choose_bus(configured_bus: int | None) -> int:
  if configured_bus is not None:
    return configured_bus
  while True:
    value = input("Panda CAN bus [0/1/2]: ").strip()
    if value in {"0", "1", "2"}:
      return int(value)
    print("Enter 0, 1, or 2.")


def parse_replay_values(text: str) -> tuple[int, ...]:
  try:
    parsed = tuple(int(part.strip(), 0) for part in text.split(","))
  except ValueError as error:
    raise argparse.ArgumentTypeError("replay values must be comma-separated decimal or 0x-prefixed integers") from error
  if len(parsed) != CMD_FIT_METRICS - CMD_MATRIX_FIRST + 1:
    raise argparse.ArgumentTypeError("replay requires exactly 10 values for commands 11 through 20")
  if any(value < -(1 << 31) or value > 0xFFFFFFFF for value in parsed):
    raise argparse.ArgumentTypeError("each replay value must fit signed or unsigned 32 bits")
  return tuple(value & 0xFFFFFFFF for value in parsed)


def upload_values(session: HrrCalibrationSession, values: tuple[int, ...]) -> CalibrationStatus | None:
  for command, value in enumerate(values, start=CMD_MATRIX_FIRST):
    staged_bit = 1 << (command - CMD_MATRIX_FIRST)
    for _ in range(STAGE_MAX_ATTEMPTS):
      session.send_command(command, value)
      status = session.wait_for(
        lambda item, bit=staged_bit: not item.running or bool(item.staged_mask & bit),
        STAGE_ACK_TIMEOUT_S,
      )
      if status is None:
        continue
      if not status.running or status.staged_mask & staged_bit:
        break
    else:
      return session.calibration
    if status is not None and not status.running:
      return status
  return session.calibration


def fit_values(fit: CalibrationFit) -> tuple[int, ...]:
  rms_tenths = min(255, round(fit.rms_error_deg * 10.0))
  max_tenths = min(255, round(fit.max_error_deg * 10.0))
  metrics = min(0xFFFF, fit.samples) | (rms_tenths << 16) | (max_tenths << 24)
  values = tuple(round(coefficient * (1 << MATRIX_Q)) & 0xFFFFFFFF for coefficient in fit.matrices)
  values += (round(fit.phase_per_steer * (1 << MATRIX_Q)) & 0xFFFFFFFF, metrics)
  return values


def upload_fit(session: HrrCalibrationSession, fit: CalibrationFit) -> CalibrationStatus | None:
  return upload_values(session, fit_values(fit))


def committed_after_validation_reset(finished: CalibrationStatus | None,
                                     staged: CalibrationStatus | None) -> bool:
  return (finished is not None and staged is not None and
          finished.failure_reason == FLASH_VALIDATE_RESET_REASON and
          finished.valid and finished.enabled and not finished.legacy_active and
          finished.parameters_complete and
          finished.samples == staged.samples and
          finished.rms_error_deg == staged.rms_error_deg)


def finish_staged_calibration(session: HrrCalibrationSession, token: int,
                              staged: CalibrationStatus | None) -> int:
  if staged is None or not staged.running or not staged.parameters_complete:
    if staged is None:
      detail = "no 0x635 status received during upload"
    else:
      missing = [
        str(command) for command in range(CMD_MATRIX_FIRST, CMD_FIT_METRICS + 1)
        if not staged.staged_mask & (1 << (command - CMD_MATRIX_FIRST))
      ]
      missing_text = "none" if not missing else ",".join(missing)
      detail = f"{staged.format()} missing_commands={missing_text}"
    print(f"ERROR: HRR did not accept the complete coefficient set: {detail}", file=sys.stderr)
    if staged is None or staged.running:
      session.send_command(CMD_CAL_ABORT, token)
    return COMMIT_FAILED

  session.send_command(CMD_CAL_FINISH_SAVE, token)
  # CAL2 temporarily uses the firmware's maximum (~8 s) IWDG interval so a
  # slow flash pulse can finish; wait beyond that recovery deadline.
  finished = session.wait_for(lambda status: not status.running, 12.0)
  if finished is not None and finished.failure_reason == 75:
    print("CAL2 body progress saved; continuing with another guarded replay session.")
    return COMMIT_RESUME
  if committed_after_validation_reset(finished, staged):
    print(f"Calibration committed atomically and recovered after final validation reset: {finished.format()}")
    return COMMIT_COMPLETE
  if finished is None or not finished.valid or not finished.enabled or finished.legacy_active or finished.failed:
    if finished is not None:
      detail = finished.format()
      if (not finished.valid and not finished.failed and finished.staged_mask == 0 and
          finished.samples == 0 and finished.failure_reason == 0):
        detail += " (firmware likely reset during flash commit)"
    elif session.calibration is not None:
      detail = f"timeout waiting for IDLE; last status: {session.calibration.format()}"
    else:
      detail = "no 0x635 status received after save command"
    print(f"ERROR: calibration was not committed: {detail}", file=sys.stderr)
    return COMMIT_FAILED
  print(f"Calibration committed atomically: {finished.format()}")
  return COMMIT_COMPLETE


def start_calibration_session(session: HrrCalibrationSession) -> int | None:
  if not session.wait_for_live_ready(1.0):
    details = "; ".join(session.readiness_errors())
    print(f"ERROR: calibration preconditions not met: {details}.", file=sys.stderr)
    return None
  token = secrets.randbits(32) or 1
  for _ in range(START_MAX_ATTEMPTS):
    session.send_command(CMD_CAL_START, token)
    started = session.wait_for(lambda status: status.running, 0.6)
    if started is not None and started.running:
      return token
  detail = "no 0x635 status" if session.calibration is None else session.calibration.format()
  safety = "no 0x634 status" if session.safety is None else session.safety.format()
  print(f"ERROR: HRR rejected calibration start: {detail}; safety: {safety}", file=sys.stderr)
  return None


def commit_values(session: HrrCalibrationSession, values: tuple[int, ...],
                  initial_token: int | None = None) -> bool:
  token = initial_token
  for _ in range(CAL2_RESUME_MAX_ATTEMPTS):
    if token is None:
      token = start_calibration_session(session)
      if token is None:
        return False
    result = finish_staged_calibration(session, token, upload_values(session, values))
    if result == COMMIT_COMPLETE:
      return True
    if result == COMMIT_FAILED:
      return False
    token = None
  print("ERROR: CAL2 save did not complete within the resume-attempt limit.", file=sys.stderr)
  return False


def run_replay(session: HrrCalibrationSession, values: tuple[int, ...], assume_yes: bool) -> bool:
  print("\nSecure the stationary vehicle. HRR relays must be open and torque output interlocked.")
  print("Replay will stage the captured fit below and immediately test its atomic flash commit:")
  for command, value in enumerate(values, start=CMD_MATRIX_FIRST):
    print(f"  command {command}: 0x{value:08x}")
  if not assume_yes:
    input("Press Enter to commit this captured fit, or Ctrl-C to cancel: ")
  if session.dry_run:
    token = 0x12345678
    session.send_command(CMD_CAL_START, token)
    for command, value in enumerate(values, start=CMD_MATRIX_FIRST):
      session.send_command(command, value)
    session.send_command(CMD_CAL_FINISH_SAVE, token)
    print("Dry run: replay frames generated; nothing was committed.")
    return True
  return commit_values(session, values)


def run_guided(session: HrrCalibrationSession, timeout: float, assume_yes: bool) -> bool:
  print("\nSecure the stationary vehicle. HRR relays must be open and torque output interlocked.")
  print("Move the wheel manually and slowly: center -> left -> right -> center.")
  print("Cover at least 360 degrees from the leftmost to rightmost reading; mechanical locks are not required.")
  print("Pause briefly and relax steering effort at several angles in both directions; constant speed is not required.")
  if not assume_yes:
    input("Press Enter when ready, or Ctrl-C to cancel: ")
  if session.dry_run:
    session.send_command(CMD_CAL_START, 0x12345678)
    print("Dry run: start frame generated; no samples were collected.")
    return True
  # input() stops CAN polling, while 0x025 is intentionally required to be no
  # more than 100 ms old. Refresh every input after the operator confirms.
  token = start_calibration_session(session)
  if token is None:
    return False

  print("\nSweep slowly. Press Enter only after READY is shown.")
  deadline = time.monotonic() + timeout
  fit_worker = CalibrationFitWorker()
  latest_result: FitWorkerResult | None = None
  next_fit = 0.0
  next_print = 0.0
  latest_fit: CalibrationFit | None = None
  while time.monotonic() < deadline:
    session.poll(collect=True)
    now = time.monotonic()
    completed_result = fit_worker.take_latest()
    if completed_result is not None:
      latest_result = completed_result
      latest_fit = completed_result.fit
    if now >= next_fit and not fit_worker.busy():
      fit_worker.start(list(session.samples), list(session.reference_history))
      next_fit = now + 0.5
    if now >= next_print:
      if latest_result is None:
        text = f"n={len(session.samples)} (fitting)"
        state = "SWEEP"
      elif latest_result.fit is not None:
        latest_fit = latest_result.fit
        text = latest_fit.format()
        state = "READY" if latest_fit.ready else "SWEEP"
      else:
        text = f"n={latest_result.collected_samples} ({latest_result.error})"
        state = "SWEEP"
      ref = "---.-" if session.reference is None else f"{session.reference.angle_deg:+.1f}"
      print(f"\r{state} steer={ref}deg {text}    ", end="", flush=True)
      next_print = now + 0.5
    if session.calibration is not None and not session.calibration.running:
      safety = "no 0x634 status" if session.safety is None else session.safety.format()
      print(f"\nERROR: firmware aborted calibration: {session.calibration.format()}; safety: {safety}", file=sys.stderr)
      return False
    if sys.stdin.isatty():
      readable, _, _ = select.select([sys.stdin], [], [], 0)
      if readable:
        sys.stdin.readline()
        if latest_fit is not None and latest_fit.ready:
          break
        print("\nCoverage or fit quality is not sufficient; continue the sweep.")
    elif latest_fit is not None and latest_fit.ready:
      break
    time.sleep(0.01)
  else:
    print("\nERROR: calibration timed out.", file=sys.stderr)
    session.send_command(CMD_CAL_ABORT, token)
    session.wait_for(lambda status: not status.running, 1.0)
    return False

  assert latest_fit is not None
  print(f"\nFit: {latest_fit.format()}")
  return commit_values(session, fit_values(latest_fit), token)


def run_self_test() -> None:
  steer = SteerReference.decode(bytes((0x00, 0x0A, 0, 0, 0xD0, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, 15.0)
  steer = SteerReference.decode(bytes((0x0F, 0xF6, 0, 0, 0x30, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, -15.0)
  steer = SteerReference.decode(bytes((0, 0, 0, 0, 0x80, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, 0.0)
  # Calibration safety deliberately does not depend on brake status bits 4..7.
  safe_flags = sum(1 << bit for bit in (2, 3, 9, 10, 13, 15))
  safety = SafetyStatus.decode(safe_flags.to_bytes(2, "little") + b"\x00\x00\x0c\x00\xff\xff")
  assert safety.calibration_safe
  assert safety.brake_age_ms == 12 and safety.torque_age_ms == 0xFFFF
  invalid_in = CalibrationStatus.decode(b"\x08\x02\x00\x00\x00\x00\x13\x00")
  assert invalid_in.failure_reason == 19 and "IN resolver vector invalid" in invalid_in.format()
  geometry = CalibrationStatus.decode(struct.pack("<BBHHBB", 0x10, 2, 100,
                                                   0x03FF | (16 << 10) | (1 << 15), 0, 10))
  assert geometry.staged_mask == 0x03FF and geometry.flash_size_kib == 512 and geometry.dual_bank
  replay_values = parse_replay_values(
    "0x000ca38d,0xffe50ea1,0x0019b855,0x000d4bf7,0x000fbd21," +
    "0xffe7d4f4,0x0017ad5f,0x001145ef,0xff78412d,0x220b05b5"
  )
  assert len(replay_values) == 10 and replay_values[-1] == 0x220B05B5
  assert shortest_mod180_delta(1.0, 179.0) == 2.0
  assert shortest_mod180_delta(179.0, 1.0) == -2.0
  for command, value in ((CMD_CAL_START, 0x12345678), (CMD_CAL_FINISH_SAVE, 0x12345678),
                         (CMD_CAL_ABORT, 0), (CMD_CAL_MODE, 1), (CMD_MATRIX_FIRST, -12345)):
    frame = build_config_frame(command, value)
    assert len(frame) == 8
    assert crc8_poly07(bytes((CONFIG_ADDR & 0xFF, CONFIG_ADDR >> 8)) + frame[:7]) == frame[7]

  synthetic: list[CalibrationSample] = []
  unwrapped = 0.0
  previous: float | None = None
  references = ([-540.0 + index * 1.5 for index in range(721)] +
                [540.0 - index * 1.5 for index in range(1, 721)])
  for index, reference in enumerate(references):
    target = -NOMINAL_PHASE_PER_STEER * reference
    raw = (target + 17.0) % 180.0
    if previous is None:
      unwrapped = raw
    else:
      unwrapped += shortest_mod180_delta(raw, previous)
    previous = raw
    angle = math.radians((raw + 90.0) % 180.0 - 90.0)
    cos_raw = round(1000.0 * math.cos(angle) + 120.0 * math.sin(angle))
    sin_raw = round(760.0 * math.sin(angle))
    synthetic.append(CalibrationSample(index * 0.1, reference, raw, unwrapped,
                                       cos_raw, sin_raw, cos_raw, sin_raw))
  fit = fit_calibration(synthetic)
  assert fit.ready, fit.format()
  assert abs(fit.phase_per_steer + NOMINAL_PHASE_PER_STEER) < 0.01
  assert fit.rms_error_deg < 0.2

  class FakeUploadSession(HrrCalibrationSession):
    def __init__(self) -> None:
      self.staged_mask = 0
      self.attempts: dict[int, int] = {}
      self.calibration: CalibrationStatus | None = None

    def send_command(self, command: int, value: int = 0) -> bytes:
      self.attempts[command] = self.attempts.get(command, 0) + 1
      # Simulate one dropped coefficient frame; an identical retry is valid.
      if command != CMD_MATRIX_FIRST + 1 or self.attempts[command] > 1:
        self.staged_mask |= 1 << (command - CMD_MATRIX_FIRST)
      return build_config_frame(command, value)

    def wait_for(self, predicate: Callable[[CalibrationStatus], bool], timeout: float,
                 collect: bool = False) -> CalibrationStatus | None:
      del timeout, collect
      flags = (1 << 0) | ((1 << 4) if self.staged_mask == 0x03FF else 0)
      status = CalibrationStatus.decode(struct.pack("<BBHHBB", flags, CAL_PROTOCOL_VERSION,
                                                    0, self.staged_mask, 0, 0))
      self.calibration = status
      return status if predicate(status) else None

  fake_upload = FakeUploadSession()
  staged = upload_fit(fake_upload, fit)
  assert staged is not None and staged.parameters_complete
  assert fake_upload.attempts[CMD_MATRIX_FIRST + 1] == 2
  recovered_flags = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)
  recovered = CalibrationStatus.decode(struct.pack(
    "<BBHHBB", recovered_flags, CAL_PROTOCOL_VERSION, staged.samples,
    staged.staged_mask, FLASH_VALIDATE_RESET_REASON, round(staged.rms_error_deg * 10.0),
  ))
  assert committed_after_validation_reset(recovered, staged)
  mismatched = CalibrationStatus.decode(struct.pack(
    "<BBHHBB", recovered_flags, CAL_PROTOCOL_VERSION, staged.samples + 1,
    staged.staged_mask, FLASH_VALIDATE_RESET_REASON, round(staged.rms_error_deg * 10.0),
  ))
  assert not committed_after_validation_reset(mismatched, staged)

  # Simulate steering effort twisting OU relative to IN in either direction.
  # Periodic relaxed samples span the complete sweep and must be selected for
  # the final fit instead of teaching the torsion-bar deflection to the matrix.
  torsion_samples: list[CalibrationSample] = []
  for index, sample in enumerate(synthetic):
    if (index * 73) % 101 < 34:
      torsion_deg = 0.0
    else:
      torsion_deg = 4.0 if index < len(synthetic) // 2 else -4.0
    ou_raw = (sample.raw_phase_deg + abs(NOMINAL_PHASE_PER_STEER) * torsion_deg) % 180.0
    ou_angle = math.radians((ou_raw + 90.0) % 180.0 - 90.0)
    ou_cos = round(1000.0 * math.cos(ou_angle) + 120.0 * math.sin(ou_angle))
    ou_sin = round(760.0 * math.sin(ou_angle))
    torsion_samples.append(CalibrationSample(
      sample.timestamp, sample.reference_deg, sample.raw_phase_deg, sample.unwrapped_phase_deg,
      sample.in_cos, sample.in_sin, ou_cos, ou_sin,
    ))
  torsion_fit = fit_calibration(torsion_samples)
  assert torsion_fit.ready, torsion_fit.format()
  assert MIN_SAMPLES <= torsion_fit.samples < torsion_fit.total_samples * 0.5, torsion_fit.format()
  assert torsion_fit.rms_error_deg < 0.2, torsion_fit.format()

  # Irregular motion with a windowed-vector delay must align automatically;
  # requiring the operator to maintain constant speed is neither realistic nor
  # necessary when both streams have timestamps.
  imposed_delay_s = 0.14

  def trajectory(timestamp: float) -> float:
    return (500.0 * math.sin(2.0 * math.pi * timestamp / 120.0) +
            35.0 * math.sin(2.0 * math.pi * timestamp / 17.0))

  timed_references = [
    TimedReference(index * 0.02, round(trajectory(index * 0.02) / 1.5) * 1.5)
    for index in range(12051)
  ]
  delayed_samples: list[CalibrationSample] = []
  unwrapped = 0.0
  delayed_previous: float | None = None
  for index in range(10, 2400):
    timestamp = index * 0.1
    reference = trajectory(timestamp - imposed_delay_s)
    target = -NOMINAL_PHASE_PER_STEER * reference
    raw = (target + 17.0) % 180.0
    if delayed_previous is None:
      unwrapped = raw
    else:
      unwrapped += shortest_mod180_delta(raw, delayed_previous)
    delayed_previous = raw
    angle = math.radians((raw + 90.0) % 180.0 - 90.0)
    cos_raw = round(1000.0 * math.cos(angle) + 120.0 * math.sin(angle))
    sin_raw = round(760.0 * math.sin(angle))
    current_reference = round(trajectory(timestamp) / 1.5) * 1.5
    delayed_samples.append(CalibrationSample(timestamp, current_reference, raw, unwrapped,
                                             cos_raw, sin_raw, cos_raw, sin_raw))
  unaligned_fit = fit_calibration(delayed_samples)
  aligned_fit = fit_calibration(delayed_samples, timed_references)
  assert abs(aligned_fit.reference_delay_s - imposed_delay_s) <= 1.5 * REFERENCE_DELAY_STEP_S, aligned_fit.format()
  assert aligned_fit.rms_error_deg < unaligned_fit.rms_error_deg * 0.25
  assert aligned_fit.ready, aligned_fit.format()

  worker = CalibrationFitWorker()
  worker.start(delayed_samples, timed_references)
  worker_deadline = time.monotonic() + 5.0
  while worker.busy() and time.monotonic() < worker_deadline:
    time.sleep(0.01)
  worker_result = worker.take_latest()
  assert worker_result is not None and worker_result.fit is not None
  assert worker_result.fit.ready, worker_result.fit.format()
  print("HRR v2 resolver/0x25 calibration self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), help="HRR Panda CAN bus; prompted when omitted")
  parser.add_argument("--reference-bus", type=int, choices=(0, 1, 2), action="append",
                      help="source bus for steering reference 0x025; repeat as needed (default: 1)")
  parser.add_argument("--timeout", type=float, default=300.0, help="maximum guided sweep duration")
  parser.add_argument("--yes", action="store_true", help="skip the initial safety confirmation")
  parser.add_argument("--legacy", action="store_true", help="persist legacy uncalibrated output and exit")
  parser.add_argument("--calibrated", action="store_true", help="select the last valid calibration and exit")
  parser.add_argument("--abort", action="store_true", help="force-abort an active calibration and exit")
  parser.add_argument("--status", action="store_true", help="print current HRR calibration status without changing it")
  parser.add_argument("--replay-values", type=parse_replay_values, metavar="V11,...,V20",
                      help="skip the sweep and commit ten captured 32-bit values for commands 11 through 20")
  parser.add_argument("--dry-run", action="store_true", help="print command frames without opening Panda")
  parser.add_argument("--self-test", action="store_true", help="verify decoding, fitting, and frame encoding")
  args = parser.parse_args()
  if args.self_test:
    run_self_test()
    return
  if args.timeout <= 0:
    parser.error("--timeout must be greater than zero")
  if sum((args.legacy, args.calibrated, args.abort, args.status, args.replay_values is not None)) > 1:
    parser.error("--legacy, --calibrated, --abort, --status, and --replay-values are mutually exclusive")

  bus = choose_bus(args.bus)
  reference_buses = tuple(args.reference_bus) if args.reference_bus is not None else (1,)
  panda = None
  if not args.dry_run:
    from panda import Panda
    panda = Panda()
    panda.set_power_save(False)
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  session = HrrCalibrationSession(panda, bus, reference_buses, args.dry_run)
  try:
    if not args.dry_run:
      status = session.wait_for(lambda item: item.version == CAL_PROTOCOL_VERSION, 3.0)
      if status is None:
        print(f"ERROR: no HRR calibration-v{CAL_PROTOCOL_VERSION} status on bus {bus}.", file=sys.stderr)
        raise SystemExit(2)
    if args.replay_values is not None:
      if not run_replay(session, args.replay_values, args.yes):
        raise SystemExit(1)
    elif args.status:
      if session.calibration is None:
        print("ERROR: no HRR calibration status available.", file=sys.stderr)
        raise SystemExit(1)
      print(session.calibration.format())
      if session.safety is not None:
        print(session.safety.format())
    elif args.legacy or args.calibrated:
      wanted_enabled = args.calibrated
      session.send_command(CMD_CAL_MODE, int(wanted_enabled))
      if not args.dry_run:
        result = session.wait_for(lambda item: item.enabled == wanted_enabled and
                                  item.legacy_active != wanted_enabled, 3.0)
        if result is None:
          detail = "no 0x635 status" if session.calibration is None else session.calibration.format()
          print(f"ERROR: HRR did not confirm the requested calibration mode; last status: {detail}", file=sys.stderr)
          raise SystemExit(1)
        print(result.format())
    elif args.abort:
      session.send_command(CMD_CAL_ABORT, 0)
      if not args.dry_run and session.wait_for(lambda item: not item.running, 2.0) is None:
        print("ERROR: HRR did not confirm abort.", file=sys.stderr)
        raise SystemExit(1)
    elif not run_guided(session, args.timeout, args.yes):
      raise SystemExit(1)
  except (KeyboardInterrupt, EOFError) as error:
    print("\nCalibration aborted; the previous committed calibration remains unchanged.")
    session.send_command(CMD_CAL_ABORT, 0)
    raise SystemExit(130) from error
  finally:
    if panda is not None:
      panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  main()
