#!/usr/bin/env python3
"""Guided 0x25-referenced calibration for the STM32G474 HRR resolvers."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import secrets
import select
import struct
import sys
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
MAX_SAMPLE_RATE_DEG_S = 60.0
MAX_RESOLVER_PHASE_RATE_DEG_S = 360.0
MAX_FIT_RMS_DEG = 1.5
MAX_FIT_ERROR_DEG = 5.0
REFERENCE_TIMEOUT_S = 0.10
STATUS_TIMEOUT_S = 0.75
VECTOR_MIN_MAGNITUDE = 4.0
VECTOR_MATRIX_INPUT_SCALE = 1024.0

FAILURE_REASONS = {
  0: "none",
  1: "unsafe state",
  2: "bad session",
  3: "incomplete parameters",
  4: "invalid coefficient",
  5: "fit quality rejected",
  6: "flash save failed",
  7: "session timed out",
}


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
  brake_fresh: bool
  brake_pressed: bool
  brake_interlock: bool
  torque_interlock: bool

  @classmethod
  def decode(cls, payload: bytes) -> SafetyStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{STATE_STATUS_ADDR:03X}, got {len(payload)}")
    flags = int.from_bytes(payload[:2], "little")
    return cls(bool(flags & (1 << 0)), bool(flags & (1 << 1)),
               bool(flags & (1 << 5)), bool(flags & (1 << 6)),
               bool(flags & (1 << 7)), bool(flags & (1 << 15)))

  @property
  def calibration_safe(self) -> bool:
    return (not self.rel and not self.rele and self.brake_fresh and self.brake_pressed and
            self.brake_interlock and self.torque_interlock)


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
  failure_reason: int
  rms_error_deg: float

  @classmethod
  def decode(cls, payload: bytes) -> CalibrationStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{CAL_STATUS_ADDR:03X}, got {len(payload)}")
    flags, version, samples, staged_mask, reason, rms = struct.unpack("<BBHHBB", payload)
    return cls(*(bool(flags & (1 << bit)) for bit in range(8)), version, samples,
               staged_mask, reason, rms * 0.1)

  def format(self) -> str:
    mode = "LEGACY" if self.legacy_active else "CALIBRATED"
    reason = FAILURE_REASONS.get(self.failure_reason, f"unknown({self.failure_reason})")
    return (f"v{self.version} state={'RUNNING' if self.running else 'IDLE'} mode={mode} "
            f"valid={int(self.valid)} staged=0x{self.staged_mask:03x} samples={self.samples} "
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
class CalibrationFit:
  matrices: tuple[float, ...]
  phase_per_steer: float
  samples: int
  rms_error_deg: float
  max_error_deg: float
  reference_span_deg: float
  phase_span_deg: float
  positive_travel_deg: float
  negative_travel_deg: float
  occupied_bins: int

  @property
  def ready(self) -> bool:
    return (self.samples >= MIN_SAMPLES and self.reference_span_deg >= MIN_REFERENCE_SPAN_DEG and
            self.phase_span_deg >= MIN_PHASE_SPAN_DEG and
            self.positive_travel_deg >= MIN_DIRECTION_TRAVEL_DEG and
            self.negative_travel_deg >= MIN_DIRECTION_TRAVEL_DEG and self.occupied_bins >= 15 and
            self.rms_error_deg <= MAX_FIT_RMS_DEG and self.max_error_deg <= MAX_FIT_ERROR_DEG)

  def format(self) -> str:
    return (f"n={self.samples} ref_span={self.reference_span_deg:.0f}/{MIN_REFERENCE_SPAN_DEG:.0f}deg "
            f"phase_span={self.phase_span_deg:.0f}/{MIN_PHASE_SPAN_DEG:.0f}deg "
            f"travel=+{self.positive_travel_deg:.0f}/-{self.negative_travel_deg:.0f}deg "
            f"bins={self.occupied_bins}/18 ratio={self.phase_per_steer:+.6f} "
            f"rms/max={self.rms_error_deg:.2f}/{self.max_error_deg:.2f}deg")


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
  if not 0.20 <= abs(slope) <= 5.0:
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


def fit_calibration(samples: list[CalibrationSample]) -> CalibrationFit:
  if len(samples) < MIN_SAMPLES:
    raise ValueError(f"need at least {MIN_SAMPLES} samples")
  ratio, indices = robust_phase_ratio(samples)
  in_matrix, indices = solve_matrix(samples, indices, ratio, "in")
  ou_matrix, indices = solve_matrix(samples, indices, ratio, "ou")
  errors = []
  bins = set()
  for index in indices:
    sample = samples[index]
    target = (ratio * sample.reference_deg) % 180.0
    pair_errors = []
    for matrix, cos_raw, sin_raw in ((in_matrix, sample.in_cos, sample.in_sin),
                                     (ou_matrix, sample.ou_cos, sample.ou_sin)):
      measured = vector_phase_deg(matrix[0] * cos_raw + matrix[1] * sin_raw,
                                  matrix[2] * cos_raw + matrix[3] * sin_raw)
      pair_errors.append(abs(shortest_mod180_delta(measured, target)) / abs(ratio))
    errors.append(max(pair_errors))
    bins.add(int(target // 10.0) % 18)
  references = [samples[i].reference_deg for i in indices]
  phases = [samples[i].unwrapped_phase_deg for i in indices]
  positive = negative = 0.0
  for previous, current in zip(references, references[1:]):
    delta = current - previous
    if delta > 0:
      positive += delta
    else:
      negative -= delta
  return CalibrationFit(in_matrix + ou_matrix, ratio, len(indices),
                        math.sqrt(sum(error * error for error in errors) / len(errors)), max(errors),
                        max(references) - min(references), max(phases) - min(phases),
                        positive, negative, len(bins))


class HrrCalibrationSession:
  def __init__(self, panda: Panda | None, bus: int, dry_run: bool) -> None:
    self.panda = panda
    self.bus = bus
    self.dry_run = dry_run
    self.calibration: CalibrationStatus | None = None
    self.safety: SafetyStatus | None = None
    self.reference: SteerReference | None = None
    self.vector: ResolverVector | None = None
    self.reference_at: float | None = None
    self.status_at: float | None = None
    self.safety_at: float | None = None
    self.samples: list[CalibrationSample] = []
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
    if self.reference is None or self.reference_at is None or now - self.reference_at > REFERENCE_TIMEOUT_S:
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
          abs(self.reference.angle_deg - self.previous_reference) / elapsed > MAX_SAMPLE_RATE_DEG_S or
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
      if rx_bus != self.bus:
        continue
      now = time.monotonic()
      try:
        if address == STEER_ANGLE_ADDR:
          self.reference = SteerReference.decode(payload)
          self.reference_at = now
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
            self.status_at is not None and now - self.status_at <= STATUS_TIMEOUT_S and
            self.safety is not None and self.safety.calibration_safe and self.safety_at is not None and
            now - self.safety_at <= STATUS_TIMEOUT_S and self.reference is not None and
            self.reference_at is not None and now - self.reference_at <= REFERENCE_TIMEOUT_S and
            self.vector is not None)


def choose_bus(configured_bus: int | None) -> int:
  if configured_bus is not None:
    return configured_bus
  while True:
    value = input("Panda CAN bus [0/1/2]: ").strip()
    if value in {"0", "1", "2"}:
      return int(value)
    print("Enter 0, 1, or 2.")


def upload_fit(session: HrrCalibrationSession, fit: CalibrationFit) -> None:
  for command, coefficient in enumerate(fit.matrices, start=CMD_MATRIX_FIRST):
    session.send_command(command, round(coefficient * (1 << MATRIX_Q)))
    time.sleep(0.02)
  session.send_command(CMD_PHASE_PER_STEER, round(fit.phase_per_steer * (1 << MATRIX_Q)))
  rms_tenths = min(255, round(fit.rms_error_deg * 10.0))
  max_tenths = min(255, round(fit.max_error_deg * 10.0))
  metrics = min(0xFFFF, fit.samples) | (rms_tenths << 16) | (max_tenths << 24)
  session.send_command(CMD_FIT_METRICS, metrics)


def run_guided(session: HrrCalibrationSession, timeout: float, assume_yes: bool) -> bool:
  print("\nSecure the stationary vehicle. HRR relays must be open and torque output interlocked.")
  print("Hold the brake and move the wheel manually and slowly: center -> left lock -> right lock -> center.")
  if not assume_yes:
    input("Press Enter when ready, or Ctrl-C to cancel: ")
  if session.dry_run:
    session.send_command(CMD_CAL_START, 0x12345678)
    print("Dry run: start frame generated; no samples were collected.")
    return True
  if not session.live_ready():
    print("ERROR: need fresh HRR v2 status, safe interlocks, resolver vector, and CAN 0x25 reference.", file=sys.stderr)
    return False

  token = secrets.randbits(32) or 1
  session.send_command(CMD_CAL_START, token)
  started = session.wait_for(lambda status: status.running, 2.0)
  if started is None:
    print("ERROR: HRR rejected calibration start; inspect 0x635 failure reason.", file=sys.stderr)
    return False

  print("\nSweep slowly. Press Enter only after READY is shown.")
  deadline = time.monotonic() + timeout
  next_print = 0.0
  latest_fit: CalibrationFit | None = None
  while time.monotonic() < deadline:
    session.poll(collect=True)
    now = time.monotonic()
    if now >= next_print:
      try:
        latest_fit = fit_calibration(session.samples)
        text = latest_fit.format()
        state = "READY" if latest_fit.ready else "SWEEP"
      except ValueError as error:
        text = f"n={len(session.samples)} ({error})"
        state = "SWEEP"
      ref = "---.-" if session.reference is None else f"{session.reference.angle_deg:+.1f}"
      print(f"\r{state} steer={ref}deg {text}    ", end="", flush=True)
      next_print = now + 0.5
    if session.calibration is not None and not session.calibration.running:
      print(f"\nERROR: firmware aborted calibration: {session.calibration.format()}", file=sys.stderr)
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
  upload_fit(session, latest_fit)
  staged = session.wait_for(lambda status: status.running and status.parameters_complete, 2.0)
  if staged is None:
    print("ERROR: HRR did not accept the complete coefficient set.", file=sys.stderr)
    session.send_command(CMD_CAL_ABORT, token)
    return False
  session.send_command(CMD_CAL_FINISH_SAVE, token)
  finished = session.wait_for(lambda status: not status.running, 5.0)
  if finished is None or not finished.valid or not finished.enabled or finished.legacy_active or finished.failed:
    detail = "no final status" if finished is None else finished.format()
    print(f"ERROR: calibration was not committed: {detail}", file=sys.stderr)
    return False
  print(f"Calibration committed atomically: {finished.format()}")
  return True


def run_self_test() -> None:
  steer = SteerReference.decode(bytes((0x00, 0x0A, 0, 0, 0xD0, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, 15.0)
  steer = SteerReference.decode(bytes((0x0F, 0xF6, 0, 0, 0x30, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, -15.0)
  steer = SteerReference.decode(bytes((0, 0, 0, 0, 0x80, 0, 0, 0)))
  assert math.isclose(steer.angle_deg, 0.0)
  assert shortest_mod180_delta(1.0, 179.0) == 2.0
  assert shortest_mod180_delta(179.0, 1.0) == -2.0
  for command, value in ((CMD_CAL_START, 0x12345678), (CMD_CAL_FINISH_SAVE, 0x12345678),
                         (CMD_CAL_ABORT, 0), (CMD_CAL_MODE, 1), (CMD_MATRIX_FIRST, -12345)):
    frame = build_config_frame(command, value)
    assert len(frame) == 8
    assert crc8_poly07(bytes((CONFIG_ADDR & 0xFF, CONFIG_ADDR >> 8)) + frame[:7]) == frame[7]

  synthetic: list[CalibrationSample] = []
  unwrapped = 0.0
  previous = None
  references = ([-540.0 + index * 4.5 for index in range(241)] +
                [540.0 - index * 4.5 for index in range(1, 241)])
  for index, reference in enumerate(references):
    target = -0.5 * reference
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
  assert abs(fit.phase_per_steer + 0.5) < 0.01
  assert fit.rms_error_deg < 0.2
  print("HRR v2 resolver/0x25 calibration self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), help="Panda CAN bus; prompted when omitted")
  parser.add_argument("--timeout", type=float, default=300.0, help="maximum guided sweep duration")
  parser.add_argument("--yes", action="store_true", help="skip the initial safety confirmation")
  parser.add_argument("--legacy", action="store_true", help="persist legacy uncalibrated output and exit")
  parser.add_argument("--calibrated", action="store_true", help="select the last valid calibration and exit")
  parser.add_argument("--abort", action="store_true", help="force-abort an active calibration and exit")
  parser.add_argument("--dry-run", action="store_true", help="print command frames without opening Panda")
  parser.add_argument("--self-test", action="store_true", help="verify decoding, fitting, and frame encoding")
  args = parser.parse_args()
  if args.self_test:
    run_self_test()
    return
  if args.timeout <= 0:
    parser.error("--timeout must be greater than zero")
  if sum((args.legacy, args.calibrated, args.abort)) > 1:
    parser.error("--legacy, --calibrated, and --abort are mutually exclusive")

  bus = choose_bus(args.bus)
  panda = None
  if not args.dry_run:
    from panda import Panda
    panda = Panda()
    panda.set_power_save(False)
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  session = HrrCalibrationSession(panda, bus, args.dry_run)
  try:
    if not args.dry_run:
      status = session.wait_for(lambda item: item.version == CAL_PROTOCOL_VERSION, 3.0)
      if status is None:
        print(f"ERROR: no HRR calibration-v{CAL_PROTOCOL_VERSION} status on bus {bus}.", file=sys.stderr)
        raise SystemExit(2)
    if args.legacy or args.calibrated:
      wanted_enabled = args.calibrated
      session.send_command(CMD_CAL_MODE, int(wanted_enabled))
      if not args.dry_run:
        result = session.wait_for(lambda item: item.enabled == wanted_enabled and
                                  item.legacy_active != wanted_enabled, 3.0)
        if result is None:
          print("ERROR: HRR did not confirm the requested calibration mode.", file=sys.stderr)
          raise SystemExit(1)
        print(result.format())
    elif args.abort:
      session.send_command(CMD_CAL_ABORT, 0)
      if not args.dry_run and session.wait_for(lambda item: not item.running, 2.0) is None:
        print("ERROR: HRR did not confirm abort.", file=sys.stderr)
        raise SystemExit(1)
    elif not run_guided(session, args.timeout, args.yes):
      raise SystemExit(1)
  except (KeyboardInterrupt, EOFError):
    print("\nCalibration aborted; the previous committed calibration remains unchanged.")
    session.send_command(CMD_CAL_ABORT, 0)
    raise SystemExit(130)
  finally:
    if panda is not None:
      panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  main()
