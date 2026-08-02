#!/usr/bin/env python3
"""Calibrate HRR DLY and SVEC_ZERO_OFFSET from stationary driver torque."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import statistics
import struct
import sys
import threading
import time
from typing import TYPE_CHECKING

from openpilot.selfdrive.debug import hrr_can_test as hrr

if TYPE_CHECKING:
  from panda import Panda


CONFIG_ACK_TIMEOUT_S = 0.75
CONFIG_MAX_ATTEMPTS = 5
STATUS_START_TIMEOUT_S = 3.0
RELAY_ACK_TIMEOUT_S = 0.75


@dataclass(frozen=True)
class ConfigStatus:
  dly: int
  zero_offset: int
  angle_offset: int
  dirty: bool
  applied: bool
  save_succeeded: bool
  last_command: int
  counter: int

  @classmethod
  def decode(cls, payload: bytes) -> ConfigStatus:
    return cls(*hrr.decode_config_status(payload))


@dataclass(frozen=True)
class TorqueSample:
  timestamp: float
  driver_torque: int
  eps_torque: int


@dataclass(frozen=True)
class Measurement:
  phase: str
  dly: int
  zero_offset: int
  command_torque: int
  samples: int
  mean: float
  median: float
  std: float
  robust_std: float
  maximum_abs: int
  score: float

  def format(self) -> str:
    return (f"DLY={self.dly:3d} zero={self.zero_offset:+4d} ({self.zero_offset / 10:+.1f}deg) "
            + f"cmd={self.command_torque:+4d} "
            + f"mean={self.mean:+7.2f} median={self.median:+7.2f} "
            + f"std={self.std:6.2f} robust_std={self.robust_std:6.2f} "
            + f"score={self.score:6.2f} N={self.samples}")


@dataclass(frozen=True)
class LoadedValidation:
  zero: Measurement
  positive: tuple[Measurement, ...]
  negative: tuple[Measurement, ...]
  returns: tuple[Measurement, ...]
  validation_torque: int
  noise_weight: float

  @staticmethod
  def _mean(points: tuple[Measurement, ...]) -> float:
    return statistics.mean(point.mean for point in points)

  @property
  def positive_mean(self) -> float:
    return self._mean(self.positive)

  @property
  def negative_mean(self) -> float:
    return self._mean(self.negative)

  @property
  def return_mean(self) -> float:
    return self._mean(self.returns)

  @property
  def odd_response(self) -> float:
    return (self.positive_mean - self.negative_mean) / 2.0

  @property
  def response_gain(self) -> float:
    return self.odd_response / self.validation_torque

  @property
  def even_error(self) -> float:
    return ((self.positive_mean + self.negative_mean) / 2.0) - self.zero.mean

  @property
  def asymmetry_percent(self) -> float:
    positive_magnitude = abs(self.positive_mean - self.zero.mean)
    negative_magnitude = abs(self.negative_mean - self.zero.mean)
    average_magnitude = (positive_magnitude + negative_magnitude) / 2.0
    return 100.0 * abs(positive_magnitude - negative_magnitude) / max(average_magnitude, 1.0)

  @property
  def repeatability(self) -> float:
    polarity_ranges = (
      max(point.mean for point in points) - min(point.mean for point in points)
      for points in (self.positive, self.negative)
    )
    return max(polarity_ranges)

  @property
  def return_rms(self) -> float:
    return math.sqrt(statistics.mean((point.mean - self.zero.mean) ** 2 for point in self.returns))

  @property
  def loaded_noise(self) -> float:
    loaded = self.positive + self.negative
    return math.sqrt(statistics.mean(point.robust_std ** 2 for point in loaded))

  @property
  def loaded_score(self) -> float:
    return math.sqrt(self.even_error ** 2 + self.return_rms ** 2 + self.repeatability ** 2 +
                     (self.noise_weight * self.loaded_noise) ** 2)

  @property
  def overall_score(self) -> float:
    return math.hypot(self.zero.score, self.loaded_score)

  def cautions(self) -> list[str]:
    response = abs(self.odd_response)
    tolerance = max(15.0, response * 0.20)
    cautions = []
    if response < max(5.0, abs(self.validation_torque) * 0.02):
      cautions.append("weak signed response")
    if self.asymmetry_percent > 25.0:
      cautions.append(f"{self.asymmetry_percent:.0f}% polarity asymmetry")
    if abs(self.even_error) > tolerance:
      cautions.append(f"loaded midpoint error {self.even_error:+.1f} Ncm")
    if self.return_rms > tolerance:
      cautions.append(f"return-to-zero RMS {self.return_rms:.1f} Ncm")
    if self.repeatability > tolerance:
      cautions.append(f"polarity repeatability range {self.repeatability:.1f} Ncm")
    return cautions


@dataclass(frozen=True)
class CalibrationResult:
  zero: Measurement
  loaded: LoadedValidation | None

  @property
  def overall_score(self) -> float:
    return self.zero.score if self.loaded is None else self.loaded.overall_score

  def selection_score(self, response_reference: float = 0.0) -> float:
    if self.loaded is None or response_reference <= 0.0:
      return self.overall_score
    response_deviation = abs(abs(self.loaded.odd_response) - response_reference)
    return math.hypot(self.overall_score, response_deviation)


def loaded_response_reference(results: list[CalibrationResult]) -> float:
  responses = [abs(result.loaded.odd_response) for result in results if result.loaded is not None]
  return statistics.median(responses) if responses else 0.0


def center_out_values(center: int, low: int, high: int) -> list[int]:
  values = [center]
  for distance in range(1, max(center - low, high - center) + 1):
    if center - distance >= low:
      values.append(center - distance)
    if center + distance <= high:
      values.append(center + distance)
  return values


def fit_zero_root(points: list[Measurement], low: int, high: int) -> int:
  if len(points) < 2:
    return max(low, min(high, points[0].zero_offset if points else 0))
  xs = [float(point.zero_offset) for point in points]
  ys = [point.mean for point in points]
  x_mean = statistics.mean(xs)
  y_mean = statistics.mean(ys)
  denominator = sum((x - x_mean) ** 2 for x in xs)
  if denominator <= 0.0:
    return round(x_mean)
  slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys, strict=True)) / denominator
  if abs(slope) < 0.2:
    return min(points, key=lambda point: abs(point.mean)).zero_offset
  return max(low, min(high, round(x_mean - y_mean / slope)))


def summarize_samples(phase: str, dly: int, zero_offset: int, command_torque: int,
                      samples: list[TorqueSample], noise_weight: float) -> Measurement:
  if len(samples) < 5:
    raise RuntimeError(f"only {len(samples)} driver-torque samples received")
  values = sorted(sample.driver_torque for sample in samples)
  trim = len(values) // 10
  trimmed = values[trim:len(values) - trim] if trim > 0 and len(values) - (2 * trim) >= 5 else values
  mean = statistics.mean(trimmed)
  median = statistics.median(trimmed)
  std = statistics.pstdev(trimmed)
  mad = statistics.median(abs(value - median) for value in trimmed)
  robust_std = 1.4826 * mad
  # Static bias and noise both matter. The robust spread prevents a few CAN or
  # mechanical transients from dominating, while std remains visible to the operator.
  score = math.hypot(mean, noise_weight * robust_std)
  return Measurement(phase, dly, zero_offset, command_torque, len(trimmed), mean, median, std,
                     robust_std, max(abs(value) for value in values), score)


class CalibrationSession:
  def __init__(self, panda: Panda, bus: int, torque_bus: int, rate_hz: float,
               max_driver_torque: int) -> None:
    self.panda = panda
    self.bus = bus
    self.torque_bus = torque_bus
    self.period = 1.0 / rate_hz
    self.max_driver_torque = max_driver_torque
    self.lock = threading.Condition()
    self.config: ConfigStatus | None = None
    self.rel: bool | None = None
    self.rele: bool | None = None
    self.samples: deque[TorqueSample] = deque(maxlen=20000)
    self.engaged = False
    self.requested_torque = 0
    self.brake_pressed = False
    self.counter = 0
    self.stop_event = threading.Event()
    self.safety_event = threading.Event()
    self.monitor_thread = threading.Thread(target=self._monitor_loop, name="hrr-cal-monitor", daemon=True)
    self.stream_thread = threading.Thread(target=self._stream_loop, name="hrr-cal-stream", daemon=True)

  def start(self) -> None:
    self.panda.can_clear(0xFFFF)
    self.monitor_thread.start()
    self.stream_thread.start()

  def _monitor_loop(self) -> None:
    while not self.stop_event.is_set():
      for address, _, payload, rx_bus in self.panda.can_recv():
        now = time.monotonic()
        if rx_bus == self.bus and address == hrr.CONFIG_STATUS_ADDR:
          try:
            config = ConfigStatus.decode(payload)
          except ValueError:
            continue
          with self.lock:
            self.config = config
            self.lock.notify_all()
        elif rx_bus == self.bus and address == hrr.IO_STATUS_ADDR:
          try:
            rel, rele = hrr.decode_io_status(payload)
          except ValueError:
            continue
          with self.lock:
            self.rel, self.rele = rel, rele
            self.lock.notify_all()
        elif rx_bus == self.torque_bus and address == hrr.STEER_TORQUE_SENSOR_ADDR:
          try:
            driver_torque, eps_torque = hrr.decode_steer_torque_sensor(payload)
          except ValueError:
            continue
          with self.lock:
            self.samples.append(TorqueSample(now, driver_torque, eps_torque))
            engaged = self.engaged
            self.lock.notify_all()
          if engaged and abs(driver_torque) > self.max_driver_torque:
            self.safety_event.set()
      self.stop_event.wait(0.005)

  def _stream_loop(self) -> None:
    next_send = time.monotonic()
    while not self.stop_event.is_set():
      with self.lock:
        engaged = self.engaged
        brake_pressed = self.brake_pressed
        requested_torque = self.requested_torque
        counter = self.counter
        self.counter = (self.counter + 1) & 0x0F
      self.panda.can_send(hrr.BRAKE_ADDR, hrr.build_brake_frame(brake_pressed), self.bus)
      self.panda.can_send(hrr.TORQUE_ADDR, hrr.build_torque_frame(requested_torque, engaged, counter), self.bus)
      next_send += self.period
      delay = next_send - time.monotonic()
      if delay < 0:
        next_send = time.monotonic()
      else:
        self.stop_event.wait(delay)

  def set_engaged(self, engaged: bool) -> None:
    with self.lock:
      self.engaged = engaged
      if not engaged:
        self.requested_torque = 0
      deadline = time.monotonic() + RELAY_ACK_TIMEOUT_S
      while time.monotonic() < deadline:
        if self.rel is engaged and self.rele is engaged:
          return
        self.lock.wait(max(0.0, deadline - time.monotonic()))
      actual = f"REL={self.rel} RELE={self.rele}"
    raise RuntimeError(f"HRR relays did not report {'ON' if engaged else 'OFF'} ({actual})")

  def set_requested_torque(self, torque_ncm: int) -> None:
    if not -hrr.MAX_TORQUE_NCM <= torque_ncm <= hrr.MAX_TORQUE_NCM:
      raise ValueError(f"validation torque must be within +/-{hrr.MAX_TORQUE_NCM} Ncm")
    with self.lock:
      if self.engaged:
        raise RuntimeError("refusing to change requested torque while HRR relays are engaged")
      self.requested_torque = torque_ncm

  def wait_initial_status(self) -> ConfigStatus:
    deadline = time.monotonic() + STATUS_START_TIMEOUT_S
    with self.lock:
      while self.config is None and time.monotonic() < deadline:
        self.lock.wait(deadline - time.monotonic())
      if self.config is None:
        raise RuntimeError("no HRR_ConfigStatus (0x639); flash firmware with transient-settings support")
      return self.config

  def _send_config_and_wait(self, command: int, value: int, *, signed: bool,
                            predicate) -> ConfigStatus:
    for _ in range(CONFIG_MAX_ATTEMPTS):
      with self.lock:
        before = None if self.config is None else self.config.counter
      payload = hrr.build_svec_config_frame(command, value, signed=signed)
      self.panda.can_send(hrr.SVEC_CONFIG_ADDR, payload, self.bus)
      deadline = time.monotonic() + CONFIG_ACK_TIMEOUT_S
      with self.lock:
        while time.monotonic() < deadline:
          status = self.config
          if (status is not None and status.counter != before and
              status.last_command == command and status.applied and predicate(status)):
            return status
          self.lock.wait(max(0.0, deadline - time.monotonic()))
    raise RuntimeError(f"HRR did not acknowledge config command {command} value {value}")

  def apply_pair(self, dly: int, zero_offset: int) -> None:
    self.set_engaged(False)
    with self.lock:
      current = self.config
    if current is None or current.dly != dly:
      self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_DLY, dly, signed=False,
                                 predicate=lambda status: status.dly == dly)
    with self.lock:
      current = self.config
    if current is None or current.zero_offset != zero_offset:
      self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_ZERO_OFFSET, zero_offset, signed=True,
                                 predicate=lambda status: status.zero_offset == zero_offset)

  def save_current(self) -> ConfigStatus:
    return self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_SAVE, 0, signed=False,
                                      predicate=lambda status: status.save_succeeded and not status.dirty)

  def measure(self, phase: str, dly: int, zero_offset: int, command_torque: int, settle_s: float,
              sample_s: float, noise_weight: float) -> Measurement:
    self.apply_pair(dly, zero_offset)
    self.set_requested_torque(command_torque)
    self.safety_event.clear()
    self.set_engaged(True)
    settle_deadline = time.monotonic() + settle_s
    while time.monotonic() < settle_deadline:
      if self.safety_event.wait(min(0.05, settle_deadline - time.monotonic())):
        self.set_engaged(False)
        raise RuntimeError(f"driver torque exceeded safety limit {self.max_driver_torque} Ncm")
    started = time.monotonic()
    deadline = started + sample_s
    while time.monotonic() < deadline:
      if self.safety_event.wait(min(0.05, deadline - time.monotonic())):
        self.set_engaged(False)
        raise RuntimeError(f"driver torque exceeded safety limit {self.max_driver_torque} Ncm")
    ended = time.monotonic()
    self.set_engaged(False)
    with self.lock:
      samples = [sample for sample in self.samples if started <= sample.timestamp <= ended]
    minimum_samples = max(5, round(sample_s * 20.0))
    if len(samples) < minimum_samples:
      raise RuntimeError(f"only {len(samples)} torque samples in {sample_s:g}s; expected at least {minimum_samples}")
    measurement = summarize_samples(phase, dly, zero_offset, command_torque, samples, noise_weight)
    print(f"  {measurement.format()}")
    return measurement

  def shutdown(self) -> None:
    with self.lock:
      self.engaged = False
      self.requested_torque = 0
      self.brake_pressed = True
    time.sleep(max(0.06, 5.0 * self.period))
    self.stop_event.set()
    self.stream_thread.join(timeout=1.0)
    self.monitor_thread.join(timeout=1.0)


def calibrate_zero(session: CalibrationSession, dly: int, seed: int, args,
                   phase: str) -> tuple[Measurement, list[Measurement]]:
  low = -args.zero_limit
  high = args.zero_limit
  seed = max(low, min(high, seed))
  offsets = [seed, max(low, seed - args.zero_probe), min(high, seed + args.zero_probe)]
  measurements: dict[int, Measurement] = {}

  def evaluate(offset: int) -> Measurement:
    if offset not in measurements:
      measurements[offset] = session.measure(phase, dly, offset, 0, args.settle_time,
                                             args.sample_time, args.noise_weight)
    return measurements[offset]

  for offset in dict.fromkeys(offsets):
    evaluate(offset)
  root = fit_zero_root(list(measurements.values()), low, high)
  for offset in (root, max(low, root - 1), min(high, root + 1)):
    evaluate(offset)
  best = min(measurements.values(), key=lambda point: (point.score, abs(point.mean), point.std))
  return best, list(measurements.values())


def validate_loaded(session: CalibrationSession, zero: Measurement, args) -> tuple[LoadedValidation, list[Measurement]]:
  torque = args.validation_torque
  assert torque is not None and torque > 0
  sequence = (torque, 0, -torque, 0, -torque, 0, torque, 0)
  measurements = []
  print(f"\nCounterbalanced loaded validation for DLY={zero.dly}, zero={zero.zero_offset:+d}, T={torque} Ncm:")
  for command_torque in sequence:
    phase = "loaded_return" if command_torque == 0 else ("loaded_positive" if command_torque > 0 else "loaded_negative")
    measurements.append(session.measure(phase, zero.dly, zero.zero_offset, command_torque,
                                        args.settle_time, args.load_sample_time, args.noise_weight))
  positive = tuple(point for point in measurements if point.command_torque > 0)
  negative = tuple(point for point in measurements if point.command_torque < 0)
  returns = tuple(point for point in measurements if point.command_torque == 0)
  loaded = LoadedValidation(zero, positive, negative, returns, torque, args.noise_weight)
  print(f"  summary: +T={loaded.positive_mean:+.2f} -T={loaded.negative_mean:+.2f} "
        + f"gain={loaded.response_gain:+.4f} driver/cmd even_error={loaded.even_error:+.2f} "
        + f"asymmetry={loaded.asymmetry_percent:.1f}% return_rms={loaded.return_rms:.2f} "
        + f"repeat={loaded.repeatability:.2f} loaded_noise={loaded.loaded_noise:.2f}")
  return loaded, measurements


def print_ranking(results: list[CalibrationResult], original: ConfigStatus,
                  dly_low: int, dly_high: int, zero_limit: int) -> None:
  response_reference = loaded_response_reference(results)
  ranked = sorted(results, key=lambda result: result.selection_score(response_reference))
  if ranked[0].loaded is None:
    return
  print("\nFinalist ranking (lower score is better):")
  print(" rk DLY zero | zero_mean zero_sd |    +T     -T   gain | even  asym% return repeat noise rdev | score")
  print(" -- --- ---- | --------- ------- | ------ ------ ------ | ----- ------ ------ ------ ----- ---- | -----")
  for rank, result in enumerate(ranked, 1):
    loaded = result.loaded
    assert loaded is not None
    print(f" {rank:2d} {result.zero.dly:3d} {result.zero.zero_offset:+4d} | "
          + f"{result.zero.mean:+9.2f} {result.zero.robust_std:7.2f} | "
          + f"{loaded.positive_mean:+6.1f} {loaded.negative_mean:+6.1f} {loaded.response_gain:+6.3f} | "
          + f"{loaded.even_error:+5.1f} {loaded.asymmetry_percent:6.1f} {loaded.return_rms:6.1f} "
          + f"{loaded.repeatability:6.1f} {loaded.loaded_noise:5.1f} "
          + f"{abs(abs(loaded.odd_response) - response_reference):4.1f} | "
          + f"{result.selection_score(response_reference):5.1f}")

  winner = ranked[0]
  loaded = winner.loaded
  assert loaded is not None
  print("\nRecommendation diagnostics:")
  print(f"  Selected DLY={winner.zero.dly}, SVEC_ZERO_OFFSET={winner.zero.zero_offset:+d} "
        + f"({winner.zero.zero_offset / 10:+.1f}deg).")
  print(f"  Zero: {winner.zero.mean:+.2f} Ncm mean, {winner.zero.robust_std:.2f} Ncm robust std.")
  print(f"  Loaded: {loaded.odd_response:+.2f} Ncm odd response at +/-{loaded.validation_torque} Ncm command, "
        + f"gain {loaded.response_gain:+.4f}.")
  print(f"  Response consistency: finalist median={response_reference:.2f} Ncm, "
        + f"deviation={abs(abs(loaded.odd_response) - response_reference):.2f} Ncm.")
  print(f"  Symmetry: {loaded.even_error:+.2f} Ncm midpoint error, {loaded.asymmetry_percent:.1f}% magnitude asymmetry.")
  print(f"  Stability: {loaded.return_rms:.2f} Ncm return RMS, {loaded.repeatability:.2f} Ncm repeatability range, "
        + f"{loaded.loaded_noise:.2f} Ncm loaded noise.")
  cautions = loaded.cautions()
  if response_reference >= 10.0 and abs(loaded.odd_response) < response_reference * 0.5:
    cautions.append("loaded response is less than half the finalist median")
  if winner.zero.dly in {dly_low, dly_high} and dly_low != dly_high:
    cautions.append("best DLY is at the sweep boundary; repeat with a wider DLY offset")
  if abs(winner.zero.zero_offset) == zero_limit:
    cautions.append("best zero offset is at the configured search boundary")
  if cautions:
    print("  CAUTION: " + "; ".join(cautions) + ".")
  else:
    print("  PASS: centered, symmetric, repeatable, and stable after both torque directions.")
  if winner.zero.dly != original.dly:
    print(f"  DLY moves {winner.zero.dly - original.dly:+d} samples from the original value {original.dly}.")


def run_calibration(session: CalibrationSession, original: ConfigStatus, dly_offset: int,
                    args) -> tuple[CalibrationResult, list[Measurement]]:
  dly_low = max(0, original.dly - dly_offset)
  dly_high = min(hrr.MAX_DLY_SAMPLES, original.dly + dly_offset)
  dly_values = center_out_values(original.dly, dly_low, dly_high)
  print(f"\nProfiling DLY {dly_low}..{dly_high}; each DLY gets its own fitted zero offset.")
  all_measurements: list[Measurement] = []
  best_by_dly: list[Measurement] = []
  zero_seed = 0
  for index, dly in enumerate(dly_values, 1):
    print(f"\n[{index}/{len(dly_values)}] DLY={dly}")
    best, measurements = calibrate_zero(session, dly, zero_seed, args, "profile")
    all_measurements.extend(measurements)
    best_by_dly.append(best)
    zero_seed = best.zero_offset
    print(f"  best for DLY {dly}: {best.format()}")

  finalists = sorted(best_by_dly, key=lambda point: point.score)[:min(args.finalists, len(best_by_dly))]
  print("\nLong-window validation of the best profiled pairs:")
  validated: list[Measurement] = []
  for finalist in finalists:
    measurement = session.measure("validation", finalist.dly, finalist.zero_offset, 0,
                                  args.settle_time, args.validation_time, args.noise_weight)
    all_measurements.append(measurement)
    validated.append(measurement)
  results = [CalibrationResult(point, None) for point in validated]
  if args.validation_torque > 0:
    results = []
    for point in validated:
      loaded, measurements = validate_loaded(session, point, args)
      all_measurements.extend(measurements)
      results.append(CalibrationResult(point, loaded))
  response_reference = loaded_response_reference(results)
  results.sort(key=lambda result: (result.selection_score(response_reference), abs(result.zero.mean),
                                    abs(result.zero.dly - original.dly)))
  print_ranking(results, original, dly_low, dly_high, args.zero_limit)
  return results[0], all_measurements


def confirm_safety(args) -> None:
  print("\nWARNING: this calibration repeatedly engages the HRR resolver relays at zero and low symmetric torque.")
  print("Secure the stationary vehicle, keep hands and people clear of the steering wheel and linkage,")
  print("choose a load that cannot rotate the wheel, and be ready to remove power. The script aborts if")
  print("measured Driver_Torque exceeds the configured limit.")
  if not args.yes and input("Type CALIBRATE to continue: ").strip() != "CALIBRATE":
    raise RuntimeError("calibration not confirmed")


def run_self_test() -> None:
  assert center_out_values(3, 1, 5) == [3, 2, 4, 1, 5]
  synthetic = [
    Measurement("test", 20, zero, 0, 100, 4.0 * zero + 12.0, 0.0, 1.0, 1.0, 50, 1.0)
    for zero in (-5, 0, 5)
  ]
  assert fit_zero_root(synthetic, -135, 135) == -3
  samples = [TorqueSample(float(index), value, -value) for index, value in enumerate([8, 9, 10, 10, 11, 12, 100])]
  summary = summarize_samples("test", 20, -3, 0, samples, 1.0)
  assert summary.samples == 7 and summary.median == 10 and summary.maximum_abs == 100
  def point(command: int, mean: float) -> Measurement:
    return Measurement("load", 20, -3, command, 100, mean, mean, 2.0, 2.0, 50, 2.0)
  loaded = LoadedValidation(point(0, 1.0), (point(100, 51.0), point(100, 53.0)),
                            (point(-100, -49.0), point(-100, -47.0)),
                            (point(0, 2.0), point(0, 0.0), point(0, 1.0), point(0, 1.0)), 100, 1.0)
  assert loaded.positive_mean == 52.0 and loaded.negative_mean == -48.0
  assert loaded.odd_response == 50.0 and loaded.response_gain == 0.5
  assert loaded.even_error == 1.0 and loaded.repeatability == 2.0
  status = ConfigStatus.decode(struct.pack("<BhhBBB", 27, -3, 40, 0x06, 9, 12))
  assert status.dly == 27 and status.zero_offset == -3 and status.applied and status.save_succeeded
  print("HRR torque-calibration self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), default=2, help="HRR command/status Panda bus")
  parser.add_argument("--torque-bus", type=int, choices=(0, 1, 2), default=0,
                      help="Panda bus carrying Toyota STEER_TORQUE_SENSOR 0x260")
  parser.add_argument("--dly-offset", type=int, help="maximum samples below/above the reported current DLY")
  parser.add_argument("--zero-limit", type=int, default=hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG,
                      help="absolute SVEC_ZERO_OFFSET search limit in tenths of a degree")
  parser.add_argument("--zero-probe", type=int, default=5,
                      help="offset distance used to estimate the local zero-torque slope")
  parser.add_argument("--settle-time", type=float, default=0.75, help="discarded settling time for each point")
  parser.add_argument("--sample-time", type=float, default=1.5, help="torque sampling time for each profile point")
  parser.add_argument("--validation-time", type=float, default=5.0, help="long sampling time for finalists")
  parser.add_argument("--validation-torque", type=int,
                      help="symmetric finalist torque in Ncm; prompted when omitted, 0 disables")
  parser.add_argument("--load-sample-time", type=float, default=1.5,
                      help="sampling time for each point in the counterbalanced loaded test")
  parser.add_argument("--finalists", type=int, default=3, help="number of best pairs to revalidate")
  parser.add_argument("--noise-weight", type=float, default=1.0, help="robust std weight in the RMS-like score")
  parser.add_argument("--max-driver-torque", type=int, default=600,
                      help="abort threshold for absolute Driver_Torque in Ncm")
  parser.add_argument("--rate-hz", type=float, default=hrr.DEFAULT_RATE_HZ, help="brake/zero-torque stream rate")
  parser.add_argument("--force-harness-relay", action=argparse.BooleanOptionalAction, default=True,
                      help="force Panda intercept and disable firmware forwarding during calibration")
  parser.add_argument("--allow-dirty-start", action="store_true",
                      help="allow calibration when active settings already differ from flash")
  parser.add_argument("--yes", action="store_true", help="skip safety and final-save confirmations")
  parser.add_argument("--self-test", action="store_true", help="run optimizer/protocol unit checks and exit")
  args = parser.parse_args()

  if args.self_test:
    run_self_test()
    return
  for name in ("settle_time", "sample_time", "validation_time", "load_sample_time", "noise_weight", "rate_hz"):
    if getattr(args, name) <= 0:
      parser.error(f"--{name.replace('_', '-')} must be greater than zero")
  if args.zero_probe <= 0 or args.zero_limit <= 0 or args.finalists <= 0 or args.max_driver_torque <= 0:
    parser.error("zero probe/limit, finalists, and torque limit must be greater than zero")
  if args.zero_limit > hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG:
    parser.error(f"--zero-limit cannot exceed {hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG}")
  if args.validation_torque is not None and not 0 <= args.validation_torque <= hrr.MAX_TORQUE_NCM:
    parser.error(f"--validation-torque must be within 0..{hrr.MAX_TORQUE_NCM} Ncm")

  try:
    confirm_safety(args)
  except RuntimeError as error:
    print(f"ERROR: {error}", file=sys.stderr)
    raise SystemExit(1) from error
  from panda import Panda
  panda = Panda()
  panda.set_power_save(False)
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
  session = CalibrationSession(panda, args.bus, args.torque_bus, args.rate_hz, args.max_driver_torque)
  relay_forced = False
  session_started = False
  original: ConfigStatus | None = None
  keep_final = False
  failed = False
  try:
    if args.force_harness_relay:
      panda.set_force_intercept_relay(True)
      panda.set_safety_forwarding_disabled(True)
      relay_forced = True
    session.start()
    session_started = True
    original = session.wait_initial_status()
    print(f"Current active settings: DLY={original.dly}, zero={original.zero_offset:+d} "
          + f"({original.zero_offset / 10:+.1f}deg), angle_offset={original.angle_offset:+d}, dirty={original.dirty}")
    if original.dirty and not args.allow_dirty_start:
      raise RuntimeError("HRR reports unsaved active settings; save/reboot them or pass --allow-dirty-start")
    if args.dly_offset is None:
      args.dly_offset = int(input("Maximum DLY sweep offset in samples: ").strip(), 0)
    if args.dly_offset < 0:
      raise RuntimeError("DLY sweep offset must be non-negative")
    if args.validation_torque is None:
      entered = input("Symmetric finalist validation torque in Ncm [100, 0 disables]: ").strip()
      args.validation_torque = 100 if not entered else int(entered, 0)
    if not 0 <= args.validation_torque <= hrr.MAX_TORQUE_NCM:
      raise RuntimeError(f"validation torque must be within 0..{hrr.MAX_TORQUE_NCM} Ncm")

    winner, _ = run_calibration(session, original, args.dly_offset, args)
    print("\nRecommended coupled calibration:")
    print(f"  {winner.zero.format()}")
    save = args.yes or input("Persist this DLY and SVEC_ZERO_OFFSET to HRR flash? [y/N]: ").strip().lower() in {"y", "yes"}
    if save:
      session.apply_pair(winner.zero.dly, winner.zero.zero_offset)
      saved = session.save_current()
      keep_final = True
      print(f"Saved once: DLY={saved.dly}, zero={saved.zero_offset:+d}, dirty={saved.dirty}.")
    else:
      print("Not saved; restoring the original active settings.")
  except (KeyboardInterrupt, EOFError):
    print("\nCalibration interrupted.", file=sys.stderr)
    failed = True
  except (RuntimeError, ValueError) as error:
    print(f"ERROR: {error}", file=sys.stderr)
    failed = True
  finally:
    if original is not None and not keep_final:
      try:
        session.apply_pair(original.dly, original.zero_offset)
        print(f"Restored active DLY={original.dly}, zero={original.zero_offset:+d}; flash was not changed.")
      except RuntimeError as error:
        print(f"WARNING: could not restore original active settings: {error}", file=sys.stderr)
        failed = True
    if session_started:
      session.shutdown()
    panda.set_safety_mode(Panda.SAFETY_SILENT)
    if relay_forced:
      panda.set_force_intercept_relay(False)
      panda.set_safety_forwarding_disabled(False)
  if failed:
    raise SystemExit(1)


if __name__ == "__main__":
  main()
