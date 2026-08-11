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
NATIVE_BASELINE_TIME_S = 5.0


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
  measurements: tuple[Measurement, ...]
  validation_torque: int
  noise_weight: float

  @property
  def loaded_points(self) -> tuple[Measurement, ...]:
    return tuple(self.measurements[index] for index in range(1, len(self.measurements), 2))

  @property
  def zero_points(self) -> tuple[Measurement, ...]:
    return tuple(self.measurements[index] for index in range(0, len(self.measurements), 2))

  def _local_response(self, index: int) -> float:
    point = self.measurements[index]
    local_zero = (self.measurements[index - 1].mean + self.measurements[index + 1].mean) / 2.0
    return point.mean - local_zero

  @property
  def positive_responses(self) -> tuple[float, ...]:
    return tuple(self._local_response(index) for index in range(1, len(self.measurements) - 1, 2)
                 if self.measurements[index].command_torque > 0)

  @property
  def negative_responses(self) -> tuple[float, ...]:
    return tuple(self._local_response(index) for index in range(1, len(self.measurements) - 1, 2)
                 if self.measurements[index].command_torque < 0)

  @property
  def positive_mean(self) -> float:
    return statistics.mean(self.positive_responses)

  @property
  def negative_mean(self) -> float:
    return statistics.mean(self.negative_responses)

  @property
  def return_mean(self) -> float:
    return statistics.mean(point.mean for point in self.zero_points)

  @property
  def initial_loaded_zero(self) -> float:
    return self.zero_points[0].mean

  @property
  def initial_zero_shift(self) -> float:
    return self.initial_loaded_zero - self.zero.mean

  @property
  def odd_response(self) -> float:
    return (self.positive_mean - self.negative_mean) / 2.0

  @property
  def response_gain(self) -> float:
    return self.odd_response / self.validation_torque

  @property
  def even_error(self) -> float:
    return (self.positive_mean + self.negative_mean) / 2.0

  @property
  def asymmetry_percent(self) -> float:
    positive_magnitude = abs(self.positive_mean)
    negative_magnitude = abs(self.negative_mean)
    average_magnitude = (positive_magnitude + negative_magnitude) / 2.0
    return 100.0 * abs(positive_magnitude - negative_magnitude) / max(average_magnitude, 1.0)

  @property
  def repeatability(self) -> float:
    return max(max(self.positive_responses) - min(self.positive_responses),
               max(self.negative_responses) - min(self.negative_responses))

  @property
  def return_rms(self) -> float:
    return math.sqrt(statistics.mean((point.mean - self.zero.mean) ** 2 for point in self.zero_points))

  @property
  def zero_span(self) -> float:
    zeros = [point.mean for point in self.zero_points]
    return max(zeros) - min(zeros)

  @property
  def loaded_noise(self) -> float:
    loaded = self.loaded_points
    return math.sqrt(statistics.mean(point.std ** 2 for point in loaded))

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
      cautions.append(f"{self.asymmetry_percent:.0f}% local-response polarity asymmetry")
    if abs(self.even_error) > tolerance:
      cautions.append(f"local-response midpoint error {self.even_error:+.1f} Ncm")
    if self.return_rms > tolerance:
      cautions.append(f"zero-baseline RMS from refined zero {self.return_rms:.1f} Ncm")
    if self.repeatability > tolerance:
      cautions.append(f"local-response repeatability range {self.repeatability:.1f} Ncm")
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


def dly_sweep_groups(center: int, low: int, high: int) -> tuple[list[int], list[int], list[int]]:
  center_group = [center]
  low_group = list(range(center - 1, low - 1, -1))
  high_group = list(range(center + 1, high + 1))
  return center_group, low_group, high_group


def profile_finalist_score(point: Measurement, zero_tolerance: float, noise_weight: float) -> float:
  # Once the zero fit is inside tolerance, DLY selection should be driven by waveform noise,
  # not by sub-tolerance short-window bias that the long finalist re-fit will correct.
  residual_bias = max(0.0, abs(point.mean) - zero_tolerance)
  return math.hypot(residual_bias, noise_weight * point.std)


def fit_zero_root(points: list[Measurement], low: int, high: int) -> int:
  if len(points) < 2:
    return max(low, min(high, points[0].zero_offset if points else 0))
  best_bias = min(points, key=lambda point: abs(point.mean))
  fit_points = sorted(points, key=lambda point: abs(point.zero_offset - best_bias.zero_offset))[:min(3, len(points))]
  xs = [float(point.zero_offset) for point in fit_points]
  ys = [point.mean for point in fit_points]
  x_mean = statistics.mean(xs)
  y_mean = statistics.mean(ys)
  denominator = sum((x - x_mean) ** 2 for x in xs)
  if denominator <= 0.0:
    return round(x_mean)
  slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys, strict=True)) / denominator
  if abs(slope) < 0.2:
    return min(fit_points, key=lambda point: abs(point.mean)).zero_offset
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
  # The outer 10% has already been trimmed. Use continuous std for ranking; MAD remains diagnostic.
  score = math.hypot(mean, noise_weight * std)
  return Measurement(phase, dly, zero_offset, command_torque, len(trimmed), mean, median, std,
                     robust_std, max(abs(value) for value in values), score)


class CalibrationSession:
  def __init__(self, panda: Panda, bus: int, torque_bus: int, rate_hz: float,
               max_driver_torque: int, transition_step_time: float,
               torque_ramp_step: int) -> None:
    self.panda = panda
    self.bus = bus
    self.torque_bus = torque_bus
    self.period = 1.0 / rate_hz
    self.max_driver_torque = max_driver_torque
    self.transition_step_time = transition_step_time
    self.torque_ramp_step = torque_ramp_step
    self.lock = threading.Condition()
    self.config: ConfigStatus | None = None
    self.rel: bool | None = None
    self.rele: bool | None = None
    self.samples: deque[TorqueSample] = deque(maxlen=20000)
    self.engaged = False
    self.requested_torque = 0
    self.brake_pressed = False
    self.counter = 0
    self.rx_frames = 0
    self.rx_frames_by_bus = [0, 0, 0]
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
        with self.lock:
          self.rx_frames += 1
          if 0 <= rx_bus < len(self.rx_frames_by_bus):
            self.rx_frames_by_bus[rx_bus] += 1
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
      if self.engaged is engaged and self.rel is engaged and self.rele is engaged:
        return
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

  def _abort_if_unsafe(self) -> None:
    if not self.safety_event.is_set():
      return
    try:
      self.set_engaged(False)
    finally:
      raise RuntimeError(f"driver torque exceeded safety limit {self.max_driver_torque} Ncm")

  def _transition_wait(self) -> None:
    if self.transition_step_time <= 0.0:
      self._abort_if_unsafe()
      return
    if self.safety_event.wait(self.transition_step_time):
      self._abort_if_unsafe()

  def set_requested_torque(self, torque_ncm: int) -> None:
    if not -hrr.MAX_TORQUE_NCM <= torque_ncm <= hrr.MAX_TORQUE_NCM:
      raise ValueError(f"validation torque must be within +/-{hrr.MAX_TORQUE_NCM} Ncm")
    with self.lock:
      current = self.requested_torque
      engaged = self.engaged
    if not engaged:
      with self.lock:
        self.requested_torque = torque_ncm
      return
    while current != torque_ncm:
      step = min(self.torque_ramp_step, abs(torque_ncm - current))
      current += step if torque_ncm > current else -step
      with self.lock:
        self.requested_torque = current
      self._transition_wait()

  def wait_initial_status(self) -> ConfigStatus:
    while True:
      deadline = time.monotonic() + STATUS_START_TIMEOUT_S
      with self.lock:
        start_frames = self.rx_frames
        start_by_bus = tuple(self.rx_frames_by_bus)
        while self.config is None and time.monotonic() < deadline:
          self.lock.wait(max(0.0, deadline - time.monotonic()))
        if self.config is not None:
          return self.config
        seen_frames = self.rx_frames - start_frames
        seen_by_bus = tuple(current - start for current, start in zip(self.rx_frames_by_bus, start_by_bus, strict=True))
      print("\nNo HRR configuration status received yet.")
      if seen_frames == 0:
        print("No CAN bus traffic was received during the startup window.")
        print("If the vehicle ignition is OFF, leave the calibration running and turn ignition ON now.")
      else:
        bus_text = ", ".join(f"bus {index}: {count}" for index, count in enumerate(seen_by_bus) if count)
        print(f"CAN traffic is present ({seen_frames} frames; {bus_text}), but HRR_ConfigStatus (0x639) was not seen.")
        print("Check that the HRR is powered and connected to the selected HRR bus.")
      print("The forced harness/intercept relay remains active while waiting.")
      response = input("Turn ignition ON/check connections, then press Enter to retry; type ABORT to stop: ").strip().upper()
      if response == "ABORT":
        raise RuntimeError("startup aborted while waiting for CAN/HRR traffic")

  def _send_config_and_wait(self, command: int, value: int, *, signed: bool, predicate) -> ConfigStatus:
    for _ in range(CONFIG_MAX_ATTEMPTS):
      with self.lock:
        before = None if self.config is None else self.config.counter
      payload = hrr.build_svec_config_frame(command, value, signed=signed)
      self.panda.can_send(hrr.SVEC_CONFIG_ADDR, payload, self.bus)
      deadline = time.monotonic() + CONFIG_ACK_TIMEOUT_S
      with self.lock:
        while time.monotonic() < deadline:
          status = self.config
          if (status is not None and status.counter != before and status.last_command == command and
              status.applied and predicate(status)):
            return status
          self.lock.wait(max(0.0, deadline - time.monotonic()))
    raise RuntimeError(f"HRR did not acknowledge config command {command} value {value}")

  def _set_dly_direct(self, dly: int) -> None:
    with self.lock:
      current = self.config
    if current is None or current.dly != dly:
      self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_DLY, dly, signed=False,
                                 predicate=lambda status: status.dly == dly)

  def _set_zero_direct(self, zero_offset: int) -> None:
    with self.lock:
      current = self.config
    if current is None or current.zero_offset != zero_offset:
      self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_ZERO_OFFSET, zero_offset, signed=True,
                                 predicate=lambda status: status.zero_offset == zero_offset)

  def _ramp_zero(self, zero_offset: int) -> None:
    with self.lock:
      current = self.config
      engaged = self.engaged
    if current is None:
      raise RuntimeError("cannot transition zero offset before HRR config status is available")
    if not engaged:
      self._set_zero_direct(zero_offset)
      return
    value = current.zero_offset
    while value != zero_offset:
      value += 1 if zero_offset > value else -1
      self._set_zero_direct(value)
      self._transition_wait()

  def transition_pair(self, dly: int, zero_offset: int) -> None:
    with self.lock:
      current = self.config
      engaged = self.engaged
    if current is None:
      raise RuntimeError("cannot transition calibration pair before HRR config status is available")
    if not engaged:
      self._set_dly_direct(dly)
      self._set_zero_direct(zero_offset)
      return

    start_dly = current.dly
    start_zero = current.zero_offset
    if start_dly == dly:
      self._ramp_zero(zero_offset)
      return

    direction = 1 if dly > start_dly else -1
    steps = abs(dly - start_dly)
    current_zero = start_zero
    for index in range(1, steps + 1):
      next_dly = start_dly + direction * index
      next_zero = round(start_zero + (zero_offset - start_zero) * index / steps)
      midpoint_zero = round((current_zero + next_zero) / 2.0)
      self._ramp_zero(midpoint_zero)
      self._set_dly_direct(next_dly)
      self._transition_wait()
      self._ramp_zero(next_zero)
      current_zero = next_zero
    self._ramp_zero(zero_offset)

  def apply_pair(self, dly: int, zero_offset: int) -> None:
    self.set_engaged(False)
    self._set_dly_direct(dly)
    self._set_zero_direct(zero_offset)

  def save_current(self) -> ConfigStatus:
    return self._send_config_and_wait(hrr.SVEC_CONFIG_CMD_SAVE, 0, signed=False,
                                      predicate=lambda status: status.save_succeeded and not status.dirty)

  def measure(self, phase: str, dly: int, zero_offset: int, command_torque: int, settle_s: float,
              sample_s: float, noise_weight: float) -> Measurement:
    self.safety_event.clear()
    self.transition_pair(dly, zero_offset)
    if not self.engaged:
      self.set_requested_torque(0)
      self.set_engaged(True)
    self.set_requested_torque(command_torque)

    settle_deadline = time.monotonic() + settle_s
    while time.monotonic() < settle_deadline:
      remaining = max(0.0, settle_deadline - time.monotonic())
      if self.safety_event.wait(min(0.05, remaining)):
        self._abort_if_unsafe()

    started = time.monotonic()
    deadline = started + sample_s
    while time.monotonic() < deadline:
      remaining = max(0.0, deadline - time.monotonic())
      if self.safety_event.wait(min(0.05, remaining)):
        self._abort_if_unsafe()
    ended = time.monotonic()

    with self.lock:
      samples = [sample for sample in self.samples if started <= sample.timestamp <= ended]
    minimum_samples = max(5, round(sample_s * 20.0))
    if len(samples) < minimum_samples:
      raise RuntimeError(f"only {len(samples)} torque samples in {sample_s:g}s; expected at least {minimum_samples}")
    measurement = summarize_samples(phase, dly, zero_offset, command_torque, samples, noise_weight)
    print(f"  {measurement.format()}")
    return measurement

  def shutdown(self) -> None:
    try:
      self.set_requested_torque(0)
    except RuntimeError:
      pass
    with self.lock:
      self.engaged = False
      self.requested_torque = 0
      self.brake_pressed = True
    time.sleep(max(0.06, 5.0 * self.period))
    self.stop_event.set()
    self.stream_thread.join(timeout=1.0)
    self.monitor_thread.join(timeout=1.0)


def measure_native_driver_torque(session: CalibrationSession, original: ConfigStatus, args, phase: str) -> Measurement:
  session.set_requested_torque(0)
  session.set_engaged(False)
  settle_deadline = time.monotonic() + args.settle_time
  while time.monotonic() < settle_deadline:
    time.sleep(min(0.05, max(0.0, settle_deadline - time.monotonic())))
  started = time.monotonic()
  deadline = started + NATIVE_BASELINE_TIME_S
  while time.monotonic() < deadline:
    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
  ended = time.monotonic()
  with session.lock:
    samples = [sample for sample in session.samples if started <= sample.timestamp <= ended]
  minimum_samples = max(5, round(NATIVE_BASELINE_TIME_S * 20.0))
  if len(samples) < minimum_samples:
    raise RuntimeError(f"only {len(samples)} native torque samples in {NATIVE_BASELINE_TIME_S:g}s; "
                       + f"expected at least {minimum_samples}")
  measurement = summarize_samples(phase, original.dly, original.zero_offset, 0, samples, args.noise_weight)
  print(f"  mean={measurement.mean:+7.2f} median={measurement.median:+7.2f} "
        + f"std={measurement.std:6.2f} robust_std={measurement.robust_std:6.2f} N={measurement.samples}")
  return measurement


def print_native_comparison(before: Measurement, after: Measurement, winner: CalibrationResult) -> None:
  native_std = statistics.mean((before.std, after.std))
  native_robust = statistics.mean((before.robust_std, after.robust_std))
  best = winner.zero
  print("\nNative Driver_Torque noise reference (HRR resolver relays OFF):")
  print(f"  before: std={before.std:.2f} robust_std={before.robust_std:.2f} Ncm")
  print(f"  after:  std={after.std:.2f} robust_std={after.robust_std:.2f} Ncm")
  print(f"  ref:    std={native_std:.2f} robust_std={native_robust:.2f} Ncm")
  print(f"  best HRR DLY={best.dly}: std={best.std:.2f} robust_std={best.robust_std:.2f} Ncm")
  if native_std > 0.0:
    print(f"  HRR/native std-noise ratio: {best.std / native_std:.2f}x")
  else:
    print("  HRR/native noise ratio: unavailable because the native reference measured zero spread.")
  drift = abs(after.mean - before.mean)
  drift_noise = math.hypot(after.std, before.std)
  if drift > max(5.0, drift_noise):
    print(f"  NOTE: native mean shifted by {after.mean - before.mean:+.2f} Ncm during calibration; "
          + "compare noise more strongly than absolute mean.")


def calibrate_zero(session: CalibrationSession, dly: int, seed: int, args,
                   phase: str) -> tuple[Measurement, list[Measurement]]:
  low = -args.zero_limit
  high = args.zero_limit
  seed = max(low, min(high, seed))
  measurements: dict[int, Measurement] = {}

  def evaluate(offset: int) -> Measurement:
    offset = max(low, min(high, offset))
    if offset not in measurements:
      measurements[offset] = session.measure(phase, dly, offset, 0, args.settle_time,
                                             args.sample_time, args.noise_weight)
    return measurements[offset]

  for offset in dict.fromkeys((seed, seed - args.zero_probe, seed + args.zero_probe)):
    evaluate(offset)
  previous_count = -1
  for _ in range(args.zero_iterations):
    best_bias = min(measurements.values(), key=lambda point: (abs(point.mean), point.std, point.robust_std))
    if abs(best_bias.mean) <= args.zero_mean_tolerance:
      break
    root = fit_zero_root(list(measurements.values()), low, high)
    for offset in (root, root - 1, root + 1):
      evaluate(offset)
    if len(measurements) == previous_count:
      nearest = sorted(measurements.values(), key=lambda point: abs(point.mean))[:2]
      if len(nearest) == 2 and nearest[0].zero_offset != nearest[1].zero_offset:
        dx = nearest[1].zero_offset - nearest[0].zero_offset
        dy = nearest[1].mean - nearest[0].mean
        if abs(dy) >= 0.2:
          estimate = round(nearest[0].zero_offset - nearest[0].mean * dx / dy)
          evaluate(estimate)
          evaluate(estimate - 1)
          evaluate(estimate + 1)
    previous_count = len(measurements)
  best = min(measurements.values(), key=lambda point: (point.score, abs(point.mean), point.std, point.robust_std))
  if abs(best.mean) > args.zero_mean_tolerance:
    print(f"  NOTE: zero fit residual {best.mean:+.2f} Ncm exceeds "
          + f"{args.zero_mean_tolerance:.1f} Ncm tolerance after {args.zero_iterations} iterations")
  session.transition_pair(dly, best.zero_offset)
  return best, list(measurements.values())


def fine_refit_zero(session: CalibrationSession, finalist: Measurement, args) -> tuple[Measurement, list[Measurement]]:
  low = -args.zero_limit
  high = args.zero_limit
  offsets = tuple(dict.fromkeys(max(low, min(high, offset))
                                for offset in (finalist.zero_offset, finalist.zero_offset - 1, finalist.zero_offset + 1)))
  measurements = []
  print(f"\nFine long-window zero re-fit for DLY={finalist.dly} around zero={finalist.zero_offset:+d}:")
  for offset in offsets:
    measurements.append(session.measure("validation_refit", finalist.dly, offset, 0,
                                        args.settle_time, args.validation_time, args.noise_weight))
  best = min(measurements, key=lambda point: (abs(point.mean), point.std, point.robust_std))
  session.transition_pair(best.dly, best.zero_offset)
  print(f"  refined finalist: {best.format()}")
  return best, measurements


def validate_loaded(session: CalibrationSession, zero: Measurement, args) -> tuple[LoadedValidation, list[Measurement]]:
  torque = args.validation_torque
  assert torque is not None and torque > 0
  sequence = (0, torque, 0, -torque, 0, -torque, 0, torque, 0)
  measurements = []
  print(f"\nCounterbalanced loaded validation for DLY={zero.dly}, zero={zero.zero_offset:+d}, T={torque} Ncm:")
  print("  Each loaded response is referenced to the average of its neighboring zero-command measurements.")
  for command_torque in sequence:
    phase = "loaded_zero" if command_torque == 0 else ("loaded_positive" if command_torque > 0 else "loaded_negative")
    measurements.append(session.measure(phase, zero.dly, zero.zero_offset, command_torque,
                                        args.settle_time, args.load_sample_time, args.noise_weight))
  loaded = LoadedValidation(zero, tuple(measurements), torque, args.noise_weight)
  print(f"  local response: +T={loaded.positive_mean:+.2f} -T={loaded.negative_mean:+.2f} "
        + f"gain={loaded.response_gain:+.4f} driver/cmd even_error={loaded.even_error:+.2f} "
        + f"asymmetry={loaded.asymmetry_percent:.1f}% repeat={loaded.repeatability:.2f}")
  print(f"  zero baseline: initial_shift={loaded.initial_zero_shift:+.2f} Ncm "
        + f"return_rms={loaded.return_rms:.2f} span={loaded.zero_span:.2f} Ncm; "
        + f"loaded_noise(std)={loaded.loaded_noise:.2f}")
  return loaded, measurements


def print_profile_summary(best_by_dly: list[Measurement], args) -> None:
  print("\nProfile summary (each DLY with its independently fitted zero):")
  print(" DLY zero |    mean  median    std robust |  score  DLYscore")
  print(" --- ---- | ------- ------- ------ ------ | ------ --------")
  for point in sorted(best_by_dly, key=lambda item: item.dly):
    dly_score = profile_finalist_score(point, args.zero_mean_tolerance, args.noise_weight)
    print(f" {point.dly:3d} {point.zero_offset:+4d} | {point.mean:+7.2f} {point.median:+7.2f} "
          + f"{point.std:6.2f} {point.robust_std:6.2f} | {point.score:6.2f} {dly_score:8.2f}")


def print_ranking(results: list[CalibrationResult], original: ConfigStatus,
                  dly_low: int, dly_high: int, zero_limit: int) -> None:
  response_reference = loaded_response_reference(results)
  ranked = sorted(results, key=lambda result: result.selection_score(response_reference))
  if ranked[0].loaded is None:
    print("\nFinal zero-load ranking (lower score is better):")
    for rank, result in enumerate(ranked, 1):
      print(f" {rank:2d}. {result.zero.format()}")
    print("  NOTE: loaded validation was disabled; recommendation is based only on stationary zero-command torque.")
    return
  print("\nFinalist ranking (lower score is better):")
  print(" rk DLY zero | zero_mean zero_sd |    +R     -R   gain | even  asym% zRMS zSpan repeat noise rdev | score")
  print(" -- --- ---- | --------- ------- | ------ ------ ------ | ----- ------ ---- ----- ------ ----- ---- | -----")
  for rank, result in enumerate(ranked, 1):
    loaded = result.loaded
    assert loaded is not None
    print(f" {rank:2d} {result.zero.dly:3d} {result.zero.zero_offset:+4d} | "
          + f"{result.zero.mean:+9.2f} {result.zero.std:7.2f} | "
          + f"{loaded.positive_mean:+6.1f} {loaded.negative_mean:+6.1f} {loaded.response_gain:+6.3f} | "
          + f"{loaded.even_error:+5.1f} {loaded.asymmetry_percent:6.1f} {loaded.return_rms:4.1f} "
          + f"{loaded.zero_span:5.1f} {loaded.repeatability:6.1f} {loaded.loaded_noise:5.1f} "
          + f"{abs(abs(loaded.odd_response) - response_reference):4.1f} | "
          + f"{result.selection_score(response_reference):5.1f}")
  winner = ranked[0]
  loaded = winner.loaded
  assert loaded is not None
  print("\nRecommendation diagnostics:")
  print(f"  Selected DLY={winner.zero.dly}, SVEC_ZERO_OFFSET={winner.zero.zero_offset:+d} "
        + f"({winner.zero.zero_offset / 10:+.1f}deg).")
  print(f"  Refined zero: {winner.zero.mean:+.2f} Ncm mean, std={winner.zero.std:.2f} Ncm, "
        + f"robust_std={winner.zero.robust_std:.2f} Ncm.")
  print(f"  Local loaded response: {loaded.odd_response:+.2f} Ncm at +/-{loaded.validation_torque} Ncm command, "
        + f"gain {loaded.response_gain:+.4f}.")
  print(f"  Response consistency: finalist median={response_reference:.2f} Ncm, "
        + f"deviation={abs(abs(loaded.odd_response) - response_reference):.2f} Ncm.")
  print(f"  Local symmetry: {loaded.even_error:+.2f} Ncm midpoint error, "
        + f"{loaded.asymmetry_percent:.1f}% magnitude asymmetry.")
  print(f"  Baseline stability: first loaded zero shift={loaded.initial_zero_shift:+.2f} Ncm, "
        + f"zero RMS={loaded.return_rms:.2f} Ncm, span={loaded.zero_span:.2f} Ncm.")
  print(f"  Repeatability/noise: {loaded.repeatability:.2f} Ncm local-response range, "
        + f"{loaded.loaded_noise:.2f} Ncm loaded std noise.")
  cautions = loaded.cautions()
  if response_reference >= 10.0 and abs(loaded.odd_response) < response_reference * 0.5:
    cautions.append("loaded response is less than half the finalist median")
  if winner.zero.dly in {dly_low, dly_high} and dly_low != dly_high:
    cautions.append("best DLY is at the sweep boundary; repeat with a wider DLY offset")
  if abs(winner.zero.zero_offset) == zero_limit:
    cautions.append("best zero offset is at the configured search boundary")
  print("  CAUTION: " + "; ".join(cautions) + "." if cautions else
        "  PASS: centered, locally symmetric, repeatable, and baseline-stable after both torque directions.")
  if winner.zero.dly != original.dly:
    print(f"  DLY moves {winner.zero.dly - original.dly:+d} samples from the original value {original.dly}.")


def walk_profile_path(session: CalibrationSession, target_dly: int,
                      profile_by_dly: dict[int, Measurement]) -> None:
  with session.lock:
    current = session.config
  if current is None:
    raise RuntimeError("cannot walk DLY path before HRR config status is available")
  while current.dly != target_dly:
    next_dly = current.dly + (1 if target_dly > current.dly else -1)
    next_point = profile_by_dly[next_dly]
    session.transition_pair(next_dly, next_point.zero_offset)
    with session.lock:
      current = session.config
    if current is None:
      raise RuntimeError("lost HRR config status while walking DLY path")


def run_calibration(session: CalibrationSession, original: ConfigStatus, dly_start: int,
                    dly_offset: int, args) -> tuple[CalibrationResult, list[Measurement]]:
  print(f"\nNative Driver_Torque baseline before sweep ({NATIVE_BASELINE_TIME_S:g}s, HRR resolver relays OFF):")
  native_before = measure_native_driver_torque(session, original, args, "native_before")
  dly_low = max(0, dly_start - dly_offset)
  dly_high = min(hrr.MAX_DLY_SAMPLES, dly_start + dly_offset)
  center_group, low_group, high_group = dly_sweep_groups(dly_start, dly_low, dly_high)
  dly_values = center_group + low_group + high_group
  print(f"\nProfiling DLY {dly_low}..{dly_high} around start {dly_start}; "
        + "each DLY gets its own converged zero offset.")
  print("HRR resolver relays stay ON during the sweep; DLY moves one sample at a time, ZERO and torque are ramped,")
  print("and every recorded point gets the full settle time after its final target values are reached.")
  all_measurements: list[Measurement] = []
  best_by_dly: list[Measurement] = []
  print(f"\n[1/{len(dly_values)}] DLY={dly_start} (center)")
  center_best, measurements = calibrate_zero(session, dly_start, original.zero_offset, args, "profile")
  all_measurements.extend(measurements)
  best_by_dly.append(center_best)
  print(f"  best for DLY {dly_start}: {center_best.format()}")
  index = 2
  low_seed = center_best.zero_offset
  for dly in low_group:
    print(f"\n[{index}/{len(dly_values)}] DLY={dly} (low side, seed={low_seed:+d})")
    best, measurements = calibrate_zero(session, dly, low_seed, args, "profile")
    all_measurements.extend(measurements)
    best_by_dly.append(best)
    low_seed = best.zero_offset
    print(f"  best for DLY {dly}: {best.format()}")
    index += 1

  profile_by_dly = {point.dly: point for point in best_by_dly}
  if low_group and high_group:
    print(f"\nReturning continuously from DLY={dly_low} to DLY={dly_start} before the high-side sweep (no measurements):")
    walk_profile_path(session, dly_start, profile_by_dly)
    print(f"  returned to DLY={dly_start}, zero={center_best.zero_offset:+d}")

  high_seed = center_best.zero_offset
  for dly in high_group:
    print(f"\n[{index}/{len(dly_values)}] DLY={dly} (high side, seed={high_seed:+d})")
    best, measurements = calibrate_zero(session, dly, high_seed, args, "profile")
    all_measurements.extend(measurements)
    best_by_dly.append(best)
    high_seed = best.zero_offset
    profile_by_dly[dly] = best
    print(f"  best for DLY {dly}: {best.format()}")
    index += 1
  print_profile_summary(best_by_dly, args)

  finalists = sorted(best_by_dly,
                     key=lambda point: (profile_finalist_score(point, args.zero_mean_tolerance, args.noise_weight),
                                        point.std, abs(point.mean), point.robust_std,
                                        abs(point.dly - dly_start)))[:min(args.finalists, len(best_by_dly))]
  print("\nLong-window +/-1 ZERO re-fit of the best profiled DLY values:")
  for finalist in finalists:
    dly_score = profile_finalist_score(finalist, args.zero_mean_tolerance, args.noise_weight)
    print(f"  finalist DLY={finalist.dly}: trimmed std={finalist.std:.2f}, mean={finalist.mean:+.2f}, "
          + f"DLYscore={dly_score:.2f}, robust_std={finalist.robust_std:.2f}")
  validated: list[Measurement] = []
  for finalist in finalists:
    walk_profile_path(session, finalist.dly, profile_by_dly)
    session.transition_pair(finalist.dly, finalist.zero_offset)
    refined, measurements = fine_refit_zero(session, finalist, args)
    all_measurements.extend(measurements)
    validated.append(refined)
    profile_by_dly[refined.dly] = refined

  results = [CalibrationResult(point, None) for point in validated]
  if args.validation_torque > 0:
    results = []
    for point in validated:
      walk_profile_path(session, point.dly, profile_by_dly)
      session.transition_pair(point.dly, point.zero_offset)
      loaded, measurements = validate_loaded(session, point, args)
      all_measurements.extend(measurements)
      results.append(CalibrationResult(point, loaded))
  response_reference = loaded_response_reference(results)
  results.sort(key=lambda result: (result.selection_score(response_reference), abs(result.zero.mean),
                                    result.zero.std, abs(result.zero.dly - dly_start)))
  print_ranking(results, original, dly_low, dly_high, args.zero_limit)
  winner = results[0]

  session.set_requested_torque(0)
  print(f"\nNative Driver_Torque baseline after sweep ({NATIVE_BASELINE_TIME_S:g}s, HRR resolver relays OFF):")
  native_after = measure_native_driver_torque(session, original, args, "native_after")
  print_native_comparison(native_before, native_after, winner)
  return winner, [native_before, *all_measurements, native_after]


def confirm_safety(args) -> None:
  print("\nWARNING: this calibration engages the HRR resolver relays for the sweep and applies low symmetric torque.")
  print("DLY is walked one sample at a time, ZERO/torque changes are ramped, and measurements occur only after settling.")
  print("Secure the stationary vehicle, keep hands and people clear of the steering wheel and linkage,")
  print("choose a load that cannot rotate the wheel, and be ready to remove power. The script aborts if")
  print("measured Driver_Torque exceeds the configured limit.")
  if not args.yes and input("Type CALIBRATE to continue: ").strip() != "CALIBRATE":
    raise RuntimeError("calibration not confirmed")


def run_self_test() -> None:
  center, low, high = dly_sweep_groups(45, 41, 49)
  assert center == [45] and low == [44, 43, 42, 41] and high == [46, 47, 48, 49]
  synthetic = [Measurement("test", 20, zero, 0, 100, 4.0 * zero + 12.0, 0.0, 1.0, 1.0, 50, 1.0)
               for zero in (-5, 0, 5)]
  assert fit_zero_root(synthetic, -135, 135) == -3
  synthetic += [Measurement("test", 20, zero, 0, 100, mean, 0.0, 1.0, 1.0, 50, 1.0)
                for zero, mean in ((-27, -3.0), (-26, 0.5), (-25, 4.0))]
  assert -27 <= fit_zero_root(synthetic, -135, 135) <= -25
  samples = [TorqueSample(float(index), value, -value) for index, value in enumerate([8, 9, 10, 10, 11, 12, 100])]
  summary = summarize_samples("test", 20, -3, 0, samples, 1.0)
  assert summary.samples == 7 and summary.median == 10 and summary.maximum_abs == 100
  assert math.isclose(summary.score, math.hypot(summary.mean, summary.std))

  quiet = Measurement("test", 4, 2, 0, 100, 2.0, 2.0, 2.0, 2.97, 50, math.hypot(2.0, 2.0))
  noisy = Measurement("test", 8, 26, 0, 100, 0.5, 0.5, 2.5, 1.48, 50, math.hypot(0.5, 2.5))
  assert profile_finalist_score(quiet, 3.0, 1.0) < profile_finalist_score(noisy, 3.0, 1.0)

  def point(command: int, mean: float) -> Measurement:
    return Measurement("load", 20, -3, command, 100, mean, mean, 2.0, 2.0, 50, 2.0)

  sequence = (point(0, 1.0), point(100, 52.0), point(0, 2.0), point(-100, -47.0), point(0, 3.0),
              point(-100, -46.0), point(0, 4.0), point(100, 55.0), point(0, 5.0))
  loaded = LoadedValidation(point(0, 1.0), sequence, 100, 1.0)
  assert loaded.positive_mean == 50.0 and loaded.negative_mean == -50.0
  assert loaded.odd_response == 50.0 and loaded.response_gain == 0.5
  assert loaded.even_error == 0.0 and loaded.asymmetry_percent == 0.0
  assert loaded.repeatability == 0.0 and loaded.initial_zero_shift == 0.0
  assert loaded.loaded_noise == 2.0
  status = ConfigStatus.decode(struct.pack("<BhhBBB", 27, -3, 40, 0x06, 9, 12))
  assert status.dly == 27 and status.zero_offset == -3 and status.applied and status.save_succeeded
  print("HRR torque-calibration self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), default=2, help="HRR command/status Panda bus")
  parser.add_argument("--torque-bus", type=int, choices=(0, 1, 2), default=0,
                      help="Panda bus carrying Toyota STEER_TORQUE_SENSOR 0x260")
  parser.add_argument("--dly-start", type=int, help="DLY sweep center; prompted when omitted, blank uses the active DLY")
  parser.add_argument("--dly-offset", type=int, help="maximum samples below/above the DLY sweep center")
  parser.add_argument("--zero-limit", type=int, default=hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG,
                      help="absolute SVEC_ZERO_OFFSET search limit in tenths of a degree")
  parser.add_argument("--zero-probe", type=int, default=5,
                      help="offset distance used to estimate the local zero-torque slope")
  parser.add_argument("--zero-iterations", type=int, default=4,
                      help="maximum iterative root-refinement passes per DLY")
  parser.add_argument("--zero-mean-tolerance", type=float, default=3.0,
                      help="stop zero refinement when absolute mean Driver_Torque is within this Ncm")
  parser.add_argument("--settle-time", type=float, default=0.75,
                      help="settling time after the final DLY/ZERO/torque target is reached before every measurement")
  parser.add_argument("--sample-time", type=float, default=1.5, help="torque sampling time for each profile point")
  parser.add_argument("--validation-time", type=float, default=5.0,
                      help="long sampling time for each +/-1 ZERO finalist re-fit point")
  parser.add_argument("--validation-torque", type=int,
                      help="symmetric finalist torque in Ncm; prompted when omitted, 0 disables")
  parser.add_argument("--load-sample-time", type=float, default=1.5,
                      help="sampling time for each point in the counterbalanced loaded test")
  parser.add_argument("--transition-step-time", type=float, default=0.02,
                      help="pause between individual DLY/ZERO/torque transition steps")
  parser.add_argument("--torque-ramp-step", type=int, default=10,
                      help="maximum requested-torque change per transition step in Ncm")
  parser.add_argument("--finalists", type=int, default=3, help="number of best DLY values to revalidate")
  parser.add_argument("--noise-weight", type=float, default=1.0,
                      help="trimmed standard-deviation weight in zero/DLY/load scoring")
  parser.add_argument("--max-driver-torque", type=int, default=600,
                      help="abort threshold for absolute Driver_Torque in Ncm")
  parser.add_argument("--rate-hz", type=float, default=hrr.DEFAULT_RATE_HZ, help="brake/torque stream rate")
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
  for name in ("settle_time", "sample_time", "validation_time", "load_sample_time", "noise_weight", "rate_hz",
               "zero_mean_tolerance", "transition_step_time"):
    if getattr(args, name) <= 0:
      parser.error(f"--{name.replace('_', '-')} must be greater than zero")
  if (args.zero_probe <= 0 or args.zero_limit <= 0 or args.zero_iterations <= 0 or
      args.finalists <= 0 or args.max_driver_torque <= 0 or args.torque_ramp_step <= 0):
    parser.error("zero probe/limit/iterations, finalists, torque limit, and torque ramp step must be greater than zero")
  if args.zero_limit > hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG:
    parser.error(f"--zero-limit cannot exceed {hrr.SVEC_ZERO_OFFSET_MAX_TENTHS_DEG}")
  if args.dly_start is not None and not 0 <= args.dly_start <= hrr.MAX_DLY_SAMPLES:
    parser.error(f"--dly-start must be within 0..{hrr.MAX_DLY_SAMPLES}")
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
  session = CalibrationSession(panda, args.bus, args.torque_bus, args.rate_hz, args.max_driver_torque,
                               args.transition_step_time, args.torque_ramp_step)
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
      print("Harness/intercept relay forced before waiting for vehicle/HRR CAN traffic.")
    session.start()
    session_started = True
    original = session.wait_initial_status()
    print(f"Current active settings: DLY={original.dly}, zero={original.zero_offset:+d} "
          + f"({original.zero_offset / 10:+.1f}deg), angle_offset={original.angle_offset:+d}, dirty={original.dirty}")
    if original.dirty and not args.allow_dirty_start:
      raise RuntimeError("HRR reports unsaved active settings; save/reboot them or pass --allow-dirty-start")
    if args.dly_start is None:
      entered = input(f"DLY sweep center [{original.dly}]: ").strip()
      args.dly_start = original.dly if not entered else int(entered, 0)
    if not 0 <= args.dly_start <= hrr.MAX_DLY_SAMPLES:
      raise RuntimeError(f"DLY sweep center must be within 0..{hrr.MAX_DLY_SAMPLES}")
    if args.dly_offset is None:
      entered = input("Maximum DLY sweep offset in samples [5]: ").strip()
      args.dly_offset = 5 if not entered else int(entered, 0)
    if args.dly_offset < 0:
      raise RuntimeError("DLY sweep offset must be non-negative")
    if args.validation_torque is None:
      entered = input("Symmetric finalist validation torque in Ncm [100, 0 disables]: ").strip()
      args.validation_torque = 100 if not entered else int(entered, 0)
    if not 0 <= args.validation_torque <= hrr.MAX_TORQUE_NCM:
      raise RuntimeError(f"validation torque must be within 0..{hrr.MAX_TORQUE_NCM} Ncm")
    if args.validation_torque == 0:
      print("NOTE: loaded validation disabled; final recommendation will be zero-load only.")

    winner, _ = run_calibration(session, original, args.dly_start, args.dly_offset, args)
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
