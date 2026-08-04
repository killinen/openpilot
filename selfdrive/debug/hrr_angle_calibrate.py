#!/usr/bin/env python3
"""Guided HRR resolver calibration with full-sweep OU shape fitting."""

from __future__ import annotations

import math
import select
import sys
import time

import hrr_angle_calibrate_core as core


# OU electrical shape is learned from the whole sweep. Only the relative
# IN<->OU zero needs an unloaded steering sample, so the operator can simply
# finish near center and let the wheel relax briefly.
OU_ZERO_CENTER_MAX_DEG = 20.0
OU_ZERO_RATE_MAX_DEG_S = 6.0
OU_ZERO_MIN_SAMPLES = 8
OU_ZERO_MIN_DURATION_S = 0.40
OU_ZERO_SEGMENT_MAX_GAP_S = 0.25
OU_SHAPE_MAX_RMS = 0.05
OU_SHAPE_MAX_ERROR = 0.15

_FIT_DETAILS: dict[int, tuple[int, float, float]] = {}


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float]:
  augmented = [row[:] + [value] for row, value in zip(matrix, vector, strict=True)]
  for column in range(3):
    pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
    if abs(augmented[pivot][column]) < 1e-12:
      raise ValueError("poorly conditioned OU ellipse fit")
    augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
    scale = augmented[column][column]
    for item in range(column, 4):
      augmented[column][item] /= scale
    for row in range(3):
      if row == column:
        continue
      factor = augmented[row][column]
      for item in range(column, 4):
        augmented[row][item] -= factor * augmented[column][item]
  return augmented[0][3], augmented[1][3], augmented[2][3]


def _matrix_apply(matrix: tuple[float, float, float, float], x: float, y: float) -> tuple[float, float]:
  return matrix[0] * x + matrix[1] * y, matrix[2] * x + matrix[3] * y


def _matrix_multiply(left: tuple[float, float, float, float],
                     right: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
  return (
    left[0] * right[0] + left[1] * right[2],
    left[0] * right[1] + left[1] * right[3],
    left[2] * right[0] + left[3] * right[2],
    left[2] * right[1] + left[3] * right[3],
  )


def _rotation_matrix(angle_deg: float) -> tuple[float, float, float, float]:
  angle = math.radians(angle_deg)
  cosine, sine = math.cos(angle), math.sin(angle)
  return cosine, -sine, sine, cosine


def _circular_center_mod180(values: list[float]) -> float:
  if not values:
    raise ValueError("no phase values")
  cosine = sum(math.cos(math.radians(2.0 * value)) for value in values)
  sine = sum(math.sin(math.radians(2.0 * value)) for value in values)
  if math.hypot(cosine, sine) < 1e-9:
    return float(core.median(values))
  return 0.5 * math.degrees(math.atan2(sine, cosine))


def _robust_rms(values: list[float]) -> float:
  if not values:
    return math.inf
  center = core.median(values)
  deviations = sorted(abs(value - center) for value in values)
  mad = deviations[len(deviations) // 2]
  limit = max(1.0, 4.5 * 1.4826 * mad)
  retained = [value for value in values if abs(value - center) <= limit]
  return math.sqrt(sum(value * value for value in retained) / len(retained))


def _fit_ellipse_shape(samples: list[core.CalibrationSample], indices: list[int], pair: str,
                       min_samples: int = core.MIN_SAMPLES) -> tuple[tuple[float, float, float, float], list[int], float, float]:
  """Whiten a zero-centered resolver ellipse without using steering angle."""
  current = [
    index for index in indices
    if math.hypot(samples[index].in_cos if pair == "in" else samples[index].ou_cos,
                  samples[index].in_sin if pair == "in" else samples[index].ou_sin) >= core.VECTOR_MIN_MAGNITUDE
  ]
  if len(current) < min_samples:
    raise ValueError(f"need at least {min_samples} valid {pair.upper()} vectors")

  matrix = (1.0, 0.0, 0.0, 1.0)
  errors: list[tuple[int, float]] = []
  for _ in range(4):
    normal = [[0.0] * 3 for _ in range(3)]
    rhs = [0.0] * 3
    for index in current:
      sample = samples[index]
      raw_x = float(sample.in_cos if pair == "in" else sample.ou_cos) / core.VECTOR_MATRIX_INPUT_SCALE
      raw_y = float(sample.in_sin if pair == "in" else sample.ou_sin) / core.VECTOR_MATRIX_INPUT_SCALE
      row = (raw_x * raw_x, 2.0 * raw_x * raw_y, raw_y * raw_y)
      for first in range(3):
        rhs[first] += row[first]
        for second in range(3):
          normal[first][second] += row[first] * row[second]

    q00, q01, q11 = _solve_3x3(normal, rhs)
    determinant = q00 * q11 - q01 * q01
    if q00 <= 0.0 or q11 <= 0.0 or determinant <= 1e-5:
      raise ValueError(f"invalid {pair.upper()} ellipse fit")

    # Q = L L^T. W=L^T therefore satisfies |W*x|^2 = x^T Q x.
    l00 = math.sqrt(q00)
    l10 = q01 / l00
    l11_squared = q11 - l10 * l10
    if l11_squared <= 1e-8:
      raise ValueError(f"singular {pair.upper()} ellipse fit")
    l11 = math.sqrt(l11_squared)
    matrix = (l00, l10, 0.0, l11)
    if abs(matrix[0] * matrix[3] - matrix[1] * matrix[2]) < 0.05 or max(map(abs, matrix)) > 4.0:
      raise ValueError(f"invalid {pair.upper()} shape correction matrix")

    errors = []
    for index in current:
      sample = samples[index]
      raw_x = float(sample.in_cos if pair == "in" else sample.ou_cos) / core.VECTOR_MATRIX_INPUT_SCALE
      raw_y = float(sample.in_sin if pair == "in" else sample.ou_sin) / core.VECTOR_MATRIX_INPUT_SCALE
      out_x, out_y = _matrix_apply(matrix, raw_x, raw_y)
      errors.append((index, abs(math.hypot(out_x, out_y) - 1.0)))

    ordered = sorted(error for _, error in errors)
    center = ordered[len(ordered) // 2]
    deviations = sorted(abs(error - center) for error in ordered)
    mad = deviations[len(deviations) // 2]
    limit = max(0.02, center + 4.5 * 1.4826 * mad)
    filtered = [index for index, error in errors if error <= limit]
    if len(filtered) == len(current) or len(filtered) < min_samples:
      break
    current = filtered

  current_set = set(current)
  retained_errors = [error for index, error in errors if index in current_set]
  shape_rms = math.sqrt(sum(error * error for error in retained_errors) / len(retained_errors))
  shape_max = max(retained_errors)
  if shape_rms > OU_SHAPE_MAX_RMS or shape_max > OU_SHAPE_MAX_ERROR:
    raise ValueError(
      f"OU electrical shape fit poor: rms={shape_rms:.3f}/{OU_SHAPE_MAX_RMS:.3f} "
      + f"max={shape_max:.3f}/{OU_SHAPE_MAX_ERROR:.3f}"
    )
  return matrix, current, shape_rms, shape_max


def _phase_for(sample: core.CalibrationSample, matrix: tuple[float, float, float, float], pair: str) -> float:
  raw_x = float(sample.in_cos if pair == "in" else sample.ou_cos)
  raw_y = float(sample.in_sin if pair == "in" else sample.ou_sin)
  out_x, out_y = _matrix_apply(matrix, raw_x, raw_y)
  return float(core.vector_phase_deg(out_x, out_y))


def _orient_ou_shape(samples: list[core.CalibrationSample], indices: list[int], ratio: float,
                     in_matrix: tuple[float, float, float, float],
                     shape: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
  """Resolve possible OU handedness without treating torsion as ellipse error."""
  del ratio
  candidates = (
    shape,
    _matrix_multiply((1.0, 0.0, 0.0, -1.0), shape),
  )
  best_matrix = candidates[0]
  best_score = math.inf
  for candidate in candidates:
    offsets = []
    for index in indices:
      sample = samples[index]
      measured = _phase_for(sample, candidate, "ou")
      in_phase = _phase_for(sample, in_matrix, "in")
      offsets.append(core.shortest_mod180_delta(measured, in_phase))
    center = _circular_center_mod180(offsets)
    residuals = [core.shortest_mod180_delta(offset, center) for offset in offsets]
    score = _robust_rms(residuals)
    if score < best_score:
      best_matrix = candidate
      best_score = score
  return best_matrix


def _find_latest_zero_segment(samples: list[core.CalibrationSample], indices: list[int], ratio: float) -> list[int]:
  valid = set(indices)
  candidates: list[int] = []
  for index in indices:
    sample = samples[index]
    if abs(sample.reference_deg) > OU_ZERO_CENTER_MAX_DEG or index == 0 or index - 1 not in valid:
      continue
    previous = samples[index - 1]
    elapsed = sample.timestamp - previous.timestamp
    if elapsed <= 0.0:
      continue
    steering_rate = abs((sample.unwrapped_phase_deg - previous.unwrapped_phase_deg) / ratio / elapsed)
    if steering_rate <= OU_ZERO_RATE_MAX_DEG_S:
      candidates.append(index)

  segments: list[list[int]] = []
  for index in candidates:
    if (not segments or
        samples[index].timestamp - samples[segments[-1][-1]].timestamp > OU_ZERO_SEGMENT_MAX_GAP_S):
      segments.append([index])
    else:
      segments[-1].append(index)

  qualified = [
    segment for segment in segments
    if len(segment) >= OU_ZERO_MIN_SAMPLES and
       samples[segment[-1]].timestamp - samples[segment[0]].timestamp >= OU_ZERO_MIN_DURATION_S
  ]
  if not qualified:
    raise ValueError(
      f"OU zero not captured: finish within +/-{OU_ZERO_CENTER_MAX_DEG:.0f}deg of center and "
      + "let the wheel relax for about 1 second"
    )
  return qualified[-1]


def fit_calibration_aligned(samples: list[core.CalibrationSample], reference_delay_s: float = 0.0) -> core.CalibrationFit:
  if len(samples) < core.MIN_SAMPLES:
    raise ValueError(f"need at least {core.MIN_SAMPLES} samples")

  # IN is the reference side: retain the supervised robust fit against 0x025.
  ratio, in_indices = core.robust_phase_ratio(samples)
  in_matrix, in_indices = core.solve_matrix(samples, in_indices, ratio, "in")

  # OU shape is independent of steering torque. Fit the ellipse from the whole
  # robust sweep, then determine only its handedness and constant phase zero.
  ou_shape, ou_indices, shape_rms, shape_max = _fit_ellipse_shape(samples, in_indices, "ou")
  ou_shape = _orient_ou_shape(samples, ou_indices, ratio, in_matrix, ou_shape)

  # The last relaxed, near-center stationary segment supplies the mechanical
  # zero. This is the only part of the calibration that needs low torsion.
  zero_indices = _find_latest_zero_segment(samples, ou_indices, ratio)
  rotations = []
  for index in zero_indices:
    sample = samples[index]
    measured = _phase_for(sample, ou_shape, "ou")
    in_phase = _phase_for(sample, in_matrix, "in")
    rotations.append(core.shortest_mod180_delta(in_phase, measured))
  zero_rotation = _circular_center_mod180(rotations)
  ou_matrix = _matrix_multiply(_rotation_matrix(zero_rotation), ou_shape)

  in_errors = []
  references = []
  phases = []
  for index in in_indices:
    sample = samples[index]
    target = (ratio * sample.reference_deg) % 180.0
    in_phase = _phase_for(sample, in_matrix, "in")
    in_errors.append(abs(core.shortest_mod180_delta(in_phase, target)) / abs(ratio))
    references.append(sample.reference_deg)
    phases.append(sample.unwrapped_phase_deg)

  # OU absolute error is evaluated only during the explicitly unloaded anchor;
  # electrical shape quality over the complete sweep was checked independently
  # above, so real torsion during the sweep is not misclassified as error.
  ou_errors = []
  zero_torsions = []
  for index in zero_indices:
    sample = samples[index]
    target = (ratio * sample.reference_deg) % 180.0
    in_phase = _phase_for(sample, in_matrix, "in")
    ou_phase = _phase_for(sample, ou_matrix, "ou")
    ou_errors.append(abs(core.shortest_mod180_delta(ou_phase, target)) / abs(ratio))
    zero_torsions.append(core.shortest_mod180_delta(ou_phase, in_phase) / abs(ratio))

  ou_bins = {
    int(_phase_for(samples[index], ou_matrix, "ou") // 10.0) % 18
    for index in ou_indices
  }
  positive = negative = 0.0
  for previous, current in zip(references, references[1:], strict=False):
    delta = current - previous
    if delta > 0.0:
      positive += delta
    else:
      negative -= delta

  torsion_center = core.median(zero_torsions)
  torsion_deviations = [abs(value - torsion_center) for value in zero_torsions]
  fit = core.CalibrationFit(
    in_matrix + ou_matrix, ratio, min(len(in_indices), len(ou_indices)), len(samples),
    math.sqrt(sum(error * error for error in in_errors) / len(in_errors)), max(in_errors),
    math.sqrt(sum(error * error for error in ou_errors) / len(ou_errors)), max(ou_errors),
    math.sqrt(sum(error * error for error in torsion_deviations) / len(torsion_deviations)),
    max(torsion_deviations), max(references) - min(references), max(phases) - min(phases),
    positive, negative, len(ou_bins), reference_delay_s,
  )
  _FIT_DETAILS[id(fit)] = (len(zero_indices), shape_rms, shape_max)
  return fit


def _format_fit(self: core.CalibrationFit) -> str:
  zero_samples, shape_rms, shape_max = _FIT_DETAILS.get(id(self), (0, math.nan, math.nan))
  return (f"n={self.samples}/{self.total_samples} " +
          f"ref_span={self.reference_span_deg:.0f}/{core.MIN_REFERENCE_SPAN_DEG:.0f}deg " +
          f"phase_span={self.phase_span_deg:.0f}/{core.MIN_PHASE_SPAN_DEG:.0f}deg " +
          f"travel=+{self.positive_travel_deg:.0f}/-{self.negative_travel_deg:.0f}deg " +
          f"OUbins={self.occupied_bins}/18 zero_n={zero_samples} " +
          f"OUshape={shape_rms:.3f}/{shape_max:.3f} ratio={self.phase_per_steer:+.6f} " +
          f"lag={self.reference_delay_s:+.2f}s " +
          f"IN={self.rms_error_deg:.2f}/{self.max_error_deg:.2f}deg " +
          f"OUzero={self.ou_rms_error_deg:.2f}/{self.ou_max_error_deg:.2f}deg " +
          f"torsion_zero={self.torsion_rms_deg:.2f}/{self.torsion_max_deg:.2f}deg")


def run_guided(session: core.HrrCalibrationSession, timeout: float, assume_yes: bool) -> bool:
  print("\nSecure the stationary vehicle. HRR relays must be open and torque output interlocked.")
  print("Calibration is simple: hold near center relaxed ~1 s, sweep ~120 deg left, ~120 deg right,")
  print("then return near center and LET THE WHEEL RELAX for about 1 second.")
  print("OU electrical shape uses the entire sweep; only that final relaxed hold establishes zero torsion.")
  print("No pauses are needed during the sweep and mechanical locks are not required.")
  if not assume_yes:
    input("Press Enter when ready, or Ctrl-C to cancel: ")
  if session.dry_run:
    session.send_command(core.CMD_CAL_START, 0x12345678)
    print("Dry run: start frame generated; no samples were collected.")
    return True

  token = core.start_calibration_session(session)
  if token is None:
    return False

  print("\nHold center relaxed briefly, sweep normally, then finish center relaxed. READY appears automatically.")
  deadline = time.monotonic() + timeout
  fit_worker = core.CalibrationFitWorker()
  latest_result: core.FitWorkerResult | None = None
  next_fit = 0.0
  next_print = 0.0
  latest_fit: core.CalibrationFit | None = None
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
        print("\nNot ready yet; finish near center and let the wheel relax for about 1 second.")
    elif latest_fit is not None and latest_fit.ready:
      break
    time.sleep(0.01)
  else:
    print("\nERROR: calibration timed out.", file=sys.stderr)
    session.send_command(core.CMD_CAL_ABORT, token)
    session.wait_for(lambda status: not status.running, 1.0)
    return False

  assert latest_fit is not None
  print(f"\nFit: {latest_fit.format()}")
  return bool(core.commit_values(session, core.fit_values(latest_fit), token))


def run_self_test() -> None:
  # Generate the actual guided motion, including relaxed center holds. During
  # the moving sweep OU carries large torsion and a different ellipse from IN;
  # this must not prevent an accurate OU shape/zero calibration.
  ratio = -core.NOMINAL_PHASE_PER_STEER
  trajectory = ([0.0] * 15 +
                [-1.5 * index for index in range(1, 81)] +
                [-120.0 + 1.5 * index for index in range(1, 161)] +
                [120.0 - 1.5 * index for index in range(1, 81)] +
                [0.0] * 15)
  samples: list[core.CalibrationSample] = []
  unwrapped = 0.0
  previous_phase: float | None = None
  for index, reference in enumerate(trajectory):
    in_true = ratio * reference + 17.0
    in_raw_phase = in_true % 180.0
    if previous_phase is None:
      unwrapped = in_raw_phase
    else:
      unwrapped += core.shortest_mod180_delta(in_raw_phase, previous_phase)
    previous_phase = in_raw_phase

    moving = 15 <= index < len(trajectory) - 15
    torsion = (3.5 if reference < 0.0 else -3.0) if moving else 0.0
    ou_true = in_true + abs(ratio) * torsion + 31.0

    in_angle = math.radians((in_raw_phase + 90.0) % 180.0 - 90.0)
    in_cos = round(1000.0 * math.cos(in_angle) + 120.0 * math.sin(in_angle))
    in_sin = round(760.0 * math.sin(in_angle))

    ou_angle = math.radians((ou_true + 90.0) % 180.0 - 90.0)
    # Deliberately different ellipse and reversed OU handedness.
    ou_cos = round(820.0 * math.cos(ou_angle) - 170.0 * math.sin(ou_angle))
    ou_sin = round(-1120.0 * math.sin(ou_angle))
    samples.append(core.CalibrationSample(index * 0.1, reference, in_raw_phase, unwrapped,
                                          in_cos, in_sin, ou_cos, ou_sin))

  fit = fit_calibration_aligned(samples)
  assert fit.ready, fit.format()
  assert abs(fit.phase_per_steer - ratio) < 0.02, fit.format()
  assert fit.rms_error_deg < 0.2, fit.format()
  assert fit.ou_rms_error_deg < 0.2, fit.format()
  assert fit.torsion_rms_deg < 0.2, fit.format()
  zero_samples, shape_rms, shape_max = _FIT_DETAILS[id(fit)]
  assert zero_samples >= OU_ZERO_MIN_SAMPLES
  assert shape_rms < 0.01 and shape_max < 0.03, fit.format()

  # Without the final relaxed center hold the calibration should explicitly ask
  # for it instead of silently guessing a torsion zero from loaded sweep data.
  # Verify that the calibrated phase difference preserves real torsion, not
  # merely a zeroed OU angle.
  in_matrix = fit.matrices[:4]
  ou_matrix = fit.matrices[4:]
  positive_twist = None
  negative_twist = None
  for index, sample in enumerate(samples):
    if index < 15 or index >= len(samples) - 15:
      continue
    twist = core.shortest_mod180_delta(_phase_for(sample, ou_matrix, "ou"),
                                       _phase_for(sample, in_matrix, "in")) / abs(fit.phase_per_steer)
    if sample.reference_deg < -30.0 and positive_twist is None:
      positive_twist = twist
    if sample.reference_deg > 30.0 and negative_twist is None:
      negative_twist = twist
  assert positive_twist is not None and abs(positive_twist - 3.5) < 0.2, positive_twist
  assert negative_twist is not None and abs(negative_twist + 3.0) < 0.2, negative_twist

  try:
    fit_calibration_aligned(samples[15:-15])
  except ValueError as error:
    assert "OU zero not captured" in str(error)
  else:
    raise AssertionError("OU calibration accepted without a relaxed zero hold")

  print("HRR relaxed-OU resolver calibration self-test passed.")


def _install() -> None:
  core.fit_calibration_aligned = fit_calibration_aligned
  core.run_guided = run_guided
  core.run_self_test = run_self_test
  core.CalibrationFit.format = _format_fit


_install()

if __name__ == "__main__":
  core.main()
