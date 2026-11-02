"""Experimental LS600h angle observer. Its outputs must never drive actuators.

All thresholds below are hypotheses for shadow evaluation, not validated
steering availability limits. The observer neither changes CarState nor tunes.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AngleShadowResult:
  mode: str
  reason: str
  angle: float
  rate: float
  usable: bool
  would_disable: bool
  offset: float
  innovation: float
  oem_age: float
  hrr_age: float
  recovery_samples: int
  sector_offset: float


class HrrAngleShadow:
  VERSION = 2
  FRESH_SECONDS = 0.05
  BRIDGE_SECONDS = 2.5
  RECOVERY_SAMPLES = 5
  # Measured HRR minus OEM angle near center. Logs do not support it beyond the envelope.
  NOMINAL_OFFSET_DEG = -7.2
  ENVELOPE_DEG = 60.0
  # HRR sectors repeat every 22.5 degrees of shaft rotation; firmware picks the one nearest raw OEM.
  SECTOR_DEG = 22.5
  SECTOR_MARGIN_DEG = 9.0
  # Firmware reports one sector change only through invalidation, so a larger step is unexplained.
  MAX_UNEXPLAINED_STEP_DEG = 11.25

  def __init__(self):
    self.oem_time = None
    self.hrr_time = None
    self.counter = None
    self.last_hrr = None
    self.sector = None
    self.reference_offset = self.NOMINAL_OFFSET_DEG
    self.last_good_time = None
    self.last_angle = 0.0
    self.good_samples = 0

  def _resolve_sector(self, hrr: float, oem: float) -> int | None:
    deviation = hrr - oem - self.NOMINAL_OFFSET_DEG
    sector = round(deviation / self.SECTOR_DEG)
    return sector if abs(deviation - sector * self.SECTOR_DEG) <= self.SECTOR_MARGIN_DEG else None

  def update(self, now: float, oem: float, oem_rate: float, oem_updated: bool,
             hrr: float, hrr_rate: float, hrr_updated: bool, counter: int,
             valid: bool, wrap: bool, hrr_sample_count: int = 1) -> AngleShadowResult:
    finite_oem = math.isfinite(oem) and math.isfinite(oem_rate)
    finite_hrr = math.isfinite(hrr) and math.isfinite(hrr_rate)
    if oem_updated and finite_oem:
      self.oem_time = now
    progressed = hrr_updated and finite_hrr and (self.counter is None or counter != self.counter)
    # A normal control iteration can receive multiple sequential HRR packets.
    counter_gap = progressed and self.counter is not None and ((counter - self.counter) & 255) != hrr_sample_count
    if progressed:
      self.hrr_time = now
      self.counter = counter
    oem_age = now - self.oem_time if self.oem_time is not None else 1e6
    hrr_age = now - self.hrr_time if self.hrr_time is not None else 1e6
    oem_fresh = finite_oem and oem_age <= self.FRESH_SECONDS
    hrr_fresh = finite_hrr and hrr_age <= self.FRESH_SECONDS
    flags_good = valid and not wrap and hrr_fresh

    unexplained_step = (progressed and flags_good and self.last_hrr is not None and
                        abs(hrr - self.last_hrr) > self.MAX_UNEXPLAINED_STEP_DEG + abs(hrr_rate) * self.FRESH_SECONDS)
    if not flags_good or counter_gap or unexplained_step:
      # Sector continuity is only guaranteed between firmware invalidations.
      self.sector = None
      self.good_samples = 0
    if not flags_good:
      self.last_hrr = None
    elif progressed:
      self.last_hrr = hrr

    in_envelope = oem_fresh and abs(oem + self.NOMINAL_OFFSET_DEG) <= self.ENVELOPE_DEG
    if flags_good and self.sector is None and in_envelope:
      self.sector = self._resolve_sector(hrr, oem)
    outside_envelope = flags_good and oem_fresh and not in_envelope
    if outside_envelope:
      # The nominal offset cannot select a sector here, so do not retain or apply one.
      self.sector = None

    resolved = flags_good and (self.sector is not None or outside_envelope)
    if resolved and progressed:
      self.good_samples = min(self.good_samples + 1, self.RECOVERY_SAMPLES)
    elif not resolved:
      self.good_samples = 0

    sector_offset = (self.sector or 0) * self.SECTOR_DEG
    corrected = hrr - sector_offset
    # Diagnostic only. OEM excursions make this large while corrected HRR stays right.
    innovation = corrected - oem - self.reference_offset if finite_oem and finite_hrr else 0.0

    if resolved and self.good_samples >= self.RECOVERY_SAMPLES:
      if progressed:
        self.last_good_time = now
        if oem_fresh:
          self.reference_offset = corrected - oem
      self.last_angle = corrected
      if outside_envelope:
        mode, reason = 'trustedHrr', 'outsideEnvelope'
      elif sector_offset:
        mode, reason = 'sectorCorrectedHrr', 'sectorCorrected'
      else:
        mode, reason = 'trustedHrr', 'none'
      angle, rate, usable = corrected, hrr_rate, True
    else:
      age = now - self.last_good_time if self.last_good_time is not None else 1e6
      reason = ('staleOem' if not oem_fresh else 'staleHrr' if not hrr_fresh else
                'counterDiscontinuity' if counter_gap else 'inconsistent' if unexplained_step else
                'invalidHrr' if not valid or wrap else 'sectorAmbiguous' if not resolved else 'qualifying')
      if oem_fresh and age <= self.BRIDGE_SECONDS:
        mode, angle, rate, usable = 'alignedOem', oem + self.reference_offset, oem_rate, True
      else:
        mode, rate, usable = 'unavailable', 0.0, False
        angle = oem + self.NOMINAL_OFFSET_DEG if oem_fresh else self.last_angle
      self.last_angle = angle

    return AngleShadowResult(mode, reason, angle, rate, usable, not usable, self.reference_offset,
                             innovation, oem_age, hrr_age, self.good_samples, sector_offset)
