"""Experimental LS600h angle observer. Its outputs must never drive actuators.

All timeouts/gates below are hypotheses for shadow evaluation, not validated
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


class HrrAngleShadow:
  VERSION = 1
  FRESH_SECONDS = 0.05
  HOLD_SECONDS = 0.03
  BRIDGE_SECONDS = 0.25
  INNOVATION_DEG = 3.0
  RECOVERY_SAMPLES = 5

  def __init__(self):
    self.oem_time = None
    self.hrr_time = None
    self.counter = None
    self.anchor_oem = None
    self.anchor_hrr = None
    self.last_good_time = None
    self.last_angle = 0.0
    self.provisional_offset = None
    self.good_samples = 0
    self.degraded = True
    self.latched = False

  def update(self, now: float, oem: float, oem_rate: float, oem_updated: bool,
             hrr: float, hrr_rate: float, hrr_updated: bool, counter: int,
             valid: bool, wrap: bool, lat_active: bool, hrr_sample_count: int = 1) -> AngleShadowResult:
    finite_oem = math.isfinite(oem) and math.isfinite(oem_rate)
    finite_hrr = math.isfinite(hrr) and math.isfinite(hrr_rate)
    if oem_updated and finite_oem:
      self.oem_time = now
    progressed = hrr_updated and finite_hrr and (self.counter is None or counter != self.counter)
    # A normal control iteration can receive multiple sequential HRR packets.
    counter_gap = progressed and self.counter is not None and ((counter - self.counter) & 255) != hrr_sample_count
    if counter_gap:
      self.degraded = True
      self.good_samples = 0
    if progressed:
      self.hrr_time = now
      self.counter = counter
    oem_age = now - self.oem_time if self.oem_time is not None else 1e6
    hrr_age = now - self.hrr_time if self.hrr_time is not None else 1e6
    oem_fresh = finite_oem and oem_age <= self.FRESH_SECONDS
    hrr_fresh = finite_hrr and hrr_age <= self.FRESH_SECONDS
    offset = self.anchor_hrr - self.anchor_oem if self.anchor_oem is not None else None
    expected = oem + offset if offset is not None and oem_fresh else self.last_angle
    reference = offset if offset is not None else self.provisional_offset
    innovation = hrr - (oem + reference) if reference is not None and finite_oem and finite_hrr else 0.0
    coherent = reference is None or abs(innovation) <= self.INNOVATION_DEG
    good = valid and not wrap and hrr_fresh and oem_fresh and coherent
    if good and progressed:
      if self.provisional_offset is None:
        self.provisional_offset = hrr - oem
      self.good_samples = min(self.good_samples + 1, self.RECOVERY_SAMPLES)
    elif not good:
      self.good_samples = 0
      self.degraded = True
      if self.anchor_oem is None:
        self.provisional_offset = None

    qualified = good and (not self.degraded or self.good_samples >= self.RECOVERY_SAMPLES)
    if qualified:
      if progressed:
        self.anchor_oem, self.anchor_hrr = oem, hrr
        self.last_good_time = now
      self.last_angle = hrr
      self.degraded = False
      if not lat_active:
        self.latched = False
      mode, reason, angle, rate, usable = 'trustedHrr', 'none', hrr, hrr_rate, True
    else:
      age = now - self.last_good_time if self.last_good_time is not None else 1e6
      reason = ('staleOem' if not oem_fresh else 'staleHrr' if not hrr_fresh else 'counterDiscontinuity' if counter_gap else
                'inconsistent' if not coherent else 'invalidHrr' if not valid or wrap else 'qualifying')
      if self.last_good_time is not None and wrap and hrr_fresh and age <= self.HOLD_SECONDS:
        mode, angle, rate, usable = 'hold', self.last_angle, 0.0, True
      elif offset is not None and oem_fresh and age <= self.BRIDGE_SECONDS:
        mode, angle, rate, usable = 'alignedOem', expected, oem_rate, True
      else:
        mode, angle, rate, usable = 'unavailable', expected, 0.0, False
      self.last_angle = angle

    if lat_active and not usable:
      self.latched = True
    if self.latched:
      mode, reason, usable = 'unavailable', 'latched', False
    return AngleShadowResult(mode, reason, angle, rate, usable, not usable,
                             (self.anchor_hrr - self.anchor_oem) if self.anchor_oem is not None else 0.0,
                             innovation, oem_age, hrr_age, self.good_samples)
