import numpy as np
import pytest

from openpilot.tools.car_porting.i30_longitudinal.fit import fit_map, score
from openpilot.tools.car_porting.i30_longitudinal.lookup import PedalMap


def observations():
  data = []
  for route in ('a', 'b', 'c'):
    for gear in (4, 5, 6):
      for speed in np.linspace(10, 24, 9):
        for pitch in (-0.04, 0.0, 0.04):
          for pedal in (0.0, 0.02, 0.07, 0.15, 0.25, 0.4, 0.6, 0.8, 1.0):
            acceleration = -0.3 - 0.01 * (speed - 17) - 9.81 * np.sin(pitch) + pedal * (2.5 - 0.2 * (gear - 4))
            data.append(dict(route=route, gear=gear, v=speed, pitch=pitch, pedal=pedal, coast=pedal == 0, a=acceleration))
  return {k: np.array([row[k] for row in data]) for k in data[0]}


def test_fits_known_plant_and_exports_a_monotone_inverse():
  data = observations()
  train = {k: v[data['route'] != 'c'] for k, v in data.items()}
  validation = {k: v[data['route'] == 'c'] for k, v in data.items()}
  calibration = fit_map(train)
  assert calibration['pitch_gain'] == pytest.approx(1.0, abs=0.01)
  assert score(calibration, validation)['all']['mae'] < 0.02
  lookup = PedalMap(calibration)
  for gear in (4, 5, 6):
    model = calibration['gears'][str(gear)]
    assert np.all(np.diff(model['accel'], axis=1) > 0)
    pedal = lookup.pedal(gear, 17.0, 0.0, 0.1)
    assert pedal is not None
    assert lookup.acceleration(gear, 17.0, 0.0, pedal) == pytest.approx(0.1)


def test_rejects_missing_gear_coast_coverage():
  data = observations()
  data['coast'][data['gear'] == 6] = False
  with pytest.raises(ValueError, match='gear 6'):
    fit_map(data)
