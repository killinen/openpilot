from openpilot.tools.car_porting.i30_longitudinal.coast import EarlyCoast


def test_weaker_coasting_lifts_earlier_for_distant_slower_lead():
  strong, weak = EarlyCoast(), EarlyCoast()
  assert not strong.update(True, 20.0, 100.0, -4.0, 0.0, -0.35, 2.3, 0.35)
  assert weak.update(True, 20.0, 100.0, -4.0, 0.0, -0.15, 2.3, 0.35)


def test_does_not_coast_for_steady_or_faster_lead():
  for relative_speed in (0.0, 2.0):
    assert not EarlyCoast().update(True, 20.0, 50.0, relative_speed, 0.0, -0.35, 2.3, 0.35)


def test_invalid_data_and_lost_lead_clear_latch():
  coast = EarlyCoast()
  assert coast.update(True, 20.0, 50.0, -4.0, 0.0, -0.35, 2.3, 0.35)
  assert not coast.update(False, 20.0, 50.0, -4.0, 0.0, -0.35, 2.3, 0.35)
  assert not coast.update(True, 20.0, float('nan'), -4.0, 0.0, -0.35, 2.3, 0.35)


def test_hysteresis_avoids_toggling_at_distance_threshold():
  coast = EarlyCoast()
  assert coast.update(True, 20.0, 75.0, -4.0, 0.0, -0.35, 2.3, 0.35)
  assert coast.update(True, 20.0, 79.0, -4.0, 0.0, -0.35, 2.3, 0.35)
  assert not coast.update(True, 20.0, 82.0, -4.0, 0.0, -0.35, 2.3, 0.35)
