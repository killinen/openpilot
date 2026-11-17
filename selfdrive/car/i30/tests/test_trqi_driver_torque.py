"""openpilot's steeringTorque is positive left; CR_Mdps_StrTq is negative left."""
from collections import defaultdict
from types import SimpleNamespace

import pytest
from cereal import car

from openpilot.selfdrive.car.i30 import carstate


def parser(**messages):
  vl = defaultdict(lambda: defaultdict(float))
  for name, signals in messages.items():
    vl[name].update(signals)
  return SimpleNamespace(vl=vl, vl_all=defaultdict(lambda: defaultdict(list)),
                         ts_nanos=defaultdict(lambda: defaultdict(int)), can_valid=True)


@pytest.mark.parametrize("trqi,expected", [(True, 2.5), (False, -2.5)])
def test_driver_left_torque_sign_by_backend(monkeypatch, trqi, expected):
  monkeypatch.setattr(carstate, "i30_uses_trqi_steering", lambda: trqi)
  state = carstate.CarState(car.CarParams.new_message(), None)
  ret, _ = state.update_i30(parser(VSM2={"CR_Mdps_StrTq": -2.5}), parser())
  assert ret.steeringTorque == pytest.approx(expected)


def test_trqi_snr_difference_matches_firmware_zero(monkeypatch):
  monkeypatch.setattr(carstate, "i30_uses_trqi_steering", lambda: True)
  state = carstate.CarState(car.CarParams.new_message(), None)
  cam = parser(TRQI_VoltageStatus={"SNR1_VIN": 2.400, "SNR2_VIN": 2.495},
               TRQI_AdcStatus={"SNR_Passthrough_Pending": 1, "SNR_Passthrough_Requested": 0})
  state.update_i30(parser(), cam)
  assert state.trqi_snr_diff_mv == pytest.approx(0.0, abs=1e-6)  # firmware zero is -95 mV
  assert state.trqi_snr_passthrough_pending and not state.trqi_snr_passthrough_requested
  state.update_i30(parser(), parser(TRQI_VoltageStatus={"SNR1_VIN": 2.400, "SNR2_VIN": 2.325}))
  assert state.trqi_snr_diff_mv == pytest.approx(170.0)
  state.update_i30(parser(), parser(TRQI_VoltageStatus={"SNR1_VIN": 2.400, "SNR2_VIN": 2.665}))
  assert state.trqi_snr_diff_mv == pytest.approx(170.0)
