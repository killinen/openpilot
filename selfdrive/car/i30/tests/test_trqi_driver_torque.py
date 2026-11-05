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
