#!/usr/bin/env python3
from openpilot.common.params import Params

class FrogPilotCard:
  def __init__(self, CP):
    self.params = Params()
    self.params_memory = Params(memory=True)

  def update(self, carState, frogpilotCarState, sm, frogpilot_toggles):
    return frogpilotCarState
