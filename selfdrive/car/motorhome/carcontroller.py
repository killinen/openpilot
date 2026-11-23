import math

from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.i30.i30can import create_steer_command
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.motorhome.values import CarControllerParams, MotorhomeFlags


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.ssc_enabled = bool(CP.flags & MotorhomeFlags.SSC_ENABLED)
    self.packer = CANPacker("hyundai_i30_2014") if self.ssc_enabled else None
    self.frame = 0
    self.torque = 0.0

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators.as_builder()
    actuators.steer = 0.0
    actuators.accel = actuators.gas = 0.0
    if not self.ssc_enabled:
      # Engagement testing: no SSC messages and no feedback dependency.
      self.torque = 0.0
      return actuators, []

    active = CC.latActive and CS.out.canValid and CS.out.cruiseState.available and CS.out.gearShifter == car.CarState.GearShifter.drive and not (
      CS.out.steerFaultTemporary or CS.out.steerFaultPermanent)
    active &= math.isfinite(CC.actuators.steer)
    if active:
      target = clip(CC.actuators.steer, -1.0, 1.0) * CarControllerParams.STEER_MAX
      increasing = target * self.torque >= 0 and abs(target) > abs(self.torque)
      rate = CarControllerParams.STEER_RATE_UP if increasing else CarControllerParams.STEER_RATE_DOWN
      if target * self.torque < 0:
        target = 0.0  # Reach zero before reversing torque direction.
      self.torque = clip(target, self.torque - rate * DT_CTRL, self.torque + rate * DT_CTRL)
    else:
      self.torque = 0.0

    # Torque mode only. Disabled commands explicitly clear angle and torque.
    sends = [create_steer_command(self.packer, int(active), 0.0, self.torque, self.frame)]
    actuators.steer = round(self.torque * 8) / (8 * CarControllerParams.STEER_MAX)
    self.frame += 1
    return actuators, sends
