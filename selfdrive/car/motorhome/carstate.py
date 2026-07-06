from cereal import car, custom
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.motorhome.values import DBC

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    self.prev_cruise_buttons = 0
    self.cruise_buttons = [0]
    self.main_buttons = [0]

  def update(self, cp, cp_cam, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    speed_kph = cp.vl["CCVS_00"]["WHEEL_BASED_VEHICLE_SPEED"]
    ret.vEgoRaw = speed_kph / 3.6
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.engineRpm = cp.vl["EEC1_ENGINE_00"]["ENGINE_SPEED"]

    ret.brakePressed = cp.vl["CCVS_17"]["BRAKE_SWITCH"] == 1 or cp.vl["CCVS_21"]["BRAKE_SWITCH"] == 1
    ret.brake = 0

    gas_pedal = cp.vl["EEC2_ENGINE_00"]["ACCELERATOR_PEDAL_POSITION_1"]
    ret.gas = gas_pedal / 100.0
    ret.gasPressed = gas_pedal > 1.0

    ret.cruiseState.available = cp.vl["CCVS_17"]["CRUISE_CONTROL_ENABLE_SWITCH"] == 1
    ret.cruiseState.enabled = cp.vl["CCVS_00"]["CRUISE_CONTROL_ACTIVE"] == 1
    ret.cruiseState.speed = 0.0
    ret.cruiseState.standstill = False

    ret.gearShifter = GearShifter.drive
    ret.parkingBrake = False
    ret.steeringAngleDeg = 0.0
    ret.steeringRateDeg = 0.0
    ret.steeringTorque = 0.0
    ret.steeringTorqueEps = 0.0
    ret.steeringPressed = False

    return ret, fp_ret

  @staticmethod
  def get_can_parser(CP, FPCP):
    messages = [
      ("CCVS_00", 5),
      ("CCVS_17", 5),
      ("CCVS_21", 10),
      ("EEC1_ENGINE_00", 20),
      ("EEC2_ENGINE_00", 20),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    # SSC will live on bus 1, but it is not connected yet. Keep this parser
    # empty so missing SSC status frames do not invalidate the interface.
    return CANParser(DBC[CP.carFingerprint]["pt"], [], 1)
