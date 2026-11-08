from cereal import car, custom
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car import create_button_events
from openpilot.selfdrive.car.i30.i30can import calc_checksum_8bit
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.motorhome.values import DBC, MotorhomeFlags, CarControllerParams

GearShifter = car.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    self.prev_cruise_buttons = 0
    self.cruise_buttons = [0]
    self.main_buttons = [0]
    self.ssc_enabled = bool(CP.flags & MotorhomeFlags.SSC_ENABLED)
    self.ssc_timestamp = 0
    self.ssc_counter = None
    self.ssc_valid = False

  def update_ssc_status(self, cp_cam):
    timestamp = cp_cam.ts_nanos["STEERING_STATUS"]["SERVO_COUNTER"]
    if timestamp == self.ssc_timestamp:
      return
    samples = cp_cam.vl_all["STEERING_STATUS"]
    batch_valid = True
    for index in range(len(samples["SERVO_COUNTER"])):
      status = {key: values[index] for key, values in samples.items()}
      counter = int(status["SERVO_COUNTER"])
      # Checksum covers bytes 1..6, excluding DEBUG_STATES, as in i30 SSC.
      angle = round(status["STEERING_ANGLE"] * 8) & 0xFFFF
      data = bytes([counter | (int(status["CONTROL_STATUS"]) << 4), round(status["STEERING_TORQUE"] * 8) & 0xFF,
                    round(status["STEERING_SPEED"] * 64) & 0xFF, round(status["TEMPERATURE"] + 60) & 0xFF,
                    angle & 0xFF, angle >> 8])
      batch_valid &= status["SERVO_CHECKSUM"] == calc_checksum_8bit(data, 0x22F) and (
        self.ssc_counter is None or counter == (self.ssc_counter + 1) % 16)
      self.ssc_counter = counter
    self.ssc_valid = batch_valid
    self.ssc_timestamp = timestamp

  def update(self, cp, cp_cam, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    ret.doorOpen = False
    ret.seatbeltUnlatched = False

    speed_kph = cp.vl["CCVS_00"]["WHEEL_BASED_VEHICLE_SPEED"]
    speed_valid = 0 <= speed_kph <= 250
    ret.vEgoRaw = speed_kph / 3.6 if speed_valid else 0.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.engineRpm = cp.vl["EEC1_ENGINE_00"]["ENGINE_SPEED"]

    ret.brakePressed = cp.vl["CCVS_17"]["BRAKE_SWITCH"] == 1 or cp.vl["CCVS_21"]["BRAKE_SWITCH"] == 1
    ret.brake = 0

    gas_pedal = cp.vl["EEC2_ENGINE_00"]["ACCELERATOR_PEDAL_POSITION_1"]
    ret.gas = gas_pedal / 100.0
    ret.gasPressed = gas_pedal > 1.0

    master = cp.vl["CCVS_17"]
    switches = [master[f"CRUISE_CONTROL_{name}_SWITCH"] for name in ("SET", "COAST", "RESUME", "ACCELERATE")]
    # 0x55 is seen briefly before source17 disappears at session end.
    switches_valid = all(value in (0, 1) for value in switches) and not all(value == 1 for value in switches)
    active = cp.vl["CCVS_00"]["CRUISE_CONTROL_ACTIVE"]
    brakes_valid = master["BRAKE_SWITCH"] in (0, 1) and cp.vl["CCVS_21"]["BRAKE_SWITCH"] in (0, 1)
    ret.cruiseState.available = cp.can_valid and speed_valid and active in (0, 1) and switches_valid and brakes_valid and master["CRUISE_CONTROL_ENABLE_SWITCH"] == 1
    ret.cruiseState.enabled = active == 1
    set_speed = cp.vl["CCVS_00"]["CRUISE_CONTROL_SET_SPEED"]
    ret.cruiseState.speed = set_speed / 3.6 if set_speed <= 250 else 0.0
    ret.cruiseState.standstill = False

    self.prev_cruise_buttons = self.cruise_buttons[-1]
    button = 1 if switches_valid and switches[2] == 1 else 2 if switches_valid and switches[0] == 1 else 0
    self.cruise_buttons = [button]
    self.main_buttons = [int(ret.cruiseState.available)]
    ret.buttonEvents = create_button_events(button, self.prev_cruise_buttons,
                                           {1: car.CarState.ButtonEvent.Type.accelCruise, 2: car.CarState.ButtonEvent.Type.decelCruise})

    transmission = cp.vl["ETC2_03"]
    selected = transmission["TRANSMISSION_SELECTED_GEAR"]
    current = transmission["TRANSMISSION_CURRENT_GEAR"]
    ret.gearShifter = GearShifter.unknown
    if cp.ts_nanos["ETC2_03"]["TRANSMISSION_SELECTED_GEAR"]:
      if selected == 126:  # J1939 raw 251 means Park.
        ret.gearShifter = GearShifter.park
      elif selected == 0:
        ret.gearShifter = GearShifter.neutral
      elif -125 <= selected < 0:
        ret.gearShifter = GearShifter.reverse
      elif 0 < selected <= 125 and 0 < current <= 125:
        ret.gearShifter = GearShifter.drive
    ret.parkingBrake = False
    ret.steeringAngleDeg = 0.0
    ret.steeringRateDeg = 0.0
    ret.steeringTorque = 0.0
    ret.steeringTorqueEps = 0.0
    ret.steeringPressed = False

    if self.ssc_enabled:
      self.update_ssc_status(cp_cam)
      status = cp_cam.vl["STEERING_STATUS"]
      control_status = int(status["CONTROL_STATUS"])
      ret.steerFaultTemporary = not (self.ssc_valid and cp_cam.can_valid and control_status & CarControllerParams.SSC_FEEDBACK_ENABLED) or bool(control_status & CarControllerParams.SSC_SOFT_OFF)
      # Firmware must report calibrated steering-wheel units; no i30 pulley ratio.
      ret.steeringAngleDeg = status["STEERING_ANGLE"]
      ret.steeringRateDeg = status["STEERING_SPEED"] * 360.0
      ret.steeringTorqueEps = status["STEERING_TORQUE"]

    return ret, fp_ret

  @staticmethod
  def get_can_parser(CP, FPCP):
    messages = [
      ("CCVS_00", 10),
      ("CCVS_17", 10),
      ("CCVS_21", 10),
      ("EEC1_ENGINE_00", 20),
      ("EEC2_ENGINE_00", 20),
      ("ETC2_03", 10),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    messages = [("STEERING_STATUS", 20)] if CP.flags & MotorhomeFlags.SSC_ENABLED else []
    return CANParser("hyundai_i30_2014", messages, 1)
