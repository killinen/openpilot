#!/usr/bin/env python3
import math

import json
import numpy as np

import cereal.messaging as messaging
from cereal import car
from common.params import Params, put_nonblocking
from common.realtime import set_realtime_priority, DT_MDL
from common.numpy_fast import clip
from selfdrive.locationd.models.car_kf import CarKalman, ObservationKind, States
from selfdrive.locationd.models.constants import GENERATED_DIR
from selfdrive.swaglog import cloudlog

ROLL_MAX_DELTA = np.radians(20.0) * DT_MDL  # 20deg in 1 second is well within curvature limits
ROLL_MIN, ROLL_MAX = math.radians(-10), math.radians(10)

class ParamsLearner:
  def __init__(self, CP, steer_ratio, stiffness_factor, angle_offset):
    self.kf = CarKalman(GENERATED_DIR, steer_ratio, stiffness_factor, angle_offset)

    self.kf.filter.set_mass(CP.mass)  # pylint: disable=no-member
    self.kf.filter.set_rotational_inertia(CP.rotationalInertia)  # pylint: disable=no-member
    self.kf.filter.set_center_to_front(CP.centerToFront)  # pylint: disable=no-member
    self.kf.filter.set_center_to_rear(CP.wheelbase - CP.centerToFront)  # pylint: disable=no-member
    self.kf.filter.set_stiffness_front(CP.tireStiffnessFront)  # pylint: disable=no-member
    self.kf.filter.set_stiffness_rear(CP.tireStiffnessRear)  # pylint: disable=no-member

    self.active = False

    self.speed = 0
    self.roll = 0.0
    self.steering_pressed = False
    self.steering_angle = 0

    self.valid = True

  def handle_log(self, t, which, msg):
    if which == 'liveLocationKalman':

      yaw_rate = msg.angularVelocityCalibrated.value[2]
      yaw_rate_std = msg.angularVelocityCalibrated.std[2]

      localizer_roll = msg.orientationNED.value[0]
      localizer_roll_std = np.radians(1) if np.isnan(msg.orientationNED.std[0]) else msg.orientationNED.std[0]
      roll_valid = msg.orientationNED.valid and ROLL_MIN < localizer_roll < ROLL_MAX
      if roll_valid:
        roll = localizer_roll
        # Experimentally found multiplier of 2 to be best trade-off between stability and accuracy or similar?
        roll_std = 2 * localizer_roll_std
      else:
        # This is done to bound the road roll estimate when localizer values are invalid
        roll = 0.0
        roll_std = np.radians(10.0)
      self.roll = clip(roll, self.roll - ROLL_MAX_DELTA, self.roll + ROLL_MAX_DELTA)

      yaw_rate_valid = msg.angularVelocityCalibrated.valid
      yaw_rate_valid = yaw_rate_valid and 0 < yaw_rate_std < 10  # rad/s
      yaw_rate_valid = yaw_rate_valid and abs(yaw_rate) < 1  # rad/s

      if self.active:
        if msg.inputsOK and msg.posenetOK:

          if yaw_rate_valid:
            self.kf.predict_and_observe(t,
                                        ObservationKind.ROAD_FRAME_YAW_RATE,
                                        np.array([[-yaw_rate]]),
                                        np.array([np.atleast_2d(yaw_rate_std**2)]))

          #self.kf.predict_and_observe(t,
          #                            ObservationKind.ROAD_ROLL,
          #                            np.array([[self.roll]]),
          #                            np.array([np.atleast_2d(roll_std**2)]))
        self.kf.predict_and_observe(t, ObservationKind.ANGLE_OFFSET_FAST, np.array([[0]]))

        # We observe the current stiffness and steer ratio (with a high observation noise) to bound
        # the respective estimate STD. Otherwise the STDs keep increasing, causing rapid changes in the
        # states in longer routes (especially straight stretches).
        stiffness = float(self.kf.x[States.STIFFNESS])
        steer_ratio = float(self.kf.x[States.STEER_RATIO])
        self.kf.predict_and_observe(t, ObservationKind.STIFFNESS, np.array([[stiffness]]))
        self.kf.predict_and_observe(t, ObservationKind.STEER_RATIO, np.array([[steer_ratio]]))

    elif which == 'carState':
      self.steering_angle = msg.steeringAngleDeg
      self.steering_pressed = msg.steeringPressed
      self.speed = msg.vEgo

      in_linear_region = abs(self.steering_angle) < 45 or not self.steering_pressed
      self.active = self.speed > 1 and in_linear_region

      if self.active:
        self.kf.predict_and_observe(t, ObservationKind.STEER_ANGLE, np.array([[[math.radians(msg.steeringAngleDeg)]]]))
        self.kf.predict_and_observe(t, ObservationKind.ROAD_FRAME_X_SPEED, np.array([[[self.speed]]]))

    if not self.active:
      # Reset time when stopped so uncertainty doesn't grow
      self.kf.filter.filter_time = t
      self.kf.filter.reset_rewind()


def main(sm=None, pm=None):
  if sm is None:
    sm = messaging.SubMaster(['liveLocationKalman', 'carState'], poll=['liveLocationKalman'])
  if pm is None:
    pm = messaging.PubMaster(['liveParameters'])

  params_reader = Params()
  # wait for stats about the car to come in from controls
  cloudlog.info("paramsd is waiting for CarParams")
  CP = car.CarParams.from_bytes(params_reader.get("CarParams", block=True))
  cloudlog.info("paramsd got CarParams")

  min_sr, max_sr = 0.5 * CP.steerRatio, 2.0 * CP.steerRatio

  params = params_reader.get("LiveParameters")

  # Check if car model matches
  if params is not None:
    params = json.loads(params)
    if params.get('carFingerprint', None) != CP.carFingerprint:
      cloudlog.info("Parameter learner found parameters for wrong car.")
      params = None

  try:
    angle_offset_sane = abs(params.get('angleOffsetAverageDeg')) < 10.0
    steer_ratio_sane = min_sr <= params['steerRatio'] <= max_sr
    params_sane = angle_offset_sane and steer_ratio_sane
    if params is not None and not params_sane:
      cloudlog.info(f"Invalid starting values found {params}")
      params = None
  except Exception as e:
    cloudlog.info(f"Error reading params {params}: {str(e)}")
    params = None

  # TODO: cache the params with the capnp struct
  if params is None:
    params = {
      'carFingerprint': CP.carFingerprint,
      'steerRatio': CP.steerRatio,
      'stiffnessFactor': 1.0,
      'angleOffsetAverageDeg': 0.0,
    }
    cloudlog.info("Parameter learner resetting to default values")

  # When driving in wet conditions the stiffness can go down, and then be too low on the next drive
  # Without a way to detect this we have to reset the stiffness every drive
  params['stiffnessFactor'] = 1.0

  learner = ParamsLearner(CP, params['steerRatio'], params['stiffnessFactor'], math.radians(params['angleOffsetAverageDeg']))

  while True:
    sm.update()

    for which, updated in sm.updated.items():
      if updated:
        t = sm.logMonoTime[which] * 1e-9
        learner.handle_log(t, which, sm[which])

    if sm.updated['liveLocationKalman']:
      msg = messaging.new_message('liveParameters')
      msg.logMonoTime = sm.logMonoTime['carState']

      msg.liveParameters.posenetValid = True
      msg.liveParameters.sensorValid = True

      x = learner.kf.x
      msg.liveParameters.steerRatio = float(x[States.STEER_RATIO]) 
      msg.liveParameters.stiffnessFactor = float(x[States.STIFFNESS])
      msg.liveParameters.angleOffsetAverageDeg = math.degrees(x[States.ANGLE_OFFSET])
      msg.liveParameters.angleOffsetDeg = msg.liveParameters.angleOffsetAverageDeg + math.degrees(x[States.ANGLE_OFFSET_FAST])
      msg.liveParameters.valid = all((
        abs(msg.liveParameters.angleOffsetAverageDeg) < 10.0,
        abs(msg.liveParameters.angleOffsetDeg) < 10.0,
        0.2 <= msg.liveParameters.stiffnessFactor <= 5.0,
        min_sr <= msg.liveParameters.steerRatio <= max_sr,
      ))

      if sm.frame % 1200 == 0:  # once a minute
        params = {
          'carFingerprint': CP.carFingerprint,
          'steerRatio': msg.liveParameters.steerRatio,
          'stiffnessFactor': msg.liveParameters.stiffnessFactor,
          'angleOffsetAverageDeg': msg.liveParameters.angleOffsetAverageDeg,
        }
        put_nonblocking("LiveParameters", json.dumps(params))

      pm.send('liveParameters', msg)


if __name__ == "__main__":
  main()
