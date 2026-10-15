#!/usr/bin/env python3
import gc
import os

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring


# Development-only active-DM timing profile. It is never used while controls are engaged.
DM_TEST_LONG_TIMINGS = os.getenv("DM_TEST_LONG_TIMINGS") == "1"

# Production active-DM timing (seconds).
DM_STOCK_TIME = 22.
DM_STOCK_PRE_TIME_TILL_TERMINAL = 16.
DM_STOCK_PROMPT_TIME_TILL_TERMINAL = 12.

# Test profile: pre at 10 s, prompt at 18 s, terminal at 30 s.
DM_TEST_TIME = 30.
DM_TEST_PRE_TIME_TILL_TERMINAL = 20.
DM_TEST_PROMPT_TIME_TILL_TERMINAL = 12.


def set_active_dm_timing_profile(DM, op_engaged):
  use_test_profile = DM_TEST_LONG_TIMINGS and not op_engaged

  if use_test_profile:
    distracted_time = DM_TEST_TIME
    pre_time_till_terminal = DM_TEST_PRE_TIME_TILL_TERMINAL
    prompt_time_till_terminal = DM_TEST_PROMPT_TIME_TILL_TERMINAL
  else:
    distracted_time = DM_STOCK_TIME
    pre_time_till_terminal = DM_STOCK_PRE_TIME_TILL_TERMINAL
    prompt_time_till_terminal = DM_STOCK_PROMPT_TIME_TILL_TERMINAL

  # Update both the settings and derived active-mode values every frame. This guarantees
  # that engaging controls immediately restores the production 22 s terminal timeout,
  # even if the test profile was active immediately beforehand.
  DM.settings._DISTRACTED_TIME = distracted_time
  DM.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL = pre_time_till_terminal
  DM.settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL = prompt_time_till_terminal

  if DM.active_monitoring_mode:
    DM.threshold_pre = pre_time_till_terminal / distracted_time
    DM.threshold_prompt = prompt_time_till_terminal / distracted_time
    DM.step_change = DM.settings._DT_DMON / distracted_time


def dmonitoringd_thread():
  gc.disable()
  set_realtime_priority(2)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2', 'carControl'], poll='driverStateV2')

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))

  # FrogPilot variables
  driver_view_enabled = params.get_bool("IsDriverViewEnabled")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      # iterate when model has new output
      continue

    valid = sm.all_checks()
    if valid:
      op_engaged = sm['controlsState'].enabled or sm['carControl'].latActive
      set_active_dm_timing_profile(DM, op_engaged)
      DM.run_step(sm)
    elif driver_view_enabled:
      DM.face_detected = sm['driverStateV2'].leftDriverData.faceProb > DM.settings._FACE_THRESHOLD or sm['driverStateV2'].rightDriverData.faceProb > DM.settings._FACE_THRESHOLD

    # publish
    dat = DM.get_state_packet(valid=valid or driver_view_enabled)
    pm.send('driverMonitoringState', dat)

    # load live always-on toggle
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     DM.wheelpos_learner.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_learner.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
