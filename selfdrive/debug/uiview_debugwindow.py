#!/usr/bin/env python3
import time
import psutil
import json
from collections import namedtuple
from cereal import messaging, log
from selfdrive.manager.process_config import managed_processes

TestAlert = False

ThermalConfig = namedtuple('ThermalConfig', ['cpu', 'gpu', 'mem', 'bat', 'ambient', 'pmic'])

# Instantiate properly:
thermal_config = ThermalConfig(
  cpu=([0, 1, 2, 3], 1000),  # thermal zones 0-3, scale factor = 1000
  gpu=None,
  mem=None,
  bat=None,
  ambient=None,
  pmic=None
)

def read_tz(x):
  if x is None:
    return 0

  try:
    with open(f"/sys/devices/virtual/thermal/thermal_zone{x}/temp") as f:
      return int(f.read())
  except FileNotFoundError:
    return 0

# You should try to do this so that you start thermald and get the cpu stats there (for practice)
if __name__ == "__main__":
  procs = ['camerad', 'ui', 'modeld', 'calibrationd']

  for p in procs:
    managed_processes[p].start()

  pm = messaging.PubMaster(['controlsState', 'deviceState', 'liveParameters', 'pandaStates', 'carParams'])

  msgs = {s: messaging.new_message(s) for s in ['controlsState', 'deviceState', 'carParams']}
  msgs['deviceState'].deviceState.started = True
  msgs['deviceState'].deviceState.cpuUsagePercent = [42, 38, 41, 39]
  msgs['deviceState'].deviceState.cpuTempC = [read_tz(z) / thermal_config.cpu[1] for z in thermal_config.cpu[0]]

  msgs['carParams'].carParams.openpilotLongitudinalControl = True

  # Set controlsState fields to simulate engage-ability
  msgs['controlsState'].controlsState.enabled = False
  msgs['controlsState'].controlsState.engageable = False  # <<< This triggers the icon
  msgs['controlsState'].controlsState.alertText1 = ""
  msgs['controlsState'].controlsState.alertSize = log.ControlsState.AlertSize.none
  #msgs['controlsState'].controlsState.alertSound = log.ControlsState.AlertSound.none  # << silent!

  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno

  # Read angleOffsetAverageDeg from LiveParameters
  try:
    with open("/data/params/d/LiveParameters") as f:
      live_params = json.load(f)
      angle_offset = float(live_params.get("angleOffsetAverageDeg", 0.0))
  except Exception as e:
    print(f"Failed to read LiveParameters: {e}")
    angle_offset = 0.0

  # Create the liveParameters message
  msgs["liveParameters"] = messaging.new_message("liveParameters")
  msgs["liveParameters"].liveParameters.angleOffsetAverageDeg = angle_offset

  # Initially not engageable
  engageable = False
  start_time = time.monotonic()

  try:
    while True:
      # Update CPU temp dynamically
      msgs['deviceState'].deviceState.cpuTempC = [read_tz(z) / thermal_config.cpu[1] for z in thermal_config.cpu[0]]
      # Update CPU usage dynamically
      msgs['deviceState'].deviceState.cpuUsagePercent = [int(round(n)) for n in psutil.cpu_percent(percpu=True)]

      now = time.monotonic()

      # After 10 seconds, set engageable = True
      if not engageable and now - start_time > 10:
        engageable = True

      # After 20 seconds: trigger alert
      show_alert = now - start_time > 20

      # Update the message dynamically
      msgs['controlsState'] = messaging.new_message('controlsState')
      msgs['controlsState'].controlsState.enabled = False
      msgs['controlsState'].controlsState.engageable = engageable
      msgs['controlsState'].controlsState.alertText1 = ""
      msgs['controlsState'].controlsState.alertSize = log.ControlsState.AlertSize.none

      # Continuously send liveParameters (angleOffsetAverageDeg)
      msgs["liveParameters"] = messaging.new_message("liveParameters")
      msgs["liveParameters"].liveParameters.angleOffsetAverageDeg = angle_offset


      if show_alert and TestAlert:
        msgs['controlsState'].controlsState.alertText1 = "Mild alert test"
        msgs['controlsState'].controlsState.alertText2 = "Triggered after 20s"
        msgs['controlsState'].controlsState.alertSize = log.ControlsState.AlertSize.mid
        msgs['controlsState'].controlsState.alertStatus = log.ControlsState.AlertStatus.userPrompt
        #msgs['controlsState'].controlsState.alertSound = log.ControlsState.AlertSound.none  # << silent!
      else:
        msgs['controlsState'].controlsState.alertText1 = ""
        msgs['controlsState'].controlsState.alertText2 = ""
        msgs['controlsState'].controlsState.alertSize = log.ControlsState.AlertSize.none
        msgs['controlsState'].controlsState.alertStatus = log.ControlsState.AlertStatus.normal
        #msgs['controlsState'].controlsState.alertSound = log.ControlsState.AlertSound.none  # << silent!

      time.sleep(1 / 100)  # continually send, rate doesn't matter
      for s in msgs:
        pm.send(s, msgs[s])
  except KeyboardInterrupt:
    for p in procs:
      managed_processes[p].stop()
