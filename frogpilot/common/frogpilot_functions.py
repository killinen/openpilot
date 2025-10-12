#!/usr/bin/env python3
import filecmp
import threading
import time

from pathlib import Path

from openpilot.common.time_helpers import system_time_valid
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.common.frogpilot_utilities import run_cmd

def frogpilot_boot_functions():
  def boot_thread():
    while not system_time_valid():
      print("Waiting for system time to become valid...")
      time.sleep(1)

  threading.Thread(target=boot_thread, daemon=True).start()

def install_frogpilot():
  paths = [
  ]
  for path in paths:
    path.mkdir(parents=True, exist_ok=True)

  boot_logo_location = Path("/usr/comma/bg.jpg")
  frogpilot_boot_logo = Path(__file__).resolve().parents[1] / "assets/other_images/frogpilot_boot_logo.jpg"

  if not filecmp.cmp(frogpilot_boot_logo, boot_logo_location, shallow=False):
    mount_options = run_cmd(['findmnt', '-n', '-o', 'OPTIONS', '/'], "Successfully retrieved mount options", "Failed to retrieve mount options")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount /")
    run_cmd(["sudo", "cp", frogpilot_boot_logo, boot_logo_location], "Successfully replaced boot logo", "Failed to replace boot logo")
    run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/"], "Successfully restored / mount options", "Failed to restore / mount options")


def uninstall_frogpilot():
  boot_logo_location = Path("/usr/comma/bg.jpg")
  stock_boot_logo = Path(__file__).resolve().parents[1] / "assets/other_images/stock_bg.jpg"

  mount_options = run_cmd(['findmnt', '-n', '-o', 'OPTIONS', '/'], "Successfully retrieved mount options", "Failed to retrieve mount options")
  run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount /")
  run_cmd(["sudo", "cp", stock_boot_logo, boot_logo_location], "Successfully restored boot logo", "Failed to restored boot logo")
  run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/"], "Successfully restored / mount options", "Failed to restore / mount options")

  HARDWARE.uninstall()
