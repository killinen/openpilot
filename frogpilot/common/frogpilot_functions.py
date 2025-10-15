#!/usr/bin/env python3
import datetime
import filecmp
import random
import shutil
import string
import tarfile
import threading
import time
import zstandard as zstd

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common.frogpilot_utilities import delete_file, run_cmd
from openpilot.frogpilot.common.frogpilot_variables import (
  EXCLUDED_KEYS,
  THEME_SAVE_PATH, FrogPilotVariables, get_frogpilot_toggles
)
from openpilot.frogpilot.system.frogpilot_stats import send_stats

def backup_directory(backup, destination, success_message, fail_message, params, minimum_backup_size=0, compressed=False):
  in_progress_destination = destination.parent / f"{destination.name}_in_progress"
  final_destination = destination.parent / f"{destination.name}.tar.zst" if compressed else destination

  if final_destination.exists():
    print("Backup already exists. Aborting...")
    delete_file(in_progress_destination, report=False)
    return False

  in_progress_destination.mkdir(parents=True, exist_ok=True)
  run_cmd(["sudo", "rsync", "-avq", f"{backup}/.", str(in_progress_destination)], "" if compressed else success_message, fail_message, report=False)

  if compressed:
    compressed_file = destination.parent / f"{destination.name}_in_progress.tar.zst"
    tar_file = destination.parent / f"{destination.name}_in_progress.tar"

    with tarfile.open(tar_file, "w") as tar:
      tar.add(in_progress_destination, arcname=destination.name)

    delete_file(in_progress_destination, report=False)

    with open(compressed_file, "wb") as f_out:
      cctx = zstd.ZstdCompressor(level=2)
      with open(tar_file, "rb") as f_in:
        with cctx.stream_writer(f_out) as compressor:
          while chunk := f_in.read(65536):
            compressor.write(chunk)

    tar_file.unlink(missing_ok=True)
    compressed_file.rename(final_destination)
    print(f"Backup saved: {final_destination}")

    compressed_backup_size = final_destination.stat().st_size
    if minimum_backup_size == 0 or compressed_backup_size < minimum_backup_size:
      params.put("MinimumBackupSize", compressed_backup_size)
  else:
    in_progress_destination.rename(destination)


def cleanup_backups(directory, limit, compressed=False):
  directory.mkdir(parents=True, exist_ok=True)

  for in_progress in directory.glob("*_in_progress*"):
    delete_file(in_progress, report=False)

  backups = sorted(directory.glob("*_auto*"), key=lambda x: x.stat().st_mtime, reverse=True)
  for oldest_backup in backups[limit:]:
    delete_file(oldest_backup, report=False)


def backup_frogpilot(build_metadata, params):
  backup_path = Path("/data/backups")
  maximum_backups = 3
  cleanup_backups(backup_path, maximum_backups, compressed=True)

  _, _, free = shutil.disk_usage(backup_path)
  minimum_backup_size = params.get("MinimumBackupSize")
  if free > minimum_backup_size * maximum_backups:
    destination_directory = backup_path / f"{build_metadata.channel}_{build_metadata.openpilot.git_commit_date[12:-16]}_auto"
    backup_directory(Path(BASEDIR), destination_directory, f"Successfully backed up FrogPilot to {destination_directory}", f"Failed to backup FrogPilot to {destination_directory}", params, minimum_backup_size, compressed=True)


def backup_toggles(params, params_cache):
  params_backup = Params("/data/params_backup")

  changes_found = False
  for key in params.all_keys():
    current_value = params_backup.get(key)
    new_value = params.get(key)

    if new_value != current_value:
      if new_value is not None:
        params_backup.put(key, new_value)
        params_cache.put(key, new_value)
      changes_found |= key not in EXCLUDED_KEYS

  backup_path = Path("/data/toggle_backups")
  maximum_backups = 5
  cleanup_backups(backup_path, maximum_backups)

  if not changes_found and list(backup_path.glob("*")):
    print("Toggles are identical to the previous backup. Aborting...")
    return

  destination_directory = backup_path / f"{datetime.datetime.now().strftime('%Y-%m-%d_%I-%M%p').lower()}_auto"
  backup_directory(Path(params_backup.get_param_path()), destination_directory, f"Successfully backed up toggles to {destination_directory}", f"Failed to backup toggles to {destination_directory}", params)


def frogpilot_boot_functions(build_metadata, params, params_cache):
  params_memory = Params(memory=True)

  FrogPilotVariables().update(started=False)
  ThemeManager(params, params_memory, boot_run=True).update_active_theme(frogpilot_toggles=get_frogpilot_toggles(), boot_run=True)

  if params.get("FrogPilotDongleId") == None:
    params.put("FrogPilotDongleId", ''.join(random.choices(string.ascii_lowercase + string.digits, k=16)))

  def boot_thread():
    while not system_time_valid():
      print("Waiting for system time to become valid...")
      time.sleep(1)

    backup_frogpilot(build_metadata, params)
    backup_toggles(params, params_cache)

    send_stats(params)

  threading.Thread(target=boot_thread, daemon=True).start()


def install_frogpilot(build_metadata):
  paths = [
    THEME_SAVE_PATH
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

  if build_metadata.channel == "FrogPilot-Development" and Path("/persist/frogsgomoo.py").is_file():
    mount_options = run_cmd(['findmnt', '-n', '-o', 'OPTIONS', '/persist'], "Successfully retrieved mount options", "Failed to retrieve mount options")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/persist"], "Successfully remounted /persist as read-write", "Failed to remount /persist")
    run_cmd(["sudo", "python3", "/persist/frogsgomoo.py"], "Ran frogsgomoo.py", "Failed to run frogsgomoo.py")
    run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/persist"], "Successfully restored /persist mount options", "Failed to restore /persist mount options")


def uninstall_frogpilot():
  boot_logo_location = Path("/usr/comma/bg.jpg")
  stock_boot_logo = Path(__file__).resolve().parents[1] / "assets/other_images/stock_bg.jpg"

  mount_options = run_cmd(['findmnt', '-n', '-o', 'OPTIONS', '/'], "Successfully retrieved mount options", "Failed to retrieve mount options")
  run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount /")
  run_cmd(["sudo", "cp", stock_boot_logo, boot_logo_location], "Successfully restored boot logo", "Failed to restored boot logo")
  run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/"], "Successfully restored / mount options", "Failed to restore / mount options")

  HARDWARE.uninstall()
