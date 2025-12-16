#!/usr/bin/env python3
import os
import time
import json
from zipfile import ZipFile, ZIP_DEFLATED
import subprocess
import requests
from datetime import datetime, timezone
from typing import Callable, List, Optional, cast

from tools.teletyped.helper import (
  log,
  get_dongle_id,
  API_URL,
  WORMHOLE_BINARY,
  SENDER_LOG,
  CHECK_INTERVAL,
  REALDATA_DIR,
  BOOT_DIR,
  has_internet_connection,
  build_auth_headers,
  capture_exception,
)

TIMEOUT = 5
ZIP_EPOCH = datetime(1980, 1, 1).timestamp()
RETRY_LIMIT = 2

DRIVE_SCAN_REQUEST_PATH = f"{API_URL}/drive-scan-requests"
DRIVE_INVENTORY_UPLOAD_PATH = f"{API_URL}/drive-inventory"
DRIVE_TRANSFER_LIST_PATH = f"{API_URL}/drive-transfers"
DRIVE_TRANSFER_UPDATE_PATH = f"{API_URL}/update-drive-transfer"

_auth_wait_logged = False


def _auth_headers():
  return build_auth_headers()


def get_pending_drive_transfers(device_id):
  headers = _auth_headers()
  if not headers:
    return []
  try:
    res = requests.get(f"{DRIVE_TRANSFER_LIST_PATH}/{device_id}", headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    transfers = res.json()
    return [t for t in transfers if t.get("status") in ("queued", "retry")]
  except Exception as e:
    capture_exception(e)
    log(f"Failed to fetch drive transfers: {e}", "ERROR")
    return []


def update_drive_transfer(device_id, drive_name, status=None, **extra):
  headers = _auth_headers()
  if not headers:
    return
  payload = {
    "device_id": device_id,
    "drive_name": drive_name,
  }
  if status:
    payload["status"] = status
  payload.update({k: v for k, v in extra.items() if v is not None})
  try:
    res = requests.post(DRIVE_TRANSFER_UPDATE_PATH, json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    log(f"Updated drive transfer: {drive_name} -> {status or 'unchanged'}")
  except Exception as e:
    capture_exception(e)
    log(f"Failed to update drive transfer for {drive_name}: {e}", "ERROR")


def should_include_file(file_name: str, requested: Optional[List[str]]) -> bool:
  if not requested:
    return True
  return file_name in requested

def send_wormhole_code(code, zip_path, filename, timestamp, device_id, drive_name=None, requested_files=None):
  headers = _auth_headers()
  if not headers:
    return False
  included_files = []
  if zip_path.endswith(".zip") and os.path.exists(zip_path):
    try:
      with ZipFile(zip_path, "r") as zipf:
        included_files = zipf.namelist()
    except Exception as e:
      capture_exception(e)
      log(f"Could not read zip contents: {e}", "WARN")

  payload = {
    "device_id": device_id,
    "wormhole_code": code,
    "filename": filename,
    "timestamp": timestamp,
    "included_files": included_files,
  }
  if drive_name:
    payload["drive_name"] = drive_name
  if requested_files:
    payload["requested_files"] = requested_files
  res = requests.post(f"{API_URL}/birdie", json=payload, headers=headers, timeout=TIMEOUT)
  res.raise_for_status()
  log("Wormhole code registered.")
  return True

def collect_drive_inventory():
  drives = []
  total_size = 0

  if not os.path.exists(REALDATA_DIR):
    return drives, total_size

  try:
    entries = sorted(os.listdir(REALDATA_DIR))
  except OSError as e:
    capture_exception(e)
    log(f"Failed to list realdata directory: {e}", "ERROR")
    return drives, total_size

  for entry in entries:
    entry_path = os.path.join(REALDATA_DIR, entry)
    if not os.path.isdir(entry_path):
      continue

    try:
      stat = os.stat(entry_path)
    except OSError as e:
      capture_exception(e)
      log(f"stat failed for {entry}: {e}", "WARN")
      continue

    size_bytes = 0
    file_count = 0
    filenames = []  # paths relative to this entry

    for root, _, files in os.walk(entry_path):
      rel_root = os.path.relpath(root, entry_path)
      for name in files:
        file_path = os.path.join(root, name)
        try:
          size_bytes += os.path.getsize(file_path)
          file_count += 1
          # store path relative to the drive folder
          rel_path = name if rel_root == "." else os.path.normpath(os.path.join(rel_root, name))
          filenames.append(rel_path)
        except OSError:
          continue

    drives.append({
      "name": entry,
      "size_bytes": size_bytes,
      "file_count": file_count,
      "files": sorted(filenames),
      "modified_at": datetime.fromtimestamp(stat.st_mtime, timezone.utc).isoformat(),
    })
    total_size += size_bytes

  return drives, total_size

def drive_inventory_step(device_id):
  headers = _auth_headers()
  if not headers:
    return

  try:
    response = requests.get(f"{DRIVE_SCAN_REQUEST_PATH}/{device_id}", headers=headers, timeout=TIMEOUT)
    if response.status_code == 404:
      return
    response.raise_for_status()
    request_info = response.json()
  except Exception as e:
    capture_exception(e)
    log(f"Failed to check drive scan requests: {e}", "WARN")
    return

  if not request_info.get("pending"):
    return

  drives, total_size = collect_drive_inventory()
  payload = {
    "device_id": device_id,
    "drives": drives,
    "total_size_bytes": total_size,
    "scanned_at": datetime.now(timezone.utc).isoformat(),
  }

  try:
    response = requests.post(
      DRIVE_INVENTORY_UPLOAD_PATH,
      json=payload,
      headers=headers,
      timeout=max(TIMEOUT, 30),
    )
    response.raise_for_status()
    log(f"📊 Reported {len(drives)} drive(s) to server.")
  except Exception as e:
    capture_exception(e)
    log(f"Failed to upload drive inventory: {e}", "ERROR")

def log_local_send(device_id, filename, code, timestamp, drive_name=None):
  entry = {
    "device_id": device_id,
    "filename": filename,
    "wormhole_code": code,
    "status": "sent",
    "timestamp": timestamp,
    "logged_at": datetime.now(timezone.utc).isoformat()
  }
  if drive_name:
    entry["drive_name"] = drive_name

  log_data = []
  if os.path.exists(SENDER_LOG):
    try:
      with open(SENDER_LOG, "r") as f:
        log_data = json.load(f)
    except json.JSONDecodeError:
      log("Sender log corrupted. Overwriting.", "WARN")

  log_data.append(entry)
  with open(SENDER_LOG, "w") as f:
    json.dump(log_data, f, indent=2)

  log(f"Logged transfer: {filename}")

def send_file_wormhole(zip_path, device_id, drive_name, requested_files):
  timestamp = datetime.now(timezone.utc).isoformat()

  for attempt in range(1, RETRY_LIMIT + 1):
    try:
      wormhole_sent = False
      code = None
      zip_path_abs = os.path.abspath(zip_path)

      proc = subprocess.Popen(
        [WORMHOLE_BINARY, "send", zip_path_abs],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
      )

      for line in proc.stdout:
        if "Wormhole code is:" in line:
          code = line.split("Wormhole code is:")[1].strip()
          log(f"Wormhole code: {code}")
          worked = send_wormhole_code(
            code,
            zip_path_abs,
            os.path.basename(zip_path_abs),
            timestamp,
            device_id,
            drive_name=drive_name,
            requested_files=requested_files,
          )
          if worked:
            wormhole_sent = True
          else:
            log("⚠️ Wormhole registration skipped (missing API token)", "WARN")
          break

      proc.wait()
      if proc.returncode != 0:
        raise RuntimeError(f"wormhole exited with code {proc.returncode}")

      if wormhole_sent:
        log(f"✅ File sent via wormhole: {zip_path}")
        log_local_send(device_id, os.path.basename(zip_path_abs), code, timestamp, drive_name=drive_name)
        return True
      else:
        log("⚠️ Wormhole code not found in output", "WARN")

    except Exception as e:
      capture_exception(e)
      log(f"Attempt {attempt} failed: {e}", "ERROR")
      time.sleep(2)

  return False

def route_sender_step(device_id):
  if not has_internet_connection():
    log("No internet connection. Skipping route sender step.", "WARN")
    return

  transfers = get_pending_drive_transfers(device_id)
  log(f"🔄 Drive sender tick - {len(transfers)} pending transfer(s)")

  for transfer in transfers:
    drive_name = transfer.get("drive_name") or transfer.get("name")
    if not drive_name:
      log("Transfer entry missing drive name; skipping", "WARN")
      continue

    is_boot = drive_name.startswith("boot_")
    base_name = drive_name[5:] if is_boot else drive_name
    requested_files = transfer.get("requested_files")
    if requested_files is not None and not isinstance(requested_files, list):
      requested_files = None

    if is_boot:
      route_path = os.path.join(BOOT_DIR, base_name)
      if not os.path.exists(route_path):
        log(f"Missing boot file: {drive_name}", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="boot file missing")
        continue

      if requested_files and not should_include_file(base_name, requested_files):
        log("Boot file not requested; skipping transfer", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="boot file not requested")
        continue

      zip_path = f"{drive_name}.zip"
      try:
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          arcname = os.path.join("boot", base_name)
          zipf.write(route_path, arcname=arcname)
        log(f"📦 Zipped boot file into folder: boot/{base_name}")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for boot file {base_name}: {e}", "ERROR")
        update_drive_transfer(device_id, drive_name, status="error", error=str(e))
        continue

    else:
      try:
        entries = os.listdir(REALDATA_DIR)
      except OSError as e:
        capture_exception(e)
        log(f"Failed to list realdata: {e}", "ERROR")
        update_drive_transfer(device_id, drive_name, status="error", error=str(e))
        continue

      segments = []
      legacy_prefix = base_name + "--"
      for d in entries:
        if d == base_name or d.startswith(legacy_prefix):
          segments.append(os.path.join(REALDATA_DIR, d))

      if not segments:
        log(f"Missing file(s) for prefix: {base_name}", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="segments missing")
        continue

      zip_path = f"/tmp/{base_name}.zip"
      try:
        missing_files = []
        added_files = 0
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          for segment in segments:
            for root, _, files in os.walk(segment):
              for file in files:
                if not should_include_file(file, requested_files):
                  continue
                abs_path = os.path.join(root, file)
                if not os.path.isfile(abs_path):
                  missing_files.append(abs_path)
                  continue
                rel_path = os.path.relpath(abs_path, REALDATA_DIR)
                arcname = os.path.join(base_name, rel_path)
                try:
                  zipf.write(abs_path, arcname=arcname)
                  added_files += 1
                except FileNotFoundError as e:
                  capture_exception(e)
                  missing_files.append(abs_path)
                  continue
        if missing_files:
          log(f"⚠️ Skipped {len(missing_files)} missing file(s) while zipping {base_name}", "WARN")
        if added_files == 0:
          log(f"No files matched requested list for {base_name}", "WARN")
          update_drive_transfer(device_id, drive_name, status="error", error="no files match request")
          continue
        log(f"📦 Zipped {len(segments)} segment(s) into folder: {base_name} (files={added_files})")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for {base_name}: {e}", "ERROR")
        update_drive_transfer(device_id, drive_name, status="error", error=str(e))
        continue

    update_drive_transfer(device_id, drive_name, status="sending")

    if send_file_wormhole(zip_path, device_id, drive_name, requested_files):
      update_drive_transfer(device_id, drive_name, status="sent")
    else:
      update_drive_transfer(device_id, drive_name, status="error", error="wormhole send failed")

    try:
      if os.path.exists(zip_path):
        os.remove(zip_path)
        log(f"🧹 Deleted temporary zip: {zip_path}")
    except Exception as e:
      capture_exception(e)
      log(f"⚠️ Failed to delete zip: {e}", "WARN")

def run_route_sender(stop_event=None, device_id=None):
  global _auth_wait_logged
  device_id = device_id or get_dongle_id()
  if not device_id or device_id == "UNKNOWN_DEVICE":
    log("❌ Route sender missing device ID; exiting", "ERROR")
    return

  log(f"🚚 Route sender loop started for {device_id}")

  wait_fn: Callable[[float], bool]
  is_set_fn: Callable[[], bool]
  def _wait_default(_: float) -> bool:
    return False

  def _is_set_default() -> bool:
    return False

  wait_fn = _wait_default
  is_set_fn = _is_set_default
  if stop_event is not None:
    wait_candidate = getattr(stop_event, "wait", None)
    if callable(wait_candidate):
      wait_fn = cast(Callable[[float], bool], wait_candidate)
    is_set_candidate = getattr(stop_event, "is_set", None)
    if callable(is_set_candidate):
      is_set_fn = cast(Callable[[], bool], is_set_candidate)

  try:
    while True:
      if is_set_fn():
        break

      headers = build_auth_headers()
      if not headers:
        if not _auth_wait_logged:
          log("⚠️ Route sender waiting for device auth", "WARN")
          _auth_wait_logged = True
        if wait_fn(CHECK_INTERVAL):
          break
        time.sleep(CHECK_INTERVAL)
        continue
      elif _auth_wait_logged:
        log("✅ Device auth detected; route sender active")
        _auth_wait_logged = False

      try:
        route_sender_step(device_id)
      except Exception as e:
        capture_exception(e)
        log(f"❌ route_sender_step() failed: {e}", "ERROR")

      try:
        drive_inventory_step(device_id)
      except Exception as e:
        capture_exception(e)
        log(f"❌ drive_inventory_step() failed: {e}", "ERROR")

      if wait_fn(CHECK_INTERVAL):
        break
      else:
        time.sleep(CHECK_INTERVAL)
  finally:
    log("🚚 Route sender loop stopped", "INFO")

def main():
  run_route_sender()

if __name__ == "__main__":
  main()
