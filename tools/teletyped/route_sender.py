#!/usr/bin/env python3
import os
import time
import json
from zipfile import ZipFile, ZIP_DEFLATED
import subprocess
import bz2
from collections.abc import Callable
from datetime import datetime, UTC
from typing import cast
import shutil

from openpilot.tools.teletyped.helper import (
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
  http_get,
  http_post,
  capture_exception,
)
from openpilot.tools.teletyped.label_utils import (
  is_drive_label,
  strip_boot_prefix,
)

TIMEOUT = 5
ZIP_EPOCH = datetime(1980, 1, 1).timestamp()
RETRY_LIMIT = 2

DRIVE_SCAN_REQUEST_PATH = f"{API_URL}/drive-scan-requests"
DRIVE_INVENTORY_UPLOAD_PATH = f"{API_URL}/drive-inventory"
DRIVE_TRANSFER_LIST_PATH = f"{API_URL}/drive-transfers"
DRIVE_TRANSFER_UPDATE_PATH = f"{API_URL}/update-drive-transfer"
AUTO_DRIVE_INVENTORY = os.environ.get("TELETYPED_AUTO_DRIVE_INVENTORY", "1").strip().lower() not in {
  "0",
  "false",
  "no",
  "off",
}

_auth_wait_logged = False
COMPRESSIBLE_BASENAMES = {"qlog", "rlog"}


def _safe_zip_name(name: str) -> str:
  return name.replace("/", "_").replace("\\", "_")


def _pick_temp_dir(min_bytes: int = 150 * 1024 * 1024) -> str:
  """Choose a temp directory with available space; fall back to /tmp."""
  candidates = ["/data/tmp", "/data/media/0/tmp", "/data", "/tmp"]
  for candidate in candidates:
    try:
      os.makedirs(candidate, exist_ok=True)
      usage = shutil.disk_usage(candidate)
      if usage.free >= min_bytes:
        return candidate
    except OSError:
      continue
  return "/tmp"


def _find_boot_file(base_name: str) -> str | None:
  """
  Locate a boot file in BOOT_DIR given a base name (with or without extension).
  """
  if not os.path.isdir(BOOT_DIR):
    return None

  safe_base = os.path.basename(base_name.strip().replace("\\", "/"))
  exact_path = os.path.join(BOOT_DIR, safe_base)
  if os.path.isfile(exact_path):
    return safe_base

  base_no_ext = safe_base.rsplit(".", 1)[0]
  try:
    for fname in os.listdir(BOOT_DIR):
      candidate_base = fname.rsplit(".", 1)[0]
      if candidate_base == base_no_ext:
        return fname
  except OSError as e:
    capture_exception(e)
    log(f"Failed to list boot dir: {e}", "WARN")

  return None


def _boot_file_requested(filename: str, requested_files: list[str] | None) -> bool:
  if not requested_files:
    return True
  base_no_ext = filename.rsplit(".", 1)[0]
  return filename in requested_files or base_no_ext in requested_files


def _maybe_compress_for_zip(src_path: str, rel_path: str, temp_dir: str, cleanup: list[str]) -> tuple[str, str]:
  """
  For rlog/qlog, create a .bz2 copy in temp_dir and return (path_to_zip, arcname_relative_to_drive).
  For all other files, return the original path and rel_path unchanged.
  """
  base = os.path.basename(rel_path)
  if base in COMPRESSIBLE_BASENAMES:
    dest_base = base + ".bz2"
    dest_path = os.path.join(temp_dir, dest_base)
    try:
      with open(src_path, "rb") as fin, bz2.open(dest_path, "wb") as fout:
        shutil.copyfileobj(fin, fout)
      cleanup.append(dest_path)
      rel_dir = os.path.dirname(rel_path)
      arc_rel = os.path.join(rel_dir, dest_base) if rel_dir not in ("", ".") else dest_base
      return dest_path, arc_rel
    except Exception as e:
      capture_exception(e)
      log(f"❌ Failed to compress {rel_path}: {e}", "ERROR")
      raise

  return src_path, rel_path


def _auth_headers():
  headers = build_auth_headers()
  if headers.get("X-Device-JWT"):
    return headers
  return None


def get_pending_drive_transfers(device_id):
  headers = _auth_headers()
  if not headers:
    return []
  try:
    res = http_get(f"{DRIVE_TRANSFER_LIST_PATH}/{device_id}", headers=headers, timeout=TIMEOUT)
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
    res = http_post(DRIVE_TRANSFER_UPDATE_PATH, json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    log(f"Updated drive transfer: {drive_name} -> {status or 'unchanged'}")
  except Exception as e:
    capture_exception(e)
    log(f"Failed to update drive transfer for {drive_name}: {e}", "ERROR")


def should_include_file(file_name: str, requested: list[str] | None) -> bool:
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
  res = http_post(f"{API_URL}/birdie", json=payload, headers=headers, timeout=TIMEOUT)
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
    if entry == "boot":
      continue
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
      "modified_at": datetime.fromtimestamp(stat.st_mtime, UTC).isoformat(),
    })
    total_size += size_bytes

  if os.path.isdir(BOOT_DIR):
    try:
      boot_entries = sorted(os.listdir(BOOT_DIR))
    except OSError as e:
      capture_exception(e)
      log(f"Failed to list boot directory: {e}", "WARN")
      boot_entries = []

    boot_files = []
    boot_size = 0
    latest_mtime = None
    for fname in boot_entries:
      file_path = os.path.join(BOOT_DIR, fname)
      if not os.path.isfile(file_path):
        continue
      base_no_ext = fname.rsplit(".", 1)[0]
      if not is_drive_label(base_no_ext):
        continue
      try:
        stat = os.stat(file_path)
      except OSError as e:
        capture_exception(e)
        log(f"stat failed for boot file {fname}: {e}", "WARN")
        continue
      boot_files.append(fname)
      boot_size += stat.st_size
      if latest_mtime is None or stat.st_mtime > latest_mtime:
        latest_mtime = stat.st_mtime

    if boot_files:
      drives.append({
        "name": "boot",
        "size_bytes": boot_size,
        "file_count": len(boot_files),
        "files": sorted(boot_files),
        "modified_at": datetime.fromtimestamp(latest_mtime, UTC).isoformat()
        if latest_mtime is not None
        else datetime.now(UTC).isoformat(),
      })
      total_size += boot_size

  return drives, total_size

def drive_inventory_step(device_id):
  headers = _auth_headers()
  if not headers:
    return

  try:
    response = http_get(f"{DRIVE_SCAN_REQUEST_PATH}/{device_id}", headers=headers, timeout=TIMEOUT)
    if response.status_code == 404:
      request_info = {}
    else:
      response.raise_for_status()
      request_info = response.json()
  except Exception as e:
    capture_exception(e)
    log(f"Failed to check drive scan requests: {e}", "WARN")
    if not AUTO_DRIVE_INVENTORY:
      return
    request_info = {}

  if not request_info.get("pending") and not AUTO_DRIVE_INVENTORY:
    return

  drives, total_size = collect_drive_inventory()
  payload = {
    "device_id": device_id,
    "drives": drives,
    "total_size_bytes": total_size,
    "scanned_at": datetime.now(UTC).isoformat(),
  }

  try:
    response = http_post(
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
    "logged_at": datetime.now(UTC).isoformat()
  }
  if drive_name:
    entry["drive_name"] = drive_name

  log_data = []
  if os.path.exists(SENDER_LOG):
    try:
      with open(SENDER_LOG) as f:
        log_data = json.load(f)
    except json.JSONDecodeError:
      log("Sender log corrupted. Overwriting.", "WARN")

  log_data.append(entry)
  with open(SENDER_LOG, "w") as f:
    json.dump(log_data, f, indent=2)

  log(f"Logged transfer: {filename}")

def send_file_wormhole(zip_path, device_id, drive_name, requested_files):
  timestamp = datetime.now(UTC).isoformat()

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
            log("⚠️ Wormhole registration skipped (missing auth headers)", "WARN")
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

    normalized_drive = str(drive_name).replace("\\", "/")
    boot_base = None
    if normalized_drive == "boot":
      boot_base = ""
    elif normalized_drive.startswith(("boot_", "boot-", "boot/")):
      boot_base = strip_boot_prefix(normalized_drive)

    base_name = os.path.basename(strip_boot_prefix(normalized_drive) if boot_base else normalized_drive)
    requested_files = transfer.get("requested_files")
    if requested_files is not None and not isinstance(requested_files, list):
      requested_files = None

    if boot_base is not None:
      if not os.path.isdir(BOOT_DIR):
        log("Boot directory missing; cannot send boot files", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="boot directory missing")
        continue

      try:
        boot_entries = sorted(os.listdir(BOOT_DIR))
      except OSError as e:
        capture_exception(e)
        log(f"Failed to list boot directory: {e}", "ERROR")
        update_drive_transfer(device_id, drive_name, status="error", error="boot dir unreadable")
        continue

      boot_files = []
      for fname in boot_entries:
        file_path = os.path.join(BOOT_DIR, fname)
        if not os.path.isfile(file_path):
          continue
        base_no_ext = fname.rsplit(".", 1)[0]
        if not is_drive_label(base_no_ext):
          continue
        if boot_base and base_no_ext != boot_base:
          continue
        if not _boot_file_requested(fname, requested_files):
          continue
        boot_files.append(fname)

      if not boot_files:
        log("No boot files match the requested set", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="boot file not requested")
        continue

      zip_label = boot_base if boot_base else "boot"
      zip_path = os.path.join(_pick_temp_dir(), f"{_safe_zip_name(zip_label)}.zip")
      try:
        added_files = 0
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          for fname in boot_files:
            route_path = os.path.join(BOOT_DIR, fname)
            arcname = os.path.join("boot", fname)
            zipf.write(route_path, arcname=arcname)
            added_files += 1
        if added_files == 0:
          log("No boot files zipped (empty selection)", "WARN")
          update_drive_transfer(device_id, drive_name, status="error", error="boot file missing")
          continue
        log(f"📦 Zipped boot files ({added_files}) into folder: boot/")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for boot files {boot_files}: {e}", "ERROR")
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
      base_name = os.path.basename(base_name)
      legacy_prefix = base_name + "--"
      for d in entries:
        if d == base_name or d.startswith(legacy_prefix):
          segments.append(os.path.join(REALDATA_DIR, d))

      if not segments:
        log(f"Missing file(s) for prefix: {base_name}", "WARN")
        update_drive_transfer(device_id, drive_name, status="error", error="segments missing")
        continue

      temp_dir = _pick_temp_dir()
      zip_path = os.path.join(temp_dir, f"{_safe_zip_name(base_name)}.zip")
      temp_cleanup: list[str] = []
      try:
        missing_files = []
        added_files = 0
        update_drive_transfer(device_id, drive_name, status="sending", stage="compressing")
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
                try:
                  src_path, rel_for_zip = _maybe_compress_for_zip(abs_path, rel_path, temp_dir, temp_cleanup)
                except Exception:
                  missing_files.append(abs_path)
                  continue
                arcname = os.path.join(base_name, rel_for_zip)
                try:
                  zipf.write(src_path, arcname=arcname)
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
        update_drive_transfer(device_id, drive_name, status="sending", stage="zipped")
        log(f"📦 Zipped {len(segments)} segment(s) into folder: {base_name} (files={added_files})")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for {base_name}: {e}", "ERROR")
        update_drive_transfer(device_id, drive_name, status="error", error=str(e))
        continue
      finally:
        for temp_file in temp_cleanup:
          try:
            os.remove(temp_file)
          except OSError:
            pass

    update_drive_transfer(device_id, drive_name, status="sending", stage="wormhole")

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

      if not _auth_headers():
        if not _auth_wait_logged:
          log("⚠️ Route sender waiting for device JWT (registration key missing?)", "WARN")
          _auth_wait_logged = True
        if wait_fn(CHECK_INTERVAL):
          break
        else:
          time.sleep(CHECK_INTERVAL)
        continue
      elif _auth_wait_logged:
        log("✅ Device JWT available; route sender active")
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
