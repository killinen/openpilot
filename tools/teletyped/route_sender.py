#!/usr/bin/env python3
import os
import time
import json
import re
from zipfile import ZipFile, ZIP_DEFLATED
import subprocess
import bz2
from collections.abc import Callable
from pathlib import Path
from datetime import datetime, UTC
from typing import cast
import shutil

from openpilot.tools.teletyped.helper import (
  log,
  get_dongle_id,
  get_op_params_info,
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

OPENPILOT_BASEDIR: str | None
try:
  from openpilot.common.basedir import BASEDIR as OPENPILOT_BASEDIR
except Exception:  # pragma: no cover - runtime fallback for unusual packaging
  OPENPILOT_BASEDIR = None

TIMEOUT = 5
ZIP_EPOCH = datetime(1980, 1, 1).timestamp()
RETRY_LIMIT = 2
ZIP_PROGRESS_CHUNK_SIZE = 1024 * 1024
ZIP_PROGRESS_REPORT_INTERVAL_SECONDS = 1.0

DRIVE_SCAN_REQUEST_PATH = f"{API_URL}/drive-scan-requests"
DRIVE_SCAN_STATUS_PATH = f"{API_URL}/drive-scan-requests"
DRIVE_INVENTORY_UPLOAD_PATH = f"{API_URL}/drive-inventory"
DRIVE_TRANSFER_LIST_PATH = f"{API_URL}/drive-transfers"
DRIVE_TRANSFER_UPDATE_PATH = f"{API_URL}/update-drive-transfer"
RLOG_UPLOAD_PATH = f"{API_URL}/rlogs/upload"
SCHEMA_BUNDLE_UPLOAD_PATH = f"{API_URL}/rlogs/schema-bundles/upload"
AUTO_DRIVE_INVENTORY = os.environ.get("TELETYPED_AUTO_DRIVE_INVENTORY", "1").strip().lower() not in {
  "0",
  "false",
  "no",
  "off",
}

_auth_wait_logged = False
COMPRESSIBLE_BASENAMES = {"qlog", "rlog"}
CAPNP_IMPORT_RE = re.compile(
  r'^\s*using(?:\s+[A-Za-z_][A-Za-z0-9_]*\s*=\s*)?\s+import\s+"([^"]+)"'
)
SCHEMA_FETCH_REQUEST_FILE = "__schema_bundle__"
SCHEMA_FETCH_DRIVE_PREFIX = "__schema__:"


def _safe_zip_name(name: str) -> str:
  return name.replace("/", "_").replace("\\", "_")


def _human_readable_bytes(num_bytes: int | float) -> str:
  try:
    size = float(num_bytes)
  except (TypeError, ValueError):
    return "?"
  if size < 0:
    return "?"
  if size == 0:
    return "0 B"
  units = ["B", "KB", "MB", "GB", "TB", "PB"]
  unit_index = 0
  while size >= 1024 and unit_index < len(units) - 1:
    size /= 1024
    unit_index += 1
  return f"{size:.1f} {units[unit_index]}"


class ZipProgressReporter:
  def __init__(
    self,
    device_id: str,
    drive_name: str,
    *,
    stage: str,
    label: str,
    total_files: int,
    total_bytes: int,
    start_percent: int = 0,
    end_percent: int = 100,
  ) -> None:
    self.device_id = device_id
    self.drive_name = drive_name
    self.stage = stage
    self.label = label
    self.total_files = max(0, total_files)
    self.total_bytes = max(0, total_bytes)
    self.start_percent = start_percent
    self.end_percent = max(start_percent, end_percent)
    self.processed_files = 0
    self.processed_bytes = 0
    self._last_percent: int | None = None
    self._last_report_at = 0.0

  def advance_bytes(self, num_bytes: int) -> None:
    if num_bytes > 0:
      self.processed_bytes += num_bytes
    self.report()

  def file_completed(self) -> None:
    self.processed_files += 1
    self.report(force=True)

  def report(self, force: bool = False) -> None:
    if self.total_bytes > 0:
      ratio = min(1.0, self.processed_bytes / self.total_bytes)
    elif self.total_files > 0:
      ratio = min(1.0, self.processed_files / self.total_files)
    else:
      ratio = 1.0
    progress_percent = int(round(self.start_percent + (self.end_percent - self.start_percent) * ratio))
    now = time.monotonic()
    if not force and self._last_percent == progress_percent and (now - self._last_report_at) < ZIP_PROGRESS_REPORT_INTERVAL_SECONDS:
      return
    if self.total_bytes > 0:
      detail = (
        f"{self.label}: {min(self.processed_files, self.total_files)}/{self.total_files} files, "
        + f"{_human_readable_bytes(min(self.processed_bytes, self.total_bytes))} / {_human_readable_bytes(self.total_bytes)}"
      )
    else:
      detail = f"{self.label}: {min(self.processed_files, self.total_files)}/{self.total_files} files"
    report_transfer_progress(
      self.device_id,
      self.drive_name,
      stage=self.stage,
      progress_percent=progress_percent,
      detail=detail,
    )
    self._last_percent = progress_percent
    self._last_report_at = now


def _copy_path_into_zip(
  zipf: ZipFile,
  src_path: str,
  arcname: str,
  *,
  progress_cb: Callable[[int], None] | None = None,
) -> None:
  with open(src_path, "rb") as src, zipf.open(arcname, "w") as dest:
    while True:
      chunk = src.read(ZIP_PROGRESS_CHUNK_SIZE)
      if not chunk:
        break
      dest.write(chunk)
      if progress_cb is not None:
        progress_cb(len(chunk))


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


def _maybe_compress_for_zip(
  src_path: str,
  rel_path: str,
  temp_dir: str,
  cleanup: list[str],
  *,
  progress_cb: Callable[[int], None] | None = None,
) -> tuple[str, str]:
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
        while True:
          chunk = fin.read(ZIP_PROGRESS_CHUNK_SIZE)
          if not chunk:
            break
          fout.write(chunk)
          if progress_cb is not None:
            progress_cb(len(chunk))
      cleanup.append(dest_path)
      rel_dir = os.path.dirname(rel_path)
      arc_rel = os.path.join(rel_dir, dest_base) if rel_dir not in ("", ".") else dest_base
      return dest_path, arc_rel
    except Exception as e:
      capture_exception(e)
      log(f"❌ Failed to compress {rel_path}: {e}", "ERROR")
      raise

  return src_path, rel_path


def _sanitize_segment_component(value: str) -> str:
  sanitized = re.sub(r"[^A-Za-z0-9._-]+", "-", value or "")
  sanitized = re.sub(r"-+", "-", sanitized).strip("-._")
  return sanitized[:180]


def _build_segment_rlog_filename(device_id: str, drive_name: str) -> str:
  if not device_id:
    raise ValueError("device_id is required")
  drive_part = _sanitize_segment_component(drive_name)
  if not drive_part:
    raise ValueError("drive_name is required")
  return f"{device_id}__{drive_part}__rlog.bz2"


def _is_direct_segment_rlog_request(requested_files: list[str] | None) -> bool:
  return requested_files in (["rlog"], ["rlog.bz2"])


def _find_direct_segment_rlog_path(segments: list[str]) -> str | None:
  matches = []
  for segment in segments:
    for basename in ("rlog", "rlog.bz2"):
      candidate = os.path.join(segment, basename)
      if os.path.isfile(candidate):
        matches.append(candidate)

  if len(matches) > 1:
    raise RuntimeError("multiple segment rlog files matched request")
  if len(matches) == 1:
    return matches[0]
  return None


def _prepare_segment_rlog_upload(src_path: str, temp_dir: str) -> tuple[str, list[str]]:
  if src_path.endswith(".bz2"):
    return src_path, []

  upload_path = os.path.join(temp_dir, f"{_safe_zip_name(os.path.basename(src_path))}.bz2")
  with open(src_path, "rb") as fin, bz2.open(upload_path, "wb") as fout:
    shutil.copyfileobj(fin, fout)
  return upload_path, [upload_path]


def _build_schema_profile_id(branch_name: str | None, device_id: str) -> str:
  branch_part = _sanitize_segment_component(branch_name or "")
  if branch_part:
    return f"schema-{branch_part.lower()}"
  return f"schema-{_sanitize_segment_component(device_id).lower()}"


def _is_schema_bundle_request(drive_name: str, requested_files: list[str] | None) -> bool:
  return bool(drive_name and drive_name.startswith(SCHEMA_FETCH_DRIVE_PREFIX)) or requested_files == [SCHEMA_FETCH_REQUEST_FILE]


def _teletyped_repo_root() -> str:
  candidates: list[str] = []

  env_basedir = os.environ.get("BASEDIR")
  if env_basedir:
    candidates.append(os.path.abspath(env_basedir))

  if OPENPILOT_BASEDIR:
    candidates.append(os.path.abspath(OPENPILOT_BASEDIR))

  script_path = Path(os.path.realpath(__file__))
  candidates.extend(str(parent) for parent in script_path.parents[:6])
  candidates.append(os.path.abspath(os.getcwd()))

  seen: set[str] = set()
  checked: list[str] = []
  for candidate in candidates:
    if not candidate or candidate in seen:
      continue
    seen.add(candidate)
    checked.append(candidate)
    if os.path.isfile(os.path.join(candidate, "cereal", "log.capnp")):
      return candidate

  raise RuntimeError(
    "could not locate source root containing cereal/log.capnp; checked: "
    + ", ".join(checked)
  )


def _current_git_branch() -> str | None:
  try:
    op_info = get_op_params_info()
  except Exception as e:
    capture_exception(e)
    log(f"Failed to read git branch from Params: {e}", "WARN")
    return None
  branch = op_info.get("git_branch")
  if isinstance(branch, str) and branch.strip():
    return branch.strip()
  return None


def _iter_capnp_imports(abs_path: str) -> list[str]:
  imports: list[str] = []
  try:
    with open(abs_path, encoding="utf-8") as src:
      for line in src:
        match = CAPNP_IMPORT_RE.match(line)
        if not match:
          continue
        import_path = match.group(1).strip()
        if import_path.startswith("/capnp/"):
          continue
        imports.append(import_path)
  except OSError as e:
    raise RuntimeError(f"failed to read {abs_path}: {e}") from e
  return imports


def _collect_schema_bundle_paths(root_dir: str) -> list[str]:
  seed_paths = ["cereal/log.capnp"]
  for extra in ("cereal/car.capnp", "cereal/custom.capnp"):
    if os.path.isfile(os.path.join(root_dir, extra)):
      seed_paths.append(extra)

  queue = list(seed_paths)
  seen: set[str] = set()
  ordered: list[str] = []

  while queue:
    rel_path = os.path.normpath(queue.pop(0)).replace('\\', '/')
    if rel_path in seen:
      continue

    abs_path = os.path.join(root_dir, rel_path)
    if not os.path.isfile(abs_path):
      raise RuntimeError(f"missing schema dependency: {rel_path}")

    seen.add(rel_path)
    ordered.append(rel_path)

    for import_path in _iter_capnp_imports(abs_path):
      dep_rel = os.path.normpath(os.path.join(os.path.dirname(rel_path), import_path)).replace('\\', '/')
      if dep_rel.startswith('../'):
        raise RuntimeError(f"schema import escapes repo root: {import_path}")
      dep_abs = os.path.join(root_dir, dep_rel)
      if not os.path.isfile(dep_abs):
        raise RuntimeError(f"missing imported schema file: {dep_rel}")
      queue.append(dep_rel)

  return ordered


def _build_schema_bundle_zip(temp_dir: str, branch_name: str | None, device_id: str) -> tuple[str, list[str], str]:
  root_dir = _teletyped_repo_root()
  profile_id = _build_schema_profile_id(branch_name, device_id)
  rel_paths = _collect_schema_bundle_paths(root_dir)
  zip_label = _safe_zip_name(branch_name or profile_id)
  zip_path = os.path.join(temp_dir, f"{zip_label}-schema.zip")
  with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
    for rel_path in rel_paths:
      abs_path = os.path.join(root_dir, rel_path)
      zipf.write(abs_path, arcname=rel_path)
  return zip_path, rel_paths, profile_id


def upload_schema_bundle(zip_path: str, device_id: str, branch_name: str | None, profile_id: str) -> dict | None:
  headers = _auth_headers()
  if not headers:
    log("⚠️ Schema bundle upload skipped (missing auth headers)", "WARN")
    return None

  label = f"{branch_name} schema" if branch_name else f"{profile_id} schema"
  try:
    with open(zip_path, "rb") as file_obj:
      response = http_post(
        SCHEMA_BUNDLE_UPLOAD_PATH,
        headers=headers,
        data={
          "device_id": device_id,
          "branch_name": branch_name or "",
          "profile_id": profile_id,
          "label": label,
          "overwrite": "true",
        },
        files={"file": (os.path.basename(zip_path), file_obj, "application/zip")},
        timeout=max(TIMEOUT, 120),
      )
  except Exception as e:
    capture_exception(e)
    log(f"❌ Schema bundle upload failed: {e}", "ERROR")
    return None

  if response.status_code == 200:
    try:
      payload = response.json()
    except ValueError:
      payload = {}
    profile = payload.get("profile") if isinstance(payload, dict) else None
    if isinstance(profile, dict):
      log(f"📨 Uploaded schema bundle -> {profile.get('id') or profile_id}")
      return profile
    return {"id": profile_id, "label": label}

  body = response.text.strip()[:200]
  log(f"❌ Schema bundle upload rejected: status={response.status_code} body={body}", "ERROR")
  return None


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


def update_drive_transfer(device_id, drive_name, status=None, *, clear_progress=False, **extra):
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
  if clear_progress:
    payload["progress_percent"] = None
  try:
    res = http_post(DRIVE_TRANSFER_UPDATE_PATH, json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    log(f"Updated drive transfer: {drive_name} -> {status or 'unchanged'}")
  except Exception as e:
    capture_exception(e)
    log(f"Failed to update drive transfer for {drive_name}: {e}", "ERROR")


def update_drive_scan_status(device_id, **extra):
  headers = _auth_headers()
  if not headers:
    return
  payload = {k: v for k, v in extra.items() if v is not None}
  try:
    res = http_post(f"{DRIVE_SCAN_STATUS_PATH}/{device_id}/status", json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
  except Exception as e:
    capture_exception(e)
    log(f"Failed to update drive scan status for {device_id}: {e}", "WARN")


def report_transfer_progress(device_id, drive_name, *, status="sending", stage=None, progress_percent=None, detail=None, clear_progress=False, **extra):
  payload = {k: v for k, v in extra.items() if v is not None}
  if stage is not None:
    payload["stage"] = stage
  if progress_percent is not None:
    payload["progress_percent"] = progress_percent
  if detail is not None:
    payload["detail"] = detail
  update_drive_transfer(device_id, drive_name, status=status, clear_progress=clear_progress, **payload)


def report_transfer_error(device_id, drive_name, error, *, detail=None, stage="failed"):
  update_drive_transfer(
    device_id,
    drive_name,
    status="error",
    stage=stage,
    detail=detail or error,
    error=error,
    clear_progress=True,
  )


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

  requested_scan = bool(request_info.get("pending"))
  if not requested_scan and not AUTO_DRIVE_INVENTORY:
    return

  scanned_at = datetime.now(UTC).isoformat()
  if requested_scan:
    update_drive_scan_status(
      device_id,
      status="in_progress",
      stage="collecting",
      detail="Collecting drive inventory on the device.",
      scanned_at=scanned_at,
    )

  try:
    drives, total_size = collect_drive_inventory()
  except Exception as e:
    capture_exception(e)
    if requested_scan:
      update_drive_scan_status(
        device_id,
        status="error",
        stage="failed",
        detail="Failed to collect drive inventory on the device.",
        error=str(e),
        )
    log(f"Failed to collect drive inventory: {e}", "ERROR")
    return

  payload = {
    "device_id": device_id,
    "drives": drives,
    "total_size_bytes": total_size,
    "scanned_at": scanned_at,
  }

  if requested_scan:
    update_drive_scan_status(
      device_id,
      status="in_progress",
      stage="uploading",
      detail=f"Uploading inventory with {len(drives)} drives to the server.",
      drive_count=len(drives),
      scanned_at=scanned_at,
    )

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
    if requested_scan:
      update_drive_scan_status(
        device_id,
        status="error",
        stage="failed",
        detail="Failed to upload drive inventory to the server.",
        error=str(e),
        drive_count=len(drives),
        scanned_at=scanned_at,
      )
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
      filename = os.path.basename(zip_path_abs)

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
          report_transfer_progress(
            device_id,
            drive_name,
            status="sending",
            stage="wormhole_ready",
            detail=f"Wormhole code ready for {filename}. Waiting for the dock to receive it.",
            clear_progress=True,
            wormhole_code=code,
            filename=filename,
          )
          worked = send_wormhole_code(
            code,
            zip_path_abs,
            filename,
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
        log_local_send(device_id, filename, code, timestamp, drive_name=drive_name)
        return True, code, filename

      log("⚠️ Wormhole code not found in output", "WARN")
    except Exception as e:
      capture_exception(e)
      log(f"Attempt {attempt} failed: {e}", "ERROR")
      time.sleep(2)

  return False, None, os.path.basename(os.path.abspath(zip_path))


def upload_segment_rlog(file_path: str, device_id: str, drive_name: str) -> str | None:
  headers = _auth_headers()
  if not headers:
    log("⚠️ Segment rlog upload skipped (missing auth headers)", "WARN")
    return None

  canonical_name = _build_segment_rlog_filename(device_id, drive_name)
  temp_cleanup: list[str] = []
  try:
    upload_path, temp_cleanup = _prepare_segment_rlog_upload(file_path, _pick_temp_dir())
    with open(upload_path, "rb") as file_obj:
      response = http_post(
        RLOG_UPLOAD_PATH,
        headers=headers,
        data={
          "device_id": device_id,
          "drive_name": drive_name,
          "overwrite": "false",
        },
        files={"file": (os.path.basename(upload_path), file_obj, "application/x-bzip2")},
        timeout=max(TIMEOUT, 120),
      )
  except Exception as e:
    capture_exception(e)
    log(f"❌ Segment rlog upload failed for {drive_name}: {e}", "ERROR")
    return None
  finally:
    for temp_file in temp_cleanup:
      try:
        os.remove(temp_file)
      except OSError:
        pass

  if response.status_code == 200:
    try:
      payload = response.json()
    except ValueError:
      payload = {}
    uploaded_name = payload.get("filename") or canonical_name
    log(f"📨 Uploaded segment rlog {drive_name} -> {uploaded_name}")
    return uploaded_name

  if response.status_code == 409:
    log(f"ℹ️ Hosted segment rlog already exists: {canonical_name}")
    return canonical_name

  body = response.text.strip()[:200]
  log(
    f"❌ Segment rlog upload rejected for {drive_name}: status={response.status_code} body={body}",
    "ERROR",
  )
  return None

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

    if _is_schema_bundle_request(drive_name, requested_files):
      branch_name = transfer.get("schema_branch") if isinstance(transfer.get("schema_branch"), str) else _current_git_branch()
      temp_dir = _pick_temp_dir(25 * 1024 * 1024)
      zip_path = None
      try:
        report_transfer_progress(device_id, drive_name, stage="collecting", detail="Collecting schema files on the device.", clear_progress=True)
        zip_path, included_files, profile_id = _build_schema_bundle_zip(temp_dir, branch_name, device_id)
        report_transfer_progress(device_id, drive_name, stage="uploading", detail="Uploading schema bundle to the server.", clear_progress=True)
        uploaded_profile = upload_schema_bundle(zip_path, device_id, branch_name, profile_id)
        if uploaded_profile:
          update_drive_transfer(
            device_id,
            drive_name,
            status="sent",
            stage="uploaded",
            detail="Schema bundle uploaded and ready for the dock.",
            included_files=included_files,
            clear_progress=True,
          )
        else:
          report_transfer_error(device_id, drive_name, "schema upload failed", detail="Schema bundle upload failed.")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Schema bundle build failed for {drive_name}: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, str(e))
      finally:
        if zip_path and os.path.exists(zip_path):
          try:
            os.remove(zip_path)
          except OSError:
            pass
      continue

    if boot_base is not None:
      if not os.path.isdir(BOOT_DIR):
        log("Boot directory missing; cannot send boot files", "WARN")
        report_transfer_error(device_id, drive_name, "boot directory missing", detail="Boot directory is missing on the device.")
        continue

      try:
        boot_entries = sorted(os.listdir(BOOT_DIR))
      except OSError as e:
        capture_exception(e)
        log(f"Failed to list boot directory: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, "boot dir unreadable", detail="Boot directory could not be read on the device.")
        continue

      boot_files: list[tuple[str, str, int]] = []
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
        try:
          file_size = os.path.getsize(file_path)
        except OSError:
          file_size = 0
        boot_files.append((fname, file_path, file_size))

      if not boot_files:
        log("No boot files match the requested set", "WARN")
        report_transfer_error(device_id, drive_name, "boot file not requested", detail="No boot files matched the requested file filter.")
        continue

      zip_label = boot_base if boot_base else "boot"
      zip_path = os.path.join(_pick_temp_dir(), f"{_safe_zip_name(zip_label)}.zip")
      try:
        added_files = 0
        total_boot_bytes = sum(file_size for _fname, _path, file_size in boot_files)
        report_transfer_progress(device_id, drive_name, stage="collecting", detail="Collecting boot files on the device.", clear_progress=True)
        progress_reporter = ZipProgressReporter(
          device_id,
          drive_name,
          stage="compressing",
          label="Compressing and zipping boot files on the device",
          total_files=len(boot_files),
          total_bytes=total_boot_bytes,
          start_percent=0,
          end_percent=100,
        )
        progress_reporter.report(force=True)
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          for fname, route_path, _file_size in boot_files:
            arcname = os.path.join("boot", fname)
            _copy_path_into_zip(zipf, route_path, arcname, progress_cb=progress_reporter.advance_bytes)
            added_files += 1
            progress_reporter.file_completed()
        if added_files == 0:
          log("No boot files zipped (empty selection)", "WARN")
          report_transfer_error(device_id, drive_name, "boot file missing", detail="No boot files were added to the archive.")
          continue
        log(f"📦 Zipped boot files ({added_files}) into folder: boot/")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for boot files {boot_files}: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, str(e))
        continue

    else:
      try:
        entries = os.listdir(REALDATA_DIR)
      except OSError as e:
        capture_exception(e)
        log(f"Failed to list realdata: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, str(e))
        continue

      segments = []
      base_name = os.path.basename(base_name)
      legacy_prefix = base_name + "--"
      for d in entries:
        if d == base_name or d.startswith(legacy_prefix):
          segments.append(os.path.join(REALDATA_DIR, d))

      if not segments:
        log(f"Missing file(s) for prefix: {base_name}", "WARN")
        report_transfer_error(device_id, drive_name, "segments missing", detail="No matching route segments were found on the device.")
        continue

      if _is_direct_segment_rlog_request(requested_files):
        try:
          rlog_path = _find_direct_segment_rlog_path(segments)
        except Exception as e:
          capture_exception(e)
          log(f"Could not resolve direct segment rlog for {drive_name}: {e}", "ERROR")
          report_transfer_error(device_id, drive_name, str(e))
          continue

        if not rlog_path:
          log(f"Missing rlog for segment {drive_name}", "WARN")
          report_transfer_error(device_id, drive_name, "rlog missing", detail="The requested rlog file was not found.")
          continue

        report_transfer_progress(device_id, drive_name, stage="uploading", detail="Uploading direct segment rlog to the server.", clear_progress=True)
        uploaded_name = upload_segment_rlog(rlog_path, device_id, drive_name)
        if uploaded_name:
          update_drive_transfer(
            device_id,
            drive_name,
            status="sent",
            stage="uploaded",
            detail="Direct segment rlog uploaded and ready for the dock.",
            filename=uploaded_name,
            clear_progress=True,
            included_files=["rlog"],
          )
        else:
          report_transfer_error(device_id, drive_name, "rlog upload failed", detail="Segment rlog upload failed.")
        continue

      temp_dir = _pick_temp_dir()
      zip_path = os.path.join(temp_dir, f"{_safe_zip_name(base_name)}.zip")
      temp_cleanup: list[str] = []
      try:
        missing_files = []
        selected_files: list[tuple[str, str, int]] = []
        for segment in segments:
          for root, _, files in os.walk(segment):
            for file in files:
              if not should_include_file(file, requested_files):
                continue
              abs_path = os.path.join(root, file)
              if not os.path.isfile(abs_path):
                missing_files.append(abs_path)
                continue
              try:
                file_size = os.path.getsize(abs_path)
              except OSError:
                file_size = 0
              rel_path = os.path.relpath(abs_path, REALDATA_DIR)
              selected_files.append((abs_path, rel_path, file_size))

        added_files = 0
        total_selected_bytes = sum(file_size for _abs_path, _rel_path, file_size in selected_files)
        report_transfer_progress(device_id, drive_name, stage="collecting", detail="Collecting route segments on the device.", clear_progress=True)
        progress_reporter = ZipProgressReporter(
          device_id,
          drive_name,
          stage="compressing",
          label="Compressing and zipping the requested files on the device",
          total_files=len(selected_files),
          total_bytes=total_selected_bytes,
          start_percent=0,
          end_percent=100,
        )
        progress_reporter.report(force=True)
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          for abs_path, rel_path, _file_size in selected_files:
            try:
              src_path, rel_for_zip = _maybe_compress_for_zip(
                abs_path,
                rel_path,
                temp_dir,
                temp_cleanup,
                progress_cb=progress_reporter.advance_bytes if os.path.basename(rel_path) in COMPRESSIBLE_BASENAMES else None,
              )
            except Exception:
              missing_files.append(abs_path)
              continue
            arcname = os.path.join(base_name, rel_for_zip)
            try:
              if src_path == abs_path:
                _copy_path_into_zip(zipf, src_path, arcname, progress_cb=progress_reporter.advance_bytes)
              else:
                _copy_path_into_zip(zipf, src_path, arcname)
              added_files += 1
              progress_reporter.file_completed()
            except FileNotFoundError as e:
              capture_exception(e)
              missing_files.append(abs_path)
              continue
        if missing_files:
          log(f"⚠️ Skipped {len(missing_files)} missing file(s) while zipping {base_name}", "WARN")
        if added_files == 0:
          log(f"No files matched requested list for {base_name}", "WARN")
          report_transfer_error(device_id, drive_name, "no files match request", detail="No files matched the requested file filter.")
          continue
        report_transfer_progress(device_id, drive_name, stage="zipped", detail=f"Archive ready with {added_files} file(s).", clear_progress=True)
        log(f"📦 Zipped {len(segments)} segment(s) into folder: {base_name} (files={added_files})")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for {base_name}: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, str(e))
        continue
      finally:
        for temp_file in temp_cleanup:
          try:
            os.remove(temp_file)
          except OSError:
            pass

    report_transfer_progress(
      device_id,
      drive_name,
      stage="wormhole",
      detail="Starting wormhole send and waiting for a wormhole code.",
      clear_progress=True,
    )

    sent, wormhole_code, archive_name = send_file_wormhole(zip_path, device_id, drive_name, requested_files)
    if sent:
      update_drive_transfer(
        device_id,
        drive_name,
        status="sent",
        stage="ready",
        detail="Wormhole code registered. Transfer is ready for the dock.",
        wormhole_code=wormhole_code,
        clear_progress=True,
        filename=archive_name,
      )
    else:
      report_transfer_error(device_id, drive_name, "wormhole send failed", detail="Wormhole send failed before a download offer was created.")

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
        drive_inventory_step(device_id)
      except Exception as e:
        capture_exception(e)
        log(f"❌ drive_inventory_step() failed: {e}", "ERROR")

      try:
        route_sender_step(device_id)
      except Exception as e:
        capture_exception(e)
        log(f"❌ route_sender_step() failed: {e}", "ERROR")

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
