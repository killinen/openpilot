#!/usr/bin/env python3
import os
import time
import json
import hashlib
import re
import threading
from zipfile import ZipFile, ZipInfo, ZIP_DEFLATED, ZIP_STORED
import subprocess
import bz2
from concurrent.futures import ThreadPoolExecutor, as_completed
from collections.abc import Callable
from pathlib import Path
from datetime import datetime, UTC
from typing import Any, cast
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
  PERSIST_ROOT,
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


def _env_nonnegative_int(name: str, default: int) -> int:
  raw = os.environ.get(name)
  if raw is None:
    return default
  try:
    return max(0, int(raw.strip()))
  except (TypeError, ValueError):
    return default


def _env_bounded_int(name: str, default: int, lower: int, upper: int) -> int:
  raw = os.environ.get(name)
  if raw is None:
    return default
  try:
    value = int(raw.strip())
  except (TypeError, ValueError):
    return default
  return min(upper, max(lower, value))


TIMEOUT = 5
ZIP_EPOCH = datetime(1980, 1, 1).timestamp()
RETRY_LIMIT = 2
ZIP_PROGRESS_CHUNK_SIZE = max(
  1024 * 1024,
  _env_nonnegative_int("TELETYPED_ZIP_CHUNK_SIZE", 4 * 1024 * 1024),
)
ZIP_PROGRESS_REPORT_INTERVAL_SECONDS = 1.0
RLOG_BZ2_COMPRESSLEVEL = _env_bounded_int("TELETYPED_RLOG_BZ2_COMPRESSLEVEL", 1, 1, 9)
ZIP_DEFLATE_COMPRESSLEVEL = _env_bounded_int("TELETYPED_ZIP_COMPRESSLEVEL", 1, 0, 9)
RLOG_BZ2_WORKERS = _env_bounded_int(
  "TELETYPED_RLOG_BZ2_WORKERS",
  min(4, max(1, os.cpu_count() or 1)),
  1,
  32,
)
WORMHOLE_ZIP_TARGET_BYTES = max(
  0,
  _env_nonnegative_int("TELETYPED_WORMHOLE_ZIP_TARGET_BYTES", 4 * 1024 * 1024 * 1024),
)

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
DRIVE_INVENTORY_STATE_FILE = "teletyped_drive_inventory_state.json"
_DRIVE_INVENTORY_STATE_PATH: str | None = None

_auth_wait_logged = False
COMPRESSIBLE_BASENAMES = {"qlog", "rlog"}
ZIP_STORED_EXTENSIONS = (
  ".7z",
  ".bz2",
  ".gz",
  ".jpg",
  ".jpeg",
  ".lz4",
  ".mp4",
  ".png",
  ".xz",
  ".zip",
  ".zst",
)
CAPNP_IMPORT_RE = re.compile(
  r'^\s*using(?:\s+[A-Za-z_][A-Za-z0-9_]*\s*=\s*)?\s+import\s+"([^"]+)"'
)
SCHEMA_FETCH_REQUEST_FILE = "__schema_bundle__"
SCHEMA_FETCH_DRIVE_PREFIX = "__schema__:"


AUTO_DRIVE_INVENTORY_MIN_INTERVAL = _env_nonnegative_int(
  "TELETYPED_AUTO_DRIVE_INVENTORY_MIN_INTERVAL",
  900,
)
AUTO_DRIVE_INVENTORY_FORCE_REFRESH = _env_nonnegative_int(
  "TELETYPED_AUTO_DRIVE_INVENTORY_FORCE_REFRESH",
  86400,
)


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


def _zip_compress_type(name: str) -> int:
  if ZIP_DEFLATE_COMPRESSLEVEL == 0:
    return ZIP_STORED
  if name.lower().endswith(ZIP_STORED_EXTENSIONS):
    return ZIP_STORED
  return ZIP_DEFLATED


def _zip_info(arcname: str, compress_type: int) -> ZipInfo:
  info = ZipInfo(arcname)
  info.compress_type = compress_type
  if compress_type == ZIP_DEFLATED:
    cast(Any, info)._compresslevel = ZIP_DEFLATE_COMPRESSLEVEL
  return info


def _zip_write_file(zipf: ZipFile, src_path: str, arcname: str) -> None:
  compress_type = _zip_compress_type(arcname)
  if compress_type == ZIP_DEFLATED:
    zipf.write(src_path, arcname=arcname, compress_type=compress_type, compresslevel=ZIP_DEFLATE_COMPRESSLEVEL)
  else:
    zipf.write(src_path, arcname=arcname, compress_type=compress_type)


def _inventory_state_default() -> dict[str, float | str]:
  return {
    "last_auto_scan_started_at": 0.0,
    "last_successful_upload_at": 0.0,
    "last_inventory_fingerprint": "",
  }


def _is_writable_dir(path: str) -> bool:
  test_path = os.path.join(path, ".teletyped_write_test")
  try:
    os.makedirs(path, exist_ok=True)
    with open(test_path, "w", encoding="utf-8") as f:
      f.write("ok")
    os.remove(test_path)
    return True
  except OSError:
    try:
      if os.path.exists(test_path):
        os.remove(test_path)
    except OSError:
      pass
    return False


def _resolve_drive_inventory_state_path() -> str:
  global _DRIVE_INVENTORY_STATE_PATH
  if _DRIVE_INVENTORY_STATE_PATH:
    return _DRIVE_INVENTORY_STATE_PATH

  candidates = [
    os.path.join(PERSIST_ROOT, "comma"),
    os.path.join(PERSIST_ROOT, "teletyped"),
    "/data/params/d/goranconnect_ssh",
    os.path.dirname(SENDER_LOG),
    "/tmp/comma",
  ]

  for base in candidates:
    path = os.path.join(base, DRIVE_INVENTORY_STATE_FILE)
    if os.path.exists(path):
      _DRIVE_INVENTORY_STATE_PATH = path
      return path

  for base in candidates:
    if _is_writable_dir(base):
      _DRIVE_INVENTORY_STATE_PATH = os.path.join(base, DRIVE_INVENTORY_STATE_FILE)
      return _DRIVE_INVENTORY_STATE_PATH

  _DRIVE_INVENTORY_STATE_PATH = os.path.join("/tmp", DRIVE_INVENTORY_STATE_FILE)
  return _DRIVE_INVENTORY_STATE_PATH


def _load_drive_inventory_state() -> dict[str, float | str]:
  path = _resolve_drive_inventory_state_path()
  state = _inventory_state_default()
  if not os.path.isfile(path):
    return state

  try:
    with open(path, encoding="utf-8") as f:
      data = json.load(f)
    if not isinstance(data, dict):
      return state
    for key in ("last_auto_scan_started_at", "last_successful_upload_at"):
      value = data.get(key)
      if isinstance(value, (int, float)):
        state[key] = float(value)
    fingerprint = data.get("last_inventory_fingerprint")
    if isinstance(fingerprint, str):
      state["last_inventory_fingerprint"] = fingerprint
  except Exception:
    return state
  return state


def _save_drive_inventory_state(state: dict[str, float | str]) -> None:
  path = _resolve_drive_inventory_state_path()
  directory = os.path.dirname(path) or "."
  try:
    os.makedirs(directory, exist_ok=True)
    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as f:
      json.dump(state, f)
    os.replace(temp_path, path)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to save drive inventory state: {e}", "WARN")


def _inventory_fingerprint(drives: list[dict], total_size: int) -> str:
  payload = {
    "drives": drives,
    "total_size_bytes": total_size,
  }
  encoded = json.dumps(
    payload,
    sort_keys=True,
    separators=(",", ":"),
    ensure_ascii=True,
  ).encode("utf-8")
  return hashlib.sha256(encoded).hexdigest()


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
    self._lock = threading.Lock()

  def advance_bytes(self, num_bytes: int) -> None:
    with self._lock:
      if num_bytes > 0:
        self.processed_bytes += num_bytes
      self._report_locked()

  def file_completed(self) -> None:
    with self._lock:
      self.processed_files += 1
      self._report_locked(force=True)

  def report(self, force: bool = False) -> None:
    with self._lock:
      self._report_locked(force=force)

  def _report_locked(self, force: bool = False) -> None:
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
  compress_type: int | None = None,
) -> None:
  if compress_type is None:
    compress_type = _zip_compress_type(arcname)
  info = _zip_info(arcname, compress_type)
  try:
    info.file_size = os.path.getsize(src_path)
  except OSError:
    pass

  with open(src_path, "rb") as src, zipf.open(info, "w") as dest:
    while True:
      chunk = src.read(ZIP_PROGRESS_CHUNK_SIZE)
      if not chunk:
        break
      dest.write(chunk)
      if progress_cb is not None:
        progress_cb(len(chunk))


def _compress_path_into_zip(
  zipf: ZipFile,
  src_path: str,
  arcname: str,
  *,
  progress_cb: Callable[[int], None] | None = None,
) -> None:
  info = _zip_info(arcname, ZIP_STORED)
  with open(src_path, "rb") as src, zipf.open(info, "w") as zip_dest:
    with bz2.BZ2File(zip_dest, "wb", compresslevel=RLOG_BZ2_COMPRESSLEVEL) as bz_dest:
      while True:
        chunk = src.read(ZIP_PROGRESS_CHUNK_SIZE)
        if not chunk:
          break
        bz_dest.write(chunk)
        if progress_cb is not None:
          progress_cb(len(chunk))


def _compress_path_to_bz2_file(
  src_path: str,
  dest_path: str,
  *,
  progress_cb: Callable[[int], None] | None = None,
) -> str:
  tmp_path = f"{dest_path}.tmp-{os.getpid()}-{threading.get_ident()}"
  try:
    with open(src_path, "rb") as fin, bz2.open(tmp_path, "wb", compresslevel=RLOG_BZ2_COMPRESSLEVEL) as fout:
      while True:
        chunk = fin.read(ZIP_PROGRESS_CHUNK_SIZE)
        if not chunk:
          break
        fout.write(chunk)
        if progress_cb is not None:
          progress_cb(len(chunk))
    os.replace(tmp_path, dest_path)
    return dest_path
  except Exception:
    try:
      if os.path.exists(tmp_path):
        os.remove(tmp_path)
    except OSError:
      pass
    raise


def _run_bz2_compressions(
  targets: list[tuple[str, str]],
  *,
  progress_reporter: ZipProgressReporter,
  remove_sources: bool = False,
) -> tuple[dict[str, str], list[str]]:
  compressed_by_src: dict[str, str] = {}
  failed_paths: list[str] = []
  max_workers = min(RLOG_BZ2_WORKERS, len(targets))

  def _compress_one(src_path: str, dest_path: str) -> str:
    compressed_path = _compress_path_to_bz2_file(
      src_path,
      dest_path,
      progress_cb=progress_reporter.advance_bytes,
    )
    if remove_sources:
      os.remove(src_path)
    return compressed_path

  if max_workers <= 1:
    for src_path, dest_path in targets:
      try:
        compressed_by_src[src_path] = _compress_one(src_path, dest_path)
        progress_reporter.file_completed()
      except Exception as e:
        capture_exception(e)
        failed_paths.append(src_path)
        log(f"❌ Failed to compress {src_path}: {e}", "ERROR")
    return compressed_by_src, failed_paths

  log(f"🗜️ Compressing {len(targets)} file(s) with {max_workers} worker(s)")
  with ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="teletyped-bz2") as executor:
    future_to_src = {
      executor.submit(_compress_one, src_path, dest_path): src_path
      for src_path, dest_path in targets
    }
    for future in as_completed(future_to_src):
      src_path = future_to_src[future]
      try:
        compressed_by_src[src_path] = future.result()
        progress_reporter.file_completed()
      except Exception as e:
        capture_exception(e)
        failed_paths.append(src_path)
        log(f"❌ Failed to compress {src_path}: {e}", "ERROR")

  return compressed_by_src, failed_paths


def _temporary_bz2_path(temp_dir: str, rel_path: str, index: int) -> str:
  digest = hashlib.sha256(rel_path.encode("utf-8", errors="replace")).hexdigest()[:12]
  return os.path.join(temp_dir, f"teletyped-bz2-{os.getpid()}-{index}-{digest}.bz2")


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


def _requested_compressible_basenames(requested_files: list[str] | None) -> list[str]:
  if not requested_files:
    return sorted(COMPRESSIBLE_BASENAMES)

  requested = set()
  for name in requested_files:
    base = os.path.basename(str(name).strip().replace("\\", "/")).lower()
    if base.endswith(".bz2"):
      base = base[:-4]
    if base in COMPRESSIBLE_BASENAMES:
      requested.add(base)
  return sorted(requested)


def _compress_file_in_place(
  src_path: str,
  *,
  progress_cb: Callable[[int], None] | None = None,
) -> str:
  dest_path = src_path + ".bz2"
  compressed_path = _compress_path_to_bz2_file(src_path, dest_path, progress_cb=progress_cb)
  os.remove(src_path)
  return compressed_path


def _compress_route_logs_in_place(
  device_id: str,
  drive_name: str,
  segments: list[str],
  requested_files: list[str] | None,
) -> None:
  basenames = _requested_compressible_basenames(requested_files)
  if not basenames:
    report_transfer_error(
      device_id,
      drive_name,
      "no compressible logs requested",
      detail="Compression requests must include rlog and/or qlog.",
    )
    return

  targets: list[tuple[str, int]] = []
  already_compressed: list[str] = []
  missing: list[str] = []

  for segment in segments:
    for base in basenames:
      src_path = os.path.join(segment, base)
      dest_path = src_path + ".bz2"
      if os.path.isfile(src_path):
        try:
          file_size = os.path.getsize(src_path)
        except OSError:
          file_size = 0
        targets.append((src_path, file_size))
      elif os.path.isfile(dest_path):
        already_compressed.append(dest_path)
      else:
        missing.append(src_path)

  if not targets:
    if already_compressed:
      update_drive_transfer(
        device_id,
        drive_name,
        status="sent",
        stage="compressed",
        detail="Requested logs were already compressed on the device.",
        clear_progress=True,
        included_files=[os.path.relpath(path, REALDATA_DIR) for path in already_compressed],
      )
      log(f"ℹ️ Requested logs already compressed for {drive_name}")
      return

    report_transfer_error(
      device_id,
      drive_name,
      "no compressible logs found",
      detail="No uncompressed rlog/qlog files were found for this drive.",
    )
    return

  total_bytes = sum(file_size for _path, file_size in targets)
  progress_reporter = ZipProgressReporter(
    device_id,
    drive_name,
    stage="compressing",
    label="Compressing requested logs on the device",
    total_files=len(targets),
    total_bytes=total_bytes,
    start_percent=0,
    end_percent=100,
  )

  report_transfer_progress(
    device_id,
    drive_name,
    stage="compressing",
    detail=f"Compressing {len(targets)} log file(s) on the device with up to {min(RLOG_BZ2_WORKERS, len(targets))} worker(s).",
    clear_progress=True,
  )
  progress_reporter.report(force=True)

  compressed_by_src, failed_paths = _run_bz2_compressions(
    [(src_path, src_path + ".bz2") for src_path, _file_size in targets],
    progress_reporter=progress_reporter,
    remove_sources=True,
  )

  if failed_paths:
    report_transfer_error(
      device_id,
      drive_name,
      "log compression failed",
      detail=f"Failed to compress {len(failed_paths)} log file(s).",
    )
    return

  compressed_paths = [
    compressed_by_src[src_path]
    for src_path, _file_size in targets
    if src_path in compressed_by_src
  ]
  included_files = [os.path.relpath(path, REALDATA_DIR) for path in compressed_paths]
  update_drive_transfer(
    device_id,
    drive_name,
    status="sent",
    stage="compressed",
    detail=f"Compressed {len(compressed_paths)} log file(s) on the device.",
    included_files=included_files,
    clear_progress=True,
  )
  log(f"🗜️ Compressed {len(compressed_paths)} log file(s) for {drive_name}")
  upload_drive_inventory_snapshot(device_id)


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
  with open(src_path, "rb") as fin, bz2.open(upload_path, "wb", compresslevel=RLOG_BZ2_COMPRESSLEVEL) as fout:
    shutil.copyfileobj(fin, fout, length=ZIP_PROGRESS_CHUNK_SIZE)
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
  with ZipFile(zip_path, 'w') as zipf:
    for rel_path in rel_paths:
      abs_path = os.path.join(root_dir, rel_path)
      _zip_write_file(zipf, abs_path, rel_path)
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


def _trailing_segment_number(name: str) -> int | None:
  match = re.search(r"--(\d+)$", name)
  if not match:
    return None
  return int(match.group(1))


def _route_segment_sort_key(base_name: str, segment_path: str) -> tuple[int, int, str]:
  segment_name = os.path.basename(segment_path)
  if segment_name == base_name:
    segment_number = _trailing_segment_number(segment_name)
    return (0, segment_number if segment_number is not None else -1, segment_name)

  prefix = base_name + "--"
  if segment_name.startswith(prefix):
    suffix = segment_name[len(prefix):]
    if suffix.isdigit():
      return (0, int(suffix), segment_name)

  segment_number = _trailing_segment_number(segment_name)
  if segment_number is not None:
    return (0, segment_number, segment_name)
  return (1, 0, segment_name)


def _positive_int(value: Any) -> int | None:
  try:
    parsed = int(value)
  except (TypeError, ValueError):
    return None
  return parsed if parsed > 0 else None


def _positive_int_set(value: Any) -> set[int]:
  if not isinstance(value, list):
    return set()
  result = set()
  for item in value:
    parsed = _positive_int(item)
    if parsed is not None:
      result.add(parsed)
  return result


def _split_segment_batches(
  segment_files: list[tuple[str, list[tuple[str, str, int]]]],
  target_bytes: int,
) -> list[list[tuple[str, list[tuple[str, str, int]]]]]:
  if target_bytes <= 0:
    return [segment_files] if segment_files else []

  batches: list[list[tuple[str, list[tuple[str, str, int]]]]] = []
  current: list[tuple[str, list[tuple[str, str, int]]]] = []
  current_bytes = 0

  for segment_name, files in segment_files:
    segment_bytes = sum(file_size for _abs_path, _rel_path, file_size in files)
    if current and current_bytes + segment_bytes > target_bytes:
      batches.append(current)
      current = []
      current_bytes = 0
    current.append((segment_name, files))
    current_bytes += segment_bytes

  if current:
    batches.append(current)
  return batches


def _zip_route_batch(
  zip_path: str,
  base_name: str,
  batch: list[tuple[str, list[tuple[str, str, int]]]],
  progress_reporter: ZipProgressReporter,
  temp_dir: str,
) -> tuple[int, list[str]]:
  added_files = 0
  missing_files: list[str] = []
  batch_files = [file_tuple for _segment_name, files in batch for file_tuple in files]
  temp_compressed_paths: list[str] = []
  compressed_zip_sources: dict[str, str] = {}
  failed_zip_compressions: set[str] = set()
  compressible_zip_entries = [
    (abs_path, rel_path)
    for abs_path, rel_path, _file_size in batch_files
    if os.path.basename(rel_path) in COMPRESSIBLE_BASENAMES
  ]

  if len(compressible_zip_entries) > 1 and RLOG_BZ2_WORKERS > 1:
    bz2_targets: list[tuple[str, str]] = []
    for index, (abs_path, rel_path) in enumerate(compressible_zip_entries):
      temp_bz2_path = _temporary_bz2_path(temp_dir, rel_path, index)
      temp_compressed_paths.append(temp_bz2_path)
      bz2_targets.append((abs_path, temp_bz2_path))
    compressed_zip_sources, failed_zip_paths = _run_bz2_compressions(
      bz2_targets,
      progress_reporter=progress_reporter,
    )
    failed_zip_compressions = set(failed_zip_paths)
    missing_files.extend(failed_zip_paths)

  try:
    with ZipFile(zip_path, 'w') as zipf:
      for abs_path, rel_path, _file_size in batch_files:
        try:
          if os.path.basename(rel_path) in COMPRESSIBLE_BASENAMES:
            rel_for_zip = rel_path + ".bz2"
            arcname = os.path.join(base_name, rel_for_zip)
            if abs_path in compressed_zip_sources:
              _copy_path_into_zip(
                zipf,
                compressed_zip_sources[abs_path],
                arcname,
                compress_type=ZIP_STORED,
              )
              added_files += 1
              continue
            if abs_path in failed_zip_compressions:
              continue
            _compress_path_into_zip(
              zipf,
              abs_path,
              arcname,
              progress_cb=progress_reporter.advance_bytes,
            )
            added_files += 1
            progress_reporter.file_completed()
            continue
        except Exception:
          missing_files.append(abs_path)
          continue

        arcname = os.path.join(base_name, rel_path)
        try:
          _copy_path_into_zip(zipf, abs_path, arcname, progress_cb=progress_reporter.advance_bytes)
          added_files += 1
          progress_reporter.file_completed()
        except FileNotFoundError as e:
          capture_exception(e)
          missing_files.append(abs_path)
          continue
  finally:
    for temp_compressed_path in temp_compressed_paths:
      try:
        if os.path.exists(temp_compressed_path):
          os.remove(temp_compressed_path)
      except OSError:
        pass
  return added_files, missing_files


def send_wormhole_code(
  code,
  zip_path,
  filename,
  timestamp,
  device_id,
  drive_name=None,
  requested_files=None,
  part_number=None,
  total_parts=None,
  send_parts=None,
):
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
  if part_number:
    payload["part_number"] = part_number
  if total_parts:
    payload["total_parts"] = total_parts
  if send_parts:
    payload["send_parts"] = sorted(send_parts)
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
  state = _load_drive_inventory_state()
  now = time.time()
  if not requested_scan:
    last_auto_scan_started_at = float(state.get("last_auto_scan_started_at") or 0.0)
    if AUTO_DRIVE_INVENTORY_MIN_INTERVAL > 0 and (
      now - last_auto_scan_started_at
    ) < AUTO_DRIVE_INVENTORY_MIN_INTERVAL:
      return
    state["last_auto_scan_started_at"] = now
    _save_drive_inventory_state(state)

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
  fingerprint = _inventory_fingerprint(drives, total_size)

  if not requested_scan:
    last_successful_upload_at = float(state.get("last_successful_upload_at") or 0.0)
    last_inventory_fingerprint = str(state.get("last_inventory_fingerprint") or "")
    needs_upload = last_successful_upload_at <= 0 or not last_inventory_fingerprint
    should_force_refresh = (
      AUTO_DRIVE_INVENTORY_FORCE_REFRESH > 0
      and last_successful_upload_at > 0
      and (now - last_successful_upload_at) >= AUTO_DRIVE_INVENTORY_FORCE_REFRESH
    )
    if fingerprint == last_inventory_fingerprint and not needs_upload and not should_force_refresh:
      log("📊 Drive inventory unchanged; skipping automatic upload.")
      return

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
    state["last_auto_scan_started_at"] = now
    state["last_successful_upload_at"] = now
    state["last_inventory_fingerprint"] = fingerprint
    _save_drive_inventory_state(state)
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


def upload_drive_inventory_snapshot(device_id):
  headers = _auth_headers()
  if not headers:
    return

  scanned_at = datetime.now(UTC).isoformat()
  try:
    drives, total_size = collect_drive_inventory()
    response = http_post(
      DRIVE_INVENTORY_UPLOAD_PATH,
      json={
        "device_id": device_id,
        "drives": drives,
        "total_size_bytes": total_size,
        "scanned_at": scanned_at,
      },
      headers=headers,
      timeout=max(TIMEOUT, 30),
    )
    response.raise_for_status()
    state = _load_drive_inventory_state()
    now = time.time()
    state["last_successful_upload_at"] = now
    state["last_inventory_fingerprint"] = _inventory_fingerprint(drives, total_size)
    _save_drive_inventory_state(state)
    log(f"📊 Reported post-compression inventory with {len(drives)} drive(s).")
  except Exception as e:
    capture_exception(e)
    log(f"Failed to upload post-compression drive inventory: {e}", "WARN")


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

def send_file_wormhole(
  zip_path,
  device_id,
  drive_name,
  requested_files,
  *,
  part_number=None,
  total_parts=None,
  send_parts=None,
):
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
            part_number=part_number,
            total_parts=total_parts,
            send_parts=send_parts,
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
    action = str(transfer.get("action") or "transfer").strip().lower()
    max_segments = _positive_int(transfer.get("max_segments"))
    received_parts = _positive_int_set(transfer.get("received_parts"))
    zip_paths_to_send: list[tuple[str, int, int]] = []

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
        with ZipFile(zip_path, 'w') as zipf:
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
        zip_paths_to_send.append((zip_path, 1, 1))
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
      segments.sort(key=lambda segment: _route_segment_sort_key(base_name, segment))
      if max_segments:
        segments = segments[:max_segments]

      if not segments:
        log(f"Missing file(s) for prefix: {base_name}", "WARN")
        report_transfer_error(device_id, drive_name, "segments missing", detail="No matching route segments were found on the device.")
        continue

      if action == "compress":
        _compress_route_logs_in_place(device_id, drive_name, segments, requested_files)
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
      try:
        missing_files = []
        segment_files: list[tuple[str, list[tuple[str, str, int]]]] = []
        for segment in segments:
          selected_files: list[tuple[str, str, int]] = []
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
          if selected_files:
            segment_files.append((os.path.basename(segment), selected_files))

        if not segment_files:
          log(f"No files matched requested list for {base_name}", "WARN")
          report_transfer_error(device_id, drive_name, "no files match request", detail="No files matched the requested file filter.")
          continue

        segment_batches = _split_segment_batches(segment_files, WORMHOLE_ZIP_TARGET_BYTES)
        total_parts = len(segment_batches)
        received_parts = {part for part in received_parts if part <= total_parts}
        missing_part_numbers = [part for part in range(1, total_parts + 1) if part not in received_parts]
        update_drive_transfer(
          device_id,
          drive_name,
          total_parts=total_parts,
          received_parts=sorted(received_parts),
        )
        if not missing_part_numbers:
          update_drive_transfer(
            device_id,
            drive_name,
            status="received",
            stage="received",
            detail="All archive parts were already received by the dock.",
            clear_progress=True,
          )
          log(f"✅ All archive parts already received for {drive_name}; skipping resend.")
          continue

        total_added_files = 0
        total_selected_bytes = sum(
          file_size
          for _segment_name, files in segment_files
          for _abs_path, _rel_path, file_size in files
        )
        report_transfer_progress(device_id, drive_name, stage="collecting", detail="Collecting route segments on the device.", clear_progress=True)
        for part_index, batch in enumerate(segment_batches, start=1):
          if part_index in received_parts:
            log(f"↩️ Skipping archive part {part_index}/{total_parts}; already received by dock.")
            continue
          batch_files = [file_tuple for _segment_name, files in batch for file_tuple in files]
          batch_bytes = sum(file_size for _abs_path, _rel_path, file_size in batch_files)
          if total_parts == 1:
            zip_path = os.path.join(temp_dir, f"{_safe_zip_name(base_name)}.zip")
            label = "Compressing and zipping the requested files on the device"
          else:
            zip_path = os.path.join(
              temp_dir,
              f"{_safe_zip_name(base_name)}.part{part_index:02d}-of{total_parts:02d}.zip",
            )
            label = f"Compressing route archive part {part_index}/{total_parts} on the device"
          progress_reporter = ZipProgressReporter(
            device_id,
            drive_name,
            stage="compressing",
            label=label,
            total_files=len(batch_files),
            total_bytes=batch_bytes,
            start_percent=0,
            end_percent=100,
          )
          progress_reporter.report(force=True)
          added_files, batch_missing_files = _zip_route_batch(zip_path, base_name, batch, progress_reporter, temp_dir)
          total_added_files += added_files
          missing_files.extend(batch_missing_files)
          if added_files:
            zip_paths_to_send.append((zip_path, part_index, total_parts))
          else:
            try:
              os.remove(zip_path)
            except OSError:
              pass
        if missing_files:
          log(f"⚠️ Skipped {len(missing_files)} missing file(s) while zipping {base_name}", "WARN")
        if total_added_files == 0:
          log(f"No files matched requested list for {base_name}", "WARN")
          report_transfer_error(device_id, drive_name, "no files match request", detail="No files matched the requested file filter.")
          continue
        split_detail = f" across {total_parts} archive part(s)" if total_parts > 1 else ""
        report_transfer_progress(device_id, drive_name, stage="zipped", detail=f"Archive ready with {total_added_files} file(s){split_detail}.", clear_progress=True)
        log(f"📦 Zipped {len(segments)} segment(s) into folder: {base_name} (files={total_added_files}, parts={total_parts}, selected={_human_readable_bytes(total_selected_bytes)})")
      except Exception as e:
        capture_exception(e)
        log(f"❌ Zip failed for {base_name}: {e}", "ERROR")
        report_transfer_error(device_id, drive_name, str(e))
        continue

    all_sent = bool(zip_paths_to_send)
    wormhole_code = None
    archive_name = None
    for zip_path, part_number, total_parts in zip_paths_to_send:
      part_detail = f" for archive part {part_number}/{total_parts}" if total_parts > 1 else ""
      report_transfer_progress(
        device_id,
        drive_name,
        stage="wormhole",
        detail=f"Starting wormhole send{part_detail} and waiting for a wormhole code.",
        clear_progress=True,
      )

      if total_parts > 1:
        sent, wormhole_code, archive_name = send_file_wormhole(
          zip_path,
          device_id,
          drive_name,
          requested_files,
          part_number=part_number,
          total_parts=total_parts,
          send_parts=[part for _path, part, _total in zip_paths_to_send],
        )
      else:
        sent, wormhole_code, archive_name = send_file_wormhole(
          zip_path,
          device_id,
          drive_name,
          requested_files,
        )
      try:
        if os.path.exists(zip_path):
          os.remove(zip_path)
          log(f"🧹 Deleted temporary zip: {zip_path}")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to delete zip: {e}", "WARN")
      if not sent:
        all_sent = False
        break

    for zip_path, _part_number, _total_parts in zip_paths_to_send:
      try:
        if os.path.exists(zip_path):
          os.remove(zip_path)
          log(f"🧹 Deleted unsent temporary zip: {zip_path}")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to delete zip: {e}", "WARN")

    if all_sent:
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
  use_wait_fn = False
  if stop_event is not None:
    wait_candidate = getattr(stop_event, "wait", None)
    if callable(wait_candidate):
      wait_fn = cast(Callable[[float], bool], wait_candidate)
      use_wait_fn = True
    is_set_candidate = getattr(stop_event, "is_set", None)
    if callable(is_set_candidate):
      is_set_fn = cast(Callable[[], bool], is_set_candidate)

  def _wait_interval(delay: float) -> bool:
    if use_wait_fn:
      return wait_fn(delay)
    time.sleep(delay)
    return False

  try:
    while True:
      if is_set_fn():
        break

      if not _auth_headers():
        if not _auth_wait_logged:
          log("⚠️ Route sender waiting for device JWT (registration key missing?)", "WARN")
          _auth_wait_logged = True
        if _wait_interval(CHECK_INTERVAL):
          break
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

      if _wait_interval(CHECK_INTERVAL):
        break
  finally:
    log("🚚 Route sender loop stopped", "INFO")

def main():
  run_route_sender()

if __name__ == "__main__":
  main()
