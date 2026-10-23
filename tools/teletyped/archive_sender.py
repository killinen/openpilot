"""GCS-controlled teletyped worker for OP-NAS, registered by default in markpilot.

Deploy with archive_lan.py and the current extensions.py. Automatic uploads stay
off until enabled per device in GCS. Explicit TELETYPED_EXTENSIONS overrides
still replace the default workers. This module never deletes source recordings.
"""

from __future__ import annotations

import bz2
import hashlib
from contextlib import ExitStack
import json
import os
import re
import select
import shutil
import subprocess
import tempfile
import threading
import time
import zipfile
from pathlib import Path

ALLOWED = {
  "rlog",
  "rlog.bz2",
  "rlog.zst",
  "qlog",
  "qlog.bz2",
  "qlog.zst",
  "fcamera.hevc",
  "ecamera.hevc",
  "dcamera.hevc",
  "qcamera.ts",
}
MAX_BYTES = 4 * 1024**3
STABLE_SECONDS = 120
_GUARD = threading.Lock()


def collect_inventory(root: Path, offroad: bool, network: str) -> dict:
  segments = []
  if root.is_dir():
    for segment in sorted(root.iterdir()):
      if not segment.is_dir() or segment.is_symlink() or not re.fullmatch(r"[a-zA-Z0-9_-][a-zA-Z0-9_.-]{0,159}", segment.name):
        continue
      files, changing = [], False
      try:
        entries = list(segment.iterdir())
        for old in entries:
          match = re.fullmatch(r"\.opnas-archive-([0-9]+)\.lock", old.name)
          if match and not old.is_symlink():
            try:
              os.kill(int(match[1]), 0)
            except ProcessLookupError:
              old.unlink(missing_ok=True)
            except PermissionError:
              pass
        entries = list(segment.iterdir())
        locked = any(p.name.endswith(".lock") for p in entries)
        for p in entries:
          if p.name not in ALLOWED or p.is_symlink() or not p.is_file():
            continue
          s = p.stat()
          changing |= time.time() - s.st_mtime < STABLE_SECONDS
          files.append({"name": p.name, "size": s.st_size, "mtime_ns": s.st_mtime_ns})
        if files:
          segments.append(
            {
              "name": segment.name,
              "closed": bool(offroad and not locked and not changing),
              "files": files,
            }
          )
      except OSError:
        continue
  return {
    "protocol": 1,
    "supports_rlog_bz2": True,
    "offroad": offroad,
    "network": network,
    "segments": segments[-10000:],
  }


def package(root: Path, job: dict, output: Path, permitted) -> list[dict]:
  if not re.fullmatch(r"[a-zA-Z0-9_-][a-zA-Z0-9_.-]{0,159}", job["segment"]):
    raise ValueError("Invalid segment")
  folder = root / job["segment"]
  if folder.is_symlink() or folder.resolve().parent != root.resolve():
    raise ValueError("Invalid source path")
  if sum(f["size"] for f in job["files"]) > MAX_BYTES:
    raise ValueError("Segment exceeds archive size limit")
  if shutil.disk_usage(output.parent).free < sum(f["size"] for f in job["files"]) + 512 * 1024**2:
    raise ValueError("Insufficient temporary storage")
  compress = job.get("constraints", {}).get("compress_rlogs") is True
  names = ["rlog.bz2" if compress and f["name"] == "rlog" else f["name"] for f in job["files"]]
  if len(set(names)) != len(names):
    raise ValueError("Compressed filename conflicts with another requested file")
  lock = folder / (".opnas-archive-" + str(os.getpid()) + ".lock")
  if any(p.name.endswith(".lock") for p in folder.iterdir()):
    raise ValueError("Source segment is locked")
  fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
  os.close(fd)
  manifest = []
  try:
    # ZIP only bundles files. A converted entry contains a complete, ordinary
    # bzip2 stream, which the receiver stores without recompressing it.
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as z:
      for f, name in zip(job["files"], names, strict=True):
        if f["name"] not in ALLOWED:
          raise ValueError("Unsupported file selection")
        src = folder / f["name"]
        if src.is_symlink():
          raise ValueError("Source file is a symbolic link")
        before = src.stat()
        if before.st_size != f["size"] or before.st_mtime_ns != f["mtime_ns"] or time.time() - before.st_mtime < STABLE_SECONDS:
          raise ValueError("Source file changed or is still active")
        encoder = bz2.BZ2Compressor(3) if name != f["name"] else None
        source_hash, stored_hash = hashlib.sha256(), hashlib.sha256()
        source_size, stored_size = 0, 0
        with src.open("rb") as data, z.open("files/" + name, "w", force_zip64=True) as out:
          for block in iter(lambda: data.read(1024 * 1024), b""):
            if not permitted():
              raise ValueError("Offroad or network condition changed")
            source_hash.update(block)
            source_size += len(block)
            packed = encoder.compress(block) if encoder else block
            out.write(packed)
            stored_hash.update(packed)
            stored_size += len(packed)
          if encoder:
            if not permitted():
              raise ValueError("Offroad or network condition changed")
            packed = encoder.flush()
            out.write(packed)
            stored_hash.update(packed)
            stored_size += len(packed)
        after = src.stat()
        if source_size != f["size"] or (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
          raise ValueError("Source file changed while packaging")
        entry = {**f, "name": name, "size": stored_size, "sha256": stored_hash.hexdigest()}
        if encoder:
          entry["source"] = {**f, "sha256": source_hash.hexdigest()}
        manifest.append(entry)
      if sum(f["size"] for f in manifest) > MAX_BYTES:
        raise ValueError("Prepared files exceed archive size limit")
      z.writestr("manifest.json", json.dumps({"job_id": job["id"], "device_id": job["device_id"], "segment": job["segment"], "files": manifest}))
    return manifest
  finally:
    lock.unlink(missing_ok=True)


def send_archive(
  binary: str,
  archive: Path,
  announce,
  permitted,
  timeout=7200,
  completed=lambda: False,
):
  process = subprocess.Popen(
    [binary, "send", str(archive)],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    stdin=subprocess.DEVNULL,
  )
  started, pending, announced = time.monotonic(), b"", False
  try:
    assert process.stdout is not None
    while process.poll() is None:
      if announced and completed():
        return  # Verified LAN receipt; stop the unused Wormhole offer.
      if not permitted() or time.monotonic() - started > timeout:
        raise ValueError("Transfer stopped because conditions changed or time limit was reached")
      ready, _, _ = select.select([process.stdout], [], [], 1)
      if not ready:
        continue
      block = os.read(process.stdout.fileno(), 65536)
      if not block:
        continue
      pending = (pending + block)[-65536:]
      match = re.search(rb"Wormhole code is:\s*([0-9]+-[a-z]+-[a-z]+)", pending)
      if match and not announced:
        announce(match[1].decode())
        announced = True
    if process.returncode or not announced:
      raise ValueError("Wormhole sender failed")
  finally:
    if process.poll() is None:
      process.terminate()
      try:
        process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def local_offer(source, stack, archive: Path, job: dict, network: str, permitted):
  if job.get("constraints", {}).get("allow_lan") is not True or network not in {"wifi", "ethernet"}:
    return None, None
  address = source.lan_address()
  if not address:
    return None, None
  return stack.enter_context(source.serve_archive(archive, address, permitted))


def archive_step(device_id: str, headers: dict) -> None:
  """Teletyped optional-worker entry point; all hardware imports stay on device."""
  if not _GUARD.acquire(blocking=False):
    return
  try:
    from openpilot.tools.teletyped import helper, health
    from openpilot.common.params import Params
    from openpilot.system.hardware import HARDWARE
    from cereal import log
    from openpilot.tools.teletyped.helper import http_get, http_post
    from openpilot.tools.teletyped import archive_lan

    root = Path(helper.REALDATA_DIR)
    base = helper.API_URL.rstrip("/") + "/archive/device/" + device_id
    params = Params()

    def facts():
      # Fail closed when manager state/Params is unavailable; health.offroad()
      # alone permits legacy configurations without a live manager snapshot.
      offroad = params.get("IsOnroad") == b"0" and health.offroad()
      raw = HARDWARE.get_network_type()
      kind = log.DeviceState.NetworkType
      network = "wifi" if raw == kind.wifi else "ethernet" if raw == kind.ethernet else "unknown" if raw == kind.none else "cellular"
      return offroad, network

    def post(path, data):
      auth = helper.build_auth_headers()
      if not auth:
        raise ValueError("Device JWT is unavailable")
      response = http_post(base + path, headers=auth, json=data, timeout=15)
      response.raise_for_status()
      return response.json()

    def inventory():
      offroad, network = facts()
      post("/inventory", collect_inventory(root, offroad, network))
      return offroad, network

    offroad, _ = inventory()
    if not offroad:
      return
    job = post("/claim", {}).get("job")
    if not job:
      return
    stop = threading.Event()

    def updates():
      while not stop.wait(30):
        try:
          inventory()
        except Exception:
          pass

    thread = threading.Thread(target=updates, daemon=True)
    thread.start()
    last_check, cached, cached_network = 0.0, False, "unknown"

    def permitted():
      nonlocal last_check, cached, cached_network
      if time.monotonic() - last_check > 1:
        offroad, network = facts()
        cached_network = network
        cached = offroad and (job["constraints"]["network_policy"] == "any" or network in {"wifi", "ethernet"})
        last_check = time.monotonic()
      return cached

    try:
      if not permitted():
        raise ValueError("Upload conditions are not satisfied")
      temp_root = Path(helper.REALDATA_DIR).parent / ".opnas-incoming"
      temp_root.mkdir(exist_ok=True)
      for stale in temp_root.iterdir():
        if re.fullmatch(r"[a-f0-9]{32}-.+", stale.name) and stale.is_dir() and not stale.is_symlink() and time.time() - stale.stat().st_mtime > 10800:
          shutil.rmtree(stale)
      with tempfile.TemporaryDirectory(prefix=job["id"] + "-", dir=temp_root) as temporary, ExitStack() as stack:
        path = Path(temporary) / "segment.zip"
        manifest = package(root, job, path, permitted)
        lan_offer, served = None, None
        _, network = facts()
        try:
          lan_offer, served = local_offer(
            archive_lan,
            stack,
            path,
            job,
            network,
            lambda: permitted() and cached_network in {"wifi", "ethernet"},
          )
        except Exception:
          helper.log("Direct archive service unavailable; using Wormhole", "WARN")
        last_status_check = 0.0

        def completed():
          nonlocal last_status_check
          if served is None or not served.is_set() or time.monotonic() - last_status_check < 5:
            return False
          last_status_check = time.monotonic()
          try:
            response = http_get(
              base + "/jobs/" + job["id"],
              headers=helper.build_auth_headers(),
              timeout=5,
            )
            response.raise_for_status()
            status = response.json().get("status")
          except Exception:
            return False  # Keep offering the fallback until a verified receipt exists.
          if status in {"failed", "cancelled"}:
            raise ValueError("Archive job was stopped by GCS")
          return status == "received"

        def announce(code):
          if not permitted():
            raise ValueError("Upload conditions changed before offer")
          post(
            "/jobs/" + job["id"] + "/offer",
            {
              "wormhole_code": code,
              "files": manifest,
              "lan_offer": lan_offer,
            },
          )

        send_archive(
          helper.WORMHOLE_BINARY,
          path,
          announce,
          permitted,
          completed=completed,
        )
    except Exception:
      # Never log JWTs, credentials, one-time codes or file contents.
      try:
        post(
          "/jobs/" + job["id"] + "/failed",
          {"detail": "Device sender stopped; source changed, conditions failed, or transfer failed"},
        )
      except Exception:
        pass
      helper.log(
        "Automatic archive job deferred or failed; source files retained",
        "WARN",
      )
    finally:
      stop.set()
      thread.join(timeout=20)
  finally:
    _GUARD.release()
