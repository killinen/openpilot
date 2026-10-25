"""GCS-controlled teletyped worker for OP-NAS, registered by default in markpilot.

Deploy with archive_lan.py and the current extensions.py. Automatic uploads stay
off until enabled per device in GCS. Explicit TELETYPED_EXTENSIONS overrides
still replace the default workers. Raw replacement is separately opted in and
retains a verified, durable rlog.bz2 on the comma before removing rlog.
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
import sys
import tempfile
import threading
import time
import zipfile
from pathlib import Path
from typing import cast

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
_PREPARE_SLOTS = threading.BoundedSemaphore(2)
_WORMHOLE_SLOTS = threading.BoundedSemaphore(2)
CONVERSION_META = ".opnas-rlog-conversion.json"


def _sync_directory(folder):
  fd = os.open(folder, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def _matches(path, descriptor):
  if path.is_symlink() or not path.is_file():
    return False
  s = path.stat()
  return (s.st_size, s.st_mtime_ns) == (descriptor["size"], descriptor["mtime_ns"])


def _conversion(folder):
  path = folder / CONVERSION_META
  if path.is_symlink():
    raise ValueError("Conversion metadata is a symbolic link")
  if not path.exists():
    return None
  if not path.is_file() or path.stat().st_size > 16384:
    raise ValueError("Invalid conversion metadata")
  try:
    record = json.loads(path.read_text())
    if record["version"] != 1 or not re.fullmatch(r"\.opnas-rlog-[a-zA-Z0-9_-]+\.bz2\.tmp", record["temporary"]):
      raise ValueError
    for key, name in (("source", "rlog"), ("stored", "rlog.bz2")):
      f = record[key]
      if (
        set(f) != {"name", "size", "mtime_ns", "sha256"}
        or f["name"] != name
        or not isinstance(f["size"], int)
        or isinstance(f["size"], bool)
        or not 0 <= f["size"] <= MAX_BYTES
        or not isinstance(f["mtime_ns"], int)
        or isinstance(f["mtime_ns"], bool)
        or f["mtime_ns"] < 0
        or not re.fullmatch(r"[a-f0-9]{64}", f["sha256"])
      ):
        raise ValueError
    if record["source"]["mtime_ns"] != record["stored"]["mtime_ns"]:
      raise ValueError
    return record
  except (KeyError, TypeError, ValueError) as exc:
    raise ValueError("Invalid conversion metadata") from exc


def _verify_stream(stream, descriptor, permitted):
  digest, count = hashlib.sha256(), 0
  for block in iter(lambda: stream.read(1024 * 1024), b""):
    if not permitted():
      raise ValueError("Offroad or network condition changed")
    count += len(block)
    if count > descriptor["size"]:
      raise ValueError("Converted source exceeds its recorded size")
    digest.update(block)
  if count != descriptor["size"] or digest.hexdigest() != descriptor["sha256"]:
    raise ValueError("Converted source checksum mismatch")


def _verify_local(path, descriptor, permitted, *, decompress=False):
  if path.is_symlink() or not path.is_file():
    raise ValueError("Converted source is missing or a symbolic link")
  with bz2.open(path, "rb") if decompress else path.open("rb") as stream:
    _verify_stream(stream, descriptor, permitted)


def _write_conversion(folder, record):
  fd, temporary = tempfile.mkstemp(prefix=".opnas-conversion-", dir=folder)
  try:
    with os.fdopen(fd, "w") as out:
      json.dump(record, out, sort_keys=True)
      out.flush()
      os.fsync(out.fileno())
    os.replace(temporary, folder / CONVERSION_META)
    _sync_directory(folder)
  finally:
    Path(temporary).unlink(missing_ok=True)


def replace_raw_rlog(folder, requested, permitted):
  """Commit verified local bzip2 and recovery metadata before removing raw data.

  The caller holds the normal segment deletion lock. Inventory retains the
  original logical identity so retries and GCS deduplication remain stable.
  """
  raw, stored = folder / "rlog", folder / "rlog.bz2"
  record = _conversion(folder)
  identity = {k: requested[k] for k in ("name", "size", "mtime_ns")}
  if record and identity != {k: record["source"][k] for k in identity}:
    raise ValueError("Conversion belongs to a different source identity")
  if record and not stored.exists() and not (folder / record["temporary"]).exists():
    # The intent was durable, but its temporary output is gone. A complete
    # original permits a fresh conversion; never remove the only copy.
    if not _matches(raw, record["source"]):
      raise ValueError("Incomplete conversion has no usable local copy")
    _verify_local(raw, record["source"], permitted)
    (folder / CONVERSION_META).unlink()
    _sync_directory(folder)
    record = None
  temporary = None
  intent_written = False
  try:
    if record is None:
      if stored.exists() or stored.is_symlink():
        raise ValueError("Existing rlog.bz2 has no matching conversion metadata")
      if not _matches(raw, identity) or time.time_ns() - identity["mtime_ns"] < STABLE_SECONDS * 10**9:
        raise ValueError("Raw rlog changed or is still active")
      before = raw.stat()
      fd, name = tempfile.mkstemp(prefix=".opnas-rlog-", suffix=".bz2.tmp", dir=folder)
      temporary = Path(name)
      source_hash, stored_hash, count = hashlib.sha256(), hashlib.sha256(), 0
      encoder = bz2.BZ2Compressor(3)
      with raw.open("rb") as src, os.fdopen(fd, "wb") as out:
        for block in iter(lambda: src.read(1024 * 1024), b""):
          if not permitted():
            raise ValueError("Offroad or network condition changed")
          source_hash.update(block)
          count += len(block)
          if count > identity["size"]:
            raise ValueError("Raw rlog changed during conversion")
          packed = encoder.compress(block)
          out.write(packed)
          stored_hash.update(packed)
        packed = encoder.flush()
        out.write(packed)
        stored_hash.update(packed)
        out.flush()
        os.fsync(out.fileno())
      after = raw.stat()
      if count != identity["size"] or (
        before.st_ino,
        before.st_dev,
        before.st_size,
        before.st_mtime_ns,
      ) != (after.st_ino, after.st_dev, after.st_size, after.st_mtime_ns):
        raise ValueError("Raw rlog changed during conversion")
      shutil.copystat(raw, temporary, follow_symlinks=False)
      os.utime(temporary, ns=(before.st_atime_ns, before.st_mtime_ns))
      with temporary.open("rb") as durable:
        os.fsync(durable.fileno())
      record = {
        "version": 1,
        "temporary": temporary.name,
        "source": {**identity, "sha256": source_hash.hexdigest()},
        "stored": {
          "name": "rlog.bz2",
          "size": temporary.stat().st_size,
          "mtime_ns": identity["mtime_ns"],
          "sha256": stored_hash.hexdigest(),
        },
      }
      if record["stored"]["size"] > MAX_BYTES:
        raise ValueError("Converted source exceeds archive size limit")
      _verify_local(temporary, record["source"], permitted, decompress=True)
      _write_conversion(folder, record)
      intent_written = True
    candidate = folder / record["temporary"]
    if not stored.exists():
      if not _matches(candidate, record["stored"]):
        raise ValueError("Incomplete conversion output changed")
      _verify_local(candidate, record["stored"], permitted)
      _verify_local(candidate, record["source"], permitted, decompress=True)
      os.link(candidate, stored, follow_symlinks=False)  # Atomic, never overwrites another file.
      _sync_directory(folder)
    if not _matches(stored, record["stored"]):
      raise ValueError("Local rlog.bz2 changed")
    _verify_local(stored, record["stored"], permitted)
    _verify_local(stored, record["source"], permitted, decompress=True)
    if raw.exists() or raw.is_symlink():
      if not _matches(raw, record["source"]):
        raise ValueError("Raw rlog changed; retaining both copies")
      before = raw.stat()
      _verify_local(raw, record["source"], permitted)
      after = raw.stat()
      if not permitted() or (
        before.st_ino,
        before.st_dev,
        before.st_size,
        before.st_mtime_ns,
      ) != (after.st_ino, after.st_dev, after.st_size, after.st_mtime_ns):
        raise ValueError("Raw rlog changed; retaining both copies")
      raw.unlink()
      _sync_directory(folder)
    if candidate.exists() and not candidate.is_symlink() and candidate.stat().st_ino == stored.stat().st_ino:
      candidate.unlink()
      _sync_directory(folder)
    return record
  finally:
    if temporary is not None and not intent_written:
      temporary.unlink(missing_ok=True)


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
        try:
          conversion = _conversion(segment)
          converted = bool(conversion and _matches(segment / "rlog.bz2", conversion["stored"]))
          if conversion:
            raw = segment / "rlog"
            if raw.exists() or raw.is_symlink():
              changing |= not _matches(raw, conversion["source"])
            elif not converted:
              changing = True
        except ValueError:
          conversion, converted, changing = None, False, True
        for p in entries:
          if p.name not in ALLOWED or p.is_symlink() or not p.is_file():
            continue
          if converted and p.name in {"rlog", "rlog.bz2"}:
            continue
          s = p.stat()
          changing |= time.time() - s.st_mtime < STABLE_SECONDS
          files.append({"name": p.name, "size": s.st_size, "mtime_ns": s.st_mtime_ns})
        if converted:
          files.append({k: conversion["source"][k] for k in ("name", "size", "mtime_ns")})
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
    "supports_local_rlog_bz2": True,
    "offroad": offroad,
    "network": network,
    "segments": segments[-10000:],
  }


def scan_inventory(root: Path, offroad: bool, network: str) -> dict:
  """Avoid sharing filesystem scans with CPU-heavy TeleTyped log analysis.

  Only the interpreter's standard library is needed, including under the
  isolated guardian runtime. No device credentials are passed to the helper.
  """
  result = subprocess.run(
    [
      sys.executable,
      "-I",
      "-S",
      "-B",
      str(Path(__file__).resolve()),
      "--inventory",
      str(root),
      "1" if offroad else "0",
      network,
    ],
    check=True,
    capture_output=True,
    text=True,
    timeout=20,
  )
  return cast(dict, json.loads(result.stdout))


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
  replace_raw = job.get("constraints", {}).get("replace_raw_rlogs") is True
  if replace_raw and not compress:
    raise ValueError("Raw replacement requires rlog compression")
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
        conversion = None
        if f["name"] == "rlog":
          conversion = replace_raw_rlog(folder, f, permitted) if replace_raw else _conversion(folder)
          if conversion:
            if any(f[k] != conversion["source"][k] for k in ("name", "size", "mtime_ns")):
              raise ValueError("Converted source identity changed")
            if not _matches(folder / "rlog.bz2", conversion["stored"]):
              if not _matches(folder / "rlog", f):
                raise ValueError("Converted source is unavailable")
              conversion = None
        src = folder / f["name"]
        physical = f
        if conversion:
          src = folder / "rlog.bz2"
          physical = conversion["stored"]
        if src.is_symlink():
          raise ValueError("Source file is a symbolic link")
        before = src.stat()
        if before.st_size != physical["size"] or before.st_mtime_ns != physical["mtime_ns"] or time.time() - before.st_mtime < STABLE_SECONDS:
          raise ValueError("Source file changed or is still active")
        encoder = bz2.BZ2Compressor(3) if name != f["name"] and not conversion else None
        source_hash, stored_hash = hashlib.sha256(), hashlib.sha256()
        source_size, stored_size = 0, 0
        with bz2.open(src, "rb") if conversion and not compress else src.open("rb") as data, z.open("files/" + name, "w", force_zip64=True) as out:
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
        expected = conversion["stored" if compress else "source"] if conversion else f
        if source_size != expected["size"] or (
          before.st_size,
          before.st_mtime_ns,
          before.st_ino,
        ) != (after.st_size, after.st_mtime_ns, after.st_ino):
          raise ValueError("Source file changed while packaging")
        if conversion and source_hash.hexdigest() != expected["sha256"]:
          raise ValueError("Converted source checksum mismatch")
        entry = {
          **f,
          "name": name,
          "size": stored_size,
          "sha256": stored_hash.hexdigest(),
        }
        if encoder:
          entry["source"] = {**f, "sha256": source_hash.hexdigest()}
        elif conversion and compress:
          entry["source"] = conversion["source"]
        manifest.append(entry)
      if sum(f["size"] for f in manifest) > MAX_BYTES:
        raise ValueError("Prepared files exceed archive size limit")
      z.writestr(
        "manifest.json",
        json.dumps(
          {
            "job_id": job["id"],
            "device_id": job["device_id"],
            "segment": job["segment"],
            "files": manifest,
          }
        ),
      )
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
  if job.get("constraints", {}).get("allow_lan") is not True or network not in {
    "wifi",
    "ethernet",
  }:
    return None, None
  address = source.lan_address()
  if not address:
    return None, None
  return stack.enter_context(source.serve_archive(archive, address, permitted))


class ArchiveAPI:
  """One persistent HTTPS session per worker, with existing JWT and DNS support."""

  def __init__(self, base, headers, helper=None):
    self.base, self.headers, self.helper = base, headers, helper
    self.local = threading.local()
    self.sessions, self.lock = [], threading.Lock()

  def request(self, method, path, payload=None):
    import requests
    from urllib.parse import urlsplit

    session = getattr(self.local, "session", None)
    if session is None:
      session = self.local.session = requests.Session()
      with self.lock:
        self.sessions.append(session)
    headers = self.headers()
    if not headers:
      raise ValueError("Device JWT is unavailable")
    # Reuse TLS connections and avoid the generic helper's extra synchronous
    # DNS refresh after every successful HTTP operation.
    host = urlsplit(self.base).hostname
    cached = self.helper.get_cached_ip(host) if self.helper and time.monotonic() >= getattr(self.local, "skip_cached_until", 0) else None
    context = self.helper._force_getaddrinfo(host, cached) if cached else ExitStack()
    try:
      with context:
        response = session.request(
          method,
          self.base + path,
          headers=headers,
          json=payload,
          timeout=(3, 10),
        )
    except (requests.ConnectionError, requests.Timeout):
      self.local.skip_cached_until = time.monotonic() + 60
      raise
    try:
      response.raise_for_status()
      return response.json()
    finally:
      response.close()

  def post(self, path, payload):
    return self.request("POST", path, payload)

  def get(self, path):
    return self.request("GET", path)

  def close(self):
    for session in self.sessions:
      session.close()


def transfer_job(root, job, client, facts, binary, source, stop):
  last_check, cached, network = 0.0, False, "unknown"

  def permitted():
    nonlocal last_check, cached, network
    if stop.is_set():
      return False
    if time.monotonic() - last_check >= 1:
      offroad, network = facts()
      cached = offroad and (job["constraints"]["network_policy"] == "any" or network in {"wifi", "ethernet"})
      last_check = time.monotonic()
    return cached

  path_base = "/jobs/" + job["id"]
  try:
    if not permitted():
      raise ValueError("Upload conditions are not satisfied")
    temp_root = root.parent / ".opnas-incoming"
    temp_root.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=job["id"] + "-", dir=temp_root) as temporary, ExitStack() as stack:
      archive = Path(temporary) / "segment.zip"
      # Keep two offers prepared ahead of the NAS's two receiving slots,
      # while bounding CPU-heavy compression to two workers.
      with _PREPARE_SLOTS:
        manifest = package(root, job, archive, permitted)
      lan_offer = None
      try:
        lan_offer, _served = local_offer(
          source,
          stack,
          archive,
          job,
          network,
          lambda: permitted() and network in {"wifi", "ethernet"},
        )
      except Exception:
        pass

      def announce(code):
        if not permitted():
          raise ValueError("Upload conditions changed")
        client.post(
          path_base + "/offer",
          {"wormhole_code": code, "files": manifest, "lan_offer": lan_offer},
        )

      last_status = 0.0
      status = {}

      def receipt(force=False):
        nonlocal last_status, status
        if force or time.monotonic() - last_status >= 1:
          import requests

          try:
            status = client.get(path_base)
          except (requests.ConnectionError, requests.Timeout):
            pass
          last_status = time.monotonic()
        if status.get("status") in {"failed", "cancelled"}:
          raise ValueError("Archive job was stopped by GCS")
        return status.get("status") == "received"

      if lan_offer and job["constraints"].get("lan_only") is True:
        # Publish the local offer immediately. Start Wormhole only when
        # the receiver explicitly requests a fallback for this same job.
        announce(None)
        deadline = time.monotonic() + 7200
        while permitted() and time.monotonic() < deadline:
          if receipt():
            return True
          if status.get("fallback_requested"):
            break
          stop.wait(0.25)
        else:
          raise ValueError("Direct transfer stopped or timed out")
      with _WORMHOLE_SLOTS:
        send_archive(
          binary,
          archive,
          announce,
          permitted,
          # Also observe failure/cancellation while Wormhole waits.
          completed=receipt,
        )
      # A successful Wormhole byte transfer still needs its verified receipt
      # before the same device starts enough work to exceed its slot limit.
      deadline = time.monotonic() + 120
      while permitted() and time.monotonic() < deadline:
        if receipt():
          return True
        stop.wait(0.25)
      raise ValueError("Verified receipt was not confirmed")
  except Exception:
    try:
      client.post(
        path_base + "/failed",
        {"detail": "Sender stopped or transfer failed; source recordings retained"},
      )
    except Exception:
      pass
    return False


def run_pipeline(root, device_id, client, facts, binary, source, stop, workers=2):
  from concurrent.futures import ThreadPoolExecutor
  import uuid

  futures = {}
  next_inventory = next_claim = 0.0
  claim_id = None
  try:
    with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="archive-send") as pool:
      try:
        while not stop.is_set():
          offroad, network = facts()
          if not offroad:
            break
          now = time.monotonic()
          for future in list(futures):
            if future.done():
              succeeded = future.result()
              del futures[future]
              # Keep the full inventory on its own cadence. Sending
              # thousands of file identities before each next claim
              # leaves the LAN idle; GCS releases reservations on receipt.
              next_claim = 0.0 if succeeded else time.monotonic() + 2
          try:
            if now >= next_inventory:
              inventory = scan_inventory(root, offroad, network)
              offroad, network = facts()
              if not offroad:
                break
              inventory.update(offroad=offroad, network=network)
              temporary_root = root.parent / ".opnas-incoming"
              allocated = 0
              for active_job in futures.values():
                planned = sum(f["size"] for f in active_job["files"]) + 64 * 1024**2
                for folder in temporary_root.glob(active_job["id"] + "-*"):
                  archive = folder / "segment.zip"
                  try:
                    allocated += min(archive.stat().st_size, planned)
                  except OSError:
                    pass
              inventory.update(
                max_parallel_transfers=workers,
                supports_lan_only=True,
                temporary_available_bytes=max(
                  0,
                  shutil.disk_usage(root.parent).free + allocated - 512 * 1024**2,
                ),
              )
              client.post("/inventory", inventory)
              next_inventory = time.monotonic() + 30
            while len(futures) < workers and time.monotonic() >= next_claim and not stop.is_set():
              claim_id = claim_id or uuid.uuid4().hex
              job = client.post("/claim", {"request_id": claim_id}).get("job")
              claim_id = None
              if not job:
                next_claim = time.monotonic() + (2 if futures else 5)
                break
              futures[
                pool.submit(
                  transfer_job,
                  root,
                  job,
                  client,
                  facts,
                  binary,
                  source,
                  stop,
                )
              ] = job
          except Exception:
            next_inventory = min(next_inventory, time.monotonic() + 3)
            next_claim = time.monotonic() + 3
            stop.wait(1)
          stop.wait(0.1 if futures else 1)
      finally:
        stop.set()  # Cancel active transfers if conditions change or shutdown starts.
  finally:
    client.close()


def archive_step(device_id: str, headers: dict) -> None:
  """Drain available archive work; only an empty queue incurs an idle backoff."""
  if not _GUARD.acquire(blocking=False):
    return
  try:
    from openpilot.tools.teletyped import helper, health, archive_lan
    from openpilot.common.params import Params
    from openpilot.system.hardware import HARDWARE
    from cereal import log

    params = Params()

    def facts():
      offroad = params.get("IsOnroad") == b"0" and health.offroad()
      raw = HARDWARE.get_network_type()
      kind = log.DeviceState.NetworkType
      network = "wifi" if raw == kind.wifi else "ethernet" if raw == kind.ethernet else "unknown" if raw == kind.none else "cellular"
      return offroad, network

    client = ArchiveAPI(
      helper.API_URL.rstrip("/") + "/archive/device/" + device_id,
      helper.build_auth_headers,
      helper,
    )
    run_pipeline(
      Path(helper.REALDATA_DIR),
      device_id,
      client,
      facts,
      helper.WORMHOLE_BINARY,
      archive_lan,
      threading.Event(),
      workers=4,
    )
  finally:
    _GUARD.release()


if __name__ == "__main__":
  import argparse

  parser = argparse.ArgumentParser(description="Internal OP-NAS inventory scanner")
  parser.add_argument("--inventory", action="store_true", required=True)
  parser.add_argument("root", type=Path)
  parser.add_argument("offroad", choices=["0", "1"])
  parser.add_argument("network", choices=["wifi", "ethernet", "cellular", "unknown"])
  args = parser.parse_args()
  print(json.dumps(collect_inventory(args.root, args.offroad == "1", args.network)))
