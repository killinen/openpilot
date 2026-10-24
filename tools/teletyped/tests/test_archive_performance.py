"""Sender instrumentation must preserve the ordinary bzip2 archive contract."""

import bz2
import hashlib
import importlib.util
import os
from pathlib import Path
import threading
import time
import zipfile

import pytest


@pytest.fixture
def sender():
  spec = importlib.util.spec_from_file_location("measured_archive_sender", Path(__file__).parents[1] / "archive_sender.py")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_measured_compression_retains_raw_and_round_trips(sender, tmp_path):
  root = tmp_path / "logs"
  folder = root / "benchmark--0"
  folder.mkdir(parents=True)
  raw = folder / "rlog"
  original = b"openpilot rlog sample\n" * 50000
  raw.write_bytes(original)
  os.utime(raw, (time.time() - 300,) * 2)
  stat = raw.stat()
  job = {
    "id": "a" * 32,
    "device_id": "car",
    "segment": folder.name,
    "files": [{"name": "rlog", "size": stat.st_size, "mtime_ns": stat.st_mtime_ns}],
    "constraints": {"compress_rlogs": True, "replace_raw_rlogs": False},
  }
  metrics = {}
  manifest = sender.package(root, job, tmp_path / "archive.zip", lambda: True, metrics)
  with zipfile.ZipFile(tmp_path / "archive.zip") as archive:
    assert bz2.decompress(archive.read("files/rlog.bz2")) == original
  assert raw.read_bytes() == original
  assert manifest[0]["source"]["sha256"] == hashlib.sha256(original).hexdigest()
  assert metrics["compression_raw_bytes"] == len(original)
  assert metrics["compression_output_bytes"] == manifest[0]["size"]
  assert metrics["package_seconds"] >= metrics["compression_seconds"] > 0
  assert not list(folder.glob("*.lock"))


def test_preparation_wait_observes_cancellation(sender, monkeypatch):
  slots = threading.BoundedSemaphore(1)
  slots.acquire()
  monkeypatch.setattr(sender, "_PREPARE_SLOTS", slots)
  with pytest.raises(ValueError, match="waiting"):
    with sender.preparation_slot(lambda: False):
      pytest.fail("Cancelled preparation acquired a slot")
  slots.release()
  assert slots.acquire(blocking=False)


@pytest.mark.parametrize("temperature,memory,allowed", [(40, 1024**3, True), (70, 1024**3, False), (40, 100, False), (None, 1024**3, False)])
def test_resource_guard_fails_closed(sender, temperature, memory, allowed):
  assert sender.resources_permit({"cpu_celsius": temperature, "available_bytes": memory}) is allowed
