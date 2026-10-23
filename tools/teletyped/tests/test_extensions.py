import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


@pytest.fixture
def extensions(monkeypatch):
  # Test worker registration without importing device hardware or auth helpers.
  helper = SimpleNamespace(capture_exception=lambda error: None, log=lambda *args: None)
  monkeypatch.setitem(sys.modules, "openpilot.tools.teletyped.helper", helper)
  spec = importlib.util.spec_from_file_location("teletyped_extensions_test", Path(__file__).parents[1] / "extensions.py")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_defaults_start_telemetry_and_archive_with_auth(monkeypatch, extensions):
  monkeypatch.delenv("TELETYPED_EXTENSIONS", raising=False)
  calls = []
  modules = {
    "openpilot.tools.vehicle_telemetry.worker": SimpleNamespace(telemetry_step=lambda device, headers: calls.append(("telemetry", device, headers))),
    "openpilot.tools.teletyped.archive_sender": SimpleNamespace(archive_step=lambda device, headers: calls.append(("archive", device, headers))),
  }
  monkeypatch.setattr(extensions, "import_module", modules.__getitem__)
  headers = {}
  workers = extensions.optional_workers(lambda: headers)
  assert len(workers) == 2
  for callback, _, _ in workers:
    callback("device")
  assert calls == []
  headers["X-Device-JWT"] = "test"
  for callback, _, _ in workers:
    callback("device")
  assert calls == [("telemetry", "device", headers), ("archive", "device", headers)]


def test_archive_starts_even_if_default_telemetry_is_unavailable(monkeypatch, extensions):
  monkeypatch.delenv("TELETYPED_EXTENSIONS", raising=False)
  calls, errors = [], []

  def load(name):
    if name == "openpilot.tools.vehicle_telemetry.worker":
      raise ImportError("Telemetry unavailable")
    assert name == "openpilot.tools.teletyped.archive_sender"
    return SimpleNamespace(archive_step=lambda device, headers: calls.append(device))

  monkeypatch.setattr(extensions, "import_module", load)
  monkeypatch.setattr(extensions, "capture_exception", errors.append)
  workers = extensions.optional_workers(lambda: {"X-Device-JWT": "test"})
  assert len(workers) == 1
  workers[0][0]("device")
  assert calls == ["device"]
  assert len(errors) == 1


def test_disabled_and_missing_extensions_are_isolated(monkeypatch, extensions):
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "")
  assert extensions.optional_workers(dict) == []
  errors = []
  monkeypatch.setattr(extensions, "capture_exception", errors.append)
  monkeypatch.setattr(extensions, "log", lambda *args: None)
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "missing.module:step")
  assert extensions.optional_workers(dict) == []
  assert len(errors) == 1


def test_extension_callbacks_keep_their_own_binding_and_auth(monkeypatch, extensions):
  calls = []
  module = SimpleNamespace(first=lambda device, headers: calls.append((1, device, headers)), second=lambda device, headers: calls.append((2, device, headers)))
  monkeypatch.setattr(extensions, "import_module", lambda name: module)
  monkeypatch.setenv("TELETYPED_EXTENSIONS", "demo:first,demo:second")
  headers = {}
  workers = extensions.optional_workers(lambda: headers)
  workers[0][0]("device")
  assert calls == []
  headers["X-Device-JWT"] = "test"
  for callback, _, _ in workers:
    callback("device")
  assert [call[0] for call in calls] == [1, 2]
