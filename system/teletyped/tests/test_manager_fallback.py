"""Exercise real manager process classes with unrelated platform services stubbed."""
import importlib.util
from pathlib import Path
import sys
import types

import pytest

REPO = Path(__file__).resolve().parents[3]


def load_file(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def manager_process(monkeypatch):
  monkeypatch.syspath_prepend(str(REPO))
  # Load the empty namespace parents before injecting their platform leaves.
  for package in ("openpilot.system", "openpilot.common", "openpilot.system.manager"):
    __import__(package)
  dependencies = {
    "cereal": {"car": types.SimpleNamespace(CarParams=object), "log": types.SimpleNamespace()},
    "cereal.messaging": {},
    "setproctitle": {"setproctitle": lambda _: None},
    "openpilot.system.sentry": {},
    "openpilot.common.basedir": {"BASEDIR": str(REPO)},
    "openpilot.common.params": {"Params": object},
    "openpilot.common.swaglog": {"cloudlog": types.SimpleNamespace(info=lambda *_: None)},
    "openpilot.system.hardware": {"PC": False, "TICI": True},
  }
  for name, attributes in dependencies.items():
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    monkeypatch.setitem(sys.modules, name, module)
  module = load_file("fallback_process_test", REPO / "system/manager/process.py")
  monkeypatch.setitem(sys.modules, "openpilot.system.manager.process", module)
  return module


def test_prepare_does_not_import_broken_teletyped(manager_process, monkeypatch):
  config = load_file("fallback_config_test", REPO / "system/manager/process_config.py")
  teletyped = config.managed_processes["teletyped"]
  assert isinstance(teletyped, manager_process.NativeProcess)
  assert teletyped.restart_on_exit
  original = manager_process.importlib.import_module

  def reject_candidate(name, *args, **kwargs):
    if "teletyped" in name:
      raise SyntaxError("broken development code")
    return original(name, *args, **kwargs)

  monkeypatch.setattr(manager_process.importlib, "import_module", reject_candidate)
  teletyped.prepare()
  assert "tools.teletyped.teletyped" in teletyped.cmdline


def test_exited_fallback_is_reaped_and_restarted_with_delay(manager_process):
  process = manager_process.NativeProcess("fixture", "", [sys.executable, "-c", "raise SystemExit(23)"], lambda *_: True,
                                          restart_on_exit=True)
  try:
    process.start()
    process.proc.join(timeout=5)
    assert process.proc.exitcode == 23
    old_pid = process.proc.pid
    process.start()
    assert process.proc is None
    process.start()
    assert process.proc is None
    process.next_restart = 0  # Advance past the explicit five-second delay.
    process.start()
    assert process.proc.pid != old_pid
    process.proc.join(timeout=5)
    assert process.proc.exitcode == 23
  finally:
    process.stop()


def test_other_native_processes_retain_existing_policy(manager_process):
  process = manager_process.NativeProcess("fixture", "", [sys.executable, "-c", "pass"], lambda *_: True)
  try:
    process.start()
    process.proc.join(timeout=5)
    assert process.proc.exitcode == 0
    old_pid = process.proc.pid
    process.start()
    assert process.proc.pid == old_pid
  finally:
    process.stop()
