import json
import time
from types import SimpleNamespace

import pytest

from openpilot.tools.teletyped import health, teletyped


def test_failed_initial_fetch_keeps_surviving_tunnel(monkeypatch):
  monkeypatch.setattr(teletyped, "_last_desired_tunnel_request", None)
  monkeypatch.setattr(teletyped, "_last_tunnel_config", None)
  monkeypatch.setattr(teletyped, "fetch_ssh_request", lambda _: None)
  monkeypatch.setattr(teletyped, "get_current_tunnel_status", lambda: "connected")
  monkeypatch.setattr(teletyped, "stop_tunnel", lambda: pytest.fail("working recovery tunnel was stopped"))
  assert teletyped.reverse_ssh_step("device", "unknown") == "connected"


def test_startup_exception_still_releases_tunnel(monkeypatch):
  stopped = []

  def crash():
    raise RuntimeError("startup failed before heartbeat thread")

  monkeypatch.setattr(teletyped, "_main", crash)
  monkeypatch.setattr(teletyped, "stop_tunnel", lambda: stopped.append(True))
  with pytest.raises(RuntimeError, match="startup"):
    teletyped.main()
  assert stopped == [True]


def test_health_distinguishes_main_progress_from_heartbeat(monkeypatch, tmp_path):
  status = tmp_path / "health.json"
  monkeypatch.setenv("TELETYPED_HEALTH_PATH", str(status))
  monkeypatch.setenv("TELETYPED_RUN_ID", "run")
  monkeypatch.setenv("TELETYPED_RELEASE_ID", "candidate")
  monkeypatch.setattr(health, "_state", {})
  health.progress()
  first = json.loads(status.read_text())
  health.success("heartbeat")
  latest = json.loads(status.read_text())
  assert latest["main_progress"] == first["main_progress"]
  assert latest["heartbeat_ok"] <= time.monotonic()
  assert latest["run_id"] == "run"
  assert latest["release_id"] == "candidate"


def test_unknown_or_onroad_state_suppresses_bulk_jobs(monkeypatch, tmp_path):
  monkeypatch.setenv("TELETYPED_DEPLOYMENT_ROOT", str(tmp_path))
  state = tmp_path / "manager.json"
  monkeypatch.setenv("TELETYPED_MANAGER_STATE", str(state))
  assert not health.offroad()
  state.write_text(json.dumps({"offroad": True, "progress": time.monotonic()}))
  assert health.offroad()
  state.write_text(json.dumps({"offroad": False, "progress": time.monotonic()}))
  assert not health.offroad()
  state.write_text(json.dumps({"offroad": True, "progress": time.monotonic() - 10}))
  assert not health.offroad()


def test_rejected_heartbeat_is_not_reported_healthy(monkeypatch):
  calls = []
  monkeypatch.setattr(teletyped, "has_internet_connection", lambda: True)
  monkeypatch.setattr(teletyped, "build_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(teletyped, "get_hardware_info", dict)
  monkeypatch.setattr(teletyped, "get_os_info", dict)
  monkeypatch.setattr(teletyped, "get_op_params_info", dict)
  monkeypatch.setattr(teletyped, "_read_tunnel_pid", lambda: None)
  monkeypatch.setattr(health, "success", calls.append)

  class Response:
    status_code = 401
    text = "rejected"

  monkeypatch.setattr(teletyped, "http_post", lambda *_, **__: Response())
  teletyped.send_heartbeat("device", "connected")
  assert calls == []
  Response.status_code = 200
  teletyped.send_heartbeat("device", "connected")
  assert calls == ["heartbeat"]


def test_manager_publishes_fresh_state_without_importing_teletyped(monkeypatch, tmp_path):
  from openpilot.system.manager import helpers

  marker, output = tmp_path / "installed.json", tmp_path / "state.json"
  helpers.publish_teletyped_manager_state(False, marker, output)
  assert not output.exists()
  marker.touch()
  helpers.publish_teletyped_manager_state(False, marker, output)
  state = json.loads(output.read_text())
  assert state["offroad"] is True
  assert 0 <= time.monotonic() - state["progress"] < 1
  helpers.publish_teletyped_manager_state(True, marker, output)
  assert json.loads(output.read_text())["offroad"] is False

  def unavailable(**_):
    raise OSError("shared memory unavailable")

  monkeypatch.setattr(helpers.tempfile, "NamedTemporaryFile", unavailable)
  helpers.publish_teletyped_manager_state(False, marker, output)  # Never blocks manager startup.


def test_preflight_checks_worker_identity_and_matching_tunnel_keys(monkeypatch, tmp_path):
  monkeypatch.setattr(teletyped, "get_dongle_id", lambda: "device")
  monkeypatch.setattr(teletyped, "build_auth_headers", dict)
  with pytest.raises(RuntimeError, match="JWT"):
    teletyped.guardian_preflight()
  monkeypatch.setattr(teletyped, "build_auth_headers", lambda: {"X-Device-JWT": "signed"})
  private = tmp_path / "id_ed25519"
  monkeypatch.setattr(teletyped, "KEY_PATH_PRIV", str(private))
  private.with_name(private.name + ".pub").write_text("ssh-ed25519 expected comment\n")
  monkeypatch.setattr(teletyped.subprocess, "run", lambda *_, **__: SimpleNamespace(stdout="ssh-ed25519 expected\n"))
  teletyped.guardian_preflight()
  monkeypatch.setattr(teletyped.subprocess, "run", lambda *_, **__: SimpleNamespace(stdout="ssh-ed25519 different\n"))
  with pytest.raises(RuntimeError, match="do not match"):
    teletyped.guardian_preflight()
