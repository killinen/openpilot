import bz2
import json
import os
import stat
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from zipfile import ZipFile, ZIP_STORED

import pytest


def test_imports_smoke():
  import openpilot.tools.teletyped.helper as helper
  import openpilot.tools.teletyped.ssh_key as ssh_key

  assert helper.API_URL
  assert hasattr(ssh_key, "send_ssh_key_if_needed")


def test_detect_local_port_env_override(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import helper

  monkeypatch.setenv(helper.LOCAL_PORT_ENV, "10022")
  assert helper._detect_local_port() == 10022


def test_detect_local_port_invalid_env_falls_back(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import helper

  monkeypatch.setenv(helper.LOCAL_PORT_ENV, "not-a-port")
  monkeypatch.setattr(helper.os.path, "exists", lambda p: False)
  assert helper._detect_local_port() == 22


def test_wormhole_sender_reports_redacted_process_output(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import route_sender

  class FailedProcess:
    stdout = iter([
      "Connecting to rendezvous server...\n",
      "Wormhole code is: 7-secret-code\n",
      "relay connection failed: certificate verify failed\n",
    ])
    returncode = 1

    def wait(self) -> None:
      return None

  captured: list[BaseException] = []
  monkeypatch.setattr(route_sender, "RETRY_LIMIT", 1)
  monkeypatch.setattr(route_sender.subprocess, "Popen", lambda *args, **kwargs: FailedProcess())
  monkeypatch.setattr(route_sender, "capture_exception", captured.append)
  monkeypatch.setattr(route_sender, "log", lambda *args, **kwargs: None)
  monkeypatch.setattr(route_sender, "send_wormhole_code", lambda *args, **kwargs: False)
  monkeypatch.setattr(route_sender.time, "sleep", lambda _seconds: None)

  sent, code, _filename = route_sender.send_file_wormhole(
    "/tmp/test.zip", "device", "drive", requested_files=[]
  )

  assert sent is False
  assert code is None
  assert len(captured) == 1
  diagnostic = str(captured[0])
  assert "certificate verify failed" in diagnostic
  assert "7-secret-code" not in diagnostic
  assert "Wormhole code is: <redacted>" in diagnostic


def test_get_os_info_prefers_agnos_version_file_for_tici(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import helper

  monkeypatch.setattr(
    helper,
    "get_hardware_info",
    lambda: {"type": "comma three", "model": "tici", "name": "Tici"},
  )
  monkeypatch.setattr(
    helper,
    "_read_first_line",
    lambda path: {
      "/VERSION": "9",
      "/BUILD": "deadbeef 2026-04-13T12:00:00Z",
    }.get(path),
  )
  monkeypatch.setattr(
    helper,
    "_read_os_release",
    lambda: {"PRETTY_NAME": "Ubuntu 20.04.6 LTS", "VERSION_ID": "20.04"},
  )

  info = helper.get_os_info()

  assert info["platform"] == "AGNOS"
  assert info["version"] == "9"
  assert info["display"] == "AGNOS 9"
  assert info["extras"]["base_os"] == "Ubuntu 20.04.6 LTS"
  assert info["extras"]["build"] == "deadbeef 2026-04-13T12:00:00Z"


def test_ensure_local_keypair_fallback_generation(tmp_path, monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import ssh_key

  priv = tmp_path / "id_ed25519_goranconnect"
  pub = tmp_path / "id_ed25519_goranconnect.pub"

  monkeypatch.setattr(ssh_key, "KEY_PATH_PRIV", str(priv))
  monkeypatch.setattr(ssh_key, "KEY_PATH", str(pub))
  monkeypatch.setattr(ssh_key.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(FileNotFoundError()))

  assert ssh_key.ensure_local_keypair() is True
  assert priv.is_file()
  assert pub.is_file()

  priv_mode = stat.S_IMODE(os.stat(priv).st_mode)
  assert priv_mode == 0o600

  priv_text = priv.read_text(encoding="utf-8", errors="strict")
  assert "OPENSSH PRIVATE KEY" in priv_text

  pub_text = pub.read_text(encoding="utf-8", errors="strict").strip()
  assert pub_text.startswith("ssh-ed25519 ")
  assert pub_text.endswith(" goranconnect")


@pytest.fixture
def api_stub_server():
  received: list[dict[str, Any]] = []
  routes: dict[tuple[str, str], dict[str, Any]] = {}

  class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):  # noqa: A002
      return

    def _handle(self):
      length = int(self.headers.get("Content-Length", "0"))
      body = self.rfile.read(length) if length else b""

      received.append({
        "method": self.command,
        "path": self.path,
        "headers": dict(self.headers),
        "body": body,
      })

      key = (self.command, self.path)
      route = routes.get(key)
      if route is None:
        self.send_response(404)
        self.end_headers()
        return

      self.send_response(int(route.get("status", 200)))
      for header_key, header_val in (route.get("headers") or {}).items():
        self.send_header(str(header_key), str(header_val))
      self.end_headers()
      resp_body = route.get("body", b"")
      if isinstance(resp_body, str):
        resp_body = resp_body.encode("utf-8")
      self.wfile.write(resp_body)

    def do_GET(self):
      self._handle()

    def do_POST(self):
      self._handle()

  server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
  thread = threading.Thread(target=server.serve_forever, daemon=True)
  thread.start()

  base_url = f"http://127.0.0.1:{server.server_address[1]}"

  yield {"base_url": base_url, "routes": routes, "received": received}

  server.shutdown()
  server.server_close()
  thread.join(timeout=2)


def test_ssh_key_upload_posts_expected_payload(tmp_path, monkeypatch: pytest.MonkeyPatch, api_stub_server):
  from openpilot.tools.teletyped import ssh_key

  priv = tmp_path / "id_ed25519_goranconnect"
  pub = tmp_path / "id_ed25519_goranconnect.pub"
  monkeypatch.setattr(ssh_key, "KEY_PATH_PRIV", str(priv))
  monkeypatch.setattr(ssh_key, "KEY_PATH", str(pub))
  monkeypatch.setattr(ssh_key.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(FileNotFoundError()))

  base = api_stub_server["base_url"]
  api_stub_server["routes"][("POST", "/upload-key")] = {"status": 200, "body": b"{}"}
  monkeypatch.setattr(ssh_key, "API_URL_KEY", f"{base}/upload-key")

  monkeypatch.setattr(ssh_key, "get_dongle_id", lambda: "DONGLE123")
  assert ssh_key.send_ssh_key({"X-Device-JWT": "test"}) is True

  req = api_stub_server["received"][-1]
  assert req["method"] == "POST"
  assert req["path"] == "/upload-key"
  assert req["headers"].get("X-Device-JWT") == "test"
  payload = json.loads(req["body"].decode("utf-8"))
  assert payload["device_id"] == "DONGLE123"
  assert payload["public_key"].startswith("ssh-ed25519 ")


def test_ssh_key_remote_has_key_statuses(monkeypatch: pytest.MonkeyPatch, api_stub_server):
  from openpilot.tools.teletyped import ssh_key

  base = api_stub_server["base_url"]
  monkeypatch.setattr(ssh_key, "API_URL_GET_KEY", f"{base}/get-key")

  api_stub_server["routes"][("GET", "/get-key/DONGLE123")] = {"status": 200, "body": b""}
  assert ssh_key._remote_has_key("DONGLE123", {"X-Device-JWT": "test"}) is True

  api_stub_server["routes"][("GET", "/get-key/DONGLE404")] = {"status": 404, "body": b""}
  assert ssh_key._remote_has_key("DONGLE404", {"X-Device-JWT": "test"}) is False


def test_ssh_key_upload_unauthorized(monkeypatch: pytest.MonkeyPatch, tmp_path, api_stub_server):
  from openpilot.tools.teletyped import ssh_key

  priv = tmp_path / "id_ed25519_goranconnect"
  pub = tmp_path / "id_ed25519_goranconnect.pub"
  monkeypatch.setattr(ssh_key, "KEY_PATH_PRIV", str(priv))
  monkeypatch.setattr(ssh_key, "KEY_PATH", str(pub))
  monkeypatch.setattr(ssh_key.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(FileNotFoundError()))

  base = api_stub_server["base_url"]
  api_stub_server["routes"][("POST", "/upload-key")] = {"status": 401, "body": b""}
  monkeypatch.setattr(ssh_key, "API_URL_KEY", f"{base}/upload-key")

  monkeypatch.setattr(ssh_key, "get_dongle_id", lambda: "DONGLE123")
  assert ssh_key.send_ssh_key({"X-Device-JWT": "test"}) is False


def test_send_heartbeat_includes_uptime(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import teletyped

  captured: dict[str, Any] = {}

  class FakeResponse:
    status_code = 200

  os_info = {
    "platform": "AGNOS",
    "version": "9",
    "display": "AGNOS 9",
    "extras": {
      "base_os": "Ubuntu 20.04.6 LTS",
      "build": "deadbeef 2026-04-13T12:00:00Z",
    },
  }

  monkeypatch.setattr(teletyped, "build_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(teletyped, "has_internet_connection", lambda: True)
  monkeypatch.setattr(teletyped, "get_hardware_info", lambda: {"type": "tici", "model": "comma 3X", "name": "test"})
  monkeypatch.setattr(teletyped, "get_os_info", lambda: os_info)
  monkeypatch.setattr(teletyped, "get_op_params_info", dict)
  monkeypatch.setattr(teletyped, "_read_tunnel_pid", lambda: 1234)
  monkeypatch.setattr(teletyped.psutil, "boot_time", lambda: 100.0)
  monkeypatch.setattr(teletyped.time, "time", lambda: 160.9)
  monkeypatch.setattr(
    teletyped,
    "http_post",
    lambda url, headers, json, timeout: captured.update({"url": url, "headers": headers, "json": json}) or FakeResponse(),
  )

  teletyped.send_heartbeat("DONGLE123", "running")

  assert captured["json"]["device_id"] == "DONGLE123"
  assert (
    captured["json"]["reverse_ssh_protocol_version"]
    == teletyped.REVERSE_SSH_PROTOCOL_VERSION
  )
  assert captured["json"]["details"]["uptime_seconds"] == 60
  assert captured["json"]["details"]["os_platform"] == "AGNOS"
  assert captured["json"]["details"]["os_version"] == "9"
  assert captured["json"]["details"]["os_base"] == "Ubuntu 20.04.6 LTS"
  assert captured["json"]["details"]["os_build"] == "deadbeef 2026-04-13T12:00:00Z"
  assert captured["json"]["details"]["os"] == os_info


def test_requested_forwards_use_assigned_ports():
  from openpilot.tools.teletyped import teletyped

  request = {"reverse_tunnel_req": True, "pond_tunnel_req": True}
  config = {
    "version": 2,
    "shell_remote_port": 22017,
    "pond_remote_port": 29017,
  }

  assert teletyped._requested_forwards(request, config) == [
    (22017, teletyped.LOCAL_PORT),
    (29017, teletyped.POND_LOCAL_PORT),
  ]


def test_requested_forwards_fall_back_to_legacy_ports():
  from openpilot.tools.teletyped import teletyped

  request = {"reverse_tunnel_req": True, "pond_tunnel_req": True}

  assert teletyped._requested_forwards(request, {"version": 2}) == [
    (teletyped.REMOTE_PORT, teletyped.LOCAL_PORT),
    (teletyped.POND_REMOTE_PORT, teletyped.POND_LOCAL_PORT),
  ]


class _StaticResponse:
  def __init__(self, status_code: int, payload: Any | None = None):
    self.status_code = status_code
    self._payload = payload if payload is not None else {}

  def raise_for_status(self) -> None:
    if self.status_code >= 400:
      raise RuntimeError(f"HTTP {self.status_code}")

  def json(self) -> Any:
    return self._payload


def test_drive_inventory_step_respects_auto_scan_min_interval(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import route_sender

  collect_called = False

  def _collect():
    nonlocal collect_called
    collect_called = True
    return [], 0

  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY", True)
  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY_MIN_INTERVAL", 900)
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(
    route_sender,
    "http_get",
    lambda url, headers, timeout: _StaticResponse(404),
  )
  monkeypatch.setattr(route_sender.time, "time", lambda: 1000.0)
  monkeypatch.setattr(
    route_sender,
    "_load_drive_inventory_state",
    lambda: {
      "last_auto_scan_started_at": 200.0,
      "last_successful_upload_at": 100.0,
      "last_inventory_fingerprint": "abc",
    },
  )
  monkeypatch.setattr(route_sender, "collect_drive_inventory", _collect)

  route_sender.drive_inventory_step("DONGLE123")

  assert collect_called is False


def test_drive_inventory_step_skips_unchanged_auto_upload(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import route_sender

  saved_states: list[dict[str, Any]] = []
  upload_called = False
  drives = [{
    "name": "2026-04-13--12-00-00--0",
    "size_bytes": 42,
    "file_count": 1,
    "files": ["rlog"],
    "modified_at": "2026-04-13T12:00:00+00:00",
  }]
  fingerprint = route_sender._inventory_fingerprint(drives, 42)

  def _save(state: dict[str, Any]) -> None:
    saved_states.append(dict(state))

  def _post(url, json, headers, timeout):
    nonlocal upload_called
    upload_called = True
    return _StaticResponse(200, {})

  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY", True)
  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY_MIN_INTERVAL", 900)
  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY_FORCE_REFRESH", 86400)
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(
    route_sender,
    "http_get",
    lambda url, headers, timeout: _StaticResponse(404),
  )
  monkeypatch.setattr(route_sender.time, "time", lambda: 2000.0)
  monkeypatch.setattr(
    route_sender,
    "_load_drive_inventory_state",
    lambda: {
      "last_auto_scan_started_at": 0.0,
      "last_successful_upload_at": 1900.0,
      "last_inventory_fingerprint": fingerprint,
    },
  )
  monkeypatch.setattr(route_sender, "_save_drive_inventory_state", _save)
  monkeypatch.setattr(route_sender, "collect_drive_inventory", lambda: (drives, 42))
  monkeypatch.setattr(route_sender, "http_post", _post)

  route_sender.drive_inventory_step("DONGLE123")

  assert upload_called is False
  assert saved_states[-1]["last_auto_scan_started_at"] == 2000.0


def test_drive_inventory_step_requested_scan_bypasses_auto_throttle(monkeypatch: pytest.MonkeyPatch):
  from openpilot.tools.teletyped import route_sender

  saved_states: list[dict[str, Any]] = []
  uploads: list[dict[str, Any]] = []
  drives = [{
    "name": "2026-04-13--12-00-00--0",
    "size_bytes": 42,
    "file_count": 1,
    "files": ["rlog"],
    "modified_at": "2026-04-13T12:00:00+00:00",
  }]

  def _save(state: dict[str, Any]) -> None:
    saved_states.append(dict(state))

  def _post(url, json, headers, timeout):
    uploads.append(json)
    return _StaticResponse(200, {})

  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY", True)
  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY_MIN_INTERVAL", 900)
  monkeypatch.setattr(route_sender, "AUTO_DRIVE_INVENTORY_FORCE_REFRESH", 86400)
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(
    route_sender,
    "http_get",
    lambda url, headers, timeout: _StaticResponse(200, {"pending": True}),
  )
  monkeypatch.setattr(route_sender.time, "time", lambda: 2000.0)
  monkeypatch.setattr(
    route_sender,
    "_load_drive_inventory_state",
    lambda: {
      "last_auto_scan_started_at": 1999.0,
      "last_successful_upload_at": 1900.0,
      "last_inventory_fingerprint": "",
    },
  )
  monkeypatch.setattr(route_sender, "_save_drive_inventory_state", _save)
  monkeypatch.setattr(route_sender, "collect_drive_inventory", lambda: (drives, 42))
  monkeypatch.setattr(route_sender, "update_drive_scan_status", lambda *args, **kwargs: None)
  monkeypatch.setattr(route_sender, "http_post", _post)

  route_sender.drive_inventory_step("DONGLE123")

  assert len(uploads) == 1
  assert uploads[0]["device_id"] == "DONGLE123"
  assert saved_states[-1]["last_successful_upload_at"] == 2000.0


def test_route_sender_step_compress_action_compresses_logs_in_place(
  tmp_path,
  monkeypatch: pytest.MonkeyPatch,
):
  from openpilot.tools.teletyped import route_sender

  drive_name = "2026-04-13--12-00-00--0"
  drive_dir = tmp_path / drive_name
  drive_dir.mkdir()
  raw_rlog = drive_dir / "rlog"
  raw_qlog = drive_dir / "qlog"
  raw_rlog.write_bytes(b"raw rlog payload")
  raw_qlog.write_bytes(b"raw qlog payload")

  updates: list[dict[str, Any]] = []

  def _get(url, headers, timeout):
    return _StaticResponse(
      200,
      [{
        "drive_name": drive_name,
        "status": "queued",
        "action": "compress",
        "requested_files": ["rlog"],
      }],
    )

  def _post(url, json, headers, timeout):
    updates.append(json)
    return _StaticResponse(200, {})

  wormhole_calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []

  def _send_file_wormhole(*args, **kwargs):
    wormhole_calls.append((args, kwargs))
    return False, None, None

  monkeypatch.setattr(route_sender, "REALDATA_DIR", str(tmp_path))
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(route_sender, "has_internet_connection", lambda: True)
  monkeypatch.setattr(route_sender, "http_get", _get)
  monkeypatch.setattr(route_sender, "http_post", _post)
  monkeypatch.setattr(route_sender, "upload_drive_inventory_snapshot", lambda device_id: None)
  monkeypatch.setattr(route_sender, "send_file_wormhole", _send_file_wormhole)

  route_sender.route_sender_step("DONGLE123")

  assert not wormhole_calls
  assert not raw_rlog.exists()
  assert raw_qlog.exists()
  with bz2.open(drive_dir / "rlog.bz2", "rb") as compressed:
    assert compressed.read() == b"raw rlog payload"

  final_update = updates[-1]
  assert final_update["device_id"] == "DONGLE123"
  assert final_update["drive_name"] == drive_name
  assert final_update["status"] == "sent"
  assert final_update["stage"] == "compressed"
  assert final_update["included_files"] == [f"{drive_name}/rlog.bz2"]


def test_route_sender_bz2_runner_uses_multiple_workers(
  tmp_path,
  monkeypatch: pytest.MonkeyPatch,
):
  from openpilot.tools.teletyped import route_sender

  src_a = tmp_path / "rlog"
  src_b = tmp_path / "qlog"
  dst_a = tmp_path / "rlog.bz2"
  dst_b = tmp_path / "qlog.bz2"
  src_a.write_bytes(b"a" * 1024)
  src_b.write_bytes(b"b" * 1024)

  lock = threading.Lock()
  active_workers = 0
  max_active_workers = 0
  original_compress = route_sender._compress_path_to_bz2_file

  def _compress_with_overlap_probe(src_path, dest_path, *, progress_cb=None):
    nonlocal active_workers, max_active_workers
    with lock:
      active_workers += 1
      max_active_workers = max(max_active_workers, active_workers)
    try:
      time.sleep(0.05)
      return original_compress(src_path, dest_path, progress_cb=progress_cb)
    finally:
      with lock:
        active_workers -= 1

  monkeypatch.setattr(route_sender, "RLOG_BZ2_WORKERS", 2)
  monkeypatch.setattr(route_sender, "report_transfer_progress", lambda *args, **kwargs: None)
  monkeypatch.setattr(route_sender, "_compress_path_to_bz2_file", _compress_with_overlap_probe)

  reporter = route_sender.ZipProgressReporter(
    "DONGLE123",
    "drive",
    stage="compressing",
    label="Compressing",
    total_files=2,
    total_bytes=src_a.stat().st_size + src_b.stat().st_size,
  )
  compressed_by_src, failed_paths = route_sender._run_bz2_compressions(
    [(str(src_a), str(dst_a)), (str(src_b), str(dst_b))],
    progress_reporter=reporter,
  )

  assert failed_paths == []
  assert compressed_by_src == {
    str(src_a): str(dst_a),
    str(src_b): str(dst_b),
  }
  assert max_active_workers == 2
  assert bz2.decompress(dst_a.read_bytes()) == b"a" * 1024
  assert bz2.decompress(dst_b.read_bytes()) == b"b" * 1024


def test_route_sender_step_zips_rlog_as_stored_bz2_stream(
  tmp_path,
  monkeypatch: pytest.MonkeyPatch,
):
  from openpilot.tools.teletyped import route_sender

  drive_name = "2026-04-13--12-00-00--0"
  drive_dir = tmp_path / drive_name
  drive_dir.mkdir()
  raw_rlog = drive_dir / "rlog"
  raw_rlog.write_bytes(b"raw rlog payload" * 1024)

  updates: list[dict[str, Any]] = []
  zipped: dict[str, Any] = {}

  def _get(url, headers, timeout):
    return _StaticResponse(
      200,
      [{
        "drive_name": drive_name,
        "status": "queued",
        "requested_files": ["rlog", "qlog"],
      }],
    )

  def _post(url, json, headers, timeout):
    updates.append(json)
    return _StaticResponse(200, {})

  def _send_file_wormhole(zip_path, device_id, transfer_drive_name, requested_files):
    arcname = f"{drive_name}/{drive_name}/rlog.bz2"
    with ZipFile(zip_path, "r") as zipf:
      info = zipf.getinfo(arcname)
      zipped["compress_type"] = info.compress_type
      with zipf.open(arcname) as compressed:
        zipped["payload"] = bz2.decompress(compressed.read())
    return True, "1-test-code", os.path.basename(zip_path)

  monkeypatch.setattr(route_sender, "REALDATA_DIR", str(tmp_path))
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(route_sender, "has_internet_connection", lambda: True)
  monkeypatch.setattr(route_sender, "http_get", _get)
  monkeypatch.setattr(route_sender, "http_post", _post)
  monkeypatch.setattr(route_sender, "send_file_wormhole", _send_file_wormhole)

  route_sender.route_sender_step("DONGLE123")

  assert zipped["compress_type"] == ZIP_STORED
  assert zipped["payload"] == b"raw rlog payload" * 1024
  assert raw_rlog.exists()
  assert updates[-1]["stage"] == "ready"


def test_route_sender_step_packs_split_parts_in_numeric_segment_order(
  tmp_path,
  monkeypatch: pytest.MonkeyPatch,
):
  from openpilot.tools.teletyped import route_sender

  realdata_dir = tmp_path / "realdata"
  realdata_dir.mkdir()
  temp_dir = tmp_path / "ziptmp"
  temp_dir.mkdir()
  route_base = "2026-04-13--12-00-00"
  unordered_segment_nums = (0, 1, 10, 2, 100, 3, 11, 20, 12, 22, 21)
  for segment_num in unordered_segment_nums:
    segment_dir = realdata_dir / f"{route_base}--{segment_num}"
    segment_dir.mkdir()
    (segment_dir / "qlog").write_bytes(f"qlog {segment_num}".encode("ascii"))

  sent_segments: list[str] = []
  sent_parts: list[int] = []

  def _get(url, headers, timeout):
    return _StaticResponse(
      200,
      [{
        "drive_name": route_base,
        "status": "queued",
        "requested_files": ["qlog"],
      }],
    )

  def _post(url, json, headers, timeout):
    return _StaticResponse(200, {})

  def _send_file_wormhole(zip_path, device_id, transfer_drive_name, requested_files, **kwargs):
    with ZipFile(zip_path, "r") as zipf:
      names = zipf.namelist()
    assert len(names) == 1
    sent_segments.append(names[0].split("/")[1])
    sent_parts.append(kwargs["part_number"])
    return True, "1-test-code", os.path.basename(zip_path)

  monkeypatch.setattr(route_sender, "REALDATA_DIR", str(realdata_dir))
  monkeypatch.setattr(route_sender, "WORMHOLE_ZIP_TARGET_BYTES", 1)
  monkeypatch.setattr(route_sender, "_pick_temp_dir", lambda *args: str(temp_dir))
  monkeypatch.setattr(route_sender, "_auth_headers", lambda: {"X-Device-JWT": "test"})
  monkeypatch.setattr(route_sender, "has_internet_connection", lambda: True)
  monkeypatch.setattr(route_sender, "http_get", _get)
  monkeypatch.setattr(route_sender, "http_post", _post)
  monkeypatch.setattr(route_sender, "send_file_wormhole", _send_file_wormhole)

  route_sender.route_sender_step("DONGLE123")

  assert sent_segments == [
    f"{route_base}--0",
    f"{route_base}--1",
    f"{route_base}--2",
    f"{route_base}--3",
    f"{route_base}--10",
    f"{route_base}--11",
    f"{route_base}--12",
    f"{route_base}--20",
    f"{route_base}--21",
    f"{route_base}--22",
    f"{route_base}--100",
  ]
  assert sent_parts == list(range(1, len(unordered_segment_nums) + 1))
