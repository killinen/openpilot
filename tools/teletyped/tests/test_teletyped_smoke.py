import json
import os
import stat
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

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
  assert captured["json"]["details"]["uptime_seconds"] == 60
  assert captured["json"]["details"]["os_platform"] == "AGNOS"
  assert captured["json"]["details"]["os_version"] == "9"
  assert captured["json"]["details"]["os_base"] == "Ubuntu 20.04.6 LTS"
  assert captured["json"]["details"]["os_build"] == "deadbeef 2026-04-13T12:00:00Z"
  assert captured["json"]["details"]["os"] == os_info
