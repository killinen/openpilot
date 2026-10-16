import json
import socket
import threading

import pytest

from openpilot.tools.teletyped import helper as h


def _reset_cache(tmp_path):
  h.DNS_CACHE_PATH = str(tmp_path / "teletyped_dns_cache.json")
  h.DNS_CACHE_REFRESH_SEC = 0
  h.DNS_CACHE_SAVE_MIN_SEC = 0
  h._DNS_CACHE_LOADED = False
  h._DNS_CACHE_DIRTY = False
  h._DNS_CACHE_LAST_SAVE = 0.0
  h._DNS_CACHE = {}


def test_dns_cache_set_get_persists(tmp_path):
  _reset_cache(tmp_path)

  h._set_cached_ip("example.com", "1.2.3.4", resolved_at=123.0)
  assert h.get_cached_ip("example.com") == "1.2.3.4"

  with open(h.DNS_CACHE_PATH, encoding="utf-8") as f:
    data = json.load(f)
  assert data["hosts"]["example.com"]["ip"] == "1.2.3.4"


def test_maybe_refresh_cached_ip_picks_ipv4(monkeypatch, tmp_path):
  _reset_cache(tmp_path)

  def fake_getaddrinfo(host, port, *args, **kwargs):
    return [
      (socket.AF_INET6, socket.SOCK_STREAM, 6, "", ("::1", port, 0, 0)),
      (socket.AF_INET, socket.SOCK_STREAM, 6, "", ("5.6.7.8", port)),
    ]

  monkeypatch.setattr(socket, "getaddrinfo", fake_getaddrinfo)

  ip = h.maybe_refresh_cached_ip("example.com", 443)
  assert ip == "5.6.7.8"
  assert h.get_cached_ip("example.com") == "5.6.7.8"


def test_force_getaddrinfo_rewrites_only_target_host(monkeypatch):
  calls = []

  def fake_getaddrinfo(host, port, *args, **kwargs):
    calls.append((host, port))
    return [(socket.AF_INET, socket.SOCK_STREAM, 6, "", ("1.1.1.1", port))]

  monkeypatch.setattr(socket, "getaddrinfo", fake_getaddrinfo)

  with h._force_getaddrinfo("a.example", "9.9.9.9"):
    socket.getaddrinfo("a.example", 443)
    socket.getaddrinfo("b.example", 443)

  assert calls[0][0] == "9.9.9.9"
  assert calls[1][0] == "b.example"


def test_is_dns_resolution_error_from_gaierror():
  try:
    raise Exception("outer") from socket.gaierror(7, "No address associated with hostname")
  except Exception as exc:
    assert h._is_dns_resolution_error(exc) is True


def test_dns_fallback_upload_does_not_block_heartbeat(monkeypatch):
  upload_started = threading.Event()
  release_upload = threading.Event()
  heartbeat_finished = threading.Event()
  errors = []
  resolved = []

  def resolver(host, port, *args, **kwargs):
    resolved.append((threading.current_thread().name, host))
    return []

  def request(method, url, **kwargs):
    # Both requests first encounter broken DNS, then retry via their cached IP.
    if not getattr(h._DNS_OVERRIDES, "hosts", {}):
      raise socket.gaierror("temporary failure in name resolution")
    socket.getaddrinfo("api.example", 443)
    if url.endswith("/upload"):
      upload_started.set()
      assert release_upload.wait(5)
    return object()

  def post(path, finished=None):
    try:
      h.http_post(f"https://api.example/{path}", timeout=5)
      if finished:
        finished.set()
    except Exception as e:
      errors.append(e)

  monkeypatch.setattr(socket, "getaddrinfo", resolver)
  monkeypatch.setattr(h._requests, "request", request)
  monkeypatch.setattr(h, "DNS_FALLBACK_ENABLED", True)
  monkeypatch.setattr(h, "get_cached_ip", lambda host: "192.0.2.2" if threading.current_thread().name == "heartbeat" else "192.0.2.1")
  monkeypatch.setattr(h, "_mark_cached_ip_used", lambda host: None)
  monkeypatch.setattr(h, "ensure_dns_config", lambda: None)
  uploader = threading.Thread(target=post, args=("upload",), name="upload")
  heartbeat = threading.Thread(target=post, args=("heartbeat", heartbeat_finished), name="heartbeat")
  uploader.start()
  try:
    assert upload_started.wait(2)
    # An unrelated thread must continue using the normal resolver.
    socket.getaddrinfo("api.example", 443)
    heartbeat.start()
    assert heartbeat_finished.wait(2), "Heartbeat waited for the upload's DNS fallback lock"
    assert uploader.is_alive()
  finally:
    release_upload.set()
    uploader.join(2)
    if heartbeat.ident is not None:
      heartbeat.join(2)
  assert not uploader.is_alive()
  assert not heartbeat.is_alive()
  assert not errors
  assert ("upload", "192.0.2.1") in resolved
  assert ("heartbeat", "192.0.2.2") in resolved
  assert (threading.current_thread().name, "api.example") in resolved
  assert socket.getaddrinfo is resolver


def test_nested_dns_override_restores_after_exception(monkeypatch):
  def resolver(host, *args, **kwargs):
    return host

  monkeypatch.setattr(socket, "getaddrinfo", resolver)
  with h._force_getaddrinfo("api.example", "192.0.2.1"):
    with pytest.raises(RuntimeError), h._force_getaddrinfo("api.example", "192.0.2.2"):
      assert socket.getaddrinfo("api.example", 443) == "192.0.2.2"
      raise RuntimeError("request failed")
    assert socket.getaddrinfo("api.example", 443) == "192.0.2.1"
  assert socket.getaddrinfo is resolver

