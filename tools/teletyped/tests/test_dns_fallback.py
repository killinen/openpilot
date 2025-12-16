import json
import socket

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

