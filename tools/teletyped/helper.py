from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import cast
from contextlib import contextmanager
import json
import os
import socket
import re
import subprocess
import shutil
import platform as py_platform
import tempfile
import threading
import time
from urllib.parse import urlparse
import jwt

import importlib
from types import ModuleType

import sentry_sdk

requests_exceptions: ModuleType | None
try:
  requests_exceptions = importlib.import_module("requests.exceptions")
except ModuleNotFoundError:
  requests_exceptions = None

from openpilot.common.params import Params
from openpilot.system.hardware import PC, HARDWARE
from openpilot.system.hardware.hw import Paths

from cereal import log as cereal_log


API_URL = "https://goranconnect.duckdns.org/api"
POLL_INTERVAL = 10
CHECK_INTERVAL = 60

def _comma_home_default() -> str:
  suffix = os.environ.get("OPENPILOT_PREFIX", "")
  return os.path.join(str(Path.home()), ".comma" + suffix)

def _persist_root() -> str:
  try:
    return Paths.persist_root()
  except Exception:
    pass
  if PC:
    return os.path.join(_comma_home_default(), "persist")
  return "/persist/"

def _realdata_root() -> str:
  try:
    return os.path.normpath(Paths.log_root())
  except Exception:
    pass
  if os.path.isdir("/data/media/0/realdata"):
    return "/data/media/0/realdata"
  return os.path.join(_comma_home_default(), "media", "0", "realdata")

HEARTBEAT_INTERVAL = 30
PERSIST_ROOT = _persist_root()
REGISTRATION_KEY_PATH = os.path.join(PERSIST_ROOT, "comma", "id_rsa")
DEVICE_JWT_TTL_SECONDS = 300
DEVICE_JWT_REFRESH_LEEWAY = 30
_DEVICE_JWT_CACHE: dict[str, float | str] = {"token": "", "refresh_at": 0.0}


def _key_dir_candidates() -> list[str]:
  return [
    os.path.join(PERSIST_ROOT, "comma"),
    "/data/params/d/goranconnect_ssh",
    os.path.join(_comma_home_default(), "persist", "comma"),
    "/tmp/comma",
  ]


def _is_writable_dir(path: str) -> bool:
  try:
    os.makedirs(path, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path, delete=True) as tmp:
      tmp.write(b"ok")
      tmp.flush()
    return True
  except OSError:
    return False


def _resolve_key_paths() -> tuple[str, str]:
  key_name = "id_ed25519_goranconnect"
  candidates = _key_dir_candidates()

  # Prefer an existing keypair, even if the directory is now read-only
  for base in candidates:
    priv = os.path.join(base, key_name)
    pub = f"{priv}.pub"
    if os.path.exists(pub) and os.path.exists(priv):
      return pub, priv

  # Otherwise pick the first writable candidate
  for base in candidates:
    if _is_writable_dir(base):
      priv = os.path.join(base, key_name)
      pub = f"{priv}.pub"
      return pub, priv

  # Last resort: drop to /tmp
  fallback_base = "/tmp/comma"
  os.makedirs(fallback_base, exist_ok=True)
  priv = os.path.join(fallback_base, key_name)
  pub = f"{priv}.pub"
  return pub, priv


KEY_PATH, KEY_PATH_PRIV = _resolve_key_paths()

def _resolve_dns_cache_path() -> str:
  file_name = "teletyped_dns_cache.json"
  candidates = _key_dir_candidates()

  # Prefer an existing cache file, even if the directory is now read-only.
  for base in candidates:
    path = os.path.join(base, file_name)
    if os.path.exists(path):
      return path

  # Otherwise pick the first writable candidate.
  for base in candidates:
    if _is_writable_dir(base):
      return os.path.join(base, file_name)

  # Last resort: drop to /tmp.
  fallback_base = "/tmp/comma"
  os.makedirs(fallback_base, exist_ok=True)
  return os.path.join(fallback_base, file_name)

DNS_CACHE_PATH = _resolve_dns_cache_path()
DNS_CACHE_REFRESH_SEC = int(os.environ.get("TELETYPED_DNS_CACHE_REFRESH_SEC", str(6 * 60 * 60)))
DNS_CACHE_SAVE_MIN_SEC = int(os.environ.get("TELETYPED_DNS_CACHE_SAVE_MIN_SEC", "600"))
DNS_FALLBACK_ENABLED = os.environ.get("TELETYPED_DNS_FALLBACK", "1").strip().lower() not in {"0", "false", "no", "off"}

_DNS_CACHE_LOCK = threading.Lock()
_DNS_PATCH_LOCK = threading.Lock()
_DNS_CACHE_LOADED = False
_DNS_CACHE_DIRTY = False
_DNS_CACHE_LAST_SAVE = 0.0
_DNS_CACHE: dict[str, dict] = {}

SENTRY_DSN_DEFAULT = "https://82a4222b21bdd8e738c0f20677110918@o1107536.ingest.us.sentry.io/4509169784848384"
_SENTRY_INITIALIZED = False

NETWORK_EXCEPTION_TYPES: tuple[type[BaseException], ...]

if requests_exceptions is not None:
  _exception_candidates = (
    getattr(requests_exceptions, "ConnectionError", BaseException),
    getattr(requests_exceptions, "Timeout", BaseException),
    getattr(requests_exceptions, "ReadTimeout", None),
  )
  NETWORK_EXCEPTION_TYPES = tuple(
    exc for exc in _exception_candidates
    if isinstance(exc, type) and issubclass(exc, BaseException)
  )
else:
  NETWORK_EXCEPTION_TYPES = tuple()


def _should_filter_exception(exc: BaseException) -> bool:
  return bool(NETWORK_EXCEPTION_TYPES) and isinstance(exc, NETWORK_EXCEPTION_TYPES)

SENTRY_NETWORK_SAMPLE_EVERY = int(os.environ.get("GCS_SENTRY_NETWORK_SAMPLE_EVERY", "500"))
SENTRY_NETWORK_SAMPLE_MIN_INTERVAL_SEC = int(os.environ.get("GCS_SENTRY_NETWORK_SAMPLE_MIN_INTERVAL_SEC", "60"))
SENTRY_NETWORK_SAMPLE_MAX_KEYS = int(os.environ.get("GCS_SENTRY_NETWORK_SAMPLE_MAX_KEYS", "50"))

_NETWORK_EXCEPTION_SAMPLE_LOCK = threading.Lock()
_NETWORK_EXCEPTION_SAMPLE_STATE: dict[str, dict[str, float]] = {}

def _network_exception_key(exc: BaseException) -> str:
  # Keep grouping stable and bounded; avoid storing full URLs/tokens.
  msg = str(exc)
  msg = re.sub(r"(Bearer|JWT)\\s+[A-Za-z0-9._-]+", r"\\1 <redacted>", msg)
  msg = re.sub(r"([?&](token|jwt|auth|authorization)=[^&\\s]+)", r"\\1<redacted>", msg, flags=re.IGNORECASE)
  msg = (msg[:220] + "…") if len(msg) > 220 else msg
  return f"{exc.__class__.__name__}:{msg}"

def _maybe_capture_sampled_network_error(exc: BaseException) -> None:
  if not _SENTRY_INITIALIZED or SENTRY_NETWORK_SAMPLE_EVERY <= 0:
    return

  key = _network_exception_key(exc)
  now = time.time()

  with _NETWORK_EXCEPTION_SAMPLE_LOCK:
    if key not in _NETWORK_EXCEPTION_SAMPLE_STATE:
      if len(_NETWORK_EXCEPTION_SAMPLE_STATE) >= SENTRY_NETWORK_SAMPLE_MAX_KEYS:
        key = "__other__"
      if key not in _NETWORK_EXCEPTION_SAMPLE_STATE:
        _NETWORK_EXCEPTION_SAMPLE_STATE[key] = {
          "count": 0.0,
          "next": float(SENTRY_NETWORK_SAMPLE_EVERY),
          "last_sent": 0.0,
        }

    st = _NETWORK_EXCEPTION_SAMPLE_STATE[key]
    st["count"] = float(st.get("count", 0.0) + 1.0)
    count = int(st["count"])
    next_at = int(st.get("next", float(SENTRY_NETWORK_SAMPLE_EVERY)))
    last_sent = float(st.get("last_sent", 0.0))

    if count < next_at:
      return
    if SENTRY_NETWORK_SAMPLE_MIN_INTERVAL_SEC > 0 and (now - last_sent) < SENTRY_NETWORK_SAMPLE_MIN_INTERVAL_SEC:
      return

    st["next"] = float(count + SENTRY_NETWORK_SAMPLE_EVERY)
    st["last_sent"] = now

  try:
    with sentry_sdk.push_scope() as scope:
      scope.set_tag("sampled_network_error", True)
      scope.set_tag("sample_every", SENTRY_NETWORK_SAMPLE_EVERY)
      scope.set_extra("suppressed_count", count)
      scope.set_extra("exception_type", exc.__class__.__name__)
      scope.set_extra("exception_str", str(exc)[:2000])
      scope.fingerprint = ["teletyped", "sampled-network-error", key]
      sentry_sdk.capture_message(
        f"Sampled network error ({count} occurrences, 1/{SENTRY_NETWORK_SAMPLE_EVERY}): {exc.__class__.__name__}",
        level="error",
      )
  except Exception:
    pass

def _init_sentry() -> None:
  global _SENTRY_INITIALIZED
  if _SENTRY_INITIALIZED:
    return

  dsn = os.environ.get("GCS_SENTRY_DSN", SENTRY_DSN_DEFAULT)
  if not dsn:
    return

  def _before_send(event, hint):
    exc_info = hint.get("exc_info") if hint else None
    if exc_info:
      _, exc, _ = exc_info
      if exc is not None and _should_filter_exception(exc):
        return None
    return event

  init_kwargs = {
    "dsn": dsn,
    "send_default_pii": True,
    "before_send": _before_send,
  }

  traces_rate = os.environ.get("GCS_SENTRY_TRACES")
  if traces_rate:
    try:
      init_kwargs["traces_sample_rate"] = float(traces_rate)
    except ValueError:
      pass

  sample_rate = os.environ.get("GCS_SENTRY_SAMPLE_RATE")
  if sample_rate:
    try:
      init_kwargs["sample_rate"] = float(sample_rate)
    except ValueError:
      pass

  sentry_sdk.init(**init_kwargs)  # type: ignore[arg-type]
  sentry_sdk.set_tag("component", "teletyped")
  env_name = os.environ.get("GCS_ENVIRONMENT") or os.environ.get("OPENPILOT_ENV", "device")
  sentry_sdk.set_tag("environment", env_name)
  _SENTRY_INITIALIZED = True


def capture_exception(exc: BaseException) -> None:
  if _should_filter_exception(exc):
    _maybe_capture_sampled_network_error(exc)
    return
  try:
    hub = sentry_sdk.Hub.current
    if hub and hub.client:
      sentry_sdk.capture_exception(exc)
  except Exception:
    pass


_init_sentry()

LOCAL_PORT_ENV = "TELETYPED_LOCAL_SSH_PORT"


def _detect_local_port() -> int:
  env_val = os.environ.get(LOCAL_PORT_ENV)
  if env_val:
    try:
      port = int(env_val)
      if 0 < port < 65536:
        return port
    except ValueError:
      pass

  if os.path.exists("/EON"):
    return 8022
  if os.path.exists("/TICI"):
    return 22
  return 22


REALDATA_DIR = _realdata_root()
BOOT_DIR = os.path.join(REALDATA_DIR, "boot")
REMOTE_USER = "ubuntu"
REMOTE_HOST = "goranconnect.duckdns.org"
REMOTE_PORT = 2222
LOCAL_PORT = _detect_local_port()
PIDFILE = "/tmp/reverse_ssh_tunnel.pid"
TELETYPED_DIR = os.path.dirname(__file__)
WORMHOLE_BINARY = os.path.join(TELETYPED_DIR, "wormhole-william")
SENDER_LOG = os.path.join(TELETYPED_DIR, "sender_log.json")
SETUP_RESOLV = os.path.join(TELETYPED_DIR, "setup_resolv.sh")
RESOLV_DEST = "/etc/resolv.conf"

NetworkType = cereal_log.DeviceState.NetworkType

def has_internet_connection() -> bool:
  """Check if the device currently has any network connectivity."""
  try:
    if HARDWARE.get_network_type() != NetworkType.none:
      return True
  except Exception:
    pass

  # Fallback to a quick TCP probe to avoid false offline when DBus/NM flakes
  try:
    with socket.create_connection(("8.8.8.8", 53), timeout=2):
      return True
  except OSError:
    return False
  except Exception:
    return False

def get_dongle_id() -> str:
  """
  Returns the device's dongle ID from params or fallback file.
  Defaults to 'UNKNOWN_DEVICE' if not found.
  """
  params = Params()
  dongle_id = cast(str | None, params.get("DongleId", encoding='utf8'))

  if dongle_id is None:
    fallback_path = Path(PERSIST_ROOT) / "comma" / "dongle_id"
    if fallback_path.is_file():
      with open(fallback_path) as f:
        dongle_id = f.read().strip()

  return dongle_id if dongle_id else "UNKNOWN_DEVICE"

def _load_registration_private_key() -> str | None:
  if not os.path.exists(REGISTRATION_KEY_PATH):
    return None
  try:
    with open(REGISTRATION_KEY_PATH) as f:
      return f.read()
  except Exception:
    return None


def build_device_jwt(device_id: str | None, ttl_seconds: int = DEVICE_JWT_TTL_SECONDS) -> str | None:
  device_id = device_id or get_dongle_id()
  private_key = _load_registration_private_key()
  if not device_id or not private_key:
    return None

  now = datetime.now(timezone.utc)  # noqa: UP017
  payload = {
    "device_id": device_id,
    "sub": device_id,
    "iat": int(now.timestamp()),
    "nbf": int(now.timestamp()),
    "exp": int((now + timedelta(seconds=ttl_seconds)).timestamp()),
  }

  try:
    token = jwt.encode(payload, private_key, algorithm="RS256")
    return str(token)
  except Exception:
    return None


def _get_cached_device_jwt() -> str | None:
  now = time.monotonic()
  token_value = _DEVICE_JWT_CACHE.get("token")
  token: str | None = token_value if isinstance(token_value, str) else None

  refresh_at_value = _DEVICE_JWT_CACHE.get("refresh_at")
  refresh_at = float(refresh_at_value) if isinstance(refresh_at_value, (int, float)) else 0.0

  if token and now < refresh_at:
    return token

  token = build_device_jwt(get_dongle_id())
  if token:
    _DEVICE_JWT_CACHE["token"] = token
    _DEVICE_JWT_CACHE["refresh_at"] = now + max(1, DEVICE_JWT_TTL_SECONDS - DEVICE_JWT_REFRESH_LEEWAY)
  else:
    _DEVICE_JWT_CACHE["token"] = ""
    _DEVICE_JWT_CACHE["refresh_at"] = 0.0
  return token


def build_auth_headers(
) -> dict[str, str]:
  """Return headers carrying the device JWT (JWT-only auth)."""
  headers: dict[str, str] = {}
  device_jwt = _get_cached_device_jwt()
  if device_jwt:
    headers["Authorization"] = f"Bearer {device_jwt}"
    headers["X-Device-JWT"] = device_jwt
  return headers


def ensure_dns_config() -> None:
  """Ensure the device has a working resolver before making network calls."""
  try:
    with open(RESOLV_DEST, encoding="utf-8", errors="ignore") as f:
      if "nameserver" in f.read():
        return
  except OSError:
    pass

  if not os.path.isfile(SETUP_RESOLV):
    log("DNS setup script missing; skipping resolver configuration", "WARN")
    return

  shell_path = shutil.which("bash") or shutil.which("sh") or "/system/bin/sh"
  if not os.path.exists(shell_path):
    log(f"No suitable shell found to run {SETUP_RESOLV}", "ERROR")
    return

  try:
    subprocess.run([shell_path, SETUP_RESOLV], check=True)
    log("DNS resolver configured via setup_resolv.sh")
  except subprocess.CalledProcessError as exc:
    log(f"Failed to configure DNS ({exc})", "ERROR")

def log(msg, level="INFO"):
  print(f"[{datetime.now().isoformat()}] [{level}] {msg}")

def _dns_cache_load_unlocked() -> None:
  global _DNS_CACHE_LOADED, _DNS_CACHE
  if _DNS_CACHE_LOADED:
    return
  _DNS_CACHE_LOADED = True
  try:
    with open(DNS_CACHE_PATH, encoding="utf-8") as f:
      data = json.load(f)
    hosts = data.get("hosts") if isinstance(data, dict) else None
    if isinstance(hosts, dict):
      _DNS_CACHE = hosts
  except OSError:
    pass
  except Exception:
    _DNS_CACHE = {}


def _dns_cache_save_unlocked(force: bool = False) -> None:
  global _DNS_CACHE_DIRTY, _DNS_CACHE_LAST_SAVE
  if not _DNS_CACHE_DIRTY and not force:
    return
  now = time.time()
  if not force and (now - _DNS_CACHE_LAST_SAVE) < DNS_CACHE_SAVE_MIN_SEC:
    return

  payload = {"version": 1, "hosts": _DNS_CACHE}
  tmp_path = f"{DNS_CACHE_PATH}.tmp"
  try:
    os.makedirs(os.path.dirname(DNS_CACHE_PATH), exist_ok=True)
    with open(tmp_path, "w", encoding="utf-8") as f:
      json.dump(payload, f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp_path, DNS_CACHE_PATH)
    _DNS_CACHE_LAST_SAVE = now
    _DNS_CACHE_DIRTY = False
  except Exception:
    try:
      if os.path.exists(tmp_path):
        os.remove(tmp_path)
    except Exception:
      pass


def get_cached_ip(host: str) -> str | None:
  with _DNS_CACHE_LOCK:
    _dns_cache_load_unlocked()
    entry = _DNS_CACHE.get(host)
    if isinstance(entry, dict):
      ip = entry.get("ip")
      if isinstance(ip, str) and ip:
        return ip
  return None


def _set_cached_ip(host: str, ip: str, *, resolved_at: float | None = None) -> None:
  global _DNS_CACHE_DIRTY
  resolved_at = time.time() if resolved_at is None else float(resolved_at)
  with _DNS_CACHE_LOCK:
    _dns_cache_load_unlocked()
    entry = _DNS_CACHE.get(host)
    if not isinstance(entry, dict):
      entry = {}
      _DNS_CACHE[host] = entry
    if entry.get("ip") != ip:
      entry["ip"] = ip
      entry["updated_at"] = resolved_at
      _DNS_CACHE_DIRTY = True
    else:
      # Keep updated_at fresh if we re-resolved the same IP.
      entry["updated_at"] = resolved_at
      _DNS_CACHE_DIRTY = True
    _dns_cache_save_unlocked()


def _mark_cached_ip_used(host: str) -> None:
  global _DNS_CACHE_DIRTY
  with _DNS_CACHE_LOCK:
    _dns_cache_load_unlocked()
    entry = _DNS_CACHE.get(host)
    if not isinstance(entry, dict):
      return
    entry["last_used_at"] = time.time()
    _DNS_CACHE_DIRTY = True
    _dns_cache_save_unlocked()


def _pick_ip_from_getaddrinfo(addrs: list[tuple]) -> str | None:
  ipv4 = None
  ipv6 = None
  for _family, _socktype, _proto, _canonname, sockaddr in addrs:
    if not sockaddr or len(sockaddr) < 2:
      continue
    ip = sockaddr[0]
    if not isinstance(ip, str):
      continue
    if ":" in ip:
      ipv6 = ipv6 or ip
    else:
      ipv4 = ipv4 or ip
  return ipv4 or ipv6


def maybe_refresh_cached_ip(host: str, port: int) -> str | None:
  now = time.time()
  with _DNS_CACHE_LOCK:
    _dns_cache_load_unlocked()
    entry = _DNS_CACHE.get(host)
    updated_at = None
    if isinstance(entry, dict):
      updated_at = entry.get("updated_at")
    try:
      updated_at_f = float(updated_at) if updated_at is not None else 0.0
    except Exception:
      updated_at_f = 0.0
    should_refresh = (now - updated_at_f) >= DNS_CACHE_REFRESH_SEC

  if not should_refresh:
    return get_cached_ip(host)

  try:
    addrs = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
    ip = _pick_ip_from_getaddrinfo(addrs)
    if ip:
      _set_cached_ip(host, ip, resolved_at=now)
    return ip
  except Exception:
    return get_cached_ip(host)


def resolve_host_for_connection(host: str, port: int) -> str:
  """
  Return a connectable host string. Prefers live DNS; falls back to cached IP.
  Intended for non-TLS uses (e.g., ssh) where connecting to an IP is fine.
  """
  ip = None
  try:
    addrs = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
    ip = _pick_ip_from_getaddrinfo(addrs)
    if ip:
      _set_cached_ip(host, ip)
      return ip
  except Exception:
    ip = get_cached_ip(host)
  return ip or host


def _iter_exception_chain(exc: BaseException):
  seen: set[int] = set()
  cur: BaseException | None = exc
  while cur is not None and id(cur) not in seen:
    seen.add(id(cur))
    yield cur
    cur = cur.__cause__ or cur.__context__


def _is_dns_resolution_error(exc: BaseException) -> bool:
  needles = (
    "no address associated with hostname",
    "temporary failure in name resolution",
    "name or service not known",
    "failed to resolve",
    "name resolution",
  )
  for e in _iter_exception_chain(exc):
    if isinstance(e, socket.gaierror):
      return True
    if e.__class__.__name__ in {"NameResolutionError"}:
      return True
    msg = str(e).lower()
    if any(n in msg for n in needles):
      return True
  return False


@contextmanager
def _force_getaddrinfo(host: str, ip: str):
  orig_getaddrinfo = socket.getaddrinfo

  def patched(name, port, family=0, type=0, proto=0, flags=0):  # noqa: A002
    if name == host:
      return orig_getaddrinfo(ip, port, family, type, proto, flags)
    return orig_getaddrinfo(name, port, family, type, proto, flags)

  with _DNS_PATCH_LOCK:
    socket.getaddrinfo = patched
    try:
      yield
    finally:
      socket.getaddrinfo = orig_getaddrinfo


_requests: ModuleType | None
try:
  _requests = importlib.import_module("requests")
except ModuleNotFoundError:
  _requests = None


def http_request(method: str, url: str, **kwargs):
  """
  requests.request() with a DNS fallback:
  - Normal attempt (uses OS resolver).
  - On DNS resolution failure, retry by temporarily forcing getaddrinfo(host)->cached_ip,
    which preserves HTTPS SNI/certificate verification because the URL still uses the hostname.
  """
  if _requests is None:
    raise RuntimeError("The 'requests' package is required for teletyped HTTP calls.")

  parsed = urlparse(url)
  host = parsed.hostname
  scheme = (parsed.scheme or "").lower()
  port = parsed.port or (443 if scheme == "https" else 80)

  try:
    resp = _requests.request(method, url, **kwargs)
    if host:
      maybe_refresh_cached_ip(host, port)
      _mark_cached_ip_used(host)
    return resp
  except Exception as e:
    if not DNS_FALLBACK_ENABLED or not host or not _is_dns_resolution_error(e):
      raise

    cached_ip = get_cached_ip(host)
    if not cached_ip:
      raise

    # Try to repair resolver state, then retry using cached IP for name resolution.
    ensure_dns_config()
    log(f"DNS failed for {host}; retrying via cached IP {cached_ip}", "WARN")
    with _force_getaddrinfo(host, cached_ip):
      resp = _requests.request(method, url, **kwargs)
      _mark_cached_ip_used(host)
      return resp


def http_get(url: str, **kwargs):
  return http_request("GET", url, **kwargs)


def http_post(url: str, **kwargs):
  return http_request("POST", url, **kwargs)

def _read_first_line(path: str) -> str | None:
  try:
    with open(path) as f:
      return f.readline().strip()
  except Exception:
    return None

def _read_os_release() -> dict:
  out = {}
  try:
    with open("/etc/os-release") as f:
      for line in f:
        line = line.strip()
        if not line or "=" not in line:
          continue
        k, v = line.split("=", 1)
        v = v.strip().strip('"').strip("'")
        out[k] = v
  except Exception:
    pass
  return out

def _which(cmd: str) -> bool:
  return shutil.which(cmd) is not None

def _getprop(prop: str) -> str:
  try:
    if _which("getprop"):
      return subprocess.check_output(["getprop", prop], text=True).strip()
  except Exception:
    pass
  return ""

def get_os_info() -> dict:
  """
  Returns a dict describing the OS the device is running.
  Keys:
    platform: 'NEOS' | 'AGNOS' | 'Linux' | 'Android' | 'Unknown'
    version:  normalized version when available (e.g., '20' for NEOS, '15' for AGNOS, '6.0.1' for Android)
    display:  human-friendly string shown in UIs (e.g., 'NEOS 20', 'AGNOS 15', 'Ubuntu 22.04.4 LTS', 'Android 6.0.1')
    extras:   optional dict with raw fields (PRETTY_NAME, ro.build fields, etc.)
  """
  # NEOS: /VERSION contains just the number (e.g., "20")
  neos_ver = _read_first_line("/VERSION")
  if neos_ver and re.fullmatch(r"\d+", neos_ver):
    android_rel = _getprop("ro.build.version.release")
    display = f"NEOS {neos_ver}"
    if android_rel:
      display += f" (Android {android_rel})"
    return {
      "platform": "NEOS",
      "version": neos_ver,
      "display": display,
      "extras": {
        "android_release": android_rel or None,
        "ro.build.display.id": _getprop("ro.build.display.id") or None,
      },
    }

  # AGNOS / general Linux: /etc/os-release
  osr = _read_os_release()
  if osr:
    pretty = osr.get("PRETTY_NAME") or ""
    id_like = (osr.get("ID") or "").lower()
    ver_id = osr.get("VERSION_ID") or ""
    if "agnos" in pretty.lower() or id_like == "agnos":
      return {
        "platform": "AGNOS",
        "version": ver_id or re.sub(r"[^0-9.]", "", pretty) or None,
        "display": pretty or (f"AGNOS {ver_id}" if ver_id else "AGNOS"),
        "extras": {"os_release": osr},
      }
    # other linux (PC/dev)
    return {
      "platform": "Linux",
      "version": ver_id or None,
      "display": pretty or f"Linux ({py_platform.platform()})",
      "extras": {"os_release": osr},
    }

  # Plain Android (rare without NEOS markers, but handle it)
  if _which("getprop"):
    android_rel = _getprop("ro.build.version.release") or "unknown"
    return {
      "platform": "Android",
      "version": android_rel,
      "display": f"Android {android_rel}",
      "extras": {
        "ro.build.display.id": _getprop("ro.build.display.id") or None,
        "ro.product.model": _getprop("ro.product.model") or None,
        "ro.product.device": _getprop("ro.product.device") or None,
      },
    }

  return {"platform": "Unknown", "version": None, "display": "Unknown", "extras": {}}

def get_hardware_info() -> dict:
  """
  Returns a dict describing the device hardware.
  Keys:
    type:   'comma three' | 'comma three X' | 'comma two / EON' | 'PC' | 'Unknown'
    model:  best-effort specific model (e.g., 'tici', 'tizi', 'OnePlus3' / 'enchilada')
    name:   low-level impl name (e.g., 'Tici', 'Tizi', 'Eon', 'PC')
  """
  # Prefer openpilot's hardware binding when available
  try:
    hw_name = type(HARDWARE).__name__
  except Exception:
    hw_name = ""
  try:
    hw_type = str(HARDWARE.get_device_type()).lower()
  except Exception:
    hw_type = ""

  # PC short-circuit
  if 'PC' in globals() and PC:
    return {"type": "PC", "model": py_platform.machine(), "name": hw_name or "PC"}

  name_l = hw_name.lower()
  hints = {name_l, hw_type}
  if "tizi" in hints:
    return {"type": "comma three X", "model": "tizi", "name": hw_name or "Tizi"}
  if "tici" in hints:  # comma three
    return {"type": "comma three", "model": "tici", "name": hw_name or "Tici"}

  if "eon" in name_l:
    # Covers EON & comma two era (NEOS Android phone-based)
    mdl = _getprop("ro.product.model") or _getprop("ro.product.device") or "unknown"
    return {"type": "comma two / EON", "model": mdl, "name": hw_name or "Eon"}

  # Heuristics if HARDWARE name wasn't informative:
  if os.path.exists("/TICI"):
    return {"type": "comma three", "model": "tici", "name": "Tici"}
  if os.path.exists("/TIZI"):
    return {"type": "comma three X", "model": "tizi", "name": "Tizi"}
  if os.path.exists("/EON") or os.path.exists("/system/build.prop"):
    mdl = _getprop("ro.product.model") or _getprop("ro.product.device") or "unknown"
    return {"type": "comma two / EON", "model": mdl, "name": "Eon"}

  return {"type": "Unknown", "model": None, "name": hw_name or "Unknown"}

def get_op_params_info() -> dict:
  """
  Read OP build info directly from Params (same style as DongleId).
  Keys returned when present:
    - version      (Params: 'Version' or fallback 'OpenpilotVersion')
    - git_branch   (Params: 'GitBranch')
    - git_commit   (Params: 'GitCommit')
  """
  params = Params()

  def _p(k: str) -> str | None:
    try:
      v = params.get(k, encoding='utf8')
      return v.strip() if v else None
    except Exception:
      return None

  def _pb(k: str) -> bool | None:
    try:
      return cast(bool, params.get_bool(k))
    except Exception:
      return None

  info: dict[str, object | None] = {
    "version": _p("Version") or _p("OpenpilotVersion"),
    "git_branch": _p("GitBranch"),
    "git_commit": _p("GitCommit"),
    "update_available": _pb("UpdateAvailable"),
    "last_update_time": _p("LastUpdateTime"),
    "update_failed_count": _p("UpdateFailedCount"),
    "update_exception": _p("LastUpdateException"),
    # Always include DisablePowerDown state for server visibility; default to False when missing
    "disable_power_down": bool(_pb("DisablePowerDown")),
  }

  failed_count_raw = info.get("update_failed_count")
  if isinstance(failed_count_raw, str):
    try:
      info["update_failed_count"] = int(failed_count_raw)
    except ValueError:
      info.pop("update_failed_count", None)
  else:
    info.pop("update_failed_count", None)

  # Drop Nones to avoid sending nulls
  return {k: v for k, v in info.items() if v is not None}
