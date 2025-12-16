from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple, TypedDict, Type, cast
import os
import re
import subprocess
import shutil
import platform as py_platform
import time
import tempfile
import socket

import importlib
from types import ModuleType

import sentry_sdk
try:
  import jwt as _jwt
except ModuleNotFoundError:
  _jwt = None

requests_exceptions: Optional[ModuleType]
try:
  requests_exceptions = importlib.import_module("requests.exceptions")
except ModuleNotFoundError:
  requests_exceptions = None

try:
  # Newer style (e.g. `python -m openpilot.tools.teletyped.helper`)
  from openpilot.common.params import Params
  from openpilot.system.hardware import PC, HARDWARE
except ModuleNotFoundError:
  # Fallback for old-style in-tree execution
  from common.params import Params
  from system.hardware import PC, HARDWARE

from cereal import log as cereal_log


API_URL = "https://goranconnect.duckdns.org/api"
POLL_INTERVAL = 10
CHECK_INTERVAL = 60

HEARTBEAT_INTERVAL = 30

def _comma_home_default() -> str:
  suffix = os.environ.get("OPENPILOT_PREFIX", "")
  return os.path.join(str(Path.home()), ".comma" + suffix)

def _persist_root() -> str:
  if PC:
    return os.path.join(_comma_home_default(), "persist")
  # Prefer /persist when available (comma device default).
  if os.path.isdir("/persist"):
    return "/persist"
  # Older NEOS layouts may not mount /persist in minimal environments.
  if os.path.isdir("/data"):
    return "/data"
  return "/"

def _realdata_root() -> str:
  if os.path.isdir("/data/media/0/realdata"):
    return "/data/media/0/realdata"
  return os.path.join(_comma_home_default(), "media", "0", "realdata")

PERSIST_ROOT = _persist_root()

def _key_dir_candidates() -> List[str]:
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

def _resolve_key_paths() -> Tuple[str, str]:
  key_name = "id_ed25519_goranconnect"
  candidates = _key_dir_candidates()

  # Prefer an existing keypair, even if the directory is now read-only.
  for base in candidates:
    priv = os.path.join(base, key_name)
    pub = f"{priv}.pub"
    if os.path.exists(pub) and os.path.exists(priv):
      return pub, priv

  # Otherwise pick the first writable candidate.
  for base in candidates:
    if _is_writable_dir(base):
      priv = os.path.join(base, key_name)
      pub = f"{priv}.pub"
      return pub, priv

  # Last resort: drop to /tmp.
  fallback_base = "/tmp/comma"
  os.makedirs(fallback_base, exist_ok=True)
  priv = os.path.join(fallback_base, key_name)
  pub = f"{priv}.pub"
  return pub, priv

KEY_PATH, KEY_PATH_PRIV = _resolve_key_paths()
REGISTRATION_KEY_PATH = os.path.join(PERSIST_ROOT, "comma", "id_rsa")

SENTRY_DSN_DEFAULT = "https://82a4222b21bdd8e738c0f20677110918@o1107536.ingest.us.sentry.io/4509169784848384"
_SENTRY_INITIALIZED = False

NETWORK_EXCEPTION_TYPES: Tuple[Type[BaseException], ...]

if requests_exceptions is not None:
  _exception_candidates = (
    getattr(requests_exceptions, 'ConnectionError', BaseException),
    getattr(requests_exceptions, 'Timeout', BaseException),
    getattr(requests_exceptions, 'ReadTimeout', None),
  )
  _resolved: Tuple[Type[BaseException], ...] = tuple(
    exc for exc in _exception_candidates
    if isinstance(exc, type) and issubclass(exc, BaseException)
  )
  NETWORK_EXCEPTION_TYPES = _resolved
else:
  NETWORK_EXCEPTION_TYPES = tuple()


def _should_filter_exception(exc: BaseException) -> bool:
  return bool(NETWORK_EXCEPTION_TYPES) and isinstance(exc, NETWORK_EXCEPTION_TYPES)

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

  sentry_sdk.init(**init_kwargs)
  sentry_sdk.set_tag("component", "teletyped")
  env_name = os.environ.get("GCS_ENVIRONMENT") or os.environ.get("OPENPILOT_ENV", "device")
  sentry_sdk.set_tag("environment", env_name)
  _SENTRY_INITIALIZED = True


def capture_exception(exc: BaseException) -> None:
  if _should_filter_exception(exc):
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

  # Fallback probe to avoid false offline when network type reporting flakes.
  try:
    with socket.create_connection(("8.8.8.8", 53), timeout=2):
      return True
  except OSError:
    return False

def get_dongle_id() -> str:
  """
  Returns the device's dongle ID from params or fallback file.
  Defaults to 'UNKNOWN_DEVICE' if not found.
  """
  params = Params()
  dongle_id: Optional[str] = cast(Optional[str], params.get("DongleId", encoding='utf8'))

  if dongle_id is None:
    fallback_path = Path(PERSIST_ROOT) / "comma" / "dongle_id"
    if fallback_path.is_file():
      with open(fallback_path) as f:
        dongle_id = f.read().strip()

  return dongle_id if dongle_id else "UNKNOWN_DEVICE"

def get_api_token() -> str:
  """
  Returns the API token from params, or an empty string if not set.
  """
  token_bytes: Optional[bytes] = cast(Optional[bytes], Params().get("GoranConnectPassword"))
  return token_bytes.decode("utf-8") if token_bytes else ""

DEVICE_JWT_TTL_SECONDS = 300
DEVICE_JWT_REFRESH_LEEWAY = 30
_DEVICE_JWT_CACHE: Dict[str, object] = {"token": "", "refresh_at": 0.0}


TOKEN_REFRESH_MAX_AGE = 300
TOKEN_REFRESH_EMPTY_MAX_AGE = 30


class _TokenCacheEntry(TypedDict):
  token: str
  updated_at: float


_TOKEN_CACHE: _TokenCacheEntry = {
  "token": "",
  "updated_at": 0.0,
}

def _load_registration_private_key() -> Optional[str]:
  if not os.path.exists(REGISTRATION_KEY_PATH):
    return None
  try:
    with open(REGISTRATION_KEY_PATH, "r", encoding="utf-8", errors="ignore") as f:
      key = f.read().strip()
      return key if key else None
  except Exception:
    return None

def build_device_jwt(device_id: Optional[str], ttl_seconds: int = DEVICE_JWT_TTL_SECONDS) -> Optional[str]:
  device_id = device_id or get_dongle_id()
  if _jwt is None:
    return None
  private_key = _load_registration_private_key()
  if not device_id or not private_key:
    return None

  now = datetime.now(timezone.utc)
  payload = {
    "device_id": device_id,
    "sub": device_id,
    "iat": int(now.timestamp()),
    "nbf": int(now.timestamp()),
    "exp": int((now + timedelta(seconds=ttl_seconds)).timestamp()),
  }

  try:
    token = _jwt.encode(payload, private_key, algorithm="RS256")
    return str(token)
  except Exception:
    return None

def _get_cached_device_jwt() -> Optional[str]:
  now = time.monotonic()
  token_value = _DEVICE_JWT_CACHE.get("token")
  token = token_value if isinstance(token_value, str) else None

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

def _build_token_auth_headers(
  max_age_seconds: int = TOKEN_REFRESH_MAX_AGE,
  empty_max_age_seconds: int = TOKEN_REFRESH_EMPTY_MAX_AGE,
) -> Dict[str, str]:
  token = get_cached_api_token(
    max_age_seconds=max_age_seconds,
    empty_max_age_seconds=empty_max_age_seconds,
  )
  return {"Authorization": f"Bearer {token}"} if token else {}

def get_cached_api_token(
  max_age_seconds: int = TOKEN_REFRESH_MAX_AGE,
  empty_max_age_seconds: int = TOKEN_REFRESH_EMPTY_MAX_AGE,
) -> str:
  """Return the cached API token, refreshing it if the cache is stale."""
  now = time.monotonic()
  token: str = _TOKEN_CACHE["token"]
  age = now - _TOKEN_CACHE["updated_at"]
  max_age = empty_max_age_seconds if not token else max_age_seconds

  if age >= max_age:
    token = get_api_token()
    _TOKEN_CACHE["token"] = token
    _TOKEN_CACHE["updated_at"] = now

  return token


def build_auth_headers(
  max_age_seconds: int = TOKEN_REFRESH_MAX_AGE,
  empty_max_age_seconds: int = TOKEN_REFRESH_EMPTY_MAX_AGE,
) -> Dict[str, str]:
  """
  Build auth headers for server calls.

  Auth mode:
    - TELETYPED_AUTH_MODE=jwt   -> device JWT only
    - TELETYPED_AUTH_MODE=token -> API token only (Params: GoranConnectPassword)
    - unset/auto                -> API token only (no JWT fallback)
  """
  mode = (os.environ.get("TELETYPED_AUTH_MODE") or "auto").strip().lower()
  if mode == "token":
    return _build_token_auth_headers(
      max_age_seconds=max_age_seconds,
      empty_max_age_seconds=empty_max_age_seconds,
    )

  if mode == "jwt":
    device_jwt = _get_cached_device_jwt()
    return {"Authorization": f"Bearer {device_jwt}"} if device_jwt else {}

  # Default/auto: keep behavior compatible with older clients:
  # only use the API token (and skip calls entirely when missing) to avoid
  # hammering the server with unauthorized JWT attempts on devices that don't support it.
  token_headers = _build_token_auth_headers(
    max_age_seconds=max_age_seconds,
    empty_max_age_seconds=empty_max_age_seconds,
  )
  return token_headers


def ensure_dns_config() -> None:
  """Ensure the device has a working resolver before making network calls."""
  try:
    with open(RESOLV_DEST, "r", encoding="utf-8", errors="ignore") as f:
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

def _read_first_line(path: str) -> Optional[str]:
  try:
    with open(path, "r") as f:
      return f.readline().strip()
  except Exception:
    return None

def _read_os_release() -> dict:
  out = {}
  try:
    with open("/etc/os-release", "r") as f:
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
    model:  best-effort specific model (e.g., 'OnePlus3' / 'enchilada', etc.)
    name:   low-level impl name (e.g., 'Tici', 'Eon', 'PC')
  """
  # Prefer openpilot's hardware binding when available
  try:
    hw_name = type(HARDWARE).__name__
  except Exception:
    hw_name = ""

  # PC short-circuit
  if 'PC' in globals() and PC:
    return {"type": "PC", "model": py_platform.machine(), "name": hw_name or "PC"}

  name_l = hw_name.lower()
  if "tici" in name_l:  # comma three / three X
    # Try to differentiate 3 vs 3X if available (both are TICI in most builds)
    # We keep it simple & robust:
    return {"type": "comma three", "model": "tici", "name": hw_name or "Tici"}

  if "eon" in name_l:
    # Covers EON & comma two era (NEOS Android phone-based)
    mdl = _getprop("ro.product.model") or _getprop("ro.product.device") or "unknown"
    return {"type": "comma two / EON", "model": mdl, "name": hw_name or "Eon"}

  # Heuristics if HARDWARE name wasn't informative:
  if os.path.exists("/TICI"):
    return {"type": "comma three", "model": "tici", "name": "Tici"}
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

  def _p(k: str) -> Optional[str]:
    try:
      v = params.get(k, encoding='utf8')
      return v.strip() if v else None
    except Exception:
      return None

  def _pb(k: str) -> Optional[bool]:
    try:
      return cast(bool, params.get_bool(k))
    except Exception:
      return None

  info: Dict[str, Optional[object]] = {
    "version": _p("Version") or _p("OpenpilotVersion"),
    "git_branch": _p("GitBranch"),
    "git_commit": _p("GitCommit"),
    "update_available": _pb("UpdateAvailable"),
    "last_update_time": _p("LastUpdateTime"),
    "update_failed_count": _p("UpdateFailedCount"),
    "update_exception": _p("LastUpdateException"),
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
