import argparse
import hashlib
import json
import os
import signal
import socket
import subprocess
import threading
import time

import psutil
import requests

from openpilot.tools.teletyped import route_sender, ssh_key
from openpilot.common.params import Params
from openpilot.tools.teletyped.helper import (
  log,
  get_dongle_id,
  API_URL,
  POLL_INTERVAL,
  HEARTBEAT_INTERVAL,
  PERSIST_ROOT,
  KEY_PATH_PRIV,
  REMOTE_USER,
  REMOTE_HOST,
  REMOTE_PORT,
  LOCAL_PORT,
  POND_REMOTE_PORT,
  POND_LOCAL_PORT,
  PIDFILE,
  has_internet_connection,
  get_os_info,
  get_hardware_info,
  get_op_params_info,
  ensure_dns_config,
  get_cached_ip,
  maybe_refresh_cached_ip,
  build_auth_headers,
  http_get,
  http_post,
  capture_exception,
)

from openpilot.system.hardware import HARDWARE

VERBOSE = False

_running = True
_last_desired_tunnel_state: bool | None = None
_route_sender_stop_event: threading.Event | None = None
_route_sender_thread: threading.Thread | None = None
_missing_auth_warned = False

base_dir = os.path.dirname(os.path.realpath(__file__))
SSH_LOG_PATH = "/tmp/reverse_ssh_tunnel.log"
TUNNEL_STARTING_TIMEOUT_SEC = 30
TUNNEL_CONNECT_WAIT_SEC = 12
TUNNEL_AUTH_SETTLE_SEC = 2.0
ERROR_LOG_DIR = os.environ.get("TELETYPED_ERROR_LOG_DIR", "/data/error_logs")
ERROR_LOG_STATE_FILE = "teletyped_error_logs.json"
ERROR_LOG_MAX_BYTES = int(os.environ.get("TELETYPED_ERROR_LOG_MAX_BYTES", str(5 * 1024 * 1024)))
_ERROR_LOG_EXTS = {".log", ".txt", ".json", ".jsonl"}
_ERROR_LOG_STATE_PATH: str | None = None

_SSH_ERROR_NEEDLES = (
  "permission denied",
  "authentication failed",
  "too many authentication failures",
  "no supported authentication methods available",
  "host key verification failed",
  "remote port forwarding failed",
  "cannot listen to port",
  "key_load_private",
  "identity file",
  "not accessible",
  "could not resolve hostname",
  "connection timed out",
  "connection refused",
  "kex_exchange_identification",
)


def _normalize_tunnel_request(request):
  if request is True:
    return {"reverse_tunnel_req": True, "pond_tunnel_req": False}
  if not isinstance(request, dict):
    return {"reverse_tunnel_req": False, "pond_tunnel_req": False}
  return {
    "reverse_tunnel_req": bool(request.get("reverse_tunnel_req")),
    "pond_tunnel_req": bool(request.get("pond_tunnel_req")),
  }


def _requested_forwards(request):
  requested = _normalize_tunnel_request(request)
  forwards = []
  if requested["reverse_tunnel_req"]:
    forwards.append((REMOTE_PORT, LOCAL_PORT))
  if requested["pond_tunnel_req"]:
    forwards.append((POND_REMOTE_PORT, POND_LOCAL_PORT))
  return forwards


def _forward_signature(forwards):
  return ",".join(f"{remote}:{local}" for remote, local in forwards)


def _get_params_memory():
  from openpilot.frogpilot.common.frogpilot_variables import params_memory

  return params_memory

def vprint(*args):
  if VERBOSE:
    message = " ".join(str(arg) for arg in args)
    log(message, "DEBUG")

def signal_handler(sig, frame):
  global _running
  log("🛑 Caught interrupt. Exiting cleanly...", "INFO")
  _running = False
  if _route_sender_stop_event is not None:
    _route_sender_stop_event.set()

def check_server(api_url, timeout=5, max_backoff=60):
  attempt = 0
  log(f"🌐 Waiting for {api_url} to become available...")

  while True:
    try:
      r = http_get(f"{api_url}/health", timeout=timeout)
      if r.status_code == 200:
        log("✅ Server is reachable.")
        return
      else:
        log(f"⚠️ Unexpected status: {r.status_code}", "WARN")
    except requests.exceptions.RequestException as e:
      log(f"🔌 Server unreachable: {e}", "WARN")

    backoff = min(2 ** attempt, max_backoff)
    log(f"⏳ Retrying in {backoff} seconds...")
    time.sleep(backoff)
    attempt += 1

def get_uptime_seconds():
  try:
    return max(0, int(time.time() - psutil.boot_time()))
  except Exception as e:
    vprint(f"⚠️ Failed to collect uptime info: {e}")
    return None

def fetch_ssh_request(device_id):
  url = f"{API_URL}/ssh-requests/{device_id}"
  headers = build_auth_headers()
  if not headers:
    return None
  try:
    response = http_get(url, headers=headers, timeout=5)
    return response.json() if response.status_code == 200 else {}
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to fetch request: {e}", "ERROR")
    return None


def fetch_device_actions(device_id):
  url = f"{API_URL}/device-actions/{device_id}"
  headers = build_auth_headers()
  if not headers:
    return []
  try:
    response = http_get(url, headers=headers, timeout=5)
    if response.status_code != 200:
      return []
    data = response.json()
    if isinstance(data, dict):
      return data.get("actions", [])
    if isinstance(data, list):
      return data
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to fetch device actions: {e}", "ERROR")
  return []


def acknowledge_device_action(device_id, action_id, status, message=None):
  url = f"{API_URL}/device-actions/{device_id}/ack"
  headers = build_auth_headers()
  if not headers:
    return False
  payload = {
    "action_id": action_id,
    "status": status,
  }
  if message:
    payload["message"] = str(message)
  try:
    response = http_post(url, headers=headers, json=payload, timeout=5)
    response.raise_for_status()
    return True
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to acknowledge action {action_id}: {e}", "ERROR")
    return False


def _is_comma_three():
  try:
    device = str(HARDWARE.get_device_type()).lower()
    return device in {"tici", "tizi"}  # treat both comma three and three X as supported
  except Exception:
    return False


def _ensure_disable_power_down_default():
  """
  On first boot, default DisablePowerDown to True so auto shutdown stays off until user changes it.
  Only touches the param if it doesn't already exist to avoid clobbering user preference.
  """
  try:
    if not _is_comma_three():
      return

    params = Params()
    if params.get("DisablePowerDown") is None:
      params.put_bool("DisablePowerDown", True)
      _get_params_memory().put_bool("DisablePowerDown", True)
      log("Initialized DisablePowerDown to True (auto shutdown disabled by default)", "INFO")
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to initialize DisablePowerDown param: {e}", "WARN")


def execute_device_actions(device_id):
  actions = fetch_device_actions(device_id)
  for action in actions:
    if action.get("action") not in {"reboot", "check_update", "disable_power_down", "enable_power_down"}:
      continue
    if action.get("status") != "pending":
      continue

    action_id = action.get("id")
    if not action_id:
      continue

    metadata = action.get("metadata") or {}
    reason = metadata.get("reason")

    if not acknowledge_device_action(device_id, action_id, "acknowledged"):
      continue

    if action.get("action") == "reboot":
      log_message = "🔄 Reboot requested via server command"
      if reason:
        log_message += f" (reason: {reason})"
      log(log_message, "WARN")

      try:
        HARDWARE.reboot(reason=reason)
        return True
      except Exception as e:
        capture_exception(e)
        log(f"❌ Failed to execute reboot: {e}", "ERROR")
        acknowledge_device_action(device_id, action_id, "failed", message=e)
    elif action.get("action") == "check_update":
      log("🔄 Update check requested via server command", "INFO")
      try:
        # Match UI behaviour: mark manual update requested and signal updater
        _get_params_memory().put_bool("ManualUpdateInitiated", True)
        # Align with UI behaviour: SIGUSR1 prompts updated to check for updates
        status = os.system("pkill -SIGUSR1 -f system.updated.updated")
        exit_code = status >> 8  # os.system returns exit status in the high byte
        if exit_code == 1:
          log("⚠️ Updater process not running (pkill matched nothing); update check may be skipped until updated starts.", "WARN")
        elif exit_code != 0:
          raise RuntimeError(f"pkill returned {exit_code}")
        acknowledge_device_action(device_id, action_id, "completed")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to signal updater: {e}", "WARN")
        acknowledge_device_action(device_id, action_id, "failed", message=e)
    elif action.get("action") == "disable_power_down":
      log("Disable automatic shutdown requested via server command", "INFO")
      try:
        if not _is_comma_three():
          raise RuntimeError("DisablePowerDown is only supported on comma three hardware")
        _get_params_memory().put_bool("DisablePowerDown", True)
        Params().put_bool("DisablePowerDown", True)  # persist for heartbeat / server visibility
        acknowledge_device_action(device_id, action_id, "completed")
        log("DisablePowerDown param set to True", "INFO")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to disable automatic shutdown: {e}", "WARN")
        acknowledge_device_action(device_id, action_id, "failed", message=e)
    elif action.get("action") == "enable_power_down":
      log("Re-enable automatic shutdown requested via server command", "INFO")
      try:
        if not _is_comma_three():
          raise RuntimeError("DisablePowerDown is only supported on comma three hardware")
        _get_params_memory().put_bool("DisablePowerDown", False)
        Params().put_bool("DisablePowerDown", False)  # persist for heartbeat / server visibility
        acknowledge_device_action(device_id, action_id, "completed")
        log("DisablePowerDown param set to False", "INFO")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to re-enable automatic shutdown: {e}", "WARN")
        acknowledge_device_action(device_id, action_id, "failed", message=e)
  return False


def _read_pidfile():
  if not os.path.isfile(PIDFILE):
    return None, None, None
  try:
    raw = open(PIDFILE).read().strip()
    if not raw:
      return None, None, None
    if raw.startswith("{"):
      data = json.loads(raw)
      pid = int(data.get("pid"))
      started_at = data.get("started_at")
      started_at = float(started_at) if started_at is not None else None
      forwards = data.get("forwards")
      if isinstance(forwards, list):
        forward_signature = _forward_signature(
          [
            (int(item.get("remote")), int(item.get("local")))
            for item in forwards
            if isinstance(item, dict)
            and item.get("remote") is not None
            and item.get("local") is not None
          ]
        )
      else:
        forward_signature = data.get("forward_signature")
      return pid, started_at, forward_signature
    return int(raw), None, None
  except Exception:
    return None, None, None


def _write_pidfile(pid: int, forwards) -> None:
  payload = {
    "pid": pid,
    "started_at": time.time(),
    "forward_signature": _forward_signature(forwards),
    "forwards": [
      {"remote": remote_port, "local": local_port}
      for remote_port, local_port in forwards
    ],
  }
  with open(PIDFILE, "w") as f:
    json.dump(payload, f)


def _read_ssh_log_tail(max_chars: int = 800) -> str | None:
  try:
    contents = open(SSH_LOG_PATH, encoding="utf-8", errors="replace").read().strip()
    if not contents:
      return None
    return contents[-max_chars:]
  except Exception:
    return None


def _ssh_log_error_detail() -> str | None:
  tail = _read_ssh_log_tail()
  if not tail:
    return None
  lower = tail.lower()
  if any(needle in lower for needle in _SSH_ERROR_NEEDLES):
    return tail
  return None


def _is_writable_dir(path: str) -> bool:
  try:
    os.makedirs(path, exist_ok=True)
    test_path = os.path.join(path, ".teletyped_write_test")
    with open(test_path, "w") as f:
      f.write("ok")
    os.remove(test_path)
    return True
  except OSError:
    return False


def _resolve_error_log_state_path() -> str:
  global _ERROR_LOG_STATE_PATH
  if _ERROR_LOG_STATE_PATH:
    return _ERROR_LOG_STATE_PATH

  candidates = [
    os.path.join(PERSIST_ROOT, "comma"),
    os.path.join(PERSIST_ROOT, "teletyped"),
    "/data/params/d/goranconnect_ssh",
    "/tmp/comma",
  ]

  for base in candidates:
    path = os.path.join(base, ERROR_LOG_STATE_FILE)
    if os.path.exists(path):
      _ERROR_LOG_STATE_PATH = path
      return path

  for base in candidates:
    if _is_writable_dir(base):
      _ERROR_LOG_STATE_PATH = os.path.join(base, ERROR_LOG_STATE_FILE)
      return _ERROR_LOG_STATE_PATH

  _ERROR_LOG_STATE_PATH = os.path.join("/tmp", ERROR_LOG_STATE_FILE)
  return _ERROR_LOG_STATE_PATH


def _load_error_log_state() -> dict:
  path = _resolve_error_log_state_path()
  if not os.path.isfile(path):
    return {"files": {}}
  try:
    with open(path, encoding="utf-8") as f:
      data = json.load(f)
    files = data.get("files") if isinstance(data, dict) else None
    return {"files": files if isinstance(files, dict) else {}}
  except Exception:
    return {"files": {}}


def _save_error_log_state(state: dict) -> None:
  path = _resolve_error_log_state_path()
  directory = os.path.dirname(path) or "."
  try:
    os.makedirs(directory, exist_ok=True)
    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as f:
      json.dump(state, f)
    os.replace(temp_path, path)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to save error log state: {e}", "WARN")


def _list_error_logs() -> list[tuple[str, str]]:
  if not os.path.isdir(ERROR_LOG_DIR):
    return []
  entries = []
  try:
    for name in os.listdir(ERROR_LOG_DIR):
      path = os.path.join(ERROR_LOG_DIR, name)
      if not os.path.isfile(path):
        continue
      ext = os.path.splitext(name)[1].lower()
      if ext not in _ERROR_LOG_EXTS:
        continue
      entries.append((name, path))
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to list error logs: {e}", "WARN")
    return []
  entries.sort(key=lambda item: item[0])
  return entries


def _read_error_log_bytes(path: str) -> tuple[bytes, str, int] | None:
  try:
    with open(path, "rb") as f:
      data = f.read()
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to read error log {path}: {e}", "WARN")
    return None

  size = len(data)
  if ERROR_LOG_MAX_BYTES and size > ERROR_LOG_MAX_BYTES:
    log(f"⚠️ Skipping large error log ({size} bytes): {path}", "WARN")
    return None

  digest = hashlib.sha256(data).hexdigest()
  return data, digest, size


def _upload_error_log(headers: dict[str, str], device_id: str, name: str, data: bytes, digest: str) -> bool:
  url = f"{API_URL}/error-logs/upload"
  payload = {
    "device_id": device_id,
    "original_filename": name,
    "file_sha256": digest,
  }
  files = {"file": (name, data)}

  try:
    resp = http_post(url, headers=headers, data=payload, files=files, timeout=10)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Error log upload failed ({name}): {e}", "WARN")
    return False

  if resp.status_code != 200:
    log(
      f"⚠️ Error log upload rejected ({name}) status={resp.status_code} body={resp.text.strip()[:200]}",
      "WARN",
    )
    return False

  log(f"📨 Uploaded error log {name}", "INFO")
  return True


def send_error_logs_on_startup(device_id: str) -> None:
  entries = _list_error_logs()
  if not entries:
    return

  headers = build_auth_headers()
  if not headers:
    log("⚠️ Skipping error log upload: missing device JWT", "WARN")
    return

  state = _load_error_log_state()
  seen = state.get("files") if isinstance(state, dict) else None
  if not isinstance(seen, dict):
    seen = {}

  updated = False
  for name, path in entries:
    result = _read_error_log_bytes(path)
    if result is None:
      continue
    data, digest, size = result
    prev = seen.get(name, {})
    if isinstance(prev, dict) and prev.get("sha256") == digest and prev.get("size") == size:
      continue

    if _upload_error_log(headers, device_id, name, data, digest):
      seen[name] = {"sha256": digest, "size": size}
      updated = True

  if updated:
    _save_error_log_state({"files": seen})


def _is_expected_ssh_process(proc: psutil.Process) -> bool:
  try:
    cmdline = proc.cmdline()
  except Exception:
    return False
  if not cmdline:
    return False
  expected_remote = f"{REMOTE_USER}@{REMOTE_HOST}"
  return expected_remote in cmdline and "-R" in cmdline


def _get_tunnel_process():
  pid, _started_at, _pidfile_forward_signature = _read_pidfile()
  if pid is None:
    return None
  try:
    proc = psutil.Process(pid)
  except Exception:
    return None
  if not _is_expected_ssh_process(proc):
    return None
  return proc


def _resolve_remote_ips(host: str):
  ips = set()
  try:
    for _family, _socktype, _proto, _canonname, sockaddr in socket.getaddrinfo(host, 22):
      if sockaddr and len(sockaddr) >= 2:
        ips.add(sockaddr[0])
  except Exception:
    pass
  cached = get_cached_ip(host)
  if cached:
    ips.add(cached)
  return ips


def _has_established_ssh_connection(proc: psutil.Process) -> bool:
  try:
    remote_ips = _resolve_remote_ips(REMOTE_HOST)
    connections = proc.connections(kind="inet")
    for conn in connections:
      raddr = conn.raddr
      if not raddr:
        continue
      try:
        rip = raddr.ip
        rport = raddr.port
      except Exception:
        rip = raddr[0]
        rport = raddr[1]
      if rport != 22:
        continue
      if remote_ips and rip not in remote_ips:
        continue
      if conn.status == psutil.CONN_ESTABLISHED:
        return True
  except Exception:
    return False
  return False


def start_tunnel(forwards):
  log("🚀 Starting tunnel...")

  if not forwards:
    return False, "No tunnel forwards requested"

  desired_signature = _forward_signature(forwards)

  existing_proc = _get_tunnel_process()
  if existing_proc is not None:
    _existing_pid, _started_at, existing_signature = _read_pidfile()
    if existing_signature == desired_signature:
      log(f"Tunnel already running with PID {existing_proc.pid}")
      return True, None
    log("Tunnel forwarding changed; restarting tunnel", "INFO")
    stop_tunnel()
  if os.path.isfile(PIDFILE):
    try:
      os.remove(PIDFILE)
    except Exception:
      pass

  # Ensure the local key exists; tunnel auth depends on KEY_PATH_PRIV even if key upload was skipped.
  try:
    ssh_key.ensure_local_keypair()
  except Exception:
    pass
  if not os.path.exists(KEY_PATH_PRIV):
    detail = f"Missing SSH private key at {KEY_PATH_PRIV}"
    log(f"❌ {detail}", "ERROR")
    return False, detail

  mapping = ", ".join(
    f"{remote_port}->localhost:{local_port}" for remote_port, local_port in forwards
  )
  log(f"🔁 Mapping remote ports {mapping}")

  connect_ip = None
  try:
    connect_ip = get_cached_ip(REMOTE_HOST)
    # Opportunistically refresh cache; if DNS is broken this will fall back to existing cached IP.
    connect_ip = maybe_refresh_cached_ip(REMOTE_HOST, 22) or connect_ip
  except Exception:
    pass

  cmd = [
    "ssh",
    "-i", KEY_PATH_PRIV,
    "-o", "IdentitiesOnly=yes",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "StrictHostKeyChecking=no",
    "-o", "BatchMode=yes",
    "-o", "PasswordAuthentication=no",
    "-o", "KbdInteractiveAuthentication=no",
    "-o", "NumberOfPasswordPrompts=0",
    "-o", "ExitOnForwardFailure=yes",
    "-o", "ServerAliveInterval=30",
    "-o", "ServerAliveCountMax=3",
    "-o", "TCPKeepAlive=yes",
    "-o", "ConnectTimeout=10",
    "-o", "LogLevel=ERROR",
    *([] if not connect_ip else ["-o", f"HostName={connect_ip}"]),
  ]
  for remote_port, local_port in forwards:
    cmd.extend(["-R", f"{remote_port}:localhost:{local_port}"])
  cmd.extend(["-N", f"{REMOTE_USER}@{REMOTE_HOST}"])

  try:
    with open(SSH_LOG_PATH, "w") as log_fp:
      proc = subprocess.Popen(
        cmd,
        stdin=subprocess.DEVNULL,
        stdout=log_fp,
        stderr=log_fp,
      )

    _write_pidfile(proc.pid, forwards)

    # Wait briefly for an established TCP connection or a fast failure.
    started_mono = time.monotonic()
    deadline = time.monotonic() + TUNNEL_CONNECT_WAIT_SEC
    while time.monotonic() < deadline:
      rc = proc.poll()
      if rc is not None:
        detail = _read_ssh_log_tail()
        log(f"❌ Tunnel failed to start (exit={rc})", "ERROR")
        try:
          os.remove(PIDFILE)
        except Exception:
          pass
        return False, detail or f"ssh exited with status {rc}"

      # Some ssh failures show up in stderr before the process exits; catch and surface them.
      log_detail = _ssh_log_error_detail()
      if log_detail:
        log("❌ Tunnel failed to start (ssh error in log)", "ERROR")
        try:
          proc.terminate()
        except Exception:
          pass
        try:
          os.remove(PIDFILE)
        except Exception:
          pass
        return False, log_detail

      try:
        ps_proc = psutil.Process(proc.pid)
        if _has_established_ssh_connection(ps_proc):
          # TCP connect can happen before auth/forward setup; wait a moment to avoid false "connected".
          if (time.monotonic() - started_mono) >= TUNNEL_AUTH_SETTLE_SEC:
            log(f"Tunnel started with PID {proc.pid}")
            return True, None
      except Exception:
        pass

      time.sleep(0.2)

    log(f"Tunnel process started with PID {proc.pid} (still connecting)", "INFO")
    return True, None
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to start tunnel: {e}", "ERROR")
    return False, str(e)

def stop_tunnel():
  log("🛑 Stopping tunnel...")
  if os.path.isfile(PIDFILE):
    try:
      proc = _get_tunnel_process()
      if proc is not None:
        proc.terminate()
        log("Tunnel stopped")
      else:
        log("PID file found but process not running", "WARN")
    except Exception as e:
      capture_exception(e)
      log(f"Error stopping tunnel: {e}", "ERROR")
    finally:
      os.remove(PIDFILE)
  else:
    log("No tunnel PID file found. Is it running?", "WARN")

def get_current_tunnel_status():
  proc = _get_tunnel_process()
  if proc is not None:
    log_detail = _ssh_log_error_detail()
    if log_detail:
      return "error"

    connected = _has_established_ssh_connection(proc)
    _pid, started_at, _pidfile_forward_signature = _read_pidfile()
    if connected and started_at and (time.time() - started_at) < TUNNEL_AUTH_SETTLE_SEC:
      return "starting"
    return "connected" if connected else "starting"
  if os.path.isfile(PIDFILE):
    try:
      os.remove(PIDFILE)
    except Exception:
      pass
  return "stopped"

def _read_tunnel_pid():
  proc = _get_tunnel_process()
  return proc.pid if proc is not None else None

def send_heartbeat(device_id, tunnel_status):
  global _missing_auth_warned
  internet_ok = has_internet_connection()
  if not internet_ok:
    return

  if not tunnel_status:
    tunnel_status = get_current_tunnel_status()

  url = f"{API_URL}/heartbeat"
  headers = build_auth_headers()
  if not headers:
    if not _missing_auth_warned:
      log(
        "⚠️ Skipping heartbeat: missing device JWT (registration private key not found?)",
        "WARN",
      )
      _missing_auth_warned = True
    return
  else:
    _missing_auth_warned = False

  auth_info = {
    "device_jwt": "X-Device-JWT" in headers,
  }

  payload = {
    "device_id": device_id,
    "status": "online",
    "tunnel_status": tunnel_status,
    "reverse_tunnel_requested": _last_desired_tunnel_state,
    "internet_up": internet_ok,
  }

  pid = _read_tunnel_pid()
  if pid is not None:
    payload.setdefault("details", {})["pid"] = pid

  # ===== NEW: hardware + OS details =====
  try:
    hw = get_hardware_info()
    osinfo = get_os_info()

    det = payload.setdefault("details", {})
    uptime_seconds = get_uptime_seconds()
    if uptime_seconds is not None:
      det["uptime_seconds"] = uptime_seconds

    # flattened for easy querying on the server
    det["hardware_type"]   = hw.get("type")
    det["hardware_model"]  = hw.get("model")
    det["hardware_name"]   = hw.get("name")
    det["os_platform"]     = osinfo.get("platform")
    det["os_version"]      = osinfo.get("version")
    det["os_display"]      = osinfo.get("display")

    # ===== NEW: OP params straight from Params =====
    opinfo = get_op_params_info()
    # Flatten for easy querying
    if "version" in opinfo:
      det["op_version"] = opinfo["version"]
    if "git_branch" in opinfo:
      det["git_branch"] = opinfo["git_branch"]
    if "git_commit" in opinfo:
      det["git_commit"] = opinfo["git_commit"]
    if "update_available" in opinfo:
      det["update_available"] = opinfo["update_available"]
    if "last_update_time" in opinfo:
      det["last_update_time"] = opinfo["last_update_time"]
    if "update_failed_count" in opinfo:
      det["update_failed_count"] = opinfo["update_failed_count"]
    if "update_exception" in opinfo:
      det["update_exception"] = opinfo["update_exception"]
    if "disable_power_down" in opinfo:
      det["disable_power_down"] = opinfo["disable_power_down"]

    # keep full objects too (if you prefer nested access)
    det["hardware"] = hw
    det["os"] = osinfo
    det["openpilot"] = opinfo
    det["local_ssh_port"] = LOCAL_PORT
    det["pond_local_port"] = POND_LOCAL_PORT
  except Exception as e:
    vprint(f"⚠️ Failed to collect HW/OS info: {e}")
  # =====================================

  try:
    resp = http_post(url, headers=headers, json=payload, timeout=5)
    if resp.status_code != 200:
      log(
        f"⚠️ Heartbeat rejected (status={resp.status_code}) auth={auth_info} device_id={device_id} response={resp.text.strip()[:200]}",
        "WARN",
      )
    vprint("💓 Heartbeat sent")
  except requests.RequestException as e:
    log(
      f"⚠️ Heartbeat failed: {e} auth={auth_info} device_id={device_id}",
      "WARN",
    )

def report_status(device_id, status, detail=None):
  url = f"{API_URL}/update-ssh-status"
  headers = build_auth_headers()
  if not headers:
    return
  payload = {"device_id": device_id, "status": status}
  if detail:
    payload["detail"] = str(detail)[:2000]
  try:
    http_post(url, headers=headers, json=payload, timeout=5)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to report status: {e}", "WARN")

def reverse_ssh_step(device_id, last_reported_status):
  global _last_desired_tunnel_state
  data = fetch_ssh_request(device_id)
  if data is None:
    # On fetch failure, keep previous desired state to avoid tearing down a working tunnel
    desired = _last_desired_tunnel_state
  else:
    desired = data.get("request", _last_desired_tunnel_state)
  remote_status = data.get("status") if isinstance(data, dict) else None
  current_status = get_current_tunnel_status()
  requested_forwards = _requested_forwards(desired)
  desired_signature = _forward_signature(requested_forwards)
  _pid, _started_at, current_signature = _read_pidfile()

  vprint(f"🧭 Desired: {desired} | Current status: {current_status}")

  should_be_running = bool(requested_forwards)
  _last_desired_tunnel_state = should_be_running

  error_detail = None
  if should_be_running and current_status == "stopped":
    ok, error_detail = start_tunnel(requested_forwards)
    current_status = get_current_tunnel_status() if ok else "error"
  elif should_be_running and current_signature != desired_signature:
    stop_tunnel()
    ok, error_detail = start_tunnel(requested_forwards)
    current_status = get_current_tunnel_status() if ok else "error"
  elif not should_be_running and current_status != "stopped":
    stop_tunnel()
    current_status = get_current_tunnel_status()

  if should_be_running and current_status == "error" and error_detail is None:
    error_detail = _ssh_log_error_detail() or "SSH tunnel error (see client log)"
    try:
      stop_tunnel()
    except Exception:
      pass

  if should_be_running and current_status == "starting":
    _pid, started_at, _pidfile_forward_signature = _read_pidfile()
    if started_at and (time.time() - started_at) > TUNNEL_STARTING_TIMEOUT_SEC:
      error_detail = _read_ssh_log_tail() or (
        f"SSH tunnel stuck in 'starting' for >{TUNNEL_STARTING_TIMEOUT_SEC}s"
      )
      try:
        stop_tunnel()
      except Exception:
        pass
      current_status = "error"

  # If the server is still showing the request-phase "pending" but we already know the tunnel
  # is connected/stopped/error, proactively sync the status even if our local status didn't change
  # (e.g., teletyped restarted while the tunnel was already up).
  if should_be_running and remote_status == "pending" and current_status in {"connected", "error", "stopped"}:
    if current_status == "error" and not error_detail:
      error_detail = _ssh_log_error_detail()
    if current_status == "error":
      report_status(device_id, "error", detail=error_detail or "SSH tunnel error (see client log)")
    else:
      report_status(device_id, current_status)
  elif current_status != last_reported_status:
    if current_status == "error" and not error_detail:
      error_detail = _ssh_log_error_detail()
    if error_detail:
      report_status(device_id, "error", detail=error_detail)
    else:
      report_status(device_id, current_status)

  return current_status

def _start_route_sender(device_id):
  global _route_sender_stop_event, _route_sender_thread
  _route_sender_stop_event = threading.Event()
  _route_sender_thread = threading.Thread(
    target=route_sender.run_route_sender,
    args=(_route_sender_stop_event, device_id),
    name="route_sender_thread",
    daemon=True,
  )
  _route_sender_thread.start()
  log("📦 Route sender thread started", "INFO")

def _stop_route_sender():
  global _route_sender_stop_event, _route_sender_thread
  if _route_sender_stop_event is not None:
    _route_sender_stop_event.set()
  if _route_sender_thread is not None and _route_sender_thread.is_alive():
    _route_sender_thread.join(timeout=10)
    if _route_sender_thread.is_alive():
      log("⚠️ Route sender thread did not shut down cleanly", "WARN")
    else:
      log("📦 Route sender thread stopped", "INFO")
  _route_sender_stop_event = None
  _route_sender_thread = None

def main():
  ssh_failures = 0
  ROUTE_BACKOFF_BASE = 10
  ROUTE_BACKOFF_MAX = 300

  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)

  device_id = get_dongle_id()
  if not device_id:
    log("❌ No device ID found; exiting teletyped", "ERROR")
    return

  _ensure_disable_power_down_default()

  while _running and not has_internet_connection():
    log("Waiting for internet connection...", "WARN")
    time.sleep(60)

  ensure_dns_config()
  check_server(API_URL)

  try:
    send_error_logs_on_startup(device_id)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to send error logs on startup: {e}", "WARN")

  key_uploaded = ssh_key.send_ssh_key_if_needed()
  if not key_uploaded:
    log("⚠️ SSH key upload skipped or failed; continuing without remote key registration.", "WARN")
  _start_route_sender(device_id)

  last_ssh_time = time.monotonic()
  last_ssh_status = get_current_tunnel_status()
  last_heartbeat_time = 0
  last_key_attempt = time.monotonic()

  try:
    while _running:
      now = time.monotonic()

      if not has_internet_connection():
        time.sleep(60)
        continue

      if now - last_ssh_time >= POLL_INTERVAL:
        if ssh_failures > 0:
          backoff_time = min(ROUTE_BACKOFF_BASE * (2 ** ssh_failures), ROUTE_BACKOFF_MAX)
          if now - last_ssh_time < backoff_time:
            time.sleep(1)
            continue

        try:
          last_ssh_status = reverse_ssh_step(device_id, last_ssh_status)
          ssh_failures = 0
        except Exception as e:
          ssh_failures += 1
          capture_exception(e)
          log(f"❌ reverse_ssh_step() failed: {e}", "ERROR")
        last_ssh_time = now

      execute_device_actions(device_id)

      if now - last_heartbeat_time >= HEARTBEAT_INTERVAL:
        send_heartbeat(device_id, last_ssh_status)
        last_heartbeat_time = now

      if (not key_uploaded) and (now - last_key_attempt >= 300):
        key_uploaded = ssh_key.send_ssh_key_if_needed()
        if key_uploaded:
          log("SSH key upload succeeded after retry.")
        last_key_attempt = now

      time.sleep(1)
  finally:
    _stop_route_sender()

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
  args = parser.parse_args()

  VERBOSE = args.verbose
  try:
    main()
  except Exception as e:
    capture_exception(e)
    log(f"Uncaught exception in teletyped: {e}", "CRITICAL")
    raise
