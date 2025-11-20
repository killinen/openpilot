import argparse
import os
import signal
import subprocess
import threading
import time

import psutil
import requests

from openpilot.tools.teletyped import route_sender, ssh_key
from openpilot.tools.teletyped.helper import (
  log,
  get_dongle_id,
  API_URL,
  POLL_INTERVAL,
  HEARTBEAT_INTERVAL,
  KEY_PATH_PRIV,
  REMOTE_USER,
  REMOTE_HOST,
  REMOTE_PORT,
  LOCAL_PORT,
  PIDFILE,
  has_internet_connection,
  get_os_info,
  get_hardware_info,
  get_op_params_info,
  ensure_dns_config,
  build_auth_headers,
  capture_exception,
)

from openpilot.system.hardware import HARDWARE

VERBOSE = False

_running = True
_last_desired_tunnel_state: bool | None = None
_route_sender_stop_event: threading.Event | None = None
_route_sender_thread: threading.Thread | None = None

base_dir = os.path.dirname(os.path.realpath(__file__))

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
      r = requests.get(f"{api_url}/health", timeout=timeout)
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

def fetch_ssh_request(device_id):
  url = f"{API_URL}/ssh-requests/{device_id}"
  headers = build_auth_headers()
  if not headers:
    return {}
  try:
    response = requests.get(url, headers=headers, timeout=5)
    return response.json() if response.status_code == 200 else {}
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to fetch request: {e}", "ERROR")
    return {}


def fetch_device_actions(device_id):
  url = f"{API_URL}/device-actions/{device_id}"
  headers = build_auth_headers()
  if not headers:
    return []
  try:
    response = requests.get(url, headers=headers, timeout=5)
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
    response = requests.post(url, headers=headers, json=payload, timeout=5)
    response.raise_for_status()
    return True
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to acknowledge action {action_id}: {e}", "ERROR")
    return False


def execute_device_actions(device_id):
  actions = fetch_device_actions(device_id)
  for action in actions:
    if action.get("action") not in {"reboot", "check_update"}:
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
        ret = os.system("pkill -1 -f selfdrive.updated")
        if ret not in (0, 1):  # pkill returns 1 when nothing matched
          raise RuntimeError(f"pkill returned {ret}")
        acknowledge_device_action(device_id, action_id, "completed")
      except Exception as e:
        capture_exception(e)
        log(f"⚠️ Failed to signal updater: {e}", "WARN")
        acknowledge_device_action(device_id, action_id, "failed", message=e)
  return False


def start_tunnel():
  log("🚀 Starting tunnel...")

  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        log(f"Tunnel already running with PID {pid}")
        return
    except Exception:
      pass

  log(f"🔁 Mapping remote port {REMOTE_PORT} to localhost:{LOCAL_PORT}")

  cmd = [
    "ssh",
    "-i", KEY_PATH_PRIV,
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "StrictHostKeyChecking=no",
    "-o", "ExitOnForwardFailure=yes",
    "-R", f"{REMOTE_PORT}:localhost:{LOCAL_PORT}",
    "-N",
    f"{REMOTE_USER}@{REMOTE_HOST}"
  ]

  try:
    proc = subprocess.Popen(cmd)
    with open(PIDFILE, "w") as f:
      f.write(str(proc.pid))
    log(f"Tunnel started with PID {proc.pid}")
  except Exception as e:
    capture_exception(e)
    log(f"❌ Failed to start tunnel: {e}", "ERROR")

def stop_tunnel():
  log("🛑 Stopping tunnel...")
  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        psutil.Process(pid).terminate()
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
  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        return "running"
    except Exception:
      pass
  return "stopped"

def _read_tunnel_pid():
  if os.path.isfile(PIDFILE):
    try:
      return int(open(PIDFILE).read().strip())
    except Exception:
      return None
  return None

def send_heartbeat(device_id, tunnel_status):
  internet_ok = has_internet_connection()
  if not internet_ok:
    return

  if not tunnel_status:
    tunnel_status = get_current_tunnel_status()

  url = f"{API_URL}/heartbeat"
  headers = build_auth_headers()
  if not headers:
    return
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

    # keep full objects too (if you prefer nested access)
    det["hardware"] = hw
    det["os"] = osinfo
    det["openpilot"] = opinfo
    det["local_ssh_port"] = LOCAL_PORT
  except Exception as e:
    vprint(f"⚠️ Failed to collect HW/OS info: {e}")
  # =====================================

  try:
    requests.post(url, headers=headers, json=payload, timeout=5)
    vprint("💓 Heartbeat sent")
  except requests.RequestException as e:
    vprint(f"⚠️ Heartbeat failed: {e}")

def report_status(device_id, status):
  url = f"{API_URL}/update-ssh-status"
  headers = build_auth_headers()
  if not headers:
    return
  payload = {"device_id": device_id, "status": status}
  try:
    requests.post(url, headers=headers, json=payload, timeout=5)
  except Exception as e:
    capture_exception(e)
    log(f"⚠️ Failed to report status: {e}", "WARN")

def reverse_ssh_step(device_id, last_reported_status):
  global _last_desired_tunnel_state
  data = fetch_ssh_request(device_id)
  desired = data.get("request")
  current_status = get_current_tunnel_status()

  vprint(f"🧭 Desired: {desired} | Current status: {current_status}")

  should_be_running = desired is True or (
    isinstance(desired, dict) and desired.get("reverse_tunnel_req") is True
  )
  _last_desired_tunnel_state = should_be_running

  if should_be_running and current_status != "running":
    start_tunnel()
    current_status = get_current_tunnel_status()
  elif not should_be_running and current_status == "running":
    stop_tunnel()
    current_status = get_current_tunnel_status()

  if current_status != last_reported_status:
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

  while _running and not has_internet_connection():
    log("Waiting for internet connection...", "WARN")
    time.sleep(60)

  ensure_dns_config()
  check_server(API_URL)

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
