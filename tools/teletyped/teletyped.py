import os
import psutil
import requests
import subprocess
import time
import signal
import argparse
from tools.teletyped import ssh_key
from tools.teletyped.helper import log, get_dongle_id, get_api_token, API_URL, POLL_INTERVAL, KEY_PATH_PRIV, REMOTE_USER, REMOTE_HOST, REMOTE_PORT, LOCAL_PORT, PIDFILE

VERBOSE = False

API_TOKEN = get_api_token()

_running = True

base_dir = os.path.dirname(os.path.realpath(__file__))

def vprint(*args, **kwargs):
  if VERBOSE:
    print(*args, **kwargs)

def signal_handler(sig, frame):
  global _running
  log("\n🛑 Caught interrupt. Exiting cleanly...", "INFO")
  _running = False


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
  headers = {"Authorization": f"Bearer {API_TOKEN}"}
  try:
    response = requests.get(url, headers=headers, timeout=5)
    return response.json() if response.status_code == 200 else {}
  except Exception as e:
    print("❌ Failed to fetch request:", e)
    return {}

def start_tunnel():
  print("🚀 Starting tunnel...")

  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        print(f"Tunnel already running with PID {pid}")
        return
    except Exception:
      pass  # Fall through and start new one

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
    print(f"Tunnel started with PID {proc.pid}")
  except Exception as e:
    print(f"❌ Failed to start tunnel: {e}")


def stop_tunnel():
  print("🛑 Stopping tunnel...")
  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        psutil.Process(pid).terminate()
        print("Tunnel stopped")
      else:
        print("PID file found but process not running")
    except Exception as e:
      print(f"Error stopping tunnel: {e}")
    finally:
      os.remove(PIDFILE)
  else:
    print("No tunnel PID file found. Is it running?")


def get_current_tunnel_status():
  if os.path.isfile(PIDFILE):
    try:
      pid = int(open(PIDFILE).read().strip())
      if psutil.pid_exists(pid):
        return "running"
    except Exception:
      pass
  return "stopped"

def report_status(device_id, status):
  url = f"{API_URL}/update-ssh-status"
  headers = {"Authorization": f"Bearer {API_TOKEN}"}
  payload = {"device_id": device_id, "status": status}
  try:
    requests.post(url, headers=headers, json=payload, timeout=5)
  except Exception as e:
    print("⚠️ Failed to report status:", e)

def reverse_ssh_step(device_id, last_reported_status):
  data = fetch_ssh_request(device_id)
  desired = data.get("request")
  current_status = get_current_tunnel_status()

  vprint(f"🧭 Desired: {desired} | Current status: {current_status}")

  should_be_running = desired is True or (
    isinstance(desired, dict) and desired.get("reverse_tunnel_req") is True
  )

  if should_be_running and current_status != "running":
    start_tunnel()
    current_status = get_current_tunnel_status()
  elif not should_be_running and current_status == "running":
    stop_tunnel()
    current_status = get_current_tunnel_status()

  if current_status != last_reported_status:
    report_status(device_id, current_status)

  return current_status


def main():
  ssh_failures = 0
  ROUTE_BACKOFF_BASE = 10
  ROUTE_BACKOFF_MAX = 300

  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)

  device_id = get_dongle_id()
  if not device_id:
    return

  # Run at startup
  setup_script = os.path.join(base_dir, "setup_resolv.sh")
  if not os.path.isfile(setup_script):
    raise FileNotFoundError(f"setup_resolv.sh not found at {setup_script}")
  subprocess.run(setup_script, check=True)

  # Check if goranconnect will respond
  check_server(API_URL)  # 👈 This blocks until server is ready

  ssh_key.send_ssh_key()

  last_ssh_time = time.monotonic()
  last_ssh_status = None

  while _running:
    now = time.monotonic()


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
        log(f"❌ reverse_ssh_step() failed: {e}", "ERROR")
      last_ssh_time = now



    time.sleep(1)

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--verbose", action="store_true", help="Enable verbose output")
  args = parser.parse_args()

  VERBOSE = args.verbose
  main()
