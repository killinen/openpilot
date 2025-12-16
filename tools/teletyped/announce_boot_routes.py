import os
from datetime import datetime
from openpilot.tools.teletyped.helper import (
  get_dongle_id,
  log,
  API_URL,
  REALDATA_DIR,
  BOOT_DIR,
  has_internet_connection,
  build_auth_headers,
  http_get,
  http_post,
  capture_exception,
)
from openpilot.tools.teletyped.label_utils import drive_base_name, is_drive_label

# === Config ===
TIMEOUT = 5

# Matches folder names like "2025-04-23--18-45-21--0"

def get_existing_routes(device_id):
  headers = build_auth_headers()
  if not headers:
    return set()
  try:
    res = http_get(f"{API_URL}/routes/{device_id}", headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    existing = res.json()
    return {route.get("name") for route in existing if "name" in route}
  except Exception as e:
    capture_exception(e)
    log(f"[!] Failed to fetch existing routes: {e}", "ERROR")
    return set()

def list_boot_routes():
  if not os.path.exists(BOOT_DIR):
    log(f"[!] Boot route directory not found: {BOOT_DIR}", "ERROR")
    return []

  boot_files = []
  for fname in os.listdir(BOOT_DIR):
    path = os.path.join(BOOT_DIR, fname)
    if not os.path.isfile(path):
      continue
    base_no_ext = fname.rsplit('.', 1)[0]
    if is_drive_label(base_no_ext):
      boot_files.append(f"boot_{fname}")
  return boot_files

def list_unique_timestamp_routes():
  try:
    all_folders = os.listdir(REALDATA_DIR)
  except FileNotFoundError:
    log(f"[!] Realdata directory not found: {REALDATA_DIR}", "ERROR")
    return []

  route_keys = set()
  for folder in all_folders:
    folder_path = os.path.join(REALDATA_DIR, folder)
    if not os.path.isdir(folder_path):
      continue
    if folder == "boot":
      continue
    if not is_drive_label(folder):
      continue
    route_keys.add(drive_base_name(folder))

  return list(route_keys)

def send_routes_to_server(device_id, routes):
  headers = build_auth_headers()
  if not headers:
    log("Missing device JWT; cannot announce routes", "WARN")
    return
  payload = {
    "device_id": device_id,
    "routes": [
      {
        "name": route_name,
        "status": "queued",
        "timestamp": datetime.utcnow().isoformat()
      } for route_name in routes
    ]
  }

  try:
    log(f"📤 Sending {len(routes)} new routes to server...")
    res = http_post(f'{API_URL}/announce-routes', json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    log("✅ Server acknowledged route list.")
  except Exception as e:
    capture_exception(e)
    log(f"[!] Failed to send routes: {e}", "ERROR")

def main():
  try:
    # device_id = read_dongle_id()
    device_id = get_dongle_id()
    if not has_internet_connection():
      log("No internet connection. Exiting.", "WARN")
      return 1
    existing_routes = get_existing_routes(device_id)

    boot_routes = list_boot_routes()
    timestamp_routes = list_unique_timestamp_routes()

    all_routes = boot_routes + timestamp_routes
    new_routes = [r for r in all_routes if r not in existing_routes]

    log(f"Found {len(all_routes)} total routes, {len(new_routes)} new.")

    if new_routes:
      send_routes_to_server(device_id, new_routes)
    else:
      log("No new routes to announce.")
    return 0
  except Exception as e:
    capture_exception(e)
    log(f"[!] Failed to announce routes: {e}", "ERROR")
    return 1

if __name__ == "__main__":
  exit_code = main()
  if exit_code not in (0, None):
    log(f"announce_boot_routes exited with code {exit_code}", "WARN")
