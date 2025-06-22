import os
import json
import requests
from datetime import datetime
import re
from tools.teletyped.helper import get_dongle_id, get_api_token, log, API_URL, REALDATA_DIR, BOOT_DIR

# === Config ===
TIMEOUT = 5
API_TOKEN = get_api_token()

# Matches folder names like "2025-04-23--18-45-21--0"
TIMESTAMP_RE = re.compile(r"^\d{4}-\d{2}-\d{2}--\d{2}-\d{2}-\d{2}--\d+$")


def get_existing_routes(device_id):
  try:
    res = requests.get(f"{API_URL}/routes/{device_id}", headers={"Authorization": f"Bearer {API_TOKEN}"}, timeout=TIMEOUT)
    res.raise_for_status()
    existing = res.json()
    return set(route.get("name") for route in existing if "name" in route)
  except Exception as e:
    log(f"[!] Failed to fetch existing routes: {e}", "ERROR")
    return set()

def list_boot_routes():
  if not os.path.exists(BOOT_DIR):
    log(f"[!] Boot route directory not found: {BOOT_DIR}", "ERROR")
    return []

  return [f"boot_{f}" for f in os.listdir(BOOT_DIR)
      if os.path.isfile(os.path.join(BOOT_DIR, f))]

def list_unique_timestamp_routes():
  try:
    all_folders = os.listdir(REALDATA_DIR)
  except FileNotFoundError:
    log(f"[!] Realdata directory not found: {REALDATA_DIR}", "ERROR")
    return []

  timestamp_set = set()
  for folder in all_folders:
    folder_path = os.path.join(REALDATA_DIR, folder)
    if not os.path.isdir(folder_path):
      continue
    if not TIMESTAMP_RE.match(folder):
      continue

    # Strip the --N suffix to get the base timestamp
    timestamp_key = folder.rsplit("--", 1)[0]
    timestamp_set.add(timestamp_key)

  return list(timestamp_set)

def send_routes_to_server(device_id, routes):
  headers = {"Authorization": f"Bearer {API_TOKEN}"}
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
    res = requests.post(f'{API_URL}/announce-routes', json=payload, headers=headers, timeout=TIMEOUT)
    res.raise_for_status()
    log("✅ Server acknowledged route list.")
  except Exception as e:
    log(f"[!] Failed to send routes: {e}", "ERROR")

def main():
  try:
    # device_id = read_dongle_id()
    device_id = get_dongle_id()
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
    log(f"[!] Failed to announce routes: {e}", "ERROR")
    return 1

if __name__ == "__main__":
  main()

