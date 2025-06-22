#!/usr/bin/env python3
import os
import time
import json
from zipfile import ZipFile, ZIP_DEFLATED, ZipInfo
import subprocess
import requests
from datetime import datetime
from tools.teletyped.helper import log, get_dongle_id, get_api_token, API_URL, WORMHOLE_BINARY, SENDER_LOG, CHECK_INTERVAL, REALDATA_DIR, BOOT_DIR

API_TOKEN = get_api_token()
TIMEOUT = 5
ZIP_EPOCH = datetime(1980, 1, 1).timestamp()
RETRY_LIMIT = 2

HEADERS = {
  "Authorization": f"Bearer {API_TOKEN}" if API_TOKEN else ""
}


def get_requested_routes(device_id):
  try:
    res = requests.get(f"{API_URL}/routes/{device_id}", headers=HEADERS, timeout=TIMEOUT)
    res.raise_for_status()
    return [r for r in res.json() if r.get("status") == "requested"]
  except Exception as e:
    log(f"Failed to fetch routes: {e}", "ERROR")
    return []


def update_status(device_id, route_name, new_status):
  payload = {
    "device_id": device_id,
    "route_name": route_name,
    "status": new_status,
  }
  try:
    res = requests.post(f"{API_URL}/update-route-status", json=payload, headers=HEADERS, timeout=TIMEOUT)
    res.raise_for_status()
    log(f"Updated status: {route_name} -> {new_status}")
  except Exception as e:
    log(f"Failed to update status for {route_name}: {e}", "ERROR")


def send_wormhole_code(code, filename, timestamp, device_id):
  included_files = []
  if filename.endswith(".zip") and os.path.exists(filename):
    try:
      with ZipFile(filename, "r") as zipf:
        included_files = zipf.namelist()
    except Exception as e:
      log(f"Could not read zip contents: {e}", "WARN")

  payload = {
    "device_id": device_id,
    "wormhole_code": code,
    "filename": filename,
    "timestamp": timestamp,
    "included_files": included_files,
  }
  res = requests.post(f"{API_URL}/birdie", json=payload, headers=HEADERS, timeout=TIMEOUT)
  res.raise_for_status()
  log("Wormhole code registered.")


def log_local_send(device_id, filename, code, timestamp):
  entry = {
    "device_id": device_id,
    "filename": filename,
    "wormhole_code": code,
    "status": "sent",
    "timestamp": timestamp,
    "logged_at": datetime.utcnow().isoformat()
  }

  log_data = []
  if os.path.exists(SENDER_LOG):
    try:
      with open(SENDER_LOG, "r") as f:
        log_data = json.load(f)
    except json.JSONDecodeError:
      log("Sender log corrupted. Overwriting.", "WARN")

  log_data.append(entry)
  with open(SENDER_LOG, "w") as f:
    json.dump(log_data, f, indent=2)

  log(f"Logged transfer: {filename}")


def send_file_wormhole(zip_path, device_id, route_name):
  timestamp = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")

  for attempt in range(1, RETRY_LIMIT + 1):
    try:
      wormhole_sent = False
      code = None

      proc = subprocess.Popen(
        [WORMHOLE_BINARY, "send", zip_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
      )

      for line in proc.stdout:
        if "Wormhole code is:" in line:
          code = line.split("Wormhole code is:")[1].strip()
          log(f"Wormhole code: {code}")
          send_wormhole_code(code, os.path.basename(zip_path), timestamp, device_id)
          wormhole_sent = True

      proc.wait()
      if proc.returncode != 0:
        raise RuntimeError(f"wormhole exited with code {proc.returncode}")

      if wormhole_sent:
        log(f"✅ File sent via wormhole: {zip_path}")
        log_local_send(device_id, os.path.basename(zip_path), code, timestamp)
        return True
      else:
        log("⚠️ Wormhole code not found in output", "WARN")

    except Exception as e:
      log(f"Attempt {attempt} failed: {e}", "ERROR")
      time.sleep(2)

  return False


def route_sender_step(device_id):
  routes = get_requested_routes(device_id)
  log(f"🔄 Route sender tick - {len(routes)} requested route(s)")

  for route in routes:
    route_name = route["name"]
    filename = route_name[5:] if route_name.startswith("boot_") else route_name

    if route_name.startswith("boot_"):
      route_path = os.path.join(BOOT_DIR, filename)
      if not os.path.exists(route_path):
        log(f"Missing file: {route_name}", "WARN")
        continue

      zip_path = f"{route_name}.zip"
      try:
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          arcname = os.path.join("boot", filename)
          zipf.write(route_path, arcname=arcname)
        log(f"📦 Zipped boot file into folder: boot/{filename}")
      except Exception as e:
        log(f"❌ Zip failed for boot file {filename}: {e}", "ERROR")
        update_status(device_id, route_name, "error")
        continue

    else:
      segments = [
        os.path.join(REALDATA_DIR, d)
        for d in os.listdir(REALDATA_DIR)
        if d.startswith(filename + "--")
      ]

      if not segments:
        log(f"Missing file(s) for prefix: {filename}", "WARN")
        continue

      zip_path = f"/tmp/{filename}.zip"
      try:
        with ZipFile(zip_path, 'w', ZIP_DEFLATED) as zipf:
          for segment in segments:
            for root, _, files in os.walk(segment):
              for file in files:
                abs_path = os.path.join(root, file)
                rel_path = os.path.relpath(abs_path, REALDATA_DIR)
                arcname = os.path.join(filename, rel_path)
                zipf.write(abs_path, arcname=arcname)
        log(f"📦 Zipped {len(segments)} segment(s) into folder: {filename}")
      except Exception as e:
        log(f"❌ Zip failed for {filename}: {e}", "ERROR")
        update_status(device_id, route_name, "error")
        continue

    update_status(device_id, route_name, "sending")

    if send_file_wormhole(zip_path, device_id, route_name):
      update_status(device_id, route_name, "sent")
    else:
      update_status(device_id, route_name, "error")

    try:
      if os.path.exists(zip_path):
        os.remove(zip_path)
        log(f"🧹 Deleted temporary zip: {zip_path}")
    except Exception as e:
      log(f"⚠️ Failed to delete zip: {e}", "WARN")


def main():
  device_id = get_dongle_id()
  log(f"👀 Monitoring for route requests: {device_id}")
  while True:
    route_sender_step(device_id)
    time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
  main()
