#!/usr/bin/env python3
import os
import time
import subprocess
from typing import Any, Optional, cast

try:
  import requests as _requests
except ModuleNotFoundError as err:
  raise RuntimeError("The 'requests' package is required for teletyped SSH key management.") from err

requests = cast(Any, _requests)
from tools.teletyped.helper import (
  log,
  get_dongle_id,
  get_api_token,
  KEY_PATH,
  KEY_PATH_PRIV,
  API_URL,
  capture_exception,
)

# === Configuration ===
# KEY_PATH = "/persist/comma/id_ed25519_goranconnect.pub"
# KEY_PATH_PRIV = "/persist/comma/id_ed25519_goranconnect"
API_URL_KEY = f"{API_URL}/upload-key"
API_URL_GET_KEY = f"{API_URL}/get-key"
TIMEOUT = 5
MAX_RETRIES = 5
RETRY_DELAY = 2

def generate_ssh_key():
  log(f"SSH key not found. Generating new key at {KEY_PATH_PRIV}")
  try:
    os.makedirs(os.path.dirname(KEY_PATH_PRIV), exist_ok=True)
    subprocess.run([
      "ssh-keygen",
      "-t", "ed25519",
      "-f", KEY_PATH_PRIV,
      "-N", "",  # No passphrase
      "-C", "goranconnect"
    ], check=True)
    log("SSH key generated successfully.")
  except subprocess.CalledProcessError as e:
    raise RuntimeError(f"Failed to generate SSH key: {e}") from e

def ensure_ssh_key():
  if not os.path.exists(KEY_PATH) or not os.path.exists(KEY_PATH_PRIV):
    generate_ssh_key()

def read_public_key():
  if not os.path.exists(KEY_PATH):
    raise FileNotFoundError(f"Missing SSH key at {KEY_PATH}")
  with open(KEY_PATH, "r") as f:
    return f.read().strip()

def _auth_headers() -> Optional[dict]:
  token = get_api_token()
  if not token:
    return None
  return {"Authorization": f"Bearer {token}"}


def _remote_has_key(device_id: str, headers: dict) -> Optional[bool]:
  try:
    response = requests.get(f"{API_URL_GET_KEY}/{device_id}", headers=headers, timeout=TIMEOUT)
    if response.status_code == 200:
      return True
    if response.status_code == 404:
      return False
    response.raise_for_status()
  except requests.HTTPError as exc:
    log(f"Failed to check remote SSH key: {exc}", level="WARN")
  except requests.RequestException as exc:
    log(f"Network error while checking SSH key: {exc}", level="WARN")
  return None


def send_ssh_key(headers: dict) -> bool:
  device_id = get_dongle_id()
  ensure_ssh_key()
  public_key = read_public_key()
  payload = {
    "device_id": device_id,
    "public_key": public_key
  }

  for attempt in range(1, MAX_RETRIES + 1):
    try:
      log(f"[{attempt}/{MAX_RETRIES}] Sending SSH key to server...")
      response = requests.post(API_URL_KEY, json=payload, headers=headers, timeout=TIMEOUT)
      response.raise_for_status()
      log(f"[✓] SSH key uploaded successfully for device {device_id}")
      return True
    except requests.RequestException as e:
      if isinstance(e, requests.HTTPError) and e.response is not None and e.response.status_code == 401:
        log("Unauthorized when uploading SSH key (check API token)", level="ERROR")
        return False

      log(f"[!] Attempt {attempt} failed: {e}", level="ERROR")
      if attempt < MAX_RETRIES:
        time.sleep(RETRY_DELAY)
      else:
        log("[✗] Giving up after max retries.", level="FAIL")
        return False

  return False


def send_ssh_key_if_needed() -> bool:
  headers = _auth_headers()
  if headers is None:
    log("Missing API token; skipping SSH key upload.", level="WARN")
    return False

  device_id = get_dongle_id()
  has_key = _remote_has_key(device_id, headers)
  if has_key is True:
    log("Server already has SSH key; skipping upload.")
    return True
  if has_key is None:
    log("Unable to determine remote SSH key status; will retry later.", level="WARN")
    return False

  return send_ssh_key(headers)

if __name__ == "__main__":
  try:
    ok = send_ssh_key_if_needed()
    if not ok:
      log("SSH key upload did not complete.", level="WARN")
  except Exception as e:
    capture_exception(e)
    log(f"[!] Uncaught error: {e}", level="CRITICAL")
