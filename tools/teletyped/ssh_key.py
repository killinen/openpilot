#!/usr/bin/env python3
import os
import time
import subprocess
from typing import Any, cast

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

try:
  import requests as _requests
except ModuleNotFoundError as err:
  raise RuntimeError("The 'requests' package is required for teletyped SSH key management.") from err

requests = cast(Any, _requests)
from openpilot.tools.teletyped.helper import (
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


def _write_ed25519_keypair() -> None:
  """Fallback key generation when ssh-keygen is unavailable on device."""
  private_key = Ed25519PrivateKey.generate()
  priv_bytes = private_key.private_bytes(
    encoding=serialization.Encoding.PEM,
    format=serialization.PrivateFormat.OpenSSH,
    encryption_algorithm=serialization.NoEncryption(),
  )

  pub_bytes = private_key.public_key().public_bytes(
    encoding=serialization.Encoding.OpenSSH,
    format=serialization.PublicFormat.OpenSSH,
  )
  pub_bytes += b" goranconnect"  # match ssh-keygen comment

  os.makedirs(os.path.dirname(KEY_PATH_PRIV), exist_ok=True)
  with open(KEY_PATH_PRIV, "wb") as f:
    f.write(priv_bytes if priv_bytes.endswith(b"\n") else priv_bytes + b"\n")
  os.chmod(KEY_PATH_PRIV, 0o600)

  with open(KEY_PATH, "wb") as f:
    f.write(pub_bytes if pub_bytes.endswith(b"\n") else pub_bytes + b"\n")


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
    ], check=True, capture_output=True, text=True)
    log("SSH key generated successfully.")
  except FileNotFoundError:
    log("ssh-keygen not available; falling back to in-process Ed25519 generation.", level="WARN")
    try:
      _write_ed25519_keypair()
      log("SSH key generated successfully via fallback.")
    except Exception as e:
      raise RuntimeError(f"Failed to generate SSH key without ssh-keygen: {e}") from e
  except subprocess.CalledProcessError as e:
    err_msg = e.stderr.strip() if isinstance(e.stderr, str) and e.stderr else str(e)
    log(f"ssh-keygen failed ({err_msg}); attempting in-process Ed25519 generation.", level="WARN")
    try:
      _write_ed25519_keypair()
      log("SSH key generated successfully via fallback.")
    except Exception as fallback_exc:
      raise RuntimeError(f"Failed to generate SSH key via fallback after ssh-keygen error: {err_msg}") from fallback_exc

def ensure_ssh_key():
  if not os.path.exists(KEY_PATH) or not os.path.exists(KEY_PATH_PRIV):
    generate_ssh_key()

def read_public_key():
  if not os.path.exists(KEY_PATH):
    raise FileNotFoundError(f"Missing SSH key at {KEY_PATH}")
  with open(KEY_PATH) as f:
    return f.read().strip()

def _auth_headers() -> dict[str, str] | None:
  token = get_api_token()
  if not token:
    return None
  return {"Authorization": f"Bearer {token}"}


def _remote_has_key(device_id: str, headers: dict[str, str]) -> bool | None:
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


def send_ssh_key(headers: dict[str, str]) -> bool:
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

  try:
    device_id = get_dongle_id()
    has_key = _remote_has_key(device_id, headers)
    if has_key is True:
      log("Server already has SSH key; skipping upload.")
      return True
    if has_key is None:
      log("Unable to determine remote SSH key status; will retry later.", level="WARN")
      return False

    return send_ssh_key(headers)
  except Exception as e:
    capture_exception(e)
    log(f"Failed to ensure/upload SSH key: {e}", level="ERROR")
    return False

if __name__ == "__main__":
  try:
    ok = send_ssh_key_if_needed()
    if not ok:
      log("SSH key upload did not complete.", level="WARN")
  except Exception as e:
    capture_exception(e)
    log(f"[!] Uncaught error: {e}", level="CRITICAL")
