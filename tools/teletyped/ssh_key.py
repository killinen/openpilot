#!/usr/bin/env python3
import os
import time
import subprocess
import requests
import sentry_sdk
from datetime import datetime
from common.params import Params
from tools.teletyped.helper import log, get_dongle_id, get_api_token, KEY_PATH, KEY_PATH_PRIV

# === Configuration ===
# KEY_PATH = "/persist/comma/id_ed25519_goranconnect.pub"
# KEY_PATH_PRIV = "/persist/comma/id_ed25519_goranconnect"
API_URL_KEY = "https://goranconnect.duckdns.org/upload-key"
TIMEOUT = 5
MAX_RETRIES = 5
RETRY_DELAY = 2
API_TOKEN = get_api_token()

sentry_sdk.init(
    dsn="https://82a4222b21bdd8e738c0f20677110918@o1107536.ingest.us.sentry.io/4509169784848384",
    send_default_pii=True,
)

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
        raise RuntimeError(f"Failed to generate SSH key: {e}")

def ensure_ssh_key():
    if not os.path.exists(KEY_PATH) or not os.path.exists(KEY_PATH_PRIV):
        generate_ssh_key()

def read_public_key():
    if not os.path.exists(KEY_PATH):
        raise FileNotFoundError(f"Missing SSH key at {KEY_PATH}")
    with open(KEY_PATH, "r") as f:
        return f.read().strip()

def send_ssh_key():
    device_id = get_dongle_id()
    ensure_ssh_key()
    public_key = read_public_key()
    payload = {
        "device_id": device_id,
        "public_key": public_key
    }
    headers = {
        "Authorization": f"Bearer {API_TOKEN}"
    }

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            log(f"[{attempt}/{MAX_RETRIES}] Sending SSH key to server...")
            response = requests.post(API_URL_KEY, json=payload, headers=headers, timeout=TIMEOUT)
            response.raise_for_status()
            log(f"[✓] SSH key uploaded successfully for device {device_id}")
            return
        except requests.RequestException as e:
            log(f"[!] Attempt {attempt} failed: {e}", level="ERROR")
            if attempt < MAX_RETRIES:
                time.sleep(RETRY_DELAY)
            else:
                log("[✗] Giving up after max retries.", level="FAIL")
                raise

if __name__ == "__main__":
    try:
        send_ssh_key()
    except Exception as e:
        sentry_sdk.capture_exception(e)
        log(f"[!] Uncaught error: {e}", level="CRITICAL")

