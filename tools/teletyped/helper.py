from datetime import datetime
from pathlib import Path
import os

try:
  # Newer style (e.g. `python -m openpilot.tools.teletyped.helper`)
  from openpilot.common.params import Params
  from openpilot.system.hardware import PC
except ModuleNotFoundError:
  # Fallback for old-style in-tree execution
  from common.params import Params
  from system.hardware import PC


API_URL = "https://goranconnect.duckdns.org"
POLL_INTERVAL = 10
CHECK_INTERVAL = 60
KEY_PATH = "/persist/comma/id_ed25519_goranconnect.pub"
KEY_PATH_PRIV = "/persist/comma/id_ed25519_goranconnect"
REALDATA_DIR = "/data/media/0/realdata"
BOOT_DIR = os.path.join(REALDATA_DIR, "boot")
REMOTE_USER = "ubuntu"
REMOTE_HOST = "goranconnect.duckdns.org"
REMOTE_PORT = 2222
LOCAL_PORT = 22
PIDFILE = "/tmp/reverse_ssh_tunnel.pid"
WORMHOLE_BINARY = os.path.join(os.path.dirname(__file__), "wormhole-william")
SENDER_LOG = os.path.join(os.path.dirname(__file__), "sender_log.json")

class Paths:
  @staticmethod
  def comma_home() -> str:
    return os.path.join(str(Path.home()), ".comma" + os.environ.get("OPENPILOT_PREFIX", ""))

  @staticmethod
  def persist_root() -> str:
    if PC:
      return os.path.join(Paths.comma_home(), "persist")
    else:
      return "/persist/"

def get_dongle_id() -> str:
  """
  Returns the device's dongle ID from params or fallback file.
  Defaults to 'UNKNOWN_DEVICE' if not found.
  """
  params = Params()
  dongle_id = params.get("DongleId", encoding='utf8')

  if dongle_id is None:
    fallback_path = Path(Paths.persist_root()) / "comma" / "dongle_id"
    if fallback_path.is_file():
      with open(fallback_path) as f:
        dongle_id = f.read().strip()

  return dongle_id if dongle_id else "UNKNOWN_DEVICE"

def get_api_token() -> str:
  """
  Returns the API token from params, or an empty string if not set.
  """
  token = Params().get("GoranConnectPassword")
  return token.decode("utf-8") if token else ""

def log(msg, level="INFO"):
    print(f"[{datetime.now().isoformat()}] [{level}] {msg}")
