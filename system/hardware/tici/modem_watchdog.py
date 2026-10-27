"""Recover a modem stuck searching for a network while the car is parked."""
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import subprocess
import time

CHECK_INTERVAL = 10
DIAGNOSTIC_INTERVAL = 5 * 60
UNREGISTERED_DELAY = 5 * 60
OFFROAD_DELAY = 5 * 60
PANDA_MAX_AGE = 2
ATTEMPT_FILE = Path('/tmp/modem_watchdog_attempted')  # /tmp is cleared on boot
LOG_FILE = '/data/modem_watchdog.log'
UNREGISTERED_STATES = {'searching', 'enabled'}


class Watchdog:
  def __init__(self):
    self.offroad_since = None
    self.unregistered_since = None

  def observe(self, now, safe_offroad, state):
    if not safe_offroad:
      self.offroad_since = self.unregistered_since = None
      return False
    if self.offroad_since is None:
      self.offroad_since = now
    if state not in UNREGISTERED_STATES:
      self.unregistered_since = None
      return False
    if self.unregistered_since is None:
      self.unregistered_since = now
    return now - self.offroad_since >= OFFROAD_DELAY and now - self.unregistered_since >= UNREGISTERED_DELAY


def safe_offroad(sm):
  from cereal import log
  if not sm.all_checks() or time.monotonic() - sm.recv_time['pandaStates'] > PANDA_MAX_AGE:
    return False
  pandas = sm['pandaStates']
  return bool(pandas) and all(p.pandaType != log.PandaState.PandaType.unknown and
                              not p.ignitionLine and not p.ignitionCan for p in pandas)


def modem_info():
  result = subprocess.run(['mmcli', '-m', 'any', '-J'], capture_output=True, text=True, timeout=5, check=True)
  return json.loads(result.stdout)['modem']


def diagnostic_snapshot(modem):
  """Only retain network state and APN; exclude SIM IDs, phone numbers and credentials."""
  generic = as_dict(modem.get('generic'))
  cellular = as_dict(modem.get('3gpp'))
  initial = as_dict(as_dict(cellular.get('eps')).get('initial-bearer'))
  settings = as_dict(initial.get('settings'))
  return {
    'state': generic.get('state'),
    'failure_reason': generic.get('state-failed-reason'),
    'registration': cellular.get('registration-state'),
    'packet_service': cellular.get('packet-service-state'),
    'operator_code': cellular.get('operator-code'),
    'signal_quality': generic.get('signal-quality'),
    'initial_apn_setting': settings.get('apn'),
    'initial_ip_type': settings.get('ip-type'),
  }


def as_dict(value):
  return value if isinstance(value, dict) else {}


def initial_bearer_apn(modem):
  cellular = as_dict(modem.get('3gpp'))
  initial = as_dict(as_dict(cellular.get('eps')).get('initial-bearer'))
  path = initial.get('dbus-path')
  if not isinstance(path, str) or not path.startswith('/org/freedesktop/ModemManager1/Bearer/'):
    return None
  try:
    result = subprocess.run(['mmcli', '-b', path, '-J'], capture_output=True, text=True, timeout=5, check=True)
    return json.loads(result.stdout)['bearer']['properties'].get('apn')
  except (OSError, ValueError, KeyError, subprocess.SubprocessError):
    return None


def main():
  import cereal.messaging as messaging
  logger = logging.getLogger('modem_watchdog')
  logger.setLevel(logging.INFO)
  handler = RotatingFileHandler(LOG_FILE, maxBytes=256 * 1024, backupCount=1)
  handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
  logger.addHandler(handler)
  sm = messaging.SubMaster(['pandaStates'])
  watchdog = Watchdog()
  last_check = 0.0
  last_state = None
  last_diagnostic = None
  while True:
    sm.update(1000)
    now = time.monotonic()
    if not safe_offroad(sm):
      watchdog.observe(now, False, None)
      continue
    if now - last_check < CHECK_INTERVAL:
      continue
    last_check = now
    try:
      modem = modem_info()
      state = modem['generic']['state']
      if state != last_state:
        logger.info('modem state: %s', state)
        last_state = state
      if state in UNREGISTERED_STATES and (last_diagnostic is None or now - last_diagnostic >= DIAGNOSTIC_INTERVAL):
        snapshot = diagnostic_snapshot(modem)
        snapshot['initial_bearer_apn'] = initial_bearer_apn(modem)
        logger.info('registration diagnostic: %s', json.dumps(snapshot, sort_keys=True))
        last_diagnostic = now
      elif state not in UNREGISTERED_STATES:
        last_diagnostic = None
      sm.update(0)
      if watchdog.observe(time.monotonic(), safe_offroad(sm), state) and not ATTEMPT_FILE.exists():
        # Create the per-boot marker before invoking recovery; do not retry if it fails.
        ATTEMPT_FILE.touch(exist_ok=False)
        logger.warning('modem unregistered for five minutes; restarting ModemManager once this boot')
        subprocess.run(['sudo', '-n', 'systemctl', 'restart', '--no-block', 'ModemManager'],
                       timeout=10, check=True, capture_output=True)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
      watchdog.observe(time.monotonic(), False, None)
      logger.exception('modem watchdog check or recovery failed')


if __name__ == '__main__':
  main()
