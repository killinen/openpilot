from types import SimpleNamespace

from cereal import log
from openpilot.system.hardware.tici import modem_watchdog
from openpilot.system.hardware.tici.modem_watchdog import Watchdog, diagnostic_snapshot, safe_offroad


def test_recovery_requires_continuous_offroad_and_unregistered():
  watchdog = Watchdog()
  assert not watchdog.observe(0, True, 'searching')
  assert not watchdog.observe(299, True, 'enabled')
  assert watchdog.observe(300, True, 'searching')
  assert not watchdog.observe(301, True, 'connected')
  assert not watchdog.observe(302, True, 'searching')
  assert not watchdog.observe(600, False, 'searching')
  assert not watchdog.observe(601, True, 'searching')
  assert watchdog.observe(901, True, 'searching')


def test_unknown_or_locked_modem_cannot_trigger_recovery():
  watchdog = Watchdog()
  for now, state in enumerate(('unknown', 'locked', 'registered', 'connecting', None)):
    assert not watchdog.observe(now * 400, True, state)


def test_requires_fresh_valid_panda_with_ignition_off(monkeypatch):
  panda = SimpleNamespace(pandaType=log.PandaState.PandaType.redPanda, ignitionLine=False, ignitionCan=False)

  class State:
    recv_time = {'pandaStates': 100.0}
    valid = True

    def all_checks(self):
      return self.valid

    def __getitem__(self, key):
      return [panda]

  sm = State()
  monkeypatch.setattr(modem_watchdog.time, 'monotonic', lambda: 101.0)
  assert safe_offroad(sm)
  panda.ignitionCan = True
  assert not safe_offroad(sm)
  panda.ignitionCan = False
  panda.pandaType = log.PandaState.PandaType.unknown
  assert not safe_offroad(sm)
  panda.pandaType = log.PandaState.PandaType.redPanda
  sm.valid = False
  assert not safe_offroad(sm)
  sm.valid = True
  sm.recv_time['pandaStates'] = 90.0
  assert not safe_offroad(sm)


def test_snapshot_omits_credentials_and_identifiers():
  snapshot = diagnostic_snapshot({
    'generic': {'state': 'searching', 'equipment-identifier': 'private-imei', 'own-numbers': ['private-number']},
    '3gpp': {'registration-state': 'searching', 'eps': {'initial-bearer': {'settings': {
      'apn': 'internet.example', 'user': 'private-user', 'password': 'private-password', 'ip-type': 'ipv4',
    }}}},
  })
  assert snapshot['initial_apn_setting'] == 'internet.example'
  assert snapshot['registration'] == 'searching'
  assert 'private-' not in str(snapshot)
