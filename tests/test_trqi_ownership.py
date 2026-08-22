import io
import subprocess
import sys
import types
from argparse import Namespace

import pytest

from openpilot.tools.trqi_updater import cli, daemon, ownership
from openpilot.tools.trqi_updater.output import Reporter
from openpilot.tools.trqi_updater.state import StateStore


@pytest.fixture
def lock_path(tmp_path, monkeypatch):
  path = tmp_path / "global-owner.lock"
  monkeypatch.setattr(ownership, "LOCK_PATH", path)
  return path


def test_exclusion_and_release(lock_path):
  first, second = ownership.UpdateOwnership(), ownership.UpdateOwnership()
  assert first.acquire()
  assert not second.acquire()
  first.release()
  assert second.acquire()
  second.release()
  assert lock_path.exists()  # Never unlink the inode other processes lock.


def test_process_death_releases_lock(lock_path):
  script = "import fcntl,sys,time; f=open(sys.argv[1], 'a+'); fcntl.flock(f, fcntl.LOCK_EX); print('locked', flush=True); time.sleep(60)"
  process = subprocess.Popen([sys.executable, "-c", script, str(lock_path)], stdout=subprocess.PIPE, text=True)
  contender = ownership.UpdateOwnership()
  try:
    assert process.stdout.readline().strip() == "locked"
    assert not contender.acquire()
    process.kill()
    process.wait(timeout=5)
    assert contender.acquire()
  finally:
    contender.release()
    if process.poll() is None:
      process.kill()
      process.wait(timeout=5)


def test_cli_busy_does_not_publish_or_dispatch(lock_path, monkeypatch):
  owner = ownership.UpdateOwnership()
  assert owner.acquire()
  monkeypatch.setattr(cli, "_main_owned", lambda *args: pytest.fail("must not dispatch"))
  monkeypatch.setattr(Reporter, "_publish", lambda *args, **kwargs: pytest.fail("must not publish"))
  try:
    # Even a different data directory shares the same global lease.
    assert cli.main(["flash", "--data-dir", "/tmp/separate-bench-state",
                     "--manifest", "a.manifest", "--image", "a.bin"]) == 14
  finally:
    owner.release()


def test_cli_exception_releases_ownership(lock_path, monkeypatch):
  def fail(*args):
    raise RuntimeError("test")
  monkeypatch.setattr(cli, "_main_owned", fail)
  with pytest.raises(RuntimeError):
    cli.main(["info"])
  owner = ownership.UpdateOwnership()
  assert owner.acquire()
  owner.release()


def fake_params(monkeypatch):
  module = types.ModuleType("openpilot.common.params")
  class Params:
    def get_bool(self, key):
      return False
  module.Params = Params
  monkeypatch.setitem(sys.modules, "openpilot.common.params", module)


def test_daemon_does_not_touch_live_session(lock_path, tmp_path, monkeypatch):
  fake_params(monkeypatch)
  owner = ownership.UpdateOwnership()
  assert owner.acquire()
  monkeypatch.setattr(StateStore, "clear_stale_active", lambda *a: pytest.fail("live state changed"))
  monkeypatch.setattr(Reporter, "_publish", lambda *a, **k: pytest.fail("live UI changed"))
  def stop(seconds):
    raise KeyboardInterrupt
  monkeypatch.setattr(daemon.time, "sleep", stop)
  try:
    with pytest.raises(KeyboardInterrupt):
      daemon.run(Namespace(data_dir=str(tmp_path)), Reporter(publish_params=True))
  finally:
    owner.release()


def test_daemon_releases_ownership_while_idle(lock_path, tmp_path, monkeypatch):
  fake_params(monkeypatch)
  monkeypatch.setattr(StateStore, "clear_stale_active", lambda *a: None)
  def stop(seconds):
    contender = ownership.UpdateOwnership()
    assert contender.acquire()
    contender.release()
    raise KeyboardInterrupt
  monkeypatch.setattr(daemon.time, "sleep", stop)
  assert daemon.run(Namespace(data_dir=str(tmp_path)), Reporter(stream=io.StringIO())) == 0
