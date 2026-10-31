"""Real subprocess failure injection; no device, API, SSH credentials or openpilot imports."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import sysconfig

import pytest

SOURCE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("teletyped_guardian_test", SOURCE / "guardian.py")
assert spec is not None and spec.loader is not None
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
spec = importlib.util.spec_from_file_location("teletyped_deploy_test", SOURCE / "deploy.py")
assert spec is not None and spec.loader is not None
deploy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deploy)

HEALTHY = """import time
from openpilot.tools.teletyped import health
def main():
  while True:
    health.progress()
    health.success('heartbeat')
    health.success('request')
    health.report(tunnel_status='connected', local_ssh_port=22)
    time.sleep(.02)
"""


class TestGuardian:
  def setup_method(self):
    self.temp = tempfile.TemporaryDirectory()
    self.root = Path(self.temp.name) / "deployment"
    self.run_dir = Path(self.temp.name) / "run"
    self.params = Path(self.temp.name) / "params"
    self.params.mkdir()
    (self.params / "GoranConnectEnabled").write_text("1")
    (self.root / "guardian").mkdir(parents=True)
    (self.root / "guardian/bootstrap.py").write_bytes((SOURCE / "bootstrap.py").read_bytes())
    runtime = self.root / "runtimes/test-runtime"
    (runtime / "python").mkdir(parents=True)
    g.atomic_json(runtime / "manifest.json", {"schema": 1, "sites": [], "files": {}})
    g.atomic_json(self.root / "control/config.json", {"probation_seconds": 0, "progress_timeout": 2, "trial_timeout": 10})
    self.make_release("good", HEALTHY)
    state = g.initial_state()
    state["confirmed"] = "good"
    g.save_state(self.root, state)
    # Fixture workers use this interpreter; production always uses the captured one.
    command = g.worker_command

    def fixture_command(*args, **kwargs):
      cmd = command(*args, **kwargs)
      cmd[0] = sys.executable
      return cmd

    self.monkeypatch = pytest.MonkeyPatch()
    self.monkeypatch.setattr(g, "worker_command", fixture_command)
    self.guardian = g.Guardian(self.root, self.run_dir, self.params)
    self.wait_for(lambda: self.guardian.status.get("tunnel_status") == "connected")

  def teardown_method(self):
    self.guardian.close()
    self.monkeypatch.undo()
    self.temp.cleanup()

  def make_release(self, name, source):
    directory = self.root / "releases" / name
    checkout = directory / "checkout"
    module = checkout / "openpilot/tools/teletyped/teletyped.py"
    module.parent.mkdir(parents=True)
    module.write_text("GUARDIAN_PROTOCOL_VERSION = 1\ndef guardian_preflight():\n  pass\n" + source)
    (module.parent / "health.py").write_bytes((SOURCE.parents[1] / "tools/teletyped/health.py").read_bytes())
    g.atomic_json(directory / "manifest.json", {"schema": 1, "id": name, "runtime": "test-runtime",
                  "runtime_manifest_sha256": g.digest(self.root / "runtimes/test-runtime/manifest.json"),
                  "files": deploy.tree_hashes(checkout)})
    g.atomic_json(self.root / "control/preflight" / (name + ".json"), {"manifest_sha256": g.digest(directory / "manifest.json")})

  def wait_for(self, predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.guardian.tick()
      if predicate():
        return
      time.sleep(.02)
    pytest.fail(f"condition timed out: {self.guardian.command({'op': 'status'})}")

  def trial(self, name, **options):
    self.guardian.command({"op": "trial", "release": name, **options})

  def test_syntax_import_and_startup_failures_roll_back_and_quarantine(self):
    for name, source in (("syntax", "def broken(:\n"), ("import", "import missing_candidate_dependency\n"),
                         ("startup", "def main():\n  raise RuntimeError('startup exploded')\n")):
      self.make_release(name, source)
      self.trial(name)
      self.wait_for(lambda name=name: name in self.guardian.state["quarantined"])
      self.wait_for(lambda: self.guardian.release == "good" and bool(self.guardian.status))
      assert self.guardian.state["confirmed"] == "good"
      with pytest.raises(ValueError, match="quarantined"):
        self.trial(name)
    assert self.guardian.proc is not None

  def test_crash_midrun_restores_lkg(self):
    self.make_release("candidate", HEALTHY)
    self.trial("candidate")
    self.wait_for(lambda: self.guardian.release == "candidate" and bool(self.guardian.status))
    os.kill(self.guardian.proc.pid, signal.SIGKILL)
    self.wait_for(lambda: "candidate" in self.guardian.state["quarantined"])
    self.wait_for(lambda: self.guardian.release == "good" and bool(self.guardian.status))

  def test_heartbeat_thread_cannot_hide_stuck_main_loop(self):
    source = """import threading, time
from openpilot.tools.teletyped import health
def main():
  health.progress()
  def heartbeat():
    while True:
      health.success('heartbeat')
      time.sleep(.02)
  threading.Thread(target=heartbeat, daemon=True).start()
  time.sleep(60)
"""
    self.make_release("hung", source)
    self.trial("hung")
    self.wait_for(lambda: self.guardian.release == "hung" and "heartbeat_ok" in self.guardian.status)
    self.wait_for(lambda: "hung" in self.guardian.state["quarantined"])
    assert "progress timeout" in self.guardian.state["quarantined"]["hung"]

  def test_deadline_catches_alive_but_disconnected_candidate(self):
    self.make_release("offline", "import time\nfrom openpilot.tools.teletyped import health\ndef main():\n  while True:\n    health.progress('offline')\n    time.sleep(.02)\n")
    self.trial("offline")
    self.wait_for(lambda: self.guardian.release == "offline" and bool(self.guardian.status))
    self.guardian.started_at -= 20
    self.guardian.tick()
    assert "deadline" in self.guardian.state["quarantined"]["offline"]

  def test_interrupted_trial_boots_confirmed_release(self):
    self.make_release("candidate", HEALTHY)
    self.trial("candidate")
    self.wait_for(lambda: self.guardian.release == "candidate")
    self.guardian.close()
    self.guardian = g.Guardian(self.root, self.run_dir, self.params)
    assert self.guardian.state["trial"] is None
    assert self.guardian.state["quarantined"]["candidate"] == "interrupted trial"
    self.wait_for(lambda: self.guardian.release == "good" and bool(self.guardian.status))

  def test_confirmation_requires_probation_and_fresh_ssh_attestation(self):
    self.make_release("candidate", HEALTHY)
    self.trial("candidate")
    with pytest.raises(ValueError, match="probation"):
      self.guardian.command({"op": "confirm", "release": "candidate", "ssh_verified": True})
    self.wait_for(self.guardian.promotion_ready)
    with pytest.raises(ValueError, match="SSH"):
      self.guardian.command({"op": "confirm", "release": "candidate"})
    with socket.socket() as listener:
      listener.bind(("127.0.0.1", 0))
      listener.listen()
      # An already queued connection gets its banner when the probe connects.
      def probe(*_args, **_kwargs):
        class Banner:
          def __enter__(self):
            return self
          def __exit__(self, *_):
            return False
          def settimeout(self, _):
            pass
          def recv(self, _):
            return b"SSH-2.0-fixture\r\n"
        return Banner()
      with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setattr(g.socket, "create_connection", probe)
        self.guardian.command({"op": "confirm", "release": "candidate", "ssh_verified": True})
    assert self.guardian.state["confirmed"] == "candidate"
    assert self.guardian.state["previous"] == "good"
    assert g.load_state(self.root)["trial"] is None
    self.guardian.command({"op": "rollback", "release": "good"})
    assert g.load_state(self.root)["confirmed"] == "good"

  def test_access_revocation_stops_worker_and_its_children(self):
    source = HEALTHY.replace("def main():", "def main():\n  import subprocess, sys, os\n  subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])")
    self.make_release("children", source)
    self.trial("children")
    self.wait_for(lambda: self.guardian.release == "children" and bool(self.guardian.status))
    pid = self.guardian.proc.pid
    (self.params / "GoranConnectEnabled").write_text("0")
    self.guardian.tick()
    assert self.guardian.proc is None
    assert "children" in self.guardian.state["quarantined"]
    # Zombies cannot hold ports or execute code; no live group member may remain.
    for process in Path("/proc").iterdir():
      if process.name.isdigit():
        try:
          stat = (process / "stat").read_text().rsplit(")", 1)[1].split()
          assert not (int(stat[2]) == pid and stat[0] != "Z")
        except (OSError, ProcessLookupError):
          pass

  def test_exclusive_guardian_lock(self):
    with pytest.raises(BlockingIOError):
      g.Guardian(self.root, self.run_dir, self.params)

  def test_tampering_is_rejected_before_lkg_stops(self):
    self.make_release("tampered", HEALTHY)
    pid = self.guardian.proc.pid
    (self.root / "releases/tampered/checkout/openpilot/tools/teletyped/teletyped.py").write_text("raise Exception()")
    with pytest.raises(ValueError, match="integrity"):
      self.trial("tampered")
    assert self.guardian.proc.pid == pid

  def test_no_trial_without_completed_preflight(self):
    self.make_release("candidate", HEALTHY)
    (self.root / "control/preflight/candidate.json").unlink()
    with pytest.raises(FileNotFoundError):
      self.trial("candidate")
    assert self.guardian.release == "good"

  def test_lkg_failures_back_off_without_auto_selecting_candidate(self):
    os.kill(self.guardian.proc.pid, signal.SIGKILL)
    self.guardian.proc.wait()
    self.guardian.tick()
    assert self.guardian.proc is None
    assert self.guardian.next_start > time.monotonic()
    assert self.guardian.state["confirmed"] == "good"

  def test_orphan_cleanup_finds_children_after_group_leader_exits(self):
    self.guardian.close()
    token = "orphan-fixture-token"
    parent = subprocess.Popen([sys.executable, "-c", "import subprocess,sys; p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); print(p.pid,flush=True)"],
                              env=dict(os.environ, TELETYPED_RUN_ID=token), start_new_session=True, stdout=subprocess.PIPE, text=True)
    child = int(parent.stdout.readline())
    parent.wait(timeout=5)
    parent.stdout.close()
    g.atomic_json(self.run_dir / "active.json", {"pid": parent.pid, "run_id": token})
    try:
      self.guardian = g.Guardian(self.root, self.run_dir, self.params)
      deadline = time.monotonic() + 3
      while time.monotonic() < deadline:
        try:
          if Path(f"/proc/{child}/stat").read_text().rsplit(")", 1)[1].split()[0] == "Z":
            break
        except FileNotFoundError:
          break
        time.sleep(.02)
      else:
        pytest.fail("orphan survived guardian restart")
    finally:
      try:
        os.killpg(parent.pid, signal.SIGKILL)
      except ProcessLookupError:
        pass

  def test_disk_full_during_quarantine_still_restores_committed_lkg(self):
    self.make_release("candidate", HEALTHY)
    self.trial("candidate")
    self.wait_for(lambda: self.guardian.release == "candidate")
    def disk_full(*_):
      raise OSError("disk full")

    with pytest.MonkeyPatch.context() as monkeypatch:
      monkeypatch.setattr(g, "save_state", disk_full)
      self.guardian.fail("candidate failed")
    self.wait_for(lambda: self.guardian.release == "good" and bool(self.guardian.status))
    assert self.guardian.state["trial"] is None

  def test_malformed_health_does_not_crash_guardian(self):
    self.make_release("candidate", HEALTHY)
    self.trial("candidate")
    self.wait_for(lambda: self.guardian.release == "candidate" and bool(self.guardian.status))
    # Freeze the worker to avoid racing its normal atomic health updates.
    os.kill(self.guardian.proc.pid, signal.SIGSTOP)
    self.guardian.status_path.write_text("[]")
    self.guardian.tick()
    assert "invalid health" in self.guardian.state["quarantined"]["candidate"]
    self.wait_for(lambda: self.guardian.release == "good" and bool(self.guardian.status))

  def test_malformed_control_request_is_rejected(self):
    with pytest.raises(ValueError, match="JSON object"):
      self.guardian.command([])


class TestDurability:
  def test_failed_commit_keeps_previous_confirmed_state(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      state = dict(g.initial_state(), confirmed="good")
      g.save_state(root, state)
      state["confirmed"] = "candidate"
      replace = os.replace

      def interrupt(source, destination):
        if Path(destination).name == "deployment.json":
          raise OSError("simulated power loss before commit")
        return replace(source, destination)

      with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setattr(g.os, "replace", interrupt)
        with pytest.raises(OSError):
          g.save_state(root, state)
      assert g.load_state(root)["confirmed"] == "good"
      assert not list(root.rglob("*.tmp"))

  def test_corrupt_manifest_uses_confirmed_backup_and_discards_trial(self):
    with tempfile.TemporaryDirectory() as temp:
      root = Path(temp)
      state = dict(g.initial_state(), confirmed="good")
      g.save_state(root, state)
      state["trial"] = "candidate"
      g.save_state(root, state)
      (root / "control/deployment.json").write_text("{")
      loaded = g.load_state(root)
      assert loaded["confirmed"] == "good"
      assert loaded["trial"] is None

  def test_release_id_rejects_paths(self):
    for value in ("../candidate", "/tmp/code", "", "..", "candidate/worker"):
      with pytest.raises(ValueError):
        g.valid_id(value)

  def test_guardian_has_no_openpilot_imports(self):
    result = subprocess.run([sys.executable, "-I", "-S", "-B", str(SOURCE / "guardian.py"), "--help"],
                            cwd="/", env={"PATH": "/usr/bin:/bin"}, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr


class TestPackaging:
  def test_real_frozen_interpreter_and_checkout_survive_source_removal(self):
    with tempfile.TemporaryDirectory() as temp:
      root, source = Path(temp) / "deployment", Path(temp) / "source"
      source.mkdir()
      for args in (("init", "-q"), ("config", "user.email", "test@example.invalid"), ("config", "user.name", "Test")):
        subprocess.run(["git", *args], cwd=source, check=True)
      module = source / "openpilot/tools/teletyped/teletyped.py"
      module.parent.mkdir(parents=True)
      module.write_text("GUARDIAN_PROTOCOL_VERSION = 1\nimport frozen_dependency\nassert frozen_dependency.VALUE == 42\ndef guardian_preflight():\n  pass\ndef main():\n  raise RuntimeError('only preflight should run')\n")
      subprocess.run(["git", "add", "."], cwd=source, check=True)
      subprocess.run(["git", "commit", "-qm", "fixture"], cwd=source, check=True)
      site = Path(temp) / "live-site-packages"
      site.mkdir()
      (site / "frozen_dependency.py").write_text("VALUE = 42\n")
      (site / "dangerous.pth").write_text("import missing_live_dependency\n")
      info = {"executable": str(Path(sys.executable).resolve()), "version": list(sys.version_info[:2]),
              "stdlib": sysconfig.get_path("stdlib"), "libdir": sysconfig.get_config_var("LIBDIR"),
              "libpython": sysconfig.get_config_var("LDLIBRARY"), "sites": [str(site)]}
      original = subprocess.check_output

      def probe(command, **kwargs):
        if "'executable': os.path.realpath" in command[-1]:
          return json.dumps(info)
        return original(command, **kwargs)

      with pytest.MonkeyPatch.context() as monkeypatch:
        monkeypatch.setattr(deploy.subprocess, "check_output", probe)
        deploy.capture_runtime(root, Path(sys.executable), "runtime")
      known_hosts = Path(temp) / "known_hosts"
      known_hosts.write_text("fixture-server fixture-key\n")
      deploy.install(root, "runtime", known_hosts, None)
      deploy.stage(root, source, "runtime", "good")
      source.rename(Path(temp) / "unavailable-development-checkout")
      site.rename(Path(temp) / "unavailable-live-packages")
      command = g.worker_command(root, "good", preflight=True)
      result = subprocess.run(command, env=g.clean_env(root, "good"), cwd="/", capture_output=True, text=True, timeout=30)
      assert result.returncode == 0, result.stderr
      assert '"preflight": "ok"' in result.stdout
      control = subprocess.run([str(root / "guardian/control"), "--help"], env={"PATH": "/usr/bin:/bin"},
                               cwd="/", capture_output=True, text=True, timeout=30)
      assert control.returncode == 0, control.stderr
      g.release_info(root, "good")  # Execution must not add bytecode or mutate either tree.
      # Frozen files are deliberately read-only; make the fixture removable by non-root test users.
      for directory, _, files in os.walk(root):
        os.chmod(directory, 0o755)
        for name in files:
          os.chmod(Path(directory) / name, 0o644)
