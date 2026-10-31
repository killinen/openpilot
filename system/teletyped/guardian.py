#!/usr/bin/env python3
"""Installed independently of openpilot. Keep this module standard-library-only."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pwd
import re
import signal
import shutil
import socket
import subprocess
import time
import uuid

DEFAULT_ROOT = Path("/data/teletyped")
RELEASE_ID = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,95}\Z")


def atomic_json(path, value):
  """One writer, same-filesystem rename, and durable file AND directory entries."""
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
  try:
    with open(tmp, "x", encoding="utf-8") as f:
      os.chmod(tmp, 0o600)
      json.dump(value, f, sort_keys=True)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
    fsync_dir(path.parent)
  finally:
    tmp.unlink(missing_ok=True)


def fsync_dir(path):
  fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def read_json(path):
  with open(path, encoding="utf-8") as f:
    return json.load(f)


def valid_id(value):
  if not isinstance(value, str) or not RELEASE_ID.fullmatch(value):
    raise ValueError("invalid release/runtime ID")
  return value


def initial_state():
  return {"schema": 1, "generation": 0, "confirmed": None, "previous": None, "trial": None, "quarantined": {}}


def validate_state(state):
  if not isinstance(state, dict) or state.get("schema") != 1 or not isinstance(state.get("generation"), int):
    raise ValueError("invalid deployment state")
  for key in ("confirmed", "previous", "trial"):
    if state.get(key) is not None:
      valid_id(state[key])
  if not isinstance(state.get("quarantined"), dict):
    raise ValueError("invalid quarantine state")
  return state


def load_state(root):
  path = root / "control/deployment.json"
  if not path.exists():
    if (root / "control/confirmed-backup.json").exists():
      state = validate_state(read_json(root / "control/confirmed-backup.json"))
      state["trial"] = None
      return state
    return initial_state()
  try:
    return validate_state(read_json(path))
  except (ValueError, TypeError, OSError):
    state = validate_state(read_json(root / "control/confirmed-backup.json"))
    state["trial"] = None
    return state


def save_state(root, state):
  validate_state(state)
  state["generation"] += 1
  # The backup deliberately never authorizes a pending candidate.
  backup = dict(state, trial=None)
  old = root / "control/deployment.json"
  if old.exists():
    try:
      backup = dict(validate_state(read_json(old)), trial=None)
    except (ValueError, TypeError, OSError):
      pass
  atomic_json(root / "control/confirmed-backup.json", backup)
  atomic_json(old, state)


def digest(path):
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      h.update(chunk)
  return h.hexdigest()


def verify_tree(directory, files):
  directory = directory.resolve()
  for name, expected in files.items():
    path = directory / name
    if Path(name).is_absolute() or not path.resolve().is_relative_to(directory):
      raise ValueError("release path escapes its directory")
    actual = "link:" + os.readlink(path) if path.is_symlink() else digest(path)
    if actual != expected:
      raise ValueError(f"release integrity check failed: {name}")
  # Extra executable/importable files must not silently become part of an LKG.
  actual_files = {str(p.relative_to(directory)) for p in directory.rglob("*") if p.is_file() or p.is_symlink()}
  if actual_files != set(files):
    raise ValueError("release contains unrecorded/missing files")


def release_info(root, release, verify=True):
  directory = root / "releases" / valid_id(release)
  manifest = read_json(directory / "manifest.json")
  if not isinstance(manifest, dict) or manifest.get("schema") != 1 or manifest.get("id") != release:
    raise ValueError("invalid release manifest")
  runtime = root / "runtimes" / valid_id(manifest["runtime"])
  runtime_manifest = read_json(runtime / "manifest.json")
  if manifest["runtime_manifest_sha256"] != digest(runtime / "manifest.json"):
    raise ValueError("runtime manifest changed")
  if verify:
    verify_tree(directory / "checkout", manifest["files"])
    verify_tree(runtime / "python", runtime_manifest["files"])
  return directory, runtime, runtime_manifest


def worker_command(root, release, preflight=False, verify=True):
  directory, runtime, manifest = release_info(root, release, verify)
  cmd = [str(runtime / "python/bin/python3"), "-I", "-S", "-B", "-u", str(root / "guardian/bootstrap.py"),
         str(directory / "checkout"), str(runtime / "python"), *manifest["sites"]]
  if preflight:
    cmd.append("--preflight")
  return cmd


def clean_env(root, release):
  _, runtime, _ = release_info(root, release, verify=False)
  # Do not inherit PYTHONPATH, Python startup hooks, LD_PRELOAD, or arbitrary application variables.
  return {"PATH": str(runtime / "python/bin") + ":/usr/bin:/bin:/usr/sbin:/sbin", "LANG": "C.UTF-8", "PYTHONDONTWRITEBYTECODE": "1",
          "LD_LIBRARY_PATH": str(runtime / "python/lib"), "TELETYPED_DEPLOYMENT_ROOT": str(root),
          "TELETYPED_KNOWN_HOSTS": str(root / "identity/known_hosts")}


def worker_identity(root):
  account = read_json(root / "control/config.json").get("worker_user")
  if account:
    user = pwd.getpwnam(account)
    return {"user": user.pw_uid, "group": user.pw_gid, "extra_groups": os.getgrouplist(account, user.pw_gid)}, user.pw_dir
  return {}, str(Path.home())


def event(message, **details):
  print(json.dumps({"time": time.time(), "event": message, **details}), flush=True)


class Guardian:
  def __init__(self, root, run_dir, params_dir=Path("/data/params/d")):
    self.root, self.run_dir, self.params_dir = Path(root), Path(run_dir), Path(params_dir)
    self.root.mkdir(parents=True, exist_ok=True)
    self.run_dir.mkdir(parents=True, exist_ok=True, mode=0o711)
    self.lock = open(self.root / "guardian.lock", "a")
    try:
      fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
      self.lock.close()
      raise
    self.config = read_json(self.root / "control/config.json")
    self.state = load_state(self.root)
    self.proc = None
    self.release = None
    self.status = {}
    self.started_at = 0.0
    self.next_start = 0.0
    self.failures = 0
    self.verified = set()
    self.healthy_since = None
    self.running = True
    self.last_reason = None
    self._cleanup_orphan()
    # An interrupted trial is never resumed, including after guardian-only restart.
    if self.state["trial"]:
      self.state["quarantined"][self.state["trial"]] = "interrupted trial"
      self.state["trial"] = None
      self.persist_recovery()
    self.socket_path = self.run_dir / "control.sock"
    self.socket_path.unlink(missing_ok=True)
    self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    self.server.bind(str(self.socket_path))
    os.chmod(self.socket_path, 0o600)
    self.server.listen(4)
    self.server.setblocking(False)

  def persist_recovery(self):
    try:
      save_state(self.root, self.state)
    except OSError as e:
      # A full /data must not prevent returning to an already committed LKG.
      # On reboot any still-pending trial is abandoned again, never resumed.
      event("could not persist quarantine; continuing with confirmed LKG", error=str(e))

  def _cleanup_orphan(self):
    try:
      active = read_json(self.run_dir / "active.json")
      token = f"TELETYPED_RUN_ID={active['run_id']}".encode()
      # The group leader may already be dead while ssh/wormhole remains alive.
      for process in Path("/proc").iterdir():
        if process.name.isdigit():
          try:
            if token in (process / "environ").read_bytes().split(b"\0") and os.getpgid(int(process.name)) == active["pid"]:
              os.killpg(active["pid"], signal.SIGKILL)
              break
          except (OSError, ProcessLookupError):
            pass
    except (OSError, ValueError, KeyError):
      pass
    (self.run_dir / "active.json").unlink(missing_ok=True)

  def enabled(self):
    try:
      return (self.params_dir / "GoranConnectEnabled").read_bytes().strip() == b"1"
    except OSError:
      return False

  def start_worker(self, release):
    cmd = worker_command(self.root, release, verify=release not in self.verified)
    self.verified.add(release)
    token = uuid.uuid4().hex
    status_dir = self.run_dir / token
    status_dir.mkdir(mode=0o700)
    credentials, home = worker_identity(self.root)
    env = clean_env(self.root, release)
    if credentials:
      os.chown(status_dir, credentials["user"], credentials["group"])
    env["HOME"] = home
    states = self.root / "worker-state"
    states.mkdir(exist_ok=True, mode=0o755)
    os.chmod(states, 0o755)
    state_dir = states / release
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    if credentials:
      os.chown(state_dir, credentials["user"], credentials["group"])
    self.status_path = status_dir / "health.json"
    env.update(TELETYPED_RUN_ID=token, TELETYPED_RELEASE_ID=release, TELETYPED_HEALTH_PATH=str(self.status_path),
               TELETYPED_STATE_DIR=str(state_dir), TELETYPED_TUNNEL_DIR=str(status_dir),
               TELETYPED_LAUNCH_READY=str(status_dir / "launch-ready"), TELETYPED_PARAMS_DIR=str(self.params_dir))
    self.proc = subprocess.Popen(cmd, cwd="/", env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, start_new_session=True, **credentials)
    self.release, self.run_id = release, token
    self.started_at = time.monotonic()
    self.healthy_since = None
    self.status = {}
    os.set_blocking(self.proc.stdout.fileno(), False)
    # Worker waits for this durable identity record before importing candidate code or spawning ssh.
    atomic_json(self.run_dir / "active.json", {"pid": self.proc.pid, "run_id": token})
    (status_dir / "launch-ready").touch()
    event("worker started", release=release, pid=self.proc.pid)

  def drain_logs(self):
    if self.proc is None:
      return
    try:
      data = os.read(self.proc.stdout.fileno(), 65536)
    except BlockingIOError:
      return
    if data:
      try:
        directory = self.root / "logs"
        directory.mkdir(exist_ok=True)
        path = directory / "worker.log"
        if path.exists() and path.stat().st_size + len(data) > 1024 * 1024:
          os.replace(path, directory / "worker.previous.log")
        with open(path, "ab") as f:
          f.write(data)
      except OSError:
        pass  # Discard diagnostics rather than losing recovery on a full /data.

  def stop_worker(self):
    if self.proc is None:
      return
    proc = self.proc
    # One active worker owns its entire process group, including ssh and transfers.
    try:
      os.killpg(proc.pid, signal.SIGTERM)
      deadline = time.monotonic() + 5
      while proc.poll() is None and time.monotonic() < deadline:
        self.drain_logs()
        time.sleep(0.05)
      try:
        os.killpg(proc.pid, signal.SIGKILL)
      except ProcessLookupError:
        pass
      proc.wait(timeout=2)
    except ProcessLookupError:
      proc.wait(timeout=2)
    finally:
      self.drain_logs()
      proc.stdout.close()
      self.proc, self.release = None, None
      self.status = {}
      (self.run_dir / "active.json").unlink(missing_ok=True)
      shutil.rmtree(self.status_path.parent, ignore_errors=True)

  def fail(self, reason):
    event("worker failed", release=self.release, reason=reason)
    self.last_reason = reason
    trial = self.state["trial"]
    if trial:
      # Roll back on the first failed trial; explicit retry is required.
      self.state["quarantined"][trial] = reason
      self.state["trial"] = None
      self.persist_recovery()
      self.next_start = time.monotonic() + 1
    else:
      self.failures += 1
      self.next_start = time.monotonic() + min(300, 2 ** min(self.failures, 8))
    self.stop_worker()

  def promotion_ready(self):
    now = time.monotonic()
    health = self.status
    return (self.proc is not None and self.proc.poll() is None and self.recent_health(now)
            and self.healthy_since is not None and now - self.healthy_since >= self.config.get("probation_seconds", 300)
            and health.get("tunnel_status") == "connected")

  def recent_health(self, now):
    return all(isinstance(self.status.get(key), (int, float)) and 0 <= now - self.status[key] <= 90
               for key in ("heartbeat_ok", "request_ok", "main_progress"))

  def tick(self):
    self.drain_logs()
    if not self.enabled():
      if self.state["trial"]:
        self.fail("access disabled during trial")
      self.stop_worker()
      return
    if self.proc is None:
      if time.monotonic() >= self.next_start:
        release = self.state["trial"] or self.state["confirmed"]
        if release:
          try:
            self.start_worker(release)
          except (OSError, ValueError, KeyError, subprocess.SubprocessError) as e:
            self.fail(f"launch failed: {e}")
      return
    if self.proc.poll() is not None:
      self.fail(f"worker exited: {self.proc.returncode}")
      return
    now = time.monotonic()
    try:
      with open(self.status_path) as f:
        raw = f.read(8193)
      if len(raw) > 8192:
        raise ValueError("health report too large")
      health = json.loads(raw)
      if not isinstance(health, dict) or health.get("run_id") != self.run_id or health.get("release_id") != self.release:
        raise ValueError("health report identity/schema mismatch")
      self.status = health
    except OSError:
      pass
    except ValueError as e:
      self.fail(f"invalid health report: {e}")
      return
    progress = self.status.get("main_progress", self.started_at)
    if not isinstance(progress, (int, float)) or not 0 <= now - progress <= self.config.get("progress_timeout", 120):
      self.fail("main loop progress timeout")
      return
    healthy = self.recent_health(now) and self.status.get("tunnel_status") == "connected"
    if healthy:
      if self.healthy_since is None:
        self.healthy_since = now
      if self.state["trial"] is None and now - self.healthy_since >= 300:
        self.failures = 0
    else:
      self.healthy_since = None
    if self.state["trial"] and now - self.started_at > self.config.get("trial_timeout", 900):
      self.fail("trial connectivity/confirmation deadline exceeded")

  def command(self, request):
    if not isinstance(request, dict):
      raise ValueError("request must be a JSON object")
    operation = request.get("op")
    if operation == "status":
      return {"state": self.state, "release": self.release, "pid": self.proc.pid if self.proc else None,
              "health": self.status, "promotion_ready": self.promotion_ready(), "last_failure": self.last_reason}
    release = valid_id(request.get("release"))
    if operation == "trial":
      if not self.enabled():
        raise ValueError("enable GoranConnect before requesting a trial")
      if not self.state["confirmed"]:
        raise ValueError("seed a verified LKG first")
      if self.state["trial"] or release == self.state["confirmed"]:
        raise ValueError("trial already active, or release already confirmed")
      if release in self.state["quarantined"] and not request.get("retry"):
        raise ValueError("candidate quarantined; explicit --retry required")
      worker_command(self.root, release)  # Verify before disrupting LKG.
      directory = self.root / "releases" / release
      record = read_json(self.root / "control/preflight" / (release + ".json"))
      if record["manifest_sha256"] != digest(directory / "manifest.json"):
        raise ValueError("candidate did not pass frozen-runtime preflight")
      self.verified.add(release)
      self.state["trial"] = release
      self.state["quarantined"].pop(release, None)
      save_state(self.root, self.state)  # Persist intent BEFORE disturbing LKG.
      self.stop_worker()
      self.next_start = 0
    elif operation == "confirm":
      if release != self.state["trial"] or release != self.release or not self.promotion_ready():
        raise ValueError("candidate has not completed healthy online probation")
      if request.get("ssh_verified") is not True:
        raise ValueError("confirmation requires a fresh SSH session and --ssh-verified")
      port = self.status.get("local_ssh_port")
      if not isinstance(port, int) or not 0 < port < 65536:
        raise ValueError("missing local SSH port")
      with socket.create_connection(("127.0.0.1", port), timeout=2) as connection:
        connection.settimeout(2)
        if not connection.recv(255).startswith(b"SSH-"):
          raise ValueError("local SSH banner probe failed")
      self.state["previous"] = self.state["confirmed"]
      self.state["confirmed"], self.state["trial"] = release, None
      save_state(self.root, self.state)
    elif operation == "rollback":
      if release not in (self.state["confirmed"], self.state["previous"]):
        raise ValueError("rollback target must be a current or previous confirmed LKG")
      if release == self.state["previous"]:
        worker_command(self.root, release)
        self.verified.add(release)
        if self.state["trial"]:
          self.state["quarantined"][self.state["trial"]] = "operator rollback"
        self.state["previous"], self.state["confirmed"] = self.state["confirmed"], release
        self.state["trial"] = None
        save_state(self.root, self.state)
        self.stop_worker()
        self.next_start = 0
      elif self.state["trial"]:
        self.fail("operator rollback")
      else:
        self.stop_worker()
        self.next_start = 0
    else:
      raise ValueError("unknown operation")
    return {"ok": True, "state": self.state}

  def serve_once(self):
    try:
      client, _ = self.server.accept()
    except BlockingIOError:
      return
    with client:
      client.settimeout(0.2)
      try:
        data = b""
        while b"\n" not in data and len(data) <= 8192:
          chunk = client.recv(8192)
          if not chunk:
            break
          data += chunk
        if len(data) > 8192:
          raise ValueError("request too large")
        result = self.command(json.loads(data))
      except (OSError, ValueError, KeyError, TypeError) as e:
        result = {"error": str(e)}
      try:
        client.sendall(json.dumps(result).encode() + b"\n")
      except OSError:
        pass

  def run(self):
    for sig in (signal.SIGINT, signal.SIGTERM):
      signal.signal(sig, lambda *_: setattr(self, "running", False))
    try:
      while self.running:
        self.tick()
        self.serve_once()
        time.sleep(0.25)
    finally:
      self.close()

  def close(self):
    self.stop_worker()
    self.server.close()
    self.socket_path.unlink(missing_ok=True)
    self.lock.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
  parser.add_argument("--run-dir", type=Path, default=Path("/run/teletyped"))
  parser.add_argument("--params-dir", type=Path, default=Path("/data/params/d"))
  args = parser.parse_args()
  Guardian(args.root.resolve(), args.run_dir.resolve(), args.params_dir).run()


if __name__ == "__main__":
  main()
