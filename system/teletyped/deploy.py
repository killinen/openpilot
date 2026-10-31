#!/usr/bin/env python3
"""Explicit on-device installation, immutable staging, and guardian control."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import shlex
import socket
import subprocess
import sys
import time
import uuid

# Also works as an installed script under python -I -S. This is the protected
# guardian directory, never a candidate or the caller's working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import guardian as g


def sync_tree(path):
  for item in path.rglob("*"):
    if item.is_file() and not item.is_symlink():
      with open(item, "rb") as f:
        os.fsync(f.fileno())
  for directory, _, _ in os.walk(path, topdown=False, followlinks=False):
    g.fsync_dir(directory)


def tree_hashes(path):
  result = {}
  for item in sorted(path.rglob("*")):
    if item.is_symlink():
      if not item.resolve().is_relative_to(path.resolve()):
        raise ValueError(f"external symlink in snapshot: {item}")
      result[str(item.relative_to(path))] = "link:" + os.readlink(item)
    elif item.is_file():
      result[str(item.relative_to(path))] = g.digest(item)
  return result


def freeze(path):
  for directory, dirs, files in os.walk(path, topdown=False, followlinks=False):
    for name in files:
      item = Path(directory) / name
      if not item.is_symlink():
        os.chmod(item, 0o555 if item.stat().st_mode & 0o111 else 0o444)
    for name in dirs:
      item = Path(directory) / name
      if not item.is_symlink():
        os.chmod(item, 0o555)
  os.chmod(path, 0o555)


def publish(root, kind, identifier, staging):
  destination = root / kind / g.valid_id(identifier)
  destination.parent.mkdir(parents=True, exist_ok=True)
  os.chmod(destination.parent, 0o755)
  if destination.exists():
    raise ValueError(f"immutable {kind} ID already exists: {identifier}")
  freeze(staging)
  sync_tree(staging)
  os.rename(staging, destination)
  g.fsync_dir(destination.parent)
  g.fsync_dir(staging.parent)
  return destination


def capture_runtime(root, python, identifier):
  """Freeze Python, stdlib, installed packages, libpython and SSH binaries.

  The OS loader, libc, OpenSSL and other OS libraries remain platform dependencies.
  No .pth hooks or editable installs are activated by bootstrap.
  """
  probe = """import json, os, sys, sysconfig
print(json.dumps({'executable': os.path.realpath(sys.executable), 'version': list(sys.version_info[:2]),
 'stdlib': sysconfig.get_path('stdlib'), 'libdir': sysconfig.get_config_var('LIBDIR'),
 'libpython': sysconfig.get_config_var('LDLIBRARY'),
 'sites': [p for p in sys.path if os.path.isdir(p) and ('site-packages' in p or 'dist-packages' in p)]}))
"""
  info = json.loads(subprocess.check_output([str(python), "-I", "-c", probe], text=True, timeout=30))
  if info["version"] < [3, 11]:
    raise ValueError("Python 3.11 or newer is required")
  staging = root / "staging" / uuid.uuid4().hex
  target = staging / "python"
  (target / "bin").mkdir(parents=True)
  shutil.copy2(info["executable"], target / "bin/python3")
  stdlib = target / "lib" / ("python" + ".".join(map(str, info["version"])))
  shutil.copytree(info["stdlib"], stdlib, symlinks=False,
                  ignore=shutil.ignore_patterns("site-packages", "dist-packages", "__pycache__", "*.pyc"))
  sites = []
  for index, site in enumerate(dict.fromkeys(info["sites"])):
    relative = f"packages/{index}"
    shutil.copytree(site, target / relative, symlinks=False, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    sites.append(relative)
  if info["libdir"] and info["libpython"]:
    library = Path(info["libdir"]) / info["libpython"]
    if library.exists():
      shutil.copy2(library, target / "lib" / library.name)
      if ".so" in library.name:
        shutil.copy2(library, target / "lib" / (library.name.split(".so")[0] + ".so.1.0"))
  for binary in ("ssh", "ssh-keygen"):
    location = shutil.which(binary)
    if location is None:
      raise ValueError(f"required OS executable missing: {binary}")
    shutil.copy2(location, target / "bin" / binary)
  final = root / "runtimes" / g.valid_id(identifier) / "python"
  relocate_native(target, Path(info["stdlib"]), final / stdlib.relative_to(target))
  for site, relative in zip(dict.fromkeys(info["sites"]), sites, strict=True):
    relocate_native(target, Path(site), final / relative)
  env = {"PATH": "/usr/bin:/bin", "LD_LIBRARY_PATH": str(target / "lib")}
  check = "import json, pathlib, ssl, sys; print(json.dumps({'prefix': sys.prefix, 'paths': sys.path}))"
  result = json.loads(subprocess.check_output([str(target / "bin/python3"), "-I", "-S", "-B", "-c", check], env=env, text=True, timeout=30))
  if Path(result["prefix"]).resolve() != target.resolve():
    raise ValueError("captured interpreter did not relocate; refusing a runtime dependent on the live environment")
  if any(not Path(p).resolve().is_relative_to(target.resolve()) for p in result["paths"]):
    raise ValueError("captured interpreter still imports outside its frozen runtime")
  g.atomic_json(staging / "manifest.json", {"schema": 1, "id": identifier, "sites": sites, "files": tree_hashes(target)})
  return publish(root, "runtimes", identifier, staging)


def snapshot_checkout(source, destination):
  # Recursive tracked files include submodules and dirty tracked bytes. Include
  # new TeleTyped files before their first commit, but never copy .git or secrets.
  names = set(subprocess.check_output(["git", "ls-files", "-z", "--recurse-submodules"], cwd=source).decode().split("\0"))
  for subtree in ("system/teletyped", "tools/teletyped"):
    for item in (source / subtree).rglob("*"):
      if item.is_file() and "__pycache__" not in item.parts and item.suffix in {".py", ".sh", ".service"}:
        names.add(str(item.relative_to(source)))
  # Built Cython/CFFI extensions are ignored by Git but required on device.
  for directory, dirs, files in os.walk(source, followlinks=False):
    dirs[:] = [d for d in dirs if d not in {".git", ".venv", "venv", "node_modules", "__pycache__", ".cache"}]
    for name in files:
      if ".so" in name and (name.endswith(".so") or ".so." in name):
        names.add(str((Path(directory) / name).relative_to(source)))
  destination.mkdir(parents=True)
  for name in sorted(names):
    if not name:
      continue
    relative = Path(name)
    if relative.is_absolute() or ".." in relative.parts or ".git" in relative.parts:
      raise ValueError("unsafe snapshot path")
    src, dst = source / name, destination / name
    if not src.exists() and not src.is_symlink():
      continue  # A tracked deletion is part of the candidate too.
    if src.is_dir() and not src.is_symlink():
      continue
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_symlink():
      link = os.readlink(src)
      if Path(link).is_absolute() or not (dst.parent / link).resolve().is_relative_to(destination.resolve()):
        raise ValueError(f"snapshot contains an external link: {name}")
      dst.symlink_to(link)
    else:
      shutil.copy2(src, dst)
  # Runtime version helpers must not fall back to the development Git repository.
  def git(*args):
    return subprocess.check_output(["git", *args], cwd=source, text=True).strip()
  g.atomic_json(destination / "build_metadata.json", {"channel": git("rev-parse", "--abbrev-ref", "HEAD"), "openpilot": {
    "version": "teletyped-snapshot", "git_commit": git("rev-parse", "HEAD"), "git_origin": "local-snapshot",
    "git_commit_date": git("show", "-s", "--format=%cI", "HEAD"), "build_style": "teletyped", "release_notes": ""}})


def relocate_native(checkout, source, final_checkout):
  """SCons extensions can encode absolute DT_NEEDED and RPATH checkout paths."""
  readelf = shutil.which("readelf")
  if readelf is None:
    raise ValueError("readelf is required to audit frozen native dependencies")
  for item in checkout.rglob("*"):
    if not item.is_file() or item.is_symlink():
      continue
    with open(item, "rb") as f:
      if f.read(4) != b"\x7fELF":
        continue
    output = subprocess.check_output([readelf, "-d", str(item)], text=True, timeout=10)
    if str(source) not in output:
      continue
    patchelf = shutil.which("patchelf")
    if patchelf is None:
      raise ValueError(f"patchelf is required to relocate {item.relative_to(checkout)} away from {source}")
    needed = subprocess.check_output([patchelf, "--print-needed", str(item)], text=True, timeout=10).splitlines()
    for library in needed:
      if library.startswith(str(source) + "/"):
        replacement = str(final_checkout / Path(library).relative_to(source))
        subprocess.run([patchelf, "--replace-needed", library, replacement, str(item)], check=True, timeout=10)
    rpath = subprocess.check_output([patchelf, "--print-rpath", str(item)], text=True, timeout=10).strip()
    if str(source) in rpath:
      subprocess.run([patchelf, "--set-rpath", rpath.replace(str(source), str(final_checkout)), str(item)], check=True, timeout=10)


def stage(root, source, runtime, identifier):
  if root.resolve().is_relative_to(source.resolve()):
    raise ValueError("deployment storage must be outside the development checkout")
  runtime = g.valid_id(runtime)
  runtime_path = root / "runtimes" / runtime
  runtime_manifest = g.read_json(runtime_path / "manifest.json")
  g.verify_tree(runtime_path / "python", runtime_manifest["files"])
  staging = root / "staging" / uuid.uuid4().hex
  snapshot_checkout(source.resolve(), staging / "checkout")
  relocate_native(staging / "checkout", source.resolve(), root / "releases" / g.valid_id(identifier) / "checkout")
  manifest = {"schema": 1, "id": identifier, "runtime": runtime,
              "runtime_manifest_sha256": g.digest(runtime_path / "manifest.json"), "files": tree_hashes(staging / "checkout")}
  g.atomic_json(staging / "manifest.json", manifest)
  directory = publish(root, "releases", identifier, staging)
  command = g.worker_command(root, identifier, preflight=True)
  # Import all worker dependencies in a fresh frozen subprocess, never here.
  credentials, home = g.worker_identity(root)
  env = dict(g.clean_env(root, identifier), HOME=home)
  subprocess.run(command, cwd="/", env=env, check=True, timeout=120, **credentials)
  g.atomic_json(root / "control" / "preflight" / (identifier + ".json"), {"manifest_sha256": g.digest(directory / "manifest.json")})
  return directory


def install(root, runtime, known_hosts, worker_user):
  if (root / "installed.json").exists():
    raise ValueError("guardian already activated; do not replace it during normal worker development")
  runtime = g.valid_id(runtime)
  manifest = g.read_json(root / "runtimes" / runtime / "manifest.json")
  g.verify_tree(root / "runtimes" / runtime / "python", manifest["files"])
  if not known_hosts.read_text().strip():
    raise ValueError("supply a trusted nonempty known_hosts file")
  directory = root / "guardian"
  directory.mkdir(parents=True, exist_ok=True)
  os.chmod(directory, 0o755)
  for name in ("guardian.py", "deploy.py", "bootstrap.py"):
    shutil.copy2(Path(__file__).parent / name, directory / name)
    os.chmod(directory / name, 0o444)
  python_root = root / "runtimes" / runtime / "python"
  # A copied pyenv executable can retain its old libpython RUNPATH. Set the
  # frozen loader path before starting Python, including for operator commands.
  control_script = directory / "control"
  control_script.write_text("#!/bin/sh\nexec /usr/bin/env " + shlex.quote("LD_LIBRARY_PATH=" + str(python_root / "lib")) + " "
                            + shlex.quote(str(python_root / "bin/python3")) + " -I -S -B " + shlex.quote(str(directory / "deploy.py"))
                            + " --root " + shlex.quote(str(root)) + ' "$@"\n')
  os.chmod(control_script, 0o555)
  identity = root / "identity"
  identity.mkdir(exist_ok=True, mode=0o755)
  os.chmod(identity, 0o755)
  shutil.copy2(known_hosts, identity / "known_hosts")
  os.chmod(identity / "known_hosts", 0o444)
  config = {"worker_user": worker_user, "probation_seconds": 300, "progress_timeout": 120, "trial_timeout": 900,
            "guardian_runtime": runtime}
  if worker_user:
    g.pwd.getpwnam(worker_user)
  g.atomic_json(root / "control/config.json", config)
  os.chmod(root, 0o755)
  os.chmod(root / "control", 0o700)
  sync_tree(directory)
  g.fsync_dir(root)


def seed(root, release, verified):
  if not verified:
    raise ValueError("seeding requires a known working version and --ssh-verified")
  with open(root / "guardian.lock", "a") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    state = g.load_state(root)
    if state["confirmed"]:
      raise ValueError("LKG already seeded; use a supervised trial")
    directory, _, _ = g.release_info(root, release)
    record = g.read_json(root / "control/preflight" / (g.valid_id(release) + ".json"))
    if record["manifest_sha256"] != g.digest(directory / "manifest.json"):
      raise ValueError("release did not pass preflight")
    state["confirmed"] = release
    g.save_state(root, state)


def activate(root):
  if os.geteuid() != 0 or root != g.DEFAULT_ROOT:
    raise ValueError("systemd activation requires root and /data/teletyped")
  if Path("/data/params/d/GoranConnectEnabled").read_bytes().strip() != b"0":
    raise ValueError("disable GoranConnect and wait for the legacy worker to stop before activation")
  for process in Path("/proc").iterdir():
    if process.name.isdigit():
      try:
        if b"tools.teletyped.teletyped" in (process / "cmdline").read_bytes():
          raise ValueError("legacy TeleTyped is still running")
      except OSError:
        pass
  if Path("/tmp/reverse_ssh_tunnel.pid").exists():
    raise ValueError("legacy tunnel PID file still exists; stop the old tunnel before activation")
  state = g.load_state(root)
  if not state["confirmed"]:
    raise ValueError("seed a verified LKG before activation")
  config = g.read_json(root / "control/config.json")
  runtime = g.valid_id(config["guardian_runtime"])
  executable = root / "runtimes" / runtime / "python/bin/python3"
  unit = f"""[Unit]
Description=TeleTyped recovery guardian
After=local-fs.target
StartLimitIntervalSec=0

[Service]
Type=simple
ExecStart={executable} -I -S -B /data/teletyped/guardian/guardian.py
Environment=LD_LIBRARY_PATH=/data/teletyped/runtimes/{runtime}/python/lib
WorkingDirectory=/
Restart=always
RestartSec=5
KillMode=control-group
TimeoutStopSec=15
RuntimeDirectory=teletyped
RuntimeDirectoryMode=0711
UMask=0077
MemoryMax=1G
CPUWeight=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"""
  unit_path = Path("/etc/systemd/system/teletyped-guardian.service")
  temporary = unit_path.with_name(".teletyped-guardian." + uuid.uuid4().hex + ".tmp")
  try:
    with open(temporary, "x") as f:
      os.fchmod(f.fileno(), 0o644)
      f.write(unit)
      f.flush()
      os.fsync(f.fileno())
    os.replace(temporary, unit_path)
    g.fsync_dir(unit_path.parent)
  finally:
    temporary.unlink(missing_ok=True)
  subprocess.run(["systemctl", "daemon-reload"], check=True, timeout=30)
  subprocess.run(["systemctl", "enable", "--now", "teletyped-guardian.service"], check=True, timeout=30)
  deadline = time.monotonic() + 10
  while True:
    try:
      control(root, Path("/run/teletyped"), {"op": "status"})
      break
    except OSError:
      if time.monotonic() >= deadline:
        raise
      time.sleep(0.2)
  # Manager switches off its legacy fallback only after the independent service is reachable.
  g.atomic_json(root / "installed.json", {"schema": 1})


def control(root, run_dir, request):
  del root  # Address the guardian socket; do not mutate deployment files here.
  with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
    client.settimeout(120)
    client.connect(str(run_dir / "control.sock"))
    client.sendall(json.dumps(request).encode() + b"\n")
    data = b""
    while b"\n" not in data:
      chunk = client.recv(65536)
      if not chunk:
        raise RuntimeError("guardian disconnected")
      data += chunk
    result = json.loads(data)
    if "error" in result:
      raise ValueError(result["error"])
    return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--root", type=Path, default=g.DEFAULT_ROOT)
  parser.add_argument("--run-dir", type=Path, default=Path("/run/teletyped"))
  commands = parser.add_subparsers(dest="command", required=True)
  runtime = commands.add_parser("capture-runtime")
  runtime.add_argument("id", type=g.valid_id)
  runtime.add_argument("--python", type=Path, default=Path(sys.executable))
  installer = commands.add_parser("install")
  installer.add_argument("--runtime", required=True, type=g.valid_id)
  installer.add_argument("--known-hosts", type=Path, required=True)
  installer.add_argument("--worker-user", default="comma")
  staging = commands.add_parser("stage")
  staging.add_argument("id", type=g.valid_id)
  staging.add_argument("--runtime", required=True, type=g.valid_id)
  source_default = Path("/data/openpilot") if Path(__file__).parent.name == "guardian" else Path(__file__).resolve().parents[2]
  staging.add_argument("--source", type=Path, default=source_default)
  seeding = commands.add_parser("seed")
  seeding.add_argument("id", type=g.valid_id)
  seeding.add_argument("--ssh-verified", action="store_true")
  commands.add_parser("activate")
  commands.add_parser("status")
  for name in ("trial", "confirm", "rollback"):
    command = commands.add_parser(name)
    command.add_argument("id", type=g.valid_id)
    if name == "trial":
      command.add_argument("--retry", action="store_true")
    if name == "confirm":
      command.add_argument("--ssh-verified", action="store_true")
  args = parser.parse_args()
  root = args.root.resolve()
  if args.command == "capture-runtime":
    result = str(capture_runtime(root, args.python, args.id))
  elif args.command == "install":
    result = install(root, args.runtime, args.known_hosts, args.worker_user)
  elif args.command == "stage":
    result = str(stage(root, args.source, args.runtime, args.id))
  elif args.command == "seed":
    result = seed(root, args.id, args.ssh_verified)
  elif args.command == "activate":
    result = activate(root)
  else:
    request = {"op": args.command}
    if hasattr(args, "id"):
      request["release"] = args.id
    if hasattr(args, "retry"):
      request["retry"] = args.retry
    if hasattr(args, "ssh_verified"):
      request["ssh_verified"] = args.ssh_verified
    result = control(root, args.run_dir, request)
  print(json.dumps(result, indent=2))


if __name__ == "__main__":
  main()
