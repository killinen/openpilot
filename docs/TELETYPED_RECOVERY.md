# TeleTyped recovery deployment

The installed guardian runs independently of openpilot manager and its Git
checkout. It launches one immutable TeleTyped release in a fresh interpreter.
Candidate failure returns to the confirmed last-known-good release (LKG).
Promotion is manual after five continuous healthy online minutes and a fresh
SSH check. There is no shadow mode or automatic promotion in this first version.

Until installation is activated, manager uses a subprocess fallback. The
fallback avoids importing TeleTyped during manager preparation and restarts
exited workers after five seconds. It does **not** provide rollback, a hang
watchdog, or survival across manager restarts. Installing this branch alone does
not activate the independent recovery service.

## Installed layout

```text
/data/teletyped/
  guardian/                    installed guardian.py, bootstrap.py, deploy.py, control
  runtimes/<id>/               captured Python, stdlib, packages, SSH executables
  releases/<id>/               manifest.json and checkout/ (including built .so files)
  identity/known_hosts         trusted SSH server identity
  control/config.json         root-owned guardian settings
  control/deployment.json     sole active deployment decision
  control/confirmed-backup.json
  control/preflight/<id>.json  successful isolated import checks
  installed.json              manager migration marker, created after service readiness
  worker-state/<release-id>/   isolated writable feature state
  staging/                    unfinished captures; never executed
  logs/worker*.log             bounded current/previous worker logs
  guardian.lock
/run/teletyped/                control socket and private per-run health/tunnel files
/dev/shm/teletyped_manager_state.json
/etc/systemd/system/teletyped-guardian.service
```

Releases, runtime files, guardian code, deployment decisions, and server host
keys are owned by root. Workers normally run as `comma`; each gets its own
writable state and runtime directory. Control commands require root. This is
accident resistance, not a sandbox against a worker with unrestricted sudo.

Existing device registration and tunnel identity keys retain their existing
locations. Do not rotate them while testing a release. Feature state is isolated
per release so candidate migrations cannot overwrite LKG caches. A new release
may repeat inventory/statistics uploads; the backend's existing idempotency
handling is required. There is no automatic deletion of releases or runtimes.

On `FP-LS600h-097`, the existing HRR telemetry worker is retained and its retry
queue is also isolated per release. Existing manual route transfers remain
available; this recovery port does not enable automatic archive downloads.

## First installation on the device

Do this while another tested access path or physical access is available. Initial
activation requires stopping the legacy worker/tunnel and will interrupt sessions
using that tunnel. A trial also closes the old tunnel; reconnect to verify the
candidate. The guardian completes or rolls back the trial without the initiating
terminal remaining connected.

Use the **absolute path of the Python 3.11+ interpreter that currently runs this
branch** in place of `/path/to/working/python` below. Do not assume `sudo python`
selects the same interpreter. These are device commands, not host installation
commands.

Prerequisites:

- A fully built, working checkout of this branch, including initialized submodules.
  Boot this branch or restart manager before installing: an already running
  manager from the older code does not honor the migration marker. Editing its
  source files alone does not update the live process's launch policy.
- `git`, `readelf`, `ssh`, and `ssh-keygen`. `patchelf` is required if native
  extensions encode checkout paths; staging refuses to leave those references
  pointing at development code.
- Enough `/data` space for a Python environment snapshot and full source/runtime
  snapshots. Capturing packages can be large. Incomplete staging directories can
  be removed after inspecting them; never remove confirmed release/runtime IDs.
- A trusted `known_hosts` file for `goranconnect.duckdns.org`, obtained through
  an independently authenticated channel. An unchecked `ssh-keyscan` result is
  not sufficient verification.
- A writable, persistent systemd unit installation supported by the installed
  AGNOS. Activation fails rather than remounting or guessing at boot hooks.

First verify that this version of TeleTyped works, including a fresh SSH login.
The seed must include guardian protocol v1; an old TeleTyped without health
reporting is rejected by preflight.

```bash
cd /data/openpilot
sudo /path/to/working/python system/teletyped/deploy.py capture-runtime device-python-v1 --python /path/to/working/python
sudo /path/to/working/python system/teletyped/deploy.py install --runtime device-python-v1 --known-hosts /path/to/trusted_known_hosts --worker-user comma
sudo /data/teletyped/guardian/control stage initial-good --runtime device-python-v1 --source /data/openpilot
sudo /data/teletyped/guardian/control seed initial-good --ssh-verified
```

Staging copies current tracked worktree bytes, submodules, built extensions, and
new TeleTyped source files. It excludes Git metadata and does not capture
arbitrary untracked development files elsewhere. Save such dependencies in Git
before staging. Source symlinks must stay inside the snapshot. Do not edit the
checkout while staging. Hashes identify actual packaged bytes, not just HEAD.

Disable the GoranConnect toggle and wait for the legacy worker to exit. Confirm
that its SSH process has stopped and `/tmp/reverse_ssh_tunnel.pid` is gone. A
worker already running the older shutdown behavior may need its tunnel stopped
separately through the alternative access path.

```bash
sudo /data/teletyped/guardian/control activate
```

Activation writes/enables a separate systemd service, verifies its control socket,
then creates the migration marker. Manager subsequently stops launching the
fallback. Re-enable GoranConnect, request a tunnel, and verify a new SSH session.
Also test manager restart and device reboot before relying on this service.

The installed service and control wrapper select the captured interpreter and
its frozen shared-library path. For all later control use:

```bash
sudo /data/teletyped/guardian/control status
journalctl -u teletyped-guardian.service
```

If AGNOS replaces `/etc`, verify/reinstall the service using `activate` before
depending on it again. The unit and OS loader remain platform dependencies;
ordinary openpilot checkout updates do not overwrite the installed guardian.
Branches predating this migration do not honor the installed-service marker;
disable their legacy TeleTyped process entry before switching to them, to avoid
two workers writing production API state or competing for reverse ports.

## Testing a candidate

Stage with the installed deployment tool and the same frozen runtime, then trial:

```bash
sudo /data/teletyped/guardian/control stage candidate-001 --runtime device-python-v1 --source /data/openpilot
sudo /data/teletyped/guardian/control trial candidate-001
```

Staging audits integrity, imports in isolation as the configured worker user,
checks device JWT signing, and validates the existing tunnel keypair without
generating or uploading keys. Wrong account/key permissions fail before takeover.
If it fails, the existing LKG continues untouched. Published failed releases
cannot be retried under the same immutable ID; fix the source and use a new ID.

A candidate exit, main-loop progress timeout (120 seconds), or failure to finish
connectivity/confirmation within 15 minutes rolls back and quarantines that ID.
Disabling access stops the worker and tunnel, and abandons any trial. Restarting
the guardian or rebooting also abandons an unconfirmed trial. A full `/data`
preventing quarantine persistence does not stop in-memory recovery to an already
committed LKG; an interrupted on-disk trial is abandoned again on next boot.

Check `status`. Promotion requires five continuous minutes with recent main-loop,
accepted heartbeat, and authenticated SSH-request progress, plus connected tunnel
status. Then establish a **new authenticated SSH session through the reverse
tunnel**, read `/data/params/d/DongleId` and compare it with the intended device,
then confirm from that session:

```bash
sudo /data/teletyped/guardian/control confirm candidate-001 --ssh-verified
```

`--ssh-verified` is an operator attestation, not an automated end-to-end server
probe. The guardian additionally checks the local SSH banner. Heartbeat presence
alone cannot confirm a candidate. It never promotes automatically.

To abandon a trial, target the current confirmed ID. After promotion, rollback
also accepts the previous confirmed ID and makes that selection durable:

```bash
sudo /data/teletyped/guardian/control rollback initial-good
```

Retrying a quarantined ID requires `trial <id> --retry`. A fixed version should
normally receive a new ID. A confirmed LKG that crashes is retried with bounded
exponential backoff; candidates are never selected as an escape from an LKG
failure. Changing packages requires a new `capture-runtime` ID and another
staged release. Normal candidate deployment never replaces guardian code.

## Access and recovery policy

The installed service checks `GoranConnectEnabled` directly without importing
Params. While opted in, recovery SSH and heartbeat can operate onroad or while
manager is unavailable. This is an explicit change from the legacy worker's
launch predicate. Disabling the toggle closes the tunnel. Reboot, update and
power-policy API actions require fresh offroad manager state.

Bulk transfer/inventory/statistics/location jobs start only with fresh offroad
manager progress in `/dev/shm`; missing or stale manager state suppresses new
jobs. Already executing work is not preempted by this check. Recovery does not
wait for manager startup, registration, or an openpilot build. It still needs
existing identity material, network access, the API, SSH server, and local sshd.

Manifest commits use file fsync, atomic rename, and directory fsync. Interrupted
updates select an explicitly confirmed version, never the newest directory.
The runtime snapshots Python packages and SSH binaries, but OS shared libraries,
kernel, power, storage, and network remain common failure domains. Verify all
native dependencies and actual recovery on the target architecture before
considering installation complete.

The service owns a process group and kills remaining SSH/transfer children when
replacing a worker. Runtime identity records and a bootstrap parent watchdog
handle guardian loss; systemd also cleans the service cgroup on restart. Worker
logs are bounded to current/previous 1 MiB files. Guardian events go to journald.

## Validation

Focused tests cover real syntax/import/startup crashes, runtime death, a stuck
main loop with continuing heartbeats, connectivity deadlines, quarantine,
interrupted trials, exclusive ownership, orphan cleanup, failed atomic commits,
and frozen execution after the source and package directories are unavailable.
Run openpilot-dependent tests in the repository Docker environment. On the real
device additionally verify manager failure, reboot, SSH host authentication,
fresh tunnel login, and power interruption during promotion.
