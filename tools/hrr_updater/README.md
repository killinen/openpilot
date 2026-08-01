# HRR updater for AGNOS/openpilot

This package stages, discovers, flashes, resumes, and confirms signed HRR A/B application releases. HRR is powered only while vehicle ignition is active, so Internet download and CAN installation are deliberately separate phases. Production CAN traffic uses cereal `can`/`sendcan` through the already-running `pandad`; it never opens Panda USB directly and has no SocketCAN or `python-can` dependency.

The complete architecture and integration contract are in [`docs/OPENPILOT_UPDATER_INTEGRATION.md`](../../docs/OPENPILOT_UPDATER_INTEGRATION.md).

## Safety model

The updater runs as the always-alive manager process `hrrUpdater`. While parked and HRR is unpowered, it downloads and verifies both slot artifacts and sets `HrrUpdatePending`. On the next ignition rising edge, `hardwared` keeps openpilot OFFROAD for a bounded preflight while HRR powers up. `pandad` remains running and owns Panda.

Before any update transmit, the updater requires all of the following:

- a verified complete A/B candidate was staged before ignition
- `deviceState.started == false`, `IsOffroad == true`, and `IsOnroad == false`
- controls not engaged and Panda `controlsAllowed == false`
- zero `carState.vEgo` when that message is available
- `HrrUpdateStartupHold` is active while Panda reports ignition
- Panda safety mode `hrrUpdater` (32), granted by `pandad`

That Panda mode permits only classic-CAN DLC-8 frames on Panda bus 2:

| ID | Purpose | Additional validation |
|---:|---|---|
| `0x60A` | Application ENTER_BOOTLOADER | magic, protocol, nonzero nonce, key, CRC-8 |
| `0x6A0` | Bootloader ISO-TP request | valid SF/FF/CF PCI and maximum 600-byte request |

It permits no torque, relay, calibration, forwarding, or arbitrary diagnostic frames. Panda safety remains authoritative. HRR application safe-state checks and the HRR bootloader's slot/range/signature/rollback checks remain independently authoritative.

Do not stop `pandad`, start a second Panda connection, or run this while `card` is publishing `sendcan` ONROAD.

## Provision the production public key first

[`keys/hrr_release_public_key.hex`](keys/hrr_release_public_key.hex) is intentionally all zero in this development tree. The updater fails closed until it is replaced with the same 32-byte raw Ed25519 public key compiled into the HRR bootloader. Never copy a private key onto the comma device.

The matching HRR repository key locations and GitHub secret setup are documented in its `docs/README_FIRMWARE_RELEASES.md`.

## Commands

```bash
python3 -m tools.hrr_updater.cli info --bus 2
python3 -m tools.hrr_updater.cli status --bus 2
python3 -m tools.hrr_updater.cli check --bus 2
python3 -m tools.hrr_updater.cli download --bus 2  # stages and verifies both slots

python3 -m tools.hrr_updater.cli flash --bus 2 \
  --manifest /data/hrr_updater/cache/hrr-v0.2.0-slot-b.manifest \
  --image /data/hrr_updater/cache/hrr-v0.2.0-slot-b.bin

python3 -m tools.hrr_updater.cli update --bus 2    # stages now; installs next ignition
python3 -m tools.hrr_updater.cli update --bus 2 --release v0.2.0
python3 -m tools.hrr_updater.cli recover --bus 2
python3 -m tools.hrr_updater.cli auto --bus 2
```

`info`, `status`, `check`, and `download` cannot call the flashing state machine. `check` and `download` do not require powered HRR. `download` and `update` stage complete verified A and B artifacts; automatic installation occurs during the next ignition startup hold. `flash` uses only local files and performs no network request. Use `--dry-run`, `--json`, `--no-reset`, `--allow-prerelease`, and `--force-retry-failed-release` where applicable.

The terminal display reports each state and shows a live durable-progress bar, throughput, negotiated chunk size, and retries. Progress advances only from a bootloader acknowledgement or a subsequent STATUS query, never merely because a CAN frame was queued.

## Automatic mode

Manager starts `tools.hrr_updater.daemon` continuously. Automatic installation defaults off:

```bash
params put HrrAutoInstall 1
params put HrrAllowPrerelease 0
```

It checks at startup and then approximately every six hours while OFFROAD. A newer signed release is fully downloaded for both slots while HRR remains untouched and usually unpowered. On the next ignition edge the UI reports preflight/installation, HRR status selects the inactive slot, and no network access is used. Network errors use exponential backoff. A candidate that rolls back is recorded by exact version and image SHA-256 and is not automatically retried; manual retry requires `--force-retry-failed-release`.

For a private release repository, set a read-only `HRR_GITHUB_TOKEN` or `GH_TOKEN` in the process environment. Tokens and Authorization headers are never logged. Override the repository with `--repository owner/repository` or `HRR_GITHUB_REPOSITORY`.

Verified artifacts and restart state default to `/data/hrr_updater`. Override this with `HRR_UPDATER_DATA_DIR` or `--data-dir`.

## Busy-bus and restart behavior

ISO-TP consecutive frames are intentionally paced at 3 ms by default even though HRR advertises STmin 0. The default application block is 128 bytes, far below the bootloader's 512-byte maximum, reducing burst pressure on a busy low-priority CAN path. CAN arbitration may delay frames without losing durable progress.

If an ACK is lost, the updater waits for the STM32 ISO-TP timeout, queries extended STATUS, checks the session token/image identity/sequence, and resumes at the bootloader's durable `next_offset`. It supports restart during transfer, after COMMIT, during TRIAL, and after the confirmation reset. It never scans partially written flash.

Before bootloader entry, failure or a 20-second preflight timeout releases the startup hold and leaves the running firmware unchanged. After entry, failures attempt ABORT/RESET and require a runnable application before the normal hold is released. Bootloader 1.1.1 independently abandons an incomplete session after 60 seconds without valid host traffic and resets to its verified confirmed slot. A renewable `HrrUpdateDeadline` provides a final 10-minute manager-side crash guard, so a dead updater cannot block startup indefinitely. Recovery mode with no valid confirmed image intentionally stays blocked because driving with no runnable HRR is not made safe by a timer.

## Tests

```bash
python3 -m unittest tests.test_hrr_updater_core -v
```

The fake application/bootloader covers A/B choice, recovery mode, duplicate commands, lost START/DATA/FINISH ACKs, durable resume, authentication errors, confirmation, rollback, and failed-candidate persistence. Panda's native safety test is `panda/tests/safety/test_hrr_updater.py` and should be built/run in the repository Docker environment.

These are software tests. Production readiness still requires real HRR, Panda, `pandad`/cereal, busy-bus loss/duplication, power interruption, process restart, trial rollback, and real OFFROAD-to-ONROAD transition testing.
