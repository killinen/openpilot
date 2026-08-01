# HRR updater integration in openpilot

## Runtime ownership

```text
AGNOS manager
├── pandad ── Panda USB ── Panda bus 2 ── HRR
│   ├── publisher: cereal can
│   └── subscriber: cereal sendcan
└── hrrUpdater
    ├── subscriber: cereal can
    ├── publisher: cereal sendcan (OFFROAD update lease only)
    ├── deviceState / pandaStates / controlsState / carState
    └── GitHub HTTPS and signed local cache
```

`pandad` is the only Panda USB owner and stays alive. `hrrUpdater` does not require Linux `can0`, SocketCAN, `python-can`, or direct USB. On this fork, `card` runs only ONROAD and `hrrUpdater` is the only `sendcan` publisher during the ignition-on, pre-ONROAD update lease.

HRR itself is ignition powered. The updater therefore prefetches and verifies both A/B artifacts while parked, before it knows the live active slot. `HrrUpdatePending` causes `hardwared` to hold the next ignition transition OFFROAD long enough to discover HRR, compare versions, select the inactive artifact, and either install it or release startup unchanged.

## Implemented integration

- `cereal/car.capnp`: safety model 32, `hrrUpdater`
- `panda/board/safety/safety_hrr_updater.h`: bus-2/DLC-8 whitelist for `0x60A` and `0x6A0`
- `selfdrive/pandad/pandad.cc`: OFFROAD lease, Panda power-save inhibition, and mode confirmation
- `system/hardware/hardwared.py`: persistent ONROAD-start block while HRR is in an update transaction
- `system/manager/process_config.py`: always-running `hrrUpdater` process
- `tools/hrr_updater`: transport adapter, signed release/cache handling, explicit state machine, recovery, CLI, daemon, and fake bootloader

The Params handshake is:

| Parameter | Meaning |
|---|---|
| `HrrUpdaterRequested` | updater asks `pandad` for the narrow safety mode |
| `HrrUpdaterReady` | `pandad` has observed safety mode 32 and power save off |
| `HrrUpdatePending` | complete signed A/B candidate is cached for next ignition |
| `HrrUpdateStartupHold` | bounded ignition-on preflight keeps driving processes stopped |
| `HrrUpdateInProgress` | HRR may be unavailable; keep openpilot OFFROAD across process restarts |
| `HrrUpdateDeadline` | renewable final process-crash deadline |
| `HrrUpdateStatus` | status text shown by `Offroad_HrrFirmwareUpdate` |
| `HrrAutoInstall` | opt in to stable automatic installation |
| `HrrAllowPrerelease` | opt in to prerelease discovery for automatic mode |

The initial lease is denied when `IsOnroad` or `IsEngaged` is active. Ignition is permitted only while `HrrUpdateStartupHold` is active. Once flashing starts, `pandad` retains only the HRR updater safety mode; `hardwared` keeps all driving processes stopped until confirmation/recovery or the final crash deadline.

## Ignition-powered startup sequence

```text
OFFROAD, ignition off
  → poll signed GitHub Releases
  → download and verify release.json plus slot A and slot B
  → set HrrUpdatePending

next ignition rising edge
  → hardwared sets HrrUpdateStartupHold before ONROAD
  → UI shows HRR firmware status
  → wait for HRR application frames on configured Panda bus
  → compare installed and staged semantic versions
  → no newer image / HRR absent / preflight failure: release hold unchanged
  → newer compatible image: acquire Panda safety mode 32 and set InProgress
  → install inactive slot, trial, confirm, clear hold
```

The pre-entry hold is limited to 20 seconds. Once HRR has entered its bootloader, bootloader 1.1.1 resets to the verified confirmed application after 60 seconds without valid updater traffic. The host deadline is renewed during durable progress and expires after 10 minutes of updater silence. The only intentional indefinite recovery state is one where no verified confirmed HRR application exists; allowing ONROAD cannot make that condition safe.

## Wire contract and filtering

Panda/openpilot bus number is explicit and defaults to 2. The updater first requires application status on that exact bus and never searches and transmits across other vehicle buses automatically.

Accepted receive IDs are:

- application information: `0x639`, `0x63A`, `0x63B`, `0x63C`, DLC 8
- bootloader response/ISO-TP flow control: `0x6A1`, classic CAN

Allowed transmit IDs are:

- `0x60A`, DLC 8, keyed and CRC-protected application entry request
- `0x6A0`, DLC 8, ISO-TP application-update request

The updater ignores the service bootloader binary and rejects any manifest whose load address intersects `0x08000000`–`0x0800FFFF`. There is no CAN bootloader self-update path.

Panda returned frames (`src = 0x80 | bus`) confirm queue acceptance. Rejected frames (`src = 0xC0 | bus`) produce an explicit safety error. End-to-end success still comes only from HRR responses and durable offsets.

## Trust boundaries

| Boundary | Authority |
|---|---|
| GitHub HTTPS/token | distribution and private-repository access |
| signed canonical `release.json` | authentic release index and filenames |
| signed binary manifest plus image SHA-256 | application authenticity |
| Python updater | local verification and orchestration |
| Panda safety model 32 | narrow host CAN transmit enforcement |
| HRR application | local safe bootloader-entry decision |
| HRR bootloader | final hardware, slot, flash, signature, security-version, trial, and rollback authority |

Compromise of the Python updater does not make unsigned firmware bootable. The production private key is never part of this repository or installed on AGNOS.

## Deployment checklist

1. Provision the same nonzero Ed25519 public key in the HRR bootloader, HRR release repository key file, and `tools/hrr_updater/keys/hrr_release_public_key.hex`.
2. Configure the HRR GitHub protected `production-release` Environment and secret.
3. Build/deploy this fork so Panda firmware contains safety model 32.
4. Confirm HRR status IDs on Panda bus 2 during an ignition-on OFFROAD startup hold.
5. Run `check` and `download` with ignition off, then `info` during ignition-on preflight before enabling `HrrAutoInstall`.
6. Perform the real-hardware fault matrix listed in the updater README.

The Panda safety implementation and its native unit test are included here; no arbitrary-CAN safety bypass remains as an external task. Production public-key provisioning and real hardware validation remain deployment tasks.
