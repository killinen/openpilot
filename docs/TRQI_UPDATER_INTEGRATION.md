# TRQI firmware updater integration

## Runtime ownership

```text
manager
├── pandad ── Panda USB ── Panda bus 1 ── TRQI
│   ├── publisher: cereal can
│   └── subscriber: cereal sendcan
└── trqiUpdater (OFFROAD/pre-ONROAD only)
    ├── subscriber: cereal can
    ├── publisher: cereal sendcan (only after update lease)
    ├── deviceState / pandaStates / controlsState / carState
    └── private GitHub Releases and signed local cache
```

`pandad` remains the only Panda USB owner. The updater is stopped whenever the
driving stack is ONROAD. Physical ignition may nevertheless be on: when a
complete candidate is pending, `hardwared` holds `deviceState.started` false on
the ignition rising edge. TRQI is then powered while the updater, UI, and
`pandad` remain in the pre-ONROAD state.

## Two-phase update

1. With openpilot OFFROAD, download and authenticate `release.json`, its
   signature, and both A/B manifest/image pairs.
2. Set `TrqiUpdatePending` only after the complete cache is durable.
3. At the next ignition, keep all driving processes stopped.
4. Read application identity frames on Panda bus 1 and compare the staged and
   installed versions.
5. Acquire Panda safety model 32, select the inactive slot, enter the bootloader,
   transfer durably, commit TRIAL, and wait for application confirmation.
6. Release the hold only after confirmation, safe rollback/recovery, or a
   pre-entry decision that no update is needed.

No network connection is required after ignition. The updater never searches
other CAN buses and never updates the service bootloader over CAN.

## Firmware publication and fast test loop

The integration baseline is TRQI branch `after-CAN-bootloader`, validated at
commit `1a78aab70267d53ccaf0fd40652a74a9e3f4f4f0`. The device intentionally does
not install mutable branch build artifacts. Production updates come from
immutable GitHub Releases whose signed `release.json` records the exact git SHA.

The TRQI repository already contains `.github/workflows/release.yml`: pushing a
stable `vMAJOR.MINOR.PATCH` tag builds, tests, signs, and publishes both slots.
Ordinary experimental commits do not need that production workflow. Build a
TEST-key A/B bundle locally, copy it to the device, and use `stage-local`; this
performs the complete host-side authentication/staging path without CAN, then
uses the same next-ignition installer. The UI always labels that path
`TEST KEY — NOT PRODUCTION`.

## Trust boundaries

- GitHub HTTPS and a read-only token in the `DONT_LOG` `TrqiGithubToken`
  Params key provide private-repository distribution.
- The canonical `release.json` signature authenticates the release index.
- Each 164-byte manifest independently authenticates its exact slot image.
- The production updater pins the same Ed25519 public key as the TRQI bootloader.
- Test-key bundles require an explicit local-only CLI mode and confirmation.
- Panda safety mode 32 permits only valid `0x60A` and `0x6A0` frames on bus 1.
- The TRQI application decides whether safe bootloader entry is allowed.
- The TRQI bootloader remains authoritative for product, flash geometry,
  inactive-slot policy, rollback security version, SHA-256, signature, atomic
  trial commit, confirmation, and rollback.

## Recovery

Tokens, sequence numbers, target slot, image identity, and durable offsets are
reconciled after timeouts and process restarts. If a confirmed application is
known, bootloader inactivity returns to it. If no confirmed application exists,
the host does not release ONROAD merely because a deadline expired.
