# Teletyped (Goran Connect)

FrogPilot now bundles the **teletyped** service that powers the Goran Connect
remote support workflow. The daemon runs on-device (comma three / three X on
AGNOS) and handles three capabilities:

- Maintain a reverse SSH tunnel so support staff can reach the device when an
  access request is created on the server.
- Upload SSH keys for the device and keep the connection heartbeat/status in
  sync with the cloud API (`https://goranconnect.duckdns.org/api`).
- Serve remote drive requests by packaging logs or boot traces and sending
  them via the bundled `wormhole-william` binary.
- While offroad, derive engagement time/distance and steering-intervention
  statistics from completed route logs and upload them to `POST /api/drive-stats`.

## Requirements

1. **Allow access** – enable *Settings → Toggles → GoranConnect access* on a comma three / three X. The `teletyped` service only runs when this toggle is on.
2. **Device registration key** – teletyped signs requests with the device RSA key at `/persist/comma/id_rsa` (created during device setup).
3. **Internet access** – the helper watches `DeviceState.NetworkType` and
   sleeps while offline.

All SSH material is written under `/persist/comma/` and re-used across boots.

## Offroad drive statistics

The route worker processes one unprocessed drive per poll by default, records
its progress in `/persist/comma/teletyped_drive_stats_state.json`, and retries
failed uploads without parsing the route again. Set
`TELETYPED_DRIVE_STATS_MAX_PER_TICK` to increase the backfill rate, or set
`TELETYPED_DRIVE_STATS=0` to disable collection. Upload failures retry after
15 minutes by default; `TELETYPED_DRIVE_STATS_RETRY_INTERVAL` changes that
delay in seconds.

The initial vehicle profile supports the custom 2014 Hyundai i30. Its odometer
is decoded from `CLU1`; legacy SSC steering interventions use the same filtered
driver/actuator torque delta as `opDriveStats`. TRQI drives use the rising edge
of the hardware-reported `steeringPressed` state. Vehicle-specific decoding is
isolated in `tools/teletyped/drive_stats.py` so another profile can be added
without changing discovery, persistence, or upload behavior.

The upload body uses the `opDriveStats` per-drive field names (`total_time`,
`active_time`, `odo_distance`, `engaged_distance`, `engagement_pct`,
`engagement_pct_odo`, `steer_intervention_count`, and normalized rates). It
also reports raw and shutdown-corrected disengagement counts, disengagements
per 100 km, and disengagements per driving hour. The final unmatched
engaged-to-disengaged transition is treated as the manual shutdown event.

The `speed_buckets` object splits the same drive into `city` (below 55 km/h),
`road` (55 km/h up to 90 km/h), and `highway` (90 km/h and above), matching
the thresholds used by `opDriveStats`. Each bucket includes total/engaged time
and distance, engagement percentages, steering interventions, raw and
shutdown-corrected disengagements, and normalized per-distance/per-driving-hour
rates. Bucket distance is integrated from `carState.vEgo` because an odometer
delta cannot be assigned to a speed range; the overall distance continues to
prefer the i30 odometer when it is available.

Metadata includes device and route IDs, recording and generation timestamps,
segment count, car name/fingerprint and stats profile, longitudinal/steering
mode, device type, software version, Git branch/commit/date, and dirty-tree
state. The server must treat `(device_id, drive)` as an idempotency key; an
HTTP 409 response is also treated as an already-stored success by the device.

## Runtime Integration

The manager launches the service as `teletyped` (see
`system/manager/process_config.py`) when the **GoranConnect access** toggle is
on. It only runs on devices that report the `/TICI` marker—i.e. comma three /
three X hardware—so it will not execute on PC emulators or legacy NEOS
devices. The helper auto-detects persist and log paths using the hardware
abstraction so the same binary works across both NEOS-style and AGNOS
filesystems.

## Troubleshooting

Logs live under `/data/media/0/realdata` (with uploads staged in `/tmp`) and
basic status messages are printed to stdout/stderr in the `teletyped` process
output. You can also tail `tools/teletyped/sender_log.json` to see a history of
route transfers. When debugging connectivity issues:

- Ensure `/etc/resolv.conf` is populated. The helper will bind-mount a known
  good resolver list via `tools/teletyped/setup_resolv.sh` if one is missing.
- Confirm that `ssh` and `wormhole-william` are executable (both are bundled
  with proper permissions in this repo).
- After enabling GoranConnect access, the daemon will automatically retry key
  uploads every five minutes until success.
