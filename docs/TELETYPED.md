# Teletyped (Goran Connect)

For independent supervision and last-known-good rollback, see
[TeleTyped recovery deployment](TELETYPED_RECOVERY.md). Until that service is
explicitly provisioned, manager uses the compatibility subprocess described below.

FrogPilot now bundles the **teletyped** service that powers the Goran Connect
remote support workflow. The daemon runs on-device (comma three / three X on
AGNOS) and handles these capabilities:

- Maintain a reverse SSH tunnel so support staff can reach the device when an
  access request is created on the server.
- Upload SSH keys for the device and keep the connection heartbeat/status in
  sync with the cloud API (`https://goranconnect.duckdns.org/api`).
- Serve remote drive requests by packaging logs or boot traces and sending
  them via the bundled `wormhole-william` binary.
- While offroad, derive engagement time/distance and steering-intervention
  statistics from completed route logs and upload them to `POST /api/drive-stats`.
- Collect optional vehicle telemetry from offroad rlogs and upload it through
  the shared `POST /api/drive-telemetry` endpoint.

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

The motorhome profile integrates distance from logged `carState.vEgo`. Its
steering-intervention fields are `null`: the current motorhome interface does
not report steering torque or driver intervention. Its steering mode is SSC.

The upload body uses the `opDriveStats` per-drive field names (`total_time`,
`active_time`, `odo_distance`, `engaged_distance`, `engagement_pct`,
`engagement_pct_odo`, `steer_intervention_count`, and normalized rates). It
also reports raw and shutdown-corrected disengagement counts, disengagements
per 100 km, and disengagements per driving hour. The final unmatched
engaged-to-disengaged transition is treated as the manual shutdown event.
The motorhome interface does not report blinkers, so blinker-related and
non-intersection disengagement fields are `null` rather than inferred.
These uploads use payload schema 3, which permits unavailable classifications.

The `speed_buckets` object splits the same drive into `city` (below 55 km/h),
`road` (55 km/h up to 90 km/h), and `highway` (90 km/h and above), matching
the thresholds used by `opDriveStats`. Each bucket includes total/engaged time
and distance, engagement percentages, steering interventions, raw and
shutdown-corrected disengagements, and normalized per-distance/per-driving-hour
rates. Unavailable steering and blinker fields are also null in each bucket. Bucket distance is
integrated from `carState.vEgo` because an odometer delta cannot be assigned to
a speed range; the overall distance prefers a profile's odometer when available
and otherwise uses the same integration.

Metadata includes device and route IDs, recording and generation timestamps,
segment count, car name/fingerprint and stats profile, longitudinal/steering
mode, device type, software version, Git branch/commit/date, and dirty-tree
state. The server must treat `(device_id, drive)` as an idempotency key; an
HTTP 409 response is also treated as an already-stored success by the device.

## Optional vehicle telemetry

`tools/teletyped/telemetry.py` handles discovery, batching, durable retries and
transport. Vehicle decoding lives under `tools/teletyped/collectors/`. Modules
activate only for matching **logged** car metadata, including CAN reports that
precede carParams. Collection has its own worker and interrupts when driving
resumes. Current device CarParams are never applied to another route.

No vehicle-specific collectors are registered in this checkout, so the telemetry
worker does not read or upload telemetry until a collector is added. To add one,
implement the `Collector` protocol (`key`, `schema_version`, `matches`, `observe`,
`result`) and register its class in `COLLECTOR_TYPES`. Collection and durable
retry use the shared `POST /api/drive-telemetry` endpoint.

## Last known device location

`tools/teletyped/location.py` is a separate optional offroad worker. In this
checkout GPS, locationd and teletyped do not run at the same time: GPS runs
onroad; teletyped requires offroad and the GoranConnect access toggle. Location
reporting therefore uses the last valid GPS fix from retained drive logs.

Every 60 seconds it checks recent segments, preferring qlogs and falling back
to rlogs, including bz2 files. Files must be settled for 30 seconds. At most
three unexamined files are read per tick, and unchanged files are skipped.
Scanning stops when driving resumes and runs independently of drive statistics,
telemetry processing and transfers. Invalid GPS reports never replace a valid fix.
The original GPS UTC timestamp, accuracy and source are retained.

The last fix is saved privately in
`/persist/comma/teletyped_last_location.json`, so deleting logs or restarting
teletyped does not erase it. When no timestamped fix is available, the persistent
`LastGPSPosition` parameter can provide coordinates; its capture time remains
unknown because that parameter has no timestamp. Unavailable GPS never produces
invented coordinates. Set `TELETYPED_LOCATION=0` to disable location reporting.

`POST /api/device-locations/{device_id}` uses the existing device JWT. The server
keeps one snapshot per device and rejects older reports over newer timestamped
fixes. It does not create a location history. Authenticated users select a device
in GoranConnect's map and press **Show last known location** to fetch and display
it, with its GPS time, age and accuracy. Server receipt time never refreshes GPS
age; untimestamped saved positions are explicitly marked unknown. The existing
OwnTracks map and playback remain available.

Deploy the GoranConnect backend and rebuilt frontend before updating the device.

## Runtime Integration

Without an installed recovery service, manager launches a fresh subprocess as `teletyped` (see
`system/manager/process_config.py`) when the **GoranConnect access** toggle is
on and the device is offroad. Manager does not pre-import TeleTyped, and retries
an exited compatibility worker after five seconds. It only runs on devices that report the `/TICI` marker—i.e. comma three /
three X hardware—so it will not execute on PC emulators or legacy NEOS
devices. The helper auto-detects persist and log paths using the hardware
abstraction so the same binary works across both NEOS-style and AGNOS
filesystems.

The independent recovery service has its own opt-in and offroad-work policy;
see the recovery deployment guide before enabling it.

Heartbeats run in their own worker, so startup uploads, SSH operations and device
actions do not delay status reports. DNS fallback overrides apply only to the
requesting thread, allowing heartbeats to proceed during long uploads. SSH retry
backoff also leaves device actions polling.

Drive inventory, drive statistics, telemetry and location reporting have independent
workers so long transfers or log analysis do not delay scan requests. Requested
route segments are protected with deletion locks while being archived or
uploaded directly; transfer locks are released after those operations.

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
