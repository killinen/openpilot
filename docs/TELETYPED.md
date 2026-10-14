# Teletyped (Goran Connect)

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

Vehicle profiles support the custom 2014 Hyundai i30 and Lexus LS600h. The i30
odometer is decoded from `CLU1`; legacy SSC steering interventions use the same
filtered driver/actuator torque delta as `opDriveStats`, while TRQI drives use
the rising edge of the hardware-reported `steeringPressed` state. The LS600h
distance is integrated from logged `carState.vEgo` because its odometer signal
has not been identified. Its steering-intervention fields remain `null` until
a reliable driver-intervention signal is available. Vehicle-specific decoding
is isolated in `tools/teletyped/drive_stats.py` so another profile can be added
without changing discovery, persistence, or upload behavior.

The upload body uses the `opDriveStats` per-drive field names (`total_time`,
`active_time`, `odo_distance`, `engaged_distance`, `engagement_pct`,
`engagement_pct_odo`, `steer_intervention_count`, and normalized rates). It
also reports raw and shutdown-corrected disengagement counts, disengagements
per 100 km, and disengagements per driving hour. The final unmatched
engaged-to-disengaged transition is treated as the manual shutdown event.
It also reports non-intersection disengagements using single-blinker activity
from eight seconds before through three seconds after each transition. Both
blinkers active together are treated as hazard lights rather than turn intent.
The payload includes the detection method and window so this heuristic remains
explicit to downstream consumers.

The `speed_buckets` object splits the same drive into `city` (below 55 km/h),
`road` (55 km/h up to 90 km/h), and `highway` (90 km/h and above), matching
the thresholds used by `opDriveStats`. Each bucket includes total/engaged time
and distance, engagement percentages, steering interventions, raw and
shutdown-corrected disengagements, and normalized per-distance/per-driving-hour
rates, including the blinker-derived non-intersection split. Bucket distance is
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

`hrr_temperature` schema **2** supports the custom Lexus LS600h. It receives HRR
`0x638` on bus 2, speed `0xB4` on bus 0, ambient `0x3B0` on bus 1, and tentative
engine/coolant temperature `0x039` on bus 1. Historical four-byte and current
eight-byte HRR layouts are supported. Valid repeated temperature counters do
not refresh the reading. Invalid reports carry an explicit null value.

Every accepted HRR report is retained. Comparison streams match the desktop
`hrr-temp-tracker` library: the last report of each absolute second is retained
with its original integer nanosecond timestamp. Comparison sampling is part of
the collector format; gzip compression itself discards no data. Ambient uses the
**provisional** byte-3-minus-49 °C decode and includes the original raw byte.
Engine uses the **tentative** byte-0 Celsius decode. Both caveats are in metadata.

Coverage includes each segment's first/last CAN timestamps, event count and HRR
valid/invalid/repeated/malformed counts. A matching vehicle with CAN but no HRR
reports uploads coverage rather than disappearing from the library. Routes with
missing rlogs upload the available segments; qlogs are excluded. All rlogs must
be at least 120 seconds old before analysis.

Snapshots contain batches of at most 600 records, with device/route IDs,
collector/schema, car and software metadata, source fingerprint, segment list,
recording time, monotonic origin and batch index/count. Changes to source logs
create distinct immutable snapshots. HTTP 409 acknowledges an already stored
batch. Schema 1 remains accepted by the server and readable by opDriveStats;
its engine data is absent and coverage is estimated from HRR report intervals.

Pending state is compressed in
`/persist/comma/teletyped_telemetry_state.json.gz`. The previous plain JSON queue
is migrated after a successful atomic write, including pending schema-1 jobs.
Malformed state is preserved for recovery. Uploads resume at the first
unacknowledged batch after a restart, even if loggerd deleted their source logs.
Requests send gzip JSON using `Content-Encoding: gzip`; the server accepts both
gzip and plain JSON and stores accepted payloads as compressed blobs. Exported
JSON retains all decoded values and timestamps.

The module defaults on for matching LS600h routes when GoranConnect access is
on. Set `TELETYPED_HRR_TEMPERATURE=0` to pause it, including pending jobs.
`TELETYPED_TELEMETRY_MAX_PER_TICK` defaults to 1 route/retry job per poll;
`TELETYPED_TELEMETRY_RETRY_INTERVAL` defaults to 900 seconds.

To add a module, implement the `Collector` protocol (`key`, `schema_version`,
`matches`, `observe`, `result`) with a no-argument constructor and register the
class in `COLLECTOR_TYPES`. Results contain `samples` with `time_s` and `segment`,
plus module metadata. Heavy snapshot metadata can go in `first_batch`, which
is sent only in batch zero. The switch is `TELETYPED_<KEY_IN_UPPERCASE>`. Bump schema
versions for changed decode semantics and register the corresponding server
validator in `telemetry_collectors.py`.

**Deploy the GoranConnect server update before the device update.** Its
`GET /api/drive-telemetry/export` feed uses the existing drive-stats export token
and negotiates gzip responses. Export consumers group batches by
`(device_id, collector, schema_version, drive, source_fingerprint)` and publish
only complete snapshots. opDriveStats synchronizes this feed independently of
engagement stats, keeps original batches compressed, and updates covered segments
without deleting previously uploaded history. Its protected `/temperatures`
page reproduces Conditions/library heatmaps, filters, coverage and CSV exports.

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
