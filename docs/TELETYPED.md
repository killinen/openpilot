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
- While offroad, report the device's last known GPS fix from retained route logs.

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
and configuration live under `tools/teletyped/profiles/`. The shared analyzer
handles engagement, distance accumulation, speed buckets and uploads.

To add a car, create a module in that directory and register its profile in
`profiles/__init__.py`'s `VEHICLE_PROFILES` tuple. No changes to `teletyped.py`,
`route_sender.py` or `drive_stats.py` are needed for the supported statistics
types. For example, a car using the standard `carState.steeringPressed` signal
and speed-integrated distance needs only:

```python
from openpilot.tools.teletyped.profiles.base import SteeringStatsConfig, VehicleStatsProfile

class ExampleStatsProfile(VehicleStatsProfile):
  name = "example_car"
  car_names = frozenset({"example_brand"})
  fingerprints = frozenset({"EXAMPLE CAR 2026"})
  steering_mode = "stock_eps"
  steering = SteeringStatsConfig(sources=("steering_pressed",))
```

The default matcher requires all configured car-name and fingerprint criteria;
names use lowercase and fingerprints uppercase, with surrounding spaces ignored.
Profiles may override `matches` for aliases, as the custom i30 does. Unmatched
vehicles do not generate statistics. Selection uses each route's logged
`carParams`; logged metadata overrides the current device's cached profile.

`SteeringStatsConfig.sources` supports `steering_pressed`, `torque_delta`, or an
empty tuple to report intervention fields as `null`. Configuration also sets the
torque filter window, trigger/release thresholds, minimum interval between
interventions, and engagement-change buffer. For torque comparison, override
`steering_values` if the car needs calibration or a different sign convention.
Override `decode_odometer_km` to supply an odometer; the default uses integrated
speed when no odometer is decoded.

Profiles supporting multiple steering sources can override `intervention_source`
and `get_steering_mode`. The i30 profile handles its logged `TRQISteeringToggle`
and legacy feedback detection this way. All configured sources are counted so
late metadata can select the correct result. Profiles are shared and stateless:
use the supplied route parameters, never live device settings or mutable profile
state. The LS600h profile enables no steering-intervention source and performs
no HRR temperature collection.

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

## Last known device location

`tools/teletyped/location.py` is a separate optional offroad worker. In this
checkout GPS, locationd and teletyped do not run at the same time: GPS runs
onroad; teletyped requires offroad and the GoranConnect access toggle. Location
reporting therefore uses the last valid GPS fix from retained drive logs.

Every 60 seconds it checks recent segments, preferring qlogs and falling back
to rlogs, including bz2 files. Files must be settled for 30 seconds. At most
three unexamined files are read per tick, and unchanged files are skipped.
Scanning stops when driving resumes and runs independently of drive statistics
and transfers. Invalid GPS reports never replace a valid fix.
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

The manager launches the service as `teletyped` (see
`system/manager/process_config.py`) when the **GoranConnect access** toggle is
on. It only runs on devices that report the `/TICI` marker—i.e. comma three /
three X hardware—so it will not execute on PC emulators or legacy NEOS
devices. The helper auto-detects persist and log paths using the hardware
abstraction so the same binary works across both NEOS-style and AGNOS
filesystems.

Heartbeats run in their own worker, so startup uploads, SSH operations and device
actions do not delay status reports. DNS fallback overrides apply only to the
requesting thread, allowing heartbeats to proceed during long uploads. SSH retry
backoff also leaves device actions polling.

Drive inventory, drive statistics and location reporting have independent
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
