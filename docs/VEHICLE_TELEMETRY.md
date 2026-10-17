# Optional vehicle telemetry

Vehicle-specific telemetry lives in `tools/vehicle_telemetry`, outside teletyped.
Teletyped only starts an authenticated offroad callback through its generic
`extensions.py` loader. Its route inventory, transfers, heartbeat and ordinary
drive statistics do not decode or depend on TRQI signals. An unavailable
extension is logged and does not prevent the core workers from starting.

## Boundaries and configuration

- `tools/teletyped/extensions.py`: imports configured `module:callable` jobs.
- `tools/vehicle_telemetry/worker.py`: route discovery, logged identity selection,
  batching, durable retries and gzip upload; no vehicle CAN definitions.
- `tools/vehicle_telemetry/collectors/`: independent stateful per-route collectors.
- goranConnectServer: collector-specific validators behind shared authenticated
  `/api/drive-telemetry` storage and cursor export.
- opDriveStats: shared cursor replication plus independent TRQI analysis, tables,
  routes and dashboard in `server/trqi_temperature`.

The default `TELETYPED_EXTENSIONS` is
`openpilot.tools.vehicle_telemetry.worker:telemetry_step`. Set it to an empty
string to disable all optional jobs. A comma-separated list enables additional
callbacks accepting `(device_id, headers)`; each receives its own worker thread.
Failures and slow work stay isolated from heartbeats and transfers.

`TELETYPED_TRQI_TEMPERATURE=0` disables only the TRQI collector;
`TELETYPED_HRR_TEMPERATURE=0` disables the ported LS600h collector. Each collector
also checks the **recorded** vehicle identity. An i30 with no TRQI frames does
not produce TRQI telemetry. Shared limits remain
`TELETYPED_TELEMETRY_MAX_PER_TICK` (1) and
`TELETYPED_TELEMETRY_RETRY_INTERVAL` (900 seconds).

Collection reads settled rlogs offroad and stops when ONROAD begins. There are
no CAN transmissions or firmware-updater dependencies. It waits 120 seconds
after log modification, uploads at most 600 samples per batch, and saves pending
payloads before sending them. Retries survive deletion of their source logs.
The existing `teletyped_telemetry_state.json.gz` queue name and schema are retained
for compatibility with the LS600h transport. Old queues remain readable.

## TRQI schema 2

`trqi_temperature` uses the shared schema-2 timestamp/coverage envelope; version
numbers are scoped to the collector, not globally to a device or route.

- Bus 1 `0x245`, 8 bytes, 1 Hz: signed little-endian temperature × 0.1 °C,
  raw ADC, validity, fault/pending flags, ADC background state, VDDA warning,
  reset cause, loop maximum, transmit counter and CRC-8/poly07.
- Original temperature frame hex is retained, including reports with bad CRCs.
  Invalid reports carry an explicit null temperature and break freshness.
- Equal temperatures with progressing counters remain observations. Exact
  repeated frames within two seconds count as duplicates; a long gap or counter
  wrap is not automatically discarded. This is a transmit counter, not proof of
  a fresh ADC conversion.
- `carState.vEgo`: km/h; `carControl.latActive` and absolute normalized
  `carControl.actuators.steer`: control context. Bus 1 `0x241` supplies the
  CRC-checked relay state.
- Bus 0 `0x350` HVAC frame: outside values `byte3/byte4 * 0.5 - 40`, blower and A/C
  request bits, compressor-load code. These are **provisional DBC candidates**,
  not verified cabin temperature or delivered cooling. Out-of-range outside
  codes are omitted. Validate the bus, decode and HVAC meaning against real logs
  before relying on them. A/C request and compressor-load code are distinct.
- Passive bus 1 `0x639..0x63C` application identity frames are retained on change.
  No identity request is sent. Missing identity remains unknown. Hardware and
  installation revisions are not inferred from firmware identity.

Companions keep the last original report per absolute second; all nonduplicate
MCU reports remain. Coverage includes observed segments without MCU reports,
plus valid, invalid, malformed, repeated, CRC-error and counter-discontinuity
counts. It covers the observed log interval, not powered time before logging.

## Analysis on opDriveStats

`/trqi-temperatures` uses a one-second grid. TRQI expires after 1.5 seconds,
speed/control context after 2 seconds, HVAC after 5 seconds and outside after
60 seconds. Invalid reports and absent segments remain gaps. The module owns
`trqi_segments`, `trqi_routes` and `trqi_published`; shared HRR tables are untouched.
Only complete batch snapshots are published, and retained older segments survive
later source revisions that contain fewer logs.

The main conditions view groups startup temperature by 10 °C and route phase
(0–5, 5–15, 15–30, 30+ minutes). Separate comparisons show provisional outside
temperature by 5 °C, blower state and speed. Missing companions remain unknown
rather than excluding MCU temperature evidence. Filters include device,
passively observed firmware, route phase and blower state. Each cell shows valid
seconds and drive counts, with user-selectable minimum coverage.

Each drive reports first valid temperature within 30 seconds of a known route
start, median, P95, grid maximum, original-report maximum, highest continuous
30-second mean, changes at 5/15/30 minutes and approximate grid seconds at or
above 82/85 °C. Firmware fault and pending flags are separate evidence. Unknown
route starts do not get a startup estimate. Route time is not firmware uptime.
Missing parked/offroad observations cannot reconstruct thermal soak or cooldown.

MCU temperature is junction temperature. Outside temperature is context, not the
local air temperature at the board; subtracting it does not isolate self-heating.
The dashboard makes no causal cooling claim about speed. Compare similar starts,
phases and cabin conditions, using independent drives as the unit of evidence.
For a physical heat-rise study, add a reference probe near the enclosure.

## Rollout and focused verification

Deploy server validators first, then opDriveStats, then enable the device worker.
Both servers run elsewhere; local checkout changes do not restart or deploy them.
Existing opDriveStats telemetry-sync URL/token settings are reused. New routes
inherit its existing authentication middleware. The dashboard offers raw sample
and condition CSV exports; integer nanosecond timestamps are preserved.

Run device tests in the disposable Docker workflow in AGENTS.md. Copy new
untracked modules into the container after copying tracked files. The focused
targets are `opendbc_python` and `common/params_pyx.so`; tests are
`tools/vehicle_telemetry/tests`, `tools/teletyped/tests/test_extensions.py`, and
the existing route-worker polling smoke test. Use `--confcutdir=tools` to avoid
loading unrelated process-manager fixtures.

Server checks are the telemetry endpoint/compression tests, new TRQI validation
tests, existing temperature/import tests, and new TRQI dashboard/analysis tests.
Use temporary databases, never the running servers' production databases.
