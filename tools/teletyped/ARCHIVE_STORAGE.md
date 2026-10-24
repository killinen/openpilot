# GCS-controlled OP-NAS archive sender

`archive_sender.py` and `archive_lan.py` implement the automatic archive protocol
with GoranConnectServer and OP-NAS. They are independent of the manual route
sender and do not delete source recordings. The module imports hardware APIs only
when its worker runs.

The worker starts by default alongside vehicle telemetry when TeleTyped runs;
install the updated `extensions.py`, `archive_sender.py` and `archive_lan.py` and
restart TeleTyped while offroad. No `launch_env.sh` edit is required. GoranConnect
must be enabled on the device. The worker reports inventory and checks GCS for
jobs; uploading starts only after the device's GCS archive policy is enabled.

`TELETYPED_EXTENSIONS` remains an explicit override of the default workers. If a
device already sets a custom list, append
`openpilot.tools.teletyped.archive_sender:archive_step` to that list, preserving
existing entries. An empty value still disables all optional workers. Installing
this source does not change a device's environment or enable its GCS policy.

GCS selects per-device log/video categories and requires an available receiver,
offroad state, stable source files and the configured Wi-Fi/Ethernet or cellular
policy. Automatic categories default off. Files are packaged with their original
sizes, source timestamps and SHA-256 checksums; source changes abort.

GCS **Convert raw rlogs to rlog.bz2 before sending** defaults off. When enabled,
raw `rlog` data is streamed through bzip2 into `files/rlog.bz2` in the temporary
ZIP. ZIP entries are stored without further compression. Other files, including
already-compressed logs and videos, are copied unchanged. Comma source files are
never renamed or deleted. The manifest includes the original log's name, size,
timestamp and SHA-256 alongside the stored bzip2 size and SHA-256. Inventory
advertises `supports_rlog_bz2`; GCS will not assign conversion jobs to older
ZIP-only senders. OP-NAS verifies the decompressed original checksum, stores the
received bzip2 file unchanged and skips its own raw-log compression worker.

The GCS per-device **Allow direct home-network transfers** (`allow_lan`) setting
defaults off. When off or absent from job constraints, the sender performs no
local address lookup and starts no local server. Enable it only for your own
devices that use the receiver at home.

For opted-in jobs on Wi-Fi/Ethernet the sender advertises a short-lived HTTPS source on that
interface, using a fresh certificate and one-job token. OP-NAS tries it only for
a directly connected private network and verifies the certificate fingerprint
provided by authenticated GCS. This detects home reachability without trusting
an SSID or requiring an SSH login. No source directory or arbitrary file path is
served. Up to two connections are allowed, reads and TLS handshakes have timeouts,
and service/key cleanup happens when the job ends. Onroad/network transitions
stop direct serving.

A Wormhole offer is prepared at the same time. If the local endpoint is not
reachable, its identity does not match, or transfer is interrupted, the receiver
uses Wormhole for that same job. After a successful local transfer the sender
polls GCS for the verified receipt and stops the unused Wormhole process. GCS
and Wormhole rendezvous are required even for local jobs in this version.

The Pi retains protected-drive, free-space and checksum checks for both routes.
Interrupted direct downloads are discarded and restarted through Wormhole rather
than resumed. Different routed subnets, VPNs and isolated guest networks fall back.

Hardware-independent checks:

    python -m pytest -c /dev/null --confcutdir=tools/teletyped/tests tools/teletyped/tests/test_archive_lan.py

Cross-repository TLS, fallback and manifest contract tests live in
`../OP-NAS/tests/test_lan_transfer.py` and `test_gcs_integration.py`.
The prepared copies in `../OP-NAS/device/` must retain the same behavior. The
LS600h extension list keeps `openpilot.tools.teletyped.telemetry:telemetry_step`;
the updater branch uses `openpilot.tools.vehicle_telemetry.worker:telemetry_step`.

### Preparation measurements and resource limits

Each new offer can include bounded preparation measurements: time waiting for a
preparation slot, packaging wall/CPU time, source-read time, bzip2 time and input/
output bytes, ZIP-write time, and offer/transport setup time. These are monotonic
local durations, never differences between device and NAS clocks. Compression
rates divide encoded raw/output bytes by compressor time; they are per-worker
rates, not aggregate production or network capacity. Already converted logs are
reused and report no new compression work. Local source replacement stays a
separate opt-in and should be disabled for non-destructive performance trials.

A lightweight observation every ten seconds identifies workers waiting,
compressing, packaging, ready, connecting, offered or receiving. GCS versions
without this endpoint continue to work. Observations never replace inventories
or refresh their eligibility timestamps. Offroad sender work also requires a
known CPU temperature below 70 C and at least 256 MiB available memory; unknown
resource readings suppress background work. Resource checks are cached for at
most one second. Preparation-slot waits observe cancellation every 250 ms, and
packaging checks conditions between 256 KiB blocks. Existing manager offroad and
network rules still apply. The sender does not bring CPU cores online or change
OpenPilot's power management.

Level 3 and the two preparation slots remain unchanged. A same-data benchmark on
C3X in its normal offroad power mode measured 3.15 MB/s raw with one level-3
worker and 6.22 MB/s with two (2.20 MB/s compressed output, ratio 0.355, peak CPU
sensor 48.2 C). Level 1 increased network bytes by about 7%; levels 6/9 reduced
production. These short compression measurements do not establish sustained
end-to-end ingestion performance. Use OP-NAS's guarded `tools/benchmark_rlogs.py`
and verified-original-rlog throughput before changing concurrency or levels.
