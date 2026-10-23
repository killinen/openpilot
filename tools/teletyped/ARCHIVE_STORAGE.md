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
policy. Automatic categories default off. Files are packaged without recompression
with their sizes, source timestamps and SHA-256 checksums; source changes abort.

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
The prepared copies in `../OP-NAS/device/` must retain the same behavior.
