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

## Requirements

1. **API token** – store your server token in Params under
   `GoranConnectPassword`. You can set this from the device UI under
   *Settings → Device → Remote Control Password* or via a shell:
   ```bash
   python3 - <<'EOF'
   from openpilot.common.params import Params
   Params().put("GoranConnectPassword", "<PASTE_TOKEN_HERE>")
   EOF
   ```
   Without a token the daemon still runs, but it will skip SSH-key uploads and
   drive transfers until the token is present.
2. **Internet access** – the helper watches `DeviceState.NetworkType` and
   sleeps while offline.

All SSH material is written under `/persist/comma/` and re-used across boots.

## Runtime Integration

The manager now launches the service as `teletyped` (see
`system/manager/process_config.py`). It only runs on devices that report the
`/TICI` marker—i.e. comma three / three X hardware—so it will not execute on
PC emulators or legacy NEOS devices. The helper auto-detects persist and log
paths using the hardware abstraction so the same binary works across both
NEOS-style and AGNOS filesystems.

## Troubleshooting

Logs live under `/data/media/0/realdata` (with uploads staged in `/tmp`) and
basic status messages are printed to stdout/stderr in the `teletyped` process
output. You can also tail `tools/teletyped/sender_log.json` to see a history of
route transfers. When debugging connectivity issues:

- Ensure `/etc/resolv.conf` is populated. The helper will bind-mount a known
  good resolver list via `tools/teletyped/setup_resolv.sh` if one is missing.
- Confirm that `ssh` and `wormhole-william` are executable (both are bundled
  with proper permissions in this repo).
- After storing the API token, the daemon will automatically retry key uploads
  every five minutes until success.
