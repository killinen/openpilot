from openpilot.tools.teletyped import collectors, telemetry


def test_empty_collector_registry_does_not_scan_logs(monkeypatch):
  assert collectors.enabled_collectors() == []

  def unexpected_scan(*args, **kwargs):
    raise AssertionError("telemetry scanned logs without a collector")

  monkeypatch.setattr(telemetry, "discover_drive_logs", unexpected_scan)
  telemetry.telemetry_step("device", {"X-Device-JWT": "token"})
