"""Optional authenticated offroad jobs, isolated from core teletyped workers."""

from importlib import import_module
import os
from collections.abc import Callable

from openpilot.tools.teletyped.helper import capture_exception, log

# The archive worker checks GCS policy; registering it does not enable uploads.
DEFAULT_EXTENSIONS = "openpilot.tools.vehicle_telemetry.worker:telemetry_step,openpilot.tools.teletyped.archive_sender:archive_step"


def optional_workers(auth_headers: Callable[[], dict[str, str]]) -> list[tuple[Callable, str, str]]:
  workers = []
  # Empty disables all extensions. Each entry is an importable module:callable.
  for index, entry in enumerate(os.getenv("TELETYPED_EXTENSIONS", DEFAULT_EXTENSIONS).split(",")):
    entry = entry.strip()
    if not entry:
      continue
    try:
      module, attribute = entry.rsplit(":", 1)
      step = getattr(import_module(module), attribute)
      if not callable(step):
        raise TypeError(f"Extension is not callable: {entry}")

      def run(device_id: str, callback: Callable = step) -> None:
        headers = auth_headers()
        if headers:
          callback(device_id, headers)

      workers.append((run, f"Optional job {entry}", f"teletyped_extension_{index}"))
    except Exception as error:
      capture_exception(error)
      log(f"Optional job unavailable ({entry}): {error}", "WARN")
  return workers
