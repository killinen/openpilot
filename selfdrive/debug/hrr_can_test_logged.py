#!/usr/bin/env python3
"""Interactive Panda HRR CAN test with synchronized openpilot route logging.

This is the logged counterpart to hrr_can_test.py. It records the road camera
to fcamera.hevc and republishes Panda RX/TX as cereal can/sendcan messages so
loggerd includes them in the route's rlog and (with normal decimation) qlog.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

from openpilot.selfdrive.debug import hrr_can_test as hrr

if TYPE_CHECKING:
  from panda import Panda


LOGGING_PROCESSES = ("loggerd", "encoderd", "camerad")
PROCESS_PATHS = {
  "loggerd": ("system/loggerd", "./loggerd"),
  "encoderd": ("system/loggerd", "./encoderd"),
  "camerad": ("system/camerad", "./camerad"),
}
LOGGING_START_TIMEOUT_S = 20.0


def process_running(name: str) -> bool:
  """Return whether an exact Linux process name is currently running."""
  for proc_dir in Path("/proc").glob("[0-9]*"):
    try:
      if (proc_dir / "comm").read_text().strip() == name:
        return True
    except (FileNotFoundError, PermissionError, ProcessLookupError):
      pass
  return False


class LoggingSession:
  def __init__(self, start_timeout_s: float = LOGGING_START_TIMEOUT_S) -> None:
    # Keep openpilot/Cython imports out of module initialization so --self-test
    # retains the lightweight behavior of the original script.
    import cereal.messaging as messaging
    from openpilot.common.params import Params
    from openpilot.selfdrive.pandad import can_list_to_can_capnp
    from openpilot.system.hardware.hw import Paths

    self.params = Params()
    self.can_list_to_can_capnp: Callable[..., bytes] = can_list_to_can_capnp
    self.repo_root = Path(__file__).resolve().parents[2]
    self.log_root = Path(Paths.log_root())
    self.start_timeout_s = start_timeout_s
    self.rx_pm = messaging.PubMaster(["can"])
    self.tx_pm = messaging.PubMaster(["sendcan"])
    self.started_processes: dict[str, subprocess.Popen] = {}
    self.reused_processes: list[str] = []
    self.route_name: str | None = None
    self.previous_route_name: str | None = None
    self.segment_paths: list[Path] = []
    self.tx_lock = threading.Lock()

  def _start_processes(self) -> None:
    for name in LOGGING_PROCESSES:
      if process_running(name):
        self.reused_processes.append(name)
        continue

      relative_cwd, command = PROCESS_PATHS[name]
      cwd = self.repo_root / relative_cwd
      executable = cwd / command
      if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError(f"{name} is not built: expected {executable}")

      env = dict(os.environ, MANAGER_DAEMON=name)
      proc = subprocess.Popen([command], cwd=cwd, env=env, start_new_session=True)
      self.started_processes[name] = proc
      time.sleep(0.25)
      if proc.poll() is not None:
        raise RuntimeError(f"{name} failed to start; check its output and confirm this is running on a comma device")

  def _wait_for_logger(self) -> None:
    deadline = time.monotonic() + self.start_timeout_s
    while time.monotonic() < deadline:
      can_ready = self.rx_pm.all_readers_updated("can")
      sendcan_ready = self.tx_pm.all_readers_updated("sendcan")
      if can_ready and sendcan_ready:
        return
      time.sleep(0.05)
    raise RuntimeError("loggerd did not subscribe to can/sendcan before the startup timeout")

  def _read_current_route(self) -> str | None:
    value = self.params.get("CurrentRoute", encoding="utf-8")
    return value if value else None

  def _wait_for_route(self) -> None:
    deadline = time.monotonic() + self.start_timeout_s
    loggerd_started = "loggerd" in self.started_processes
    while time.monotonic() < deadline:
      route_name = self._read_current_route()
      is_new_route = not loggerd_started or route_name != self.previous_route_name
      if route_name is not None and is_new_route and list(self.log_root.glob(f"{route_name}--*")):
        self.route_name = route_name
        return
      time.sleep(0.05)
    raise RuntimeError(f"loggerd did not create a route under {self.log_root}")

  def _wait_for_fcamera(self) -> None:
    assert self.route_name is not None
    previous_sizes = {
      path: path.stat().st_size
      for path in self.log_root.glob(f"{self.route_name}--*/fcamera.hevc")
    }
    deadline = time.monotonic() + self.start_timeout_s
    while time.monotonic() < deadline:
      self.segment_paths = sorted(self.log_root.glob(f"{self.route_name}--*"))
      active_segments = [segment for segment in self.segment_paths if (segment / "rlog.lock").is_file()]
      for segment in active_segments:
        fcamera = segment / "fcamera.hevc"
        if fcamera.is_file() and fcamera.stat().st_size > previous_sizes.get(fcamera, 0):
          return
      time.sleep(0.1)
    raise RuntimeError("camera logging did not create fcamera.hevc before the startup timeout")

  def start(self) -> None:
    try:
      self.previous_route_name = self._read_current_route()
      self._start_processes()
      self._wait_for_logger()
      self._wait_for_route()
      self._wait_for_fcamera()
    except BaseException:
      self.stop()
      raise

    process_text = []
    if self.started_processes:
      process_text.append(f"started={','.join(self.started_processes)}")
    if self.reused_processes:
      process_text.append(f"reused={','.join(self.reused_processes)}")
    print(f"Logging route {self.route_name} ({' '.join(process_text)}).")
    print(f"Route files: {self.log_root / f'{self.route_name}--*'}")

  def publish_rx(self, frames: list[tuple[int, int, bytes, int]]) -> None:
    if frames:
      self.rx_pm.send("can", self.can_list_to_can_capnp(frames))

  def publish_tx(self, address: int, payload: bytes, bus: int) -> None:
    # send() can be called by both the streaming and interactive threads.
    with self.tx_lock:
      frame = [(address, 0, payload, bus)]
      self.tx_pm.send("sendcan", self.can_list_to_can_capnp(frame, msgtype="sendcan"))

  def stop(self) -> None:
    # Stop camera production first, flush encoderd next, and close rlog/qlog
    # last. Never stop a process that was already running when this tool began.
    for name in ("camerad", "encoderd", "loggerd"):
      proc = self.started_processes.get(name)
      if proc is None or proc.poll() is not None:
        continue
      try:
        proc.send_signal(signal.SIGINT)
      except ProcessLookupError:
        proc.wait()
        continue
      try:
        proc.wait(timeout=5.0)
      except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    self.started_processes.clear()

    if self.route_name is not None:
      self.segment_paths = sorted(self.log_root.glob(f"{self.route_name}--*"))

  def print_summary(self) -> None:
    if self.route_name is None:
      return
    print(f"Recorded route: {self.route_name}")
    for segment in self.segment_paths:
      files = sorted(path.name for path in segment.iterdir() if path.is_file())
      print(f"  {segment}: {', '.join(files)}")
    print("Use rlog for full-rate CAN analysis; qlog contains the repository's normal decimated CAN subset.")


class LoggedHrrCanTest(hrr.HrrCanTest):
  def __init__(self, panda: Panda | None, bus: int, rate_hz: float, dry_run: bool,
               logging_session: LoggingSession) -> None:
    super().__init__(panda, bus, rate_hz, dry_run)
    self.logging_session = logging_session
    self.panda_send_lock = threading.Lock()

  def send(self, address: int, payload: bytes) -> None:
    if self.dry_run:
      return
    # Serialize direct Panda access with sendcan publication so configuration
    # commands cannot interleave with the 100 Hz streaming thread.
    with self.panda_send_lock:
      super().send(address, payload)
      self.logging_session.publish_tx(address, payload, self.bus)

  def monitor_loop(self) -> None:
    assert self.panda is not None
    next_display = time.monotonic()
    while not self.stop_event.is_set():
      frames = self.panda.can_recv()
      self.logging_session.publish_rx(frames)
      for address, _, payload, rx_bus in frames:
        if rx_bus == self.bus:
          self.device_status.update(address, payload)
        if rx_bus == hrr.STEER_TORQUE_SENSOR_BUS and address == hrr.STEER_TORQUE_SENSOR_ADDR:
          self.steering_torque_status.update(payload)

      now = time.monotonic()
      if now >= next_display:
        self.status_display.update(self.format_status())
        next_display = now + 0.1
      self.stop_event.wait(0.01)


def run_self_test() -> None:
  hrr.run_self_test()

  class FakePanda:
    def __init__(self) -> None:
      self.sent: list[tuple[int, bytes, int]] = []

    def can_send(self, address: int, payload: bytes, bus: int) -> None:
      self.sent.append((address, payload, bus))

  class FakeLoggingSession:
    def __init__(self) -> None:
      self.sent: list[tuple[int, bytes, int]] = []

    def publish_tx(self, address: int, payload: bytes, bus: int) -> None:
      self.sent.append((address, payload, bus))

  panda = FakePanda()
  logging_session = FakeLoggingSession()
  test = LoggedHrrCanTest(panda, 1, hrr.DEFAULT_RATE_HZ, False, logging_session)  # type: ignore[arg-type]
  payload = hrr.build_torque_frame(100, True, 3)
  test.send(hrr.TORQUE_ADDR, payload)
  expected = [(hrr.TORQUE_ADDR, payload, 1)]
  assert panda.sent == expected
  assert logging_session.sent == expected
  print("Logged TX self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), help="Panda CAN bus; prompted when omitted")
  parser.add_argument("--rate-hz", type=float, default=hrr.DEFAULT_RATE_HZ, help="torque and brake streaming rate")
  parser.add_argument("--force-harness-relay", action="store_true",
                      help="force the Panda harness relay and disable firmware forwarding while the tool runs")
  parser.add_argument("--dry-run", action="store_true",
                      help="exercise controls and camera logging without opening or transmitting through Panda")
  parser.add_argument("--self-test", action="store_true", help="verify known frame encodings and exit")
  parser.add_argument("--logging-start-timeout", type=float, default=LOGGING_START_TIMEOUT_S,
                      help="seconds to wait for loggerd and fcamera startup")
  args = parser.parse_args()

  if args.self_test:
    run_self_test()
    return
  if args.rate_hz <= 0:
    parser.error("--rate-hz must be greater than zero")
  if args.logging_start_timeout <= 0:
    parser.error("--logging-start-timeout must be greater than zero")

  bus = hrr.choose_bus(args.bus)
  panda = None
  logging_session: LoggingSession | None = None
  test: LoggedHrrCanTest | None = None

  try:
    if not args.dry_run:
      from panda import Panda
      panda = Panda()
      panda.set_power_save(False)
      panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

    logging_session = LoggingSession(args.logging_start_timeout)
    logging_session.start()
    test = LoggedHrrCanTest(panda, bus, args.rate_hz, args.dry_run, logging_session)

    if args.force_harness_relay:
      test.set_force_harness_relay(True)
    print(f"Streaming 0x{hrr.TORQUE_ADDR:03X} and 0x{hrr.BRAKE_ADDR:03X} on Panda bus {bus} at {args.rate_hz:g} Hz.")
    print("Initial state is disengaged, zero torque, and brake released.")
    test.start()
    hrr.run_interactive(test)
  except (KeyboardInterrupt, EOFError):
    print()
  finally:
    try:
      try:
        if test is not None:
          print("Safe shutdown: torque=0, relays off, BRAKE_PRESSED=1.")
          test.safe_shutdown()
      finally:
        try:
          if panda is not None:
            panda.set_safety_mode(Panda.SAFETY_SILENT)
        finally:
          if test is not None:
            test.restore_harness_relay()
    finally:
      if logging_session is not None:
        logging_session.stop()
        logging_session.print_summary()


if __name__ == "__main__":
  main()
