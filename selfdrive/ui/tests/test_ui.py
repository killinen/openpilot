import contextlib
import os
import pathlib
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import jinja2
import numpy as np
from cereal import messaging, log
from msgq.visionipc import VisionIpcServer, VisionStreamType
from openpilot.common.mock import generate_messages_loop
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.test.process_replay.vision_meta import meta_from_camera_state
from openpilot.tools.webcam.camera import Camera

TEST_DIR = pathlib.Path(__file__).parent
REPORT_DIR = TEST_DIR / "test_ui" / "report"
SCREENSHOTS_DIR = REPORT_DIR / "screenshots"
SNAPSHOT_BIN = TEST_DIR / "ui_snapshot"


@dataclass(frozen=True)
class CaseConfig:
  started: bool
  extra_services: tuple[str, ...] = ()


CASES: dict[str, CaseConfig] = {
  "homescreen": CaseConfig(started=False),
  "settings_device": CaseConfig(started=False),
  "settings_network": CaseConfig(started=False),
  "onroad": CaseConfig(started=True),
  "onroad_map": CaseConfig(started=True, extra_services=("liveLocationKalman",)),
  "onroad_sidebar": CaseConfig(started=True),
}

@contextlib.contextmanager
def temporary_params(overrides: Dict[str, Optional[bool]]):
  params = Params()
  previous: Dict[str, Optional[bytes]] = {}
  for key, value in overrides.items():
    previous[key] = params.get(key)
    if value is None:
      params.remove(key)
    else:
      params.put_bool(key, value)
  try:
    yield
  finally:
    for key, value in previous.items():
      if value is None:
        params.remove(key)
      else:
        params.put(key, value)


class UIMockPublishers:
  def __init__(self, started: bool):
    self.started = started
    self.pm = messaging.PubMaster(["deviceState", "pandaStates", "roadCameraState", "wideRoadCameraState"])
    self.stop_event = threading.Event()
    self.threads: list[threading.Thread] = []
    self.vision_server: Optional[VisionIpcServer] = None
    self.vision_thread: Optional[threading.Thread] = None
    self.vision_streams: list[tuple] = []

  def start(self):
    self.threads.append(threading.Thread(target=self._device_loop, daemon=True))
    self.threads.append(threading.Thread(target=self._panda_loop, daemon=True))
    for thread in self.threads:
      thread.start()
    if self.started:
      self._start_vision_publishers()

  def stop(self):
    self.stop_event.set()
    for thread in self.threads:
      thread.join()
    self.threads.clear()
    if self.vision_thread:
      self.vision_thread.join()
      self.vision_thread = None
    self.vision_server = None

  def _device_loop(self):
    while not self.stop_event.wait(0.1):
      msg = messaging.new_message('deviceState')
      dat = msg.deviceState
      dat.started = self.started
      dat.networkType = log.DeviceState.NetworkType.cell4G
      dat.networkStrength = log.DeviceState.NetworkStrength.moderate
      dat.freeSpacePercent = 80
      dat.memoryUsagePercent = 2
      dat.cpuTempC = [2] * 3
      dat.gpuTempC = [2] * 3
      dat.cpuUsagePercent = [2] * 8
      self.pm.send("deviceState", msg)

  def _panda_loop(self):
    while not self.stop_event.wait(0.5):
      msg = messaging.new_message('pandaStates', 1)
      panda = msg.pandaStates[0]
      panda.ignitionLine = self.started
      panda.ignitionCan = self.started
      panda.pandaType = log.PandaState.PandaType.uno
      self.pm.send("pandaStates", msg)

  def _start_vision_publishers(self):
    camera = DEVICE_CAMERAS[("tici", "ar0231")]
    rng = np.random.default_rng(0)
    frames = {
      "roadCameraState": Camera.bgr2nv12(
        rng.integers(0, 255, size=(camera.fcam.height, camera.fcam.width, 3), dtype=np.uint8)
      ).flatten().tobytes(),
      "wideRoadCameraState": Camera.bgr2nv12(
        rng.integers(0, 255, size=(camera.fcam.height, camera.fcam.width, 3), dtype=np.uint8)
      ).flatten().tobytes(),
    }
    self.vision_server = VisionIpcServer("camerad")
    for cam_name, frame in frames.items():
      meta = meta_from_camera_state(cam_name)
      if meta is None:
        continue
      width = camera.fcam.width
      height = camera.fcam.height
      self.vision_server.create_buffers(meta.stream, 4, False, width, height)
      self.vision_streams.append((meta, cam_name, frame))
    self.vision_server.start_listener()
    self.vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
    self.vision_thread.start()

  def _vision_loop(self):
    frame_id = 0
    while not self.stop_event.is_set():
      if self.vision_server is None or not self.vision_streams:
        break
      ts = int((frame_id * DT_MDL) * 1e9)
      for meta, cam_name, frame in self.vision_streams:
        self.vision_server.send(meta.stream, frame, frame_id, ts, ts)
        msg = messaging.new_message(cam_name)
        cam_state = getattr(msg, cam_name)
        cam_state.frameId = frame_id
        cam_state.timestampSof = ts
        cam_state.timestampEof = ts
        self.pm.send(cam_name, msg)
      frame_id += 1
      self.stop_event.wait(DT_MDL)


@contextlib.contextmanager
def mocked_services(services: Iterable[str]):
  if not services:
    yield
    return

  done = threading.Event()
  thread = threading.Thread(target=generate_messages_loop, args=(list(services), done), daemon=True)
  thread.start()
  try:
    yield
  finally:
    done.set()
    thread.join()


def ensure_dirs():
  if REPORT_DIR.exists():
    shutil.rmtree(REPORT_DIR)
  SCREENSHOTS_DIR.mkdir(parents=True)


def run_snapshot(case: str, output: pathlib.Path):
  env = os.environ.copy()
  env.setdefault("QT_QPA_PLATFORM", "offscreen")
  env.setdefault("QT_OPENGL", "software")
  env.setdefault("QT_XCB_FORCE_SOFTWARE_OPENGL", "1")
  env.setdefault("SCALE", "1")
  subprocess.run(
    [str(SNAPSHOT_BIN), "-o", str(output), "--case", case],
    check=True,
    env=env,
  )


def render_case(case: str, config: CaseConfig):
  overrides = {
    "ForceOnroad": config.started,
    "ForceOffroad": not config.started,
  }
  with temporary_params(overrides):
    publishers = UIMockPublishers(config.started)
    publishers.start()
    # give messaging + VIPC threads a moment to spin up before launching the UI
    time.sleep(0.5)
    try:
      with mocked_services(config.extra_services):
        run_snapshot(case, SCREENSHOTS_DIR / f"{case}.png")
    finally:
      publishers.stop()


def build_report():
  template = (TEST_DIR / "test_ui" / "template.html").read_text()
  html = jinja2.Template(template).render(
    cases=[(case, f"screenshots/{case}.png") for case in CASES]
  )
  (REPORT_DIR / "index.html").write_text(html)


def main():
  Params().put("DongleId", "123456789012345")
  ensure_dirs()
  for case, config in CASES.items():
    render_case(case, config)
  build_report()


if __name__ == "__main__":
  main()
