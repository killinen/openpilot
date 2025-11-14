from collections import namedtuple
import pathlib
import shutil
import sys
import jinja2
import matplotlib.pyplot as plt
import numpy as np
import os
import time
import subprocess
import atexit

from cereal import messaging, car, log
from msgq.visionipc import VisionIpcServer, VisionStreamType

from cereal.messaging import SubMaster, PubMaster
from openpilot.common.mock import mock_messages
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.test.helpers import with_processes
from openpilot.selfdrive.test.process_replay.vision_meta import meta_from_camera_state
from openpilot.tools.webcam.camera import Camera

UI_DELAY = 0.5 # may be slower on CI?

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength

EventName = car.CarEvent.EventName
EVENTS_BY_NAME = {v: k for k, v in EventName.schema.enumerants.items()}


def setup_common(click, pm: PubMaster):
  Params().put("DongleId", "123456789012345")
  dat = messaging.new_message('deviceState')
  dat.deviceState.started = True
  dat.deviceState.networkType = NetworkType.cell4G
  dat.deviceState.networkStrength = NetworkStrength.moderate
  dat.deviceState.freeSpacePercent = 80
  dat.deviceState.memoryUsagePercent = 2
  dat.deviceState.cpuTempC = [2,]*3
  dat.deviceState.gpuTempC = [2,]*3
  dat.deviceState.cpuUsagePercent = [2,]*8

  pm.send("deviceState", dat)

def setup_homescreen(click, pm: PubMaster):
  setup_common(click, pm)

def setup_settings_device(click, pm: PubMaster):
  setup_common(click, pm)

  click(100, 100)

def setup_settings_network(click, pm: PubMaster):
  setup_common(click, pm)

  setup_settings_device(click, pm)
  click(300, 600)

def setup_onroad(click, pm: PubMaster):
  setup_common(click, pm)

  dat = messaging.new_message('pandaStates', 1)
  dat.pandaStates[0].ignitionLine = True
  dat.pandaStates[0].pandaType = log.PandaState.PandaType.uno

  pm.send("pandaStates", dat)

  d = DEVICE_CAMERAS[("tici", "ar0231")]
  server = VisionIpcServer("camerad")
  server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 40, False, d.fcam.width, d.fcam.height)
  server.create_buffers(VisionStreamType.VISION_STREAM_DRIVER, 40, False, d.dcam.width, d.dcam.height)
  server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 40, False, d.fcam.width, d.fcam.height)
  server.start_listener()

  time.sleep(0.5) # give time for vipc server to start

  IMG = Camera.bgr2nv12(np.random.randint(0, 255, (d.fcam.width, d.fcam.height, 3), dtype=np.uint8))
  IMG_BYTES = IMG.flatten().tobytes()

  cams = ('roadCameraState', 'wideRoadCameraState')

  frame_id = 0
  for cam in cams:
    msg = messaging.new_message(cam)
    cs = getattr(msg, cam)
    cs.frameId = frame_id
    cs.timestampSof = int((frame_id * DT_MDL) * 1e9)
    cs.timestampEof = int((frame_id * DT_MDL) * 1e9)
    cam_meta = meta_from_camera_state(cam)

    pm.send(msg.which(), msg)
    server.send(cam_meta.stream, IMG_BYTES, cs.frameId, cs.timestampSof, cs.timestampEof)

@mock_messages(['liveLocationKalman'])
def setup_onroad_map(click, pm: PubMaster):
  setup_onroad(click, pm)

  click(500, 500)

  time.sleep(UI_DELAY) # give time for the map to render

def setup_onroad_sidebar(click, pm: PubMaster):
  setup_onroad_map(click, pm)
  click(500, 500)

WindowInfo = namedtuple("WindowInfo", ["title", "left", "top", "width", "height"])


def scan_x11_windows(min_width=1, min_height=1):
  try:
    from Xlib import display as xdisplay
  except Exception as e:
    print(f"x11 scan unavailable: {e}")
    return []

  try:
    disp = xdisplay.Display()
    root = disp.screen().root
  except Exception as e:
    print(f"x11 scan failed to connect to display: {e}")
    return []

  windows: list[WindowInfo] = []
  stack = [root]
  seen = set()
  while stack:
    win = stack.pop()
    if win.id in seen:
      continue
    seen.add(win.id)
    try:
      children = win.query_tree().children
      stack.extend(children)
    except Exception:
      continue
    if win == root:
      continue
    try:
      geom = win.get_geometry()
      _, left, top = win.translate_coords(root, 0, 0)
      name = win.get_wm_name()
    except Exception:
      continue
    if geom.width >= min_width and geom.height >= min_height:
      windows.append(WindowInfo(name or "", left, top, geom.width, geom.height))
  return windows


CASES = {
  "homescreen": setup_homescreen,
  "settings_device": setup_settings_device,
  "settings_network": setup_settings_network,
  "onroad": setup_onroad,
  "onroad_map": setup_onroad_map,
  "onroad_sidebar": setup_onroad_sidebar
}

TEST_DIR = pathlib.Path(__file__).parent

TEST_OUTPUT_DIR = TEST_DIR / "report"
SCREENSHOTS_DIR = TEST_OUTPUT_DIR / "screenshots"
DISPLAY_NUM = 99
_xvfb_proc = None
_openbox_proc = None


def start_virtual_display():
  global _xvfb_proc, _openbox_proc
  if _xvfb_proc is not None:
    return

  xvfb_bin = shutil.which("Xvfb")
  if xvfb_bin is None:
    raise RuntimeError("Xvfb not found in PATH")

  env_display = f":{DISPLAY_NUM}"
  _xvfb_proc = subprocess.Popen(
    [xvfb_bin, env_display, "-screen", "0", "2160x1080x24", "-ac", "-nolisten", "tcp"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  sock_path = pathlib.Path(f"/tmp/.X11-unix/X{DISPLAY_NUM}")
  for _ in range(100):
    if sock_path.exists():
      break
    time.sleep(0.1)
  else:
    raise RuntimeError("Xvfb failed to start")

  os.environ["DISPLAY"] = env_display
  os.environ["XAUTHORITY"] = ""
  os.environ["XDG_RUNTIME_DIR"] = f"/tmp/runtime-{os.getuid()}"
  pathlib.Path(os.environ["XDG_RUNTIME_DIR"]).mkdir(mode=0o700, exist_ok=True)

  openbox_bin = shutil.which("openbox")
  if openbox_bin is not None:
    _openbox_proc = subprocess.Popen([openbox_bin], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  xvfb = _xvfb_proc
  openbox = _openbox_proc

  def _cleanup():
    if openbox and openbox.poll() is None:
      openbox.terminate()
    if xvfb and xvfb.poll() is None:
      xvfb.terminate()

  atexit.register(_cleanup)


start_virtual_display()

import pywinctl


class TestUI:
  def __init__(self):
    os.environ["SCALE"] = "1"
    os.environ["QT_OPENGL"] = "software"
    os.environ["QT_XCB_FORCE_SOFTWARE_OPENGL"] = "1"
    os.environ["QSG_RHI_BACKEND"] = "software"
    os.environ["QT_QUICK_BACKEND"] = "software"
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    os.environ["QT_DEBUG_PLUGINS"] = "1"
    sys.modules["mouseinfo"] = False

  def setup(self):
    from openpilot.system.manager.process_config import managed_processes
    self.sm = SubMaster(["uiDebug"])
    self.pm = PubMaster(["deviceState", "pandaStates", "controlsState", 'roadCameraState', 'wideRoadCameraState', 'liveLocationKalman'])
    print(f"[debug] DISPLAY={os.environ.get('DISPLAY')} QT_QPA_PLATFORM={os.environ.get('QT_QPA_PLATFORM')}")
    while not self.sm.valid["uiDebug"]:
      self.sm.update(1)
    time.sleep(UI_DELAY) # wait a bit more for the UI to start rendering
    ui_proc = managed_processes.get("ui")
    if ui_proc and ui_proc.proc is not None:
      print(f"[debug] ui process pid={ui_proc.proc.pid} exitcode={ui_proc.proc.exitcode}")
      try:
        import subprocess
        subprocess.run(["ps", "-p", str(ui_proc.proc.pid), "-o", "pid,cmd"], check=False)
        with open(f"/proc/{ui_proc.proc.pid}/environ", "rb") as f:
          env_bytes = f.read().split(b'\x00')
          env_map = {kv.split(b"=", 1)[0]: kv.split(b"=", 1)[1] for kv in env_bytes if b"=" in kv}
          for key in [b"DISPLAY", b"QT_QPA_PLATFORM"]:
            if key in env_map:
              print(f"[debug] ui env {key.decode()}={env_map[key].decode(errors='ignore')}")
      except Exception as e:
        print(f"[debug] ps failed: {e}")
    self.ui = None
    find_start = time.monotonic()
    deadline = find_start + 10.0
    while time.monotonic() < deadline:
      try:
        wins = pywinctl.getWindowsWithTitle("ui")
        if wins:
          self.ui = wins[0]
          print(f"pywinctl: found 'ui' window by title in {time.monotonic() - find_start:.2f}s "
                f"at ({self.ui.left}, {self.ui.top}) size {self.ui.width}x{self.ui.height}")
          break
      except Exception as e:
        print(f"pywinctl error while searching for ui window: {e}")
      time.sleep(0.2)

    if self.ui is None:
      try:
        wins = pywinctl.getAllWindows()
        print(f"pywinctl fallback scan: found {len(wins)} windows")
        candidates = [w for w in wins if w.width >= 1000 and w.height >= 500]
        candidates.sort(key=lambda w: w.width * w.height, reverse=True)
        if candidates:
          self.ui = candidates[0]
          print(f"pywinctl: selected largest window "
                f"('{self.ui.title}') at ({self.ui.left}, {self.ui.top}) size {self.ui.width}x{self.ui.height}")
      except Exception as e:
        print(f"pywinctl error while scanning all windows: {e}")

    if self.ui is None:
      try:
        titles = pywinctl.getAllTitles()
        print(f"pywinctl could not find the ui window. Available titles: {titles}")
      except Exception as e:
        print(f"pywinctl error fetching titles: {e}")

      x11_windows = scan_x11_windows(min_width=100, min_height=100)
      print(f"x11 scan: found {len(x11_windows)} candidate windows")
      for win in x11_windows:
        print(f"  - '{win.title}' at ({win.left}, {win.top}) size {win.width}x{win.height}")
      if x11_windows:
        best = max(x11_windows, key=lambda w: w.width * w.height)
        print(f"x11 scan: using '{best.title}' at ({best.left}, {best.top}) size {best.width}x{best.height}")
        self.ui = best

    if self.ui is None:
      print("failed to find ui window, assuming that it's in the top left (for Xvfb)")
      self.ui = WindowInfo("fallback", 0, 0, 2160, 1080)
    else:
      print(f"Using window '{getattr(self.ui, 'title', 'unknown')}' at "
            f"({self.ui.left}, {self.ui.top}) size {self.ui.width}x{self.ui.height}")

  def screenshot(self):
    import pyautogui
    im = pyautogui.screenshot(region=(self.ui.left, self.ui.top, self.ui.width, self.ui.height))
    assert im.width == 2160
    assert im.height == 1080
    img = np.array(im)
    print(f"screenshot stats: mean={img.mean():.2f} min={img.min()} max={img.max()} std={img.std():.2f}")
    im.close()
    return img

  def click(self, x, y, *args, **kwargs):
    import pyautogui
    pyautogui.click(self.ui.left + x, self.ui.top + y, *args, **kwargs)
    time.sleep(UI_DELAY) # give enough time for the UI to react

  @with_processes(["ui"])
  def test_ui(self, name, setup_case):
    self.setup()

    setup_case(self.click, self.pm)

    time.sleep(UI_DELAY) # wait a bit more for the UI to finish rendering

    im = self.screenshot()
    plt.imsave(SCREENSHOTS_DIR / f"{name}.png", im)


def create_html_report():
  OUTPUT_FILE = TEST_OUTPUT_DIR / "index.html"

  with open(TEST_DIR / "template.html") as f:
    template = jinja2.Template(f.read())

  cases = {f.stem: (str(f.relative_to(TEST_OUTPUT_DIR)), "reference.png") for f in SCREENSHOTS_DIR.glob("*.png")}
  cases = dict(sorted(cases.items()))

  with open(OUTPUT_FILE, "w") as f:
    f.write(template.render(cases=cases))

def create_screenshots():
  if TEST_OUTPUT_DIR.exists():
    shutil.rmtree(TEST_OUTPUT_DIR)

  SCREENSHOTS_DIR.mkdir(parents=True)

  t = TestUI()
  for name, setup in CASES.items():
    t.test_ui(name, setup)

if __name__ == "__main__":
  print("creating test screenshots")
  create_screenshots()

  print("creating html report")
  create_html_report()
