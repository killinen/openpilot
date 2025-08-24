#!/usr/bin/env python3
import json
import os
import random
import requests
import threading
import time
import traceback
import bz2
import io
import subprocess
from pathlib import Path

from cereal import log
import cereal.messaging as messaging
from common.api import Api
from common.params import Params
from common.realtime import set_core_affinity
from system.hardware import TICI
from selfdrive.loggerd.xattr_cache import getxattr, setxattr
from selfdrive.loggerd.config import ROOT
from system.swaglog import cloudlog
from cereal.services import service_list
from tools.lib.logreader import LogReader

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.upload'
UPLOAD_ATTR_VALUE = b'1'

UPLOAD_QLOG_QCAM_MAX_SIZE = 100 * 1e6  # MB

allow_sleep = bool(os.getenv("UPLOADER_SLEEP", "1"))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


def get_directory_sort(d):
  return list(map(lambda s: s.rjust(10, '0'), d.rsplit('--', 1)))

def listdir_by_creation(d):
  try:
    paths = os.listdir(d)
    paths = sorted(paths, key=get_directory_sort)
    return paths
  except OSError:
    cloudlog.exception("listdir_by_creation failed")
    return list()

def clear_locks(root):
  for logname in os.listdir(root):
    path = os.path.join(root, logname)
    try:
      for fname in os.listdir(path):
        if fname.endswith(".lock"):
          os.unlink(os.path.join(path, fname))
    except OSError:
      cloudlog.exception("clear_locks failed")

def generate_qlog_from_rlog(rlog_path):
  rlog_path = Path(rlog_path)
  qlog_path = rlog_path.with_name("qlog")
  qlog_bz2_path = qlog_path.with_suffix(".bz2")
  qlog_lock_path = qlog_bz2_path.with_suffix(".bz2.lock")

  # 🧹 Clean up stale .lock files older than 5 min
  if qlog_lock_path.exists():
    lock_age = time.time() - qlog_lock_path.stat().st_mtime
    if lock_age > 300:
      print(f"⚠️ Stale lock file found, removing: {qlog_lock_path}")
      try:
        qlog_lock_path.unlink()
      except Exception as e:
        print(f"❌ Failed to remove lock: {e}")
        return None
    else:
      print(f"⚠️ qlog is currently locked: {qlog_lock_path}")
      return None

  # ✅ Skip if already uploaded
  try:
    if qlog_bz2_path.exists() and getxattr(str(qlog_bz2_path), "user.upload") == b"1":
      print(f"⚠️ qlog already exists and is marked uploaded: {qlog_bz2_path}")
      return str(qlog_bz2_path)
  except OSError:
    pass

  # 🧹 Delete corrupt or empty .qlog.bz2
  if qlog_bz2_path.exists():
    try:
      if os.path.getsize(qlog_bz2_path) < 100:  # 100 bytes threshold
        print(f"⚠️ Deleting corrupt/empty qlog: {qlog_bz2_path}")
        qlog_bz2_path.unlink()
      else:
        return str(qlog_bz2_path)
    except Exception as e:
      print(f"❌ Failed to stat/delete qlog: {e}")
      return None

  # Determine qlog services and their decimation
  qlog_services = {name: svc.decimation for name, svc in service_list.items() if svc.decimation is not None}
  counters = {name: 0 for name in qlog_services}

  # Read from rlog
  try:
    lr = LogReader(str(rlog_path))
  except Exception as e:
    print(f"❌ Failed to read rlog: {e}")
    return None

  out_msgs = []
  for m in lr:
    try:
      which = m.which()
    except Exception:
      continue
    if which in qlog_services:
      counters[which] += 1
      if counters[which] >= qlog_services[which]:
        out_msgs.append(m.as_builder())
        counters[which] = 0

  if not out_msgs:
    print("⚠️ No qlog messages found in rlog")
    return None

  encoded = b''.join([m.to_bytes() for m in out_msgs])
  compressed = bz2.compress(encoded)

  # Write safely
  try:
    # Lock file to prevent races with uploader
    qlog_lock_path.touch()
    with open(qlog_bz2_path, 'wb') as f:
      f.write(compressed)

    # setxattr(str(qlog_bz2_path), "user.upload", b"1")
    print(f"✅ qlog created and xattr set: {qlog_bz2_path}")
    return str(qlog_bz2_path)
  except Exception as e:
    print(f"❌ Failed to write or xattr qlog: {e}")
    return None
  finally:
    # Always remove lock
    try:
      qlog_lock_path.unlink()
    except Exception:
      pass

def generate_qcamera_from_fcamera(fcamera_path):
  fcamera_path = Path(fcamera_path)
  qcamera_path = fcamera_path.with_name("qcamera.ts")
  lock_path = qcamera_path.with_suffix(".ts.lock")

  # TICI resolution logic
  width = 526 if TICI else 480
  height = 330 if TICI else 360

  # Skip if already uploaded
  try:
    if qcamera_path.exists() and getxattr(str(qcamera_path), "user.upload") == b"1":
      print(f"⚠️ qcamera.ts already exists and is marked uploaded: {qcamera_path}")
      return str(qcamera_path)
  except OSError:
    pass

  # Stale lock cleanup
  if lock_path.exists():
    try:
      age = time.time() - lock_path.stat().st_mtime
      if age > 300:
        print(f"⚠️ Removing stale lock: {lock_path}")
        lock_path.unlink()
      else:
        print(f"⚠️ qcamera is locked: {lock_path}")
        return None
    except Exception as e:
      print(f"❌ Failed to check/remove lock: {e}")
      return None

  # Skip corrupt/incomplete files
  if qcamera_path.exists() and qcamera_path.stat().st_size < 100_000:
    print(f"⚠️ Deleting corrupt/incomplete qcamera.ts: {qcamera_path}")
    try:
      qcamera_path.unlink()
    except Exception:
      pass

  print(f"🎥 Generating qcamera.ts from {fcamera_path}")
  try:
    lock_path.touch()
    cmd = [
      "ffmpeg",
      "-y",                           # overwrite output
      "-loglevel", "error",          # quiet logs
      "-framerate", "20",            # treat input as 20 fps (for raw .hevc)
      "-i", str(fcamera_path),       # input file
      "-an",                          # strip audio
      "-vf", f"scale={width}:{height},fps=20",  # resize and fix output fps
      "-frames:v", "1200",           # exactly 60s at 20 fps
      "-c:v", "libx264",             # use H.264 codec
      "-preset", "ultrafast",        # fast encode
      "-b:v", "256k",                # target bitrate
      "-f", "mpegts",                # output as MPEG-TS
      str(qcamera_path)
    ]
    subprocess.run(cmd, check=True)

    # setxattr(str(qcamera_path), "user.upload", b"1")
    print(f"✅ qcamera.ts created and xattr set: {qcamera_path}")
    return str(qcamera_path)
  except subprocess.CalledProcessError as e:
    print(f"❌ ffmpeg failed: {e}")
  except Exception as e:
    print(f"❌ General failure: {e}")
  finally:
    try:
      lock_path.unlink()
    except Exception:
      pass

  return None


class Uploader():
  def __init__(self, dongle_id, root):
    self.dongle_id = dongle_id
    self.api = Api(dongle_id)
    self.root = root

    self.upload_thread = None

    self.last_resp = None
    self.last_exc = None

    self.immediate_size = 0
    self.immediate_count = 0

    # stats for last successfully uploaded file
    self.last_time = 0.0
    self.last_speed = 0.0
    self.last_filename = ""

    self.immediate_folders = ["crash/", "boot/"]
    # Fix for new type of logging
    # self.immediate_priority = {"qlog": 0, "qlog.bz2": 0, "qcamera.ts": 1}
    self.immediate_priority = {"rlog.bz2": 0, "rlog": 0, "qlog.bz2": 1, "fcamera.hevc": 1, "qcamera.ts": 1}
    # self.immediate_priority = {"rlog.bz2": 0, "qlog.bz2": 1, "fcamera.hevc": 0, "qcamera.ts": 1}

  def get_upload_sort(self, name):
    if name in self.immediate_priority:
      return self.immediate_priority[name]
    return 1000

  def list_upload_files(self):
    if not os.path.isdir(self.root):
      return

    self.immediate_size = 0
    self.immediate_count = 0

    for logname in listdir_by_creation(self.root):
      path = os.path.join(self.root, logname)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=self.get_upload_sort):
        key = os.path.join(logname, name)
        fn = os.path.join(path, name)
        # skip files already uploaded
        try:
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME)
        except OSError:
          cloudlog.event("uploader_getxattr_failed", exc=self.last_exc, key=key, fn=fn)
          is_uploaded = True  # deleter could have deleted
        if is_uploaded:
          continue

        try:
          if name in self.immediate_priority:
            self.immediate_count += 1
            self.immediate_size += os.path.getsize(fn)
        except OSError:
          pass

        yield (name, key, fn)

  def next_file_to_upload(self):
    upload_files = list(self.list_upload_files())

    for name, key, fn in upload_files:
      if any(f in fn for f in self.immediate_folders):
        return (name, key, fn)

    for name, key, fn in upload_files:
      if name in self.immediate_priority:
        return (name, key, fn)

    return None

  def do_upload(self, key, fn):
    try:
      # url_resp = self.api.get("v1.4/" + self.dongle_id + "/upload_url/", timeout=10, path=key, access_token=self.api.get_token())
      url_resp = self.api.get("v1.3/" + self.dongle_id + "/upload_url/", timeout=10, path=key, access_token=self.api.get_token())
      if url_resp.status_code == 412:
        self.last_resp = url_resp
        return

      url_resp_json = json.loads(url_resp.text)
      url = url_resp_json['url']
      headers = url_resp_json['headers']
      cloudlog.debug("upload_url v1.4 %s %s", url, str(headers))

      if fake_upload:
        cloudlog.debug(f"*** WARNING, THIS IS A FAKE UPLOAD TO {url} ***")

        class FakeResponse():
          def __init__(self):
            self.status_code = 200

        self.last_resp = FakeResponse()
      else:
        with open(fn, "rb") as f:
          if key.endswith('.bz2') and not fn.endswith('.bz2'):
            data = bz2.compress(f.read())
            data = io.BytesIO(data)
            print(f"Compressing {fn} before uploading as {key}")
          else:
            data = f

          self.last_resp = requests.put(url, data=data, headers=headers, timeout=10)
    except Exception as e:
      self.last_exc = (e, traceback.format_exc())
      raise

  def normal_upload(self, key, fn):
    self.last_resp = None
    self.last_exc = None

    try:
      self.do_upload(key, fn)
    except Exception:
      pass

    return self.last_resp

  def upload(self, name, key, fn, network_type, metered):
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("upload: getsize failed")
      return False

    cloudlog.event("upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
    print("upload()")

    # Only use .bz2 in key (upload path), not in file path
    upload_key = key
    # Only append .bz2 ONCE and ONLY if fn isn't already compressed
    if (upload_key.endswith('qlog') or upload_key.endswith('rlog') or upload_key.startswith('boot/')) and not key.endswith('.bz2') and not fn.endswith('.bz2'):
      upload_key += ".bz2"
      print(f'upload_key is {upload_key}')

    if sz == 0:
      success = True  # skip upload, mark as uploaded
      print('skip upload')
    elif name in self.immediate_priority and sz > UPLOAD_QLOG_QCAM_MAX_SIZE:
      cloudlog.event("uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
      print('uploader_too_large')
    else:
      start_time = time.monotonic()
      stat = self.normal_upload(upload_key, fn)  # Upload using compressed key, actual file
      if stat is not None and stat.status_code in (200, 201, 401, 403, 412):
        self.last_filename = fn
        self.last_time = time.monotonic() - start_time
        self.last_speed = (sz / 1e6) / self.last_time
        success = True
        cloudlog.event("upload_success" if stat.status_code != 412 else "upload_ignored",
                       key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        print('upload_success')
      else:
        success = False
        cloudlog.event("upload_failed", stat=stat, exc=self.last_exc,
                       key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        # print(f'upload_failed stat={stat}, exc={self.last_exc}, key={key}, fn={fn}, sz={sz}, network_type={network_type}, metered={metered}')

    if success:
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
        print(f'setxattr to {fn} {UPLOAD_ATTR_NAME} {UPLOAD_ATTR_VALUE.decode()} ')
      except OSError as e:
        cloudlog.event("uploader_setxattr_failed", exc=str(e), key=key, fn=fn, sz=sz)
        print(f'uploader_setxattr_failed {fn} {key} {sz} ')

    return success


  def get_msg(self):
    msg = messaging.new_message("uploaderState")
    us = msg.uploaderState
    us.immediateQueueSize = int(self.immediate_size / 1e6)
    us.immediateQueueCount = self.immediate_count
    us.lastTime = self.last_time
    us.lastSpeed = self.last_speed
    us.lastFilename = self.last_filename
    return msg


def uploader_fn(exit_event):
  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  clear_locks(ROOT)

  params = Params()
  dongle_id = params.get("DongleId", encoding='utf8')

  if dongle_id is None:
    cloudlog.info("uploader missing dongle_id")
    raise Exception("uploader can't start without dongle id")

  if TICI and not Path("/data/media").is_mount():
    cloudlog.warning("NVME not mounted")

  sm = messaging.SubMaster(['deviceState'])
  pm = messaging.PubMaster(['uploaderState'])
  uploader = Uploader(dongle_id, ROOT)

  backoff = 0.1
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool("IsOffroad")
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    d = uploader.next_file_to_upload()

    if d is None:  # Nothing to upload
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    name, key, fn = d

    # If rlog is raw (not compressed), compress and generate qlog
    if name == "rlog" and not fn.endswith(".bz2"):
      rlog_bz2_path = fn + ".bz2"

      # Check if already uploaded
      try:
        already_uploaded = os.path.isfile(rlog_bz2_path) and getxattr(rlog_bz2_path, "user.upload") == b"1"
      except OSError:
        already_uploaded = False

      if not already_uploaded:
        try:
          with open(fn, "rb") as f:
            data = bz2.compress(f.read())
          with open(rlog_bz2_path, "wb") as fz:
            fz.write(data)
          print(f"✅ Compressed rlog to {rlog_bz2_path}")
          fn = rlog_bz2_path
          key += ".bz2"
        except Exception as e:
          print(f"❌ Failed to compress rlog: {e}")

        # Now generate qlog.bz2 from rlog.bz2, this is for the newer logger to be combatible with old retropilot-server
        try:
          qlog_path = generate_qlog_from_rlog(rlog_bz2_path)
          if qlog_path:
            print(f"✅ qlog handled at: {qlog_path}")
        except Exception as e:
          print(f"❌ Failed to generate qlog: {e}")

    # Do same generation to of qcamera.ts from fcamera.hevc to make retropilot-server happy
    if name == "fcamera.hevc":
      try:
        qcamera_path = generate_qcamera_from_fcamera(fn)
        if qcamera_path:
          print(f"✅ qcamera.ts handled at: {qcamera_path}")
      except Exception as e:
        print(f"❌ Failed to generate qcamera.ts: {e}")

    # Preserve original logic for upload key bz2 suffix
    if (key.endswith(('qlog', 'rlog')) or key.startswith('boot/')) and not key.endswith('.bz2'):
      key += ".bz2"

    success = uploader.upload(name, key, fn, sm['deviceState'].networkType.raw, sm['deviceState'].networkMetered)
    if success:
      backoff = 0.1
    elif allow_sleep:
      cloudlog.info("upload backoff %r", backoff)
      time.sleep(backoff + random.uniform(0, backoff))
      backoff = min(backoff*2, 120)

    pm.send("uploaderState", uploader.get_msg())


def main():
  uploader_fn(threading.Event())


if __name__ == "__main__":
  main()