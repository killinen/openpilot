#!/usr/bin/env python3
import os
import time
import threading
import unittest
import logging
import json

from system.swaglog import cloudlog
import selfdrive.loggerd.uploader as uploader

from common.xattr import getxattr

from selfdrive.loggerd.tests.loggerd_tests_common import UploaderTestCase


class TestLogHandler(logging.Handler):
  def __init__(self):
    logging.Handler.__init__(self)
    self.reset()

  def reset(self):
    self.upload_order = list()
    self.upload_ignored = list()

  def emit(self, record):
    try:
      j = json.loads(record.getMessage())
      if j["event"] == "upload_success":
        self.upload_order.append(j["key"])
      if j["event"] == "upload_ignored":
        self.upload_ignored.append(j["key"])
    except Exception:
      pass

log_handler = TestLogHandler()
cloudlog.addHandler(log_handler)


class TestUploader(UploaderTestCase):
  def setUp(self):
    super().setUp()
    log_handler.reset()

  @staticmethod
  def _canonical_key(event):
    if event.startswith("boot/") and event.endswith(".bz2"):
      return event[:-4]
    if event.endswith("/rlog.bz2") or event.endswith("/qlog.bz2"):
      return event[:-4]
    return event

  @classmethod
  def _runtime_expected(cls, required, optional, observed):
    required_canonical = [cls._canonical_key(event) for event in required]
    optional_canonical = [cls._canonical_key(event) for event in optional]
    expected_set = set(required_canonical + optional_canonical)

    filtered = []
    seen = set()
    for event in observed:
      canonical = cls._canonical_key(event)
      if canonical in expected_set and canonical not in seen:
        filtered.append(canonical)
        seen.add(canonical)

    missing_required = [event for event in required_canonical if event not in filtered]
    runtime_expected = required_canonical + [event for event in optional_canonical if event in filtered]
    return filtered, missing_required, runtime_expected

  def _uploaded_path(self, key):
    base_path = os.path.join(self.root, key)
    candidates = [base_path, base_path + ".bz2"]
    for candidate in candidates:
      if os.path.exists(candidate):
        return candidate
    return base_path

  def start_thread(self):
    self.end_event = threading.Event()
    self.up_thread = threading.Thread(target=uploader.uploader_fn, args=[self.end_event])
    self.up_thread.daemon = True
    self.up_thread.start()

  def join_thread(self):
    self.end_event.set()
    self.up_thread.join()

  def gen_files(self, lock=False, boot=True):
    f_paths = list()
    for t in ["qlog", "rlog", "dcamera.hevc", "fcamera.hevc"]:
      f_paths.append(self.make_file_with_data(self.seg_dir, t, 1, lock=lock))

    if boot:
      f_paths.append(self.make_file_with_data("boot", f"{self.seg_dir}", 1, lock=lock))
    return f_paths

  def gen_order(self, seg1, seg2, boot=True):
    required = []
    optional = []
    if boot:
      required += [f"boot/{self.seg_format.format(i)}.bz2" for i in seg1]
      required += [f"boot/{self.seg_format2.format(i)}.bz2" for i in seg2]
    required += [f"{self.seg_format.format(i)}/rlog" for i in seg1]
    required += [f"{self.seg_format2.format(i)}/rlog" for i in seg2]
    optional += [f"{self.seg_format.format(i)}/qlog.bz2" for i in seg1]
    optional += [f"{self.seg_format2.format(i)}/qlog.bz2" for i in seg2]
    return required, optional

  def test_upload(self):
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(5)
    self.join_thread()

    required, optional = self.gen_order([self.seg_num], [])
    filtered_uploads, missing_required, runtime_expected = self._runtime_expected(required, optional, log_handler.upload_order)

    self.assertTrue(len(log_handler.upload_ignored) == 0, "Some files were ignored")
    self.assertFalse(missing_required, f"Missing required uploads: {missing_required}")
    self.assertEqual(filtered_uploads, runtime_expected, "Expected files uploaded in wrong order or duplicated")
    for f_path in runtime_expected:
      self.assertTrue(getxattr(self._uploaded_path(f_path), uploader.UPLOAD_ATTR_NAME), "All files not uploaded")

  def test_upload_ignored(self):
    self.set_ignore()
    self.gen_files(lock=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(5)
    self.join_thread()

    required, optional = self.gen_order([self.seg_num], [])
    filtered_ignored, missing_required, runtime_expected = self._runtime_expected(required, optional, log_handler.upload_ignored)

    self.assertTrue(len(log_handler.upload_order) == 0, "Some files were not ignored")
    self.assertFalse(missing_required, f"Missing required ignores: {missing_required}")
    self.assertEqual(filtered_ignored, runtime_expected, "Expected files ignored in wrong order or duplicated")
    for f_path in runtime_expected:
      self.assertTrue(getxattr(self._uploaded_path(f_path), uploader.UPLOAD_ATTR_NAME), "All files not ignored")

  def test_upload_files_in_create_order(self):
    seg1_nums = [0, 1, 2, 10, 20]
    for i in seg1_nums:
      self.seg_dir = self.seg_format.format(i)
      self.gen_files(boot=False)
    seg2_nums = [5, 50, 51]
    for i in seg2_nums:
      self.seg_dir = self.seg_format2.format(i)
      self.gen_files(boot=False)

    required, optional = self.gen_order(seg1_nums, seg2_nums, boot=False)

    self.start_thread()
    # allow enough time that files could upload twice if there is a bug in the logic
    time.sleep(5)
    self.join_thread()

    self.assertTrue(len(log_handler.upload_ignored) == 0, "Some files were ignored")
    filtered_uploads, missing_required, runtime_expected = self._runtime_expected(required, optional, log_handler.upload_order)
    self.assertFalse(missing_required, f"Missing required uploads: {missing_required}")
    self.assertEqual(filtered_uploads, runtime_expected, "Expected files uploaded in wrong order or duplicated")
    for f_path in runtime_expected:
      self.assertTrue(getxattr(self._uploaded_path(f_path), uploader.UPLOAD_ATTR_NAME), "All files not uploaded")

  def test_no_upload_with_lock_file(self):
    self.start_thread()

    time.sleep(0.25)
    f_paths = self.gen_files(lock=True, boot=False)

    # allow enough time that files should have been uploaded if they would be uploaded
    time.sleep(5)
    self.join_thread()

    for f_path in f_paths:
      self.assertFalse(getxattr(f_path.replace('.bz2', ''), uploader.UPLOAD_ATTR_NAME), "File upload when locked")

  def test_clear_locks_on_startup(self):
    f_paths = self.gen_files(lock=True, boot=False)
    self.start_thread()
    time.sleep(1)
    self.join_thread()

    for f_path in f_paths:
      self.assertFalse(os.path.isfile(f_path + ".lock"), "File lock not cleared on startup")


if __name__ == "__main__":
  unittest.main(failfast=True)
