"""Archive transport regressions without hardware or live GCS connections."""

from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest
import requests


@pytest.fixture
def sender():
  spec = importlib.util.spec_from_file_location("archive_sender_test", Path(__file__).parents[1] / "archive_sender.py")
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def transfer(sender, tmp_path, monkeypatch):
  root = tmp_path / "realdata"
  root.mkdir()
  job = {"id": "job-1", "segment": "route--0", "constraints": {"network_policy": "wifi"}}
  posts = []

  def post(path, payload):
    posts.append((path, payload))
    return {}

  client = SimpleNamespace(post=post, get=lambda _: {"status": "received"})
  stop = threading.Event()
  monkeypatch.setattr(stop, "wait", lambda _: None)
  monkeypatch.setattr(sender, "package", lambda *args: [])
  monkeypatch.setattr(sender, "send_archive", lambda binary, archive, announce, permitted, completed: announce("1-test-code"))

  def run():
    return sender.transfer_job(root, job, client, lambda: (True, "wifi"), "wormhole", None, stop)

  return SimpleNamespace(run=run, client=client, posts=posts, root=root, job=job)


@pytest.mark.parametrize("error", [requests.Timeout("slow GCS"), requests.HTTPError("503"), ValueError("invalid JSON")])
def test_status_poll_errors_keep_live_send_running(sender, transfer, monkeypatch, error):
  responses = iter([error, {"status": "received"}])

  def get(_):
    result = next(responses)
    if isinstance(result, Exception):
      raise result
    return result

  def send(binary, archive, announce, permitted, completed):
    announce("1-test-code")
    assert not completed(force=True)
    assert permitted()
    assert completed(force=True)

  transfer.client.get = get
  monkeypatch.setattr(sender, "send_archive", send)
  assert transfer.run()
  assert [path for path, _ in transfer.posts] == ["/jobs/job-1/offer"]


@pytest.mark.parametrize("status", ["offered", "receiving"])
def test_offer_timeout_checks_whether_gcs_already_accepted_it(transfer, status):
  def post(path, payload):
    transfer.posts.append((path, payload))
    raise requests.Timeout("response lost after acceptance")

  responses = iter([{"status": status}, {"status": "received"}])
  transfer.client.post = post
  transfer.client.get = lambda _: next(responses)
  assert transfer.run()
  assert [path for path, _ in transfer.posts] == ["/jobs/job-1/offer"]


@pytest.mark.parametrize("poll_error", [False, True])
def test_offer_retries_transient_errors_then_completes(transfer, poll_error):
  attempts = []

  def post(path, payload):
    attempts.append((path, payload))
    if len(attempts) < 3:
      raise requests.ConnectionError("temporary disconnect")
    return {}

  responses = iter([{"status": "queued"}, {"status": "queued"}, {"status": "received"}])

  def get(_):
    result = next(responses)
    if poll_error and result["status"] == "queued":
      raise requests.Timeout("status unavailable")
    return result

  transfer.client.post = post
  transfer.client.get = get
  assert transfer.run()
  assert len(attempts) == 3
  assert all(attempt == attempts[0] for attempt in attempts)


def test_offer_retry_limit_reports_failure(transfer):
  def post(path, payload):
    transfer.posts.append((path, payload))
    if path.endswith("/offer"):
      raise requests.Timeout("GCS unavailable")
    return {}

  transfer.client.post = post
  transfer.client.get = lambda _: {"status": "queued"}
  assert not transfer.run()
  assert [path for path, _ in transfer.posts].count("/jobs/job-1/offer") == 3
  path, payload = transfer.posts[-1]
  assert path == "/jobs/job-1/failed"
  assert "Timeout: GCS unavailable" in payload["detail"]
  assert payload["sender_stopped"] is True


@pytest.mark.parametrize("stage", ["preparing", "sending", "awaiting_receipt"])
def test_failures_report_stage_and_only_stop_receiver_for_interrupted_send(sender, transfer, monkeypatch, stage):
  def fail(*args, **kwargs):
    raise ValueError("injected failure")

  if stage == "preparing":
    monkeypatch.setattr(sender, "package", fail)
  elif stage == "sending":
    monkeypatch.setattr(sender, "send_archive", fail)
  else:
    transfer.client.get = lambda _: {"status": "failed"}

  assert not transfer.run()
  path, payload = transfer.posts[-1]
  assert path == "/jobs/job-1/failed"
  assert payload["detail"].startswith("Sender failed while " + stage + ": ValueError:")
  assert payload["sender_stopped"] == (stage == "sending")
  failure = json.loads((transfer.root.parent / ".opnas-sender-failures.jsonl").read_text())
  assert failure["job_id"] == transfer.job["id"]
  assert failure["segment"] == transfer.job["segment"]
  assert failure["stage"] == stage
  assert not list((transfer.root.parent / ".opnas-incoming").iterdir())


def test_failure_log_is_bounded_and_retains_concurrent_failures(sender, tmp_path):
  root = tmp_path / "realdata"
  path = tmp_path / ".opnas-sender-failures.jsonl"
  path.write_text("{}\n" * sender.FAILURE_LOG_LINES)
  with ThreadPoolExecutor(max_workers=8) as pool:
    list(pool.map(lambda index: sender.record_failure(root, {"id": str(index)}, "sending", "network error"), range(32)))

  records = [json.loads(line) for line in path.read_text().splitlines()]
  assert len(records) == sender.FAILURE_LOG_LINES
  assert {record["job_id"] for record in records[-32:]} == {str(index) for index in range(32)}


def test_failure_log_write_error_does_not_hide_transfer_failure(sender, tmp_path):
  # A missing parent makes the log unwritable without depending on user permissions.
  sender.record_failure(tmp_path / "missing" / "realdata", {"id": "job"}, "preparing", "bad source")
