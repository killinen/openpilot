#!/usr/bin/env python3
from __future__ import annotations

from collections import namedtuple
import bz2
import hashlib
import os
import random
import socket
import time
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse
import urllib.error
import urllib.request
import warnings

import capnp
from cereal import log as capnp_log

from panda.tests.safety_replay.replay_drive import replay_drive
from panda import Panda


DEFAULT_BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"
BASE_URL = os.environ.get("SAFETY_REPLAY_BASE_URL", DEFAULT_BASE_URL).strip() or DEFAULT_BASE_URL

DEFAULT_CACHE_DIR = "/tmp/safety_replay_cache"
CACHE_DIR = os.environ.get("SAFETY_REPLAY_CACHE_DIR", DEFAULT_CACHE_DIR).strip() or DEFAULT_CACHE_DIR

ReplayRoute = namedtuple("ReplayRoute", ("route", "safety_mode", "param", "alternative_experience"), defaults=(0, 0))

logs = [
  # These are curated for panda safety replay (same list as upstream panda).
  ReplayRoute("2425568437959f9d|2019-12-22--16-24-37.bz2", Panda.SAFETY_HONDA_NIDEC),       # HONDA.CIVIC
  ReplayRoute("38bfd238edecbcd7|2019-06-07--10-15-25.bz2", Panda.SAFETY_TOYOTA, 66),        # TOYOTA.PRIUS
  #ReplayRoute("f89c604cf653e2bf|2018-09-29--13-46-50.bz2", Panda.SAFETY_GM),                # GM.VOLT
  ReplayRoute("6fb4948a7ebe670e|2019-11-12--00-35-53.bz2", Panda.SAFETY_CHRYSLER),          # CHRYSLER.PACIFICA_2018_HYBRID
  #ReplayRoute("791340bc01ed993d|2019-04-08--10-26-00.bz2", Panda.SAFETY_SUBARU),            # SUBARU.IMPREZA
  #ReplayRoute("76b83eb0245de90e|2020-03-05--19-16-05.bz2", Panda.SAFETY_VOLKSWAGEN_MQB),    # VOLKSWAGEN.GOLF (MK7)
  #ReplayRoute("d12cd943127f267b|2020-03-27--15-57-18.bz2", Panda.SAFETY_VOLKSWAGEN_PQ),     # VW Passat B6
  #ReplayRoute("fbbfa6af821552b9|2020-03-03--08-09-43.bz2", Panda.SAFETY_NISSAN),            # NISSAN.XTRAIL
  #ReplayRoute("5b7c365c50084530_2020-04-15--16-13-24--3--rlog.bz2", Panda.SAFETY_HYUNDAI),  # HYUNDAI.SONATA
  ReplayRoute("610ebb9faaad6b43|2020-06-13--15-28-36.bz2", Panda.SAFETY_HYUNDAI_LEGACY),    # HYUNDAI.IONIQ_EV_LTD
  ReplayRoute("5ab784f361e19b78_2020-06-08--16-30-41.bz2", Panda.SAFETY_SUBARU_PREGLOBAL),  # SUBARU.OUTBACK (legacy safety)
  #ReplayRoute("bb50caf5f0945ab1|2021-06-19--17-20-18.bz2", Panda.SAFETY_TESLA),             # TESLA.AP2_MODELS
  #ReplayRoute("bd6a637565e91581_2021-10-29--22-18-31--1--rlog.bz2", Panda.SAFETY_MAZDA),    # MAZDA.CX9_2021
  #ReplayRoute("1a5d045d2c531a6d_2022-06-07--22-03-00--1--rlog.bz2", Panda.SAFETY_HONDA_BOSCH, Panda.FLAG_HONDA_RADARLESS, ALT_EXP.DISABLE_DISENGAGE_ON_GAS),
]

LOG_CARS = {
  "2425568437959f9d|2019-12-22--16-24-37.bz2": "HONDA.CIVIC",
  "38bfd238edecbcd7|2019-06-07--10-15-25.bz2": "TOYOTA.PRIUS",
  "f89c604cf653e2bf|2018-09-29--13-46-50.bz2": "GM.VOLT",
  "6fb4948a7ebe670e|2019-11-12--00-35-53.bz2": "CHRYSLER.PACIFICA_2018_HYBRID",
  "791340bc01ed993d|2019-04-08--10-26-00.bz2": "SUBARU.IMPREZA",
  "76b83eb0245de90e|2020-03-05--19-16-05.bz2": "VOLKSWAGEN.GOLF_MK7",
  "d12cd943127f267b|2020-03-27--15-57-18.bz2": "VOLKSWAGEN.PASSAT_B6",
  "fbbfa6af821552b9|2020-03-03--08-09-43.bz2": "NISSAN.XTRAIL",
  "5b7c365c50084530_2020-04-15--16-13-24--3--rlog.bz2": "HYUNDAI.SONATA",
  "610ebb9faaad6b43|2020-06-13--15-28-36.bz2": "HYUNDAI.IONIQ_EV_LTD",
  "5ab784f361e19b78_2020-06-08--16-30-41.bz2": "SUBARU.OUTBACK",
  "bb50caf5f0945ab1|2021-06-19--17-20-18.bz2": "TESLA.AP2_MODELS",
  "bd6a637565e91581_2021-10-29--22-18-31--1--rlog.bz2": "MAZDA.CX9_2021",
  "1a5d045d2c531a6d_2022-06-07--22-03-00--1--rlog.bz2": "HONDA.BOSCH_RADARLESS",
}

class LogReader:
  def __init__(self, path: str):
    self.path = path
    self.reset()

  def reset(self) -> None:
    with open(self.path, "rb") as f:
      dat = f.read()
    if dat.startswith(b"BZh"):
      dat = bz2.decompress(dat)

    ents = capnp_log.Event.read_multiple_bytes(dat)
    _ents = []
    try:
      for e in ents:
        _ents.append(e)
    except capnp.KjException:
      warnings.warn("Corrupted events detected", RuntimeWarning, stacklevel=1)
    self._ents = _ents

  def __iter__(self):
    return iter(self._ents)

def _is_url(s: str) -> bool:
  return s.startswith(("http://", "https://"))


def _infer_safety_params(lr):
  for msg in lr:
    if msg.which() == "carParams":
      cp = msg.carParams
      safety_cfg = None
      try:
        safety_cfgs = cp.safetyConfigs
        n = len(safety_cfgs)
        if n > 0:
          safety_cfg = safety_cfgs[n - 1]
      except Exception:
        safety_cfg = None

      if safety_cfg is not None:
        mode = safety_cfg.safetyModel.raw
        param = safety_cfg.safetyParam
      else:
        safety_model = getattr(cp, "safetyModel", 0)
        mode = safety_model.raw if hasattr(safety_model, "raw") else int(safety_model)
        param = int(getattr(cp, "safetyParam", 0))

      alt_exp = int(getattr(cp, "alternativeExperience", 0))
      if hasattr(lr, "reset"):
        lr.reset()
      return mode, param, alt_exp
  raise Exception("carParams not found in log. Set safety mode/param manually.")

def _infer_car(lr) -> str:
  for msg in lr:
    if msg.which() == "carParams":
      cp = msg.carParams
      car = str(getattr(cp, "carFingerprint", "") or getattr(cp, "carName", "") or "")
      if hasattr(lr, "reset"):
        lr.reset()
      return car
  if hasattr(lr, "reset"):
    lr.reset()
  return ""


def _get_request_headers() -> dict[str, str]:
  # Avoid proxies/servers applying a content-encoding on already-compressed .bz2 logs.
  return {"Accept-Encoding": "identity", "User-Agent": "panda-safety-replay"}


def _get_auth_header() -> str:
  return os.environ.get("SAFETY_REPLAY_AUTHORIZATION", "").strip()

def _get_auth_header_name() -> str:
  return os.environ.get("SAFETY_REPLAY_AUTH_HEADER_NAME", "").strip() or "Authorization"


def _get_auth_header_value() -> str:
  raw = _get_auth_header()
  if not raw:
    return ""

  # Optional prefix to make it easy to use token-only secrets (e.g. set prefix to "Bearer ").
  prefix = os.environ.get("SAFETY_REPLAY_AUTHORIZATION_PREFIX", "").strip()
  if not prefix:
    return raw
  if " " not in prefix and not prefix[-1].isspace():
    prefix += " "

  # Allow passing a full header value already containing scheme/prefix.
  if raw.lower().startswith(prefix.lower()):
    return raw
  return f"{prefix}{raw}"


def _auth_should_apply(route: str) -> bool:
  # Don't send user tokens to the default comma blob store: Azure treats an unexpected Authorization
  # header as a signed request and returns 403.
  if not _get_auth_header_value():
    return False
  if _is_url(route):
    return True
  return BASE_URL != DEFAULT_BASE_URL


def _get_request_headers_for_route(route: str) -> dict[str, str]:
  headers = _get_request_headers()
  if _auth_should_apply(route):
    headers[_get_auth_header_name()] = _get_auth_header_value()
  return headers


def _file_has_bzip2_magic(path: str) -> bool:
  try:
    with open(path, "rb") as f:
      return f.read(3) == b"BZh"
  except OSError:
    return False


def _download_attempts() -> int:
  # A couple environment variable names supported for compatibility.
  raw = os.environ.get("SAFETY_REPLAY_DOWNLOAD_ATTEMPTS", "").strip()
  if not raw:
    raw = os.environ.get("SAFETY_REPLAY_DOWNLOAD_RETRIES", "").strip()
  try:
    attempts = int(raw) if raw else 10
  except ValueError:
    attempts = 10
  return max(1, attempts)


def _download_timeouts() -> tuple[float, float]:
  connect_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_CONNECT_TIMEOUT_SEC", "").strip()
  read_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_READ_TIMEOUT_SEC", "").strip()
  try:
    connect = float(connect_s) if connect_s else 15.0
  except ValueError:
    connect = 15.0
  try:
    read = float(read_s) if read_s else 60.0
  except ValueError:
    read = 60.0
  return (max(1.0, connect), max(1.0, read))


def _is_retryable_status(status_code: int) -> bool:
  return status_code in (408, 425, 429, 500, 502, 503, 504)


def _retry_sleep(attempt_index: int) -> None:
  base_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_BACKOFF_SEC", "").strip()
  max_s = os.environ.get("SAFETY_REPLAY_DOWNLOAD_BACKOFF_MAX_SEC", "").strip()
  try:
    base = float(base_s) if base_s else 0.5
  except ValueError:
    base = 0.5
  try:
    max_delay = float(max_s) if max_s else 10.0
  except ValueError:
    max_delay = 10.0

  delay = min(max(0.0, max_delay), max(0.0, base) * (2 ** max(0, attempt_index)))
  delay *= random.uniform(0.7, 1.3)
  if delay > 0:
    time.sleep(delay)


def _cached_local_path(url: str) -> str:
  parsed = urlparse(url)
  basename = os.path.basename(parsed.path) or "safety_replay_log.bz2"
  url_key = (url.split("?", 1)[0]).encode("utf-8", errors="ignore")
  digest = hashlib.sha256(url_key).hexdigest()[:16]
  Path(CACHE_DIR).mkdir(parents=True, exist_ok=True)
  return os.path.join(CACHE_DIR, f"{digest}-{basename}")


def _download_if_needed(route: str, force: bool = False) -> str:
  if _is_url(route):
    url = route
  else:
    url = BASE_URL + route.lstrip("/")

  local = _cached_local_path(url)

  if os.path.isfile(local) and not force:
    # If a previous run cached a non-bzip2 response (e.g. HTML/JSON auth error), redownload.
    if _file_has_bzip2_magic(local):
      return local
    force = True

  if not os.path.isfile(local) or force:
    tmp = local + ".tmp"
    headers = _get_request_headers_for_route(route)
    timeouts = _download_timeouts()
    attempts = _download_attempts()
    last_exc: Optional[BaseException] = None

    for attempt in range(1, attempts + 1):
      if os.path.exists(tmp):
        os.remove(tmp)

      try:
        # urllib uses one timeout for both connect + read. Use the larger value.
        timeout = max(timeouts[0], timeouts[1])
        req = urllib.request.Request(url, headers=headers)

        try:
          resp = urllib.request.urlopen(req, timeout=timeout)
          status = int(resp.getcode() or 0)
        except urllib.error.HTTPError as e:
          resp = e
          status = int(e.code or 0)

        try:
          if status >= 400:
            if _is_retryable_status(status) and attempt < attempts:
              print(f"download failed (status={status}) for {url}; retrying ({attempt}/{attempts})")
              _retry_sleep(attempt - 1)
              continue
            auth_set = "set" if _auth_should_apply(route) else "unset"
            www_auth = ""
            try:
              www_auth = str(resp.headers.get("WWW-Authenticate") or "")
            except Exception:
              www_auth = ""
            err_preview = ""
            try:
              body = resp.read(512)
              err_preview = body.decode("utf-8", errors="replace").strip()
            except Exception:
              err_preview = ""

            msg = f"download failed (status={status}) for {url}; "
            msg += f"auth_header_name={_get_auth_header_name()!r}, auth_header={auth_set}, "
            msg += f"www_authenticate={www_auth!r}, body_preview={err_preview!r}"
            raise Exception(msg)

          first = resp.read(64 * 1024)
          if len(first) < 3 or first[:3] != b"BZh":
            preview = first[:200]
            try:
              preview_text = preview.decode("utf-8", errors="replace")
            except Exception:
              preview_text = repr(preview)

            if _is_retryable_status(status) and attempt < attempts:
              print(f"download returned non-bzip2 content for {url}; retrying ({attempt}/{attempts})")
              _retry_sleep(attempt - 1)
              continue
            raise Exception(
              f"downloaded non-bzip2 content from {url} "
              + f"(status={status}, auth_header={'set' if 'Authorization' in headers else 'unset'}, "
              + f"content-type={resp.headers.get('Content-Type')}, "
              + f"content-encoding={resp.headers.get('Content-Encoding')}, preview={preview_text!r})"
            )

          with open(tmp, "wb") as f:
            f.write(first)
            while True:
              chunk = resp.read(64 * 1024)
              if not chunk:
                break
              f.write(chunk)

          os.replace(tmp, local)
          break
        finally:
          try:
            resp.close()
          except Exception:
            pass
      except (urllib.error.URLError, socket.timeout, TimeoutError, OSError) as e:
        last_exc = e
        if attempt < attempts:
          print(f"download error for {url}: {type(e).__name__}: {e}; retrying ({attempt}/{attempts})")
          _retry_sleep(attempt - 1)
          continue
        raise
      except Exception as e:
        last_exc = e
        raise
      finally:
        if os.path.exists(tmp):
          os.remove(tmp)

    if not os.path.isfile(local):
      raise Exception(f"failed to download {url} after {attempts} attempts: {last_exc!r}")
  return local


if __name__ == "__main__":
  # Optional custom log, configured via CI env/vars.
  custom_url = os.environ.get("SAFETY_REPLAY_CUSTOM_URL", "").strip()
  if custom_url:
    print(f"adding custom safety replay log: {custom_url}")
    prefix_dbg = os.environ.get("SAFETY_REPLAY_AUTHORIZATION_PREFIX", "").strip()
    prefix_state = "unset" if not prefix_dbg else ("set" if prefix_dbg.endswith(" ") else "set (no trailing space)")
    print(
      f"custom safety replay auth header: {'set' if _get_auth_header_value() else 'unset'} "
      + f"({_get_auth_header_name()}, prefix {prefix_state})"
    )
    if _get_auth_header() and not prefix_dbg and " " not in _get_auth_header():
      print(
        "note: SAFETY_REPLAY_AUTHORIZATION_PREFIX is unset; if your log server expects Bearer tokens, "
        + "set SAFETY_REPLAY_AUTHORIZATION_PREFIX=Bearer or include 'Bearer ' in SAFETY_REPLAY_AUTHORIZATION"
      )
    mode_s = os.environ.get("SAFETY_REPLAY_CUSTOM_SAFETY_MODE", "").strip()
    param_s = os.environ.get("SAFETY_REPLAY_CUSTOM_SAFETY_PARAM", "").strip() or "0"
    alt_s = (
      os.environ.get("SAFETY_REPLAY_CUSTOM_ALT_EXP", "").strip()
      or os.environ.get("SAFETY_REPLAY_CUSTOM_ALTERNATIVE_EXPERIENCE", "").strip()
      or "0"
    )
    print(f"custom safety replay params: mode={mode_s or '<unset>'}, param={param_s}, alt_exp={alt_s}")

    if mode_s.lower() == "auto":
      logs.append(ReplayRoute(custom_url, -1, -1, -1))
    else:
      if not mode_s:
        raise Exception("SAFETY_REPLAY_CUSTOM_SAFETY_MODE must be set (or 'auto') when SAFETY_REPLAY_CUSTOM_URL is set")
      logs.append(ReplayRoute(custom_url, int(mode_s), int(param_s), int(alt_s)))
  elif "SAFETY_REPLAY_CUSTOM_URL" in os.environ:
    print("SAFETY_REPLAY_CUSTOM_URL is set but empty; skipping custom safety replay log")

  for route, _, _, _ in logs:
    _download_if_needed(route)

  failed = []
  for route, mode, param, alt_exp in logs:
    local = _download_if_needed(route)
    try:
      lr = LogReader(local)
    except OSError:
      # Corrupt/truncated cache; redownload once.
      local = _download_if_needed(route, force=True)
      lr = LogReader(local)

    car = LOG_CARS.get(route, "")
    if mode < 0 or param < 0 or alt_exp < 0:
      mode, param, alt_exp = _infer_safety_params(lr)
      if not car:
        car = _infer_car(lr)
    elif not car:
      car = _infer_car(lr)

    car_s = f" ({car})" if car else ""
    print(f"\nreplaying {route}{car_s} with safety mode {mode}, param {param}, alternative experience {alt_exp}")
    # All inputs here are single log files (segments), so initialize safety state from the first steering command.
    if not replay_drive(lr, mode, param, alt_exp, segment=True):
      failed.append(f"{route}{car_s}")

    for f in failed:
      print(f"\n**** failed on {f} ****")
    assert len(failed) == 0, f"\nfailed on {len(failed)} logs"
