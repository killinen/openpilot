# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Resumable OP-NAS log-data API fetch of i30 CAN + log signals, one gz file per segment."""

import gzip
import json
import os
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

BASE = os.environ.get("OPNAS_URL", "http://192.168.8.150:8090")
ROOT = Path(os.environ['I30_LONG_DATA'])  # data directory outside the repository (required)
I30 = ["ff528bde139e99de", "41c5cb6e", "23a0dabf", "0be758db"]
SIGNALS = {
  "can": ["address", "src", "dat"],
  "sendcan": ["address", "src", "dat"],
  "carState": ["vEgo", "aEgo", "gas", "gasPressed", "brakePressed", "clutchPressed", "standstill", "engineRpm", "canValid"],
  "carControl": ["enabled", "longActive", "actuators.accel", "actuators.gas", "orientationNED"],
  "carOutput": ["actuatorsOutput.gas", "actuatorsOutput.accel"],
  "controlsState": ["enabled", "longControlState", "vCruise", "vPid", "upAccelCmd", "uiAccelCmd", "ufAccelCmd", "forceDecel"],
  "longitudinalPlan": ["aTarget", "allowThrottle", "hasLead", "speeds", "accels", "longitudinalPlanSource"],
  "radarState": ["leadOne.status", "leadOne.dRel", "leadOne.vRel", "leadOne.vLead", "leadOne.aLeadK", "leadOne.modelProb"],
  "livePose": ["orientationNED.y", "orientationNED.valid", "inputsOK"],
  "liveLocationKalman": ["calibratedOrientationNED.value", "calibratedOrientationNED.valid", "status"],
  "frogpilotPlan": ["vCruise", "maxAcceleration", "minAcceleration", "disableThrottle"],
  "initData": ["gitCommit", "gitBranch", "version", "wallTimeNanos"],
  "carParams": ["carFingerprint", "enableGasInterceptor"],
}


def fetch(dev, route, seg):
  out = ROOT / "raw" / dev / route / f"{seg}.jsonl.gz"
  if out.exists():
    return "cached"
  out.parent.mkdir(parents=True, exist_ok=True)
  q = {"device_id": dev, "route": route, "segments": [seg], "signals": SIGNALS}
  for attempt in range(4):
    try:
      req = urllib.request.Request(
        BASE + "/api/log-data/query", data=json.dumps(q).encode(), headers={"Content-Type": "application/json", "Accept-Encoding": "gzip"}
      )
      with urllib.request.urlopen(req, timeout=900) as r:
        data = r.read()
      lines = gzip.decompress(data).splitlines()
      end = json.loads(lines[-1])
      if end.get("type") != "end":
        raise RuntimeError("no end line")
      if not end.get("complete"):
        reason = end.get("failed") or end.get("missing")
        if any(f.get("reason") in ("pi_busy_with_transfer", "read_failed") for f in end.get("failed", [])):
          raise RuntimeError(f"retryable {reason}")
        (out.parent / f"{seg}.failed.json").write_text(json.dumps(end))
        return f"failed {reason}"
      tmp = out.with_suffix(".tmp")
      tmp.write_bytes(data)
      tmp.rename(out)
      return "ok"
    except Exception as e:
      err = repr(e)
      time.sleep(20 * (attempt + 1))
  (out.parent / f"{seg}.error.txt").write_text(err)
  return "error " + err


def main():
  drives = json.load(open(ROOT / "opnas-drives-2026-11-12.json"))["drives"]
  jobs = []
  for dev in I30:
    for d in sorted((x for x in drives if x["device_id"] == dev), key=lambda x: x["route"], reverse=True):
      info = json.loads(urllib.request.urlopen(f"{BASE}/api/log-data/drives/{dev}/{d['route']}", timeout=60).read())
      for s in info["segments"]:
        jobs.append((dev, d["route"], s["segment"]))
  (ROOT / "segment-inventory.json").write_text(json.dumps(jobs))
  print("jobs", len(jobs), flush=True)
  t0 = time.time()
  with ThreadPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else 4) as ex:
    for i, (job, res) in enumerate(zip(jobs, ex.map(lambda j: fetch(*j), jobs))):
      print(i, round(time.time() - t0), *job, res, flush=True)


if __name__ == "__main__":
  main()
