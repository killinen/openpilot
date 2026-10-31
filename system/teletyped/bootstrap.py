#!/usr/bin/env python3
"""Frozen-worker entry point. Never use the development checkout's import path."""
import importlib
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time


def main():
  checkout, runtime, *sites = sys.argv[1:]
  preflight = "--preflight" in sites
  sites = [site for site in sites if site != "--preflight"]
  checkout, runtime = Path(checkout).resolve(), Path(runtime).resolve()
  # -I -S avoids site hooks and .pth files. Only frozen package directories are added.
  sys.path[:] = [str(checkout), str(checkout / "frogpilot/third_party"),
                 *(str(runtime / site) for site in sites), *sys.path]
  sys.dont_write_bytecode = True
  os.chdir(checkout)
  os.environ["BASEDIR"] = str(checkout)
  if not preflight:
    parent = os.getppid()
    ready = Path(os.environ["TELETYPED_LAUNCH_READY"])
    deadline = time.monotonic() + 10
    while not ready.exists():
      if os.getppid() != parent or time.monotonic() > deadline:
        raise RuntimeError("guardian did not commit worker identity")
      time.sleep(0.05)

    def watch_parent():
      while True:
        if os.getppid() != parent:
          os.killpg(os.getpgrp(), signal.SIGKILL)
        time.sleep(1)

    threading.Thread(target=watch_parent, daemon=True, name="guardian_lifetime").start()
  module = importlib.import_module("openpilot.tools.teletyped.teletyped")
  if getattr(module, "GUARDIAN_PROTOCOL_VERSION", None) != 1:
    raise RuntimeError("worker does not support guardian health protocol v1")
  if preflight:
    module.guardian_preflight()
    print(json.dumps({"preflight": "ok", "module": module.__file__}), flush=True)
    return
  module.main()


if __name__ == "__main__":
  main()
