#!/usr/bin/env python3
import os
import subprocess
import textwrap
from pathlib import Path

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from common.basedir import BASEDIR
from common.spinner import Spinner
from common.text_window import TextWindow
from system.hardware import TICI
from system.swaglog import cloudlog, add_file_handler
from system.version import is_dirty

MAX_CACHE_SIZE = 4e9 if "CI" in os.environ else 2e9
CACHE_DIR = Path("/data/scons_cache" if TICI else "/tmp/scons_cache")

TOTAL_SCONS_NODES = 2035
MAX_BUILD_PROGRESS = 100
PREBUILT = os.path.exists(os.path.join(BASEDIR, 'prebuilt'))


def build(spinner: Spinner, dirty: bool = False) -> None:
  # CHECK AND INSTALL MISSING PYTHON LIBRARIES FOR 0815
  if subprocess.call("python3 -c 'import hatanaka'", shell=True) != 0:
    print("[INSTALL] hatanaka not found, installing required packages...")
    spinner.update("[INSTALL] hatanaka not found, installing required packages...")

    install_script = """
    set -ex
    mount -o rw,remount /system

    pip install importlib_resources==5.12.0 ncompress==1.0.0
    pip install --no-deps hatanaka==2.4.0
    apt-get update
    apt-get install -y libyaml-dev
    pip install --no-cache-dir --force-reinstall setuptools==67.6.0 wheel==0.38.4 cython==0.29.33
    pip install --no-cache-dir --verbose --force-reinstall -I pyyaml==6.0 --global-option=--with-libyaml --no-build-isolation
    python3 -c 'from yaml import CSafeLoader; print("CSafeLoader is available.")'

    mount -o remount,r /system
    """
    # Run the shell script and show text windond if installtion fails
    try:
      subprocess.run(install_script, shell=True, executable="/data/data/com.termux/files/usr/bin/sh", check=True)

      # Show TextWindow after successful install
      spinner.close()
      if not os.getenv("CI"):
        with TextWindow("Missing python libraries have been installed.\nPlease reboot the device.") as t:
          print('Missing python libraries have been installed. Please reboot the device.')
          t.wait_for_exit()
      exit(1)

    except subprocess.CalledProcessError as e:
      spinner.close()
      err_msg = f"Python dependency install script failed with code {e.returncode}.\n\n{e}"
      add_file_handler(cloudlog)
      cloudlog.error(err_msg)

      if not os.getenv("CI"):
        wrapped = "\n".join(textwrap.wrap(err_msg, 65))
        with TextWindow("openpilot failed to install dependencies\n\n" + wrapped) as t:
          t.wait_for_exit()
      exit(1)

    except Exception as e:
      spinner.close()
      err_msg = f"Unexpected error during dependency install:\n\n{str(e)}"
      add_file_handler(cloudlog)
      cloudlog.error(err_msg)

      if not os.getenv("CI"):
        wrapped = "\n".join(textwrap.wrap(err_msg, 65))
        with TextWindow("openpilot crashed during setup\n\n" + wrapped) as t:
          t.wait_for_exit()
      exit(1) 
  # END CHECK AND INSTALL FIX FOR 0815
  
  env = os.environ.copy()
  env['SCONS_PROGRESS'] = "1"
  nproc = os.cpu_count()
  j_flag = "" if nproc is None else f"-j{nproc - 1}"

  scons: subprocess.Popen = subprocess.Popen(["scons", j_flag, "--cache-populate"], cwd=BASEDIR, env=env, stderr=subprocess.PIPE)
  assert scons.stderr is not None

  compile_output = []

  # Read progress from stderr and update spinner
  while scons.poll() is None:
    try:
      line = scons.stderr.readline()
      if line is None:
        continue
      line = line.rstrip()

      prefix = b'progress: '
      if line.startswith(prefix):
        i = int(line[len(prefix):])
        spinner.update_progress(MAX_BUILD_PROGRESS * min(1., i / TOTAL_SCONS_NODES), 100.)
      elif len(line):
        compile_output.append(line)
        print(line.decode('utf8', 'replace'))
    except Exception:
      pass

  if scons.returncode != 0:
    # Read remaining output
    r = scons.stderr.read().split(b'\n')
    compile_output += r

    # Build failed log errors
    errors = [line.decode('utf8', 'replace') for line in compile_output
              if any(err in line for err in [b'error: ', b'not found, needed by target'])]
    error_s = "\n".join(errors)
    add_file_handler(cloudlog)
    cloudlog.error("scons build failed\n" + error_s)

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      error_s = "\n \n".join("\n".join(textwrap.wrap(e, 65)) for e in errors)
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)


  # enforce max cache size
  cache_files = [f for f in CACHE_DIR.rglob('*') if f.is_file()]
  cache_files.sort(key=lambda f: f.stat().st_mtime)
  cache_size = sum(f.stat().st_size for f in cache_files)
  for f in cache_files:
    if cache_size < MAX_CACHE_SIZE:
      break
    cache_size -= f.stat().st_size
    f.unlink()


if __name__ == "__main__" and not PREBUILT:
  spinner = Spinner()
  spinner.update_progress(0, 100)
  build(spinner, is_dirty())
