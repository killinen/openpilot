import os
import pathlib
import shutil
import subprocess
from dataclasses import dataclass

import jinja2

TEST_DIR = pathlib.Path(__file__).parent
REPORT_DIR = TEST_DIR / "test_ui" / "report"
SCREENSHOTS_DIR = REPORT_DIR / "screenshots"
SNAPSHOT_BIN = TEST_DIR / "tests" / "ui_snapshot"
CASES = [
  "homescreen",
  "settings_device",
  "settings_network",
  "onroad",
  "onroad_map",
  "onroad_sidebar",
]


def ensure_binary():
  if not SNAPSHOT_BIN.exists():
    subprocess.run(["scons", "--extras", "selfdrive/ui/tests/ui_snapshot"], check=True)


def ensure_dirs():
  if REPORT_DIR.exists():
    shutil.rmtree(REPORT_DIR)
  SCREENSHOTS_DIR.mkdir(parents=True)


def render_case(case: str):
  env = os.environ.copy()
  env["QT_QPA_PLATFORM"] = "offscreen"
  subprocess.run([str(SNAPSHOT_BIN), "-o", str(SCREENSHOTS_DIR / f"{case}.png")], check=True, env=env)


def build_report():
  template = (TEST_DIR / "test_ui" / "template.html").read_text()
  html = jinja2.Template(template).render(
    cases=[(case, f"screenshots/{case}.png") for case in CASES]
  )
  (REPORT_DIR / "index.html").write_text(html)


def main():
  ensure_binary()
  ensure_dirs()
  for case in CASES:
    render_case(case)
  build_report()

if __name__ == "__main__":
  main()
