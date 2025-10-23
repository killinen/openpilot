#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

FAILED=0

IGNORED_DIRS_REGEX="^(cereal|third_party|tinygrad|tinygrad_repo|msgq|msgq_repo|rednose|rednose_repo|panda|opendbc|frogpilot/third_party|openpilot/third_party)"

function run_step() {
  local label="$1"
  shift

  printf "%-40s" "$label"
  if "$@" >/tmp/lint_step.log 2>&1; then
    echo "[✔]"
  else
    echo "[✗]"
    cat /tmp/lint_step.log
    FAILED=1
  fi
}

TRACKED_FILES="$(git ls-files | grep -Ev "$IGNORED_DIRS_REGEX" || true)"
PY_FILES="$(echo "$TRACKED_FILES" | grep -E '\.py$' || true)"

if [[ -z "$PY_FILES" ]]; then
  echo "No Python files to lint. Skipping."
  exit 0
fi

run_step "ruff" ruff check --quiet --select E9,F63,F7,F82 .
run_step "mypy" mypy --config-file pyproject.toml $PY_FILES

exit $FAILED
