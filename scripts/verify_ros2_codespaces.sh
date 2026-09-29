#!/usr/bin/env bash
# Preserve a complete, non-masking validation record for this Codespace.
set -Eeuo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
stamp=$(date -u +%Y%m%dT%H%M%SZ)
evidence=${RTMC_VERIFY_OUTPUT_DIR:-"${HOME}/rtmc-ros2-evidence/${stamp}-$$"}
if [[ -d $evidence && -n $(ls -A "$evidence") ]]; then
  printf 'Evidence directory must be empty: %s\n' "$evidence" >&2
  exit 2
fi
mkdir -p "$evidence"
evidence=$(cd -- "$evidence" && pwd)
failures=()

record_failure() {
  failures+=("$1")
  printf '%s\tFAIL\n' "$1" >> "$evidence/phases.tsv"
  printf 'FAIL: %s (see %s)\n' "$1" "$evidence" >&2
}

run_step() {
  local label=$1
  shift
  printf '\n== %s ==\n' "$label"
  if "$@" 2>&1 | tee "$evidence/${label}.log"; then
    printf '%s\tPASS\n' "$label" >> "$evidence/phases.tsv"
    printf 'PASS: %s\n' "$label"
    return 0
  else
    record_failure "$label"
    return 1
  fi
}

port_is_free() {
  python3 - <<'PY'
import socket
with socket.socket() as server:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 8082))
PY
}

finish() {
  printf '\nEvidence: %s\n' "$evidence"
  if ((${#failures[@]})); then
    printf 'Failed phases (%d):\n' "${#failures[@]}" >&2
    printf '  %s\n' "${failures[@]}" >&2
    exit 1
  fi
  printf 'All verification phases passed.\n'
}

cd "$repo"
printf 'phase\tresult\n' > "$evidence/phases.tsv"
{
  printf 'UTC: %s\nRepository: %s\nEvidence: %s\n' "$stamp" "$repo" "$evidence"
  printf 'Kernel: '; uname -a
  printf 'Python: '; python3 --version
  printf 'CMake: '; cmake --version | head -1
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    printf 'OS: %s %s\n' "${ID:-unknown}" "${VERSION_ID:-unknown}"
  fi
} > "$evidence/environment.txt" 2>&1 || true
git rev-parse HEAD > "$evidence/git-revision.txt" 2>/dev/null || true
git status --porcelain=v1 > "$evidence/git-status.txt" 2>/dev/null || true
git diff --binary | sha256sum > "$evidence/tracked-diff-sha256.txt" 2>/dev/null || true
python3 - "$evidence/source-sha256.json" <<'PY'
import hashlib
import json
from pathlib import Path
import subprocess
import sys

names = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'])
hashes = {}
for raw in names.split(b'\0'):
    if raw:
        path = Path(raw.decode())
        if path.is_file():
            hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
Path(sys.argv[1]).write_text(json.dumps(hashes, indent=2, sort_keys=True) + '\n')
PY

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  record_failure 'missing-jazzy-setup'
  finish
fi
# shellcheck disable=SC1091
set +u
if ! source /opt/ros/jazzy/setup.bash; then
  set -u
  record_failure 'jazzy-setup-failed'
  finish
fi
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
for tool in cmake ctest colcon ros2 python3; do
  command -v "$tool" >/dev/null || record_failure "missing-$tool"
done

cmake_ok=false
if command -v cmake >/dev/null; then
  if run_step cmake-configure cmake -S "$repo" -B "$repo/build" &&
     run_step cmake-build cmake --build "$repo/build" --parallel 2; then
    cmake_ok=true
  fi
fi

if [[ $cmake_ok == true ]]; then
  if port_is_free; then
    run_step ctest ctest --test-dir "$repo/build" --output-on-failure || true
  else
    record_failure 'ctest-port-8082-occupied'
  fi
else
  record_failure 'ctest-skipped-no-build'
fi

run_step reference-check python3 tools/generate_collision_reference.py --check || true
run_step reference-test python3 tests/test_collision_reference.py || true

if [[ -d ros2/src/rtmc_ros2/test ]]; then
  run_step bridge-policy env PYTHONPATH="$repo/ros2/src/rtmc_ros2" \
    python3 -m unittest discover -s ros2/src/rtmc_ros2/test -p 'test_*.py' || true
else
  record_failure 'bridge-policy-tests-missing'
fi

colcon_ok=false
if command -v colcon >/dev/null; then
  if run_step colcon-build colcon --log-base "$evidence/colcon-log" build \
      --base-paths "$repo/ros2/src" \
      --build-base "$evidence/colcon-build" \
      --install-base "$evidence/colcon-install" \
      --event-handlers console_direct+; then
    colcon_ok=true
    run_step colcon-test colcon --log-base "$evidence/colcon-log" test \
      --base-paths "$repo/ros2/src" \
      --build-base "$evidence/colcon-build" \
      --install-base "$evidence/colcon-install" \
      --event-handlers console_direct+ || true
    run_step colcon-results colcon test-result \
      --test-result-base "$evidence/colcon-build" --verbose || true
  fi
fi

if [[ $cmake_ok == true && $colcon_ok == true ]]; then
  # shellcheck disable=SC1091
  set +u
  if source "$evidence/colcon-install/setup.bash"; then
    overlay_ok=true
  else
    overlay_ok=false
    record_failure 'colcon-overlay-source'
  fi
  set -u
  if [[ $overlay_ok == true ]] && port_is_free; then
    mkdir -p "$evidence/smoke"
    run_step ros-smoke timeout --signal=INT --kill-after=10s 120s ros2 run rtmc_ros2 smoke \
      --binary "$repo/build/pcan_demo" --assets "$repo" \
      --evidence-dir "$evidence/smoke" || true
  elif [[ $overlay_ok == true ]]; then
    record_failure 'ros-smoke-port-8082-occupied'
  fi
else
  record_failure 'ros-smoke-skipped-no-build'
fi

finish
