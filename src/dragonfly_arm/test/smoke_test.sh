#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not available; source ROS 2 and this workspace first." >&2
  exit 2
fi

smoke_log="${TMPDIR:-/tmp}/dragonfly_arm_smoke_$$.log"
launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

setsid ros2 launch dragonfly_arm demo.launch.py gui:=false >"${smoke_log}" 2>&1 &
launch_pid=$!

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! timeout 190 python3 "${script_dir}/verify_runtime.py"; then
  echo "Controllers or joint states did not become ready." >&2
  cat "${smoke_log}" >&2
  exit 4
fi

trajectory_succeeded=false
for _ in {1..50}; do
  if grep -q "Trajectory completed successfully" "${smoke_log}"; then
    trajectory_succeeded=true
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
  sleep 1
done

if [[ "${trajectory_succeeded}" != true ]]; then
  echo "The demonstration trajectory did not complete successfully." >&2
  cat "${smoke_log}" >&2
  exit 6
fi

if grep -Eq '\[ERROR\]|\[FATAL\]' "${smoke_log}"; then
  echo "Simulation logged a startup/runtime error despite completed trajectories."
  cat "${smoke_log}"
  exit 7
fi

if [[ -n "${1:-}" ]]; then
  python3 "${script_dir}/capture_gazebo.py" "$1"
fi

echo "Dragonfly dual-arm smoke test passed."
echo "Log: ${smoke_log}"
