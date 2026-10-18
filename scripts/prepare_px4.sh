#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
test "$(git -C "$PX4_ROS_WS/src/px4_msgs" rev-parse HEAD)" = 392e831c1f659429ca83902e66820d7094591410
test "$(git -C "$XRCE_AGENT_ROOT" describe --tags --always)" = v2.4.3
patch_file="$PROJECT_ROOT/src/dragonfly_flight/config/px4_mag_enu.patch"
if ! git -C "$PX4_ROOT" apply --reverse --check "$patch_file" 2>/dev/null; then
  git -C "$PX4_ROOT" apply --check "$patch_file"
  git -C "$PX4_ROOT" apply "$patch_file"
fi
bash "$PROJECT_ROOT/src/dragonfly_flight/scripts/apply_px4_vector.sh" "$PX4_ROOT"
make -C "$PX4_ROOT" px4_sitl_default
