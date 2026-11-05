#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
if [[ -f "$PX4_ROS_WS/install/setup.bash" ]]; then
  source "$PX4_ROS_WS/install/setup.bash"
fi
cd "$PROJECT_ROOT"
colcon build \
  --build-base build-onboard \
  --install-base install-onboard \
  --symlink-install \
  --packages-select \
    dragonfly_arm \
    dragonfly_control \
    dragonfly_fcu_bridge \
    dragonfly_interfaces \
    dragonfly_onboard \
    dragonfly_serial \
  "$@"
