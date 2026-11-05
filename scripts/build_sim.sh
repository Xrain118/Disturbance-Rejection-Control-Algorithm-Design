#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
cd "$PROJECT_ROOT"
colcon build \
  --build-base build-sim \
  --install-base install-sim \
  --symlink-install \
  --packages-select \
    dragonfly_arm \
    dragonfly_control \
    dragonfly_fcu_bridge \
    dragonfly_flight \
    dragonfly_interfaces \
    dragonfly_onboard \
    dragonfly_serial \
    dragonfly_sim \
  "$@"
