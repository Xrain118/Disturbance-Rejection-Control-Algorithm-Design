#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/../../../scripts/env.sh"
cd "$PROJECT_ROOT"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
# Building this workspace never changes or builds the shared PX4 checkout.
colcon build --build-base build-flight --install-base install-flight --symlink-install --packages-select dragonfly_arm dragonfly_flight dragonfly_interfaces dragonfly_serial "$@"
