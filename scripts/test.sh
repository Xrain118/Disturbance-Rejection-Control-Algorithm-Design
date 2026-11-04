#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source "$PROJECT_ROOT/install-flight/setup.bash"
cd "$PROJECT_ROOT"
colcon test --build-base build-flight --install-base install-flight --packages-select dragonfly_arm dragonfly_flight dragonfly_serial dragonfly_fcu_bridge --event-handlers console_direct+
colcon test-result --test-result-base build-flight --verbose
