#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source "$PROJECT_ROOT/install-sim/setup.bash"
cd "$PROJECT_ROOT"
colcon test --build-base build-sim --install-base install-sim --packages-select dragonfly_arm dragonfly_control dragonfly_flight dragonfly_serial dragonfly_fcu_bridge dragonfly_sim --event-handlers console_direct+
colcon test-result --test-result-base build-sim --verbose
