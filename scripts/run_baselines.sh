#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source "$PROJECT_ROOT/install-sim/setup.bash"
cd "$PROJECT_ROOT"
exec python3 -m dragonfly_flight.baseline "$@"
