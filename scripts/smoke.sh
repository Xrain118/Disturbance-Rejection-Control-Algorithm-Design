#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PROJECT_ROOT/install-flight/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-174}"
cd "$PROJECT_ROOT"
bash src/dragonfly_arm/test/smoke_test.sh
