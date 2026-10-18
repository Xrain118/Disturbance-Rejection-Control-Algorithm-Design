#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/env.sh"
source "$ROS_SETUP"
cd "$PROJECT_ROOT"
colcon build --build-base build-hardware --install-base install-hardware --symlink-install --packages-select dragonfly_interfaces dragonfly_serial "$@"
