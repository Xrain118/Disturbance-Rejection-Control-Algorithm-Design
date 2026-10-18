#!/usr/bin/env bash
source "$(dirname "$0")/../../../scripts/env.sh"
set -e
cd "$(dirname "$0")/../../.."
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source install-flight/setup.bash
prefix="${1:-log/acceptance_$(date +%Y%m%d_%H%M%S)}"
for round in 1 2 3; do
  ros2 run dragonfly_flight flight --scenario acceptance --record --output "${prefix}_$round"
  ros2 run dragonfly_flight verify "${prefix}_$round"
  python3 src/dragonfly_flight/test/plot_run.py "${prefix}_$round"
done
