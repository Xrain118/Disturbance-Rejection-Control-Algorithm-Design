#!/usr/bin/env bash
source "$(dirname "$0")/../../../scripts/env.sh"
set -e
cd "$(dirname "$0")/../../.."
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=75
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source install-sim/setup.bash
prefix="${1:-log/diagnostics_$(date +%Y%m%d_%H%M%S)}"
for scenario in freefall force_x force_y force_z; do
  ros2 run dragonfly_flight flight --mode ideal_wrench --scenario "$scenario" --output "${prefix}_$scenario"
done
for step in .002 .001 .0005; do
  ros2 run dragonfly_flight flight --mode physics --scenario physics --step "$step" --output "${prefix}_momentum_$step"
done
