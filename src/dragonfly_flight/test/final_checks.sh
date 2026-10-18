#!/usr/bin/env bash
source "$(dirname "$0")/../../../scripts/env.sh"
set -e
cd "$(dirname "$0")/../../.."
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=0
source "$ROS_SETUP"
source "$GZ_CONTROL_WS/install/setup.bash"
source "$PX4_ROS_WS/install/setup.bash"
source install-flight/setup.bash
ros2 run dragonfly_flight flight --scenario acceptance --record --output log/effort_acceptance_4
ros2 run dragonfly_flight verify log/effort_acceptance_4
python3 src/dragonfly_flight/test/plot_run.py log/effort_acceptance_4
python3 src/dragonfly_flight/test/probe_dds_loss.py log/dds_loss_verified
