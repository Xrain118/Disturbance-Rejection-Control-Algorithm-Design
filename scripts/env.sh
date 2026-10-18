#!/usr/bin/env bash
# Source this file before build/run. Optional machine-local overrides (not committed).
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ ! -f "$PROJECT_ROOT/config/local.env" ]] || source "$PROJECT_ROOT/config/local.env"
export PROJECT_ROOT
export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export GZ_CONTROL_WS="${GZ_CONTROL_WS:-$HOME/gz_ros2_control_ws}"
export PX4_ROS_WS="${PX4_ROS_WS:-$HOME/px4_ros2_ws}"
export PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
export XRCE_AGENT_ROOT="${XRCE_AGENT_ROOT:-$HOME/Micro-XRCE-DDS-Agent}"
export PYTHONNOUSERSITE=1
