#!/usr/bin/env bash
source "$(dirname "$0")/../../../scripts/env.sh"
set -e
cd "$(dirname "$0")/../../.."
export ROS_DOMAIN_ID=75
prefix="${1:-log/momentum_$(date +%Y%m%d_%H%M%S)}"
for step in .002 .001 .0005; do
  bash src/dragonfly_flight/scripts/sim.sh --mode physics --scenario physics --step "$step" --output "${prefix}_$step"
done
