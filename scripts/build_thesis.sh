#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$workspace"

colcon build --symlink-install --packages-up-to \
  ranger_nav rangermini_dynamic_semantic ranger_nav_metrics
