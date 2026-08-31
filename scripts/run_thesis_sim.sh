#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "install/setup.bash 不存在，请先运行 ./scripts/build_thesis.sh" >&2
  exit 1
fi

source "$workspace/install/setup.bash"
exec ros2 launch rangermini_dynamic_semantic thesis_benchmark.launch.py "$@"
