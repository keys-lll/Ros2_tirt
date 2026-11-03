#!/usr/bin/env bash
set -eo pipefail
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$task_ws"
colcon build --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF

