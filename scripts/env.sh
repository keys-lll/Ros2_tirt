#!/usr/bin/env bash
# Source this file before using ROS commands from this workspace.
export TIRT_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$TIRT_WORKSPACE/install/setup.bash" ]]; then
    echo '請先在 Ros2_tirt 執行 bash scripts/build.sh' >&2
    return 1 2>/dev/null || exit 1
fi
source "$TIRT_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}

