#!/usr/bin/env bash
# Start navigation + passive recording, or a terminal mission with a TTY log.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/scripts/env.sh"
logs_root="$task_root/test_logs"
mkdir -p "$logs_root"
mode=${1:-nav}
if [[ "$mode" == nav ]]; then
    exec 9>/tmp/mecanum_10hz_test.lock
    flock -n 9 || { echo '測試導航腳本已在執行'; exit 1; }
    run_dir=$(mktemp -d "$logs_root/$(date +%Y%m%d_%H%M%S)_10hz_XXXX")
    printf '%s\n' "$run_dir" > "$logs_root/latest_test.txt"
    export ROS_LOG_DIR="$run_dir/ros"
    mkdir -p "$ROS_LOG_DIR"
    cp "$TIRT_WORKSPACE/src/mecanum_navigation/config/nav2_params.yaml" "$run_dir/nav2_params.yaml"
    cp "$TIRT_WORKSPACE/src/car_waypoints/config/waypoints.yaml" "$run_dir/waypoints.yaml"
    cat > "$run_dir/bag_qos.yaml" <<'YAML'
/tf_static:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 100
YAML
    ros2 bag record --include-hidden-topics -o "$run_dir/bag" --qos-profile-overrides-path "$run_dir/bag_qos.yaml" \
        /tf /tf_static /scan_raw /scan /stm32/odom /stm32/imu \
        /odom /odometry/filtered /amcl_pose /cmd_vel_nav /cmd_vel \
        /plan /local_plan /rosout /navigate_to_pose/_action/status \
        > "$run_dir/recorder.log" 2>&1 9>&- &
    recorder_pid=$!
    cleanup() {
        trap - EXIT
        kill -INT "$recorder_pid" 2>/dev/null || true
        wait "$recorder_pid" 2>/dev/null || true
        echo "測試紀錄：$run_dir"
    }
    trap cleanup EXIT
    sleep 1
    kill -0 "$recorder_pid" 2>/dev/null || { cat "$run_dir/recorder.log"; exit 1; }
    echo "10 Hz 測試；請先將車放在 start。紀錄：$run_dir"
    ros2 launch mecanum_navigation navigation.launch.py \
        map:="$TIRT_WORKSPACE/maps/mecanum_map.yaml" \
        use_rviz:=false initial_waypoint:=start 9>&- 2>&1 | tee "$run_dir/navigation.log"
elif [[ "$mode" == mission || "$mode" == gui ]]; then
    [[ -r "$logs_root/latest_test.txt" ]] || { echo '請先執行本腳本 nav'; exit 1; }
    IFS= read -r run_dir < "$logs_root/latest_test.txt"
    export ROS_LOG_DIR="$run_dir/ros"
    if [[ "$mode" == gui ]]; then
        echo "GUI 任務紀錄：$run_dir/mission_gui.log"
        script -q -e -f -c 'ros2 run auto mission_gui' "$run_dir/mission_gui.log"
    else
        echo "任務紀錄：$run_dir/mission.log"
        script -q -e -f -c 'ros2 run auto run_mission' "$run_dir/mission.log"
    fi
else
    echo "用法：bash $0 nav | mission | gui" >&2
    exit 2
fi
