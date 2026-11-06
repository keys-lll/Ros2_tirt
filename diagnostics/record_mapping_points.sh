#!/usr/bin/env bash
# Run alongside mapping: save start, prep, t1-t5 and the two transit points.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/scripts/env.sh"
echo '請先啟動建圖，等待 map → base_link 定位可用。'
echo '開到備餐區前方輸入 a；開到 5 號桌旁輸入 b；list 檢查；q 離開。'
exec ros2 run car_waypoints wp_record
