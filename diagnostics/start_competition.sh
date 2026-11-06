#!/usr/bin/env bash
# Open navigation and mission in separate terminal windows with one command.
set -eo pipefail
trap 'status=$?; echo "啟動失敗：第 $LINENO 行，錯誤碼 $status。" >&2; exit "$status"' ERR
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runner="$script_dir/test_controller_10hz.sh"
latest="$script_dir/../test_logs/latest_test.txt"

if [[ "${1:-}" == --mission ]]; then
    marker=$2
    trap 'rm -f -- "$marker"' EXIT
    echo '等待本次導航啟動完成，再開啟任務與相機……'
    deadline=$((SECONDS + 120))
    while (( SECONDS < deadline )); do
        if [[ -r "$latest" && "$latest" -nt "$marker" ]]; then
            IFS= read -r run_dir < "$latest"
            if [[ -r "$run_dir/navigation.log" ]] &&
                grep -q 'lifecycle_manager_navigation.*Managed nodes are active' "$run_dir/navigation.log"; then
                rm -f -- "$marker"
                trap - EXIT
                if [[ "${3:-mission}" == gui ]]; then
                    /usr/bin/python3 "$script_dir/mission_window.py" --gui &
                else
                    /usr/bin/python3 "$script_dir/mission_window.py" &
                fi
                exec bash "$runner" "${3:-mission}"
            fi
        fi
        sleep 0.5
    done
    echo '等待導航超過 120 秒，未啟動任務。請查看導航視窗的錯誤。'
    read -r -p '按 Enter 關閉此視窗。' _ || true
    exit 1
fi

mission_mode=${1:-mission}
if [[ "$mission_mode" != mission && "$mission_mode" != gui ]]; then
    echo "用法：bash $0 [mission|gui]" >&2
    exit 1
fi
for command_name in gnome-terminal grep flock mktemp; do
    if ! command -v "$command_name" >/dev/null; then
        echo "缺少必要指令：$command_name，無法啟動。" >&2
        exit 1
    fi
done
if [[ ! -r "$runner" ]]; then
    echo "找不到導航／任務腳本：$runner" >&2
    exit 1
fi
if ! flock -n /tmp/mecanum_10hz_test.lock true; then
    echo '導航腳本已在執行；請先停止原本的導航與任務再啟動。' >&2
    exit 1
fi
marker=$(mktemp /tmp/mecanum_competition_start.XXXXXX)
echo '請先將車放在 start 位置與朝向。正在開啟導航及任務視窗。'
if ! gnome-terminal --window --title='導航與紀錄' -- bash "$runner" nav; then
    rm -f -- "$marker"
    exit 1
fi
if ! gnome-terminal --window --title='比賽任務' -- bash "$script_dir/start_competition.sh" --mission "$marker" "$mission_mode"; then
    rm -f -- "$marker"
    exit 1
fi
