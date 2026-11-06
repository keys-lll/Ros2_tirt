#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$task_root/diagnostics/test_controller_10hz.sh" "$@"
