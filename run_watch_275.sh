#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

HOST="${HOST:-192.166.51.23}"              # 远端Redis IP: 23号机IP变了就改这里
PORT="${PORT:-6379}"                       # 远端Redis端口: 一般保持 6379
DB="${DB:-0}"                              # 远端Redis库号: 一般保持 0
PASSWORD="${PASSWORD:-uav123}"             # 远端Redis密码: 没密码就留空
SLOT_ID="${SLOT_ID:-275}"                  # 监视哪个 flat 槽位: 当前盯 275, 想看别的就改成 276/301 等
INTERVAL="${INTERVAL:-1.0}"                # 轮询周期(秒): 小一点更实时, 大一点刷屏更少

cd "${SCRIPT_DIR}"

CMD=(
  "${PYTHON_BIN}" -m tools.watch_enemy_slot_updates
  --host "${HOST}"
  --port "${PORT}"
  --db "${DB}"
  --slot-id "${SLOT_ID}"
  --interval "${INTERVAL}"
)

if [[ -n "${PASSWORD}" ]]; then
  CMD+=(--password "${PASSWORD}")
fi

CMD+=("$@")

exec "${CMD[@]}"
