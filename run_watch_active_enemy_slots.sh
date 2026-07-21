#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

HOST="${HOST:-192.166.51.23}"            # 远端Redis地址: 23号机IP变了就改这里
PORT="${PORT:-6379}"                     # Redis端口: 一般保持 6379
DB="${DB:-0}"                            # Redis库号: 一般保持 0
PASSWORD="${PASSWORD:-uav123}"           # Redis密码: 没密码就留空
INTERVAL="${INTERVAL:-2.0}"              # 轮询间隔(秒): 小一点更实时, 大一点更不刷屏
FRESH_SEC="${FRESH_SEC:-10.0}"           # 多久内算“活数据”: 默认 10 秒; 如果对方刷得慢可改成 20/30
TOP_K="${TOP_K:-10}"                     # 每轮最多打印多少条

cd "${SCRIPT_DIR}"

CMD=(
  "${PYTHON_BIN}" -m tools.watch_active_enemy_slots
  --host "${HOST}"
  --port "${PORT}"
  --db "${DB}"
  --interval "${INTERVAL}"
  --fresh-sec "${FRESH_SEC}"
  --top-k "${TOP_K}"
)

if [[ -n "${PASSWORD}" ]]; then
  CMD+=(--password "${PASSWORD}")
fi

CMD+=("$@")

exec "${CMD[@]}"
