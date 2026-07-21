#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

HOST="${HOST:-192.166.51.199}"             # 远端Redis地址: 现在这台是 192.166.51.199, IP变了就改这里
PORT="${PORT:-6379}"                       # Redis端口: 一般保持 6379
DB="${DB:-0}"                              # Redis库号: 一般保持 0
PASSWORD="${PASSWORD:-uav123}"             # Redis密码: 当前是 uav123
INTERVAL="${INTERVAL:-1.0}"                # 刷新间隔(秒): 1.0 比较适合盯实时变化
GEO_ORIGIN_LAT="${GEO_ORIGIN_LAT:-34.2134}" # 当前 hash -> UI 映射参考原点纬度
GEO_ORIGIN_LON="${GEO_ORIGIN_LON:-108.7632}" # 当前 hash -> UI 映射参考原点经度
TOP_K="${TOP_K:-10}"                       # 每轮最多打印多少条
ENEMY_ASSOC="${ENEMY_ASSOC:-off}"          # 监视时默认看原始 enemy_id, 所以先关本地关联器; 如果你想看 UI 里最终合并效果再改 on
ENEMY_HASH_REMAP_MODE="${ENEMY_HASH_REMAP_MODE:-inbound}" # 监视时默认也按“只看入侵段”显示
ENEMY_HASH_CENTER_X_RATIO="${ENEMY_HASH_CENTER_X_RATIO:-0.5}" # 监视时和主程序保持同一横向中心
ENEMY_HASH_LATERAL_SCALE="${ENEMY_HASH_LATERAL_SCALE:-1.0}"   # 监视时和主程序保持同一横向缩放
ENEMY_HASH_RANGE_SCALE="${ENEMY_HASH_RANGE_SCALE:-5.0}"       # 监视时和主程序保持同一纵深推进倍率
ENEMY_HASH_HIDE_OUTBOUND="${ENEMY_HASH_HIDE_OUTBOUND:-on}"    # 监视时也隐藏越过原点后的那段

cd "${SCRIPT_DIR}"

CMD=(
  "${PYTHON_BIN}" -m tools.watch_active_enemy_hash
  --host "${HOST}"
  --port "${PORT}"
  --db "${DB}"
  --interval "${INTERVAL}"
  --geo-origin-lat "${GEO_ORIGIN_LAT}"
  --geo-origin-lon "${GEO_ORIGIN_LON}"
  --top-k "${TOP_K}"
  --enemy-assoc "${ENEMY_ASSOC}"
  --enemy-hash-remap-mode "${ENEMY_HASH_REMAP_MODE}"
  --enemy-hash-center-x-ratio "${ENEMY_HASH_CENTER_X_RATIO}"
  --enemy-hash-lateral-scale "${ENEMY_HASH_LATERAL_SCALE}"
  --enemy-hash-range-scale "${ENEMY_HASH_RANGE_SCALE}"
  --enemy-hash-hide-outbound "${ENEMY_HASH_HIDE_OUTBOUND}"
)

if [[ -n "${PASSWORD}" ]]; then
  CMD+=(--password "${PASSWORD}")
fi

CMD+=("$@")

exec "${CMD[@]}"
