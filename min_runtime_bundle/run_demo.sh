#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PYTHON_BIN="${PYTHON_BIN:-python}"
SCENE_KM="${SCENE_KM:-4}"
INTERCEPT_MODE="${INTERCEPT_MODE:-hit}"
UI_STYLE="${UI_STYLE:-rect}"
FULLSCREEN="${FULLSCREEN:-0}"
DEMO_INTERFERENCE_ENABLE="${DEMO_INTERFERENCE_ENABLE:-1}"
DEMO_INTERFERENCE_VISIBLE="${DEMO_INTERFERENCE_VISIBLE:-1}"
DEMO_SCHEME="${DEMO_SCHEME:-0}" # 0=启动后右上角点1/2/3; 1=传统; 2=协同; 3=干扰失联
LLM_DASHBOARD="${LLM_DASHBOARD:-1}"
LLM_DASHBOARD_HOST="${LLM_DASHBOARD_HOST:-127.0.0.1}"
LLM_DASHBOARD_PORT="${LLM_DASHBOARD_PORT:-8765}"
LLM_DASHBOARD_OPEN="${LLM_DASHBOARD_OPEN:-1}"
ENABLE_PUBLISH_REDIS="${ENABLE_PUBLISH_REDIS:-1}"
PUBLISH_INTERVAL="${PUBLISH_INTERVAL:-0.07}"
FRIENDLY_START="${FRIENDLY_START:-1}"
ENEMY_START="${ENEMY_START:-100}"
REDIS_HOST="${REDIS_HOST:-127.0.0.1}"
REDIS_PORT="${REDIS_PORT:-6379}"
REDIS_DB="${REDIS_DB:-0}"

CMD=(
  "${PYTHON_BIN}" marl_main.py
  --mode demo
  --source demo
  --scene-km "${SCENE_KM}"
  --intercept-mode "${INTERCEPT_MODE}"
  --ui-style "${UI_STYLE}"
  --redis-host "${REDIS_HOST}"
  --redis-port "${REDIS_PORT}"
  --redis-db "${REDIS_DB}"
  --demo-interference-enable "${DEMO_INTERFERENCE_ENABLE}"
  --demo-interference-visible "${DEMO_INTERFERENCE_VISIBLE}"
  --demo-scheme "${DEMO_SCHEME}"
  --llm-dashboard-host "${LLM_DASHBOARD_HOST}"
  --llm-dashboard-port "${LLM_DASHBOARD_PORT}"
  --publish-interval "${PUBLISH_INTERVAL}"
  --friendly-start "${FRIENDLY_START}"
  --enemy-start "${ENEMY_START}"
)

if [[ "${FULLSCREEN}" == "1" ]]; then
  CMD+=(--fullscreen)
fi

if [[ "${ENABLE_PUBLISH_REDIS}" == "1" ]]; then
  CMD+=(--publish-redis)
fi

if [[ "${LLM_DASHBOARD}" == "1" ]]; then
  CMD+=(--llm-dashboard)
fi

if [[ "${LLM_DASHBOARD_OPEN}" == "1" ]]; then
  CMD+=(--llm-dashboard-open)
fi

CMD+=("$@")

printf 'Running command:\n'
printf ' %q' "${CMD[@]}"
printf '\n'

exec "${CMD[@]}"
