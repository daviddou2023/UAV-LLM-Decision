#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
python marl_friendlies_to_redis.py --source demo --scene-km 4 --intercept-mode hybrid --publish-interval 0.03 --loop --redis-host 127.0.0.1 --redis-port 6379 --redis-db 0
