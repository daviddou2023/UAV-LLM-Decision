#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
python marl_main.py --mode demo --source demo --scene-km 4 --intercept-mode hybrid --ui-style rect --fullscreen --publish-redis --publish-interval 0.5 --redis-host 127.0.0.1 --redis-port 6379 --redis-db 0 --friendly-start 1 --enemy-start 101
