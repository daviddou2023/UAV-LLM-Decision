# Minimal Runtime Bundle

This folder is the smallest self-contained runtime set for the current demo path.

Included:
- GUI demo
- Redis export
- Task assignment
- Deconfliction
- Reassignment
- Teacher data adapter
- LLM analyst
- Optional voice path

## Files

- `marl_main.py`: main GUI demo entry
- `marl_friendlies_to_redis.py`: headless Redis/CSV exporter
- `marl_common.py`: config and entity model
- `marl_cooperation.py`: assignment core
- `marl_reassignment.py`: stable retask policy
- `marl_deconfliction.py`: deconfliction and detour logic
- `marl_data.py`: teacher Redis feed adapter
- `marl_redis_export.py`: shared Redis publish helpers
- `marl_llm_kit.py`: analyst / LLM bridge
- `marl_ui.py`: renderer
- `marl_voice.py`: optional voice client
- `whisper_server.py`: optional whisper service

## Basic Run

Change into this folder first:

```bash
cd /opt/Wireless_Embodied_AI/Embodied_Intellgence/LLM_Decision/11LLMctrl/voice_change/min_runtime_bundle
```

### 1. GUI demo only

```bash
python marl_main.py --mode demo --source demo --scene-km 4 --intercept-mode hybrid --ui-style rect --fullscreen
```

After the window opens, press `Enter` to start.

### 2. Redis export only

```bash
python marl_friendlies_to_redis.py --source demo --scene-km 4 --intercept-mode hybrid --publish-interval 0.03 --loop --redis-host 127.0.0.1 --redis-port 6379 --redis-db 0
```

Friendly nodes now publish:

```text
n_x n_y n_z n_status n_frame n_timestamp n_battery
```

Enemy nodes publish:

```text
n_x n_y n_z n_status n_frame n_timestamp
```

### 3. GUI + same-frame Redis sync

```bash
python marl_main.py --mode demo --source demo --scene-km 4 --intercept-mode hybrid --ui-style rect --fullscreen --publish-redis --publish-interval 0.5 --redis-host 127.0.0.1 --redis-port 6379 --redis-db 0 --friendly-start 1 --enemy-start 101
```

After the window opens, press `Enter` to start.

## Redis Check

Local check:

```bash
redis-cli -h 127.0.0.1 -p 6379 -n 0 MGET 1_x 1_y 1_z 1_status 1_frame 1_timestamp 1_battery
```

Teacher remote check:

```bash
redis-cli -h 192.166.51.18 -p 6379 -n 0 MGET 101_x 101_y 101_z 101_status 101_frame 101_timestamp
```

## Optional Voice

Start whisper service first if you need voice:

```bash
python whisper_server.py --model medium --port 5555
```

If the machine is weak:

```bash
python whisper_server.py --model small --port 5555
```
