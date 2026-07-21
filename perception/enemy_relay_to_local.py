"""
远端敌方 Redis -> 本地 Redis 中继

用途:
1. 从远端 Redis 读取敌方数据
2. 复用当前 voice_change 的战术映射/敌方本地关联逻辑
3. 将映射后的敌方写入本地 Redis，便于单机测试
"""

import argparse
import time

from perception.radar_feed import TeacherDataFeed
from integrations.redis_export import (
    RedisNodeWriter,
    build_teacher_redis_payload,
    stale_keys,
)


def _parse_bool(text):
    return str(text or "").strip().lower() in ("1", "true", "on", "yes")


def _parse_id_filter(text):
    tokens = []
    for piece in str(text or "").split(","):
        token = piece.strip()
        if token:
            tokens.append(token)
    return set(tokens)


def _track_matches_filter(track, allowed_ids):
    if not allowed_ids:
        return True
    raw = dict(track.get("raw", {}) or {})
    candidates = {
        str(track.get("external_id", "")).strip(),
        str(track.get("source_external_id", "")).strip(),
        str(raw.get("drone_id", "")).strip(),
        str(raw.get("id", "")).strip(),
        str(raw.get("slot_id", "")).strip(),
    }
    extra = set()
    for candidate in list(candidates):
        if not candidate:
            continue
        if candidate.startswith("slot-"):
            extra.add(candidate[len("slot-"):])
        if candidate.startswith("enemy"):
            extra.add(candidate[len("enemy"):])
        if candidate.startswith("uav"):
            extra.add(candidate[len("uav"):])
    candidates |= extra
    return any(candidate in allowed_ids for candidate in candidates if candidate)


def _enemy_entity(track, idx):
    return {
        "id": idx,
        "external_id": track.get("external_id"),
        "x": float(track.get("x", 0.0)),
        "y": float(track.get("y", 0.0)),
        "z": float(track.get("z", 0.0) or 0.0),
        "vz": float(track.get("vz", 0.0) or 0.0),
        "speed": float(track.get("speed", 0.0) or 0.0),
        "heading": float(track.get("heading", 90.0) or 90.0),
        "status_text": str(track.get("status", "")),
        "source": "relay",
    }


def _cleanup_range(writer, start_num, count):
    if count <= 0:
        return
    keys = []
    for node_num in range(start_num, start_num + count):
        keys.extend(
            [
                f"{node_num}_x",
                f"{node_num}_y",
                f"{node_num}_z",
                f"{node_num}_status",
                f"{node_num}_type",
                f"{node_num}_frame",
                f"{node_num}_timestamp",
            ]
        )
    writer.delete(keys)


def main():
    parser = argparse.ArgumentParser(description="远端敌方 Redis -> 本地 Redis 中继")
    parser.add_argument("--remote-host", default="192.166.51.23")
    parser.add_argument("--remote-port", type=int, default=6379)
    parser.add_argument("--remote-db", type=int, default=0)
    parser.add_argument("--remote-password", default=None)
    parser.add_argument("--remote-format", default="flat", choices=["auto", "flat", "hash"])
    parser.add_argument("--poll-interval", type=float, default=0.4)
    parser.add_argument("--geo-origin-lat", type=float, default=34.2134)
    parser.add_argument("--geo-origin-lon", type=float, default=108.7597)
    parser.add_argument("--enemy-assoc", default="on")
    parser.add_argument("--enemy-assoc-max-distance", type=float, default=450.0)
    parser.add_argument("--enemy-assoc-max-altitude", type=float, default=140.0)
    parser.add_argument("--enemy-assoc-keep-sec", type=float, default=18.0)
    parser.add_argument("--enemy-flat-rotate-deg", type=float, default=135.0)
    parser.add_argument("--enemy-flat-flip-x", default="off")
    parser.add_argument("--enemy-flat-flip-y", default="off")
    parser.add_argument("--enemy-flat-scale", type=float, default=1.1)
    parser.add_argument("--enemy-flat-center-x-ratio", type=float, default=0.7)
    parser.add_argument("--enemy-flat-center-y-ratio", type=float, default=0.2)

    parser.add_argument("--local-host", default="127.0.0.1")
    parser.add_argument("--local-port", type=int, default=6379)
    parser.add_argument("--local-db", type=int, default=0)
    parser.add_argument("--local-password", default=None)
    parser.add_argument("--enemy-start", type=int, default=101)
    parser.add_argument("--cleanup-count", type=int, default=120)

    parser.add_argument("--only-normal", default="off")
    parser.add_argument("--only-fresh", default="off")
    parser.add_argument("--source-id-filter", default="", help="只转存指定敌方源编号, 例如 275 或 275,276")
    parser.add_argument("--print-interval", type=float, default=2.0)
    args = parser.parse_args()

    feed = TeacherDataFeed(
        host=args.remote_host,
        port=args.remote_port,
        db=args.remote_db,
        password=args.remote_password,
        poll_interval=args.poll_interval,
        redis_format=args.remote_format,
        side_filter="enemy",
        geo_origin_lat=args.geo_origin_lat,
        geo_origin_lon=args.geo_origin_lon,
        enemy_assoc=args.enemy_assoc,
        enemy_assoc_max_distance=args.enemy_assoc_max_distance,
        enemy_assoc_max_altitude=args.enemy_assoc_max_altitude,
        enemy_assoc_keep_sec=args.enemy_assoc_keep_sec,
        enemy_flat_rotate_deg=args.enemy_flat_rotate_deg,
        enemy_flat_flip_x=args.enemy_flat_flip_x,
        enemy_flat_flip_y=args.enemy_flat_flip_y,
        enemy_flat_scale=args.enemy_flat_scale,
        enemy_flat_center_x_ratio=args.enemy_flat_center_x_ratio,
        enemy_flat_center_y_ratio=args.enemy_flat_center_y_ratio,
    )
    writer = RedisNodeWriter(
        host=args.local_host,
        port=args.local_port,
        db=args.local_db,
        password=args.local_password,
    )

    only_normal = _parse_bool(args.only_normal)
    only_fresh = _parse_bool(args.only_fresh)
    source_id_filter = _parse_id_filter(args.source_id_filter)
    published_nodes = set()
    frame_num = 0
    last_print = 0.0

    _cleanup_range(writer, args.enemy_start, args.cleanup_count)
    print(
        "Enemy relay started | "
        f"remote={args.remote_host}:{args.remote_port}/{args.remote_db} "
        f"format={args.remote_format} -> "
        f"local={args.local_host}:{args.local_port}/{args.local_db} "
        f"enemy_start={args.enemy_start}"
    )

    try:
        while True:
            snapshot = feed.poll(time.time())
            enemies = list(snapshot.get("enemies", []))

            if source_id_filter:
                enemies = [enemy for enemy in enemies if _track_matches_filter(enemy, source_id_filter)]
            if only_normal:
                enemies = [enemy for enemy in enemies if str(enemy.get("status", "")).strip().lower() == "normal"]
            if only_fresh:
                enemies = [
                    enemy
                    for enemy in enemies
                    if not enemy.get("stale", False) and not enemy.get("lost", False)
                ]

            rows = []
            for idx, track in enumerate(enemies):
                rows.append((args.enemy_start + idx, _enemy_entity(track, idx)))

            now = time.time()
            frame_num += 1
            payload, active_nodes = build_teacher_redis_payload([], rows, frame_num, now)
            writer.mset(payload)
            writer.delete(stale_keys(published_nodes, active_nodes))
            published_nodes = active_nodes

            if now - last_print >= max(0.2, args.print_interval):
                meta = snapshot.get("meta", {}) or {}
                print(
                    f"[relay] enemies={len(enemies)} "
                    f"diag={meta.get('diag', '')} "
                    f"frame={frame_num}"
                )
                last_print = now

            time.sleep(max(0.03, args.poll_interval))
    except KeyboardInterrupt:
        print("\nEnemy relay stopped.")
    finally:
        if published_nodes:
            writer.delete(stale_keys(published_nodes, set()))


if __name__ == "__main__":
    main()
