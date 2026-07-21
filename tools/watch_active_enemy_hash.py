"""
实时观察远端 hash 敌方，并打印“映射到本机 UI 之前”的内部坐标。

用途:
1. 直接读取 `enemy:status:*`
2. 只保留当前没 lost 的敌方
3. 打印:
   - 敌方数量
   - 哪些 enemy_id 是新出现/变化了
   - 当前会送进主程序的 x/y/z
"""

import argparse
import time

from perception.radar_feed import TeacherDataFeed


WATCH_FIELDS = ("x", "y", "z", "status", "frame", "stamp")


def _track_changed(prev_track, curr_track):
    if prev_track is None:
        return True
    for field in WATCH_FIELDS:
        if prev_track.get(field) != curr_track.get(field):
            return True
    return False


def _fmt(track):
    return (
        f"{track['external_id']}: "
        f"x={track['x']:.3f} y={track['y']:.3f} z={track['z']:.3f} "
        f"status={track.get('status')} age={track.get('age', 0.0):.2f}s "
        f"stale={track.get('stale')} lost={track.get('lost')}"
    )


def main():
    parser = argparse.ArgumentParser(description="实时观察远端 hash 敌方并打印映射后的内部坐标")
    parser.add_argument("--host", default="192.166.51.199")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--db", type=int, default=0)
    parser.add_argument("--password", default="uav123")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--geo-origin-lat", type=float, default=34.2134)
    parser.add_argument("--geo-origin-lon", type=float, default=108.7632)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--enemy-assoc", choices=("on", "off"), default="off")
    parser.add_argument("--enemy-hash-remap-mode", choices=("direct", "inbound"), default="inbound")
    parser.add_argument("--enemy-hash-center-x-ratio", type=float, default=0.5)
    parser.add_argument("--enemy-hash-lateral-scale", type=float, default=1.0)
    parser.add_argument("--enemy-hash-range-scale", type=float, default=5.0)
    parser.add_argument("--enemy-hash-hide-outbound", choices=("on", "off"), default="on")
    args = parser.parse_args()

    feed = TeacherDataFeed(
        host=args.host,
        port=args.port,
        db=args.db,
        password=args.password,
        redis_format="hash",
        side_filter="enemy",
        geo_origin_lat=args.geo_origin_lat,
        geo_origin_lon=args.geo_origin_lon,
        enemy_assoc=args.enemy_assoc,
        enemy_hash_remap_mode=args.enemy_hash_remap_mode,
        enemy_hash_center_x_ratio=args.enemy_hash_center_x_ratio,
        enemy_hash_lateral_scale=args.enemy_hash_lateral_scale,
        enemy_hash_range_scale=args.enemy_hash_range_scale,
        enemy_hash_hide_outbound=args.enemy_hash_hide_outbound,
    )

    previous = {}
    print(
        "Watching active enemy hash | "
        f"redis={args.host}:{args.port}/{args.db} interval={args.interval:.1f}s "
        f"origin=({args.geo_origin_lat},{args.geo_origin_lon}) assoc={args.enemy_assoc} "
        f"remap={args.enemy_hash_remap_mode}"
    )

    try:
        while True:
            snap = feed.poll(0.0)
            enemies = list(snap.get("enemies", []))
            changed = [track for track in enemies if _track_changed(previous.get(track["external_id"]), track)]
            changed.sort(key=lambda item: item["external_id"])
            enemies.sort(key=lambda item: item["external_id"])

            now_text = time.strftime("%H:%M:%S", time.localtime())
            print(f"\n[{now_text}] enemies={len(enemies)} diag={snap.get('meta', {}).get('diag')}")
            if enemies:
                print("  Active enemies:")
                for track in enemies[: max(1, args.top_k)]:
                    print("   ", _fmt(track))
            else:
                print("  Active enemies: none")

            if changed:
                print("  Changed this round:")
                for track in changed[: max(1, args.top_k)]:
                    print("   ", _fmt(track))
            else:
                print("  Changed this round: none")

            previous = {track["external_id"]: dict(track) for track in enemies}
            time.sleep(max(0.5, float(args.interval)))
    except KeyboardInterrupt:
        print("\nWatch stopped.")


if __name__ == "__main__":
    main()
