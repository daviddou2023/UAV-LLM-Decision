"""
敌我无人机数据导出器

作用:
1. 无界面运行当前拦截仿真
2. 将己方和敌方状态按要求的 Redis 键格式持续写出
3. 同步落一份 CSV，便于留档与回放
4. 友方/敌方使用同一套 n_x/n_y/n_z/n_status/n_frame/n_timestamp
"""
import argparse
import csv
import time

from core.common import CFG, EState, IState
from simulation.main import InterceptionEnvironment
from integrations.redis_export import (
    RedisNodeWriter,
    build_geo_hash_payload,
    build_payload,
    build_teacher_redis_payload,
    enemy_rows as build_enemy_rows,
    enemy_status,
    friendly_rows as build_friendly_rows,
    friendly_status,
    planned_node_nums,
    stale_keys,
)
from perception.udp_gateway import UDPFramePublisher


class CSVWriter:
    def __init__(self, path, append=False):
        self.path = path
        mode = "a" if append else "w"
        self.fp = open(path, mode, newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.fp,
            fieldnames=[
                "export_frame",
                "cycle_index",
                "sim_time",
                "side",
                "node_num",
                "node_id",
                "x",
                "y",
                "z",
                "battery",
                "status",
                "node_frame",
                "timestamp",
            ],
        )
        if not append or self.fp.tell() == 0:
            self.writer.writeheader()
            self.fp.flush()

    def write_rows(self, interceptors, enemies, frame_num, cycle_index, sim_time, stamp):
        raise RuntimeError("write_rows requires explicit node numbering")

    def write_numbered_rows(self, friendly_rows, enemy_rows, frame_num, cycle_index, sim_time, stamp):
        for node_num, node_id, entity in friendly_rows:
            self.writer.writerow(
                {
                    "export_frame": frame_num,
                    "cycle_index": cycle_index,
                    "sim_time": f"{sim_time:.3f}",
                    "side": "friendly",
                    "node_num": node_num,
                    "node_id": node_id,
                    "x": f"{entity['x']:.3f}",
                    "y": f"{entity['y']:.3f}",
                    "z": f"{entity.get('z', 0.0):.3f}",
                    "battery": f"{max(0.0, entity.get('fuel', 0.0)):.2f}",
                    "status": friendly_status(entity),
                    "node_frame": frame_num,
                    "timestamp": f"{stamp:.6f}",
                }
            )
        for node_num, node_id, entity in enemy_rows:
            self.writer.writerow(
                {
                    "export_frame": frame_num,
                    "cycle_index": cycle_index,
                    "sim_time": f"{sim_time:.3f}",
                    "side": "enemy",
                    "node_num": node_num,
                    "node_id": node_id,
                    "x": f"{entity['x']:.3f}",
                    "y": f"{entity['y']:.3f}",
                    "z": f"{entity.get('z', 0.0):.3f}",
                    "battery": "",
                    "status": enemy_status(entity),
                    "node_frame": frame_num,
                    "timestamp": f"{stamp:.6f}",
                }
            )
        self.fp.flush()

    def close(self):
        self.fp.close()


def _friendly_id(node_num):
    return f"Friendly_uav-{node_num}"


def _enemy_id(node_num):
    return f"Enemy-{node_num}"


def _friendly_rows(interceptors, friendly_start):
    rows = []
    for idx, (node_num, entity) in enumerate(build_friendly_rows(interceptors, friendly_start)):
        rows.append((node_num, _friendly_id(idx + 1), entity))
    return rows


def _enemy_rows(enemies, enemy_start):
    rows = []
    for idx, (node_num, entity) in enumerate(build_enemy_rows(enemies, enemy_start)):
        rows.append((node_num, _enemy_id(idx + 1), entity))
    return rows


def run_export(
    seed=42,
    scene_km=5.0,
    source="demo",
    intercept_mode="hybrid",
    demo_case=None,
    redis_host="127.0.0.1",
    redis_port=6379,
    redis_db=0,
    redis_password=None,
    publish_interval=0.03,
    sim_speed=1.0,
    loop=False,
    max_frames=0,
    csv_path="air_uav_export.csv",
    csv_append=False,
    friendly_start=1,
    enemy_start=101,
    redis_format="default",
    publish_udp=False,
    udp_out_host="127.0.0.1",
    udp_out_port=9999,
    udp_enemy_only=False,
    publish_udp_mode="teacher",
    udp_in_host="0.0.0.0",
    udp_in_port=8020,
    enemy_redis_format="auto",
    friendly_return_source="udp",
    geo_origin_lat=34.2663,
    geo_origin_lon=108.9549,
):
    env = InterceptionEnvironment(
        seed=seed,
        scene_km=scene_km,
        source=source,
        redis_host=redis_host,
        redis_port=redis_port,
        redis_db=redis_db,
        redis_password=redis_password,
        intercept_mode=intercept_mode,
        demo_case=demo_case,
        udp_in_host=udp_in_host,
        udp_in_port=udp_in_port,
        enemy_redis_format=enemy_redis_format,
        friendly_return_source=friendly_return_source,
        geo_origin_lat=geo_origin_lat,
        geo_origin_lon=geo_origin_lon,
    )
    teacher_compat_mode = (redis_format == "teacher-friendly")
    geo_hash_mode = (redis_format == "geo-hash")
    if teacher_compat_mode:
        if publish_udp:
            udp_enemy_only = True
    writer = RedisNodeWriter(host=redis_host, port=redis_port, db=redis_db, password=redis_password)
    cleanup_node_nums = set()
    if not teacher_compat_mode and not geo_hash_mode:
        writer.cleanup_legacy_keys()
        cleanup_node_nums = planned_node_nums(
            interceptor_count=len(env.interceptors),
            total_enemy_count=env.stats.get("total_enemies", 0),
            live_enemy_count=len(env.enemies),
            friendly_start=friendly_start,
            enemy_start=enemy_start,
        )
        writer.cleanup_node_nums(cleanup_node_nums)
    csv_writer = CSVWriter(csv_path, append=csv_append)
    udp_publisher = None
    publish_interval = max(0.03, float(publish_interval))
    sim_speed = max(0.1, float(sim_speed))
    steps_per_publish = max(1, int(round((publish_interval * sim_speed) / CFG.DT)))

    if publish_udp:
        udp_publisher = UDPFramePublisher(
            host=udp_out_host,
            port=udp_out_port,
            publish_interval=publish_interval,
            enemy_only=udp_enemy_only,
            friendly_start=friendly_start,
            enemy_start=enemy_start,
            publish_mode=publish_udp_mode,
            geo_origin_lat=geo_origin_lat,
            geo_origin_lon=geo_origin_lon,
        )

    frame_num = 0
    cycle_index = 1
    last_print = 0.0
    published_nodes = set()

    friendly_range_end = friendly_start + len(env.interceptors) - 1
    print("=" * 60)
    print("Air UAV Redis Exporter")
    print(
        f"Scene={scene_km:.0f}km Source={source} Mode={intercept_mode} "
        f"Case={demo_case or 'default'} Redis={redis_host}:{redis_port}/{redis_db}"
    )
    print(f"Publish every {publish_interval:.2f}s | sim_speed={sim_speed:.1f}x | loop={loop}")
    if teacher_compat_mode:
        print(
            f"Teacher兼容模式: 己方编号段 {friendly_start}-{friendly_range_end} | "
            f"敌方编号从 {enemy_start} 开始 | status=normal/unnormal | type=ally/enemy"
        )
    elif geo_hash_mode:
        print(
            f"Geo Hash模式: uav:status:uav* / enemy:status:enemy* | "
            f"origin=({geo_origin_lat:.4f},{geo_origin_lon:.4f})"
        )
    else:
        print(f"Friendly_uav 编号段: {friendly_start}-{friendly_range_end} -> {friendly_start}_x ... {friendly_range_end}_timestamp")
        print(f"Enemy 编号段: 从 {enemy_start} 开始按活跃敌机顺序占用 -> {enemy_start}_x ...")
        print("Redis keys format: n_x n_y n_z n_status n_frame n_timestamp friendly:n_battery")
    if publish_udp:
        print(
            f"UDP output: {udp_out_host}:{udp_out_port} | "
            f"{'enemy-only' if udp_enemy_only else 'enemy+friendly'} | mode={publish_udp_mode}"
        )
    print(f"CSV output: {csv_path}")
    print("=" * 60)

    try:
        while True:
            tick_start = time.time()

            if not env.done:
                for _ in range(steps_per_publish):
                    env.step(CFG.DT)
                    if env.done:
                        break
            elif loop:
                env.seed += 1
                env.reset()
                cycle_index += 1

            frame_num += 1
            stamp = time.time()
            friendly_rows = _friendly_rows(env.interceptors, friendly_start)
            enemy_rows = _enemy_rows(env.enemies, enemy_start)
            if teacher_compat_mode:
                payload, active_nodes = build_teacher_redis_payload(
                    [(node_num, entity) for node_num, _, entity in friendly_rows],
                    [(node_num, entity) for node_num, _, entity in enemy_rows],
                    frame_num,
                    stamp,
                )
                writer.mset(payload)
            elif geo_hash_mode:
                payload, active_nodes = build_geo_hash_payload(
                    [(node_num, entity) for node_num, _, entity in friendly_rows],
                    [(node_num, entity) for node_num, _, entity in enemy_rows],
                    frame_num,
                    stamp,
                    geo_reference=env.geo_reference,
                )
                writer.bulk_hset(payload)
            else:
                payload, active_nodes = build_payload(
                    [(node_num, entity) for node_num, _, entity in friendly_rows],
                    [(node_num, entity) for node_num, _, entity in enemy_rows],
                    frame_num,
                    stamp,
                )
                writer.mset(payload)
            if geo_hash_mode:
                stale_hashes = published_nodes - active_nodes
                if stale_hashes:
                    writer.delete(sorted(stale_hashes))
            elif not teacher_compat_mode:
                writer.delete(stale_keys(published_nodes, active_nodes))
            published_nodes = active_nodes
            if udp_publisher:
                udp_publisher.maybe_publish(env, force=True)
            csv_writer.write_numbered_rows(friendly_rows, enemy_rows, frame_num, cycle_index, env.time, stamp)

            now = time.time()
            if now - last_print >= 1.0:
                active_friendlies = sum(
                    1 for entity in env.interceptors
                    if entity["state"] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
                )
                alive_enemies = sum(1 for entity in env.enemies if entity["state"] != EState.DESTROYED)
                print(
                    f"frame={frame_num} cycle={cycle_index} sim_t={env.time:.1f}s "
                    f"friendly_active={active_friendlies} enemy_alive={alive_enemies} "
                    f"kills={env.stats['kills']} pen={env.stats['penetrations']}"
                )
                last_print = now

            sleep_left = publish_interval - (time.time() - tick_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

            if max_frames and frame_num >= max_frames:
                if geo_hash_mode and published_nodes:
                    writer.delete(sorted(published_nodes))
                elif not teacher_compat_mode:
                    writer.cleanup_node_nums(cleanup_node_nums | published_nodes)
                published_nodes = set()
                print("max_frames reached; last frame published")
                break

            if env.done and not loop:
                if geo_hash_mode and published_nodes:
                    writer.delete(sorted(published_nodes))
                elif not teacher_compat_mode:
                    writer.cleanup_node_nums(cleanup_node_nums | published_nodes)
                published_nodes = set()
                print("simulation finished; last frame published")
                break
    finally:
        if geo_hash_mode and published_nodes:
            writer.delete(sorted(published_nodes))
        elif not teacher_compat_mode:
            writer.cleanup_node_nums(cleanup_node_nums | published_nodes)
        if udp_publisher:
            udp_publisher.close()
        if getattr(env.feed, "close", None):
            try:
                env.feed.close()
            except Exception:
                pass
        csv_writer.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--scene-km", type=float, default=5.0)
    parser.add_argument("--source", default="demo", choices=["auto", "redis", "udp", "fusion", "demo"])
    parser.add_argument("--intercept-mode", default="hybrid", choices=["hybrid", "hit", "net", "legacy-net"])
    parser.add_argument("--demo-case", default=None, choices=["net-single", "barrier-single"])
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-password", default=None)
    parser.add_argument("--enemy-redis-format", default="auto", choices=["auto", "flat", "hash"])
    parser.add_argument("--friendly-return-source", default="udp", choices=["udp", "redis"])
    parser.add_argument("--geo-origin-lat", type=float, default=34.2663)
    parser.add_argument("--geo-origin-lon", type=float, default=108.9549)
    parser.add_argument("--publish-interval", type=float, default=0.5)
    parser.add_argument("--sim-speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--csv-path", default="air_uav_export.csv")
    parser.add_argument("--csv-append", action="store_true")
    parser.add_argument("--friendly-start", type=int, default=1)
    parser.add_argument("--enemy-start", type=int, default=101)
    parser.add_argument("--redis-format", default="default", choices=["default", "teacher-friendly", "geo-hash"])
    parser.add_argument("--publish-udp", action="store_true")
    parser.add_argument("--publish-udp-mode", default="teacher", choices=["teacher", "geo"])
    parser.add_argument("--udp-out-host", default="127.0.0.1")
    parser.add_argument("--udp-out-port", type=int, default=9999)
    parser.add_argument("--udp-enemy-only", action="store_true")
    parser.add_argument("--udp-in-host", default="0.0.0.0")
    parser.add_argument("--udp-in-port", type=int, default=8020)
    args = parser.parse_args()

    run_export(
        seed=args.seed,
        scene_km=args.scene_km,
        source=args.source,
        intercept_mode=args.intercept_mode,
        demo_case=args.demo_case,
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        redis_db=args.redis_db,
        redis_password=args.redis_password,
        enemy_redis_format=args.enemy_redis_format,
        friendly_return_source=args.friendly_return_source,
        geo_origin_lat=args.geo_origin_lat,
        geo_origin_lon=args.geo_origin_lon,
        publish_interval=args.publish_interval,
        sim_speed=args.sim_speed,
        loop=args.loop,
        max_frames=args.max_frames,
        csv_path=args.csv_path,
        csv_append=args.csv_append,
        friendly_start=args.friendly_start,
        enemy_start=args.enemy_start,
        redis_format=args.redis_format,
        publish_udp=args.publish_udp,
        publish_udp_mode=args.publish_udp_mode,
        udp_out_host=args.udp_out_host,
        udp_out_port=args.udp_out_port,
        udp_enemy_only=args.udp_enemy_only,
        udp_in_host=args.udp_in_host,
        udp_in_port=args.udp_in_port,
    )


if __name__ == "__main__":
    main()
