"""
远端敌方槽位更新监视器

用途:
1. 盯住类似 275_x/275_y/275_z 这类 flat 键
2. 实时显示 frame/timestamp/x/y/z/status/type 是否变化
3. 快速判断“对方真的在刷新”还是“键还在但内容没动”
"""

import argparse
import time

from perception.radar_feed import RedisCLIClient


WATCH_FIELDS = ("x", "y", "z", "status", "type", "frame", "timestamp")


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _build_keys(slot_id):
    return [f"{slot_id}_{field}" for field in WATCH_FIELDS]


def _summarize_changes(prev_data, curr_data):
    changed = []
    for field in WATCH_FIELDS:
        if prev_data.get(field) != curr_data.get(field):
            changed.append(field)
    return changed


def _format_age(stamp):
    if stamp is None:
        return "n/a"
    age = time.time() - stamp
    return f"{age:.1f}s"


def main():
    parser = argparse.ArgumentParser(description="监视远端 flat 敌方槽位是否真的在刷新")
    parser.add_argument("--host", default="192.166.51.23")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--db", type=int, default=0)
    parser.add_argument("--password", default="uav123")
    parser.add_argument("--slot-id", default="275")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    client = RedisCLIClient(
        host=args.host,
        port=args.port,
        db=args.db,
        password=args.password,
    )
    keys = _build_keys(args.slot_id)
    prev = None
    unchanged_count = 0

    print(
        "Watching enemy slot | "
        f"redis={args.host}:{args.port}/{args.db} slot={args.slot_id} interval={args.interval:.1f}s"
    )
    print("Fields:", ", ".join(keys))

    try:
        while True:
            raw = client.mget(keys)
            data = {field: raw.get(f"{args.slot_id}_{field}") for field in WATCH_FIELDS}
            stamp = _safe_float(data.get("timestamp"))
            frame = data.get("frame")
            changed = _summarize_changes(prev or {}, data) if prev is not None else list(WATCH_FIELDS)

            if prev is None:
                status = "INIT"
            elif changed:
                status = "UPDATED"
                unchanged_count = 0
            else:
                status = "UNCHANGED"
                unchanged_count += 1

            now_text = time.strftime("%H:%M:%S", time.localtime())
            changed_text = ",".join(changed) if changed else "-"
            print(
                f"[{now_text}] {status:<9} "
                f"frame={frame or 'None':>6} "
                f"age={_format_age(stamp):>8} "
                f"same_for={unchanged_count:>3} "
                f"changed={changed_text}"
            )
            print(
                f"           x={data.get('x')} y={data.get('y')} z={data.get('z')} "
                f"status={data.get('status')} type={data.get('type')} ts={data.get('timestamp')}"
            )
            prev = data
            time.sleep(max(0.2, float(args.interval)))
    except KeyboardInterrupt:
        print("\nWatch stopped.")


if __name__ == "__main__":
    main()
