"""
扫描远端 flat 槽位里哪些敌方目标真的在更新。

用途:
1. 扫描所有 `*_timestamp` 槽位
2. 只关注 `type=enemy` 的槽位
3. 同时判断:
   - 哪些槽位最近 fresh 秒内仍是“新鲜”的
   - 哪些槽位相对上一轮真的发生了变化
"""

import argparse
import re
import time

from perception.radar_feed import RedisCLIClient


WATCH_FIELDS = ("x", "y", "z", "status", "type", "frame", "timestamp")


def _safe_float(value):
    try:
        return float(value)
    except Exception:
        return None


def _safe_int(value):
    try:
        return int(float(value))
    except Exception:
        return None


def _build_keys(slot_id):
    return [f"{slot_id}_{field}" for field in WATCH_FIELDS]


def _discover_slot_ids(client):
    slot_ids = []
    for key in client.keys("*_timestamp"):
        match = re.match(r"^(\d+)_timestamp$", key)
        if match:
            slot_ids.append(match.group(1))
    return sorted(set(slot_ids), key=lambda text: int(text))


def _load_slot(client, slot_id):
    raw = client.mget(_build_keys(slot_id))
    data = {field: raw.get(f"{slot_id}_{field}") for field in WATCH_FIELDS}
    data["slot_id"] = slot_id
    data["frame_int"] = _safe_int(data.get("frame"))
    data["timestamp_float"] = _safe_float(data.get("timestamp"))
    return data


def _is_enemy(slot):
    return str(slot.get("type") or "").strip().lower() == "enemy"


def _slot_age(slot):
    stamp = slot.get("timestamp_float")
    if stamp is None:
        return None
    return max(0.0, time.time() - stamp)


def _slot_changed(prev_slot, curr_slot):
    if prev_slot is None:
        return True
    for field in WATCH_FIELDS:
        if prev_slot.get(field) != curr_slot.get(field):
            return True
    return False


def _format_slot(slot):
    age = _slot_age(slot)
    age_text = "n/a" if age is None else f"{age:.1f}s"
    return (
        f"{slot['slot_id']}: frame={slot.get('frame')} age={age_text} "
        f"status={slot.get('status')} type={slot.get('type')} "
        f"x={slot.get('x')} y={slot.get('y')} z={slot.get('z')}"
    )


def main():
    parser = argparse.ArgumentParser(description="扫描远端 Redis 里哪些敌方 flat 槽位真的在更新")
    parser.add_argument("--host", default="192.166.51.23")
    parser.add_argument("--port", type=int, default=6379)
    parser.add_argument("--db", type=int, default=0)
    parser.add_argument("--password", default="uav123")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--fresh-sec", type=float, default=10.0, help="认为是活数据的时间窗口(秒)")
    parser.add_argument("--top-k", type=int, default=10, help="每轮最多打印多少个活/变更槽位")
    args = parser.parse_args()

    client = RedisCLIClient(
        host=args.host,
        port=args.port,
        db=args.db,
        password=args.password,
    )

    previous = {}
    print(
        "Watching active enemy slots | "
        f"redis={args.host}:{args.port}/{args.db} interval={args.interval:.1f}s fresh_sec={args.fresh_sec:.1f}s"
    )

    try:
        while True:
            slot_ids = _discover_slot_ids(client)
            slots = [_load_slot(client, slot_id) for slot_id in slot_ids]
            enemy_slots = [slot for slot in slots if _is_enemy(slot)]

            fresh_slots = []
            changed_slots = []
            for slot in enemy_slots:
                age = _slot_age(slot)
                if age is not None and age <= args.fresh_sec:
                    fresh_slots.append(slot)
                if _slot_changed(previous.get(slot["slot_id"]), slot):
                    changed_slots.append(slot)

            fresh_slots.sort(key=lambda slot: (slot.get("timestamp_float") or 0.0), reverse=True)
            changed_slots.sort(key=lambda slot: int(slot["slot_id"]))

            now_text = time.strftime("%H:%M:%S", time.localtime())
            print(
                f"\n[{now_text}] enemy_slots={len(enemy_slots)} "
                f"fresh<={args.fresh_sec:.1f}s={len(fresh_slots)} changed={len(changed_slots)}"
            )

            if fresh_slots:
                print("  Fresh slots:")
                for slot in fresh_slots[: max(1, args.top_k)]:
                    print("   ", _format_slot(slot))
            else:
                print("  Fresh slots: none")

            if changed_slots:
                print("  Changed slots:")
                for slot in changed_slots[: max(1, args.top_k)]:
                    print("   ", _format_slot(slot))
            else:
                print("  Changed slots: none")

            previous = {slot["slot_id"]: slot for slot in enemy_slots}
            time.sleep(max(0.5, float(args.interval)))
    except KeyboardInterrupt:
        print("\nWatch stopped.")


if __name__ == "__main__":
    main()
