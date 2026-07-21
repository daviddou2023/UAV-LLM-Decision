"""
数据适配层:
1. 通过本机 redis-cli 读取实时键值
2. 兼容 `enemy:set/uav:set` 与 `1_x/1_y/1_z/...` 两类数据风格
3. 对跳点、丢帧、误识别做平滑与保守降级
"""
import ast
import json
import math
import re
import subprocess
import time
from typing import Dict, List, Optional, Tuple

from marl_common import CFG


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RedisCLIClient:
    def __init__(self, host="127.0.0.1", port=6379, db=0, timeout=1.2):
        self.base_cmd = [
            "redis-cli",
            "-h", str(host),
            "-p", str(port),
            "-n", str(db),
            "--raw",
        ]
        self.timeout = timeout

    def _run(self, *args) -> str:
        res = subprocess.run(
            self.base_cmd + list(args),
            capture_output=True,
            text=True,
            timeout=self.timeout,
            check=False,
        )
        if res.returncode != 0:
            raise RuntimeError((res.stderr or res.stdout or "redis-cli failed").strip())
        return res.stdout

    def keys(self, pattern="*") -> List[str]:
        out = self._run("KEYS", pattern)
        return [line.strip() for line in out.splitlines() if line.strip()]

    def smembers(self, key: str) -> List[str]:
        out = self._run("SMEMBERS", key)
        return [line.strip() for line in out.splitlines() if line.strip()]

    def mget(self, keys: List[str]) -> Dict[str, Optional[str]]:
        if not keys:
            return {}
        out = self._run("MGET", *keys)
        lines = out.split("\n")
        if lines and lines[-1] == "":
            lines = lines[:-1]
        if len(lines) < len(keys):
            lines.extend([""] * (len(keys) - len(lines)))
        return {
            key: (value if value != "" else None)
            for key, value in zip(keys, lines[:len(keys)])
        }

    def get_auto(self, key: str):
        try:
            out = self._run("GET", key)
            if out.endswith("\n"):
                out = out[:-1]
            return out if out != "" else None
        except RuntimeError as exc:
            if "WRONGTYPE" not in str(exc):
                raise
        out = self._run("HGETALL", key)
        lines = [line for line in out.splitlines()]
        data = {}
        for idx in range(0, len(lines), 2):
            k = lines[idx]
            v = lines[idx + 1] if idx + 1 < len(lines) else ""
            data[k] = v
        return data or None


class TeacherDataFeed:
    def __init__(self, host="127.0.0.1", port=6379, db=0, poll_interval=None):
        self.client = RedisCLIClient(host=host, port=port, db=db)
        self.poll_interval = poll_interval or CFG.RADAR_POLL_INTERVAL
        self.last_poll = 0.0
        self.track_cache: Dict[Tuple[str, str], dict] = {}
        self.last_diag = ""
        self.last_error = ""
        self.last_result = None

    def poll(self, sim_time: float):
        now = time.time()
        if self.last_result and (now - self.last_poll) < self.poll_interval:
            return self.last_result

        meta = {
            "connected": False,
            "mode": "redis",
            "diag": "",
            "frame": None,
            "keys": 0,
        }
        events = []
        result = {"enemies": [], "friendlies": [], "meta": meta, "events": events}

        try:
            keys = self.client.keys("*")
            meta["connected"] = True
            meta["keys"] = len(keys)
            if not keys:
                meta["diag"] = "Redis在线，但当前无数据"
                self.last_result = result
                self.last_poll = now
                self.last_error = ""
                return result

            set_keys = [k for k in ("enemy:set", "uav:set") if k in keys]
            string_keys = [k for k in keys if k not in set_keys]
            set_values = {k: self.client.smembers(k) for k in set_keys}

            try:
                raw_values = self.client.mget(string_keys)
            except RuntimeError:
                raw_values = {k: self.client.get_auto(k) for k in string_keys}

            parsed = {k: self._parse_value(v) for k, v in raw_values.items()}
            enemies, friendlies = self._normalize_tracks(parsed, set_values, events)
            meta["frame"] = self._extract_meta_frame(parsed)
            meta["diag"] = self._build_diag(enemies, friendlies, parsed)
            result = {
                "enemies": enemies,
                "friendlies": friendlies,
                "meta": meta,
                "events": events,
            }
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)
            meta["diag"] = f"Redis数据读取失败: {exc}"

        self.last_result = result
        self.last_poll = now
        return result

    def _parse_value(self, value):
        if value is None:
            return None
        if isinstance(value, dict):
            return {str(k): self._parse_scalar(v) for k, v in value.items()}
        return self._parse_scalar(value)

    def _parse_scalar(self, value):
        if value is None:
            return None
        if isinstance(value, (int, float, bool, dict, list)):
            return value
        text = str(value).strip()
        if text == "":
            return None
        if text.lower() in ("true", "false"):
            return text.lower() == "true"
        try:
            if "." in text:
                return float(text)
            return int(text)
        except ValueError:
            pass

        for parser in (json.loads, ast.literal_eval):
            try:
                return parser(text)
            except Exception:
                continue

        if "=" in text and ("," in text or ";" in text):
            parts = re.split(r"[;,]", text)
            data = {}
            for part in parts:
                if "=" not in part:
                    continue
                k, v = part.split("=", 1)
                data[k.strip()] = self._parse_scalar(v)
            if data:
                return data
        return text

    def _normalize_tracks(self, parsed: Dict[str, object], set_values: Dict[str, List[str]], events: List[str]):
        named_groups: Dict[str, dict] = {}
        slot_groups: Dict[str, dict] = {}

        for ext_id in set_values.get("enemy:set", []):
            named_groups.setdefault(ext_id, {"hint": "enemy", "sources": {}})
        for ext_id in set_values.get("uav:set", []):
            named_groups.setdefault(ext_id, {"hint": "uav", "sources": {}})

        for key, value in parsed.items():
            if value is None:
                continue
            m_slot = re.match(r"^(\d+)_(x|y|z|status|timestamp|frame)$", key)
            if m_slot:
                slot_id, field = m_slot.groups()
                slot_groups.setdefault(slot_id, {})[field] = value
                continue

            m_named = re.match(r"^([^:]+):([^:]+):(.+)$", key)
            if not m_named:
                continue
            prefix, field, ext_id = m_named.groups()
            grp = named_groups.setdefault(ext_id, {"hint": prefix, "sources": {}})
            grp["sources"][f"{prefix}:{field}"] = value
            if prefix in ("enemy", "uav"):
                grp["hint"] = prefix

        enemies: List[dict] = []
        friendlies: List[dict] = []

        for ext_id, group in named_groups.items():
            track = self._normalize_named_track(ext_id, group)
            if not track:
                continue
            if track["kind"] == "uav":
                friendlies.append(track)
            else:
                enemies.append(track)

        if not enemies:
            for slot_id, fields in slot_groups.items():
                track = self._normalize_slot_track(slot_id, fields)
                if track:
                    enemies.append(track)

        enemies.sort(key=lambda item: (item["stale"], item["lost"], item["y"]))
        friendlies.sort(key=lambda item: item["external_id"])
        return enemies, friendlies

    def _normalize_named_track(self, ext_id: str, group: dict):
        bag = {}
        status_value = None
        speed_value = None
        for source_key, value in group["sources"].items():
            _, field = source_key.split(":", 1)
            if isinstance(value, dict):
                for k, v in value.items():
                    bag[str(k)] = v
            else:
                if field == "status":
                    status_value = value
                elif field == "speeds":
                    speed_value = value
                bag[field] = value

        if status_value is not None and "status" not in bag:
            bag["status"] = status_value
        if speed_value is not None and "speed" not in bag:
            bag["speed"] = speed_value

        x = self._extract_number(bag, ("x", "pos_x", "coord_x"))
        y = self._extract_number(bag, ("y", "pos_y", "coord_y"))
        z = self._extract_number(bag, ("z", "alt", "altitude", "alt_m", "height"), default=0.0)
        if x is None or y is None:
            return None

        heading = self._extract_number(bag, ("heading", "heading_deg", "yaw", "course"))
        speed = self._extract_number(bag, ("speed", "speed_mps", "velocity", "v"), default=0.0)
        stamp = self._extract_timestamp(bag)
        quality = self._extract_number(bag, ("quality", "confidence", "score"), default=1.0)
        frame = self._extract_number(bag, ("frame", "frame_id", "seq"))
        status = bag.get("status")
        kind = self._infer_kind(ext_id, group.get("hint"), status)
        return self._stabilize_track(
            {
                "external_id": ext_id,
                "kind": kind,
                "x": float(x),
                "y": float(y),
                "z": float(z or 0.0),
                "heading": heading,
                "speed": float(speed or 0.0),
                "stamp": stamp,
                "frame": int(frame) if frame is not None else None,
                "quality": float(quality or 0.0),
                "status": status,
                "raw": bag,
            }
        )

    def _normalize_slot_track(self, slot_id: str, fields: dict):
        x = self._coerce_number(fields.get("x"))
        y = self._coerce_number(fields.get("y"))
        if x is None or y is None:
            return None
        z = self._coerce_number(fields.get("z"), default=0.0)
        status = fields.get("status")
        stamp = self._coerce_number(fields.get("timestamp"))
        frame = self._coerce_number(fields.get("frame"))
        return self._stabilize_track(
            {
                "external_id": f"slot-{slot_id}",
                "kind": self._infer_kind(f"slot-{slot_id}", "enemy", status),
                "x": float(x),
                "y": float(y),
                "z": float(z or 0.0),
                "heading": None,
                "speed": 0.0,
                "stamp": stamp,
                "frame": int(frame) if frame is not None else None,
                "quality": 0.85,
                "status": status,
                "raw": dict(fields),
            }
        )

    def _stabilize_track(self, track: dict):
        now = time.time()
        key = (track["kind"], track["external_id"])
        prev = self.track_cache.get(key)
        stamp = track["stamp"] if track["stamp"] is not None else now
        if stamp > 1e12:
            stamp /= 1000.0
        if stamp < 1e9:
            stamp = now

        if prev:
            dt = max(now - prev["local_time"], 1e-3)
            dx = track["x"] - prev["x"]
            dy = track["y"] - prev["y"]
            dz = track["z"] - prev["z"]
            jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            smoothing = CFG.POSITION_SMOOTHING
            if jump > CFG.MAX_TRACK_JUMP_M and dt < 1.0:
                smoothing = 0.15
            track["x"] = prev["x"] + (track["x"] - prev["x"]) * smoothing
            track["y"] = prev["y"] + (track["y"] - prev["y"]) * smoothing
            track["z"] = prev["z"] + (track["z"] - prev["z"]) * smoothing
            if not track["speed"]:
                track["speed"] = jump / dt
            if track["heading"] is None and abs(dx) + abs(dy) > 1e-6:
                track["heading"] = math.degrees(math.atan2(dy, dx)) % 360
            track["vz"] = dz / dt
        else:
            track["vz"] = 0.0
            if track["heading"] is None:
                track["heading"] = 90.0

        age = max(0.0, now - stamp)
        track["age"] = age
        track["stale"] = age > CFG.RADAR_STALE_SEC
        track["lost"] = age > CFG.RADAR_LOST_SEC
        fresh = 1.0 - _clamp(age / max(CFG.RADAR_LOST_SEC, 0.1), 0.0, 1.0)
        track["track_quality"] = _clamp(track["quality"] * (0.3 + 0.7 * fresh), 0.0, 1.0)
        track["classification_confidence"] = self._classification_confidence(track["external_id"], track["status"])
        track["local_time"] = now
        self.track_cache[key] = dict(track)
        return track

    def _classification_confidence(self, ext_id: str, status) -> float:
        text = f"{ext_id} {status or ''}".lower()
        if any(token in text for token in ("false", "unknown", "suspect", "jam", "uncertain")):
            return 0.35
        if "decoy" in text or "诱饵" in text:
            return 0.5
        return 0.95

    def _extract_number(self, bag: dict, keys, default=None):
        for key in keys:
            if key in bag:
                val = self._coerce_number(bag.get(key))
                if val is not None:
                    return val
        return default

    def _extract_timestamp(self, bag: dict):
        return self._extract_number(bag, ("timestamp", "stamp", "ts", "time"))

    def _coerce_number(self, value, default=None):
        if value is None:
            return default
        if isinstance(value, bool):
            return float(value)
        if isinstance(value, (int, float)):
            return float(value)
        try:
            text = str(value).strip()
            if text == "":
                return default
            return float(text)
        except Exception:
            return default

    def _infer_kind(self, ext_id: str, hint: Optional[str], status) -> str:
        text = f"{ext_id} {hint or ''} {status or ''}".lower()
        if "enemy" in text or "threat" in text or "hostile" in text or "敌" in text:
            return "enemy"
        if "uav" in text or "friendly" in text or "ally" in text or "己" in text:
            return "uav"
        if hint == "uav":
            return "uav"
        return "enemy"

    def _extract_meta_frame(self, parsed: Dict[str, object]):
        value = parsed.get("total_frame")
        if value is None:
            return None
        number = self._coerce_number(value)
        return int(number) if number is not None else None

    def _build_diag(self, enemies: List[dict], friendlies: List[dict], parsed: Dict[str, object]) -> str:
        total = len(enemies) + len(friendlies)
        frame = self._extract_meta_frame(parsed)
        frame_text = f" frame={frame}" if frame is not None else ""
        return f"数据: 目标{len(enemies)} 我方{len(friendlies)} 总键{len(parsed)}{frame_text}" if total else "数据在线，但无有效坐标"
