"""
    负责根据启动脚本中的配置，去连接真实的外部数据源（如 Redis、UDP），抓取敌方无人机的原始轨迹数据，并进行清洗、转换、平滑，最终变成系统内部能理解的统一格式

数据适配层:
1. 通过本机 redis-cli 读取实时键值
2. 兼容 `enemy:set/uav:set` 与 `1_x/1_y/1_z/...` 两类数据风格
3. 对跳点、丢帧、误识别做平滑与保守降级

    在 simulation/main.py 的 InterceptionEnvironment 初始化时，会根据 SOURCE 参数（如 "fusion", "redis", "udp"）来实例化本文件中的 TeacherDataFeed 或 FusionTrackFeed
    本文件的_normalize_tracks 函数的末尾，调用了perception/enemy_association.py中的self.enemy_associator.associate(enemies)。过滤掉跳闪的伪重号目标，生成稳定 ID 队列

"""
import ast
import json
import math
import os
import re
import subprocess
import time
from typing import Dict, List, Optional, Tuple

from core.common import CFG, angle_diff
from perception.enemy_association import LocalEnemyAssociator
from core.geo import GeoReference, DEFAULT_GEO_ORIGIN_LAT, DEFAULT_GEO_ORIGIN_LON


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RedisCLIClient:
    """直接通过在操作系统中执行 redis-cli 命令来获取数据"""
    _HGETALL_MANY_LUA = """
local out = {}
for i, key in ipairs(KEYS) do
    local vals = redis.call('HGETALL', key)
    table.insert(out, key)
    table.insert(out, tostring(#vals))
    for _, v in ipairs(vals) do
        table.insert(out, v)
    end
end
return out
""".strip()

    def __init__(self, host="127.0.0.1", port=6379, db=0, timeout=1.2, password=None):
        self.base_cmd = [
            "redis-cli",
            "-h", str(host),
            "-p", str(port),
            "-n", str(db),
            "--raw",
        ]
        self.timeout = timeout
        self.env = None
        if password not in (None, ""):
            self.env = dict(os.environ)
            self.env["REDISCLI_AUTH"] = str(password)

    # 执行bash命令
    def _run(self, *args) -> str:
        res = subprocess.run(
            self.base_cmd + list(args),
            capture_output=True,
            text=True,
            timeout=self.timeout,
            check=False,
            env=self.env,
        )
        if res.returncode != 0:
            raise RuntimeError((res.stderr or res.stdout or "redis-cli failed").strip())
        return res.stdout

    def keys(self, pattern="*") -> List[str]:
        out = self._run("KEYS", pattern)
        return [line.strip() for line in out.splitlines() if line.strip()]

    def get(self, key: str):
        out = self._run("GET", key)
        if out.endswith("\n"):
            out = out[:-1]
        if out.startswith("WRONGTYPE"):
            raise RuntimeError(out)
        return out if out != "" else None

    def hgetall(self, key: str) -> Dict[str, str]:
        out = self._run("HGETALL", key)
        lines = [line for line in out.splitlines()]
        data = {}
        for idx in range(0, len(lines), 2):
            k = lines[idx]
            v = lines[idx + 1] if idx + 1 < len(lines) else ""
            data[k] = v
        return data

    # 使用 Lua 脚本一次性并发拉取多个键的哈希值，解决数据量大时的网络延迟瓶颈
    def hgetall_many(self, keys: List[str]) -> Dict[str, Dict[str, str]]:
        if not keys:
            return {}
        out = self._run("EVAL", self._HGETALL_MANY_LUA, str(len(keys)), *keys)
        lines = [line for line in out.splitlines()]
        parsed: Dict[str, Dict[str, str]] = {}
        idx = 0
        while idx < len(lines):
            key = lines[idx]
            idx += 1
            if idx >= len(lines):
                break
            field_count = int(lines[idx] or "0")
            idx += 1
            data = {}
            for _ in range(field_count // 2):
                if idx >= len(lines):
                    break
                field = lines[idx]
                value = lines[idx + 1] if idx + 1 < len(lines) else ""
                data[field] = value
                idx += 2
            parsed[key] = data
        return parsed

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
            return self.get(key)
        except RuntimeError as exc:
            if "WRONGTYPE" not in str(exc):
                raise
        data = self.hgetall(key)
        return data or None


class TeacherDataFeed:
    """数据解析与清洗层"""
    def __init__(
        self,
        host="127.0.0.1",
        port=6379,
        db=0,
        poll_interval=None,
        password=None,
        redis_format="auto",
        side_filter=None,
        geo_origin_lat=None,
        geo_origin_lon=None,
        geo_reference=None,
        enemy_assoc="on",
        enemy_assoc_max_distance=450.0,
        enemy_assoc_max_altitude=140.0,
        enemy_assoc_keep_sec=18.0,
        enemy_hash_remap_mode="direct",
        enemy_hash_center_x_ratio=0.5,
        enemy_hash_lateral_scale=1.0,
        enemy_hash_range_scale=5.0,
        enemy_hash_start_range_m=0.0,
        enemy_hash_y_offset_m=0.0,
        enemy_hash_hide_outbound="off",
        enemy_flat_remap_mode="legacy",
        enemy_flat_rotate_deg=135.0,
        enemy_flat_flip_x="off",
        enemy_flat_flip_y="off",
        enemy_flat_scale=1.0,
        enemy_flat_center_x_ratio=0.5,
        enemy_flat_center_y_ratio=0.2,
    ):
        self.client = RedisCLIClient(host=host, port=port, db=db, password=password)
        self.poll_interval = poll_interval or CFG.RADAR_POLL_INTERVAL
        self.redis_format = str(redis_format or "auto").strip().lower()
        self.side_filter = str(side_filter or "").strip().lower() or None
        if geo_reference is None:
            geo_reference = GeoReference(
                origin_lat=34.2663 if geo_origin_lat is None else geo_origin_lat,
                origin_lon=108.9549 if geo_origin_lon is None else geo_origin_lon,
            )
        self.geo_reference = geo_reference
        # 兼容老师当前 flat 数据:
        # 以旧参考原点投影后的平面坐标写在 x/z，高度写在 y。
        self.flat_slot_reference = GeoReference(
            origin_lat=DEFAULT_GEO_ORIGIN_LAT,
            origin_lon=DEFAULT_GEO_ORIGIN_LON,
        )
        self.enemy_associator = LocalEnemyAssociator(
            enabled=str(enemy_assoc or "on").strip().lower() != "off",
            max_distance_m=enemy_assoc_max_distance,
            max_altitude_diff_m=enemy_assoc_max_altitude,
            keep_sec=enemy_assoc_keep_sec,
        )
        self.enemy_hash_remap_mode = str(enemy_hash_remap_mode or "direct").strip().lower()
        self.enemy_hash_center_x_ratio = float(enemy_hash_center_x_ratio)
        self.enemy_hash_lateral_scale = max(0.01, float(enemy_hash_lateral_scale or 1.0))
        self.enemy_hash_range_scale = max(0.1, float(enemy_hash_range_scale or 5.0))
        self.enemy_hash_start_range_m = max(0.0, float(enemy_hash_start_range_m or 0.0))
        self.enemy_hash_y_offset_m = float(enemy_hash_y_offset_m or 0.0)
        self.enemy_hash_hide_outbound = str(enemy_hash_hide_outbound or "off").strip().lower() in ("1", "true", "on", "yes")
        self.enemy_hash_projection_cache: Dict[str, dict] = {}
        self.enemy_flat_remap_mode = str(enemy_flat_remap_mode or "legacy").strip().lower()
        self.enemy_flat_rotate_deg = float(enemy_flat_rotate_deg or 0.0)
        self.enemy_flat_flip_x = str(enemy_flat_flip_x or "off").strip().lower() in ("1", "true", "on", "yes")
        self.enemy_flat_flip_y = str(enemy_flat_flip_y or "off").strip().lower() in ("1", "true", "on", "yes")
        self.enemy_flat_scale = max(0.05, float(enemy_flat_scale or 1.0))
        self.enemy_flat_center_x_ratio = float(enemy_flat_center_x_ratio)
        self.enemy_flat_center_y_ratio = float(enemy_flat_center_y_ratio)
        self.last_poll = 0.0
        self.track_cache: Dict[Tuple[str, str], dict] = {}
        self.last_diag = ""
        self.last_error = ""
        self.last_result = None

    # (核心入口) 每帧被主循环调用，执行数据拉取和处理的完整管线
    def poll(self, sim_time: float):
        """设备1 Redis 航迹轮询入口。

        调用回路：simulation/main.py::_sync_teacher_data() -> feed.poll()。
        本函数完成一次输入帧处理：限流复用、读取 Redis、解析原始值、
        归一化敌我航迹，并返回 {enemies, friendlies, meta, events}。
        """
        now = time.time()
        # 限制轮询频率，避免把Redis压垮
        if self.last_result and (now - self.last_poll) < self.poll_interval:
            return self.last_result

        # 初始化元数据
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
            # 从 Redis 原始拉取数据 (收集键值对)
            set_values, string_keys, raw_values, key_count = self._collect_live_sources()
            meta["connected"] = True
            meta["keys"] = key_count
            if key_count <= 0:
                meta["diag"] = "Redis在线，但当前无数据"
                self.last_result = result
                self.last_poll = now
                self.last_error = ""
                return result

            # 将字符串数字尝试解析为浮点/整型等 Python 对象
            parsed = {k: self._parse_value(v) for k, v in raw_values.items()}
            # 核心清洗步骤：归类、坐标投影、关联、平滑
            enemies, friendlies = self._normalize_tracks(parsed, set_values, events)
            # 记录最新帧号和诊断信息
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
            # 如果断网了，记录错误并返回空数据，防止程序崩溃
            self.last_error = str(exc)
            meta["diag"] = f"Redis数据读取失败: {exc}"

        self.last_result = result
        self.last_poll = now
        return result

    # 从 Redis 提取原始字符串/哈希数据；只负责取数，不判断敌我业务含义。
    def _collect_live_sources(self):
        """读取 Redis 原始输入，统一返回 set、key、value 和 key 数量。

        hash 模式下优先按 enemy:set/uav:set 找状态 hash；普通模式下扫描
        key/value。后续 _normalize_tracks() 再按前缀和字段做业务归类。
        """
        if self.redis_format == "hash":
            return self._collect_hash_sources()

        keys = self.client.keys("*")
        if not keys:
            return {}, [], {}, 0
        set_keys = [k for k in ("enemy:set", "uav:set") if k in keys]
        string_keys = [k for k in keys if k not in set_keys]
        set_values = {k: self.client.smembers(k) for k in set_keys}
        raw_values = self._fetch_key_values(string_keys)
        return set_values, string_keys, raw_values, len(keys)

    def _collect_hash_sources(self):
        set_values: Dict[str, List[str]] = {}
        string_keys: List[str] = []
        raw_values: Dict[str, object] = {}

        set_names = []
        if self.side_filter in (None, "enemy"):
            set_names.append("enemy:set")
        if self.side_filter in (None, "uav", "friendly", "ally"):
            set_names.append("uav:set")

        for set_name in set_names:
            members = self.client.smembers(set_name)
            if members:
                set_values[set_name] = members

        for ext_id in set_values.get("enemy:set", []):
            string_keys.extend(self._candidate_named_hash_keys("enemy", ext_id))
        for ext_id in set_values.get("uav:set", []):
            string_keys.extend(self._candidate_named_hash_keys("uav", ext_id))

        # 兼容上游未维护 set 时的兜底，但只在确实没有 set 的情况下触发一次全局扫描。
        if not string_keys:
            keys = self.client.keys("enemy:status:*")
            if self.side_filter not in ("enemy",):
                keys += self.client.keys("uav:status:*")
            string_keys = sorted(set(keys))

        string_keys = sorted(set(string_keys))
        for key, data in self.client.hgetall_many(string_keys).items():
            if data:
                raw_values[key] = data

        try:
            total_frame = self.client.get("total_frame")
        except RuntimeError:
            total_frame = None
        if total_frame not in (None, ""):
            raw_values["total_frame"] = total_frame

        key_count = len(string_keys) + sum(len(v) for v in set_values.values()) + (1 if total_frame not in (None, "") else 0)
        return set_values, string_keys, raw_values, key_count

    def _candidate_named_hash_keys(self, prefix: str, ext_id: str) -> List[str]:
        ext_text = str(ext_id).strip()
        if not ext_text:
            return []
        keys = [f"{prefix}:status:{ext_text}"]
        if not ext_text.startswith(prefix):
            keys.append(f"{prefix}:status:{prefix}{ext_text}")
        return keys

    # 自动推断并转换原始数据类型
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

    # (分类枢纽) 将一大堆杂乱的 Redis 键值归类，判断哪些是敌人、哪些是自己，并格式化成标准的字典对象
    def _normalize_tracks(self, parsed: Dict[str, object], set_values: Dict[str, List[str]], events: List[str]):
        """把 Redis 原始键值转换为设备2内部统一敌我航迹。

        处理顺序：按 named/slot 分组 -> 提取坐标/速度/姿态 -> 投影到本地
        meters_xy_alt -> 敌方目标稳定 ID 关联 -> 输出 enemies/friendlies。
        下游 _sync_teacher_data() 只消费这个统一结果，不再关心 Redis 字段形态。
        """
        # 从分好类的字典中，提取经纬度或坐标，并用对应的 remap 函数转换到本地坐标系
        named_groups: Dict[str, dict] = {}
        slot_groups: Dict[str, dict] = {}

        for ext_id in set_values.get("enemy:set", []):
            named_groups.setdefault(ext_id, {"hint": "enemy", "sources": {}})
        for ext_id in set_values.get("uav:set", []):
            named_groups.setdefault(ext_id, {"hint": "uav", "sources": {}})

        for key, value in parsed.items():
            if value is None:
                continue
            m_slot = re.match(r"^(\d+)_(x|y|z|status|type|timestamp|frame|battery)$", key)
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

        named_enemies: List[dict] = []
        named_friendlies: List[dict] = []

        for ext_id, group in named_groups.items():
            track = self._normalize_named_track(ext_id, group)
            if not track:
                continue
            if track["kind"] == "uav":
                named_friendlies.append(track)
            else:
                named_enemies.append(track)

        slot_enemies: List[dict] = []
        slot_friendlies: List[dict] = []
        for slot_id, fields in slot_groups.items():
            track = self._normalize_slot_track(slot_id, fields)
            if not track:
                continue
            if track["kind"] == "uav":
                slot_friendlies.append(track)
            else:
                slot_enemies.append(track)

        # 确定最终使用的是哪种格式的数据
        if self.redis_format == "hash":
            enemies = named_enemies
            friendlies = named_friendlies
        elif self.redis_format == "flat":
            enemies = slot_enemies
            friendlies = slot_friendlies
        else:
            if named_enemies or named_friendlies:
                enemies = named_enemies
                friendlies = named_friendlies
            else:
                enemies = slot_enemies
                friendlies = slot_friendlies

        if self.side_filter == "enemy":
            friendlies = []
        elif self.side_filter in ("uav", "friendly", "ally"):
            enemies = []

        # 将敌方目标交给关联器，防止雷达 ID 闪烁
        enemies = self.enemy_associator.associate(enemies)
        # 对所有的目标轨迹调用平滑函数进行“防抖”处理
        enemies = [self._stabilize_track(track) for track in enemies]
        friendlies = [self._stabilize_track(track) for track in friendlies]
        # 剔除彻底丢失的死目标，并按照位置危险程度(Y坐标)排序
        if self.redis_format == "hash":
            enemies = [track for track in enemies if not track["lost"]]
            friendlies = [track for track in friendlies if not track["lost"]]
        enemies.sort(key=lambda item: (item["stale"], item["lost"], item["y"]))
        friendlies.sort(key=lambda item: item["external_id"])
        return enemies, friendlies

    # 将不同格式（如带有 101_x 或 enemy:status:xxx 标签）的坐标解析出来
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
        if x is None or y is None:
            lon = self._extract_number(bag, ("lon", "longitude"))
            lat = self._extract_number(bag, ("lat", "latitude"))
            if lon is not None and lat is not None:
                x, y = self.geo_reference.lonlat_to_xy(lat, lon)
        z = self._extract_number(bag, ("z", "alt", "altitude", "alt_m", "height"), default=0.0)
        if x is None or y is None:
            return None

        heading = self._extract_number(bag, ("heading", "heading_deg", "yaw", "course"))
        roll = self._extract_number(bag, ("roll", "roll_deg", "bank", "bank_angle"))
        pitch = self._extract_number(bag, ("pitch", "pitch_deg"))
        yaw = self._extract_number(bag, ("yaw", "yaw_deg"), default=heading)
        speed = self._extract_number(bag, ("speed", "speed_mps", "velocity", "v"), default=0.0)
        battery = self._extract_number(bag, ("battery", "fuel", "power"))
        stamp = self._extract_timestamp(bag)
        quality = self._extract_number(bag, ("quality", "confidence", "score"), default=1.0)
        frame = self._extract_number(bag, ("frame", "frame_id", "seq"))
        status = bag.get("status")
        hint = group.get("hint")
        if hint == "enemy":
            kind = "enemy"
        elif hint == "uav":
            kind = "uav"
        else:
            kind = self._infer_kind(ext_id, hint, status)
        if kind == "enemy":
            remapped = self._remap_hash_enemy_xy(ext_id, x, y)
            if remapped is None:
                return None
            x, y = remapped
        return {
            "external_id": ext_id,
            "source_external_id": ext_id,
            "kind": kind,
            "x": float(x),
            "y": float(y),
            "z": float(z or 0.0),
            "heading": heading,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "speed": float(speed or 0.0),
            "stamp": stamp,
            "frame": int(frame) if frame is not None else None,
            "quality": float(quality or 0.0),
            "battery": float(battery) if battery is not None else None,
            "status": status,
            "raw": bag,
        }

    def _normalize_slot_track(self, slot_id: str, fields: dict):
        x = self._coerce_number(fields.get("x"))
        y = self._coerce_number(fields.get("y"))
        if x is None or y is None:
            return None
        z = self._coerce_number(fields.get("z"), default=0.0)
        status = fields.get("status")
        battery = self._coerce_number(fields.get("battery"))
        stamp = self._coerce_number(fields.get("timestamp"))
        frame = self._coerce_number(fields.get("frame"))
        heading = self._coerce_number(fields.get("heading"), default=self._coerce_number(fields.get("yaw")))
        roll = self._coerce_number(fields.get("roll"))
        pitch = self._coerce_number(fields.get("pitch"))
        yaw = self._coerce_number(fields.get("yaw"), default=heading)
        source_id = f"slot-{slot_id}"
        kind = self._infer_kind(source_id, fields.get("type"), status)
        if kind == "enemy":
            x, y, z = self._remap_flat_enemy_xyz(x, y, z)
        return {
            "external_id": source_id,
            "source_external_id": source_id,
            "kind": kind,
            "x": float(x),
            "y": float(y),
            "z": float(z or 0.0),
            "heading": heading,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "speed": 0.0,
            "stamp": stamp,
            "frame": int(frame) if frame is not None else None,
            "quality": 0.85,
            "battery": float(battery) if battery is not None else None,
            "status": status,
            "raw": dict(fields),
        }

    # (核心投影) 处理历史遗留的 flat 格式数据，将其从经纬度投影到你当前运行界面的局部坐标系中
    def _apply_flat_enemy_transform(self, x, y):
        mapped_x = float(x)
        mapped_y = float(y)
        angle_deg = float(self.enemy_flat_rotate_deg or 0.0)
        if abs(angle_deg) > 1e-6:
            ar = math.radians(angle_deg)
            cos_a = math.cos(ar)
            sin_a = math.sin(ar)
            rotated_x = mapped_x * cos_a - mapped_y * sin_a
            rotated_y = mapped_x * sin_a + mapped_y * cos_a
            mapped_x, mapped_y = rotated_x, rotated_y
        if self.enemy_flat_flip_x:
            mapped_x = -mapped_x
        if self.enemy_flat_flip_y:
            mapped_y = -mapped_y
        if abs(self.enemy_flat_scale - 1.0) > 1e-6:
            mapped_x *= self.enemy_flat_scale
            mapped_y *= self.enemy_flat_scale
        return mapped_x, mapped_y

    # 将新的 hash 格式数据投影到屏幕，保证目标能正常展示在你面前
    def _remap_flat_enemy_xyz(self, x, y, z):
        raw_x = float(x)
        raw_y = float(y)
        raw_z = float(z or 0.0)

        if self.enemy_flat_remap_mode == "direct":
            # 关闭上层 remap 后，仅保留基础轴对应:
            #   flat_x -> 内部 x
            #   flat_z 取反 -> 内部 y
            #   flat_y(高度) -> 内部 z
            return raw_x, -raw_z, max(0.0, raw_y)

        # 老师当前 flat 数据样式（以 275 为代表）：
        #   x = 旧参考原点下的东向偏移
        #   y = 高度
        #   z = 旧参考原点下的北向偏移
        # 例如:
        #   275_x = -18020...
        #   275_y = 451
        #   275_z = -5815...
        # 这并不是系统内部期望的 x/y/z，需要先转回经纬度，再投影到当前场景原点。
        looks_like_teacher_radar_flat = (
            abs(raw_x) > CFG.AREA_WIDTH
            or abs(raw_z) > CFG.OUR_BASE_LINE
            or abs(raw_y) > max(CFG.ENEMY_MAX_ALT, CFG.INTERCEPTOR_MAX_ALT) * 4.0
        )
        if not looks_like_teacher_radar_flat:
            return raw_x, raw_y, raw_z

        lat, lon = self.flat_slot_reference.xy_to_lonlat(raw_x, raw_z)
        mapped_x, mapped_y = self.geo_reference.lonlat_to_xy(lat, lon)
        mapped_x, mapped_y = self._apply_flat_enemy_transform(mapped_x, mapped_y)

        # 内部世界坐标要求 x 在 [0, AREA_WIDTH]。
        # 对老师当前 flat 数据，先做旋转/翻转，把来袭方向拉正到“朝我方飞”；
        # 再做平移，把目标抬进当前战术窗里，避免整体贴在边界。
        mapped_x += CFG.AREA_WIDTH * self.enemy_flat_center_x_ratio
        mapped_y += CFG.OUR_BASE_LINE * self.enemy_flat_center_y_ratio
        mapped_y = max(0.0, mapped_y)
        mapped_z = max(0.0, raw_y)
        return mapped_x, mapped_y, mapped_z

    def _remap_hash_enemy_xy(self, ext_id: str, x, y):
        raw_x = float(x)
        raw_y = float(y)
        if self.enemy_hash_remap_mode != "inbound":
            return raw_x, raw_y

        now = time.time()
        state = self.enemy_hash_projection_cache.setdefault(
            ext_id,
            {
                "axis": None,
                "sign": 1.0,
                "last_seen": now,
            },
        )
        state["last_seen"] = now

        if state["axis"] is None:
            if abs(raw_y) >= abs(raw_x):
                state["axis"] = "y"
                state["sign"] = 1.0 if raw_y >= 0.0 else -1.0
            else:
                state["axis"] = "x"
                state["sign"] = 1.0 if raw_x >= 0.0 else -1.0

        if state["axis"] == "y":
            longitudinal = state["sign"] * raw_y
            lateral = raw_x
        else:
            longitudinal = state["sign"] * raw_x
            lateral = raw_y

        if self.enemy_hash_hide_outbound and longitudinal <= 0.0:
            return None

        mapped_x = CFG.AREA_WIDTH * self.enemy_hash_center_x_ratio + lateral * self.enemy_hash_lateral_scale
        if self.enemy_hash_start_range_m > 0.0:
            if longitudinal > self.enemy_hash_start_range_m:
                return None
            progress = self.enemy_hash_start_range_m - max(0.0, longitudinal)
            mapped_y = CFG.INTERCEPT_FAIL_LINE * (progress / self.enemy_hash_start_range_m)
        else:
            mapped_y = CFG.INTERCEPT_FAIL_LINE - max(0.0, longitudinal) * self.enemy_hash_range_scale
        mapped_y += self.enemy_hash_y_offset_m
        mapped_y = max(0.0, min(CFG.INTERCEPT_FAIL_LINE, mapped_y))
        return mapped_x, mapped_y

    # (核心平滑) 数据防抖。如果雷达给的位置瞬间跳变了几百米，这个函数会利用卡尔曼滤波的思想，把这个跳变“抹平”，并计算出平滑的速度和航向角
    def _stabilize_track(self, track: dict):
        """
                防抖与状态推演（卡尔曼滤波的简易版）。
                作用：
                1. 抹平雷达数据跳变的“毛刺”。
                2. 如果外部数据只给了坐标没给速度和航向，该函数会通过前后两帧坐标反推出速度和航向。
                """
        now = time.time()
        key = (track["kind"], track["external_id"])
        prev = self.track_cache.get(key)
        stamp = self._normalize_timestamp_seconds(track["stamp"], now)
        if self.redis_format == "hash":
            stamp = self._resolve_hash_effective_stamp(track, prev, now, stamp)

        if prev:
            # 计算时间间隔和坐标位移
            dt = max(now - prev["local_time"], 1e-3)
            dx = track["x"] - prev["x"]
            dy = track["y"] - prev["y"]
            dz = track["z"] - prev["z"]
            jump = math.sqrt(dx * dx + dy * dy + dz * dz)
            # 判定平滑权重 (如果发生超过门限的大跳变，且时间间隔极短，极有可能是雷达杂波，调小权重，保守移动)
            smoothing = CFG.POSITION_SMOOTHING
            if jump > CFG.MAX_TRACK_JUMP_M and dt < 1.0:
                # 强烈抑制跳点
                smoothing = 0.15
            # 对坐标应用平滑滤波
            track["x"] = prev["x"] + (track["x"] - prev["x"]) * smoothing
            track["y"] = prev["y"] + (track["y"] - prev["y"]) * smoothing
            track["z"] = prev["z"] + (track["z"] - prev["z"]) * smoothing
            # 反向推算物理参数(如果雷达没给)
            if not track["speed"]:
                # # 根据位移倒推速度
                track["speed"] = jump / dt
            if track["heading"] is None and abs(dx) + abs(dy) > 1e-6:
                track["heading"] = math.degrees(math.atan2(dy, dx)) % 360
            if track["heading"] is None:
                prev_heading = prev.get("heading")
                track["heading"] = float(prev_heading if prev_heading is not None else 90.0)
            track["vz"] = dz / dt
            if track.get("pitch") is None:
                track["pitch"] = math.degrees(math.atan2(track["vz"], max(float(track.get("speed") or 0.0), 0.05)))
            if track.get("roll") is None:
                prev_heading = prev.get("heading", track["heading"])
                yaw_rate = math.radians(angle_diff(track["heading"], prev_heading)) / dt
                track["roll"] = _clamp(
                    math.degrees(math.atan((float(track.get("speed") or 0.0) * yaw_rate) / 9.80665)),
                    -70.0,
                    70.0,
                )
        else:
            # 如果是第一次出现的新目标，进行默认初始化
            track["vz"] = 0.0
            if track["heading"] is None:
                track["heading"] = 90.0
            if track.get("pitch") is None:
                track["pitch"] = 0.0
            if track.get("roll") is None:
                track["roll"] = 0.0
        if track.get("yaw") is None:
            track["yaw"] = track["heading"]
        track["roll"] = _clamp(float(track.get("roll") or 0.0), -85.0, 85.0)
        track["pitch"] = _clamp(float(track.get("pitch") or 0.0), -85.0, 85.0)
        track["yaw"] = float(track.get("yaw") or track["heading"]) % 360.0

        # 更新目标的老化状态 (stale/lost) 和置信度
        age = max(0.0, now - stamp)
        track["age"] = age
        track["stale"] = age > CFG.RADAR_STALE_SEC
        track["lost"] = age > CFG.RADAR_LOST_SEC
        fresh = 1.0 - _clamp(age / max(CFG.RADAR_LOST_SEC, 0.1), 0.0, 1.0)
        track["track_quality"] = _clamp(track["quality"] * (0.3 + 0.7 * fresh), 0.0, 1.0)
        track["classification_confidence"] = self._classification_confidence(
            track.get("source_external_id", track["external_id"]),
            track["status"],
        )
        track["_effective_stamp"] = stamp
        track["_source_stamp"] = track.get("stamp")
        # 缓存当前数据，供下一帧平滑使用
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

    def _normalize_timestamp_seconds(self, stamp, now: float) -> float:
        if stamp is None:
            return now
        value = self._coerce_number(stamp)
        if value is None:
            return now

        candidates = [value]
        for divisor in (1e3, 1e6, 1e9):
            candidates.append(value / divisor)

        plausible = [c for c in candidates if 1e9 <= c <= (now + 86400.0 * 30.0)]
        if plausible:
            return min(plausible, key=lambda c: abs(now - c))
        if value < 1e9:
            return now
        return value

    # 时间戳对齐与纠错，处理不同雷达（毫秒/秒级）时钟不同步的问题
    def _resolve_hash_effective_stamp(self, track: dict, prev: Optional[dict], now: float, normalized_stamp: float) -> float:
        frame = track.get("frame")
        if frame is None:
            return normalized_stamp

        if prev is None:
            return normalized_stamp

        prev_frame = prev.get("frame")
        prev_source_stamp = prev.get("_source_stamp")
        prev_effective_stamp = prev.get("_effective_stamp", prev.get("local_time", now))
        source_stamp = track.get("stamp")

        if prev_frame != frame or prev_source_stamp != source_stamp:
            return now
        return prev_effective_stamp

    def _fetch_key_values(self, string_keys: List[str]) -> Dict[str, object]:
        if not string_keys:
            return {}

        if self.redis_format == "hash":
            return {k: self.client.get_auto(k) for k in string_keys}

        if self.redis_format == "flat":
            return self.client.mget(string_keys)

        named_keys = [k for k in string_keys if re.match(r"^[^:]+:[^:]+:.+$", k)]
        flat_keys = [k for k in string_keys if k not in named_keys]

        raw_values = {}
        if flat_keys:
            try:
                raw_values.update(self.client.mget(flat_keys))
            except RuntimeError:
                raw_values.update({k: self.client.get_auto(k) for k in flat_keys})
        if named_keys:
            raw_values.update({k: self.client.get_auto(k) for k in named_keys})
        return raw_values

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
        assoc_text = " 敌方本地关联ON" if self.enemy_associator.enabled else ""
        return f"数据: 目标{len(enemies)} 我方{len(friendlies)} 总键{len(parsed)}{frame_text}{assoc_text}" if total else "数据在线，但无有效坐标"


class FusionTrackFeed:
    """负责在中间把两个流的数据合并成一个包裹，统一交给主程序,
        比如敌方数据从 Redis 读（雷达给的），己方数据从 UDP 读（真实飞控传回的）。
    """
    def __init__(self, enemy_feed=None, friendly_feed=None):
        self.enemy_feed = enemy_feed
        self.friendly_feed = friendly_feed
        self.last_result = None

    def poll(self, sim_time: float):
        """融合敌方输入源和己方输入源，输出同一个 RadarTrackFrame 形态。

        常见场景是敌方走 Redis、己方走 UDP。这里只拼接 enemies/friendlies
        和 meta/events，不做二次任务判断，保证主循环仍只面对一个 feed.poll()。
        """
        enemy_snapshot = self.enemy_feed.poll(sim_time) if self.enemy_feed else {"enemies": [], "friendlies": [], "meta": {}, "events": []}
        friendly_snapshot = self.friendly_feed.poll(sim_time) if self.friendly_feed else {"enemies": [], "friendlies": [], "meta": {}, "events": []}

        enemies = list(enemy_snapshot.get("enemies", []))
        friendlies = list(friendly_snapshot.get("friendlies", []))
        enemy_meta = enemy_snapshot.get("meta", {}) or {}
        friendly_meta = friendly_snapshot.get("meta", {}) or {}
        frame_candidates = [
            value for value in (
                enemy_meta.get("frame"),
                enemy_meta.get("seq"),
                friendly_meta.get("frame"),
                friendly_meta.get("seq"),
            )
            if value is not None
        ]
        meta = {
            "connected": bool(enemy_meta.get("connected")) or bool(friendly_meta.get("connected")),
            "mode": "fusion",
            "diag": (
                f"融合数据: 敌方{len(enemies)}(redis) 己方{len(friendlies)}"
                f"({friendly_meta.get('mode') or 'udp'})"
            ),
            "frame": int(max(frame_candidates)) if frame_candidates else None,
            "keys": len(enemies) + len(friendlies),
            "device_id": friendly_meta.get("device_id") or enemy_meta.get("device_id"),
            "seq": friendly_meta.get("seq") or enemy_meta.get("seq"),
            "timestamp": friendly_meta.get("timestamp") or enemy_meta.get("timestamp"),
            "enemy_meta": enemy_meta,
            "friendly_meta": friendly_meta,
        }
        result = {
            "enemies": enemies,
            "friendlies": friendlies,
            "meta": meta,
            "events": list(enemy_snapshot.get("events", [])) + list(friendly_snapshot.get("events", [])),
        }
        self.last_result = result
        return result

    def close(self):
        for feed in (self.enemy_feed, self.friendly_feed):
            if getattr(feed, "close", None):
                try:
                    feed.close()
                except Exception:
                    pass
