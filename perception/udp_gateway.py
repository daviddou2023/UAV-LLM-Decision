"""
负责将仿真器内部的作战态势打包通过UDP网络广播给指挥系统，同时也负责将外部回传的无人机真实状态接入仿真系统

    simulation/main.py：在 simulation/main.py 初始化时，如果命令行参数配置了 --publish-udp，主程序会实例化 UDPFramePublisher，并在主循环里不断调用它的 maybe_publish 向外部地面站广播态势。
                如果在启动脚本配置了输入源 SOURCE="udp" 或 FRIENDLY_RETURN_SOURCE="udp"，主程序会实例化 TeacherUDPFeed，并在 step() 函数里通过 env._pump_live_data() 拉取真实飞机的数据来驱动沙盘

    perception/radar_feed.py：TeacherUDPFeed 继承了 TeacherDataFeed：复用了perception/radar_feed.py 里的航迹卡尔曼平滑 (_stabilize_track) 和雷达丢失/延迟判定逻辑
    core/geo.py和core/common.py
    integrations/redis_export.py：翻译 status（如把内部的“拦截中”转为外部识别的文本）时，直接复用了 Redis 导出模块写好的状态映射函数
"""


import json
import math
import socket
import time

from core.common import CFG, enemy_export_view, friendly_export_view
from perception.radar_feed import TeacherDataFeed
from core.geo import GeoReference
from integrations.redis_export import enemy_status, friendly_status


def _coerce_float(value, default=None):
    """
    将传入的数值或者字符串强制转换为浮点数
    :param value:
    :param default:
    :return:
    """
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _raw_value(entity, *names, default=None):
    """
    在原始字典 raw_track 中依次查找传入的键名，找到的一个有效值即返回
    :param entity:
    :param names:
    :param default:
    :return:
    """
    raw = entity.get("raw_track") or {}
    for name in names:
        value = raw.get(name)
        if value not in (None, ""):
            return value
    return default


def _entity_view(entity, side):
    """
    根据己方或者敌方阵营，调用 core/common.py 中不同的数据导出视图格式
    :param entity:
    :param side:
    :return:
    """
    if side == "friendly":
        return friendly_export_view(entity)
    return enemy_export_view(entity)


def _velocity_components(entity, speed, azimuth, speedz):
    """
    速度向量分解，如果外部雷达直接给出了三轴速度vx,vy,vz，则直接提取，如果只给了标量速度speed和航向角azimuth,
    利用三角函数将其分解为X和Y方向上的速度分量
    :param entity:
    :param speed:
    :param azimuth:
    :param speedz:
    :return:
    """
    vx = _coerce_float(_raw_value(entity, "speedx", "vx"))
    vy = _coerce_float(_raw_value(entity, "speedy", "vy"))
    vz = _coerce_float(_raw_value(entity, "speedz", "vz"), default=speedz)
    if vx is not None and vy is not None:
        return vx, vy, float(vz or 0.0)

    heading_deg = _coerce_float(azimuth, default=_coerce_float(_raw_value(entity, "heading"), default=0.0))
    total_speed = _coerce_float(speed, default=0.0) or 0.0
    heading_rad = math.radians(heading_deg or 0.0)
    return (
        total_speed * math.cos(heading_rad),
        total_speed * math.sin(heading_rad),
        float(vz or 0.0),
    )


def _teacher_status(entity, side):
    """
    负责状态标准化：将外部状态字符清洗为小写，并将“destroyed”等字符串统一为“unnormal”；对于正常状态，复用Redis状态转换逻辑
    :param entity:
    :param side:
    :return:
    """
    raw_status = str(_raw_value(entity, "status", default="")).strip().lower()
    if raw_status in ("unnormal", "destroyed", "dead", "lost", "penetrated"):
        return "unnormal"
    return friendly_status(entity) if side == "friendly" else enemy_status(entity)


def _teacher_pitch(entity, speed, speedz):
    """
    计算俯仰角：如果原始数据无俯仰角，通过垂直速度speedz和总标量速度speed用反正切函数atan2估算出来
    :param entity:
    :param speed:
    :param speedz:
    :return:
    """


    pitch = _coerce_float(_raw_value(entity, "pitch"))
    if pitch is not None:
        return pitch
    total_speed = max(abs(_coerce_float(speed, default=0.0) or 0.0), 1e-6)
    vertical_speed = _coerce_float(speedz, default=0.0) or 0.0
    return math.degrees(math.atan2(vertical_speed, total_speed))


def _teacher_distance(entity, view):
    """计算距离防线的距离"""
    distance = _coerce_float(_raw_value(entity, "distance", "range"))
    if distance is not None:
        return distance
    base_y = CFG.OUR_BASE_LINE
    return max(0.0, float(base_y - view.get("y", base_y)))


def _teacher_drone_item(entity, drone_id, side, publish_mode="teacher", geo_reference=None):
    """
    将一架无人机的内部属性，打包为外部 UDP 协议所需的标准 JSON 字典
    :param entity:
    :param drone_id:
    :param side:
    :param publish_mode:
    :param geo_reference:
    :return:
    """
    view = _entity_view(entity, side)
    lon = _raw_value(entity, "lon", "longitude")
    lat = _raw_value(entity, "lat", "latitude")
    if publish_mode == "geo":
        geo_reference = geo_reference or GeoReference()
        lat_value, lon_value = geo_reference.xy_to_lonlat(view.get("x", 0.0), view.get("y", 0.0))
        lon = lon_value
        lat = lat_value
    altitude = _coerce_float(
        _raw_value(entity, "altitude", "alt", "alt_m", "height"),
        default=_coerce_float(view.get("z"), default=0.0),
    )
    azimuth = _coerce_float(
        _raw_value(entity, "azimuth", "heading"),
        default=_coerce_float(view.get("heading"), default=0.0),
    )
    speed = _coerce_float(_raw_value(entity, "speed"), default=_coerce_float(view.get("speed"), default=0.0))
    speedz = _coerce_float(
        _raw_value(entity, "speedz", "vz"),
        default=_coerce_float(view.get("vz"), default=0.0),
    )
    roll = _coerce_float(_raw_value(entity, "roll"), default=_coerce_float(view.get("roll"), default=0.0))
    yaw = _coerce_float(_raw_value(entity, "yaw"), default=_coerce_float(view.get("yaw"), default=azimuth))
    speedx, speedy, speedz = _velocity_components(entity, speed, azimuth, speedz)
    item = {
        "drone_id": int(drone_id),
        "lon": _coerce_float(lon, default=_coerce_float(_raw_value(entity, "x"), default=_coerce_float(view.get("x"), default=0.0))),
        "lat": _coerce_float(lat, default=_coerce_float(_raw_value(entity, "y"), default=_coerce_float(view.get("y"), default=0.0))),
        "altitude": round(float(altitude or 0.0), 3),
        "roll": round(float(roll or 0.0), 3),
        "pitch": round(float(_teacher_pitch(entity, speed, speedz)), 3),
        "yaw": round(float(yaw or 0.0), 3),
        "azimuth": round(float(azimuth or 0.0), 3),
        "speed": round(float(speed or 0.0), 3),
        "speedx": round(float(speedx or 0.0), 3),
        "speedy": round(float(speedy or 0.0), 3),
        "speedz": round(float(speedz or 0.0), 3),
        "distance": round(float(_teacher_distance(entity, view)), 3),
        "status": _teacher_status(entity, side),
    }
    if publish_mode == "geo":
        item["side"] = "ally" if side == "friendly" else "enemy"
    return item


def build_udp_packet(
    env,
    frame_num,
    stamp,
    enemy_only=False,
    friendly_start=1,
    enemy_start=101,
    publish_mode="teacher",
    geo_reference=None,
):
    """
    构建完整的UDP数据包：取当前仿真的时间戳和帧序号，提取当前存活且可见的己方拦截机和敌机列表，循环调用 _teacher_drone_item 处理每一架飞机，
    最后把它们塞进一个带有 device_id 和 timestamp 报头的超级大字典里
    :param env:
    :param frame_num:
    :param stamp:
    :param enemy_only:
    :param friendly_start:
    :param enemy_start:
    :param publish_mode:
    :param geo_reference:
    :return:
    """
    packet_meta = getattr(env, "last_live_packet_meta", {}) or {}
    device_id = packet_meta.get("device_id") or "RADAR_001"
    timestamp = packet_meta.get("timestamp")
    timestamp_value = _coerce_float(timestamp)
    if timestamp_value is None:
        timestamp_ms = int(round(stamp * 1000.0))
    elif timestamp_value < 10_000_000_000:
        timestamp_ms = int(round(timestamp_value * 1000.0))
    else:
        timestamp_ms = int(round(timestamp_value))
    seq_value = packet_meta.get("seq")
    try:
        seq = int(seq_value) if seq_value is not None else int(frame_num)
    except (TypeError, ValueError):
        seq = int(frame_num)

    friendlies = env.visible_interceptors() if hasattr(env, "visible_interceptors") else env.interceptors
    if any(item.get("external_controlled") for item in getattr(env, "interceptors", [])):
        friendlies = [item for item in friendlies if item.get("external_controlled")] or friendlies
    friendlies = sorted(friendlies, key=lambda item: item["id"])
    enemies = sorted(env.enemies, key=lambda item: item["id"])

    drone_info = []
    if not enemy_only:
        for offset, entity in enumerate(friendlies):
            drone_info.append(
                _teacher_drone_item(
                    entity,
                    friendly_start + offset,
                    "friendly",
                    publish_mode=publish_mode,
                    geo_reference=geo_reference,
                )
            )
    for offset, entity in enumerate(enemies):
        drone_info.append(
            _teacher_drone_item(
                entity,
                enemy_start + offset,
                "enemy",
                publish_mode=publish_mode,
                geo_reference=geo_reference,
            )
        )
    drone_info.sort(key=lambda item: item["drone_id"])

    return {
        "device_id": str(device_id),
        "timestamp": timestamp_ms,
        "seq": seq,
        "drone_info": drone_info,
    }


class UDPFramePublisher:
    """
    维护一个UDP Socket客户端，按照设定的频率往外发数据
    """
    def __init__(
        self,
        host="127.0.0.1",
        port=9999,
        publish_interval=0.03,
        enemy_only=False,
        friendly_start=1,
        enemy_start=101,
        publish_mode="teacher",
        geo_origin_lat=34.2663,
        geo_origin_lon=108.9549,
    ):
        self.host = host
        self.port = int(port)
        self.publish_interval = max(0.01, float(publish_interval))
        self.enemy_only = bool(enemy_only)
        self.friendly_start = int(friendly_start)
        self.enemy_start = int(enemy_start)
        self.publish_mode = str(publish_mode or "teacher").strip().lower()
        self.geo_reference = GeoReference(origin_lat=geo_origin_lat, origin_lon=geo_origin_lon)
        self.frame_num = 0
        self.last_publish_at = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def maybe_publish(self, env, force=False):
        if (
            getattr(env, "source", None) in ("udp", "redis", "auto", "fusion")
            and not getattr(env, "demo_mode", False)
            and not getattr(env, "has_live_data", False)
        ):
            return False
        now = time.time()
        if not force and (now - self.last_publish_at) < self.publish_interval:
            return False
        self.frame_num += 1
        packet = build_udp_packet(
            env,
            self.frame_num,
            now,
            enemy_only=self.enemy_only,
            friendly_start=self.friendly_start,
            enemy_start=self.enemy_start,
            publish_mode=self.publish_mode,
            geo_reference=self.geo_reference,
        )
        payload = json.dumps(packet, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(payload, (self.host, self.port))
        self.last_publish_at = now
        return True

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class TeacherUDPFeed(TeacherDataFeed):
    """
    在本地绑定一个UDP端口（默认是8020），非阻塞地接收其他设备发来的态势包
    """
    def __init__(
        self,
        bind_host="0.0.0.0",
        port=8020,
        poll_interval=0.03,
        side_filter=None,
        geo_origin_lat=34.2663,
        geo_origin_lon=108.9549,
        geo_reference=None,
    ):
        super().__init__(
            host="127.0.0.1",
            port=6379,
            db=0,
            poll_interval=poll_interval,
            side_filter=side_filter,
            geo_origin_lat=geo_origin_lat,
            geo_origin_lon=geo_origin_lon,
            geo_reference=geo_reference,
        )
        self.bind_host = bind_host
        self.port = int(port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_host, self.port))
        self.sock.setblocking(False)
        self.last_packet_time = 0.0
        self.last_sender = None

    def poll(self, sim_time: float):
        """
        每一帧拉取数据的入口：快速抽空底层socket缓冲区，只保留最新的一个数据包进行JSON解码，然后调用底层归一化逻辑将其转化为己方/敌方列表
        :param sim_time:
        :return:
        """
        now = time.time()
        meta = {
            "connected": self.last_packet_time > 0.0,
            "mode": "udp",
            "diag": "",
            "frame": None,
            "keys": 0,
            "device_id": None,
            "seq": None,
            "timestamp": None,
        }
        events = []
        result = {"enemies": [], "friendlies": [], "meta": meta, "events": events}

        latest_packet = None
        latest_sender = None
        while True:
            try:
                data, sender = self.sock.recvfrom(1024 * 1024)
            except BlockingIOError:
                break
            latest_sender = sender
            latest_packet = data

        if latest_packet is None:
            if self.last_packet_time <= 0.0:
                meta["diag"] = f"UDP在线，等待数据 {self.bind_host}:{self.port}"
            else:
                age = now - self.last_packet_time
                meta["connected"] = age <= max(self.poll_interval * 6.0, CFG.RADAR_LOST_SEC)
                meta["diag"] = f"UDP等待新包 age={age:.2f}s"
            self.last_result = result
            self.last_poll = now
            return result

        try:
            packet = json.loads(latest_packet.decode("utf-8"))
        except Exception as exc:
            meta["diag"] = f"UDP数据解析失败: {exc}"
            self.last_result = result
            self.last_poll = now
            return result

        self.last_sender = latest_sender
        self.last_packet_time = now
        enemies, friendlies = self._normalize_udp_packet(packet)
        meta["connected"] = True
        meta["frame"] = self._coerce_number(packet.get("frame"))
        meta["seq"] = self._coerce_number(packet.get("seq"))
        meta["timestamp"] = self._coerce_number(packet.get("timestamp"))
        meta["device_id"] = packet.get("device_id")
        meta["keys"] = len(enemies) + len(friendlies)
        sender_text = f"{latest_sender[0]}:{latest_sender[1]}" if latest_sender else "-"
        meta["diag"] = f"UDP数据: 敌方{len(enemies)} 己方{len(friendlies)} from={sender_text}"
        result = {
            "enemies": enemies,
            "friendlies": friendlies,
            "meta": meta,
            "events": events,
        }
        self.last_result = result
        self.last_poll = now
        return result

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def _normalize_udp_packet(self, packet):
        """
        解析解码后的JSON，区分里面的enemies和friendlies,并交给下游细化处理
        :param packet:
        :return:
        """
        top_frame = self._coerce_number(packet.get("frame"), default=self._coerce_number(packet.get("seq")))
        top_stamp = self._coerce_number(packet.get("timestamp"))

        enemies = []
        friendlies = []

        packet_enemies = packet.get("enemies")
        packet_friendlies = packet.get("friendlies")
        packet_targets = packet.get("targets")
        packet_tracks = packet.get("tracks")
        packet_drone_info = packet.get("drone_info")

        if isinstance(packet_targets, list) and not packet_enemies:
            packet_enemies = packet_targets

        if isinstance(packet_tracks, list):
            for item in packet_tracks:
                if not isinstance(item, dict):
                    continue
                side = str(item.get("side", item.get("kind", ""))).lower()
                if side in ("friendly", "uav", "ally"):
                    packet_friendlies = (packet_friendlies or []) + [item]
                else:
                    packet_enemies = (packet_enemies or []) + [item]

        if isinstance(packet_drone_info, list):
            for idx, item in enumerate(packet_drone_info):
                track = self._normalize_teacher_track(item, idx, top_frame, top_stamp)
                if not track:
                    continue
                if track["kind"] == "uav":
                    friendlies.append(track)
                else:
                    enemies.append(track)
            if self.side_filter == "enemy":
                friendlies = []
            elif self.side_filter in ("uav", "friendly", "ally"):
                enemies = []
            enemies.sort(key=lambda item: (item["stale"], item["lost"], item["y"]))
            friendlies.sort(key=lambda item: item["external_id"])
            return enemies, friendlies

        for idx, item in enumerate(packet_enemies or []):
            track = self._normalize_udp_track(item, idx, "enemy", top_frame, top_stamp)
            if track:
                enemies.append(track)

        for idx, item in enumerate(packet_friendlies or []):
            track = self._normalize_udp_track(item, idx, "uav", top_frame, top_stamp)
            if track:
                friendlies.append(track)

        if self.side_filter == "enemy":
            friendlies = []
        elif self.side_filter in ("uav", "friendly", "ally"):
            enemies = []
        enemies.sort(key=lambda item: (item["stale"], item["lost"], item["y"]))
        friendlies.sort(key=lambda item: item["external_id"])
        return enemies, friendlies

    def _normalize_teacher_track(self, item, idx, top_frame, top_stamp):
        if not isinstance(item, dict):
            return None
        drone_id = self._coerce_number(item.get("drone_id"))
        side = str(item.get("side", item.get("kind", ""))).lower()
        if side in ("friendly", "uav", "ally"):
            kind_hint = "uav"
        elif side in ("enemy", "target", "hostile"):
            kind_hint = "enemy"
        else:
            kind_hint = "enemy" if (drone_id is not None and int(drone_id) >= 100) else "uav"

        x = self._coerce_number(item.get("x"))
        y = self._coerce_number(item.get("y"))
        if x is None or y is None:
            x, y = self._project_lonlat(item)
        if x is None or y is None:
            return None

        z = self._coerce_number(item.get("z"), default=self._coerce_number(item.get("altitude"), default=0.0))
        heading = self._coerce_number(item.get("heading"), default=self._coerce_number(item.get("azimuth")))
        roll = self._coerce_number(item.get("roll"))
        pitch = self._coerce_number(item.get("pitch"))
        yaw = self._coerce_number(item.get("yaw"), default=heading)
        speed = self._coerce_number(item.get("speed"), default=0.0)
        battery = self._coerce_number(item.get("battery"), default=self._coerce_number(item.get("fuel")))
        stamp = self._coerce_number(item.get("timestamp"), default=top_stamp)
        frame = self._coerce_number(item.get("frame"), default=top_frame)
        status = item.get("status")
        ext_id = str(
            item.get("id")
            or item.get("external_id")
            or item.get("drone_id")
            or f"udp-{kind_hint}-{idx + 1}"
        )
        return self._stabilize_track(
            {
                "external_id": ext_id,
                "kind": kind_hint,
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
                "quality": float(self._coerce_number(item.get("quality"), default=1.0) or 0.0),
                "battery": float(battery) if battery is not None else None,
                "status": status,
                "raw": dict(item),
            }
        )

    def _project_lonlat(self, item):
        lon = self._coerce_number(item.get("lon"), default=self._coerce_number(item.get("longitude")))
        lat = self._coerce_number(item.get("lat"), default=self._coerce_number(item.get("latitude")))
        if lon is None or lat is None:
            return None, None
        return self.geo_reference.lonlat_to_xy(lat, lon)

    def _normalize_udp_track(self, item, idx, kind_hint, top_frame, top_stamp):
        """
        从JSON中的单个飞机字典中提取XYZ，速度和航向等
        :param item:
        :param idx:
        :param kind_hint:
        :param top_frame:
        :param top_stamp:
        :return:
        """
        if not isinstance(item, dict):
            return None
        x = self._coerce_number(item.get("x"))
        y = self._coerce_number(item.get("y"))
        if x is None or y is None:
            x, y = self._project_lonlat(item)
        if x is None or y is None:
            return None
        z = self._coerce_number(item.get("z"), default=self._coerce_number(item.get("altitude"), default=0.0))
        heading = self._coerce_number(item.get("heading"), default=self._coerce_number(item.get("azimuth")))
        roll = self._coerce_number(item.get("roll"))
        pitch = self._coerce_number(item.get("pitch"))
        yaw = self._coerce_number(item.get("yaw"), default=heading)
        speed = self._coerce_number(item.get("speed"), default=0.0)
        battery = self._coerce_number(item.get("battery"), default=self._coerce_number(item.get("fuel")))
        stamp = self._coerce_number(item.get("timestamp"), default=top_stamp)
        frame = self._coerce_number(item.get("frame"), default=top_frame)
        status = item.get("status")
        ext_id = str(item.get("id") or item.get("external_id") or f"udp-{kind_hint}-{idx + 1}")
        return self._stabilize_track(
            {
                "external_id": ext_id,
                "kind": kind_hint,
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
                "quality": float(self._coerce_number(item.get("quality"), default=1.0) or 0.0),
                "battery": float(battery) if battery is not None else None,
                "status": status,
                "raw": dict(item),
            }
        )
