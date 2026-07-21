import os
import subprocess

from core.common import EState, IState, enemy_export_view, entity_is_destroyed, friendly_export_view, friendly_view
from core.geo import GeoReference


FRIENDLY_STATUS_MAP = {
    IState.STANDBY: "standby",
    IState.LAUNCHING: "launching",
    IState.INTERCEPTING: "intercepting",
    IState.FOLLOWING: "following",
    IState.RETURNING: "returning",
    IState.DESTROYED: "destroyed",
    IState.LANDED: "landed",
}

LEGACY_KEY_PATTERNS = (
    "friendly_*",
    "enemy_*",
    "*_id",
    "total_frame",
    "friendly_total",
    "enemy_total",
)

TEACHER_STATUS_NORMAL = "normal"
TEACHER_STATUS_UNNORMAL = "unnormal"
TEACHER_TYPE_FRIENDLY = "ally"
TEACHER_TYPE_ENEMY = "enemy"
TEACHER_EXPORT_ALTITUDE_M = 50.0
GEO_HASH_PATTERNS = (
    "uav:status:uav*",
    "enemy:status:enemy*",
)


def friendly_status(entity):
    return TEACHER_STATUS_UNNORMAL if entity_is_destroyed(entity) else TEACHER_STATUS_NORMAL


def enemy_status(entity):
    return TEACHER_STATUS_UNNORMAL if entity_is_destroyed(entity) else TEACHER_STATUS_NORMAL


def _teacher_status_text(entity):
    return str(entity.get("status_text", "")).strip().lower()


def teacher_track_status(entity):
    raw_status = _teacher_status_text(entity)
    if raw_status in (TEACHER_STATUS_UNNORMAL, "destroyed", "dead", "lost"):
        return TEACHER_STATUS_UNNORMAL
    if entity.get("state") in (EState.DESTROYED, IState.DESTROYED):
        return TEACHER_STATUS_UNNORMAL
    return TEACHER_STATUS_NORMAL


def teacher_export_altitude(_entity):
    return TEACHER_EXPORT_ALTITUDE_M


def friendly_rows(interceptors, friendly_start):
    """
        功能：为己方拦截机列表生成带编号的行数据。

        参数:
            interceptors: 己方无人机对象列表。
            friendly_start: 编号起始偏移量（例如 1）。

        返回:
            list: 返回一个元组列表，格式为 [(编号, 无人机对象), ...]
    """
    # 使用enumerate获取索引idx，
    return [(friendly_start + idx, entity) for idx, entity in enumerate(interceptors)]


def enemy_rows(enemies, enemy_start):
    """
        功能：为敌方目标列表生成带编号的行数据（带有排序逻辑）。

        参数:
            enemies: 敌方目标对象列表。
            enemy_start: 编号起始偏移量（例如 101）。

        返回:
            list: 返回一个元组列表，格式为 [(编号, 敌机对象), ...]
    """
    # 先对敌机的id进行排序，这是因为敌机会被摧毁，通过排序即使列表变了，编号逻辑仍具有一定的稳定性
    ordered = sorted(enemies, key=lambda entity: entity["id"])
    return [(enemy_start + idx, entity) for idx, entity in enumerate(ordered)]


def node_keys(node_num):
    return {
        f"{node_num}_x",
        f"{node_num}_y",
        f"{node_num}_z",
        f"{node_num}_roll",
        f"{node_num}_pitch",
        f"{node_num}_yaw",
        f"{node_num}_heading",
        f"{node_num}_status",
        f"{node_num}_type",
        f"{node_num}_frame",
        f"{node_num}_timestamp",
        f"{node_num}_battery",
    }


def build_payload(friendly_rows_data, enemy_rows_data, frame_num, stamp):
    payload = {}
    active_nodes = set()

    for node_num, entity in friendly_rows_data:
        view = friendly_export_view(entity)
        active_nodes.add(node_num)
        payload[f"{node_num}_x"] = f"{view['x']:.3f}"
        payload[f"{node_num}_y"] = f"{view['y']:.3f}"
        payload[f"{node_num}_z"] = f"{view.get('z', 0.0):.3f}"
        payload[f"{node_num}_roll"] = f"{view.get('roll', 0.0):.3f}"
        payload[f"{node_num}_pitch"] = f"{view.get('pitch', 0.0):.3f}"
        payload[f"{node_num}_yaw"] = f"{view.get('yaw', view.get('heading', 0.0)):.3f}"
        payload[f"{node_num}_heading"] = f"{view.get('heading', 0.0):.3f}"
        payload[f"{node_num}_status"] = friendly_status(entity)
        payload[f"{node_num}_frame"] = frame_num
        payload[f"{node_num}_timestamp"] = f"{stamp:.6f}"
        payload[f"{node_num}_battery"] = f"{max(0.0, entity.get('fuel', 0.0)):.2f}"

    for node_num, entity in enemy_rows_data:
        view = enemy_export_view(entity)
        active_nodes.add(node_num)
        payload[f"{node_num}_x"] = f"{view['x']:.3f}"
        payload[f"{node_num}_y"] = f"{view['y']:.3f}"
        payload[f"{node_num}_z"] = f"{view.get('z', 0.0):.3f}"
        payload[f"{node_num}_roll"] = f"{view.get('roll', 0.0):.3f}"
        payload[f"{node_num}_pitch"] = f"{view.get('pitch', 0.0):.3f}"
        payload[f"{node_num}_yaw"] = f"{view.get('yaw', view.get('heading', 0.0)):.3f}"
        payload[f"{node_num}_heading"] = f"{view.get('heading', 0.0):.3f}"
        payload[f"{node_num}_status"] = enemy_status(entity)
        payload[f"{node_num}_frame"] = frame_num
        payload[f"{node_num}_timestamp"] = f"{stamp:.6f}"

    return payload, active_nodes


def build_teacher_redis_payload(friendly_rows_data, enemy_rows_data, frame_num, stamp):
    payload = {}
    active_nodes = set()

    for node_num, entity in friendly_rows_data:
        view = friendly_view(entity)
        altitude = teacher_export_altitude(entity)
        active_nodes.add(node_num)
        payload[f"{node_num}_x"] = f"{view['x']:.3f}"
        payload[f"{node_num}_y"] = f"{altitude:.3f}"
        payload[f"{node_num}_z"] = f"{view['y']:.3f}"
        payload[f"{node_num}_roll"] = f"{view.get('roll', 0.0):.3f}"
        payload[f"{node_num}_pitch"] = f"{view.get('pitch', 0.0):.3f}"
        payload[f"{node_num}_yaw"] = f"{view.get('yaw', view.get('heading', 0.0)):.3f}"
        payload[f"{node_num}_heading"] = f"{view.get('heading', 0.0):.3f}"
        payload[f"{node_num}_status"] = teacher_track_status(entity)
        payload[f"{node_num}_type"] = TEACHER_TYPE_FRIENDLY
        payload[f"{node_num}_frame"] = frame_num
        payload[f"{node_num}_timestamp"] = f"{stamp:.6f}"

    for node_num, entity in enemy_rows_data:
        view = enemy_export_view(entity)
        altitude = teacher_export_altitude(entity)
        active_nodes.add(node_num)
        payload[f"{node_num}_x"] = f"{view['x']:.3f}"
        payload[f"{node_num}_y"] = f"{altitude:.3f}"
        payload[f"{node_num}_z"] = f"{view['y']:.3f}"
        payload[f"{node_num}_roll"] = f"{view.get('roll', 0.0):.3f}"
        payload[f"{node_num}_pitch"] = f"{view.get('pitch', 0.0):.3f}"
        payload[f"{node_num}_yaw"] = f"{view.get('yaw', view.get('heading', 0.0)):.3f}"
        payload[f"{node_num}_heading"] = f"{view.get('heading', 0.0):.3f}"
        payload[f"{node_num}_status"] = teacher_track_status(entity)
        payload[f"{node_num}_type"] = TEACHER_TYPE_ENEMY
        payload[f"{node_num}_frame"] = frame_num
        payload[f"{node_num}_timestamp"] = f"{stamp:.6f}"

    return payload, active_nodes


def _geo_hash_friendly_key(node_num):
    return f"uav:status:uav{int(node_num)}"


def _geo_hash_enemy_key(node_num):
    return f"enemy:status:enemy{int(node_num)}"


def build_geo_hash_payload(friendly_rows_data, enemy_rows_data, frame_num, stamp, geo_reference=None):
    """
    负责执行坐标转换，还要把数据组装成类似 {"hash_key": {"field1": "val1", "field2": "val2"}} 的层级结构。
    :param friendly_rows_data:
    :param enemy_rows_data:
    :param frame_num:
    :param stamp:
    :param geo_reference:
    :return:
    """
    geo_reference = geo_reference or GeoReference()
    payload = {}
    active_keys = set()

    for node_num, entity in friendly_rows_data:
        view = friendly_export_view(entity)
        lat, lon = geo_reference.xy_to_lonlat(view["x"], view["y"])
        key = _geo_hash_friendly_key(node_num)
        active_keys.add(key)
        payload[key] = {
            "drone_id": f"uav{int(node_num)}",
            "lon": f"{lon:.9f}",
            "lat": f"{lat:.9f}",
            "alt": f"{view.get('z', 0.0):.3f}",
            "altitude": f"{view.get('z', 0.0):.3f}",
            "roll": f"{view.get('roll', 0.0):.3f}",
            "pitch": f"{view.get('pitch', 0.0):.3f}",
            "yaw": f"{view.get('yaw', view.get('heading', 0.0)):.3f}",
            "heading": f"{view.get('heading', 0.0):.3f}",
            "speed": f"{view.get('speed', 0.0):.3f}",
            "battery": f"{max(0.0, entity.get('fuel', 0.0)):.2f}",
            "status": friendly_status(entity),
            "frame": str(frame_num),
            "timestamp": f"{stamp:.6f}",
        }

    for node_num, entity in enemy_rows_data:
        view = enemy_export_view(entity)
        lat, lon = geo_reference.xy_to_lonlat(view["x"], view["y"])
        key = _geo_hash_enemy_key(node_num)
        active_keys.add(key)
        payload[key] = {
            "drone_id": f"enemy{int(node_num)}",
            "lon": f"{lon:.9f}",
            "lat": f"{lat:.9f}",
            "alt": f"{view.get('z', 0.0):.3f}",
            "altitude": f"{view.get('z', 0.0):.3f}",
            "roll": f"{view.get('roll', 0.0):.3f}",
            "pitch": f"{view.get('pitch', 0.0):.3f}",
            "yaw": f"{view.get('yaw', view.get('heading', 0.0)):.3f}",
            "heading": f"{view.get('heading', 0.0):.3f}",
            "speed": f"{view.get('speed', 0.0):.3f}",
            "status": enemy_status(entity),
            "frame": str(frame_num),
            "timestamp": f"{stamp:.6f}",
        }

    return payload, active_keys


def stale_keys(prev_nodes, active_nodes):
    stale = []
    for node_num in sorted(prev_nodes - active_nodes):
        stale.extend(sorted(node_keys(node_num)))
    return stale


def planned_node_nums(interceptor_count, total_enemy_count, live_enemy_count, friendly_start, enemy_start):
    """
        计算预期需要进行状态管理（发布/清理）的所有节点编号集合。

        参数:
            interceptor_count: 当前己方无人机（拦截机）的数量。
            total_enemy_count: 仿真开始至今，总共生成过的敌机总数（包含已被击毁的）。
            live_enemy_count: 当前仍在场上存活的敌机数量。
            friendly_start: 己方无人机编号的起始基数（例如 1）。
            enemy_start: 敌方目标编号的起始基数（例如 100）。
    """
    # 计算己方无人机编号集合
    friendly_nums = {
        friendly_start + idx
        for idx in range(interceptor_count)
    }
    # 计算敌机编号的最大覆盖范围
    # 如果总共生成过 5 架，目前只存活 2 架。我们需要把 5 架的编号都算上。
    # 因为这 5 架都曾经向 Redis 写过坐标，我们要把包含被击毁的这 3 架的编号也返回给上层，让上层去 Redis 里执行 Delete 删除操作。
    projected_enemy_count = max(1, int(total_enemy_count or 0), int(live_enemy_count or 0))

    # 计算敌方目标编号集合
    enemy_nums = {
        enemy_start + idx
        for idx in range(projected_enemy_count)
    }
    # 返回己方和敌方编号集合的并集
    return friendly_nums | enemy_nums


class RedisNodeWriter:
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

    def _run(self, extra_args):
        res = subprocess.run(
            self.base_cmd + extra_args,
            capture_output=True,
            text=True,
            timeout=self.timeout,
            check=False,
            env=self.env,
        )
        if res.returncode != 0:
            raise RuntimeError((res.stderr or res.stdout or "redis-cli command failed").strip())
        return res.stdout

    def mset(self, mapping):
        if not mapping:
            return
        cmd = ["MSET"]
        for key, value in mapping.items():
            cmd.extend([str(key), str(value)])
        self._run(cmd)

    def delete(self, keys):
        keys = [str(key) for key in keys if key]
        if keys:
            self._run(["DEL"] + keys)

    def hset_mapping(self, key, mapping):
        if not mapping:
            return
        cmd = ["HSET", str(key)]
        for field, value in mapping.items():
            cmd.extend([str(field), str(value)])
        self._run(cmd)

    def bulk_hset(self, mappings):
        for key, mapping in mappings.items():
            self.hset_mapping(key, mapping)

    def scan(self, pattern):
        output = self._run(["--scan", "--pattern", str(pattern)])
        return [line.strip() for line in output.splitlines() if line.strip()]

    def cleanup_legacy_keys(self):
        legacy = set()
        for pattern in LEGACY_KEY_PATTERNS:
            legacy.update(self.scan(pattern))
        self.delete(sorted(legacy))

    def cleanup_hash_keys(self):
        legacy = set()
        for pattern in GEO_HASH_PATTERNS:
            legacy.update(self.scan(pattern))
        self.delete(sorted(legacy))

    def cleanup_node_nums(self, node_nums):
        keys = []
        for node_num in sorted(node_nums):
            keys.extend(sorted(node_keys(node_num)))
        self.delete(keys)
