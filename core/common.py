"""
MARL公共模块 - 空域拦截系统
v8.0:
1. 场景尺寸支持 1/2/3/4/5/10km 动态缩放
2. 运动与拦截计算升级为 3D，界面仍可保持 2D 投影
3. 为实时雷达数据接入预留稳态阈值与容错参数
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import Tuple


@dataclass
class Config:
    SCENE_KM: float = 10.0

    # 空域 (米)
    AREA_WIDTH: float = 10000.0
    AREA_HEIGHT: float = 12000.0

    ENEMY_SPAWN_LINE: float = 0.0
    DETECTION_LINE: float = 2000.0
    INTERCEPT_FAIL_LINE: float = 10000.0
    OUR_BASE_LINE: float = 12000.0

    # 拦截机
    NUM_INTERCEPTORS: int = 10
    INTERCEPTOR_SPEED: float = 24.0
    INTERCEPTOR_BOOST_SPEED: float = 42.0
    INTERCEPTOR_NET_SPEED: float = 38.0
    INTERCEPTOR_BARRIER_SPEED: float = 15.0
    INTERCEPTOR_FAST_MARGIN: float = 2.0
    INTERCEPTOR_CLIMB_RATE: float = 18.0
    INTERCEPTOR_MAX_ALT: float = 50.0
    INTERCEPTOR_LAUNCH_ALT: float = 10.0
    INTERCEPTOR_CRUISE_ALT: float = 28.0
    INTERCEPTOR_TERMINAL_ALT: float = 42.0
    INTERCEPTOR_RETURN_ALT: float = 16.0
    INTERCEPTOR_ALT_LAYER_STEP: float = 4.0
    #  提高机动性以适应3米判定
    INTERCEPTOR_MAX_ANG: float = 35.0
    INTERCEPTOR_ENDURANCE: float = 600.0

    # 机巢横向分布，具体数量会按规模自动扩展
    HANGAR_MODE: str = "multi"
    HANGAR_POSITIONS: Tuple[float] = (5000.0,)

    # 敌机 - 真实战术参数
    ENEMY_SPEED: float = 20.0
    ENEMY_SPEED_VAR: float = 5.0
    ENEMY_CLIMB_RATE: float = 12.0
    ENEMY_MAX_ALT: float = 48.0
    ENEMY_MANEUVER_RATE: float = 45.0
    ENEMY_HDG_BASE: float = 90.0
    ENEMY_HDG_VAR: float = 10.0

    # 战术参数
    LOITER_RADIUS: float = 600.0
    LOITER_DURATION: float = 15.0
    DECOY_SPEED_MULT: float = 1.5

    # 拦截
    # 3.0米严苛判定
    INTERCEPT_RADIUS: float = 5.0
    REDUNDANCY_OFFSET: float = 150.0
    PRONAV_GAIN: float = 5.0  # 提高导引律增益
    POI_MARGIN: float = 100.0

    # 数据链/目标质量
    RADAR_POLL_INTERVAL: float = 0.4
    RADAR_STALE_SEC: float = 4.0
    RADAR_LOST_SEC: float = 12.0
    TARGET_SEARCH_SEC: float = 10.0
    TARGET_CONFIRM_SEC: float = 1.2
    POSITION_SMOOTHING: float = 0.35
    MAX_TRACK_JUMP_M: float = 850.0
    MISCLASSIFY_CONFIDENCE: float = 0.55
    FRIENDLY_SAFE_SEPARATION: float = 120.0
    FRIENDLY_COLLISION_RADIUS: float = 28.0
    FORMATION_SPACING: float = 140.0
    RESERVE_INTERCEPTOR_BUFFER: int = 2
    MIN_FREE_INTERCEPTORS_FOR_FOLLOWER: int = 4
    HIGH_PRESSURE_THREAT_RATIO: float = 0.45
    TERMINAL_GUIDE_RANGE: float = 550.0
    TELEMETRY_BLEND: float = 0.18
    TELEMETRY_MAX_CORRECTION: float = 120.0
    LOCAL_PLANNER_ENABLE: bool = True
    LOCAL_PLANNER_REPLAN_SEC: float = 0.18
    LOCAL_PLANNER_TIME_HORIZON_SEC: float = 2.0
    LOCAL_PLANNER_NEIGHBOR_DIST: float = 220.0
    LOCAL_PLANNER_MAX_NEIGHBORS: int = 6
    BARRIER_CORE_KEEP_OUT_MARGIN: float = 18.0
    BARRIER_BUFFER_MARGIN: float = 90.0
    BASE_OUTBOUND_CORRIDOR_WIDTH: float = 120.0
    BASE_INBOUND_CORRIDOR_WIDTH: float = 120.0
    BASE_CORRIDOR_CAPACITY: int = 2
    ROUTE_HYSTERESIS_SEC: float = 1.1
    ROUTE_SWITCH_MIN_GAIN_M: float = 70.0
    BARRIER_WAIT_TIMEOUT_SEC: float = 8.0
    NET_GROUP_SIZE: int = 4
    MAX_CONCURRENT_NET_TARGETS: int = 2
    NET_RESOURCE_BUFFER: int = 4
    NET_TYPE_NAMES: Tuple[str, ...] = ("SNAKE", "JINK", "LOITER")
    NET_CAPTURE_RADIUS: float = 220.0
    NET_SLOT_TOLERANCE: float = 65.0
    NET_CLOSE_RADIUS: float = 120.0
    NET_CLOSE_TOLERANCE: float = 90.0
    NET_LEAD_TIME: float = 2.2
    NET_CAPTURE_HOLD: float = 0.35
    NET_MIN_SPAN_DEG: float = 180.0
    BARRIER_GROUP_SIZE: int = 4
    MAX_CONCURRENT_BARRIER_TARGETS: int = 2
    BARRIER_RESOURCE_BUFFER: int = 4
    BARRIER_TYPE_NAMES: Tuple[str, ...] = ("SNAKE", "JINK", "LOITER", "DASH")
    BARRIER_STATION_OFFSET: float = 1500.0
    BARRIER_HALF_WIDTH: float = 240.0
    BARRIER_HALF_DEPTH: float = 90.0
    BARRIER_SLOT_SPACING: float = 150.0
    BARRIER_NET_RADIUS: float = 180.0
    BARRIER_SLOT_TOLERANCE: float = 65.0
    BARRIER_CAPTURE_MARGIN: float = 35.0
    BARRIER_TIME_MARGIN: float = 0.8
    BARRIER_ALT_BASE: float = 18.0
    BARRIER_ALT_STEP: float = 6.0
    BARRIER_REPOSITION_TRIGGER_X: float = 120.0
    BARRIER_REPOSITION_TRIGGER_Y: float = 60.0
    BARRIER_REPOSITION_RATE: float = 15.0
    HIT_2D_RADIUS: float = 26.0
    HIT_ALT_TOLERANCE: float = 140.0

    # 波次
    WAVE_INTERVAL: float = 50.0
    LAUNCH_DELAY: float = 2.0

    # 仿真
    # 0.01秒高精物理步长
    DT: float = 0.01
    TIME_LIMIT: float = float("inf")

    # 显示
    SCREEN_WIDTH: int = 1400
    SCREEN_HEIGHT: int = 850
    FPS: int = 60
    SEED: int = 42

    def apply_scene_scale(self, scene_km: float):
        scene_km = max(1.0, float(scene_km))
        scene_m = scene_km * 1000.0

        self.SCENE_KM = scene_km
        self.AREA_WIDTH = scene_m
        self.INTERCEPT_FAIL_LINE = scene_m
        self.DETECTION_LINE = max(250.0, scene_m * 0.2)
        self.OUR_BASE_LINE = scene_m * 1.2
        self.AREA_HEIGHT = self.OUR_BASE_LINE

        self.LOITER_RADIUS = max(150.0, scene_m * 0.06)
        self.REDUNDANCY_OFFSET = max(80.0, scene_m * 0.015)
        self.POI_MARGIN = max(60.0, scene_m * 0.01)
        self.FRIENDLY_SAFE_SEPARATION = max(90.0, scene_m * 0.018)
        self.FRIENDLY_COLLISION_RADIUS = max(22.0, scene_m * 0.0035)
        self.FORMATION_SPACING = max(110.0, scene_m * 0.022)
        self.TERMINAL_GUIDE_RANGE = max(450.0, scene_m * 0.11)
        self.TELEMETRY_MAX_CORRECTION = max(80.0, scene_m * 0.02)
        self.LOCAL_PLANNER_NEIGHBOR_DIST = max(180.0, self.FRIENDLY_SAFE_SEPARATION * 2.1)
        self.BARRIER_CORE_KEEP_OUT_MARGIN = max(10.0, self.FRIENDLY_SAFE_SEPARATION * 0.12)
        self.BARRIER_BUFFER_MARGIN = max(70.0, self.FRIENDLY_SAFE_SEPARATION * 0.95)
        self.BASE_OUTBOUND_CORRIDOR_WIDTH = max(110.0, self.FRIENDLY_SAFE_SEPARATION * 1.25)
        self.BASE_INBOUND_CORRIDOR_WIDTH = max(110.0, self.FRIENDLY_SAFE_SEPARATION * 1.15)
        self.ROUTE_SWITCH_MIN_GAIN_M = max(55.0, self.FORMATION_SPACING * 0.6)
        self.NET_CAPTURE_RADIUS = max(180.0, scene_m * 0.05)
        self.NET_SLOT_TOLERANCE = max(55.0, scene_m * 0.012)
        self.NET_CLOSE_RADIUS = max(90.0, scene_m * 0.02)
        self.NET_CLOSE_TOLERANCE = max(75.0, scene_m * 0.018)
        self.NET_LEAD_TIME = min(3.0, max(1.6, scene_m / 2200.0))
        self.NET_CAPTURE_HOLD = 0.35
        self.NET_MIN_SPAN_DEG = 180.0
        self.BARRIER_STATION_OFFSET = min(1500.0, max(350.0, scene_m * 0.3))
        self.BARRIER_HALF_WIDTH = max(160.0, scene_m * 0.045)
        self.BARRIER_HALF_DEPTH = max(60.0, scene_m * 0.015)
        self.BARRIER_SLOT_SPACING = max(120.0, scene_m * 0.028)
        self.BARRIER_NET_RADIUS = max(150.0, scene_m * 0.036)
        self.BARRIER_SLOT_TOLERANCE = max(45.0, scene_m * 0.01)
        self.BARRIER_CAPTURE_MARGIN = max(28.0, scene_m * 0.007)
        self.BARRIER_REPOSITION_TRIGGER_X = max(90.0, scene_m * 0.018)
        self.BARRIER_REPOSITION_TRIGGER_Y = max(40.0, scene_m * 0.008)
        self.BARRIER_REPOSITION_RATE = max(15.0, min(self.INTERCEPTOR_BARRIER_SPEED * 1.1, 20.0))
        self.HIT_2D_RADIUS = max(22.0, scene_m * 0.005)
        self.HIT_ALT_TOLERANCE = max(100.0, scene_m * 0.03)

        flight_window = scene_m / max(self.ENEMY_SPEED, 1.0)
        self.WAVE_INTERVAL = max(12.0, flight_window * 0.35)
        self.TIME_LIMIT = float("inf")

        if str(self.HANGAR_MODE).lower() == "single":
            hangar_count = 1
        else:
            hangar_count = max(1, min(6, math.ceil(self.NUM_INTERCEPTORS / 5.0)))
        if hangar_count == 1:
            self.HANGAR_POSITIONS = (scene_m * 0.5,)
        else:
            x_margin = max(scene_m * 0.12, self.FRIENDLY_SAFE_SEPARATION * 1.4)
            usable_span = max(self.FORMATION_SPACING, scene_m - 2.0 * x_margin)
            self.HANGAR_POSITIONS = tuple(
                x_margin + usable_span * (idx / float(hangar_count - 1))
                for idx in range(hangar_count)
            )

    def set_hangar_mode(self, mode: str):
        mode_text = str(mode or "multi").strip().lower()
        self.HANGAR_MODE = "single" if mode_text == "single" else "multi"


CFG = Config()
CFG.apply_scene_scale(CFG.SCENE_KM)

# 设备2 -> 设备一决策输出配置。
# 与 CFG 放在同一公共配置入口，避免另设根目录配置文件。
PLAN_EXPORT = {
    "enabled": False,
    "transport": "tcp_json_lines",
    "publish_policy": "input_frame",
    "output_frame_type": "decision_output_frame",
    "publish_interval": 1.0,
    "valid_duration_ms": 1500,
    "waypoint_count": 3,
    "waypoint_spacing_m": 500.0,
    "primary_control_mode": "mission_waypoints",
    "supported_control_modes": ("mission_waypoints", "goto"),
    "replace_policy": "task_change_immediate_same_task_smooth",
    "smooth_replace_threshold_m": 30.0,
    "min_reupload_interval_sec": 2.0,
    "uav_id_map": "",
    "preflight_takeoff_alt_m": 15.0,
    "socket_host": "127.0.0.1",
    "socket_port": 7001,
    "socket_reconnect_sec": 1.0,
    "socket_connect_timeout_sec": 0.2,
    "debug_redis_enable": False,
    "debug_redis_host": "127.0.0.1",
    "debug_redis_port": 6379,
    "debug_redis_db": 0,
    "debug_redis_password": "",
    "debug_redis_key": "d2:d3:plan_latest",
    "failed_uav_cooldown_sec": 5.0,
    "kafka_enable": False,
    "kafka_topic": "d2.d3.plan_frame",
}

# 旧地面站（station）旁路转接配置，仅作为调试/兼容转接保留。
STATION_BRIDGE = {
    "enabled": False,
    "host": "127.0.0.1",
    "port": 7101,
    "reconnect_sec": 1.0,
    "connect_timeout_sec": 0.2,
    "strict_binding": True,
}

# 重力加速度常量
GRAVITY_MPS2 = 9.80665


def _to_float(value, default=0.0):
    """安全转换工具：将输入尝试转为浮点数，失败则返回默认值"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp(v, lo, hi):
    """限制工具：确保数值 v 在 [lo, hi] 范围内"""
    return max(lo, min(hi, v))


def _angle_diff_raw(a, b):
    """计算角度差：返回 a - b 的最小角度差，范围在 [-180, 180]"""
    return (a - b + 180) % 360 - 180


def compute_attitude_from_motion(entity, dt=0.0, roll=None, pitch=None, yaw=None, prev_heading=None):
    """
        核心算法：根据遥测数据或飞行运动学推导无人机姿态角
        输入:
            entity: 包含无人机状态信息的字典 (speed, heading, vz 等)
            dt: 时间间隔
            roll/pitch/yaw: 若传入则使用固定值，否则自动计算

        marl.main.py循环中，每当雷达数据更新或无人机位置移动后，系统会调用此函数更新 entity 字典。这样，后续模块（如 UI 显示或设备 3 的态势显示）
            就能从该字典中直接读取到合法的 roll, pitch, yaw
        integrations/middle_layer.py：函数计算出的 roll/pitch/yaw 会直接决定 Unity 中无人机模型在场景里的渲染姿态
        """
    # 基础数据解析
    heading = _to_float(entity.get('heading', entity.get('yaw', 0.0))) % 360.0
    speed = max(0.0, _to_float(entity.get('speed', 0.0)))
    vz = _to_float(entity.get('vz', 0.0))

    # 记录上一帧航向，用于计算角速度
    if prev_heading is None:
        prev_heading = entity.get('_att_prev_heading', heading)
    prev_heading = _to_float(prev_heading, heading)

    # 偏航角 (Yaw)：若未指定则直接使用当前航向
    yaw_value = heading if yaw is None else _to_float(yaw, heading) % 360.0

    # 俯仰角 (Pitch)：基于垂直速度 vz 和水平速度 speed 计算爬升/俯冲角
    if pitch is None:
        if speed > 0.05 or abs(vz) > 0.05:
            pitch_value = math.degrees(math.atan2(vz, max(speed, 0.05)))
        else:
            pitch_value = 0.0
    else:
        pitch_value = _to_float(pitch, 0.0)

    # 滚转角 (Roll)：基于转向角速度 (yaw_rate) 计算离心力产生的倾斜
    if roll is None:
        if dt and dt > 1e-6 and speed > 0.2:
            yaw_rate = math.radians(_angle_diff_raw(heading, prev_heading)) / float(dt)
            roll_value = math.degrees(math.atan((speed * yaw_rate) / GRAVITY_MPS2))
        else:
            roll_value = 0.0
    else:
        roll_value = _to_float(roll, 0.0)

    # 结果限幅与存储
    roll_limit = 85.0 if roll is not None else 70.0
    entity['roll'] = _clamp(roll_value, -roll_limit, roll_limit)
    entity['pitch'] = _clamp(pitch_value, -85.0, 85.0)
    entity['yaw'] = yaw_value
    entity['_att_prev_heading'] = heading # 更新上一帧数据
    return entity['roll'], entity['pitch'], entity['yaw']


class IState(Enum):
    # 定义了己方无人机在拦截任务全生命周期中的各种状态
    STANDBY = 0  # 待机中，未分配任务
    LAUNCHING = 1  # 发射/起飞阶段，处于升空过程
    INTERCEPTING = 2  # 拦截中，正在飞往拦截点的过程中
    FOLLOWING = 3  # 伴飞/跟随中，对目标进行跟踪或备份监视
    RETURNING = 4  # 返航中，任务结束或目标丢失后回归
    DESTROYED = 5  # 被毁，仿真中该单元已失效
    LANDED = 6  # 已着陆，处于地面静止状态


class EState(Enum):
    # 描述雷达监测到的敌方目标的当前机动行为或生存状态
    APPROACHING = 0  # 正在逼近我方防线
    MANEUVERING = 1  # 正在进行复杂机动（如蛇形机动等）
    DESTROYED = 2  # 已被拦截或摧毁
    PENETRATED = 3  # 已突破防线


class IRole(Enum):
    # 己方无人机角色
    PRIMARY = "主拦截"  # 主攻手，负责执行最终撞击或打击
    FOLLOWER = "随动机"  # 僚机/备份机，负责补位或提供数据支持
    RESERVE = "待命"  # 预备队，等待分配


class EType(Enum):
    # 敌方目标类型
    NORMAL = 0  # 普通直线飞行
    SNAKE = 1  # 蛇形机动
    JINK = 2  # 诡异机动（大过载摆动）
    DASH = 3  # 突防冲刺
    LOITER = 4  # 盘旋/巡航
    DECOY = 5  # 诱饵/假目标


def create_interceptor(iid, rng=None):
    """创建己方拦截机"""
    # 逻辑是将多架无人机按矩阵排列在防线内侧
    hangar_count = max(1, len(CFG.HANGAR_POSITIONS))
    hangar_idx = iid % hangar_count
    stack_idx = iid // hangar_count
    base_x = CFG.HANGAR_POSITIONS[hangar_idx]
    # 计算行列偏移，保持无人机间距
    local_row = stack_idx // 3
    local_col = (stack_idx % 3) - 1
    lateral_step = max(26.0, CFG.FRIENDLY_SAFE_SEPARATION * 0.42)
    longitudinal_step = max(30.0, CFG.FRIENDLY_SAFE_SEPARATION * 0.38)
    # 确定出生点坐标 (spawn_x, spawn_y)
    spawn_x = max(0.0, min(CFG.AREA_WIDTH, base_x + local_col * lateral_step))
    spawn_y = CFG.INTERCEPT_FAIL_LINE + 200 + local_row * longitudinal_step
    # 返回无人机的全量状态字典，这些键值对构成了系统对单机状态的全部认知
    return {'id': iid, 'x': spawn_x, 'y': spawn_y,  # 稍微靠后一点，在防线内侧
            'z': 0.0, 'vz': 0.0, 'target_z': 0.0,
            'heading': 270.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 270.0,
            '_att_prev_heading': 270.0, 'speed': 0.0, 'state': IState.STANDBY,
            'role': IRole.RESERVE, 'target_id': None, 'partner_id': None,
            'fuel': CFG.INTERCEPTOR_ENDURANCE, 'launch_time': -1.0,
            'flight_time': 0.0, 'poi': None, 'poi_time': None,
            'path_plan': [], 'path_reason': "", 'search_until': 0.0,
            'reported_x': spawn_x, 'reported_y': spawn_y, 'reported_z': 0.0,
            'reported_speed': 0.0, 'reported_heading': 270.0, 'reported_at': -1.0,
            'reported_vz': 0.0, 'reported_roll': 0.0, 'reported_pitch': 0.0,
            'reported_yaw': 270.0, 'reported_frame': None,
            'mission_label': "待命", 'target_label': "-", 'net_slot': None,
            'barrier_slot': None, 'barrier_center': None,
            'hangar_idx': hangar_idx, 'z_cap': CFG.INTERCEPTOR_MAX_ALT,
            'return_fast': False, 'local_avoid_mode': "",
            'local_hold_reason': "", 'local_plan_stamp': -1.0,
            'external_id': f"sim-uav-{iid+1:02d}",
            'track_quality': 1.0, 'status_text': "", 'stale': False,
            'lost': False, 'last_update': -1.0, 'age': 0.0,
            'frame': None, 'source': 'sim', 'external_controlled': False,
            'search_point': None,
            'search_distance': 0.0,
            'task_reserved': False,
            'jammed_by_interference': False, 'jam_zone': None,
            'jam_since': -1.0,
            'jam_loss_logged': False,
            'raw_track': {}}  # 记录所属机槽


def create_enemy(eid, x, speed, heading, spawn_time, rng_val=0.0):
    r = rng_val
    # 根据随机数 r 决定敌机的机动类型 (EType)
    etype = EType.NORMAL
    if r < 0.1:
        etype = EType.LOITER; speed *= 0.8
    elif r < 0.3:
        etype = EType.DECOY; speed *= CFG.DECOY_SPEED_MULT
    elif r < 0.5:
        etype = EType.DASH; speed *= 1.3
    elif r < 0.8:
        etype = EType.SNAKE if r < 0.65 else EType.JINK
    # 根据机动类型决定飞行高度
    altitude_map = {
        EType.NORMAL: 24.0,
        EType.SNAKE: 30.0,
        EType.JINK: 34.0,
        EType.DASH: 28.0,
        EType.LOITER: 38.0,
        EType.DECOY: 20.0,
    }
    enemy_z = min(CFG.ENEMY_MAX_ALT, altitude_map.get(etype, 24.0) + (eid % 2) * 2.0)

    # 返回敌方目标的初始字典，包含状态和类型
    return {'id': eid, 'x': x, 'y': CFG.ENEMY_SPAWN_LINE + 50, 'z': enemy_z, 'vz': 0.0,
            'z_cap': CFG.ENEMY_MAX_ALT, 'target_z': enemy_z, 'climb_rate': CFG.ENEMY_CLIMB_RATE,
            'heading': heading, 'roll': 0.0, 'pitch': 0.0, 'yaw': heading % 360.0,
            '_att_prev_heading': heading % 360.0, 'speed': speed,
            'state': EState.APPROACHING, 'spawn_time': spawn_time,
            'maneuver_timer': 0.0, 'detected': False, 'detect_time': -1.0,
            'type': etype, 'phase': r * 100, 'target_heading': heading,
            'loiter_center': None, 'loiter_timer': CFG.LOITER_DURATION, 'is_diving': False,
            'external_id': f"sim-{eid+1:02d}", 'track_quality': 1.0, 'stale': False,
            'lost': False, 'last_update': spawn_time, 'classification_confidence': 1.0,
            'source': 'demo', 'raw_track': {}}


def move_entity(e, dt):
    """
        更新实体的物理位置与姿态 (核心仿真循环函数)
        参数:
            e: 实体字典 (包含 x, y, z, speed, heading, vz 等)
            dt: 时间步长 (秒)
        """
    prev_heading = e.get('_att_prev_heading', e.get('heading', 0.0))
    # 1. 位置更新：根据当前航向和速度计算位移
    r = math.radians(e['heading'])
    e['x'] += math.cos(r) * e['speed'] * dt
    e['y'] += math.sin(r) * e['speed'] * dt
    e['x'] = max(0, min(CFG.AREA_WIDTH, e['x']))
    e['y'] = max(0, min(CFG.AREA_HEIGHT, e['y']))
    # 2. 高度更新：处理垂直运动
    if 'z' in e:
        target_z = e.get('target_z')
        # 如果存在目标高度且具备爬升率，则向目标高度逼近
        if target_z is not None and e.get('climb_rate', 0.0) > 0:
            dz = target_z - e['z']
            step = max(-e['climb_rate'] * dt, min(e['climb_rate'] * dt, dz))
            e['z'] += step
            e['vz'] = step / dt if dt > 0 else 0.0
        else:
            e['z'] += e.get('vz', 0.0) * dt
        z_cap = e.get('z_cap')
        if z_cap is not None:
            e['z'] = min(z_cap, e['z'])
        e['z'] = max(0.0, e['z'])
    # 3. 姿态更新：调用姿态计算逻辑更新 roll/pitch/yaw
    compute_attitude_from_motion(e, dt, prev_heading=prev_heading)


def angle_diff(a, b):
    """返回两个角度之间的最小差值，范围在 [-180, 180] 度之间"""
    return (a - b + 180) % 360 - 180


def dist2d(a, b):
    """计算两个实体在二维平面上的欧几里得距离"""
    dx, dy = a['x'] - b['x'], a['y'] - b['y']
    return math.sqrt(dx * dx + dy * dy)


def dist3d(a, b):
    """计算两个实体在三维空间中的欧几里得距离"""
    dz = a.get('z', 0.0) - b.get('z', 0.0)
    dxy = dist2d(a, b)
    return math.sqrt(dxy * dxy + dz * dz)


def friendly_is_external(entity):
    """判断无人机是否由外部执行侧（如设备一飞控链路）实际控制"""
    # 只要 entity 中标记了 external_controlled 为 True，且上报时间戳有效，即认为由外部控制
    return bool(entity.get('external_controlled')) and entity.get('reported_at', -1.0) >= 0.0


def entity_is_destroyed(entity):
    """鲁棒性判断：判定实体是否已失效（被摧毁或失联）"""
    # 1. 检查状态枚举值是否为 DESTROYED
    state = entity.get('state')
    if state in (IState.DESTROYED, EState.DESTROYED):
        return True
    # 2. 检查文本描述（适配各种来源的脏数据）
    for key in ('status_text', 'status'):
        raw = str(entity.get(key, '')).strip().lower()
        if raw in ('unnormal', 'destroyed', 'dead', 'lost'):
            return True
    # 3. 深入检查原始轨迹数据 raw_track (适配雷达接入的数据)
    raw_track = entity.get('raw_track') or {}
    for key in ('status', 'node_status'):
        raw = str(raw_track.get(key, '')).strip().lower()
        if raw in ('unnormal', 'destroyed', 'dead', 'lost'):
            return True

    return False


def friendly_view(entity):
    """生成友好的无人机视图：区分‘自身仿真数据’与‘外部上报数据’"""
    # 若是外部控制，优先选用 reported_x/y/z 等上报坐标，而非仿真坐标
    if friendly_is_external(entity):
        # 返回包含外部上报状态的数据包
        return {
            'id': entity.get('id'),
            'external_id': entity.get('external_id'),
            'x': entity.get('reported_x', entity.get('x', 0.0)),
            'y': entity.get('reported_y', entity.get('y', 0.0)),
            'z': entity.get('reported_z', entity.get('z', 0.0)),
            'vz': entity.get('reported_vz', entity.get('vz', 0.0)),
            'speed': entity.get('reported_speed', entity.get('speed', 0.0)),
            'heading': entity.get('reported_heading', entity.get('heading', 0.0)),
            'roll': entity.get('reported_roll', entity.get('roll', 0.0)),
            'pitch': entity.get('reported_pitch', entity.get('pitch', 0.0)),
            'yaw': entity.get('reported_yaw', entity.get('yaw', entity.get('heading', 0.0))),
            'fuel': entity.get('fuel', 0.0),
            'state': entity.get('state'),
            'role': entity.get('role'),
            'status_text': entity.get('status_text', ''),
            'reported_at': entity.get('reported_at', -1.0),
            'source': entity.get('source', 'sim'),
        }
    # 否则返回纯仿真状态的数据包
    return {
        'id': entity.get('id'),
        'external_id': entity.get('external_id'),
        'x': entity.get('x', 0.0),
        'y': entity.get('y', 0.0),
        'z': entity.get('z', 0.0),
        'vz': entity.get('vz', 0.0),
        'speed': entity.get('speed', 0.0),
        'heading': entity.get('heading', 0.0),
        'roll': entity.get('roll', 0.0),
        'pitch': entity.get('pitch', 0.0),
        'yaw': entity.get('yaw', entity.get('heading', 0.0)),
        'fuel': entity.get('fuel', 0.0),
        'state': entity.get('state'),
        'role': entity.get('role'),
        'status_text': entity.get('status_text', ''),
        'reported_at': entity.get('reported_at', -1.0),
        'source': entity.get('source', 'sim'),
    }


def friendly_export_origin(entity):
    """计算导出坐标的基准原点（坐标偏移校准）"""
    # 根据无人机的 hangars_idx 确定对应的原点位置，用于将全局坐标转换为相对坐标
    hangar_positions = CFG.HANGAR_POSITIONS or (CFG.AREA_WIDTH * 0.5,)
    try:
        hangar_idx = int(entity.get('hangar_idx', 0) or 0)
    except (TypeError, ValueError):
        hangar_idx = 0
    hangar_idx = max(0, min(len(hangar_positions) - 1, hangar_idx))
    return {
        'x': float(hangar_positions[hangar_idx]),
        'y': float(CFG.INTERCEPT_FAIL_LINE + 200.0),
        'z': 0.0,
    }


def friendly_export_view(entity):
    """导出格式：将绝对坐标转换为相对导出原点的相对坐标"""
    view = dict(friendly_view(entity))
    origin = friendly_export_origin(entity)
    view['x'] = float(view.get('x', 0.0)) - origin['x']
    view['y'] = float(view.get('y', 0.0)) - origin['y']
    return view


def enemy_view(entity):
    """生成敌方目标的精简展示视图"""
    return {
        'id': entity.get('id'),
        'external_id': entity.get('external_id'),
        'x': entity.get('x', 0.0),
        'y': entity.get('y', 0.0),
        'z': entity.get('z', 0.0),
        'vz': entity.get('vz', 0.0),
        'speed': entity.get('speed', 0.0),
        'heading': entity.get('heading', 0.0),
        'roll': entity.get('roll', 0.0),
        'pitch': entity.get('pitch', 0.0),
        'yaw': entity.get('yaw', entity.get('heading', 0.0)),
        'state': entity.get('state'),
        'type': entity.get('type'),
        'status_text': entity.get('status_text', ''),
        'reported_at': entity.get('reported_at', -1.0),
        'source': entity.get('source', 'sim'),
    }


def enemy_export_view(entity):
    """敌方导出视图（当前与 enemy_view 逻辑一致）"""
    return enemy_view(entity)
