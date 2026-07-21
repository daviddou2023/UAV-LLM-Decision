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

    # 单机巢: 所有拦截机从中央同一点起飞
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
    RADAR_STALE_SEC: float = 2.5
    RADAR_LOST_SEC: float = 6.0
    TARGET_SEARCH_SEC: float = 5.0
    TARGET_CONFIRM_SEC: float = 1.2
    POSITION_SMOOTHING: float = 0.35
    MAX_TRACK_JUMP_M: float = 850.0
    MISCLASSIFY_CONFIDENCE: float = 0.55
    FRIENDLY_SAFE_SEPARATION: float = 120.0
    FRIENDLY_COLLISION_RADIUS: float = 28.0
    FORMATION_SPACING: float = 140.0
    TERMINAL_GUIDE_RANGE: float = 550.0
    TELEMETRY_BLEND: float = 0.18
    TELEMETRY_MAX_CORRECTION: float = 120.0
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
    TIME_LIMIT: float = 900.0

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

        self.HANGAR_POSITIONS = (scene_m * 0.5,)
        self.LOITER_RADIUS = max(150.0, scene_m * 0.06)
        self.REDUNDANCY_OFFSET = max(80.0, scene_m * 0.015)
        self.POI_MARGIN = max(60.0, scene_m * 0.01)
        self.FRIENDLY_SAFE_SEPARATION = max(90.0, scene_m * 0.018)
        self.FRIENDLY_COLLISION_RADIUS = max(22.0, scene_m * 0.0035)
        self.FORMATION_SPACING = max(110.0, scene_m * 0.022)
        self.TERMINAL_GUIDE_RANGE = max(450.0, scene_m * 0.11)
        self.TELEMETRY_MAX_CORRECTION = max(80.0, scene_m * 0.02)
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
        self.TIME_LIMIT = max(180.0, flight_window * 3.5)


CFG = Config()
CFG.apply_scene_scale(CFG.SCENE_KM)


class IState(Enum):
    STANDBY = 0;
    LAUNCHING = 1;
    INTERCEPTING = 2;
    FOLLOWING = 3
    RETURNING = 4;
    DESTROYED = 5;
    LANDED = 6


class EState(Enum):
    APPROACHING = 0;
    MANEUVERING = 1;
    DESTROYED = 2;
    PENETRATED = 3


class IRole(Enum):
    PRIMARY = "主拦截";
    FOLLOWER = "随动机";
    RESERVE = "待命"


class EType(Enum):
    NORMAL = 0;
    SNAKE = 1;
    JINK = 2;
    DASH = 3;
    LOITER = 4;
    DECOY = 5


def create_interceptor(iid, rng=None):
    hangar_idx = 0
    base_x = CFG.HANGAR_POSITIONS[hangar_idx]

    return {'id': iid, 'x': base_x, 'y': CFG.INTERCEPT_FAIL_LINE + 200,  # 稍微靠后一点，在防线内侧
            'z': 0.0, 'vz': 0.0, 'target_z': 0.0,
            'heading': 270.0, 'speed': 0.0, 'state': IState.STANDBY,
            'role': IRole.RESERVE, 'target_id': None, 'partner_id': None,
            'fuel': CFG.INTERCEPTOR_ENDURANCE, 'launch_time': -1.0,
            'flight_time': 0.0, 'poi': None, 'poi_time': None,
            'path_plan': [], 'path_reason': "", 'search_until': 0.0,
            'reported_x': base_x, 'reported_y': CFG.INTERCEPT_FAIL_LINE + 200, 'reported_z': 0.0,
            'reported_speed': 0.0, 'reported_heading': 270.0, 'reported_at': -1.0,
            'mission_label': "待命", 'target_label': "-", 'net_slot': None,
            'barrier_slot': None, 'barrier_center': None,
            'hangar_idx': hangar_idx, 'z_cap': CFG.INTERCEPTOR_MAX_ALT,
            'return_fast': False,
            'task_reserved': False,
            'jammed_by_interference': False, 'jam_zone': None,
            'jam_since': -1.0,
            'jam_loss_logged': False,
            'local_avoid_mode': "", 'local_hold_reason': ""}  # 记录所属机槽


def create_enemy(eid, x, speed, heading, spawn_time, rng_val=0.0):
    r = rng_val
    etype = EType.NORMAL
    if r < 0.1:
        etype = EType.LOITER; speed *= 0.8
    elif r < 0.3:
        etype = EType.DECOY; speed *= CFG.DECOY_SPEED_MULT
    elif r < 0.5:
        etype = EType.DASH; speed *= 1.3
    elif r < 0.8:
        etype = EType.SNAKE if r < 0.65 else EType.JINK

    altitude_map = {
        EType.NORMAL: 24.0,
        EType.SNAKE: 30.0,
        EType.JINK: 34.0,
        EType.DASH: 28.0,
        EType.LOITER: 38.0,
        EType.DECOY: 20.0,
    }
    enemy_z = min(CFG.ENEMY_MAX_ALT, altitude_map.get(etype, 24.0) + (eid % 2) * 2.0)

    return {'id': eid, 'x': x, 'y': CFG.ENEMY_SPAWN_LINE + 50, 'z': enemy_z, 'vz': 0.0,
            'z_cap': CFG.ENEMY_MAX_ALT, 'target_z': enemy_z, 'climb_rate': CFG.ENEMY_CLIMB_RATE,
            'heading': heading, 'speed': speed,
            'state': EState.APPROACHING, 'spawn_time': spawn_time,
            'maneuver_timer': 0.0, 'detected': False, 'detect_time': -1.0,
            'type': etype, 'phase': r * 100, 'target_heading': heading,
            'loiter_center': None, 'loiter_timer': CFG.LOITER_DURATION, 'is_diving': False,
            'external_id': f"sim-{eid+1:02d}", 'track_quality': 1.0, 'stale': False,
            'lost': False, 'last_update': spawn_time, 'classification_confidence': 1.0,
            'source': 'demo'}


def move_entity(e, dt):
    r = math.radians(e['heading'])
    e['x'] += math.cos(r) * e['speed'] * dt
    e['y'] += math.sin(r) * e['speed'] * dt
    e['x'] = max(0, min(CFG.AREA_WIDTH, e['x']))
    e['y'] = max(0, min(CFG.AREA_HEIGHT, e['y']))
    if 'z' in e:
        target_z = e.get('target_z')
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


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180


def dist2d(a, b):
    dx, dy = a['x'] - b['x'], a['y'] - b['y']
    return math.sqrt(dx * dx + dy * dy)


def dist3d(a, b):
    dz = a.get('z', 0.0) - b.get('z', 0.0)
    dxy = dist2d(a, b)
    return math.sqrt(dxy * dxy + dz * dz)
