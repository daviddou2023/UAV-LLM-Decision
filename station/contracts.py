"""
Device2 planning contract builder for frames returned to Device1.

This module only translates the current assignment/path state into a transport
neutral PlanFrame. It does not change the simulator, flight control, or task
assignment logic.

    在内部，simulation/main.py 和 decision/cooperation.py 只关心数学模型（追击点、速度、航向角）。
    但是，负责统筹、操控和显示的设备一不能直接消费这些内部状态，
    它需要标准的、结构化的 JSON 任务指令（比如具体的航路点、控制意图、编队 ID）。
    这个文件就是负责把系统内部的数学状态翻译成可回发给设备一的 PlanFrame JSON 协议包

在simulation/main.py的step()执行完毕后，环境里所有无人机和敌机的数学状态（坐标、状态码等）都更新完毕。此时 env 对象被传递给本模块。本模块从中读取 env.interceptors 和 env.enemies
同级的 station/exporter.py 会调用本模块的 build() 方法拿到字典，然后把它传递给 station/socket_client.py
station/socket_client.py 利用底层的 TCP Socket 连接设备一，将字典转换为 tcp_json_lines 发送出去。
"""
import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from core.common import CFG, EState, IRole, IState


ACTIVE_ENEMY_STATES = {EState.APPROACHING, EState.MANEUVERING}
ACTIVE_UAV_STATES = {IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING}


@dataclass
class PlannerContractOptions:
    """
    保存从外部（比如启动脚本）读入的各项规划参数
    """
    producer_id: str = "device2_planner"
    schema_version: str = "planframe.v1"
    valid_duration_ms: int = 1500
    waypoint_count: int = 3
    waypoint_spacing_m: float = 500.0
    primary_control_mode: str = "mission_waypoints"
    supported_control_modes: Tuple[str, ...] = ("mission_waypoints", "goto")
    replace_policy: str = "task_change_immediate_same_task_smooth"
    smooth_replace_threshold_m: float = 30.0
    min_reupload_interval_sec: float = 2.0
    uav_id_map: str = ""
    preflight_takeoff_alt_m: float = 15.0


@dataclass
class _AssignmentIdentity:
    """
    用来记录每个无人机当前任务的“签名”和世代，用来判断任务是否发生了改变
    """
    signature: Tuple[Any, ...]
    assignment_id: str
    assignment_epoch: int


@dataclass
class PlannerContractState:
    """
    维护全过程的任务分配状态表，确保同一任务的epoch不变，换任务时epoch递增
    """
    assignment_by_uav: Dict[str, _AssignmentIdentity] = field(default_factory=dict)

    def reset_assignments(self):
        self.assignment_by_uav.clear()

    def identity_for(self, uav_id: str, signature: Tuple[Any, ...]) -> Tuple[str, int, bool]:
        previous = self.assignment_by_uav.get(uav_id)
        if previous and previous.signature == signature:
            return previous.assignment_id, previous.assignment_epoch, False

        next_epoch = 1 if previous is None else previous.assignment_epoch + 1
        assignment_id = f"asg_{_clean_id(uav_id)}_e{next_epoch:04d}"
        self.assignment_by_uav[uav_id] = _AssignmentIdentity(
            signature=signature,
            assignment_id=assignment_id,
            assignment_epoch=next_epoch,
        )
        return assignment_id, next_epoch, True


class PlanFrameBuilder:
    """把设备2内部环境状态转换为可回发设备一的完整规划帧。

    本类不改变 env，只在当前毫秒读取任务绑定、路径和敌我状态，生成
    assignment_plan、route_plan 和 groups。状态快照也在这里固定下来。
    """
    def __init__(self, state: Optional[PlannerContractState] = None):
        self.state = state or PlannerContractState()

    def reset_assignments(self):
        self.state.reset_assignments()

    def build(self, env, options: PlannerContractOptions, plan_seq: int, timestamp_ms: int) -> Dict[str, Any]:
        """遍历当前 UAV 状态，组装完整 PlanFrame 协议包。

        调用方通常是 PlannerExporter.maybe_publish()。每架可导出的 UAV 都会
        同时生成 assignment（打谁、什么任务）和 route（怎么飞、飞控意图），
        二者通过 assignment_id/uav_id/drone_id 绑定。
        """
        frame_id = f"plan_frame_{int(plan_seq):08d}"
        radar_frame_id = _radar_frame_id(env)
        # 解析真实无人机ID的映射关系
        uav_map = parse_uav_id_map(options.uav_id_map, len(getattr(env, "interceptors", [])))
        active_enemies = [enemy for enemy in getattr(env, "enemies", []) if _enemy_available(enemy)]

        assignments: List[Dict[str, Any]] = []
        routes: List[Dict[str, Any]] = []
        groups: Dict[str, Dict[str, Any]] = {}

        # 遍历所有己方无人机
        for it in getattr(env, "interceptors", []):
            if not _should_export_uav(it):
                continue

            # 获取真实ID和它的攻击目标
            uav_ids = uav_map.get(int(it.get("id", 0)), _default_uav_ids(int(it.get("id", 0))))
            target = _get_target_for_uav(env, it)
            # 判断当前的任务大类
            mission_type = _mission_type_for(it, target, bool(active_enemies))

            if mission_type == "idle":
                continue

            target_public_id = _target_public_id(target) if target else None
            role = _role_name(it.get("role"))
            formation_role = _formation_role(it)
            group_id = _group_id_for(mission_type, target_public_id, uav_ids["uav_id"])
            # 生成任务签名：如果签名变了，说明换任务了，identity_for 会自动增加 epoch
            signature = (mission_type, target_public_id, role, formation_role, group_id)
            assignment_id, assignment_epoch, changed = self.state.identity_for(uav_ids["uav_id"], signature)
            lock_state = _target_lock_state(mission_type, target_public_id, changed)

            # 组装任务分配包（告诉飞控你要打谁，你是什么角色）
            assignment = self._build_assignment(
                env=env,
                it=it,
                target=target,
                uav_ids=uav_ids,
                assignment_id=assignment_id,
                assignment_epoch=assignment_epoch,
                group_id=group_id,
                mission_type=mission_type,
                role=role,
                formation_role=formation_role,
                lock_state=lock_state,
                plan_seq=plan_seq,
                timestamp_ms=timestamp_ms,
            )
            # 组装路径规划包：告诉飞控你要怎么飞，具体去哪几个坐标
            route = self._build_route(
                env=env,
                it=it,
                target=target,
                uav_ids=uav_ids,
                assignment_id=assignment_id,
                assignment_epoch=assignment_epoch,
                group_id=group_id,
                mission_type=mission_type,
                role=role,
                formation_role=formation_role,
                lock_state=lock_state,
                plan_seq=plan_seq,
                frame_id=frame_id,
                timestamp_ms=timestamp_ms,
                options=options,
            )
            assignments.append(assignment)
            routes.append(route)

            # 组装编队信息 (如果有几架飞机打同一个目标，把它们编进一个 Group 里)
            group = groups.setdefault(
                group_id,
                {
                    "group_id": group_id,
                    "group_type": _group_type(mission_type, formation_role),
                    "target_id": target_public_id,
                    "members": [],
                    "sync": {
                        "terminal_time_constraint": "shared_when_present",
                        "formation_center": None,
                    },
                },
            )
            group["members"].append(
                {
                    "uav_id": uav_ids["uav_id"],
                    "drone_id": uav_ids["drone_id"],
                    "assignment_id": assignment_id,
                    "role": role,
                    "formation_role": formation_role,
                }
            )
            if target:
                group["sync"]["formation_center"] = _position_snapshot(env, target)

        # 返回完整决策帧；主链路可直接发送，也可裁剪成 decision_output_frame。
        return {
            "msg_type": "plan_frame",
            "schema_version": options.schema_version,
            "producer": {
                "device": "device2",
                "module": options.producer_id,
            },
            "frame_id": frame_id,
            "plan_seq": int(plan_seq),
            "radar_frame_id": radar_frame_id,
            "timestamp_ms": int(timestamp_ms),
            "valid_duration_ms": int(options.valid_duration_ms),
            "valid_until_ms": int(timestamp_ms + int(options.valid_duration_ms)),
            "situation_alert": _situation_alert(env),
            "coordinate_frame": {
                "local": "meters_xy_alt",
                "geo": "wgs84_lat_lon_alt",
                "origin_lat": float(getattr(env, "geo_origin_lat", 0.0)),
                "origin_lon": float(getattr(env, "geo_origin_lon", 0.0)),
            },
            "transport": {
                "type": "tcp_json_lines",
                "framing": "one_json_object_per_line",
                "delivery_rule": "latest_valid_frame_wins",
            },
            "execution_policy": {
                "send_policy": "continuous_latest_wins",
                "replace_policy": options.replace_policy,
                "same_assignment": "overwrite_route_smoothly",
                "task_changed": "switch_immediately",
                "smooth_replace_threshold_m": float(options.smooth_replace_threshold_m),
                "min_reupload_interval_sec": float(options.min_reupload_interval_sec),
            },
            # 各种元数据，控制策略和版本号等
            "assignment_plan": {
                "status": "planned",
                "assignments": assignments,
            },
            "route_plan": {
                "status": _overall_route_status(routes),
                "routes": routes,
            },
            "groups": list(groups.values()),
        }

    def _build_assignment(
        self,
        env,
        it: Dict[str, Any],
        target: Optional[Dict[str, Any]],
        uav_ids: Dict[str, Any],
        assignment_id: str,
        assignment_epoch: int,
        group_id: str,
        mission_type: str,
        role: str,
        formation_role: Optional[str],
        lock_state: str,
        plan_seq: int,
        timestamp_ms: int,
    ) -> Dict[str, Any]:
        """
        提取单架无人机的任务分配信息（指派打谁，什么角色）
        :param env:
        :param it:
        :param target:
        :param uav_ids:
        :param assignment_id:
        :param assignment_epoch:
        :param group_id:
        :param mission_type:
        :param role:
        :param formation_role:
        :param lock_state:
        :param plan_seq:
        :param timestamp_ms:
        :return:
        """
        return {
            "assignment_id": assignment_id,
            "assignment_epoch": int(assignment_epoch),
            "group_id": group_id,
            "uav_id": uav_ids["uav_id"],
            "drone_id": uav_ids["drone_id"],
            "uav_internal_id": int(it.get("id", 0)),
            "target_id": _target_public_id(target) if target else None,
            "target_internal_id": int(target["id"]) if target else None,
            "mission_type": mission_type,
            "role": role,
            "formation_role": formation_role,
            "state": _assignment_state(it, mission_type),
            "uav_state": _enum_name(it.get("state")),
            "target_lock_state": lock_state,
            "assigned_at_plan_seq": int(plan_seq),
            "updated_at_ms": int(timestamp_ms),
            "target_snapshot": _target_snapshot(env, target) if target else None,
        }

    def _build_route(
        self,
        env,
        it: Dict[str, Any],
        target: Optional[Dict[str, Any]],
        uav_ids: Dict[str, Any],
        assignment_id: str,
        assignment_epoch: int,
        group_id: str,
        mission_type: str,
        role: str,
        formation_role: Optional[str],
        lock_state: str,
        plan_seq: int,
        frame_id: str,
        timestamp_ms: int,
        options: PlannerContractOptions,
    ) -> Dict[str, Any]:
        """提取单架无人机的具体飞行路线（航点，速度，执行意图）"""
        # route 只采样设备2已经算好的 path_plan/返航点，不在这里重新规划。
        raw_points = _route_points_for(env, it, target, mission_type)
        sampled_points = _sample_short_path(
            raw_points,
            int(options.waypoint_count),
            float(options.waypoint_spacing_m),
        )
        speed_mps = _route_speed(it, target, mission_type)
        waypoints = _waypoints(env, sampled_points, speed_mps)
        status, failure_reason = _route_status(mission_type, target, waypoints)
        if mission_type == "uav_unavailable" and it.get("device3_failure_reason"):
            failure_reason = str(it.get("device3_failure_reason"))

        return {
            "route_id": f"route_{assignment_id}_{int(plan_seq):08d}",
            "assignment_id": assignment_id,
            "assignment_epoch": int(assignment_epoch),
            "group_id": group_id,
            "formation_id": group_id if formation_role else None,
            "uav_id": uav_ids["uav_id"],
            "drone_id": uav_ids["drone_id"],
            "target_id": _target_public_id(target) if target else None,
            "frame_id": frame_id,
            "plan_seq": int(plan_seq),
            "timestamp_ms": int(timestamp_ms),
            "valid_duration_ms": int(options.valid_duration_ms),
            "replan_policy": "receding_horizon",
            "send_policy": "continuous_latest_wins",
            "overwrite_policy": "always_overwrite_previous",
            "target_lock_state": lock_state,
            "status": status,
            "failure_reason": failure_reason,
            "control_intent": _control_intent(options, mission_type, bool(waypoints)),
            "route_upload": {
                "type": "short_waypoint_task",
                "waypoint_count": int(options.waypoint_count),
                "waypoint_spacing_m": float(options.waypoint_spacing_m),
                "same_assignment_replace": "smooth_overwrite",
                "task_change_replace": "immediate_overwrite",
                "future_goto_reserved": "schema_ready",
            },
            "uav_snapshot": _uav_snapshot(env, it),
            "path_reason": it.get("path_reason", ""),
            "speed_mps": speed_mps,
            "waypoints": waypoints,
            "next_command_point": waypoints[0] if waypoints else None,
        }


def parse_uav_id_map(text: str, interceptor_count: int = 0) -> Dict[int, Dict[str, Any]]:
    """
    解析启动脚本里的ID映射表，将仿真系统内部的0，1,2映射为真实飞机的uav_01,uav_02

    解析 core.common.PLAN_EXPORT["uav_id_map"] 中的 "uav_01:1,uav_02:2" 映射文本。
    将仿真系统内部自增的 internal_id (0,1,2) 绑定到真实的飞控 ID (1,2,3)。
    如果不做这一步，你派出的 0号机，真实飞控会因为不认识 0 号机而拒绝执行。
    :param text:
    :param interceptor_count:
    :return:
    """
    mapping: Dict[int, Dict[str, Any]] = {}
    for piece in str(text or "").split(","):
        item = piece.strip()
        if not item or ":" not in item:
            continue
        left, right = [part.strip() for part in item.split(":", 1)]
        if not left or not right:
            continue
        try:
            # 处理 "uav_01:1" 的格式
            if left.lower().startswith("uav_"):
                drone_id = int(right)
                internal_id = max(0, drone_id - 1)
                mapping[internal_id] = {"uav_id": left, "drone_id": drone_id}
            # 处理反向格式 "1:uav_01"
            elif right.lower().startswith("uav_"):
                internal_id = int(left)
                mapping[internal_id] = {"uav_id": right, "drone_id": internal_id + 1}
            # 处理纯数字格式 "0:1"
            else:
                internal_id = int(left)
                drone_id = int(right)
                mapping[internal_id] = {"uav_id": f"uav_{drone_id:02d}", "drone_id": drone_id}
        except (TypeError, ValueError):
            continue

    # 对没有提供映射的无人机进行默认自动分配
    for internal_id in range(max(0, int(interceptor_count))):
        mapping.setdefault(internal_id, _default_uav_ids(internal_id))
    return mapping


def _default_uav_ids(internal_id: int) -> Dict[str, Any]:
    drone_id = int(internal_id) + 1
    return {"uav_id": f"uav_{drone_id:02d}", "drone_id": drone_id}


def _clean_id(value: Any) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", str(value or "none")).strip("_") or "none"


def _enum_name(value: Any) -> str:
    if hasattr(value, "name"):
        return str(value.name).lower()
    return str(value or "unknown").lower()


def _enemy_available(enemy: Dict[str, Any]) -> bool:
    if not enemy:
        return False
    if enemy.get("state") not in ACTIVE_ENEMY_STATES:
        return False
    return not bool(enemy.get("lost", False))


def _should_export_uav(it: Dict[str, Any]) -> bool:
    state = it.get("state")
    if state in (IState.DESTROYED, IState.LANDED):
        return False
    if state in ACTIVE_UAV_STATES:
        return True
    if it.get("target_id") is not None:
        return True
    return False


def _get_target_for_uav(env, it: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    target_id = it.get("target_id")
    if target_id is None:
        return None
    getter = getattr(env, "_get_enemy", None)
    target = getter(target_id) if getter else None
    if not target or target.get("state") not in ACTIVE_ENEMY_STATES:
        return None
    return target


def _mission_type_for(it: Dict[str, Any], target: Optional[Dict[str, Any]], has_active_enemies: bool) -> str:
    state = it.get("state")
    if it.get("device3_temporarily_unavailable"):
        return "uav_unavailable"
    if target:
        if it.get("net_slot") is not None:
            return "net_capture"
        if it.get("barrier_slot") is not None:
            return "barrier_net"
        return "intercept"
    if state == IState.RETURNING or (state in ACTIVE_UAV_STATES and not has_active_enemies):
        return "return_home"
    if state in ACTIVE_UAV_STATES and has_active_enemies:
        return "retask_pending"
    return "idle"


def _target_public_id(target: Optional[Dict[str, Any]]) -> Optional[str]:
    if not target:
        return None
    external_id = target.get("external_id")
    if external_id not in (None, ""):
        return str(external_id)
    return f"target_{int(target.get('id', 0)) + 1:03d}"


def _role_name(role: Any) -> str:
    if role == IRole.PRIMARY:
        return "primary"
    if role == IRole.FOLLOWER:
        return "backup"
    return "reserve"


def _formation_role(it: Dict[str, Any]) -> Optional[str]:
    if it.get("net_slot") is not None:
        return f"net_slot_{int(it.get('net_slot', 0))}"
    if it.get("barrier_slot") is not None:
        return f"barrier_slot_{int(it.get('barrier_slot', 0))}"
    return None


def _group_id_for(mission_type: str, target_public_id: Optional[str], uav_id: str) -> str:
    if target_public_id:
        return f"group_target_{_clean_id(target_public_id)}"
    return f"group_{_clean_id(uav_id)}_{_clean_id(mission_type)}"


def _target_lock_state(mission_type: str, target_public_id: Optional[str], changed: bool) -> str:
    if mission_type in ("return_home", "uav_unavailable"):
        return "released"
    if mission_type == "retask_pending":
        return "retask_pending"
    if target_public_id:
        return "new_lock" if changed else "locked"
    return "none"


def _assignment_state(it: Dict[str, Any], mission_type: str) -> str:
    state = it.get("state")
    if mission_type == "uav_unavailable":
        return "failed"
    if state == IState.LAUNCHING:
        return "planned"
    if mission_type == "retask_pending":
        return "planned"
    if state in (IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING):
        return "en_route"
    if state == IState.STANDBY:
        return "completed"
    if state == IState.DESTROYED:
        return "failed"
    return "planned"


def _group_type(mission_type: str, formation_role: Optional[str]) -> str:
    if mission_type in ("net_capture", "barrier_net") or formation_role:
        return "cooperative_formation"
    if mission_type == "return_home":
        return "single_return"
    return "single_or_primary_backup"


def _route_points_for(env, it: Dict[str, Any], target: Optional[Dict[str, Any]], mission_type: str) -> List[Tuple[float, float, float]]:
    """
    从内部对象提取出完整的原始规划路径（XYZ路径）
    :param env:
    :param it:
    :param target:
    :param mission_type:
    :return:
    """
    current = (_float(it.get("x")), _float(it.get("y")), _float(it.get("z")))
    path = [_as_point(point) for point in (it.get("path_plan") or [])]
    path = [point for point in path if point is not None]
    if path:
        if _distance3(current, path[0]) > 2.0:
            path.insert(0, current)
        return _dedupe_points(path)

    if mission_type == "return_home":
        return [current, _home_point(it)]
    if mission_type == "uav_unavailable":
        return [current]
    if target:
        return [current, (_float(target.get("x")), _float(target.get("y")), _float(target.get("z")))]
    return [current]


def _home_point(it: Dict[str, Any]) -> Tuple[float, float, float]:
    hangars = CFG.HANGAR_POSITIONS or (CFG.AREA_WIDTH * 0.5,)
    try:
        hangar_idx = int(it.get("hangar_idx", 0) or 0)
    except (TypeError, ValueError):
        hangar_idx = 0
    hangar_idx = max(0, min(len(hangars) - 1, hangar_idx))
    return (float(hangars[hangar_idx]), float(CFG.INTERCEPT_FAIL_LINE + 200.0), 0.0)


def _as_point(value: Any) -> Optional[Tuple[float, float, float]]:
    if isinstance(value, dict):
        return (_float(value.get("x")), _float(value.get("y")), _float(value.get("z")))
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        z = value[2] if len(value) >= 3 else 0.0
        return (_float(value[0]), _float(value[1]), _float(z))
    return None


def _dedupe_points(points: Iterable[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    result: List[Tuple[float, float, float]] = []
    for point in points:
        if not result or _distance3(result[-1], point) > 0.5:
            result.append(point)
    return result


def _sample_short_path(
    points: Sequence[Tuple[float, float, float]],
    count: int,
    spacing_m: float,
) -> List[Tuple[Tuple[float, float, float], float]]:
    """
    下采样函数：设备一飞控执行侧不需要几百个点的长曲线，只需要前面几百米内的短航迹点
    沿给定的原始路径(折线)，按照指定的间距(spacing_m)等距采样，
    最多提取 count 个点，供设备一作为引导航点。
    :param points:
    :param count:
    :param spacing_m:
    :return:[(坐标点, 累计距离), ...]
    """
    if count <= 0:
        return []
    # 过滤掉距离过近的重叠点
    cleaned = _dedupe_points(points)
    if len(cleaned) < 2:
        return []
    spacing_m = max(1.0, float(spacing_m))
    # 计算原始折线每一段的长度
    segment_lengths = [_distance3(cleaned[idx], cleaned[idx + 1]) for idx in range(len(cleaned) - 1)]
    total_length = sum(segment_lengths)
    if total_length <= 1e-6:
        return []

    wanted = spacing_m
    sampled: List[Tuple[Tuple[float, float, float], float]] = []
    # 沿着折线进行插值，每隔spacing_m 提取一个新坐标点
    while len(sampled) < count and wanted < total_length:
        sampled.append((_point_at_distance(cleaned, segment_lengths, wanted), wanted))
        wanted += spacing_m

    # 兜底逻辑：如果路径总长不够切出要求的个数，强制把终点作为最后一个航点加进去
    end_point = cleaned[-1]
    if len(sampled) < count and (not sampled or _distance3(sampled[-1][0], end_point) > 0.5):
        sampled.append((end_point, total_length))
    return sampled[:count]


def _point_at_distance(
    points: Sequence[Tuple[float, float, float]],
    segment_lengths: Sequence[float],
    distance_m: float,
) -> Tuple[float, float, float]:
    """沿着折线计算特定距离处的具体坐标"""
    remaining = float(distance_m)
    for idx, seg_len in enumerate(segment_lengths):
        if seg_len <= 1e-6:
            continue
        if remaining <= seg_len:
            ratio = remaining / seg_len
            a = points[idx]
            b = points[idx + 1]
            return (
                a[0] + (b[0] - a[0]) * ratio,
                a[1] + (b[1] - a[1]) * ratio,
                a[2] + (b[2] - a[2]) * ratio,
            )
        remaining -= seg_len
    return points[-1]


def _route_speed(it: Dict[str, Any], target: Optional[Dict[str, Any]], mission_type: str) -> float:
    current_speed = _float(it.get("speed"), 0.0)
    if mission_type == "return_home":
        return round(max(CFG.INTERCEPTOR_SPEED, current_speed), 2)
    if target:
        enemy_speed = _float(target.get("speed"), CFG.ENEMY_SPEED)
        return round(min(CFG.INTERCEPTOR_BOOST_SPEED, max(CFG.INTERCEPTOR_SPEED, current_speed, enemy_speed + 2.0)), 2)
    return round(max(CFG.INTERCEPTOR_SPEED, current_speed), 2)


def _waypoints(
    env,
    points: Sequence[Tuple[Tuple[float, float, float], float]],
    speed_mps: float,
) -> List[Dict[str, Any]]:
    """将XYZ局部坐标转换为带有经纬度，到达时间的标准航路点字典"""
    waypoints: List[Dict[str, Any]] = []
    for idx, sample in enumerate(points):
        point, cumulative = sample
        lat, lon = _xy_to_lonlat(env, point[0], point[1])
        eta = cumulative / max(float(speed_mps), 0.1)
        waypoints.append(
            {
                "seq": idx,
                "x": round(point[0], 3),
                "y": round(point[1], 3),
                "z": round(point[2], 3),
                "lat": round(lat, 8) if lat is not None else None,
                "lon": round(lon, 8) if lon is not None else None,
                "lng": round(lon, 8) if lon is not None else None,
                "alt": round(point[2], 3),
                "speed_mps": round(float(speed_mps), 2),
                "arrival_time_sec": round(eta, 3),
            }
        )
    return waypoints


def _route_status(mission_type: str, target: Optional[Dict[str, Any]], waypoints: Sequence[Dict[str, Any]]) -> Tuple[str, str]:
    if mission_type == "uav_unavailable":
        return "failed", "execution_side_reported_unavailable"
    if mission_type == "retask_pending":
        return "pending", "waiting_for_next_assignment"
    if not waypoints:
        return "infeasible", "no_future_waypoint"
    if mission_type in ("intercept", "net_capture", "barrier_net") and not target:
        return "infeasible", "target_unavailable"
    return "planned", ""


def _overall_route_status(routes: Sequence[Dict[str, Any]]) -> str:
    if not routes:
        return "empty"
    if any(route.get("status") == "failed" for route in routes):
        return "partial_failed"
    if any(route.get("status") == "infeasible" for route in routes):
        return "partial_infeasible"
    if any(route.get("status") == "pending" for route in routes):
        return "partial_pending"
    return "planned"


def _situation_alert(env) -> str:
    enemies = list(getattr(env, "enemies", []) or [])
    interceptors = list(getattr(env, "interceptors", []) or [])
    active_enemies = [enemy for enemy in enemies if _enemy_available(enemy)]
    available_uavs = [
        it for it in interceptors
        if _enum_name(it.get("state")) not in ("destroyed", "landed")
        and not it.get("device3_temporarily_unavailable")
    ]
    bound_count = sum(1 for it in interceptors if it.get("target_id") is not None)
    return f"当前发现{len(active_enemies)}个活动目标，{len(available_uavs)}架我方无人机可用，{bound_count}个任务绑定。"


def _control_intent(options: PlannerContractOptions, mission_type: str, has_waypoints: bool) -> Dict[str, Any]:
    """
    根据当前任务类型，生成给飞控的底层动作建议
    :param options:
    :param mission_type:
    :param has_waypoints:
    :return:
    """
    if mission_type == "uav_unavailable":
        intent = "hold_for_recovery"
        low_level = [{"command": "hold", "policy": "do_not_upload_new_route"}]
        fallback = "operator_or_execution_side_recovery"
    elif mission_type == "return_home":
        intent = "return_home"
        low_level = [
            {"command": "arm", "policy": "if_needed"},
            {"command": "takeoff", "policy": "if_not_airborne", "alt_m": float(options.preflight_takeoff_alt_m)},
            {"command": "set_mode", "mode": "AUTO", "policy": "if_needed"},
            {"command": "upload_short_waypoints", "policy": "overwrite_previous"},
        ]
        fallback = "rtl"
    elif mission_type == "retask_pending":
        intent = "hold_for_retask"
        low_level = [{"command": "hold", "policy": "short_valid_duration_only"}]
        fallback = "keep_last_valid_route"
    else:
        intent = "execute_intercept_route"
        low_level = [
            {"command": "arm", "policy": "if_needed"},
            {"command": "takeoff", "policy": "if_not_airborne", "alt_m": float(options.preflight_takeoff_alt_m)},
            {"command": "set_mode", "mode": "AUTO", "policy": "if_needed"},
            {"command": "upload_short_waypoints", "policy": "overwrite_previous"},
        ]
        fallback = "hold_or_replan" if not has_waypoints else "none"

    return {
        "intent": intent,
        "preferred_mode": str(options.primary_control_mode),
        "supported_modes": list(options.supported_control_modes),
        "low_level_actions": low_level,
        "idempotent": True,
        "fallback_action": fallback,
    }


def _target_snapshot(env, target: Dict[str, Any]) -> Dict[str, Any]:
    """将内部复杂的实体对象，精简成只包含外部协议需要的核心物理量字典。"""
    snapshot = {
        "target_id": _target_public_id(target),
        "target_internal_id": int(target.get("id", 0)),
        "target_type": _enum_name(target.get("type")),
        "state": _enum_name(target.get("state")),
        "confidence": round(_float(target.get("classification_confidence"), 1.0), 3),
        "track_quality": round(_float(target.get("track_quality"), 1.0), 3),
        "speed_mps": round(_float(target.get("speed")), 3),
        "heading_deg": round(_float(target.get("heading")), 3),
        "frame": target.get("frame"),
        "source": target.get("source", ""),
        "lost": bool(target.get("lost", False)),
        "stale": bool(target.get("stale", False)),
        "position": _position_snapshot(env, target),
    }
    return snapshot


def _uav_snapshot(env, it: Dict[str, Any]) -> Dict[str, Any]:
    endurance = max(float(CFG.INTERCEPTOR_ENDURANCE), 1.0)
    battery = max(0.0, min(1.0, _float(it.get("fuel"), endurance) / endurance))
    return {
        "uav_internal_id": int(it.get("id", 0)),
        "state": _enum_name(it.get("state")),
        "speed_mps": round(_float(it.get("speed")), 3),
        "heading_deg": round(_float(it.get("heading")), 3),
        "battery": round(battery, 3),
        "fault": it.get("device3_failure_reason", ""),
        "position": _position_snapshot(env, it),
    }


def _position_snapshot(env, entity: Dict[str, Any]) -> Dict[str, Any]:
    x = _float(entity.get("x"))
    y = _float(entity.get("y"))
    z = _float(entity.get("z"))
    lat, lon = _xy_to_lonlat(env, x, y)
    return {
        "x": round(x, 3),
        "y": round(y, 3),
        "z": round(z, 3),
        "lat": round(lat, 8) if lat is not None else None,
        "lon": round(lon, 8) if lon is not None else None,
        "lng": round(lon, 8) if lon is not None else None,
        "alt": round(z, 3),
    }


def _xy_to_lonlat(env, x: float, y: float) -> Tuple[Optional[float], Optional[float]]:
    geo = getattr(env, "geo_reference", None)
    if not geo or not hasattr(geo, "xy_to_lonlat"):
        return None, None
    try:
        lat, lon = geo.xy_to_lonlat(x, y)
        return float(lat), float(lon)
    except Exception:
        return None, None


def _radar_frame_id(env) -> Optional[int]:
    meta = getattr(env, "last_live_packet_meta", {}) or {}
    frame = meta.get("frame")
    if frame is not None:
        try:
            return int(frame)
        except (TypeError, ValueError):
            pass
    candidates = []
    for enemy in getattr(env, "enemies", []):
        if enemy.get("frame") is not None:
            candidates.append(enemy.get("frame"))
    for it in getattr(env, "interceptors", []):
        if it.get("reported_frame") is not None:
            candidates.append(it.get("reported_frame"))
    try:
        return int(max(candidates)) if candidates else None
    except (TypeError, ValueError):
        return None


def _distance3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)
