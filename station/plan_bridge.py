"""
地面站（station）侧规划接口转接。

本模块只消费已经生成好的 PlanFrame，不参与任务分配和路径规划。
默认关闭；需要转发时在 core/common.py 的 STATION_BRIDGE 中开启。
"""
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from station.socket_client import PlannerSocketClient


@dataclass
class StationBridgeConfig:
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 7101
    reconnect_sec: float = 1.0
    connect_timeout_sec: float = 0.2
    strict_binding: bool = True

    @classmethod
    def from_config(cls, settings=None):
        """从 core.common 读取地面站旁路转接配置。"""
        if settings is None:
            from core.common import STATION_BRIDGE as settings
        return cls(
            enabled=_cfg_bool(settings, "enabled", False),
            host=_cfg_str(settings, "host", "127.0.0.1"),
            port=_cfg_int(settings, "port", 7101),
            reconnect_sec=max(0.1, _cfg_float(settings, "reconnect_sec", 1.0)),
            connect_timeout_sec=max(0.05, _cfg_float(settings, "connect_timeout_sec", 0.2)),
            strict_binding=_cfg_bool(settings, "strict_binding", True),
        )


class StationPlanBridge:
    def __init__(self, config: StationBridgeConfig):
        self.config = config
        self.socket: Optional[PlannerSocketClient] = None
        self.last_send_ok = False
        self.last_error_log_at = 0.0
        if config.enabled:
            self.socket = PlannerSocketClient(
                host=config.host,
                port=config.port,
                reconnect_sec=config.reconnect_sec,
                connect_timeout_sec=config.connect_timeout_sec,
            )

    @classmethod
    def from_config(cls, settings=None):
        return cls(StationBridgeConfig.from_config(settings))

    def close(self):
        if self.socket:
            self.socket.close()

    def forward(self, frame: Dict[str, Any], now: Optional[float] = None) -> List[Tuple[str, str, str]]:
        """将 PlanFrame 转成地面站（station）侧转接包并发送，返回 UI 日志。"""
        if not self.config.enabled:
            return []
        now = time.time() if now is None else float(now)
        packet = build_station_transfer_packet(frame)

        if self.config.strict_binding and not packet["binding_ok"]:
            return [("[STATION]", f"转接包绑定校验失败，已阻止发送: {packet['binding_errors']}", "red")]

        if not self.socket:
            return []
        sent = self.socket.send_json(packet)
        if sent:
            logs: List[Tuple[str, str, str]] = []
            if not self.last_send_ok:
                logs.append(("[STATION]", f"地面站（station）转接Socket已连接 -> {self.config.host}:{self.config.port}", "green"))
            self.last_send_ok = True
            return logs

        self.last_send_ok = False
        if now - self.last_error_log_at >= 5.0:
            reason = self.socket.last_error if self.socket else "socket_unavailable"
            self.last_error_log_at = now
            return [("[STATION]", f"地面站（station）转接暂未发送成功: {reason}", "amber")]
        return []


def build_station_transfer_packet(frame: Dict[str, Any]) -> Dict[str, Any]:
    """
    生成地面站（station）侧转接包。

    绑定规则：assignment 和 route 必须使用同一个 assignment_id、uav_id、drone_id。
    这样每个规划指令包都会固定落到同一架无人机上。
    """
    assignments = frame.get("assignment_plan", {}).get("assignments", [])
    routes = frame.get("route_plan", {}).get("routes", [])
    # 先按 assignment_id 建 route 索引，再逐条校验 assignment/route 是否绑定同一架 UAV。
    binding_errors: List[Dict[str, Any]] = []
    route_by_assignment: Dict[Any, Dict[str, Any]] = {}
    for route in routes:
        assignment_id = route.get("assignment_id")
        if assignment_id in route_by_assignment:
            binding_errors.append(_binding_error("duplicate_route_assignment_id", {"assignment_id": assignment_id}, route))
            continue
        route_by_assignment[assignment_id] = route

    seen_assignments = set()
    uav_plans: List[Dict[str, Any]] = []

    for assignment in assignments:
        assignment_id = assignment.get("assignment_id")
        if not assignment_id or assignment_id in seen_assignments:
            binding_errors.append(_binding_error("duplicate_or_empty_assignment_id", assignment, None))
            continue
        seen_assignments.add(assignment_id)
        route = route_by_assignment.get(assignment.get("assignment_id"))
        if not route:
            binding_errors.append(_binding_error("route_missing", assignment, None))
            continue
        if not _same_uav_binding(assignment, route):
            binding_errors.append(_binding_error("uav_binding_mismatch", assignment, route))
            continue
        uav_plans.append(_merge_uav_plan(assignment, route))

    return {
        "msg_type": "station_plan_transfer",
        "schema_version": "station.plan_bridge.v1",
        "source_msg_type": frame.get("msg_type"),
        "source_schema_version": frame.get("schema_version"),
        "frame_id": frame.get("frame_id"),
        "plan_seq": frame.get("plan_seq"),
        "timestamp_ms": frame.get("timestamp_ms"),
        "valid_duration_ms": frame.get("valid_duration_ms"),
        "valid_until_ms": frame.get("valid_until_ms"),
        "coordinate_frame": frame.get("coordinate_frame"),
        "binding_rule": "assignment_id_uav_id_drone_id_must_match",
        "binding_ok": not binding_errors,
        "binding_errors": binding_errors,
        "groups": frame.get("groups", []),
        "uav_plans": uav_plans,
    }


def _merge_uav_plan(assignment: Dict[str, Any], route: Dict[str, Any]) -> Dict[str, Any]:
    """把任务分配和路径规划合并成地面站可直接消费的单机计划。"""
    return {
        "uav_id": assignment.get("uav_id"),
        "drone_id": assignment.get("drone_id"),
        "uav_internal_id": assignment.get("uav_internal_id"),
        "assignment_id": assignment.get("assignment_id"),
        "assignment_epoch": assignment.get("assignment_epoch"),
        "group_id": assignment.get("group_id"),
        "mission_type": assignment.get("mission_type"),
        "role": assignment.get("role"),
        "formation_role": assignment.get("formation_role"),
        "state": assignment.get("state"),
        "target_id": assignment.get("target_id"),
        "target_internal_id": assignment.get("target_internal_id"),
        "target_snapshot": assignment.get("target_snapshot"),
        "route_id": route.get("route_id"),
        "route_status": route.get("status"),
        "failure_reason": route.get("failure_reason"),
        "control_intent": route.get("control_intent"),
        "path_reason": route.get("path_reason"),
        "speed_mps": route.get("speed_mps"),
        "next_command_point": route.get("next_command_point"),
        "waypoints": route.get("waypoints", []),
        "uav_snapshot": route.get("uav_snapshot"),
    }


def _same_uav_binding(assignment: Dict[str, Any], route: Dict[str, Any]) -> bool:
    if assignment.get("assignment_id") in (None, ""):
        return False
    if assignment.get("uav_id") in (None, "") or route.get("uav_id") in (None, ""):
        return False
    if assignment.get("drone_id") in (None, "") or route.get("drone_id") in (None, ""):
        return False
    return (
        assignment.get("assignment_id") == route.get("assignment_id")
        and assignment.get("uav_id") == route.get("uav_id")
        and assignment.get("drone_id") == route.get("drone_id")
    )


def _binding_error(reason: str, assignment: Dict[str, Any], route: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "reason": reason,
        "assignment_id": assignment.get("assignment_id"),
        "assignment_uav_id": assignment.get("uav_id"),
        "assignment_drone_id": assignment.get("drone_id"),
        "route_uav_id": route.get("uav_id") if route else None,
        "route_drone_id": route.get("drone_id") if route else None,
    }


def _cfg_value(settings, key: str, default):
    if not isinstance(settings, dict):
        return default
    return settings.get(key, default)


def _cfg_str(settings, key: str, default: str) -> str:
    return str(_cfg_value(settings, key, default))


def _cfg_int(settings, key: str, default: int) -> int:
    try:
        return int(float(_cfg_value(settings, key, default)))
    except (TypeError, ValueError):
        return int(default)


def _cfg_float(settings, key: str, default: float) -> float:
    try:
        return float(_cfg_value(settings, key, default))
    except (TypeError, ValueError):
        return float(default)


def _cfg_bool(settings, key: str, default: bool) -> bool:
    value = _cfg_value(settings, key, default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "on", "true", "yes", "y", "enable", "enabled"):
        return True
    if text in ("0", "off", "false", "no", "n", "disable", "disabled"):
        return False
    return bool(default)


def _self_check():
    frame = {
        "msg_type": "plan_frame",
        "schema_version": "planframe.v1",
        "frame_id": "f1",
        "plan_seq": 1,
        "assignment_plan": {
            "assignments": [
                {
                    "assignment_id": "asg_uav_01_e0001",
                    "uav_id": "uav_01",
                    "drone_id": 1,
                    "uav_internal_id": 0,
                    "mission_type": "intercept",
                }
            ]
        },
        "route_plan": {
            "routes": [
                {
                    "assignment_id": "asg_uav_01_e0001",
                    "uav_id": "uav_01",
                    "drone_id": 1,
                    "route_id": "r1",
                    "next_command_point": {"x": 1, "y": 2, "z": 3},
                    "waypoints": [],
                }
            ]
        },
    }
    packet = build_station_transfer_packet(frame)
    assert packet["binding_ok"]
    assert packet["uav_plans"][0]["drone_id"] == 1


if __name__ == "__main__":
    _self_check()
    print("station.plan_bridge自检通过")
