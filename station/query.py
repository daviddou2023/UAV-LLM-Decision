"""PlanFrame 查询服务。

面向后续业务接口的最小查询层：只裁剪已有 PlanFrame，不重新定义协议。
"""
import time
from typing import Any, Dict, List, Optional

from station.contracts import PlanFrameBuilder, PlannerContractOptions
from station.interfaces import PlanFrameBuilderPort


class PlannerQueryService:
    """从 PlanFrame 中查询任务分配和下一指令点。"""

    def __init__(
        self,
        builder: Optional[PlanFrameBuilderPort] = None,
        options: Optional[PlannerContractOptions] = None,
    ):
        self.builder = builder or PlanFrameBuilder()
        self.options = options or PlannerContractOptions()
        self.plan_seq = 0

    def build_frame(self, env: Any, options: Optional[PlannerContractOptions] = None) -> Dict[str, Any]:
        """基于当前 env 构造一次查询快照。"""
        self.plan_seq += 1
        timestamp_ms = int(time.time() * 1000)
        return self.builder.build(env, options or self.options, self.plan_seq, timestamp_ms)

    def query_assignments(self, env: Any, options: Optional[PlannerContractOptions] = None) -> Dict[str, Any]:
        """返回当前协同分配摘要。"""
        return assignments_from_frame(self.build_frame(env, options))

    def query_next_command_point(
        self,
        env: Any,
        uav_id: Optional[str] = None,
        drone_id: Optional[Any] = None,
        internal_id: Optional[Any] = None,
        options: Optional[PlannerContractOptions] = None,
    ) -> Dict[str, Any]:
        """返回某架无人机的下一指令点。"""
        frame = self.build_frame(env, options)
        return next_command_point_from_frame(frame, uav_id=uav_id, drone_id=drone_id, internal_id=internal_id)

    def query_station_state_snapshot(self, env: Any, options: Optional[PlannerContractOptions] = None) -> Dict[str, Any]:
        """返回设备一侧敌我状态快照视图。"""
        return station_state_snapshot_from_frame(self.build_frame(env, options))

    def query_station_flight_commands(self, env: Any, options: Optional[PlannerContractOptions] = None) -> Dict[str, Any]:
        """返回设备一侧飞控指令过滤视图。"""
        return station_flight_commands_from_frame(self.build_frame(env, options))

    def query_decision_output_frame(self, env: Any, options: Optional[PlannerContractOptions] = None) -> Dict[str, Any]:
        """返回设备一直接消费的精简决策帧。"""
        return decision_output_frame_from_plan_frame(self.build_frame(env, options))


def assignments_from_frame(frame: Dict[str, Any]) -> Dict[str, Any]:
    """从 PlanFrame 裁剪任务分配结果，供外部接口直接返回。"""
    assignments = frame.get("assignment_plan", {}).get("assignments", []) or []
    return {
        "ok": True,
        "frame_id": frame.get("frame_id"),
        "plan_seq": frame.get("plan_seq"),
        "assignments": [
            {
                "assignment_id": item.get("assignment_id"),
                "assignment_epoch": item.get("assignment_epoch"),
                "uav_id": item.get("uav_id"),
                "drone_id": item.get("drone_id"),
                "uav_internal_id": item.get("uav_internal_id"),
                "target_id": item.get("target_id"),
                "target_internal_id": item.get("target_internal_id"),
                "mission_type": item.get("mission_type"),
                "role": item.get("role"),
                "formation_role": item.get("formation_role"),
                "state": item.get("state"),
                "group_id": item.get("group_id"),
            }
            for item in assignments
        ],
    }


def next_command_point_from_frame(
    frame: Dict[str, Any],
    uav_id: Optional[str] = None,
    drone_id: Optional[Any] = None,
    internal_id: Optional[Any] = None,
) -> Dict[str, Any]:
    """从 PlanFrame 查询某架 UAV 的下一指令点。"""
    if uav_id is None and drone_id is None and internal_id is None:
        return {"ok": False, "reason": "missing_uav_identifier"}

    for route in frame.get("route_plan", {}).get("routes", []) or []:
        if not _route_matches(route, uav_id=uav_id, drone_id=drone_id, internal_id=internal_id):
            continue
        point = route.get("next_command_point")
        if not point:
            return _route_response(False, route, "next_command_point_unavailable")
        response = _route_response(True, route, "")
        response["point"] = point
        return response

    return {"ok": False, "reason": "uav_route_not_found", "uav_id": uav_id, "drone_id": drone_id, "internal_id": internal_id}


def station_state_snapshot_from_frame(frame: Dict[str, Any]) -> Dict[str, Any]:
    """从 PlanFrame 裁剪设备一侧敌我状态快照。

    敌方状态来自 assignment.target_snapshot，己方状态来自 route.uav_snapshot。
    这里不补算状态，只做去重和封装，保证显示数据与规划帧同源。
    """
    targets: Dict[str, Dict[str, Any]] = {}
    for assignment in frame.get("assignment_plan", {}).get("assignments", []) or []:
        snapshot = assignment.get("target_snapshot")
        if not snapshot:
            continue
        key = _stable_key(assignment.get("target_id"), assignment.get("target_internal_id"), len(targets))
        targets[key] = {
            "target_id": assignment.get("target_id"),
            "target_internal_id": assignment.get("target_internal_id"),
            "mission_type": assignment.get("mission_type"),
            "assignment_id": assignment.get("assignment_id"),
            "snapshot": snapshot,
        }

    uavs: Dict[str, Dict[str, Any]] = {}
    for route in frame.get("route_plan", {}).get("routes", []) or []:
        snapshot = route.get("uav_snapshot")
        if not snapshot:
            continue
        key = _stable_key(route.get("uav_id"), route.get("drone_id"), len(uavs))
        uavs[key] = {
            "uav_id": route.get("uav_id"),
            "drone_id": route.get("drone_id"),
            "route_id": route.get("route_id"),
            "assignment_id": route.get("assignment_id"),
            "snapshot": snapshot,
        }

    response = _frame_meta(frame)
    response.update({"ok": True, "targets": list(targets.values()), "uavs": list(uavs.values())})
    return response


def station_flight_commands_from_frame(frame: Dict[str, Any]) -> Dict[str, Any]:
    """从 PlanFrame 裁剪设备一侧飞控指令输入。

    以 route 为执行主线，并按 assignment_id 合并“打谁/任务类型”。若找不到
    对应 assignment，会写入 binding_errors，供设备一拒绝或人工确认。
    """
    assignments = frame.get("assignment_plan", {}).get("assignments", []) or []
    # route 负责“怎么飞”，assignment 负责“打谁”；这里用 assignment_id 合并两半数据。
    assignment_by_id = {item.get("assignment_id"): item for item in assignments if item.get("assignment_id")}
    binding_errors: List[Dict[str, Any]] = []
    commands: List[Dict[str, Any]] = []

    for route in frame.get("route_plan", {}).get("routes", []) or []:
        assignment_id = route.get("assignment_id")
        assignment = assignment_by_id.get(assignment_id, {})
        if assignment_id and not assignment:
            binding_errors.append({"reason": "assignment_not_found", "assignment_id": assignment_id, "route_id": route.get("route_id")})
        control_intent = route.get("control_intent") or {}
        commands.append({
            "assignment_id": assignment_id,
            "assignment_epoch": _first_present(route.get("assignment_epoch"), assignment.get("assignment_epoch")),
            "route_id": route.get("route_id"),
            "uav_id": _first_present(route.get("uav_id"), assignment.get("uav_id")),
            "drone_id": _first_present(route.get("drone_id"), assignment.get("drone_id")),
            "target_id": _first_present(assignment.get("target_id"), route.get("target_id")),
            "mission_type": _first_present(assignment.get("mission_type"), control_intent.get("mission_type")),
            "role": assignment.get("role"),
            "formation_role": assignment.get("formation_role"),
            "status": route.get("status"),
            "failure_reason": route.get("failure_reason"),
            "target_snapshot": assignment.get("target_snapshot"),
            "uav_snapshot": route.get("uav_snapshot"),
            "speed_mps": route.get("speed_mps"),
            "next_command_point": route.get("next_command_point"),
            "waypoints": route.get("waypoints", []) or [],
            "control_intent": control_intent,
        })

    response = _frame_meta(frame)
    response.update({"ok": not binding_errors, "binding_errors": binding_errors, "commands": commands})
    return response


def decision_output_frame_from_plan_frame(frame: Dict[str, Any]) -> Dict[str, Any]:
    """从完整 PlanFrame 裁剪设备一直接消费的决策输出帧。

    该视图只保留设备一操控和大屏显示最常用的信息：一句话态势、
    单机任务绑定、低层动作、短航点和下一指令点。
    """
    command_view = station_flight_commands_from_frame(frame)
    commands: List[Dict[str, Any]] = []
    for command in command_view.get("commands", []) or []:
        control_intent = command.get("control_intent") or {}
        commands.append({
            "uav_id": command.get("uav_id"),
            "drone_id": command.get("drone_id"),
            "target_id": command.get("target_id"),
            "assignment_id": command.get("assignment_id"),
            "assignment_epoch": command.get("assignment_epoch"),
            "route_id": command.get("route_id"),
            "mission_type": command.get("mission_type"),
            "role": command.get("role"),
            "formation_role": command.get("formation_role"),
            "status": command.get("status"),
            "failure_reason": command.get("failure_reason"),
            "speed_mps": command.get("speed_mps"),
            "actions": control_intent.get("low_level_actions", []) or [],
            "control_intent": control_intent.get("intent"),
            "next_command_point": command.get("next_command_point"),
            "waypoints": command.get("waypoints", []) or [],
        })

    return {
        "msg_type": "decision_output_frame",
        "schema_version": "decision_output.v1",
        "source_msg_type": frame.get("msg_type"),
        "source_schema_version": frame.get("schema_version"),
        "source_frame_id": frame.get("frame_id"),
        "radar_frame_id": frame.get("radar_frame_id"),
        "plan_seq": frame.get("plan_seq"),
        "timestamp_ms": frame.get("timestamp_ms"),
        "valid_until_ms": frame.get("valid_until_ms"),
        "situation_alert": frame.get("situation_alert"),
        "binding_ok": command_view.get("ok", True),
        "binding_errors": command_view.get("binding_errors", []) or [],
        "commands": commands,
    }


def _frame_meta(frame: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "frame_id": frame.get("frame_id"),
        "plan_seq": frame.get("plan_seq"),
        "timestamp_ms": frame.get("timestamp_ms"),
        "valid_until_ms": frame.get("valid_until_ms"),
    }


def _first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _stable_key(*values: Any) -> str:
    for value in values:
        if value is not None and value != "":
            return str(value)
    return "unknown"


def _route_response(ok: bool, route: Dict[str, Any], reason: str) -> Dict[str, Any]:
    return {
        "ok": bool(ok),
        "reason": reason,
        "uav_id": route.get("uav_id"),
        "drone_id": route.get("drone_id"),
        "uav_internal_id": (route.get("uav_snapshot") or {}).get("uav_internal_id"),
        "assignment_id": route.get("assignment_id"),
        "assignment_epoch": route.get("assignment_epoch"),
        "target_id": route.get("target_id"),
        "mission_type": (route.get("control_intent") or {}).get("mission_type"),
        "route_id": route.get("route_id"),
        "route_status": route.get("status"),
        "failure_reason": route.get("failure_reason"),
    }


def _route_matches(
    route: Dict[str, Any],
    uav_id: Optional[str],
    drone_id: Optional[Any],
    internal_id: Optional[Any],
) -> bool:
    if uav_id is not None and str(route.get("uav_id")) == str(uav_id):
        return True
    if drone_id is not None and str(route.get("drone_id")) == str(drone_id):
        return True
    route_internal_id = (route.get("uav_snapshot") or {}).get("uav_internal_id")
    return internal_id is not None and str(route_internal_id) == str(internal_id)


def _self_check() -> None:
    frame = {
        "frame_id": "frame_1",
        "plan_seq": 1,
        "assignment_plan": {
            "assignments": [
                {
                    "assignment_id": "asg_uav_01_e0001",
                    "assignment_epoch": 1,
                    "uav_id": "uav_01",
                    "drone_id": 1,
                    "uav_internal_id": 0,
                    "target_id": "target_01",
                    "target_internal_id": 0,
                    "mission_type": "intercept_hit",
                    "target_snapshot": {"target_id": "target_01", "position": {"x": 30.0, "y": 40.0, "z": 10.0}},
                    "role": "PRIMARY",
                    "formation_role": None,
                    "state": "assigned",
                    "group_id": "grp_target_01",
                }
            ]
        },
        "route_plan": {
            "routes": [
                {
                    "route_id": "route_1",
                    "assignment_id": "asg_uav_01_e0001",
                    "assignment_epoch": 1,
                    "uav_id": "uav_01",
                    "drone_id": 1,
                    "target_id": "target_01",
                    "status": "ready",
                    "failure_reason": "",
                    "control_intent": {"mission_type": "intercept_hit"},
                    "uav_snapshot": {"uav_internal_id": 0, "position": {"x": 1.0, "y": 2.0, "z": 3.0}},
                    "speed_mps": 22.0,
                    "next_command_point": {"x": 10.0, "y": 20.0, "z": 30.0},
                    "waypoints": [{"x": 10.0, "y": 20.0, "z": 30.0, "speed_mps": 22.0}],
                }
            ]
        },
    }
    assert assignments_from_frame(frame)["assignments"][0]["uav_id"] == "uav_01"
    assert next_command_point_from_frame(frame, uav_id="uav_01")["point"]["x"] == 10.0
    assert next_command_point_from_frame(frame, drone_id=1)["ok"]
    assert next_command_point_from_frame(frame, internal_id=0)["ok"]
    assert not next_command_point_from_frame(frame)["ok"]
    snapshot = station_state_snapshot_from_frame(frame)
    assert snapshot["targets"][0]["target_id"] == "target_01"
    assert snapshot["uavs"][0]["uav_id"] == "uav_01"
    commands = station_flight_commands_from_frame(frame)
    assert commands["ok"] and commands["commands"][0]["speed_mps"] == 22.0
    assert commands["commands"][0]["next_command_point"]["x"] == 10.0
    decision_frame = decision_output_frame_from_plan_frame(frame)
    assert decision_frame["msg_type"] == "decision_output_frame"
    assert decision_frame["commands"][0]["waypoints"][0]["x"] == 10.0
    bad_frame = {"route_plan": {"routes": [{"route_id": "route_bad", "assignment_id": "missing"}]}}
    assert not station_flight_commands_from_frame(bad_frame)["ok"]
    print("station.query自检通过")


if __name__ == "__main__":
    _self_check()
