"""
Reserved feedback handling for Device1 -> Device2 messages.

    负责听取设备一执行侧的“回话”，并据此决定仿真系统内部的无人机状态是继续执行、还是直接判死（触发故障降级）

该模块生成的逻辑被 station/exporter.py 中的 PlannerExporter.maybe_publish 调用。每当 PlannerExporter 发送完规划帧后，会通过 self.socket.poll_feedback() 拉取外部数据，并将这些数据转交给 station/feedback.py 进行逻辑解析

"""
from typing import Any, Dict, Iterable, List, Tuple

from core.common import IRole, IState
from station.contracts import parse_uav_id_map


class Device3FeedbackHandler:
    def __init__(self, uav_id_map: str = "", failed_cooldown_sec: float = 5.0):
        self.uav_id_map_text = uav_id_map
        self.failed_cooldown_sec = max(0.0, float(failed_cooldown_sec))
        self.last_ack_seq = None

    def process(self, env, events: Iterable[Dict[str, Any]]) -> List[Tuple[str, str, str]]:
        """处理设备一回执并把必要状态写回设备2环境。

        plan_ack 只记录接收结果；execution_report 在 failed/aborted 时会释放
        对应 UAV 任务并触发重分配；uav_status 只同步执行侧状态缓存。
        """
        logs: List[Tuple[str, str, str]] = []
        for event in events:
            msg_type = str(event.get("msg_type", "")).strip().lower()
            if msg_type == "plan_ack":
                log = self._handle_plan_ack(event)
                if log:
                    logs.append(log)
            elif msg_type == "execution_report":
                log = self._handle_execution_report(env, event)
                if log:
                    logs.append(log)
            elif msg_type == "uav_status":
                self._handle_uav_status(env, event)
        return logs

    def _handle_plan_ack(self, event: Dict[str, Any]):
        """处理指令回执（确认设备一是否收到了任务）"""
        plan_seq = event.get("plan_seq")
        accepted = event.get("accepted", True)
        self.last_ack_seq = plan_seq
        if accepted is False:
            reason = event.get("reason") or event.get("failure_reason") or "unknown"
            return ("[PLAN]", f"设备一拒收决策帧 seq={plan_seq}: {reason}", "amber")
        return None

    def _handle_execution_report(self, env, event: Dict[str, Any]):
        """处理执行结果：如果设备一返回 failed 或 aborted，它会直接强制标记该无人机为“不可用”并触发任务重分配"""
        # 找到哪架飞机failed
        it = self._find_interceptor(env, event)
        if not it:
            return ("[PLAN]", f"收到设备一执行回报，但未匹配无人机: {event}", "amber")

        status = str(event.get("status", "")).strip().lower()
        reason = str(event.get("reason") or event.get("failure_reason") or "").strip()
        it["device3_last_execution_report"] = dict(event)
        # 如果状态是失败或中止
        if status in ("failed", "aborted"):
            sim_now = float(getattr(env, "time", 0.0))
            # 设置该飞机的“冷却时间”，防止它在故障状态下立即被重复调度
            it["device3_temporarily_unavailable"] = True
            it["device3_failed_until"] = sim_now + self.failed_cooldown_sec
            it["device3_failure_reason"] = reason
            # 强制释放它的任务绑定 (清空 target_id, poi 等)
            self._release_failed_binding(env, it)
            # 触发全局任务重分配：让分配器把目标分给别的飞机
            reassign_msg = self._trigger_reassignment(env)
            suffix = f"，{reassign_msg}" if reassign_msg else ""
            return (
                "[PLAN]",
                f"{_uav_label(it)} 设备一执行失败: {reason or 'unknown'}，已标记暂不可用并触发重分配{suffix}",
                "amber",
            )
        # 如果执行成功，清除错误标志
        if status in ("completed", "engaging", "en_route", "executing"):
            it["device3_temporarily_unavailable"] = False
            it["device3_failure_reason"] = ""
        return None

    def _handle_uav_status(self, env, event: Dict[str, Any]):
        """接收无人机的实时状态流（位置、电量等），同步更新内部缓存"""
        it = self._find_interceptor(env, event)
        if not it:
            return
        it["device3_last_uav_status"] = dict(event)
        if "fault" in event:
            it["device3_failure_reason"] = str(event.get("fault") or "")
        if it.get("device3_temporarily_unavailable"):
            sim_now = float(getattr(env, "time", 0.0))
            if sim_now >= float(it.get("device3_failed_until", 0.0)):
                it["device3_temporarily_unavailable"] = False
                it["device3_failure_reason"] = ""

    def _trigger_reassignment(self, env):
        force_reassignment = getattr(env, "_force_task_reassignment", None)
        if not force_reassignment:
            return ""
        try:
            result = force_reassignment("设备一执行失败回执")
        except Exception as exc:
            return f"重分配触发异常: {exc}"
        if not isinstance(result, dict):
            return ""
        return result.get("message") or result.get("execution") or ""

    def _release_failed_binding(self, env, it):
        release_binding = getattr(env, "_release_interceptor_task_binding", None)
        if release_binding:
            try:
                release_binding(it)
            except Exception:
                pass
        it["target_id"] = None
        it["role"] = IRole.RESERVE
        it["search_until"] = 0.0
        it["path_plan"] = []
        it["path_reason"] = ""
        it["poi"] = None
        it["search_point"] = None
        it["search_distance"] = 0.0
        it["net_slot"] = None
        it["barrier_slot"] = None
        it["barrier_center"] = None

    def _find_interceptor(self, env, event: Dict[str, Any]):
        """映射还原：将外部传来的 drone_id 或 uav_id 转换回仿真系统内部使用的 internal_id"""
        # 外部传来的 ID
        drone_id = event.get("drone_id")
        # 外部传来的字符串 ID
        uav_id = event.get("uav_id")
        # 使用 station/contracts.py 里的工具解析 ID 映射规则
        mapping = parse_uav_id_map(self.uav_id_map_text, len(getattr(env, "interceptors", [])))
        for internal_id, ids in mapping.items():
            # 匹配逻辑：支持 DroneID 和 UAV_ID 双重校验
            if drone_id is not None:
                try:
                    if int(drone_id) == int(ids.get("drone_id")):
                        return _get_interceptor(env, internal_id)
                except (TypeError, ValueError):
                    pass
            if uav_id and str(uav_id) == str(ids.get("uav_id")):
                return _get_interceptor(env, internal_id)
        return None


def _get_interceptor(env, internal_id: int):
    getter = getattr(env, "_get_interceptor", None)
    if getter:
        return getter(internal_id)
    return next((it for it in getattr(env, "interceptors", []) if it.get("id") == internal_id), None)


def _uav_label(it: Dict[str, Any]) -> str:
    state = it.get("state")
    state_name = state.name if isinstance(state, IState) else str(state)
    return f"I-{int(it.get('id', 0)) + 1}({state_name})"
