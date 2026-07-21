"""设备2任务分配与路径规划流程接口。"""
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Protocol, Tuple

from station.contracts import PlanFrameBuilder, PlannerContractOptions

PlanLog = Tuple[str, str, str]


@dataclass
class DecisionSnapshot:
    """设备2当前决策态势摘要。"""

    sim_time: float = 0.0
    active_enemy_ids: List[Any] = field(default_factory=list)
    available_uav_ids: List[Any] = field(default_factory=list)
    assignments: List[Dict[str, Any]] = field(default_factory=list)
    alert: str = "当前无活动目标。"

    @classmethod
    def from_env(cls, env: Any) -> "DecisionSnapshot":
        enemies = list(getattr(env, "enemies", []) or [])
        interceptors = list(getattr(env, "interceptors", []) or [])
        active_enemies = [e for e in enemies if _state_name(e.get("state")) in ("APPROACHING", "MANEUVERING")]
        available = [it for it in interceptors if _state_name(it.get("state")) not in ("DESTROYED", "LOST")]
        assignments = [
            {
                "uav_id": it.get("id"),
                "target_id": it.get("target_id"),
                "state": _state_name(it.get("state")),
                "role": it.get("role"),
            }
            for it in interceptors
            if it.get("target_id") is not None
        ]
        alert = f"当前发现{len(active_enemies)}个活动目标，{len(available)}架我方无人机可用，{len(assignments)}个任务绑定。"
        return cls(
            sim_time=float(getattr(env, "time", 0.0) or 0.0),
            active_enemy_ids=[e.get("id") for e in active_enemies],
            available_uav_ids=[it.get("id") for it in available],
            assignments=assignments,
            alert=alert,
        )


@dataclass
class DecisionTickResult:
    """一次设备2决策步结果。"""

    snapshot: DecisionSnapshot
    logs: List[Any] = field(default_factory=list)
    plan_frame: Optional[Dict[str, Any]] = None


class DecisionPipelinePort(Protocol):
    """设备2决策流程端口。"""

    def tick(self, dt: float, build_plan: bool = False) -> DecisionTickResult:
        ...


class DecisionPipelineService:
    """设备2流程门面：雷达同步、任务分配、路径规划仍由 env.step(dt) 完成。"""

    def __init__(
        self,
        env: Any,
        builder: Optional[PlanFrameBuilder] = None,
        options: Optional[PlannerContractOptions] = None,
    ):
        self.env = env
        self.builder = builder or PlanFrameBuilder()
        self.options = options or PlannerContractOptions()
        self.plan_seq = 0

    def tick(self, dt: float, build_plan: bool = False) -> DecisionTickResult:
        """推进一次设备2决策循环，并返回本帧摘要。

        真正的雷达同步、任务分配和路径规划仍在 env.step(dt) 内完成；
        本门面只截取本帧新增日志，并按需复用 PlanFrameBuilder 生成输出帧。
        """
        before = len(getattr(self.env, "logs", []) or [])
        self.env.step(dt)
        logs = list((getattr(self.env, "logs", []) or [])[before:])
        frame = self.build_plan_frame() if build_plan else None
        return DecisionTickResult(snapshot=DecisionSnapshot.from_env(self.env), logs=logs, plan_frame=frame)

    def snapshot(self) -> DecisionSnapshot:
        return DecisionSnapshot.from_env(self.env)

    def build_plan_frame(self) -> Dict[str, Any]:
        self.plan_seq += 1
        return self.builder.build(self.env, self.options, self.plan_seq, int(time.time() * 1000))


def _state_name(value: Any) -> str:
    return str(getattr(value, "name", value) or "").upper()


def _self_check() -> None:
    class DummyEnv:
        def __init__(self):
            self.time = 0.0
            self.logs = []
            self.enemies = [{"id": 1, "state": "APPROACHING"}]
            self.interceptors = [{"id": 2, "state": "STANDBY", "target_id": 1, "role": "primary"}]

        def step(self, dt: float) -> None:
            self.time += dt
            self.logs.append(("[TEST]", "tick", "green"))

    result = DecisionPipelineService(DummyEnv()).tick(0.1)
    assert result.snapshot.active_enemy_ids == [1]
    assert result.snapshot.available_uav_ids == [2]
    assert result.snapshot.assignments[0]["target_id"] == 1
    assert result.logs[0][1] == "tick"


if __name__ == "__main__":
    _self_check()
    print("decision.pipeline自检通过")
