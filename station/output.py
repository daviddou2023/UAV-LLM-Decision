"""设备2到设备一的决策输出接口。"""
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Protocol

from station.contracts import PlanFrameBuilder, PlannerContractOptions
from station.plan_bridge import build_station_transfer_packet
from station.query import (
    decision_output_frame_from_plan_frame,
    station_flight_commands_from_frame,
    station_state_snapshot_from_frame,
)


@dataclass
class StationOutputFrame:
    """设备一视角的一次输出。"""

    plan_frame: Dict[str, Any]
    decision_output_frame: Dict[str, Any]
    state_snapshot: Dict[str, Any]
    flight_commands: Dict[str, Any]
    transfer_packet: Dict[str, Any]


class StationOutputPort(Protocol):
    """设备2到设备一输出端口。"""

    def from_plan_frame(self, frame: Dict[str, Any]) -> StationOutputFrame:
        ...


class StationOutputService:
    """决策输出门面：复用 PlanFrame，不重新做任务分配或路径规划。"""

    def __init__(
        self,
        builder: Optional[PlanFrameBuilder] = None,
        options: Optional[PlannerContractOptions] = None,
    ):
        self.builder = builder or PlanFrameBuilder()
        self.options = options or PlannerContractOptions()
        self.plan_seq = 0

    def build_plan_frame(self, env: Any) -> Dict[str, Any]:
        """从当前设备2环境状态生成完整 PlanFrame。"""
        self.plan_seq += 1
        return self.builder.build(env, self.options, self.plan_seq, int(time.time() * 1000))

    def from_env(self, env: Any) -> StationOutputFrame:
        """测试/查询入口：先构建 PlanFrame，再裁剪设备一消费视图。"""
        return self.from_plan_frame(self.build_plan_frame(env))

    def from_plan_frame(self, frame: Dict[str, Any]) -> StationOutputFrame:
        """从同一 PlanFrame 派生完整帧、精简决策帧、状态快照、飞控指令和转接包。"""
        return StationOutputFrame(
            plan_frame=frame,
            decision_output_frame=decision_output_frame_from_plan_frame(frame),
            state_snapshot=station_state_snapshot_from_frame(frame),
            flight_commands=station_flight_commands_from_frame(frame),
            transfer_packet=build_station_transfer_packet(frame),
        )


def _self_check() -> None:
    frame = {
        "msg_type": "device2_plan_frame",
        "schema_version": "device2.plan_frame.v1",
        "frame_id": "frame_1",
        "plan_seq": 1,
        "timestamp_ms": 1,
        "assignment_plan": {"assignments": []},
        "route_plan": {"routes": []},
        "groups": [],
    }
    output = StationOutputService().from_plan_frame(frame)
    assert output.plan_frame["frame_id"] == "frame_1"
    assert output.decision_output_frame["msg_type"] == "decision_output_frame"
    assert output.transfer_packet["msg_type"] == "station_plan_transfer"
    assert output.transfer_packet["binding_ok"] is True


if __name__ == "__main__":
    _self_check()
    print("station.output自检通过")
