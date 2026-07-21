"""规划发布链路的轻量端口定义。

这里只放运行编排层需要的最小方法集合，具体实现仍在现有模块中。
"""
from typing import Any, Dict, List, Optional, Protocol, Tuple

PlanLog = Tuple[str, str, str]


class PlanFrameBuilderPort(Protocol):
    """把当前环境状态转换为 PlanFrame 的端口。"""

    def reset_assignments(self) -> None:
        ...

    def build(self, env: Any, options: Any, plan_seq: int, timestamp_ms: int) -> Dict[str, Any]:
        ...


class PlanFrameTransportPort(Protocol):
    """PlanFrame 主传输端口，当前默认实现为 TCP JSON Lines。"""

    last_error: str

    def send_json(self, payload: Dict[str, Any]) -> bool:
        ...

    def poll_feedback(self) -> List[Dict[str, Any]]:
        ...

    def close(self) -> None:
        ...


class PlanFeedbackHandlerPort(Protocol):
    """设备一执行回执处理端口。"""

    def process(self, env: Any, events: List[Dict[str, Any]]) -> List[PlanLog]:
        ...


class PlanDebugMirrorPort(Protocol):
    """规划帧调试镜像端口。"""

    def publish_text(self, text: str) -> bool:
        ...


class PlanSideBridgePort(Protocol):
    """旁路转接端口，例如地面站侧转接。"""

    def forward(self, frame: Dict[str, Any], now: Optional[float] = None) -> List[PlanLog]:
        ...

    def close(self) -> None:
        ...
