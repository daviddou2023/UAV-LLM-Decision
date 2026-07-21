"""雷达/外部航迹接入的功能接口。"""
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Protocol


@dataclass
class RadarTrackFrame:
    """设备1输入经过清洗后的单帧航迹。"""

    enemies: List[Dict[str, Any]] = field(default_factory=list)
    friendlies: List[Dict[str, Any]] = field(default_factory=list)
    meta: Dict[str, Any] = field(default_factory=dict)
    events: List[Any] = field(default_factory=list)

    @classmethod
    def from_snapshot(cls, snapshot: Mapping[str, Any]) -> "RadarTrackFrame":
        return cls(
            enemies=list(snapshot.get("enemies") or []),
            friendlies=list(snapshot.get("friendlies") or []),
            meta=dict(snapshot.get("meta") or {}),
            events=list(snapshot.get("events") or []),
        )

    def to_snapshot(self) -> Dict[str, Any]:
        return {
            "enemies": self.enemies,
            "friendlies": self.friendlies,
            "meta": self.meta,
            "events": self.events,
        }


class RadarIngestPort(Protocol):
    """雷达/外部数据源端口；TeacherDataFeed、FusionTrackFeed 已满足该端口。"""

    def poll(self, sim_time: float) -> Mapping[str, Any]:
        ...


class RadarIngestService:
    """统一数据接入口：调用现有 feed，并输出 RadarTrackFrame。"""

    def __init__(self, feed: RadarIngestPort):
        self.feed = feed

    def poll(self, sim_time: float) -> RadarTrackFrame:
        """调用底层 feed 的 poll，并把字典快照收口成统一 RadarTrackFrame。

        这里不解析业务字段，目的是让设备1输入源可以被替换：只要新 feed
        返回 enemies/friendlies/meta/events，就能进入后续环境同步流程。
        """
        return RadarTrackFrame.from_snapshot(self.feed.poll(sim_time))


def _self_check() -> None:
    class DummyFeed:
        def poll(self, sim_time: float) -> Mapping[str, Any]:
            return {"enemies": [{"id": "e1"}], "friendlies": [], "meta": {"time": sim_time}, "events": ["ok"]}

    frame = RadarIngestService(DummyFeed()).poll(1.5)
    assert frame.enemies[0]["id"] == "e1"
    assert frame.meta["time"] == 1.5
    assert frame.to_snapshot()["events"] == ["ok"]


if __name__ == "__main__":
    _self_check()
    print("perception.ingest自检通过")
