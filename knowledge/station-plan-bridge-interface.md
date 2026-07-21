# 地面站侧（station）规划转接接口结构

> 当前主链路已调整为设备二回发设备一的 `decision_output_frame`。本文档仅描述历史 `station_plan_transfer` 旁路转接包，作为兼容/调试参考，不是当前主消费协议。

最后更新：2026-07-11

本文档按当前代码实现整理，来源以 `station/plan_bridge.py` 和 `station/contracts.py` 为准。正文统一称“地面站（station）”。它描述的是地面站侧（station）旁路转接包（消息名 `station_plan_transfer`），不是当前设备二回发设备一的主决策帧。

## 命名说明

当前实现统一使用 station 命名：代码文件 `station/plan_bridge.py`，消息名 `station_plan_transfer`，协议版本 `station.plan_bridge.v1`。

## 接口边界

- 生成位置：`station.plan_bridge.build_station_transfer_packet(frame)`。
- 数据来源：`station/exporter.py` 已生成的同一份 `PlanFrame`。
- 转发方式：TCP JSON Lines，一行一个 JSON 对象，UTF-8 编码，末尾 `\n`。
- 默认地址：`core/common.py` 中 `STATION_BRIDGE["host"]="127.0.0.1"`，`STATION_BRIDGE["port"]=7101`。
- 开启条件：`core/common.py` 中 `PLAN_EXPORT["enabled"] = True` 且 `STATION_BRIDGE["enabled"] = True`。
- 发送频率：跟随 `core/common.py` 中 `PLAN_EXPORT["publish_interval"]`，默认 1 秒一次。
- 当前方向：设备2 -> 地面站侧（station）单向转发；该旁路当前不单独处理回执。
- 设计原则：只合并字段，不重新分配、不重新规划、不改变原始仿真状态。

## 顶层 JSON 结构

```json
{
  "msg_type": "station_plan_transfer",
  "schema_version": "station.plan_bridge.v1",
  "source_msg_type": "plan_frame",
  "source_schema_version": "planframe.v1",
  "frame_id": "plan_frame_00000001",
  "plan_seq": 1,
  "timestamp_ms": 1780992000000,
  "valid_duration_ms": 1500,
  "valid_until_ms": 1780992001500,
  "coordinate_frame": {
    "local": "meters_xy_alt",
    "geo": "wgs84_lat_lon_alt",
    "origin_lat": 34.15878,
    "origin_lon": 108.692348
  },
  "binding_rule": "assignment_id_uav_id_drone_id_must_match",
  "binding_ok": true,
  "binding_errors": [],
  "groups": [],
  "uav_plans": []
}
```

## Python 成员结构

下面是按当前真实字段整理的 `TypedDict` 风格结构，方便联调时逐项对字段。

```python
from typing import Literal, Optional, TypedDict


class Position(TypedDict):
    x: float
    y: float
    z: float
    lat: Optional[float]
    lon: Optional[float]
    lng: Optional[float]
    alt: float


class CoordinateFrame(TypedDict):
    local: str
    geo: str
    origin_lat: float
    origin_lon: float


class TargetSnapshot(TypedDict):
    target_id: str
    target_internal_id: int
    target_type: str
    state: str
    confidence: float
    track_quality: float
    speed_mps: float
    heading_deg: float
    frame: Optional[int]
    source: str
    lost: bool
    stale: bool
    position: Position


class UavSnapshot(TypedDict):
    uav_internal_id: int
    state: str
    speed_mps: float
    heading_deg: float
    battery: float
    fault: str
    position: Position


class LowLevelAction(TypedDict, total=False):
    command: str
    policy: str
    alt_m: float
    mode: str


class ControlIntent(TypedDict):
    intent: str
    preferred_mode: str
    supported_modes: list[str]
    low_level_actions: list[LowLevelAction]
    idempotent: bool
    fallback_action: str


class Waypoint(TypedDict):
    seq: int
    x: float
    y: float
    z: float
    lat: Optional[float]
    lon: Optional[float]
    lng: Optional[float]
    alt: float
    speed_mps: float
    arrival_time_sec: float


class BindingError(TypedDict):
    reason: Literal[
        "duplicate_route_assignment_id",
        "duplicate_or_empty_assignment_id",
        "route_missing",
        "uav_binding_mismatch",
    ]
    assignment_id: Optional[str]
    assignment_uav_id: Optional[str]
    assignment_drone_id: Optional[int]
    route_uav_id: Optional[str]
    route_drone_id: Optional[int]


class GroupMember(TypedDict):
    uav_id: str
    drone_id: int
    assignment_id: str
    role: str
    formation_role: Optional[str]


class GroupSync(TypedDict):
    terminal_time_constraint: str
    formation_center: Optional[Position]


class Group(TypedDict):
    group_id: str
    group_type: str
    target_id: Optional[str]
    members: list[GroupMember]
    sync: GroupSync


class UavPlan(TypedDict):
    uav_id: str
    drone_id: int
    uav_internal_id: int
    assignment_id: str
    assignment_epoch: int
    group_id: str
    mission_type: str
    role: str
    formation_role: Optional[str]
    state: str
    target_id: Optional[str]
    target_internal_id: Optional[int]
    target_snapshot: Optional[TargetSnapshot]
    route_id: str
    route_status: str
    failure_reason: str
    control_intent: ControlIntent
    path_reason: str
    speed_mps: float
    next_command_point: Optional[Waypoint]
    waypoints: list[Waypoint]
    uav_snapshot: UavSnapshot


class StationPlanTransfer(TypedDict):
    msg_type: Literal["station_plan_transfer"]
    schema_version: Literal["station.plan_bridge.v1"]
    source_msg_type: str
    source_schema_version: str
    frame_id: str
    plan_seq: int
    timestamp_ms: int
    valid_duration_ms: int
    valid_until_ms: int
    coordinate_frame: CoordinateFrame
    binding_rule: Literal["assignment_id_uav_id_drone_id_must_match"]
    binding_ok: bool
    binding_errors: list[BindingError]
    groups: list[Group]
    uav_plans: list[UavPlan]
```

## 单架 UAV 规划结构

`uav_plans[]` 是地面站侧（station）最需要看的结构。它由同一个 `assignment_id` 下的任务分配条目和路径条目合并而来。

```json
{
  "uav_id": "uav_01",
  "drone_id": 1,
  "uav_internal_id": 0,
  "assignment_id": "asg_uav_01_e0001",
  "assignment_epoch": 1,
  "group_id": "group_target_target_001",
  "mission_type": "intercept",
  "role": "primary",
  "formation_role": null,
  "state": "planned",
  "target_id": "target_001",
  "target_internal_id": 0,
  "target_snapshot": {
    "target_id": "target_001",
    "target_internal_id": 0,
    "target_type": "normal",
    "state": "approaching",
    "confidence": 1.0,
    "track_quality": 1.0,
    "speed_mps": 22.0,
    "heading_deg": 90.0,
    "frame": 123,
    "source": "redis",
    "lost": false,
    "stale": false,
    "position": {
      "x": 1500.0,
      "y": 3200.0,
      "z": 42.0,
      "lat": 34.1585,
      "lon": 108.695,
      "lng": 108.695,
      "alt": 42.0
    }
  },
  "route_id": "route_asg_uav_01_e0001_00000001",
  "route_status": "planned",
  "failure_reason": "",
  "control_intent": {
    "intent": "execute_intercept_route",
    "preferred_mode": "mission_waypoints",
    "supported_modes": ["mission_waypoints", "goto"],
    "low_level_actions": [
      {"command": "arm", "policy": "if_needed"},
      {"command": "takeoff", "policy": "if_not_airborne", "alt_m": 15.0},
      {"command": "set_mode", "mode": "AUTO", "policy": "if_needed"},
      {"command": "upload_short_waypoints", "policy": "overwrite_previous"}
    ],
    "idempotent": true,
    "fallback_action": "none"
  },
  "path_reason": "intercept_path",
  "speed_mps": 24.0,
  "next_command_point": {
    "seq": 0,
    "x": 4200.0,
    "y": 7200.0,
    "z": 28.0,
    "lat": 34.1592,
    "lon": 108.6938,
    "lng": 108.6938,
    "alt": 28.0,
    "speed_mps": 24.0,
    "arrival_time_sec": 20.833
  },
  "waypoints": [],
  "uav_snapshot": {
    "uav_internal_id": 0,
    "state": "launching",
    "speed_mps": 18.0,
    "heading_deg": 270.0,
    "battery": 0.9,
    "fault": "",
    "position": {
      "x": 500.0,
      "y": 1000.0,
      "z": 15.0,
      "lat": 34.158,
      "lon": 108.692,
      "lng": 108.692,
      "alt": 15.0
    }
  }
}
```

## 字段来源

| 地面站侧（station）字段 | 来源 |
| --- | --- |
| `frame_id` / `plan_seq` / `timestamp_ms` / `valid_*` | `PlanFrameBuilder.build()` 顶层字段 |
| `coordinate_frame` | `PlanFrame.coordinate_frame` |
| `groups` | `PlanFrame.groups` 原样透传 |
| `uav_plans[].assignment_*` | `PlanFrame.assignment_plan.assignments[]` |
| `uav_plans[].uav_id` / `drone_id` | `PLAN_EXPORT["uav_id_map"]` 解析结果，默认内部 0 -> `uav_01`/`1` |
| `uav_plans[].target_snapshot` | 当前目标快照，含本地坐标和经纬度 |
| `uav_plans[].route_*` | `PlanFrame.route_plan.routes[]` |
| `uav_plans[].next_command_point` | `waypoints[0]`，没有可用航点时为 `null` |
| `uav_plans[].waypoints` | 由 `path_plan` 或当前目标/返航点采样得到的短航迹点 |
| `uav_plans[].uav_snapshot` | 当前 UAV 状态快照 |

## 绑定校验规则

地面站侧（station）转接前会校验每条任务和路径是否能一一对应：

```text
assignment.assignment_id == route.assignment_id
assignment.uav_id        == route.uav_id
assignment.drone_id      == route.drone_id
```

默认 `STATION_BRIDGE["strict_binding"] = True`。只要出现以下问题，整包会被阻止发送：

- `duplicate_route_assignment_id`：同一个 `assignment_id` 出现多条 route。
- `duplicate_or_empty_assignment_id`：assignment_id 为空或重复。
- `route_missing`：有 assignment 但找不到对应 route。
- `uav_binding_mismatch`：assignment 和 route 的 UAV 绑定不一致。

如果现场只想先看数据、不阻断发送，可临时设置 `STATION_BRIDGE["strict_binding"] = False`，但正式联调建议保持默认严格校验。

## 枚举取值

### `mission_type`

- `intercept`：撞击/主拦截任务。
- `net_capture`：网阻编队任务。
- `barrier_net`：列阵/扯网编队任务。
- `return_home`：返航。
- `retask_pending`：等待下一次重分配。
- `uav_unavailable`：设备一执行侧回传失败后临时不可用。

### `role`

- `primary`：主拦截机。
- `backup`：随动/备份机。
- `reserve`：预备或非主备角色。

### `route_status`

- `planned`：路径可执行。
- `pending`：等待重分配或下一版路径。
- `infeasible`：当前无可用未来航点或目标不可用。
- `failed`：UAV 已被设备一执行失败回执标记不可用。

### `control_intent.intent`

- `execute_intercept_route`：执行拦截路径。
- `return_home`：返航。
- `hold_for_retask`：短时保持，等待重分配。
- `hold_for_recovery`：故障恢复等待，不上传新路径。

## 联调启动示例

先改 `core/common.py`：

```python
PLAN_EXPORT["enabled"] = True
PLAN_EXPORT["uav_id_map"] = "uav_01:1,uav_02:2,uav_03:3,uav_04:4"

STATION_BRIDGE["enabled"] = True
STATION_BRIDGE["host"] = "192.168.1.50"
STATION_BRIDGE["port"] = 7101
```

再正常启动：

```bash
bash run_fusion_custom.sh
```

## 对接口时优先确认

1. 地面站侧（station）是否接收 TCP JSON Lines，还是需要 Redis/UDP/HTTP。
2. 地面站侧（station）需要完整 `groups`，还是只需要 `uav_plans`。
3. `uav_id` 与 `drone_id` 的现场映射是否就是 `core/common.py` 中 `PLAN_EXPORT["uav_id_map"]`。
4. 经纬度是否使用 `GEO_ORIGIN_LAT` / `GEO_ORIGIN_LON` 生成，还是只看本地 `x/y/z`。
5. 控制侧使用 `waypoints` 短航点，还是只使用 `next_command_point` 做 GOTO。
6. `route_status != planned` 时，对方是忽略该 UAV、保持上一条路径，还是显示告警。
