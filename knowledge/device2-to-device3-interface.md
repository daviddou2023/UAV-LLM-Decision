# 设备二回发设备一决策帧接口说明

最后更新：2026-07-14

> 文件名沿用旧“device2-to-device3”命名，避免历史引用断裂；当前真实语义是“设备二回发设备一”。

## 1. 范围

设备一负责雷达接收、数据清理、态势统筹、真实无人机操控和指挥大屏显示。设备二作为我方决策外挂，接收设备一给出的当前战场态势，运行决策算法，并回发决策帧。

本接口规定设备二向设备一发送的决策结果内容、触发方式、TCP JSON Lines 消息格式和执行回执处理方式。

## 2. 总体方案

| 项目 | 说明 |
| --- | --- |
| 主传输 | TCP JSON Lines |
| 方向 | 设备二 -> 设备一；设备一可在同连接回传 ACK/执行状态 |
| 默认输出 | `decision_output_frame` |
| 完整来源 | `PlanFrame` |
| 默认触发 | 设备一输入帧变化触发，即 `PLAN_EXPORT["publish_policy"]="input_frame"` |
| 默认裁剪 | `PLAN_EXPORT["output_frame_type"]="decision_output_frame"` |

设备二内部仍先生成完整 `PlanFrame`，再裁剪成设备一主消费的 `decision_output_frame`。这样能保证完整调试帧、精简控制帧和旁路查询都来自同一份任务分配和路径规划结果。

## 3. 配置

配置统一放在 `core/common.py`：

```python
PLAN_EXPORT = {
    "enabled": False,
    "transport": "tcp_json_lines",
    "publish_policy": "input_frame",
    "output_frame_type": "decision_output_frame",
    "socket_host": "127.0.0.1",
    "socket_port": 7001,
    "valid_duration_ms": 1500,
    "waypoint_count": 3,
    "waypoint_spacing_m": 500.0,
    "uav_id_map": "",
}
```

`publish_policy` 支持：

| 值 | 说明 |
| --- | --- |
| `input_frame` | 设备一输入帧变化时发送，默认 |
| `interval` | 按 `publish_interval` 周期发送 |
| `input_frame_or_interval` | 输入帧变化立即发送，否则周期兜底 |

`output_frame_type` 支持：

| 值 | 说明 |
| --- | --- |
| `decision_output_frame` | 发送精简决策帧，默认 |
| `plan_frame` | 发送完整 `PlanFrame` |

## 4. 完整帧 `PlanFrame`

生成位置：`station/contracts.py::PlanFrameBuilder.build()`。

核心字段：

| 字段 | 含义 |
| --- | --- |
| `msg_type="plan_frame"` | 完整帧消息类型 |
| `schema_version="planframe.v1"` | 完整帧版本 |
| `frame_id` / `plan_seq` | 设备二输出帧编号 |
| `radar_frame_id` | 本次决策基于的设备一输入帧号 |
| `timestamp_ms` / `valid_until_ms` | 生成时间和有效期 |
| `situation_alert` | 一句话态势总结 |
| `assignment_plan.assignments[]` | 单机任务绑定 |
| `route_plan.routes[]` | 单机路径、动作和航点 |
| `groups[]` | 多机协同编组 |

## 5. 精简帧 `decision_output_frame`

生成位置：`station/query.py::decision_output_frame_from_plan_frame(frame)`。

示例：

```json
{
  "msg_type": "decision_output_frame",
  "schema_version": "decision_output.v1",
  "source_frame_id": "plan_frame_00000023",
  "radar_frame_id": 1001,
  "plan_seq": 23,
  "timestamp_ms": 1783766400000,
  "valid_until_ms": 1783766401500,
  "situation_alert": "当前发现2个活动目标，6架我方无人机可用，3个任务绑定。",
  "binding_ok": true,
  "binding_errors": [],
  "commands": []
}
```

`commands[]` 每条对应一架己方 UAV：

| 字段 | 含义 |
| --- | --- |
| `uav_id` / `drone_id` | 设备一执行主体 |
| `target_id` | 当前绑定的敌方目标 |
| `assignment_id` / `assignment_epoch` | 单机任务 ID 和任务代数 |
| `route_id` | 当前路径 ID |
| `mission_type` | 拦截、返航、待重分配、不可用等 |
| `role` / `formation_role` | 主拦截、随动、编队槽位 |
| `status` / `failure_reason` | 路径状态和失败原因 |
| `speed_mps` | 建议速度 |
| `actions[]` | 解锁、起飞、切模式、上传航点或等待 |
| `control_intent` | 高层控制意图 |
| `next_command_point` | 下一指令点 |
| `waypoints[]` | 短路径点 |

## 6. 设备一回执

设备一可通过同一 TCP JSON Lines 连接回传：

| `msg_type` | 用途 | 处理位置 |
| --- | --- | --- |
| `plan_ack` | 告知是否接受某个 `plan_seq` | `station/feedback.py::_handle_plan_ack()` |
| `execution_report` | 回传执行中、完成、失败、中止等状态 | `station/feedback.py::_handle_execution_report()` |
| `uav_status` | 可选回传真实 UAV 状态或故障 | `station/feedback.py::_handle_uav_status()` |

当 `execution_report.status` 为 `failed` 或 `aborted` 时，设备二会将对应 UAV 标记为临时不可用，释放旧任务绑定，并触发重分配。

## 7. 绑定和切换规则

1. `assignment_id + uav_id + drone_id` 必须能在 assignment 和 route 中一致对应。
2. 同一 `uav_id` 下 `assignment_epoch` 不变，表示仍是同一任务，只更新路径。
3. `assignment_epoch` 递增或 `target_id` 变化，表示切换任务。
4. `mission_type=return_home` 表示无目标返航。
5. `mission_type=retask_pending` 表示设备二正在等待下一次重分配。
6. `mission_type=uav_unavailable` 表示执行侧回执或内部状态认为该 UAV 暂不可用。

## 8. 联调待确认

1. 设备一 TCP JSON Lines 服务端监听地址和端口。
2. 设备一输入帧元数据中哪个字段作为稳定帧号。
3. 设备一收到短航点后的覆盖执行频率上限。
4. `uav_id/drone_id` 的现场映射。
5. 设备一回传 `plan_ack`、`execution_report`、`uav_status` 的最终字段。
6. TCP 断线期间设备一保持上一条有效路径、等待新帧或执行安全动作的策略。
