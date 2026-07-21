# station 决策输出模块

职责：把设备2决策结果转换为可回发设备一的 `PlanFrame` / `decision_output_frame`，并处理设备一执行回执。

主要文件：

- `contracts.py`：完整 `PlanFrame` 构造。
- `exporter.py`：运行时发布 `PlanFrame`。
- `query.py`：从 `PlanFrame` 裁剪状态快照、飞控指令和精简决策帧。
- `plan_bridge.py`：历史地面站转接包，当前作为兼容旁路。
- `socket_client.py`：TCP JSON Lines 客户端。
- `feedback.py`：设备一执行回执处理。
- `output.py`：决策输出模块接口。
