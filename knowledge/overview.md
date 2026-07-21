# 项目总览

最后更新：2026-07-14

本文件用于新会话快速理解项目形态、关键模块和接口方向。更细的开发细节见 `knowledge/develop.md`，风险和不一致点见 `knowledge/bugs.md`。

## 总体结论

这个仓库不是典型 Web 后端项目，而是 Python/Pygame 仿真 + Redis/UDP/TCP Socket 联调程序。当前系统边界为两设备：

1. **设备一**：郭老师设备/地面统筹设备，负责雷达数据清理、当前战场态势输出、无人机真实操控和指挥大屏显示。
2. **设备二**：我方决策外挂，也就是本仓库主体，负责接收设备一态势输入，完成任务分配、路径规划、动作建议和一句话态势总结，并把决策帧回发设备一。

当前运行链路不启动独立 HTTP/RPC 查询服务。设备二由 `station/exporter.py` 构造完整 `PlanFrame`，默认裁剪成 `decision_output_frame`，通过 TCP JSON Lines 回发设备一。`station/query.py` 是进程内查询助手，只从同一份 `PlanFrame` 裁剪协同分配、下一指令点、状态快照、飞控指令和精简决策帧。

## 项目结构

### 仿真主流程

`simulation/main.py`

- 项目主入口。
- 定义 `InterceptionEnvironment`，维护敌机、我方无人机、仿真时间、任务分配、路径规划、LLM 约束、Redis/UDP/决策帧发布等。
- `step()` 是核心循环：拉取设备一态势输入，更新 LLM/约束，调用任务分配，更新无人机运动，刷新路径。
- `run_demo()` 是 Pygame 主循环入口。

### 基础数据结构与运动模型

`core/common.py`

- 定义全局配置 `CFG`。
- 定义我方无人机状态 `IState`、敌机状态 `EState`、角色 `IRole`、敌机类型 `EType`。
- 定义 `PLAN_EXPORT`：设备二回发设备一的决策输出配置，默认 `publish_policy="input_frame"`、`output_frame_type="decision_output_frame"`。
- `create_interceptor()` / `create_enemy()` 创建状态字典。

### 输入与决策

- `perception/`：设备一当前战场态势输入适配、归一化和稳定 ID 关联。
- `decision/cooperation.py`：撞击模式任务分配。
- `decision/reassignment.py`：失败/目标丢失后的重分配策略。
- `decision/deconfliction.py`：局部避碰、绕飞和下一指令点规划。
- `decision/llm_task_constraints.py`：自然语言任务约束解析。
- `decision/llm_kit.py`：态势分析与一句话提示。

### 决策输出

`station/` 是历史目录名，当前表示设备二决策输出模块。

- `station/contracts.py`：构造完整 `PlanFrame`，包含 `situation_alert`、`assignment_plan`、`route_plan` 和 `groups`。
- `station/query.py`：裁剪 `decision_output_frame`、飞控指令视图、状态快照视图和下一指令点。
- `station/exporter.py`：按设备一输入帧触发发送；也可配置成周期发送。
- `station/socket_client.py`：TCP JSON Lines 客户端。
- `station/feedback.py`：处理设备一执行回执，失败时触发重分配。
- `station/plan_bridge.py`：历史地面站旁路转接包，当前作为兼容/调试参考。

## 主流程调用关系

```text
设备一态势帧
  -> perception 输入适配
  -> InterceptionEnvironment.step()
      -> _pump_live_data()
      -> analyst.update_situation()
      -> assigner.update(...) / 编队分配
      -> _move_interceptors()
      -> _update_mission_labels()
  -> PlanFrameBuilder.build()
  -> decision_output_frame_from_plan_frame()
  -> PlannerExporter.maybe_publish()
  -> 设备一执行与显示
```

## 当前已有能力和缺口

已有能力：

- 设备一态势输入归一化。
- 稳定目标关联。
- 任务分配、重分配、路径规划和局部避碰。
- 一句话态势总结。
- 完整 `PlanFrame` 输出。
- 精简 `decision_output_frame` 输出。
- TCP JSON Lines 发布和执行失败回执闭环。

缺口：

- 没有业务 HTTP API。
- `station/`、`Device3FeedbackHandler`、`device3_*` 等历史命名尚未整体迁移，只做语义兼容。
- 若设备一输入 `meta.frame/seq/timestamp` 不稳定，按输入帧触发可能无法精确去重。
- 现场 `uav_id/drone_id` 映射仍需联调确认。

## 维护说明

- 工程结构不为设备名调整做大迁移。
- 后续新增字段优先进入 `PlanFrameBuilder`，再由 `station/query.py` 裁剪给设备一。
- 不把设备一飞控执行逻辑写入设备二核心决策模块。
