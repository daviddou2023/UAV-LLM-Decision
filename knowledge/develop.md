# 开发知识

最后通读时间：2026-07-14

## 架构

本项目把仿真、数据接入、LLM 命令处理、任务分配、路径生成、显示导出和决策帧回发组合在一起。当前联调系统为两设备：

- 设备一负责雷达数据清理、战场态势输入、真实飞控执行和指挥大屏显示。
- 设备二是本仓库主体，作为决策外挂，负责根据设备一态势输入输出任务绑定、路径点、动作建议和一句话态势总结。

`simulation/main.py` 中的主要运行流程：

1. 从 demo、UDP、Redis 或 fusion 数据源接入设备一态势输入。
2. 更新 LLM 态势分析状态，并解析命令约束。
3. 识别活动目标，并按撞击、列阵、网阻模式拆分。
4. 执行任务分配。
5. 更新起飞状态和干扰效果。
6. 通过局部避碰移动无人机，并刷新 `it["path_plan"]`。
7. 生成完整 `PlanFrame`，默认裁剪为 `decision_output_frame` 回发设备一。

## 关键状态结构

我方无人机是 `core/common.py` 中 `create_interceptor()` 创建的可变字典。

重要无人机字段：

- `id`：从 0 开始的内部无人机编号。
- `state`：`IState`。
- `role`：`IRole`。
- `target_id`：从 0 开始的内部目标编号，或 `None`。
- `poi`：当前预测拦截点或编队槽位点。
- `path_plan`：本地 `(x, y, z)` 路径点列表，由 `_update_route_plan()` 刷新。
- `path_reason`：路径原因说明。
- `net_slot`、`barrier_slot`、`barrier_center`：协同编队字段。
- `task_reserved`：被 LLM 保留约束锁定时为 true。
- `device3_temporarily_unavailable`：历史字段名，当前表示设备一执行侧回传失败后临时不可用。

敌方目标是 `create_enemy()` 或实时数据源创建的可变字典。

重要敌方字段：

- `id`：从 0 开始的内部目标编号。
- `external_id`：存在时作为稳定对外编号。
- `state`：`EState`。
- `type`：`EType`。
- `detected`、`lost`、`stale`。
- `x`、`y`、`z`、`speed`、`heading`、`vz`。

输入帧元数据保存在 `env.last_live_packet_meta`，用于 `radar_frame_id` 和 `publish_policy="input_frame"` 的发送去重。

## 分配逻辑

`decision/cooperation.py` 实现撞击模式任务分配：

- `assignments` 结构：`{enemy_id: {"primary": iid, "follower": iid, "poi": point, "eta": sec}}`。
- 一个目标先分配主拦截机。
- 随动机是可选的，只在资源/保留规则允许时分配。
- LLM 约束通过 `task_constraints` 影响目标排序和出动容量。

`simulation/main.py` 实现编队分配：

- `barrier_team_assignments`：`{enemy_id: [iid, ...]}`。
- `net_team_assignments`：`{enemy_id: [iid, ...]}`。
- 默认编队规模由 `CFG.BARRIER_GROUP_SIZE` 和 `CFG.NET_GROUP_SIZE` 配置。

## 规划输出

`station/contracts.py` 把当前环境状态转换为传输无关的 `PlanFrame`。

关键输出：

- `situation_alert`：一句话态势总结。
- `assignment_plan.assignments`：每架被导出无人机一条分配记录。
- `route_plan.routes`：每架被导出无人机一条路径记录。
- `route.next_command_point`：面向后续 GOTO 控制的即时指令点。
- `route.control_intent.low_level_actions`：设备一可转换为飞控动作的建议动作列表。
- `groups`：按目标或单机任务形成编组。

`station/query.py` 从同一份 `PlanFrame` 裁剪：

- `decision_output_frame_from_plan_frame(frame)`：设备一主消费精简帧。
- `station_flight_commands_from_frame(frame)`：飞控指令过滤视图。
- `station_state_snapshot_from_frame(frame)`：状态快照过滤视图。
- `next_command_point_from_frame(frame, ...)`：按 UAV 查询下一指令点。

`station/exporter.py` 负责运行时发布：

- 统一从 `core/common.py` 的 `PLAN_EXPORT` 读取配置。
- `PLAN_EXPORT["enabled"] = True` 开启导出。
- `PLAN_EXPORT["publish_policy"] = "input_frame"` 表示设备一输入帧变化时回发。
- `PLAN_EXPORT["output_frame_type"] = "decision_output_frame"` 表示主链路发送精简帧。
- TCP JSON Lines 是主传输方式。
- Redis 镜像仅用于调试，写入的是实际发送载荷。

## LLM 处理

LLM 态势分析和对话位于 `decision/llm_kit.py`，可以生成副官文本和 fallback 建议。

结构化命令约束由 `decision/llm_task_constraints.py` 解析，`simulation/main.py` 负责应用到环境状态。当前约束包括：

- `reserve_count`
- `target_priority`
- `preferred_sector`
- `avoid_jam`
- `max_active_count`

这些约束会同步到 `self.assigner.task_constraints`。

## 接口设计与实现

当前运行链路为：`station/exporter.py` 生成 `PlanFrame`，按配置裁剪成 `decision_output_frame` 后回发设备一；`station/query.py` 是进程内查询助手，不启动独立服务。

若后续恢复外部 HTTP/TCP 查询服务，应从 `decision/`、`station/` 的模块接口向外暴露，不在 `simulation/main.py` 中直接写协议处理。

## 后续服务封装建议

- `station/query_server.py`：如果设备一需要请求/响应式查询，可新增 TCP JSON Lines 服务。
- `planner_http_api.py`：如果后续需要 REST 调用，再引入轻量服务。
- 不要把业务 API 放进 `ui/whisper_server.py`；它只负责语音转写服务。
