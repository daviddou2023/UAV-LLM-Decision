# Bug 与风险记录

最后通读时间：2026-07-14

本文件记录已知或疑似问题。除非已有测试证明，否则这些条目都应视为集成前需要验证。

## 历史命名漂移

代码和文档中仍保留部分旧“三设备”命名，例如：

- `station/` 目录名。
- `Device3FeedbackHandler` 类名。
- `device3_temporarily_unavailable`、`device3_failure_reason` 等内部状态字段。
- `knowledge/05_设备2到设备3...`、`knowledge/06_设备2到设备3...` 文件名。

影响：语义上当前都表示设备一执行侧或历史决策输出模块。暂不批量重命名，避免破坏分配器、反馈处理和已有引用。新增文档和用户可见日志应使用设备一/设备二语义。

## 输入帧触发风险

`station/exporter.py` 的 `publish_policy="input_frame"` 依赖 `env.last_live_packet_meta` 中的 `frame/seq/timestamp`，优先级依次为 `frame`、`seq`、`timestamp`。

影响：

- 如果设备一每次输入都没有帧号或时间戳变化，设备二可能无法触发新决策帧。
- 如果设备一帧号抖动或时间戳精度过高但内容未更新，设备二可能频繁发送。

联调前应确认设备一输入元数据的稳定字段，并优先使用 `frame` 或 `seq` 表示一组态势帧。

## 规划新鲜度

`next_command_point` 由 `PlanFrameBuilder` 基于 `path_plan` 生成。`path_plan` 通常在任务分配和起飞状态更新之后，由 `_move_interceptors()` 中的路径逻辑刷新。

影响：如果刚完成任务分配、还没进入一次运动/规划 tick 就立刻查询，可能返回不可行路径或没有未来航点。设备二主循环应先完成 `env.step(dt)`，再构造输出帧。

## 设备一执行回执匹配

`station/feedback.py` 匹配执行回执依赖 `PLAN_EXPORT["uav_id_map"]` 或默认 `uav_XX` / `drone_id`。

影响：如果设备一回传的编号体系不同，将无法匹配内部无人机，失败回执不会触发正确重分配。现场必须确认 `uav_id/drone_id` 映射表。

## 分配能力限制

撞击模式每个目标只支持主拦截机加可选随动机。超过两架的协同编组由列阵/网阻模式处理，不在 `InterceptionAssigner` 内部完成。

影响：如果设备一要求“N架无人机协同”，需要把 N=1/2 映射到撞击模式，或针对 N>2 明确选择列阵、网阻或新增自定义协同行为。

## 缺少业务 API 服务

仓库当前已有 TCP JSON Lines 发布、Redis/UDP 数据接口、Whisper Flask 语音转写服务和 LLM 看板，但没有用于任务分配查询的业务 HTTP/RPC 服务。

影响：如果设备一需要请求/响应式查询，而不是被动接收决策帧，需要新增一个小型服务或扩展 TCP JSON Lines 协议消息类型。

## Kafka 未实现

`station/exporter.py` 中 Kafka 字段仅预留，不实际发布。

影响：未实现前不要承诺 Kafka 交付能力。
