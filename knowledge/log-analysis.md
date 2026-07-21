# 日志分析指南

最后通读时间：2026-07-14

运行时日志同时存在普通字符串和带标签元组，UI 渲染器两者都能处理。很多子系统会直接追加 `[ASGN]`、`[PATH]`、`[PLAN]` 等标签。

## 重要标签

- `[DATA]`：设备一态势输入配置、实时/融合数据状态。
- `[LLM]`：LLM 意图解析、约束生成、LLM 状态事件。
- `[AI情报]`：周期性 LLM 态势告警。
- `[副官]`：LLM 副官对话回复。
- `[CMD]`：指挥命令执行和派机结果。
- `[ASGN]`：撞击模式主拦截/随动分配日志。
- `[PATH]`：路径更新。
- `[REPLAN]`：目标丢失、搜索、重分配导致的路径变化。
- `[PLAN]`：设备二回发设备一决策帧，以及设备一执行回执。
- `[BARRIER]`、`[BARRIER-KILL]`：列阵编队分配/捕获。
- `[NET]`、`[NET-KILL]`、`[NET-EMG]`：网阻编队分配/捕获/紧急兜底。
- `[RTB]`：返航状态转换。

## 正常分配序列

撞击模式下通常应看到：

1. `[ASGN] I-x -> F-y 主拦截 ...`
2. 可选的 `[ASGN] I-z -> F-y 随动 ...`
3. `_update_route_plan()` 运行后出现 `[PATH] I-x ...`
4. 如果开启决策输出且设备一可达，会出现 `[PLAN] 设备一决策Socket已连接...`

## 设备二回发设备一排查

如果设备一没有收到决策帧：

1. 检查 `core/common.py` 中 `PLAN_EXPORT["enabled"]` 是否为 `True`。
2. 检查 `PLAN_EXPORT["socket_host"]` 和 `PLAN_EXPORT["socket_port"]`。
3. 检查 `PLAN_EXPORT["publish_policy"]`。若为 `input_frame`，确认 `env.last_live_packet_meta` 中 `frame/seq/timestamp` 至少一个字段会随设备一输入帧变化；代码优先使用 `frame`，再用 `seq`，最后用 `timestamp`。
4. 搜索 `[PLAN] 设备一决策Socket暂未发送成功: ...`。
5. 如需调试镜像，开启 `PLAN_EXPORT["debug_redis_enable"] = True`，查看 `PLAN_EXPORT["debug_redis_key"]` 中的实际发送载荷。

如果设备一报告执行失败：

1. `station/feedback.py` 应记录 `[PLAN] I-x(...) 设备一执行失败...`。
2. 对应 UAV 会被设置 `device3_temporarily_unavailable=True`。这是历史字段名，当前表示执行侧临时不可用。
3. 已有任务绑定会被释放。
4. `_force_task_reassignment()` 会被调用。

## 常用搜索

PowerShell 示例：

```powershell
Select-String -Path *.py -Pattern '\[PLAN\]|\[ASGN\]|\[PATH\]|\[LLM\]'
Select-String -Path simulation/main.py -Pattern '_update_route_plan|_force_task_reassignment|process_command'
Select-String -Path station/contracts.py -Pattern 'next_command_point|assignment_plan|route_plan|situation_alert'
Select-String -Path station/exporter.py -Pattern 'publish_policy|output_frame_type|last_published_input_key'
```

## 规划接口的日志预期

- 按输入帧触发不应对同一 `last_live_packet_meta` 重复发帧。
- 下一个指令点查询不应每帧刷日志。
- 如果查询不到点，应返回原因：空闲、无路径、目标丢失、UAV 不可用或规划过期。
