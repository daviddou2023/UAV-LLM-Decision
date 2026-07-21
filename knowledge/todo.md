# 待办记录

最后更新：2026-07-14

## 当前任务

- [x] 将系统边界从旧“三设备”同步为当前“两设备”。
- [x] 保留工程目录结构，明确 `station/` 当前作为决策输出模块使用。
- [x] 在 `PlanFrame` 中补充 `situation_alert`。
- [x] 新增 `decision_output_frame_from_plan_frame()`，从完整 `PlanFrame` 裁剪设备一主消费精简帧。
- [x] 在 `PLAN_EXPORT` 中新增 `publish_policy` 和 `output_frame_type`。
- [x] `station/exporter.py` 支持 `publish_policy="input_frame"`，按设备一输入帧触发回发，避免同一输入帧重复发送。
- [x] 将主要 knowledge 文档改为设备一/设备二边界。
- [ ] 使用真实设备一输入帧和设备一接收端补一条端到端 smoke 测试脚本。
- [ ] 现场确认 `meta.frame/seq/timestamp` 哪个字段最稳定，必要时固定为唯一输入帧号。
- [ ] 现场确认 `uav_id/drone_id` 映射表，写入 `core/common.py::PLAN_EXPORT["uav_id_map"]`。
- [ ] 若后续需要请求式查询服务，再从 `decision/`、`station/` 的模块接口向外暴露，不在主循环里直接写协议处理。

## 接口实现备注

主链路默认：

```python
PLAN_EXPORT["publish_policy"] = "input_frame"
PLAN_EXPORT["output_frame_type"] = "decision_output_frame"
```

`decision_output_frame` 复用 `PlanFrame` 的 assignment 和 route，不重新分配、不重新规划。

## 历史命名备注

- `station/` 目录名保留，不做大范围迁移。
- `Device3FeedbackHandler` 和 `device3_*` 内部状态字段暂时保留，当前语义是“设备一执行侧回执/不可用状态”。
- `knowledge/05_设备2到设备3...` 和 `knowledge/06_设备2到设备3...` 文件名保留以避免引用断裂，正文已按新两设备语义更新。
