# UAV LLM 项目执行规范入口

最后通读时间：2026-07-14

本文件是进入本仓库时的第一份项目规范和记忆索引。修改代码前必须先读这里，再按任务需要阅读 `knowledge/` 下的相关文档。

## 必读顺序

1. `knowledge/README.md`：SR1 风格编号文档索引和阅读顺序。
2. `knowledge/00_项目总览.md`：设备一/设备二边界与主链路。
3. `knowledge/01_雷达数据输入接口.md`：设备一态势输入进入设备二后的处理接口。
4. `knowledge/02_设备2决策流程接口.md`：任务分配、路径规划、态势警报的流程接口。
5. `knowledge/03_地面站输出接口.md`：设备二回发设备一的完整帧、精简决策帧和飞控指令视图。
6. `knowledge/04_端到端联调流程.md`：从设备一态势输入到设备二决策回发的联调步骤。
7. `knowledge/overview.md`：项目整体结构、主流程、接口建议落点。
8. `knowledge/develop.md`：项目架构、主流程、关键状态、接口设计。
9. `knowledge/design-principles.md`：代码设计原则、SOLID 在本项目中的落地规则。
10. `knowledge/todo.md`：当前待办、接口状态、后续联调事项。
11. `knowledge/bugs.md`：已知风险、文档与实现不一致点、集成前必须验证的问题。
12. `knowledge/log-analysis.md`：运行日志标签、排查路径、设备二回发设备一调试方式。
13. `knowledge/project-summary.md`：项目总体说明。
14. `knowledge/device2-to-device3-interface.md`：旧名保留，修改设备二回发设备一决策协议时必须阅读。
15. `knowledge/interference-legacy-notes.md`：历史/外部干扰仿真方案，仅作参考。

## 项目快照

本仓库是一个多无人机反无人机仿真与接口联调项目。当前系统由两个设备组成：设备一（郭老师设备/地面统筹设备）负责接收雷达、清理过滤战场态势、真实操控无人机并在指挥大屏显示；设备二（本仓库）作为我方决策外挂，接收设备一态势输入，完成目标处理、任务分配、路径规划、单机动作建议和一句话态势总结，并把决策帧回发设备一。

核心模块：

- `simulation/main.py`：仿真环境、主循环、命令处理、LLM约束应用、任务和路径刷新。
- `core/common.py`：全局配置、枚举、无人机/敌方目标状态模型、运动辅助函数。
- `decision/cooperation.py`：撞击拦截模式下的任务分配逻辑，选择主拦截机和随动机。
- `decision/deconfliction.py`：局部避碰、绕飞和航线合规规划。
- `station/contracts.py`：把当前环境状态转换为完整 `PlanFrame`。
- `station/query.py`：从 `PlanFrame` 裁剪设备一主消费的 `decision_output_frame`。
- `station/exporter.py`：按设备一输入帧触发回发决策帧。
- `station/socket_client.py`：设备二到设备一决策帧的 TCP JSON Lines 客户端。
- `station/feedback.py`：处理设备一执行回执，并在失败时触发重分配。
- `decision/llm_kit.py`：LLM态势分析和对话封装。
- `ui/llm_dashboard.py`：LLM推理看板，不是业务接口。

## 功能模块目录

- `perception/`：设备一态势输入接口，薄封装现有 `TeacherDataFeed` / `FusionTrackFeed`。
- `decision/`：设备2任务分配、路径规划、态势警报流程接口，薄封装 `InterceptionEnvironment.step(dt)`。
- `station/`：历史目录名，当前作为设备二决策输出接口，复用 `PlanFrame` 裁剪精简决策帧、状态快照和飞控指令。

## 规划接口

当前运行链路以 `station/exporter.py` 生成完整 `PlanFrame`、再裁剪 `decision_output_frame` 回发设备一为准；地面站（station）旁路转接位于 `station/plan_bridge.py`，仅作历史兼容/调试参考。

后续若恢复“模型决策协同选机”或“按 UAV 查询下一个指令点”的独立接口，仍必须复用 `PlanFrameBuilder` 和已有环境状态，不要另造一套协议结构。

## 重要边界

- 未经明确需求，不要改 `integrations/middle_layer.py` / `integrations/redis_export.py` 中既有 Redis 显示接口。
- 设备二当前通过 TCP JSON Lines 回发 `decision_output_frame`；Redis 仅作为可选调试镜像。
- LLM 当前主要输出约束和建议，不是结构化的“直接选择 UAV 列表”对象。
- 撞击模式使用主拦截/随动角色；列阵/网阻模式使用多机编组映射。
- 允许按功能模块进行大范围重构，但必须保持数据结构和协议语义可测；不为兼容旧根目录文件名额外留壳。

## 工作规则

- 先读代码再改代码，优先沿用现有模式和辅助函数。
- 接口新增要小而可测。
- 编写代码时优先遵循 `ponytail`：先判断是否需要新增代码，再优先复用仓库现有实现、标准库和最小可行改动。
- `ponytail` 与 SOLID 配合使用：用最短路径完成真实需求，用单一职责、接口隔离和依赖倒置约束模块边界。
- 仓库可能已有未提交修改，不要回退无关改动。
- 修改设备二/设备一协议字段时，同步更新 `knowledge/device2-to-device3-interface.md`；如果仍有不一致，记录到 `knowledge/bugs.md`。
- 新增或修改代码注释时尽量使用中文；英文协议字段、类名、函数名和行业术语可保留英文。
- 注释只解释边界、意图、约束和复杂逻辑，不做逐行翻译。
