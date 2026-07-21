# 多无人机反无人机拦截仿真与接口联调项目

本项目是一个 Python/Pygame 仿真 + Redis/UDP/TCP Socket 联调程序。当前联调边界为两设备：

- **设备一**：郭老师设备/地面统筹设备，负责雷达数据清理、当前战场态势输出、真实无人机操控和指挥大屏显示。
- **设备二**：我方决策外挂，也就是本仓库主体，负责接收设备一态势输入，完成任务分配、路径规划、动作建议和一句话态势总结，并把决策帧回发设备一。

代码已按功能模块拆分到 `core/`、`perception/`、`decision/`、`station/`、`simulation/`、`integrations/`、`ui/`、`tools/`。其中 `station/` 是历史目录名，当前表示设备二决策输出模块。

## 核心模块

- `simulation/main.py`：主入口，维护 `InterceptionEnvironment`，串联态势输入、LLM 约束、任务分配、路径规划和决策输出。
- `core/common.py`：全局配置、枚举、实体状态字典、运动辅助函数；`PLAN_EXPORT` 配置设备二回发设备一的决策帧。
- `perception/`：设备一态势输入适配，支持 Redis/UDP/Fusion 数据源、坐标归一化和稳定目标 ID 关联。
- `decision/`：任务分配、重分配、局部避碰、LLM 任务约束和态势分析。
- `station/contracts.py`：把当前环境状态转换为完整 `PlanFrame`，包含 `situation_alert`、任务绑定和路径计划。
- `station/query.py`：从 `PlanFrame` 裁剪 `decision_output_frame`、飞控指令视图、状态快照和下一指令点。
- `station/exporter.py`：默认按设备一输入帧触发，通过 TCP JSON Lines 回发决策帧。
- `station/feedback.py`：处理设备一执行回执，失败/中止时释放任务并触发重分配。
- `integrations/`：历史显示和飞控实验适配，默认不作为新主链路。
- `ui/`：本地 Pygame 显示、语音和 LLM 看板。
- `tools/`：联调辅助脚本。

## 主链路

```text
设备一清理后的当前战场态势
  -> perception 输入归一化
  -> simulation/main.py 决策主循环
  -> decision 任务分配与路径规划
  -> station/contracts.py 生成完整 PlanFrame
  -> station/query.py 裁剪 decision_output_frame
  -> station/exporter.py 回发设备一
  -> 设备一操控无人机并在指挥大屏显示
```

## 决策输出配置

`core/common.py` 中的默认配置：

```python
PLAN_EXPORT = {
    "enabled": False,
    "transport": "tcp_json_lines",
    "publish_policy": "input_frame",
    "output_frame_type": "decision_output_frame",
    "socket_host": "127.0.0.1",
    "socket_port": 7001,
}
```

联调时把 `enabled` 设为 `True`，并将 `socket_host/socket_port` 改为设备一接收端地址。`publish_policy="input_frame"` 表示设备二每收到一组新的设备一输入态势帧，就回发一帧决策结果。

## 文档入口

进入仓库后优先阅读：

1. `AGENTS.md`
2. `knowledge/README.md`
3. `knowledge/00_项目总览.md`
4. `knowledge/03_地面站输出接口.md`
5. `knowledge/06_设备2到设备3雷达触发精简决策帧.md`

`05/06` 文件名保留旧“设备2到设备3”命名以避免引用断裂，正文已按当前设备一/设备二边界更新。
