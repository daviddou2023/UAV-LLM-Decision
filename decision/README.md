# decision 设备2决策模块

职责：任务分配、路径规划/避碰、重分配、LLM 约束解析和态势分析。

主要文件：

- `cooperation.py`：撞击拦截任务分配。
- `deconfliction.py`：局部避碰、绕飞和航线合规规划。
- `reassignment.py`：稳定重分配策略。
- `llm_task_constraints.py`：自然语言任务约束解析。
- `llm_kit.py`：LLM 态势分析和对话封装。
- `pipeline.py`：设备2决策流程模块接口。
