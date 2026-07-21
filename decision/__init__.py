"""设备2任务分配、路径规划与态势决策模块。"""

__all__ = ["DecisionPipelinePort", "DecisionPipelineService", "DecisionSnapshot", "DecisionTickResult"]


def __getattr__(name):
    if name in __all__:
        from . import pipeline
        return getattr(pipeline, name)
    raise AttributeError(name)
