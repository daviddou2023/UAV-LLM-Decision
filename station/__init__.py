"""设备2回发设备一的决策输出模块。"""

__all__ = ["StationOutputFrame", "StationOutputPort", "StationOutputService"]


def __getattr__(name):
    if name in __all__:
        from . import output
        return getattr(output, name)
    raise AttributeError(name)
