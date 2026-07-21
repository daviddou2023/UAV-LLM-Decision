"""雷达感知与航迹接入模块。"""

__all__ = ["RadarIngestPort", "RadarIngestService", "RadarTrackFrame"]


def __getattr__(name):
    if name in __all__:
        from . import ingest
        return getattr(ingest, name)
    raise AttributeError(name)
