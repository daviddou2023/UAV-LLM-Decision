"""
地理坐标适配层:
1. 固定参考原点的经纬度 <-> 米制坐标互转
2. 计算方式与 integrations/middle_layer.py 保持一致
3. 仅用于输入输出适配，不改变内部拦截逻辑
"""
import math


DEFAULT_GEO_ORIGIN_LAT = 34.2663
DEFAULT_GEO_ORIGIN_LON = 108.9549


class GeoReference:
    def __init__(self, origin_lat=DEFAULT_GEO_ORIGIN_LAT, origin_lon=DEFAULT_GEO_ORIGIN_LON):
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.a = 6378137.0
        self.f = 1.0 / 298.257223563
        self.e2 = 2.0 * self.f - self.f * self.f
        self._origin_lat_rad = math.radians(self.origin_lat)
        self._origin_lon_rad = math.radians(self.origin_lon)
        self._sin_origin_lat = math.sin(self._origin_lat_rad)
        self._n_center = self.a / math.sqrt(1.0 - self.e2 * self._sin_origin_lat ** 2)
        self._m_center = (
            self.a * (1.0 - self.e2)
            / (1.0 - self.e2 * self._sin_origin_lat ** 2) ** (3.0 / 2.0)
        )

    def lonlat_to_xy(self, lat, lon):
        lat_rad = math.radians(float(lat))
        lon_rad = math.radians(float(lon))
        east = (lon_rad - self._origin_lon_rad) * (self._n_center * math.cos(self._origin_lat_rad))
        north = (lat_rad - self._origin_lat_rad) * self._m_center
        return east, north

    def xy_to_lonlat(self, x, y):
        lat_rad = self._origin_lat_rad + (float(y) / max(self._m_center, 1e-9))
        lon_scale = max(self._n_center * math.cos(self._origin_lat_rad), 1e-9)
        lon_rad = self._origin_lon_rad + (float(x) / lon_scale)
        return math.degrees(lat_rad), math.degrees(lon_rad)
