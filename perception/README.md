# perception 雷达感知接入模块

职责：接收设备1/雷达/UDP/Redis/融合航迹数据，完成清洗、归一化、平滑和敌方稳定 ID 关联。

主要文件：

- `radar_feed.py`：Redis/融合航迹读取与归一化。
- `udp_gateway.py`：UDP 航迹输入输出。
- `enemy_association.py`：敌方目标稳定 ID 关联。
- `ingest.py`：雷达输入模块接口。
- `enemy_relay_to_local.py`、`friendlies_to_redis.py`：联调辅助转发。
