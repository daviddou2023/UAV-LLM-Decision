# drone_manager.py
from mavlink_handler import MavlinkHandler


class DroneManager:
    def __init__(self, drone_links: dict[int, dict]):
        self.handlers: dict[int, MavlinkHandler] = {}
        self.meta: dict[int, dict] = {}

        for drone_id, cfg in drone_links.items():
            self.handlers[drone_id] = MavlinkHandler(
                port=cfg["port"],
                baud=cfg.get("baud", 57600),
                sys_id=cfg.get("sys_id", 1),
                comp_id=cfg.get("comp_id", 1),
            )
            self.meta[drone_id] = dict(cfg)

    def connect_all(self) -> dict[int, bool]:
        results: dict[int, bool] = {}
        for drone_id, handler in self.handlers.items():
            try:
                results[drone_id] = bool(handler.connect())
            except Exception as exc:
                print(f"❌ 无人机 {drone_id} 连接异常: {exc}")
                results[drone_id] = False
        return results

    def get(self, drone_id: int) -> MavlinkHandler | None:
        return self.handlers.get(drone_id)

    def items(self):
        return self.handlers.items()

    def known_ids(self) -> list[int]:
        return sorted(self.handlers.keys())

    def get_meta(self, drone_id: int) -> dict:
        return self.meta.get(drone_id, {})

    def close_all(self) -> None:
        for drone_id, handler in self.handlers.items():
            try:
                handler.close()
            except Exception as exc:
                print(f"⚠️ 无人机 {drone_id} 关闭异常: {exc}")