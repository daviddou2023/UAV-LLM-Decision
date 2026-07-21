# main.py
import time
from server import ProtocolServer, get_configured_drone_links
import config


class DroneGroundStation:
    def __init__(self):
        self.running = True
        self.server = None
        self.drone_links = {}
        self.config_source = "manual"

    def _prompt_non_empty(self, prompt_text):
        while True:
            value = input(prompt_text).strip()
            if value:
                return value
            print("输入不能为空，请重新输入。")

    def _prompt_int(self, prompt_text, min_value=None, max_value=None, default=None):
        while True:
            raw = input(prompt_text).strip()
            if raw == "" and default is not None:
                return default
            try:
                value = int(raw)
            except ValueError:
                print("请输入整数。")
                continue

            if min_value is not None and value < min_value:
                print(f"输入值不能小于 {min_value}。")
                continue
            if max_value is not None and value > max_value:
                print(f"输入值不能大于 {max_value}。")
                continue
            return value

    def collect_drone_links_from_console(self):
        print("\n请输入无人机连接信息。")
        drone_count = self._prompt_int("无人机数量: ", min_value=1)

        drone_links = {}
        for index in range(1, drone_count + 1):
            print(f"\n--- 配置第 {index} 架无人机 ---")

            while True:
                drone_id = self._prompt_int("drone_id (0-255): ", min_value=0, max_value=255)
                if drone_id in drone_links:
                    print(f"drone_id={drone_id} 已存在，请输入不同的 drone_id。")
                    continue
                break

            port = self._prompt_non_empty("port (例如 udpin:0.0.0.0:14554): ")
            baud = self._prompt_int("baud (默认57600): ", min_value=1, default=57600)
            sys_id = self._prompt_int("sys_id: ", min_value=1, max_value=255)
            comp_id = self._prompt_int("comp_id (默认1): ", min_value=1, max_value=255, default=1)

            drone_links[drone_id] = {
                "port": port,
                "baud": baud,
                "sys_id": sys_id,
                "comp_id": comp_id,
                "label": f"uav-{drone_id}",
            }

        return drone_links

    def choose_drone_links(self):
        while True:
            choice = input("是否使用 config.py 默认配置? (Y/n): ").strip().lower()
            if choice in ("", "y", "yes"):
                self.config_source = "config.py"
                return get_configured_drone_links()
            if choice in ("n", "no"):
                self.config_source = "manual"
                return self.collect_drone_links_from_console()
            print("请输入 Y 或 n。")
    
    def start_server_mode(self):
        """启动TCP协议服务器"""
        print("\n启动TCP协议服务器...")
        self.server = ProtocolServer(drone_links=self.drone_links)
        return self.server.start()
    
    def run(self):
        """运行主程序"""
        self.drone_links = self.choose_drone_links()

        print("=" * 70)
        print("   TCP-MAVLink 飞行辅助控制系统")
        print("=" * 70)
        print(f"\n📡 服务器配置:")
        print(f"   TCP端口: {config.TCP_SERVER_PORT}")
        print(f"   配置来源: {self.config_source}")
        print(f"   已配置无人机数量: {len(self.drone_links)}")
        for drone_id, cfg in sorted(self.drone_links.items()):
            print(
                f"   - DroneID={drone_id}, MAVLink={cfg['port']}, "
                f"baud={cfg['baud']}, sys_id={cfg['sys_id']}, comp_id={cfg['comp_id']}"
            )
        print(f"\n📦 协议格式:")
        print("   0x55AA + MsgType + DroneID + Length + Payload + CRC16")
        print("\n支持的命令:")
        print("   0x01=解锁, 0x02=上锁, 0x03=起飞, 0x04=切换模式")
        print("   0x05=降落, 0x06=返航, 0x07=定点飞行，0x08=设置轨迹")
        print("-" * 70)
        
        if not self.start_server_mode():
            print("❌ 服务器启动失败")
            return
        
        try:
            # 主循环
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n⏹️ 收到中断信号...")
        finally:
            self.stop()
    
    def stop(self):
        """停止程序"""
        self.running = False
        if self.server:
            self.server.stop()
        print("程序已退出")


def main():
    gs = DroneGroundStation()
    gs.run()


if __name__ == '__main__':
    main()
