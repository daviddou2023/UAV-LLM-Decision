import socket
import threading
import time
import struct
from protocol import Protocol
from drone_manager import DroneManager
import config


def get_configured_drone_links():
    """读取多机配置；若未提供 DRONE_LINKS，则回退为旧版单机配置。"""
    drone_links = getattr(config, "DRONE_LINKS", None)
    if isinstance(drone_links, dict) and drone_links:
        normalized = {}
        for drone_id, cfg in drone_links.items():
            if not isinstance(cfg, dict):
                continue
            normalized[int(drone_id)] = {
                "port": cfg["port"],
                "baud": cfg.get("baud", getattr(config, "BAUD_RATE", 57600)),
                "sys_id": cfg.get("sys_id", getattr(config, "SYS_ID", 1)),
                "comp_id": cfg.get("comp_id", getattr(config, "COMP_ID", 1)),
                "label": cfg.get("label", f"drone-{int(drone_id)}"),
            }
        if normalized:
            return normalized

    legacy_drone_id = int(getattr(config, "DRONE_ID", 1))
    return {
        legacy_drone_id: {
            "port": getattr(config, "SERIAL_PORT", "udpin:0.0.0.0:14554"),
            "baud": getattr(config, "BAUD_RATE", 57600),
            "sys_id": getattr(config, "SYS_ID", 1),
            "comp_id": getattr(config, "COMP_ID", 1),
            "label": f"drone-{legacy_drone_id}",
        }
    }


class ProtocolServer:
    def __init__(self, drone_links=None):
        self.drone_configs = drone_links if drone_links else get_configured_drone_links()
        self.drone_manager = DroneManager(self.drone_configs)
        self.running = True
        self.clients = []
        self.clients_lock = threading.Lock()
        self.pending_commands = {}
        self.last_broadcast_summary = {}
        self.last_broadcast_log_time = {}
        self.active_mission = {}

    def _get_handler(self, drone_id):
        return self.drone_manager.get(int(drone_id))
    def _send_packet(self, client, msg_type, drone_id, payload=b""):
        try:
            packet = Protocol.build_packet(
                msg_type,
                drone_id,
                payload,
            )
            client.sendall(packet)
        except Exception as exc:
            print(f"   ⚠️ 发送数据包失败: {exc}")
    def _send_mission_progress_to_client(
        self,
        client,
        drone_id,
        current_wp,
        total_wp,
        event_type,
    ):
        try:

            payload = struct.pack(
                "<HHB",
                int(current_wp),
                int(total_wp),
                int(event_type),
            )
            print(
                f"   📤 发送航点进度: "
                f"drone={drone_id}, "
                f"wp={current_wp}/{total_wp}, "
                f"event={event_type}"
            )
            self._send_packet(
                client,
                0x19,
                drone_id,
                payload,
            )

        except Exception as exc:
            print(f"   ⚠️ 发送航点进度失败: {exc}")
        
    def _send_ack(self, client, msg_type, drone_id, success, reason=""):
        ack_type = Protocol.get_ack_type(msg_type) if success else Protocol.get_fail_type(msg_type)

        try:
            payload = reason.encode("utf-8") if reason else b""
            packet = Protocol.build_packet(ack_type, drone_id, payload)
            client.sendall(packet)

            print(f"   📤 已发送反馈: 0x{ack_type:02X}, reason={reason}")

        except Exception as exc:
            print(f"   ⚠️ 发送反馈失败: {exc}")

    def _print_handler_summary(self, drone_id, handler):
        firmware_name = {
            1: "ArduPilot",
            2: "PX4",
            0: "Unknown"
        }.get(handler.firmware_type, "Unknown")

        cfg = self.drone_configs.get(drone_id, {})
        print(f"   无人机 {drone_id}:")
        print(f"      MAVLink: {cfg.get('port')}")
        print(f"      固件类型: {firmware_name}")
        print(f"      固件版本: {handler.firmware_version or '未知'}")
        print(f"      GOTO命令: {'MAV_CMD_DO_REPOSITION' if handler.goto_supports_speed else 'MAV_CMD_NAV_WAYPOINT'}")
        print("      航点任务: 支持 (Mission Protocol)")

    def start(self):
        """启动服务器"""
        results = self.drone_manager.connect_all()
        connected_count = 0
        for drone_id, connected in results.items():
            handler = self.drone_manager.get(drone_id)
            if connected:
                connected_count += 1
                if handler is not None:
                    self._print_handler_summary(drone_id, handler)
            else:
                cfg = self.drone_manager.get_meta(drone_id)
                print(f"   端口: {cfg.get('port')}")
                print("   状态: 连接失败")

        if connected_count == 0:
            print("❌ 没有任何无人机连接成功")
            return False

        print(f"\n✅ TCP服务器已启动")
        print(f"   监听地址: {config.TCP_SERVER_IP}:{config.TCP_SERVER_PORT}")
        print(f"   已配置无人机: {len(self.drone_manager.known_ids())}")
        print(f"   已连接成功: {connected_count}")
        print(f"   状态上报间隔: {config.STATUS_BROADCAST_INTERVAL * 1000}ms")
        print("\n等待客户端连接...")

        tcp_thread = threading.Thread(target=self._tcp_server_loop, daemon=True)
        tcp_thread.start()

        broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
        broadcast_thread.start()

        return True

    def _tcp_server_loop(self):
        """TCP服务器主循环"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((config.TCP_SERVER_IP, config.TCP_SERVER_PORT))
        server.listen(5)
        server.settimeout(1.0)

        while self.running:
            try:
                client, addr = server.accept()
                print(f"\n🔗 [新连接] 来自: {addr[0]}:{addr[1]}")

                with self.clients_lock:
                    self.clients.append(client)

                threading.Thread(
                    target=self._handle_client,
                    args=(client, addr),
                    daemon=True
                ).start()

            except socket.timeout:
                continue
            except Exception as exc:
                if self.running:
                    print(f"TCP服务器错误: {exc}")

        server.close()

    def _handle_client(self, client, addr):
        """处理客户端连接"""
        buffer = bytearray()

        while self.running:
            try:
                data = client.recv(4096)
                if not data:
                    break

                print(f"\n📥 [收到数据] 来自 {addr[0]}:{addr[1]}, {len(data)}字节")
                buffer.extend(data)

                while len(buffer) >= 8:
                    msg_type, drone_id, payload = Protocol.parse_packet(buffer)

                    if msg_type is None:
                        if len(buffer) > 1:
                            buffer.pop(0)
                        continue

                    print(f"📦 [解析成功] 消息类型: 0x{msg_type:02X}, DroneID: {drone_id}")
                    self._execute_command(msg_type, payload, client, drone_id, addr)

                    total_len = 8 + len(payload)
                    buffer = buffer[total_len:]

            except ConnectionResetError:
                break
            except Exception as exc:
                print(f"客户端处理错误: {exc}")
                break

        with self.clients_lock:
            if client in self.clients:
                self.clients.remove(client)

        client.close()
        print(f"\n🔌 [断开连接] {addr[0]}:{addr[1]}")

    def _execute_command(self, msg_type, payload, client, drone_id, addr):
        """执行命令 (异步版本，支持中断)"""
        handler = self._get_handler(drone_id)
        if handler is None:
            self._send_ack(client, msg_type, drone_id, False, f"未配置的无人机 ID: {drone_id}")
            return
        if not handler.connected:
            self._send_ack(client, msg_type, drone_id, False, f"无人机 {drone_id} 未连接")
            return

        # 1. 立即标记新指令开始，这会中断该无人机当前正在运行的其他指令
        cmd_id = handler.start_new_command()

        # 2. 在新线程中异步执行
        def cmd_thread_func():
            try:
                success = False
                reason = ""
                
                cmd_names = {
                    Protocol.MsgType.ARM: "ARM",
                    Protocol.MsgType.DISARM: "DISARM",
                    Protocol.MsgType.TAKEOFF: "TAKEOFF",
                    Protocol.MsgType.SET_MODE: "SET_MODE",
                    Protocol.MsgType.LAND: "LAND",
                    Protocol.MsgType.RTH: "RTH",
                    Protocol.MsgType.GOTO: "GOTO",
                    Protocol.MsgType.MISSION_WP: "MISSION_WP",
                }
                cmd_name = cmd_names.get(msg_type, f"UNKNOWN(0x{msg_type:02X})")
                print(f"\n🎯 [执行异步指令] {cmd_name} (ID:{cmd_id}) -> 无人机 {drone_id}")

                if msg_type == Protocol.MsgType.ARM:
                    success = handler.arm(cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.DISARM:
                    success = handler.disarm(cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.TAKEOFF and len(payload) >= 4:
                    alt = struct.unpack('<f', payload[:4])[0]
                    success = handler.takeoff(alt, cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.LAND:
                    success = handler.land(cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.RTH:
                    success = handler.rtl(cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.SET_MODE and len(payload) >= 1:
                    mode_id = payload[0]
                    mode_name = Protocol.MODE_MAP.get(mode_id, f"MODE_{mode_id}")
                    success = handler.set_mode(mode_name, cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.GOTO and len(payload) >= 16:
                    lat, lng, alt, speed = struct.unpack('<ffff', payload[:16])
                    success = handler.goto(lat, lng, alt, speed, cmd_id=cmd_id)
                elif msg_type == Protocol.MsgType.MISSION_WP:
                    mission_data = Protocol.parse_mission_payload(payload)
                    if mission_data:
                        handler.mission_progress_sender = (
                            lambda current_wp, total_wp, event_type, c=client, d=drone_id:
                                self._send_mission_progress_to_client(c, d, current_wp, total_wp, event_type)
                        )
                        success = handler.upload_mission(
                            mission_data['waypoints'],
                            global_speed=mission_data.get('global_speed', 0.0),
                            cmd_id=cmd_id
                        )
                
                # 检查是否是因为被取消而结束
                if handler.is_command_cancelled(cmd_id):
                    print(f"   ℹ️ 指令 {cmd_name} (ID:{cmd_id}) 已由于新指令而被放弃响应")
                    return

                if success:
                    print(f"   ✅ 指令 {cmd_name} (ID:{cmd_id}) 执行成功")
                    self._send_ack(client, msg_type, drone_id, True)
                else:
                    now = time.time()
                    fc_reason = None
                    if hasattr(handler, "last_arm_status_text") and handler.last_arm_status_text:
                        if now - handler.last_arm_status_time < 5.0:
                            fc_reason = handler.last_arm_status_text
                    if not fc_reason and hasattr(handler, "last_statustext") and handler.last_statustext:
                        if now - handler.last_statustext_time < 3.0:
                            fc_reason = handler.last_statustext

                    reason = f"飞控信息: {fc_reason}" if fc_reason else "执行失败或超时"
                    print(f"   ❌ 指令 {cmd_name} (ID:{cmd_id}) 失败: {reason}")
                    self._send_ack(client, msg_type, drone_id, False, reason)

            except Exception as exc:
                if not handler.is_command_cancelled(cmd_id):
                    print(f"   ❌ 指令异步执行异常: {exc}")
                    self._send_ack(client, msg_type, drone_id, False, str(exc))

        threading.Thread(target=cmd_thread_func, daemon=True).start()

    def _broadcast_loop(self):
        """状态广播循环"""
        error_count = 0
        while self.running:
            try:
                for drone_id, handler in self.drone_manager.items():
                    if not handler.connected:
                        continue

                    state = handler.update_state()

                    armed = bool(state.get('armed', False))
                    battery = float(state.get('battery', 100))
                    mode = str(state.get('mode', 'UNKNOWN'))
                    lat = float(state.get('lat', 0.0))
                    lng = float(state.get('lng', 0.0))
                    alt = float(state.get('alt_rel', 0.0))
                    groundspeed = float(state.get('groundspeed', 0.0))
                    verticalspeed = float(state.get('verticalspeed', 0.0))
                    satellites = int(state.get('satellites', 0))
                    hdop = float(state.get('hdop', 99.9))
                    flying = bool(state.get('flying', False))
                    has_compass = hdop > 0 and hdop < 2.0

                    status_payload = Protocol.build_status_payload(
                        armed=armed,
                        battery=battery,
                        mode=mode,
                        lat=lat,
                        lng=lng,
                        alt=alt,
                        groundspeed=groundspeed,
                        verticalspeed=verticalspeed,
                        satellites=satellites,
                        hdop=hdop,
                        flying=flying,
                        has_compass=has_compass,
                        battery_voltage=float(state.get('battery_voltage', 0.0)),
                        battery_current=float(state.get('battery_current', 0.0))
                    )

                    packet = Protocol.build_packet(
                        Protocol.MsgType.STATUS_REPORT,
                        drone_id,
                        status_payload
                    )

                    with self.clients_lock:
                        key_summary = f"mode={mode}, armed={armed}, flying={flying}, gps={satellites}"
                        now = time.time()
                        last_summary = self.last_broadcast_summary.get(drone_id)
                        last_log_time = self.last_broadcast_log_time.get(drone_id, 0.0)
                        if key_summary != last_summary or now - last_log_time >= 10.0:
                            print(
                                f"   [状态][无人机 {drone_id}] {key_summary} | "
                                f"位置: ({lat:.6f}, {lng:.6f}) | 高度: {alt:.1f}m | 速度: {groundspeed:.1f}m/s"
                            )
                            self.last_broadcast_summary[drone_id] = key_summary
                            self.last_broadcast_log_time[drone_id] = now

                        for client in list(self.clients):
                            try:
                                client.sendall(packet)
                            except Exception:
                                if client in self.clients:
                                    self.clients.remove(client)

                error_count = 0
                time.sleep(config.STATUS_BROADCAST_INTERVAL)

            except Exception as exc:
                if self.running:
                    error_count += 1
                    if error_count <= 3:
                        print(f"广播循环错误: {exc}")
                time.sleep(0.5)

    def stop(self):
        """停止服务器"""
        print("\n正在停止服务器...")
        self.running = False

        self.drone_manager.close_all()

        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except Exception:
                    pass
            self.clients.clear()

        print("服务器已停止")
