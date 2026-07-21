import math
import threading
import time
import struct

from pymavlink import mavutil

import config
from mavlink_handler_base import FirmwareType


class MavlinkMissionMixin:
    def _reset_mission_protocol_state(self):
        with self.mission_protocol_lock:
            self.mission_request_cache.clear()

            for event in self.mission_request_events.values():
                event.set()
            self.mission_request_events.clear()

            self.mission_ack_cache = None
            if self.mission_ack_event is not None:
                self.mission_ack_event.set()
            self.mission_ack_event = None

            self.mission_count_cache = None
            if self.mission_count_event is not None:
                self.mission_count_event.set()
            self.mission_count_event = None

    def _register_mission_request_waiter(self, seq):
        event = threading.Event()

        with self.mission_protocol_lock:
            cached_msg = self.mission_request_cache.pop(seq, None)
            if cached_msg is not None:
                event.set()
                return event, cached_msg

            self.mission_request_events[seq] = event

        return event, None

    def _register_mission_ack_waiter(self):
        event = threading.Event()

        with self.mission_protocol_lock:
            cached_msg = self.mission_ack_cache
            if cached_msg is not None:
                self.mission_ack_cache = None
                event.set()
                return event, cached_msg

            self.mission_ack_event = event

        return event, None

    def _register_mission_count_waiter(self):
        event = threading.Event()

        with self.mission_protocol_lock:
            cached_msg = self.mission_count_cache
            if cached_msg is not None:
                self.mission_count_cache = None
                event.set()
                return event, cached_msg

            self.mission_count_event = event

        return event, None

    def _wait_for_mission_request(self, seq, timeout):
        event, cached_msg = self._register_mission_request_waiter(seq)
        if cached_msg is not None:
            return cached_msg

        if event.wait(timeout):
            with self.mission_protocol_lock:
                self.mission_request_events.pop(seq, None)
                return self.mission_request_cache.pop(seq, None)

        with self.mission_protocol_lock:
            self.mission_request_events.pop(seq, None)

        return None

    def _wait_for_mission_ack(self, timeout):
        event, cached_msg = self._register_mission_ack_waiter()
        if cached_msg is not None:
            return cached_msg

        if event.wait(timeout):
            with self.mission_protocol_lock:
                self.mission_ack_event = None
                ack_msg = self.mission_ack_cache
                self.mission_ack_cache = None
                return ack_msg

        with self.mission_protocol_lock:
            self.mission_ack_event = None

        return None

    def _wait_for_mission_count(self, timeout):
        event, cached_msg = self._register_mission_count_waiter()
        if cached_msg is not None:
            return cached_msg

        if event.wait(timeout):
            with self.mission_protocol_lock:
                self.mission_count_event = None
                count_msg = self.mission_count_cache
                self.mission_count_cache = None
                return count_msg

        with self.mission_protocol_lock:
            self.mission_count_event = None

        return None

    def _mission_seq_to_user_wp_index(self, seq):
        """
        将 MAVLink mission seq 转换为用户航点编号。

        当前 mission 结构：
            seq=0 HOME 占位
            seq=1 用户航点1
            seq=2 用户航点2
            ...

        返回：
            None -> HOME / 系统任务项 / 越界项
            1..N -> 用户真实航点编号
        """

        first_seq = getattr(self, "_first_real_wp_seq", 1)
        count = getattr(self, "_user_waypoint_count", 0)

        if seq < first_seq:
            return None

        index = seq - first_seq + 1

        if index < 1 or index > count:
            return None

        return index

    def upload_mission(self, waypoints, global_speed=0.0, takeoff_alt=None, cmd_id=None):
        """
        上传航点任务并开始执行，接近 Mission Planner 逻辑。
        """
        if cmd_id is None:
            cmd_id = self.start_new_command()

        print(f"\n🚁 开始上传航点任务: {len(waypoints)}个航点")

        if not waypoints:
            print("   ❌ 航点列表为空")
            return False

        if self.is_command_cancelled(cmd_id): return False

        if len(waypoints) + 2 > 255:
            print(f"   ❌ 航点数量超过限制: 用户航点={len(waypoints)}, 加HOME/TAKEOFF后可能超过255")
            return False

        for i, wp in enumerate(waypoints):
            if "lat" not in wp or "lng" not in wp or "alt" not in wp:
                print(f"   ❌ 航点{i + 1}缺少 lat/lng/alt")
                return False

            lat = wp["lat"]
            lng = wp["lng"]
            alt = wp["alt"]

            if not (-90 <= lat <= 90 and -180 <= lng <= 180):
                print(f"   ❌ 航点{i + 1}经纬度非法: lat={lat}, lng={lng}")
                return False

            if lat == 0 or lng == 0:
                print(f"   ❌ 航点{i + 1}坐标疑似无效: lat={lat}, lng={lng}")
                return False

            if alt <= 0:
                print(f"   ❌ 航点{i + 1}高度非法: alt={alt}")
                return False

        self.mission_complete = False

        armed = self.current_state.get("armed", False)
        flying = self.current_state.get("flying", False)
        relative_alt = self.current_state.get("relative_alt", 0.0)
        alt_rel = self.current_state.get("alt_rel", 0.0)

        current_alt = max(relative_alt, alt_rel)
        is_airborne = armed and (flying or current_alt > 1.5)

        print(
            f"   📌 当前状态: armed={armed}, flying={flying}, "
            f"relative_alt={relative_alt:.2f}m, alt_rel={alt_rel:.2f}m, "
            f"airborne={is_airborne}"
        )

        if not armed:
            print("   🔓 无人机未解锁，先执行解锁...")
            if not self.arm(cmd_id=cmd_id):
                print("   ❌ 解锁失败")
                return False
            time.sleep(0.5)
            armed = self.current_state.get("armed", False)

        if self.is_command_cancelled(cmd_id): return False

        if global_speed > 0:
            print(f"   📡 设置全局速度: {global_speed} m/s")
            # ArduPilot: p1=1 (Groundspeed), p2=speed, p3=-1 (no throttle change)
            speed_result = self._send_command_long_with_retry(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                (1, global_speed, -1, 0, 0, 0, 0),
                timeout=config.CMD_ACK_TIMEOUT,
                retries=2,
                label="速度设置",
            )
            if self._is_ack_rejected(speed_result):
                print("   ⚠️ 速度设置失败，将使用飞控默认速度")

        if self.is_command_cancelled(cmd_id): return False

        print("   📡 清除现有任务...")
        self._clear_mission()
        time.sleep(0.5)

        if self.is_command_cancelled(cmd_id): return False

        mission_items = []

        home_lat = self.current_state.get("lat")
        home_lng = self.current_state.get("lng")

        if home_lat is None or home_lng is None or home_lat == 0 or home_lng == 0:
            print("   ⚠️ 当前GPS无效，使用第一个航点作为HOME占位")
            home_lat = waypoints[0]["lat"]
            home_lng = waypoints[0]["lng"]

        home_item = self._create_mission_item(
            seq=0,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            lat=home_lat,
            lng=home_lng,
            alt=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        mission_items.append(home_item)

        print(f"   🏠 添加 HOME 占位: seq=0, ({home_lat:.7f}, {home_lng:.7f}, 0m)")

        if is_airborne:
            print("   🛫 已在空中：不添加 TAKEOFF")
            first_wp_seq = 1
            start_seq = 1
        else:
            if takeoff_alt is None:
                takeoff_alt = max(5.0, waypoints[0].get("alt", 10.0))

            print(f"   ✈️ 未起飞：添加 TAKEOFF seq=1，高度 {takeoff_alt}m")

        # ArduPilot TAKEOFF 使用 lat/lng=0 表示当前位置起飞
            takeoff_item = self._create_mission_item(
                seq=1,
                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                lat=0,
                lng=0,
                alt=takeoff_alt,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            mission_items.append(takeoff_item)

            first_wp_seq = 2
            start_seq = 1

        self._first_real_wp_seq = first_wp_seq
        self._user_waypoint_count = len(waypoints)

        for i, wp in enumerate(waypoints):
            yaw = wp.get("yaw")
            if yaw is None or (isinstance(yaw, float) and math.isnan(yaw)):
                yaw = 0

            seq = first_wp_seq + i

            item = self._create_mission_item(
                seq=seq,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                lat=wp["lat"],
                lng=wp["lng"],
                alt=wp["alt"],
                param1=wp.get("hold_time", 0.0),
                param2=wp.get("accept_radius", 5.0),
                param3=wp.get("pass_radius", 0.0),
                param4=yaw,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            mission_items.append(item)

            print(
                f"   航点{i + 1}: seq={seq}, "
                f"({wp['lat']:.6f}, {wp['lng']:.6f}, {wp['alt']}m), "
                f"停留={wp.get('hold_time', 0)}s, "
                f"半径={wp.get('accept_radius', 5)}m"
            )

        self._current_mission_count = len(mission_items)

        if self.firmware_type == FirmwareType.PX4:
            if not self._upload_mission_items(mission_items, cmd_id=cmd_id):
                print("   ❌ 任务上传失败")
                return False
        else:
            if not self._upload_compatible_mission_items(mission_items, cmd_id=cmd_id):
                print("   ❌ 任务上传失败")
                return False

        print(
            f"   ✅ 任务上传成功: "
            f"{len(waypoints)}个用户航点，共{len(mission_items)}个任务项"
        )

        time.sleep(0.5)

        if self.is_command_cancelled(cmd_id): return False

        if not self._verify_mission_uploaded(len(mission_items)):
            print("   ⚠️ 无法验证任务上传状态，继续执行...")

        if self.is_command_cancelled(cmd_id): return False

        print(f"   📡 设置当前任务项: seq={start_seq}")
        if not self._set_current_mission_item(start_seq):
            print("   ❌ 设置当前任务项失败")
            return False

        time.sleep(0.5)

        if self.is_command_cancelled(cmd_id): return False

        print("   🔄 切换到 AUTO 模式...")
        if not self.set_mode("AUTO", cmd_id=cmd_id):
            print("   ❌ AUTO 模式切换失败")
            return False

        print("   ✅ 航点任务已启动")
        return True

    def _upload_compatible_mission_items(self, mission_items, cmd_id=None):
        """兼容固件，如 ArduPilot，上传任务项的方式。"""
        if not mission_items:
            return False

        if cmd_id is None: cmd_id = self.current_cmd_id

        count = len(mission_items)
        target_system = self.master.target_system
        target_component = self.master.target_component

        print(f"   📡 开始上传 {count} 个任务项 (兼容模式)...")
        
        # 增加整体重试，防止 MISSION_COUNT 丢失导致后续 seq=0 超时
        for attempt in range(1, 4):
            if self.is_command_cancelled(cmd_id): return False
            self._reset_mission_protocol_state()
            try:
                # 确保在发送新任务前，清空可能存在的残留请求
                self._poll_messages() 
                
                with self.io_lock:
                    # 显式指定 mission_type=0 (MAV_MISSION_TYPE_MISSION)
                    self.master.mav.mission_count_send(
                        target_system,
                        target_component,
                        count,
                        0 
                    )
                print(f"   📤 已发送 MISSION_COUNT: {count} (attempt={attempt}/3)")
            except Exception as exc:
                print(f"   ❌ 发送 MISSION_COUNT 失败: {exc}")
                return False

            # 等待 seq=0 的请求，作为 MISSION_COUNT 已被接收的标志
            # 稍微延长第一次请求的等待时间
            msg = self._wait_for_mission_request(0, timeout=3.0)
            if msg is not None:
                print(f"   📥 收到第一个请求: seq=0 (来自 {msg.get_srcSystem()}:{msg.get_srcComponent()})")
                break
            
            if attempt < 3:
                print(f"   ⚠️ 等待 MISSION_REQUEST seq=0 超时，准备重试 MISSION_COUNT...")
                time.sleep(1.0)
        else:
            print("   ❌ 多次尝试发送 MISSION_COUNT 后仍未收到 seq=0 请求")
            return False

        uploaded = 0
        request_timeout = 8.0 

        for seq in range(count):
            if self.is_command_cancelled(cmd_id):
                print("   🛑 任务上传已被新指令中断")
                return False

            try:
                if seq == 0:
                    msg_for_seq = msg
                else:
                    msg_for_seq = self._wait_for_mission_request(seq, request_timeout)
                
                if msg_for_seq is None:
                    # 如果中间断了，尝试最后看一下有没有 ACK（有时候飞控其实收全了但请求包丢了）
                    print(f"   ❌ 等待 MISSION_REQUEST 超时: seq={seq}")
                    break

                item = mission_items[seq]
                with self.io_lock:
                    self.master.mav.send(item)

                uploaded += 1
                if uploaded % 5 == 0 or uploaded == count:
                    print(f"   📤 已发送 {uploaded}/{count} 个任务项")

            except Exception as exc:
                print(f"   ⚠️ 发送任务项异常: {exc}")
                return False

        # 如果发送的数量不足，直接判定失败
        if uploaded < count:
            return False

        print("   ⏳ 等待 MISSION_ACK...")
        ack = self._wait_for_mission_ack(5.0)
        if ack is None:
            print("   ⚠️ 等待 MISSION_ACK 超时")
            return False

        if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("   ✅ 任务上传完成，飞控已确认")
            return True

        print(f"   ❌ 任务上传被拒绝: type={ack.type}")
        return False
    
    def _send_mission_progress(self, current_wp, total_wp, event_type=0):
        """
        向 GUI 发送航点任务进度。

        event_type:
            0 = 切换到当前航点
            1 = 航点已到达
            2 = 全部任务完成
        """
        try:
            sender = getattr(
                self,
                "mission_progress_sender",
                None,
            )

            if sender is None:
                return

            sender(
                int(current_wp),
                int(total_wp),
                int(event_type),
            )

        except Exception as exc:
            print(f"   ⚠️ 发送航点进度失败: {exc}")
    def _clear_mission(self):
        """清除飞控中的任务。"""
        try:
            with self.io_lock:
                self.master.mav.mission_clear_all_send(
                    self.master.target_system,
                    self.master.target_component,
                )

            print("   📡 已发送清除任务命令")
            time.sleep(0.5)
            return True

        except Exception as exc:
            print(f"   ⚠️ 清除任务失败: {exc}")
            return False

    def _create_mission_item(
        self,
        seq,
        command,
        lat,
        lng,
        alt,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        frame=None,
    ):
        """创建 MISSION_ITEM_INT 任务项。"""
        if frame is None:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

        if isinstance(param4, float) and math.isnan(param4):
            param4 = 0

        item = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=self.master.target_system,
            target_component=self.master.target_component,
            seq=seq,
            frame=frame,
            command=command,
            current=0,
            autocontinue=1,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            x=int(lat * 1e7),
            y=int(lng * 1e7),
            z=alt,
        )

        return item

    def _upload_mission_items(self, mission_items, cmd_id=None):
        """上传任务项到飞控。"""
        if not mission_items:
            return False

        if cmd_id is None: cmd_id = self.current_cmd_id

        count = len(mission_items)
        target_system = self.master.target_system
        target_component = self.master.target_component

        print(f"   📡 开始上传 {count} 个任务项...")
        
        # 增加整体重试，防止 MISSION_COUNT 丢失导致后续 seq=0 超时
        for attempt in range(1, 4):
            if self.is_command_cancelled(cmd_id): return False
            self._reset_mission_protocol_state()
            try:
                self._poll_messages()

                with self.io_lock:
                    self.master.mav.mission_count_send(
                        target_system,
                        target_component,
                        count,
                        0
                    )
                print(f"   📤 已发送 MISSION_COUNT: {count} (attempt={attempt}/3)")
            except Exception as exc:
                print(f"   ❌ 发送 MISSION_COUNT 失败: {exc}")
                return False

            # 等待 seq=0 的请求，作为 MISSION_COUNT 已被接收的标志
            msg = self._wait_for_mission_request(0, timeout=3.0)
            if msg is not None:
                print(f"   📥 收到第一个请求: seq=0 (来自 {msg.get_srcSystem()}:{msg.get_srcComponent()})")
                break
            
            if attempt < 3:
                print(f"   ⚠️ 等待 MISSION_REQUEST seq=0 超时，准备重试 MISSION_COUNT...")
                time.sleep(1.0)
        else:
            print("   ❌ 多次尝试发送 MISSION_COUNT 后仍未收到 seq=0 请求")
            return False

        uploaded = 0
        request_timeout = 8.0 

        for seq in range(count):
            if self.is_command_cancelled(cmd_id):
                print("   🛑 任务上传已被新指令中断")
                return False

            try:
                if seq == 0:
                    msg_for_seq = msg
                else:
                    msg_for_seq = self._wait_for_mission_request(seq, request_timeout)

                if msg_for_seq is None:
                    print(f"   ❌ 等待 MISSION_REQUEST 超时: seq={seq}")
                    break

                item = mission_items[seq]
                with self.io_lock:
                    self.master.mav.send(item)

                uploaded += 1
                if uploaded % 5 == 0 or uploaded == count:
                    print(f"   📤 已发送 {uploaded}/{count} 个任务项")

            except Exception as exc:
                print(f"   ⚠️ 发送任务项异常: {exc}")
                return False

        if uploaded < count:
            return False

        print("   ⏳ 等待 MISSION_ACK...")
        ack = self._wait_for_mission_ack(5.0)
        if ack is None:
            print("   ⚠️ 等待 MISSION_ACK 超时")
            return False

        if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("   ✅ 任务上传完成，飞控已确认")
            return True

        print(f"   ❌ 任务上传被拒绝: type={ack.type}")
        return False

    def _set_current_mission_item(self, seq):
        """设置当前任务项序号。"""
        try:
            with self.io_lock:
                self.master.mav.mission_set_current_send(
                    self.master.target_system,
                    self.master.target_component,
                    seq,
                )

            print(f"   📡 已设置当前任务项: seq={seq}")
            return True

        except Exception as exc:
            print(f"   ⚠️ 设置当前任务项失败: {exc}")
            return False

    def _verify_mission_uploaded(self, expected_count):
        """验证任务是否已上传成功。"""
        try:
            with self.mission_protocol_lock:
                self.mission_count_cache = None
                if self.mission_count_event is not None:
                    self.mission_count_event.set()
                self.mission_count_event = None

            with self.io_lock:
                self.master.mav.mission_request_list_send(
                    self.master.target_system,
                    self.master.target_component,
                )

            msg = self._wait_for_mission_count(3.0)

            if msg and msg.count > 0:
                print(f"   📋 飞控确认: 共 {msg.count} 个任务项 (期望 {expected_count})")
                return msg.count == expected_count

            print("   ⚠️ 未收到任务计数确认")
            return False

        except Exception as exc:
            print(f"   ⚠️ 验证任务上传失败: {exc}")
            return False