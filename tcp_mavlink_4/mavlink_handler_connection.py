import math
import threading
import time

from pymavlink import mavutil

from mavlink_handler_base import FirmwareType


class MavlinkConnectionMixin:
    def _get_expected_flight_controller_ids(self):
        expected_system = int(self.sys_id) if self.sys_id is not None else None
        expected_component = int(self.comp_id) if self.comp_id is not None else None
        return expected_system, expected_component

    def _matches_expected_flight_controller_source(self, msg):
        expected_system, expected_component = self._get_expected_flight_controller_ids()
        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()

        if expected_system not in (None, 0) and src_system != expected_system:
            return False
        if expected_component not in (None, 0) and src_component != expected_component:
            return False
        return True

    def _wait_for_configured_heartbeat(self, timeout=10.0):
        deadline = time.time() + timeout
        mismatch_log_count = 0

        while time.time() < deadline:
            remaining = max(0.1, min(1.0, deadline - time.time()))
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is None:
                continue

            if not self._is_flight_controller_heartbeat(msg):
                continue

            if not self._matches_expected_flight_controller_source(msg):
                if mismatch_log_count < 5:
                    expected_system, expected_component = self._get_expected_flight_controller_ids()
                    print(
                        "   ⚠️ 忽略非目标飞控 HEARTBEAT: "
                        f"expected={expected_system}:{expected_component}, "
                        f"received={msg.get_srcSystem()}:{msg.get_srcComponent()}"
                    )
                    mismatch_log_count += 1
                continue

            return msg

        return None

    def connect(self):
        """连接到飞控"""
        try:
            self.master = mavutil.mavlink_connection(
                self.port,
                baud=self.baud,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )
            print("⏳ 等待飞控心跳包...")
            heartbeat_msg = self._wait_for_configured_heartbeat(timeout=10.0)
            if heartbeat_msg is None:
                expected_system, expected_component = self._get_expected_flight_controller_ids()
                raise TimeoutError(
                    "未在指定端口上收到目标飞控心跳 "
                    f"(expected sysid={expected_system}, compid={expected_component}, port={self.port})"
                )

            self.flight_controller_ids = (
                heartbeat_msg.get_srcSystem(),
                heartbeat_msg.get_srcComponent(),
            )
            if self.master is not None:
                self.master.target_system = self.flight_controller_ids[0]
                self.master.target_component = self.flight_controller_ids[1]

            self.connected = True
            self._ensure_target_ids()
            print("✅ 飞控连接成功")
            print(f"   已绑定飞控来源: {self.flight_controller_ids[0]}:{self.flight_controller_ids[1]}")
            print(f"   target_system: {self.master.target_system}")
            print(f"   target_component: {self.master.target_component}")
            print("   source_system: 255")
            print(f"   source_component: {mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER}")

            self._start_rx_loop()
            time.sleep(0.2)
            self._start_gcs_heartbeat()

            self.detect_and_adapter()
            self.request_telemetry_streams()
            self._mode_mapping = self.master.mode_mapping()
            if self._mode_mapping:
                print(f"   支持的模式: {len(self._mode_mapping)}个")
            self.update_state()
            return True
        except Exception as exc:
            print(f"❌ 飞控连接失败: {exc}")
            return False

    def _ensure_target_ids(self):
        """修正无效的目标 system/component ID。"""
        if self.master is None:
            return

        if self.flight_controller_ids is not None:
            fc_system, fc_component = self.flight_controller_ids
            if getattr(self.master, "target_system", 0) != fc_system:
                self.master.target_system = fc_system
                print(f"   🎯 根据 HEARTBEAT 设置 target_system={fc_system}")
            if getattr(self.master, "target_component", 0) != fc_component:
                self.master.target_component = fc_component
                print(f"   🎯 根据 HEARTBEAT 设置 target_component={fc_component}")
            return

        fallback_system = int(self.sys_id) if self.sys_id is not None else 1
        fallback_component = int(self.comp_id) if self.comp_id is not None else 1

        if getattr(self.master, "target_system", 0) in (None, 0):
            self.master.target_system = fallback_system
            print(f"   ⚠️ 检测到无效 target_system，回退使用配置 SYS_ID={fallback_system}")

        if getattr(self.master, "target_component", 0) in (None, 0):
            self.master.target_component = fallback_component
            print(f"   ⚠️ 检测到无效 target_component，回退使用配置 COMP_ID={fallback_component}")

    def _start_gcs_heartbeat(self):
        if self.master is None or self.heartbeat_running:
            return

        self.heartbeat_running = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        print("💓 已启动 GCS HEARTBEAT 发送线程")

    def _heartbeat_loop(self):
        while self.connected and self.heartbeat_running and self.master is not None:
            try:
                with self.io_lock:
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
            except Exception as exc:
                print(f"   ⚠️ GCS HEARTBEAT 发送异常: {exc}")
            time.sleep(1.0)

    def _start_rx_loop(self):
        if self.master is None or self.rx_running:
            return

        self.rx_running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        print("📥 已启动 MAVLink 接收线程")

    def _rx_loop(self):
        idle_sleep = 0.002
        busy_yield = 0.0001

        while self.connected and self.rx_running and self.master is not None:
            try:
                processed = self._poll_messages(max_batch=100)
                if processed == 0:
                    time.sleep(idle_sleep)
                else:
                    time.sleep(busy_yield)
            except Exception as exc:
                print(f"   ⚠️ MAVLink 接收线程异常: {exc}")
                time.sleep(0.01)

    def _is_flight_controller_heartbeat(self, msg):
        autopilot = getattr(msg, "autopilot", None)
        vehicle_type = getattr(msg, "type", None)
        if vehicle_type == getattr(mavutil.mavlink, "MAV_TYPE_GCS", None):
            return False
        if autopilot == getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", None):
            return False
        if autopilot in (
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavutil.mavlink.MAV_AUTOPILOT_PX4,
        ):
            expected_system, _expected_component = self._get_expected_flight_controller_ids()
            if expected_system not in (None, 0):
                return self._matches_expected_flight_controller_source(msg)
            return True
        if self.flight_controller_ids:
            return self.flight_controller_ids == (msg.get_srcSystem(), msg.get_srcComponent())
        return False

    def _matches_flight_controller_source(self, msg):
        if not self._matches_expected_flight_controller_source(msg):
            return False
        if self.flight_controller_ids is None:
            return True
        return (
            msg.get_srcSystem() == self.flight_controller_ids[0]
            and msg.get_srcComponent() == self.flight_controller_ids[1]
        )

    def _get_command_target_ids(self, target_system=None, target_component=None):
        if target_system is None:
            target_system = getattr(self.master, "target_system", 0) if self.master else 0
        if target_component is None:
            target_component = getattr(self.master, "target_component", 0) if self.master else 0

        if not target_system and self.flight_controller_ids:
            target_system = self.flight_controller_ids[0]
        if not target_component and self.flight_controller_ids:
            target_component = self.flight_controller_ids[1]

        target_system = int(target_system) if target_system is not None else 0
        target_component = int(target_component) if target_component is not None else 0
        return target_system, target_component

    def _update_state_from_message(self, msg):
        msg_type = msg.get_type()
        src_system = msg.get_srcSystem()
        src_component = msg.get_srcComponent()

        if msg_type == "HEARTBEAT":
            if not self._is_flight_controller_heartbeat(msg):
                return

            src_ids = (src_system, src_component)
            if self.flight_controller_ids != src_ids:
                self.flight_controller_ids = src_ids
                if self.master is not None:
                    self.master.target_system, self.master.target_component = src_ids
                print(f"   🎯 使用飞控来源: sysid={src_ids[0]}, compid={src_ids[1]}")

            self.last_fc_heartbeat_msg = msg
            self.last_fc_heartbeat_time = time.time()

            previous_armed = self.current_state["armed"]
            self.current_state["armed"] = (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) > 0
            if self.current_state["armed"]:
                self.last_armed_evidence_time = time.time()
                self.last_armed_evidence_source = f"HEARTBEAT {msg.get_srcSystem()}:{msg.get_srcComponent()}"

            mode_name = self._get_current_mode_from_heartbeat(msg)
            if mode_name:
                self.current_state["mode"] = mode_name

            self.current_state["flying"] = (
                self.current_state["armed"]
                and self.current_state["mode"] not in ["LAND", "RTL"]
                and self.current_state["alt_rel"] > 1.0
            )
            self.last_heartbeat = time.time()

            now = time.time()
            if self.current_state["armed"] != previous_armed or now - self.last_heartbeat_debug_time >= 30.0:
                print(
                    "   [HEARTBEAT] "
                    f"src={msg.get_srcSystem()}:{msg.get_srcComponent()} "
                    f"type={getattr(msg, 'type', 'N/A')} "
                    f"autopilot={getattr(msg, 'autopilot', 'N/A')} "
                    f"base_mode=0x{msg.base_mode:02X} "
                    f"custom_mode={getattr(msg, 'custom_mode', 'N/A')} "
                    f"armed={self.current_state['armed']} "
                    f"mode={self.current_state['mode']}"
                )
                self.last_heartbeat_debug_time = now
            return

        if not self._matches_flight_controller_source(msg):
            return

        if msg_type == "MISSION_ITEM_REACHED":
            seq = getattr(msg, "seq", 0)
            now = time.time()
            wp_index = self._mission_seq_to_user_wp_index(seq)
            if wp_index is None:
                return

           
            if (
                not hasattr(self, "_last_mission_reached_time")
                or now - self._last_mission_reached_time >= 1.0
            ):
                print(f"   🎯 [任务完成] 航点 {wp_index} 已到达")
                self._send_mission_progress(
                    current_wp=wp_index,
                    total_wp=self._user_waypoint_count,
                    event_type=1,
               )
                self._last_mission_reached_time = now

            if wp_index >= self._user_waypoint_count:
                self.mission_complete = True
                print("   ✅ 航线任务全部完成！")
                self._send_mission_progress(
                current_wp=wp_index,
                total_wp=self._user_waypoint_count,
                event_type=2,
        )
            return

        if msg_type == "MISSION_CURRENT":
            seq = getattr(msg, "seq", 0)
            wp_index = self._mission_seq_to_user_wp_index(seq)
            if wp_index is None:
                self._last_mission_current_seq = seq
                return
            if hasattr(self, "_current_mission_count") and self._current_mission_count and self._current_mission_count > 0:
                if seq != self._last_mission_current_seq and seq > 0:
                    print(f"   📍 [任务] 切换到航点 {wp_index}")
                    self._send_mission_progress(
                        current_wp=wp_index,
                        total_wp=self._user_waypoint_count,
                        event_type=0,
                    )
                self._last_mission_current_seq = seq
            return

        if msg_type == "GLOBAL_POSITION_INT":
            self.current_state["lat"] = msg.lat / 1e7
            self.current_state["lng"] = msg.lon / 1e7
            self.current_state["alt_abs"] = msg.alt / 1000.0
            self.current_state["alt_rel"] = msg.relative_alt / 1000.0

            if self.current_state["home_alt_abs"] is None:
                self.current_state["home_alt_abs"] = self.current_state["alt_abs"]

            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            self.current_state["groundspeed"] = math.sqrt(vx * vx + vy * vy)
            self.current_state["verticalspeed"] = msg.vz / 100.0

        elif msg_type == "HOME_POSITION":
            self.current_state["home_lat"] = msg.latitude / 1e7
            self.current_state["home_lng"] = msg.longitude / 1e7
            self.current_state["home_alt"] = msg.altitude / 1000.0
            self.current_state["home_set"] = True
            print(f"   🏠 收到飞控 Home 点更新: ({self.current_state['home_lat']:.7f}, {self.current_state['home_lng']:.7f})")

        elif msg_type == "GPS_RAW_INT":
            self.current_state["satellites"] = msg.satellites_visible
            self.current_state["hdop"] = msg.eph / 100.0
            self.current_state["gps_fix_type"] = getattr(msg, "fix_type", 0)

        elif msg_type == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                self.current_state["battery"] = msg.battery_remaining
            self.current_state["battery_voltage"] = msg.voltage_battery / 1000.0  # mV -> V
            self.current_state["battery_current"] = msg.current_battery / 100.0   # 10mA -> A
            
        elif msg_type == "BATTERY_STATUS":
            # 优先使用更详细的电池消息
            if hasattr(msg, 'voltages'):
                self.current_state["battery_voltage"] = msg.voltages[0] / 1000.0
            if hasattr(msg, 'current_battery'):
                self.current_state["battery_current"] = msg.current_battery / 100.0
            if hasattr(msg, 'battery_remaining'):
                self.current_state["battery"] = msg.battery_remaining
                
        elif msg_type == "STATUSTEXT":
            raw_text = getattr(msg, "text", b"")
            if isinstance(raw_text, bytes):
                text = raw_text.decode("utf-8", errors="ignore").strip("\x00").strip()
            else:
                text = str(raw_text).strip("\x00").strip()

            if text:
                now = time.time()
                text_lower = text.lower()

                # 记录所有状态文本及其时间
                self.last_statustext = text
                self.last_statustext_time = now

                # 特别识别“拒绝”或“错误”类信息
                is_rejection = any(k in text_lower for k in [
                    "denied", "reject", "refuse", "fail", "invalid", "not supported",
                    "prearm:", "arm:", "error", "missing"
                ])
                
                # 忽略一些常见的周期性提示/警告，避免它们覆盖真正的错误原因
                is_generic_warning = any(k in text_lower for k in [
                    "sched_loop_rate", "slow loop", "ekf2", "ekf3", "vibration", 
                    "ardu", "px4", "heartbeat", "initiali"
                ])

                if is_rejection or not is_generic_warning:
                    self.last_arm_status_text = text
                    self.last_arm_status_time = now
                    # 如果是拒绝信息，打印出来方便调试
                    if is_rejection:
                        print(f"   ⚠️ [飞控拒绝/错误] {text}")

                armed_keywords = (
                    "throttle armed",
                    "armed by",
                    "arming motors",
                    "motors armed",
                    "armed",
                )
                disarmed_keywords = (
                    "throttle disarmed",
                    "disarming motors",
                    "motors disarmed",
                )

                if any(keyword in text_lower for keyword in armed_keywords) and not (
                    text_lower.startswith("prearm:")
                    or "disarm" in text_lower
                    or "arming checks" in text_lower
                ):
                    if not self.current_state["armed"]:
                        self.current_state["armed"] = True
                        self.current_state["flying"] = self.current_state["alt_rel"] > 1.0
                        print("   🔓 根据 STATUSTEXT 更新 armed=True")
                    self.last_armed_evidence_time = now
                    self.last_armed_evidence_source = f"STATUSTEXT: {text}"
                elif any(keyword in text_lower for keyword in disarmed_keywords) or text_lower.startswith("disarm:"):
                    if self.current_state["armed"]:
                        self.current_state["armed"] = False
                        self.current_state["flying"] = False
                        print("   🔒 根据 STATUSTEXT 更新 armed=False")

                # 打印仍然限频，避免刷屏
                if (
                     not hasattr(self, "_last_statustext_print_text")
                    or text != self._last_statustext_print_text
                    or now - getattr(self, "_last_statustext_print_time", 0.0) >= 2.0
                ):
                    severity = getattr(msg, "severity", "N/A")
                    # 如果是普通警告且不是拒绝信息，则不作为重点输出
                    if not is_generic_warning or is_rejection:
                        print(f"   [STATUSTEXT] severity={severity} text={text}")
                    self._last_statustext_print_text = text
                    self._last_statustext_print_time = now

    def _process_incoming_message(self, msg):
        msg_type = msg.get_type()
        now = time.time()

        if msg_type == "COMMAND_ACK":
            command_id = msg.command if hasattr(msg, "command") else None
            if command_id is not None:
                src_system = msg.get_srcSystem()
                src_component = msg.get_srcComponent()
                ack_key = self._make_command_ack_key(
                    command_id,
                    src_system,
                    src_component,
                )
                result = getattr(msg, "result", None)
                self.command_ack_cache[ack_key] = (result, now, msg)

                # 1. 精确匹配事件
                event_data = self.command_ack_events.get(ack_key)
                if event_data:
                    event_data["result"] = result
                    event_data["msg"] = msg
                    event_data["event"].set()

                # 2. 模糊匹配事件 (如果有人在等待 compid=0)
                fuzzy_key = self._make_command_ack_key(command_id, src_system, 0)
                fuzzy_event_data = self.command_ack_events.get(fuzzy_key)
                if fuzzy_event_data:
                    fuzzy_event_data["result"] = result
                    fuzzy_event_data["msg"] = msg
                    fuzzy_event_data["event"].set()
            return

        if msg_type in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = getattr(msg, "seq", None)
            if seq is not None:
                with self.mission_protocol_lock:
                    self.mission_request_cache[int(seq)] = msg
                    event = self.mission_request_events.get(int(seq))
                    if event is not None:
                        event.set()
            return

        if msg_type == "MISSION_ACK":
            with self.mission_protocol_lock:
                self.mission_ack_cache = msg
                if self.mission_ack_event is not None:
                    self.mission_ack_event.set()
            return

        if msg_type == "MISSION_COUNT":
            with self.mission_protocol_lock:
                self.mission_count_cache = msg
                if self.mission_count_event is not None:
                    self.mission_count_event.set()
            return

        if msg_type == "PARAM_VALUE":
            param_name = self._normalize_param_id(getattr(msg, "param_id", ""))
            if param_name:
                self.param_value_cache[param_name] = (msg, now)
            return

        if msg_type == "AUTOPILOT_VERSION":
            self.autopilot_version_msg = msg
            self.autopilot_version_time = now
            return

        self._update_state_from_message(msg)

    def _poll_messages(self, max_batch=100):
        count = 0

        # 使用锁保护接收过程，防止多线程竞争导致丢包
        with self.io_lock:
            if self.master is None:
                return 0
                
            while True:
                try:
                    msg = self.master.recv_match(blocking=False)
                    if not msg:
                        break

                    self._process_incoming_message(msg)
                    count += 1

                    if count >= max_batch:
                        break
                except Exception as exc:
                    print(f"   ⚠️ _poll_messages 异常: {exc}")
                    break

        return count

    def _request_message_interval(self, requests):
        for name, message_id, interval_us in requests:
            try:
                with self.io_lock:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        message_id,
                        interval_us,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                print(f"   - 已请求 {name}: {interval_us / 1_000_000:.1f}s/次")
            except Exception as exc:
                print(f"   - 请求 {name} 失败: {exc}")

    def request_telemetry_streams(self):
        """连接成功后主动申请关键遥测流。"""
        if self.master is None:
            return

        print("📡 请求关键遥测流...")

        interval_requests = [
            ("HEARTBEAT", mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 200_000),
            ("SYS_STATUS", mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500_000),
            ("GPS_RAW_INT", mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 500_000),
            ("GLOBAL_POSITION_INT", mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000),
            ("HOME_POSITION", mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 2_000_000), # 每2秒一次即可
        ]
        self._request_message_interval(interval_requests)

        if self.firmware_type == FirmwareType.PX4:
            px4_interval_requests = [
                ("LOCAL_POSITION_NED", mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200_000),
                (
                    "POSITION_TARGET_GLOBAL_INT",
                    mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,
                    200_000,
                ),
            ]
            self._request_message_interval(px4_interval_requests)

        stream_requests = [
            ("EXTENDED_STATUS", mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            ("POSITION", mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),
            ("EXTRA1", mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 2),
        ]
        self._request_message_interval(stream_requests)

    def _request_fast_arm_status_streams(self):
        """在 ARM 前临时请求更高频的状态流，提高短暂解锁的捕获率。"""
        if self.master is None:
            return

        fast_requests = [
            ("HEARTBEAT", mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 100_000),
            ("SYS_STATUS", mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 100_000),
            ("GPS_RAW_INT", mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 100_000),
            ("GLOBAL_POSITION_INT", mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100_000),
        ]
        print("   📡 ARM 前提升状态流频率...")
        self._request_message_interval(fast_requests)
        time.sleep(0.5)
