import math
import threading
import time

from pymavlink import mavutil

import config


class FirmwareType:
    """固件类型枚举"""
    UNKNOWN = 0
    ARDUPILOT = 1
    PX4 = 2


class MavlinkHandlerBaseMixin:
    def start_new_command(self):
        """生成并返回一个新的指令版本 ID"""
        with self.cmd_lock:
            self.current_cmd_id += 1
            return self.current_cmd_id

    def is_command_cancelled(self, cmd_id):
        """检查指定 ID 的指令是否已被新指令中断"""
        with self.cmd_lock:
            return cmd_id != self.current_cmd_id

    def _wait_until_with_cancel(self, predicate, timeout, cmd_id, poll_interval=0.05):
        """带中断检测的等待"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_command_cancelled(cmd_id):
                return False
            self._poll_messages()
            if predicate():
                return True
            time.sleep(poll_interval)
        self._poll_messages()
        return predicate()

    def get_px4_mode_mapping(self):
        return dict(getattr(mavutil, "px4_map", {}))

    def _get_current_mode_from_heartbeat(self, msg):
        if self.firmware_type == FirmwareType.PX4:
            try:
                mode_name = mavutil.interpret_px4_mode(msg.base_mode, msg.custom_mode)
                if mode_name == "FOLLOWME":
                    return "FOLLOW"
                return mode_name
            except Exception:
                return "UNKNOWN"

        if self._mode_mapping:
            for name, mid in self._mode_mapping.items():
                if msg.custom_mode == mid:
                    return name
        return "UNKNOWN"

    def _normalize_param_id(self, param_id):
        if isinstance(param_id, bytes):
            return param_id.decode("utf-8", errors="ignore").strip("\x00").strip()
        return str(param_id).strip("\x00").strip()

    def _get_recent_arm_blocker(self, window_seconds=8.0):
        now = time.time()
        candidates = []
        if self.last_arm_status_text and now - self.last_arm_status_time <= window_seconds:
            candidates.append(self.last_arm_status_text)
        if self.last_statustext and now - self.last_statustext_time <= window_seconds:
            candidates.append(self.last_statustext)

        for text in reversed(candidates):
            text_lower = text.lower()
            if "prearm" in text_lower or text_lower.startswith("arm:"):
                return text
        return candidates[-1] if candidates else None

    def _is_position_estimate_ready(self):
        gps_fix_type = self.current_state.get("gps_fix_type", 0)
        has_global_position = not (
            abs(self.current_state["lat"]) < 1e-7
            and abs(self.current_state["lng"]) < 1e-7
        )
        return gps_fix_type >= 3 and has_global_position

    def _is_position_related_blocker(self, text):
        """判断是否为“位置/GPS相关”的阻塞原因（统一标准）"""
        if not text:
            return False

        t = text.lower()
        keywords = (
            "need position estimate",
            "need 3d fix",
            "need ekf",
            "ekf variance",
            "ahrs",
            "gps glitch",
            "bad gps",
            "no gps",
            "home not set",
            "position",
        )
        return any(k in t for k in keywords)

    def _is_prearm_blocker_fatal(self, arm_blocker):
        if not arm_blocker:
            return False

        blocker = arm_blocker.lower()
        if self._is_position_related_blocker(blocker):
            return True
        if "prearm:" in blocker:
            return True
        return False

    def _describe_arm_readiness(self):
        arm_blocker = self._get_recent_arm_blocker()
        gps_fix_type = self.current_state.get("gps_fix_type", 0)
        satellites = self.current_state.get("satellites", 0)
        hdop = self.current_state.get("hdop", 0.0)
        has_position = self._is_position_estimate_ready()

        if arm_blocker and self._is_prearm_blocker_fatal(arm_blocker):
            return (
                False,
                f"{arm_blocker} (gps_fix={gps_fix_type}, sats={satellites}, "
                f"hdop={hdop:.1f}, pos_ready={has_position})",
            )

        if gps_fix_type < 3:
            return False, f"GPS fix 不足 (fix_type={gps_fix_type}, sats={satellites}, hdop={hdop:.1f})"

        if not has_position:
            return False, "位置估计未就绪（未获得有效 GLOBAL_POSITION_INT）"

        return True, None

    def _mode_requires_position_estimate(self, mode_name):
        if self.firmware_type != FirmwareType.ARDUPILOT:
            return False
        mode = str(mode_name or "").upper()
        return mode in {"GUIDED", "LOITER", "AUTO", "RTL", "SMART_RTL", "POSHOLD"}

    def _describe_mode_readiness(self, mode_name):
        if not self._mode_requires_position_estimate(mode_name):
            return True, None

        arm_blocker = self._get_recent_arm_blocker()
        gps_fix_type = self.current_state.get("gps_fix_type", 0)
        satellites = self.current_state.get("satellites", 0)
        hdop = self.current_state.get("hdop", 0.0)
        has_position = self._is_position_estimate_ready()

        if arm_blocker and self._is_prearm_blocker_fatal(arm_blocker):
            return (
                False,
                f"{mode_name} 模式需要稳定位置估计，但当前存在: {arm_blocker} "
                f"(gps_fix={gps_fix_type}, sats={satellites}, hdop={hdop:.1f}, pos_ready={has_position})",
            )

        if gps_fix_type < 3:
            return (
                False,
                f"{mode_name} 模式需要 GPS 3D Fix "
                f"(fix_type={gps_fix_type}, sats={satellites}, hdop={hdop:.1f})",
            )

        if not has_position:
            return False, f"{mode_name} 模式需要有效位置估计（未获得有效 GLOBAL_POSITION_INT）"
            
        # 特别针对 RTL 模式检查 Home 点是否已设置
        if mode_name.upper() == "RTL" and not self.current_state.get("home_set"):
            return False, "RTL 模式要求已设置 Home 点，当前飞控未报告 Home 位置"

        return True, None

    def _get_mode_observation_timeout(self, mode_name):
        base_timeout = config.MODE_SWITCH_TIMEOUT + 2.0
        if self.firmware_type != FirmwareType.ARDUPILOT:
            return base_timeout

        mode_upper = (mode_name or "").upper()
        if mode_upper in {"GUIDED", "AUTO", "RTL", "LOITER", "SMART_RTL", "POSHOLD"}:
            return max(base_timeout, 12.0)
        return max(base_timeout, 6.0)

    def _should_try_stabilize_fallback(self, arm_blocker):
        if self.firmware_type != FirmwareType.ARDUPILOT:
            return False
        if self.current_state["mode"] != "GUIDED":
            return False
        if not arm_blocker:
            return True
        return self._is_position_related_blocker(arm_blocker.lower())

    def _make_command_ack_key(self, command_id, target_system=None, target_component=None):
        target_system = int(target_system) if target_system is not None else 0
        target_component = int(target_component) if target_component is not None else 0
        return target_system, target_component, int(command_id)

    def wait_command_ack(
        self,
        command_id,
        timeout=None,
        target_system=None,
        target_component=None,
        continue_on_in_progress=True,
    ):
        """事件驱动 + 缓存兜底（工业稳定版）"""
        if timeout is None:
            timeout = config.CMD_ACK_TIMEOUT

        # 如果 target_component 是 0 (或者 None 变为 0)，我们可能需要匹配来自任何 component 的 ACK
        search_target_component = int(target_component) if target_component is not None else 0
        ack_key = self._make_command_ack_key(command_id, target_system, search_target_component)
        start_time = time.time()

        # 1. 检查精确匹配的缓存
        entry = self.command_ack_cache.get(ack_key)
        if entry and entry[1] >= start_time:
            result = entry[0]
            self.command_ack_cache.pop(ack_key, None)
            print(f"📨 (cache) COMMAND_ACK: command={command_id}, result={result}")
            return result

        # 2. 如果 search_target_component 是 0，尝试模糊匹配（同一系统的任意组件）
        if search_target_component == 0:
            for key, (result, cache_time, _) in list(self.command_ack_cache.items()):
                if key[0] == target_system and key[2] == command_id and cache_time >= start_time:
                    self.command_ack_cache.pop(key, None)
                    print(f"📨 (fuzzy cache) COMMAND_ACK: command={command_id}, result={result} from comp={key[1]}")
                    return result

        event = threading.Event()
        self.command_ack_events[ack_key] = {
            "event": event,
            "result": None,
            "msg": None,
        }

        # 3. 再次检查缓存（防止在注册 event 的瞬间收到消息）
        entry = self.command_ack_cache.get(ack_key)
        if entry and entry[1] >= start_time:
            self.command_ack_events.pop(ack_key, None)
            result = entry[0]
            self.command_ack_cache.pop(ack_key, None)
            print(f"📨 (late cache) COMMAND_ACK: command={command_id}, result={result}")
            return result

        if event.wait(timeout):
            data = self.command_ack_events.pop(ack_key, None)
            if data:
                result = data["result"]
                if continue_on_in_progress and result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    print(f"📨 COMMAND_ACK 收到: command={command_id}, result={result}")
                    return self.wait_command_ack(
                        command_id,
                        timeout,
                        target_system,
                        target_component,
                        continue_on_in_progress,
                    )
                print(f"   📨 COMMAND_ACK: command={command_id}, result={result}")
                return result
            return None

        # 4. 如果超时了，但我们是在等待广播 (comp=0)，最后扫一遍缓存
        if search_target_component == 0:
            for key, (result, cache_time, _) in list(self.command_ack_cache.items()):
                if key[0] == target_system and key[2] == command_id and cache_time >= start_time:
                    self.command_ack_cache.pop(key, None)
                    print(f"📨 (late fuzzy cache) COMMAND_ACK: command={command_id}, result={result}")
                    return result

        self.command_ack_events.pop(ack_key, None)
        print(f"⚠️ COMMAND_ACK 超时: command={command_id}")
        return None

    def _is_ack_rejected(self, ack_result):
        return ack_result in (
            mavutil.mavlink.MAV_RESULT_DENIED,
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED,
            mavutil.mavlink.MAV_RESULT_FAILED,
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
        )

    def _is_ack_positive(self, ack_result):
        return ack_result in (
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
        )

    def _wait_for_mode_change_with_ack(
        self,
        command_id,
        mode_name,
        target_system,
        target_component,
        timeout,
    ):
        """事件驱动 + 状态检测"""
        start_time = time.time()

        ack_result = self.wait_command_ack(
            command_id,
            timeout=1.0,
            target_system=target_system,
            target_component=target_component,
        )

        if ack_result is not None and self._is_ack_rejected(ack_result):
            return ack_result, False

        while time.time() - start_time < timeout:
            if self.current_state["mode"] == mode_name:
                return mavutil.mavlink.MAV_RESULT_ACCEPTED, True
            time.sleep(0.01)

        return (
            mavutil.mavlink.MAV_RESULT_ACCEPTED if ack_result else None,
            False,
        )

    def _wait_for_command_effect_with_ack(
        self,
        command_id,
        predicate,
        target_system,
        target_component,
        timeout,
        success_message=None,
    ):
        """事件驱动 + 状态判断"""
        start_time = time.time()

        ack_result = self.wait_command_ack(
            command_id,
            timeout=1.0,
            target_system=target_system,
            target_component=target_component,
        )

        if ack_result is not None and self._is_ack_rejected(ack_result):
            return ack_result, False

        while time.time() - start_time < timeout:
            if predicate():
                if success_message:
                    print(success_message)
                return mavutil.mavlink.MAV_RESULT_ACCEPTED, True
            time.sleep(0.01)

        return (
            mavutil.mavlink.MAV_RESULT_ACCEPTED if ack_result else None,
            False,
        )

    def _wait_until(self, predicate, timeout, poll_interval=0.01):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._poll_messages()
            if predicate():
                return True
            time.sleep(poll_interval)
        self._poll_messages()
        return predicate()

    def _clear_command_ack(self, command_id, target_system=None, target_component=None):
        ack_key = self._make_command_ack_key(command_id, target_system, target_component)
        self.command_ack_cache.pop(ack_key, None)

    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        """Haversine 公式计算地球表面两点间距离"""
        radius = 6371000
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = (
            math.sin(delta_lat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return radius * c

    def update_state(self, force_update=False):
        """更新无人机状态"""
        if not force_update and time.time() - self.last_update_time < self.update_interval:
            return self.current_state

        try:
            self._poll_messages()
        except Exception as exc:
            print(f"⚠️ update_state 异常: {exc}")

        self.last_update_time = time.time()
        return self.current_state

    def close(self):
        """关闭连接"""
        self.connected = False
        self.heartbeat_running = False
        self.rx_running = False

        try:
            if self.heartbeat_thread and self.heartbeat_thread.is_alive():
                self.heartbeat_thread.join(timeout=1.0)
        except Exception as exc:
            print(f"   ⚠️ 关闭心跳线程异常: {exc}")

        try:
            if self.rx_thread and self.rx_thread.is_alive():
                self.rx_thread.join(timeout=1.0)
        except Exception as exc:
            print(f"   ⚠️ 关闭接收线程异常: {exc}")

        if self.master:
            try:
                self.master.close()
                print("🔌 MAVLink连接已关闭")
            except Exception as exc:
                print(f"   ⚠️ 关闭MAVLink连接失败: {exc}")
