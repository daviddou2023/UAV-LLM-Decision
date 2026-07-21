import time

from pymavlink import mavutil

from mavlink_handler_base import FirmwareType


class MavlinkFirmwareMixin:
    def detect_and_adapter(self):
        """检测固件类型并适配参数"""
        print("\n" + "=" * 50)
        print("   固件类型检测与适配")
        print("=" * 50)

        self._detect_firmware()
        self._get_firmware_details()
        self._print_firmware_info()
        self._adapter_parameters()

        print("=" * 50 + "\n")

    def _detect_firmware(self):
        """检测飞控固件类型"""
        print("\n🔍 检测固件类型...")

        known_heartbeat = self.last_fc_heartbeat_msg
        if known_heartbeat:
            if known_heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.firmware_type = FirmwareType.ARDUPILOT
                print("   ✅ 检测到: ArduPilot (通过首个心跳)")
                return True
            if known_heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.firmware_type = FirmwareType.PX4
                print("   ✅ 检测到: PX4 (通过首个心跳)")
                return True

        try:
            start_time = time.time()
            while time.time() - start_time < 3:
                self._poll_messages()
                msg = self.last_fc_heartbeat_msg
                if msg:
                    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                        self.firmware_type = FirmwareType.ARDUPILOT
                        print("   ✅ 检测到: ArduPilot (通过心跳)")
                        return True
                    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                        self.firmware_type = FirmwareType.PX4
                        print("   ✅ 检测到: PX4 (通过心跳)")
                        return True
                    print(f"   ⚠️ 未知 autopilot 类型: {msg.autopilot}")
                time.sleep(0.1)
        except Exception as exc:
            print(f"   心跳检测失败: {exc}")

        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                "ARMING_CHECK",
                -1,
            )
            request_time = time.time()
            while time.time() - request_time < 2:
                self._poll_messages()
                entry = self.param_value_cache.get("ARMING_CHECK")
                if entry and entry[1] >= request_time:
                    self.firmware_type = FirmwareType.ARDUPILOT
                    print("   ✅ 检测到: ArduPilot (通过参数 ARMING_CHECK)")
                    return True
                time.sleep(0.1)
        except Exception as exc:
            print(f"   无法通过参数 ARMING_CHECK 检测固件类型: {exc}")

        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                "SYS_AUTOSTART",
                -1,
            )
            request_time = time.time()
            while time.time() - request_time < 2:
                self._poll_messages()
                entry = self.param_value_cache.get("SYS_AUTOSTART")
                if entry and entry[1] >= request_time:
                    self.firmware_type = FirmwareType.PX4
                    print("   ✅ 检测到: PX4 (通过参数 SYS_AUTOSTART)")
                    return True
                time.sleep(0.1)
        except Exception as exc:
            print(f"   无法通过参数 SYS_AUTOSTART 检测固件类型: {exc}")

        print("   ⚠️ 无法检测固件类型，默认使用 ArduPilot 兼容模式")
        self.firmware_type = FirmwareType.ARDUPILOT
        return False

    def _get_firmware_details(self):
        """获取固件详细信息"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            request_time = time.time()
            while time.time() - request_time < 3:
                self._poll_messages()
                msg = self.autopilot_version_msg if self.autopilot_version_time >= request_time else None
                if msg:
                    if getattr(msg, "flight_sw_version", None):
                        major = (msg.flight_sw_version >> 24) & 0xFF
                        minor = (msg.flight_sw_version >> 16) & 0xFF
                        patch = (msg.flight_sw_version >> 8) & 0xFF
                        self.firmware_version = f"{major}.{minor}.{patch}"

                    if getattr(msg, "vendor_name", None):
                        try:
                            self.firmware_vendor = msg.vendor_name.decode("utf-8", errors="ignore").strip("\x00")
                        except Exception:
                            self.firmware_vendor = str(msg.vendor_name)
                    break
                time.sleep(0.1)
        except Exception as exc:
            print(f"   获取固件详细信息失败: {exc}")

    def _print_firmware_info(self):
        firmware_names = {
            FirmwareType.ARDUPILOT: "ArduPilot",
            FirmwareType.PX4: "PX4",
            FirmwareType.UNKNOWN: "Unknown",
        }

        print("\n📋 固件详细信息:")
        print(f"   类型: {firmware_names.get(self.firmware_type, 'Unknown')}")
        if self.firmware_version:
            print(f"   版本: {self.firmware_version}")
        if self.firmware_vendor:
            print(f"   厂商: {self.firmware_vendor}")

    def _adapter_parameters(self):
        print("\n🔧 适配参数:")

        if self.firmware_type == FirmwareType.ARDUPILOT:
            self.goto_command = mavutil.mavlink.MAV_CMD_DO_REPOSITION
            self.goto_supports_speed = True
            self.default_mode = "GUIDED"
            self.arm_mode = None
            self.takeoff_mode = "GUIDED"
            print("   - 解锁模式: 保持当前模式")
            print("   - 起飞模式: GUIDED")
            print("   - GOTO模式: GUIDED")
            print("   - GOTO命令: MAV_CMD_DO_REPOSITION (192)")

        elif self.firmware_type == FirmwareType.PX4:
            self.goto_command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.goto_supports_speed = False
            self.default_mode = "OFFBOARD"
            self.arm_mode = "POSCTL"
            self.takeoff_mode = "POSCTL"
            print("   - 解锁模式: POSCTL")
            print("   - 起飞模式: POSCTL")
            print("   - GOTO模式: OFFBOARD")
            print("   - GOTO实现: 持续发送位置 setpoint")

        else:
            self.goto_command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.goto_supports_speed = False
            self.default_mode = "GUIDED"
            self.arm_mode = None
            self.takeoff_mode = "GUIDED"
            print("   - 使用兼容模式")
