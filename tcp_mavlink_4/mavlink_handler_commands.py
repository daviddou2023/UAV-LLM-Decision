import time

from pymavlink import mavutil

import config
from mavlink_handler_base import FirmwareType


class MavlinkCommandMixin:
    def _send_command_long_with_retry(
        self,
        command_id,
        params,
        timeout=None,
        retries=3,
        target_system=None,
        target_component=None,
        retry_components=None,
        label=None,
        continue_on_in_progress=True,
        cmd_id=None, # 新增 cmd_id
    ):
        if timeout is None:
            timeout = config.CMD_ACK_TIMEOUT
            
        # 如果没有传入 cmd_id，使用当前最新的（如果是顶级调用，会自动在外部函数生成）
        if cmd_id is None: cmd_id = self.current_cmd_id

        target_system, target_component = self._get_command_target_ids(target_system, target_component)
        if retry_components is None:
            retry_components = [target_component]
            if target_component != 0:
                retry_components.append(0)

        last_ack_result = None
        label_text = label or f"command {command_id}"

        for attempt in range(1, retries + 1):
            if self.is_command_cancelled(cmd_id): return None
            
            component_for_attempt = retry_components[min(attempt - 1, len(retry_components) - 1)]
            self._clear_command_ack(command_id, target_system, component_for_attempt)

            try:
                with self.io_lock:
                    self.master.mav.command_long_send(
                        target_system,
                        component_for_attempt,
                        command_id,
                        0,
                        *params,
                    )
                print(
                    f"   📤 {label_text} 已发送 "
                    f"(attempt={attempt}/{retries}, sysid={target_system}, compid={component_for_attempt})"
                )
            except Exception as exc:
                print(f"   ❌ 发送 {label_text} 失败: {exc}")
                return None

            # --- 修改：改为可中断的 ACK 等待 ---
            start_wait = time.time()
            ack_result = None
            while time.time() - start_wait < timeout:
                if self.is_command_cancelled(cmd_id): return None
                
                # 每次只等待一小会儿，以便能快速检测到中断
                ack_result = self.wait_command_ack(
                    command_id,
                    timeout=0.2, 
                    target_system=target_system,
                    target_component=component_for_attempt,
                    continue_on_in_progress=continue_on_in_progress,
                )
                if ack_result is not None:
                    return ack_result
                time.sleep(0.05)

            last_ack_result = ack_result
            if attempt < retries:
                print(f"   ⚠️ {label_text} 未收到 ACK，准备重试...")
                time.sleep(0.2)

        return last_ack_result

    def set_mode(self, mode_name, cmd_id=None):
        print(f"🔄 切换模式: {mode_name}")
        
        # 如果是作为独立指令运行，且没有传入 cmd_id，则生成一个
        if cmd_id is None:
            cmd_id = self.start_new_command()

        self.update_state()
        
        if self.is_command_cancelled(cmd_id): return False

        mode_ready, reason = self._describe_mode_readiness(mode_name)
        if not mode_ready:
            print(f"   ❌ 模式切换前检查未通过: {reason}")
            return False

        if self.is_command_cancelled(cmd_id): return False

        if self.firmware_type == FirmwareType.PX4:
            px4_modes = self.get_px4_mode_mapping()
            mode_tuple = px4_modes.get(mode_name)
            if mode_tuple is None:
                print(f"   ❌ PX4 未知模式: {mode_name}")
                return False
        else:
            if not self._mode_mapping:
                self._mode_mapping = self.master.mode_mapping()
            mode_id = self._mode_mapping.get(mode_name)
            if mode_id is None:
                print(f"   ❌ ArduPilot 未知模式: {mode_name}")
                return False

        command_id = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        target_system, target_component = self._get_command_target_ids()

        try:
            if self.firmware_type == FirmwareType.PX4:
                base_mode, custom_mode, custom_sub_mode = mode_tuple
                self.master.set_mode_px4(base_mode, custom_mode, custom_sub_mode)
            else:
                with self.io_lock:
                    self.master.mav.command_long_send(
                        target_system,
                        target_component,
                        command_id,
                        0,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )

            print("   📤 模式切换命令已发送")

            ack_result = self.wait_command_ack(
                command_id,
                timeout=1.0,
                target_system=target_system,
                target_component=target_component,
            )
            if self._is_ack_rejected(ack_result):
                print("   ❌ 飞控拒绝模式切换命令")
                return False

            start = time.time()
            timeout = config.MODE_SWITCH_TIMEOUT
            while time.time() - start < timeout:
                if self.is_command_cancelled(cmd_id):
                    print("   🛑 模式切换已被新指令中断")
                    return False
                if self.current_state["mode"] == mode_name:
                    print("   ✅ 模式切换成功")
                    return True
                time.sleep(0.05)

            if self._is_ack_positive(ack_result):
                print(f"   ⚠️ 已收到ACK但状态未刷新，当前模式: {self.current_state['mode']}")
                return True

            print(f"   ❌ 模式切换超时，当前模式: {self.current_state['mode']}")
            return False
        except Exception as exc:
            print(f"   ❌ 发送模式切换命令失败: {exc}")
            return False

    def arm(self, allow_mode_fallback=True, cmd_id=None):
        print("🔓 执行 ARM...")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        if self.arm_mode and self.current_state["mode"] != self.arm_mode:
            print(f"   🔄 切换到 {self.arm_mode} 模式...")
            if not self.set_mode(self.arm_mode, cmd_id=cmd_id):
                return False

        if self.is_command_cancelled(cmd_id): return False

        self._request_fast_arm_status_streams()
        self.update_state()

        ready, reason = self._describe_arm_readiness()
        if not ready:
            print(f"   ❌ 解锁前检查未通过: {reason}")
            return False

        command_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        target_system, target_component = self._get_command_target_ids()

        try:
            with self.io_lock:
                self.master.mav.command_long_send(
                    target_system,
                    target_component,
                    command_id,
                    0,
                    1,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )

            print("   📤 ARM 命令已发送")

            ack_result = self.wait_command_ack(
                command_id,
                timeout=1.0,
                target_system=target_system,
                target_component=target_component,
            )
            if self._is_ack_rejected(ack_result):
                print("   ❌ 飞控拒绝解锁命令")
                return False

            start = time.time()
            timeout = 8.0
            while time.time() - start < timeout:
                if self.is_command_cancelled(cmd_id):
                    print("   🛑 ARM 已被新指令中断")
                    return False
                if self.current_state["armed"]:
                    print("   ✅ 解锁成功")
                    return True
                time.sleep(0.05)

            if ack_result is None:
                try:
                    print("   ⚠️ 未收到 ACK，尝试 arducopter_arm()")
                    with self.io_lock:
                        self.master.arducopter_arm()
                except Exception as exc:
                    print(f"   ⚠️ fallback失败: {exc}")

            start = time.time()
            while time.time() - start < 5.0:
                if self.is_command_cancelled(cmd_id): return False
                if self.current_state["armed"]:
                    print("   ✅ 解锁成功")
                    return True
                time.sleep(0.05)

            if self._is_ack_positive(ack_result):
                print("   ⚠️ 已收到ACK但状态未刷新")
                return True

            print("   ❌ 解锁失败")
            return False
        except Exception as exc:
            print(f"   ❌ ARM异常: {exc}")
            return False

    def disarm(self, cmd_id=None):
        """上锁"""
        print("🔒 执行 DISARM...")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        if not self.current_state["armed"]:
            print("   ⚠️ 当前已处于上锁状态，未执行上锁命令")
            return False

        command_id = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        target_system, target_component = self._get_command_target_ids()
        ack_result = None
        disarmed_changed = False
        retry_components = [target_component]
        if target_component != 0:
            retry_components.append(0)

        for attempt in range(1, 4):
            if self.is_command_cancelled(cmd_id): return False
            
            component_for_attempt = retry_components[min(attempt - 1, len(retry_components) - 1)]
            self._clear_command_ack(command_id, target_system, component_for_attempt)
            with self.io_lock:
                self.master.mav.command_long_send(
                    target_system,
                    component_for_attempt,
                    command_id,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            print(
                f"   📤 上锁命令 已发送 "
                f"(attempt={attempt}/3, sysid={target_system}, compid={component_for_attempt})"
            )

            # --- 修改 wait 逻辑，使其能被中断 ---
            start_wait = time.time()
            while time.time() - start_wait < config.CMD_ACK_TIMEOUT:
                if self.is_command_cancelled(cmd_id): return False
                
                # 这里简单处理，实际上 _wait_for_command_effect_with_ack 内部也需要改造
                # 但为了减少改动，我们这里用轮询 + 较小的超时
                attempt_ack, disarmed_changed = self._wait_for_command_effect_with_ack(
                    command_id,
                    lambda: not self.current_state["armed"],
                    target_system,
                    component_for_attempt,
                    timeout=0.5, # 缩短内部等待时间
                )
                if attempt_ack is not None:
                    ack_result = attempt_ack
                if disarmed_changed:
                    break
                if self._is_ack_rejected(attempt_ack):
                    ack_result = attempt_ack
                    break
            
            if disarmed_changed:
                break
            if attempt < 3:
                print("   ⚠️ 上锁命令 未收到 ACK，准备重试...")

        if self._is_ack_rejected(ack_result):
            print("   ❌ 飞控拒绝上锁命令")
            return False

        if disarmed_changed:
            print("   ✅ 上锁成功")
            return True

        print("   ⏳ 等待 armed 状态清除，最长 5.0s")
        if self._wait_until_with_cancel(lambda: not self.current_state["armed"], 5.0, cmd_id):
            print("   ✅ 上锁成功")
            return True

        if self._is_ack_positive(ack_result):
            print("   ⚠️ 已收到上锁 ACK，但 armed 状态未刷新")
            return True

        print("   ❌ 上锁失败")
        return False

    def takeoff(self, altitude, cmd_id=None):
        """起飞"""
        print(f"🚁 执行 TAKEOFF，高度: {altitude}m")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        target_flight_mode = self.takeoff_mode
        print(f"   当前模式: {self.current_state['mode']}")

        start_alt = self.current_state["alt_rel"]
        if self.current_state["flying"] or (
            self.current_state["armed"] and start_alt > max(1.0, config.LAND_ALT_THRESHOLD + 0.3)
        ):
            print(f"   ⚠️ 当前已处于空中状态(相对高度 {start_alt:.1f}m)，未执行起飞命令")
            return False

        if self.current_state["mode"] != target_flight_mode:
            print(f"   🔄 切换到 {target_flight_mode} 模式...")
            if not self.set_mode(target_flight_mode, cmd_id=cmd_id):
                print(f"   ❌ {target_flight_mode} 模式切换失败")
                return False
            time.sleep(1)
            self.update_state()

        if self.is_command_cancelled(cmd_id): return False

        if not self.current_state["armed"]:
            print("   🔓 无人机未解锁，执行解锁...")
            if not self.arm(cmd_id=cmd_id):
                print("   ❌ 解锁失败")
                return False
            time.sleep(1)

        if self.is_command_cancelled(cmd_id): return False

        start_alt = self.current_state["alt_rel"]
        print(f"   起始相对高度: {start_alt:.1f}m")

        command_id = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        target_system, target_component = self._get_command_target_ids()
        ack_result = self._send_command_long_with_retry(
            command_id,
            (0, 0, 0, 0, 0, 0, altitude),
            timeout=config.CMD_ACK_TIMEOUT,
            retries=3,
            target_system=target_system,
            target_component=target_component,
            label="起飞命令",
            continue_on_in_progress=True,
        )
        if self._is_ack_rejected(ack_result):
            print("   ❌ 飞控拒绝起飞命令")
            return False

        start_time = time.time()
        target_gain = max(0.5, altitude * config.TAKEOFF_ALT_THRESHOLD)
        current_alt = start_alt

        while time.time() - start_time < 30:
            if self.is_command_cancelled(cmd_id):
                print("   🛑 起飞监控已被新指令中断")
                return False
                
            self.update_state()
            current_alt = self.current_state["alt_rel"]
            alt_diff = current_alt - start_alt
            print(f"   📊 当前相对高度: {current_alt:.1f}m (上升: {alt_diff:.1f}m)")

            if alt_diff >= target_gain:
                print(f"   ✅ 起飞成功! 到达高度: {current_alt:.1f}m")
                return True

            time.sleep(1)

        if self._is_ack_positive(ack_result):
            print(f"   ⚠️ 已收到起飞 ACK，但高度未刷新，最终高度: {current_alt:.1f}m")
            return True

        print(f"   ❌ 起飞超时，最终高度: {current_alt:.1f}m")
        return False

    def land(self, cmd_id=None):
        """降落"""
        print("🛬 执行 LAND...")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        start_alt = self.current_state["alt_rel"]
        currently_airborne = (
            self.current_state["armed"]
            or self.current_state["flying"]
            or start_alt > (config.LAND_ALT_THRESHOLD + 0.3)
        )
        if not currently_airborne:
            print(f"   ⚠️ 当前已在地面或未飞行(相对高度 {start_alt:.1f}m)，未执行降落命令")
            return False

        command_id = mavutil.mavlink.MAV_CMD_NAV_LAND
        target_system, target_component = self._get_command_target_ids()
        ack_result = self._send_command_long_with_retry(
            command_id,
            (0, 0, 0, 0, 0, 0, 0),
            timeout=config.CMD_ACK_TIMEOUT,
            retries=3,
            target_system=target_system,
            target_component=target_component,
            label="降落命令",
            continue_on_in_progress=True,
        )
        if self._is_ack_rejected(ack_result):
            print("   ❌ 飞控拒绝降落命令")
            return False

        descent_observed = False
        land_mode_observed = self.current_state["mode"] == "LAND"
        start_time = time.time()
        current_alt = start_alt
        while time.time() - start_time < config.LAND_TIMEOUT:
            if self.is_command_cancelled(cmd_id):
                print("   🛑 降落监控已被新指令中断")
                return False
                
            time.sleep(1)
            self.update_state()
            current_alt = self.current_state["alt_rel"]
            print(f"   📊 当前相对高度: {current_alt:.1f}m")

            if self.current_state["mode"] == "LAND":
                land_mode_observed = True
            if current_alt <= start_alt - 0.3:
                descent_observed = True

            if (
                current_alt < config.LAND_ALT_THRESHOLD
                and (land_mode_observed or descent_observed or self._is_ack_positive(ack_result))
            ):
                print("   ✅ 降落成功!")
                return True

            if land_mode_observed and descent_observed:
                print("   ✅ 已确认进入降落流程")
                return True

        if self._is_ack_positive(ack_result) and (land_mode_observed or descent_observed):
            print(f"   ⚠️ 已收到降落 ACK，并观察到降落迹象，最终高度: {current_alt:.1f}m")
            return True

        if self._is_ack_positive(ack_result):
            print(f"   ⚠️ 已收到降落 ACK，但未观察到模式切换或高度下降，最终高度: {current_alt:.1f}m")
            return False

        print("   ❌ 降落超时")
        return False

    def rtl(self, cmd_id=None):
        """返航"""
        print("🏠 执行 RTL...")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        # 检查高度和飞行状态
        start_alt = self.current_state.get("alt_rel", 0.0)
        is_airborne = self.current_state.get("armed") or start_alt > 1.0
        
        if not is_airborne:
            print("   ⚠️ 无人机当前不在空中，未执行返航命令")
            return False

        # 对于 ArduPilot 和 PX4，切换到 RTL 模式是最稳健的返航方式
        target_mode = "RTL"
        if self.firmware_type == FirmwareType.PX4:
            target_mode = "RTL" # PX4 也支持 RTL 字符模式名

        print(f"   🔄 正在切换到 {target_mode} 模式触发返航...")
        
        # 使用 set_mode 来执行切换，它已经包含了 ACK 等待和状态检查逻辑
        success = self.set_mode(target_mode, cmd_id=cmd_id)
        
        if success:
            print("   ✅ 返航模式已成功激活")
            return True
        else:
            # 如果 set_mode 失败，尝试作为最后的备选：直接发送 MAV_CMD_NAV_RETURN_TO_LAUNCH
            print(f"   ⚠️ {target_mode} 模式切换失败，尝试发送备选 MAV_CMD_NAV_RETURN_TO_LAUNCH...")
            
            if self.is_command_cancelled(cmd_id): return False
            
            command_id = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
            target_system, target_component = self._get_command_target_ids()
            
            ack_result = self._send_command_long_with_retry(
                command_id,
                (0, 0, 0, 0, 0, 0, 0),
                timeout=config.CMD_ACK_TIMEOUT,
                retries=2,
                target_system=target_system,
                target_component=target_component,
                label="返航备选命令",
                cmd_id=cmd_id
            )
            
            if self._is_ack_positive(ack_result):
                print("   ✅ 备选返航命令已被接受")
                return True
            
            print("   ❌ 返航启动失败：飞控拒绝了模式切换和备选命令")
            return False

    def _send_px4_position_setpoint(self, lat, lng, alt_rel, yaw=None):
        """
        发送 PX4 OFFBOARD 位置 setpoint
        使用 GLOBAL_RELATIVE_ALT_INT
        """
        type_mask = 0b110111111000
        if yaw is None:
            type_mask = 0b110111111100

        self.master.mav.set_position_target_global_int_send(
            int(time.time() * 1000),
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lng * 1e7),
            alt_rel,
            0,
            0,
            0,
            0,
            0,
            0,
            0 if yaw is None else yaw,
            0,
        )
        self.offboard_last_send_time = time.time()

    def _px4_offboard_preheat(self, lat, lng, alt_rel, duration=None):
        """进入 OFFBOARD 前预热发送 setpoint"""
        if duration is None:
            duration = self.offboard_preheat_seconds

        print(f"   📡 PX4 OFFBOARD 预热 {duration:.1f} 秒...")
        count = 0
        start = time.time()

        while time.time() - start < duration:
            self._send_px4_position_setpoint(lat, lng, alt_rel)
            count += 1
            time.sleep(self.offboard_setpoint_period)

        print(f"   ✅ 预热完成，发送了 {count} 个 setpoint")
        return count >= self.offboard_min_preheat_count

    def _px4_wait_offboard_mode(self, timeout=5.0):
        """等待 PX4 进入 OFFBOARD 模式"""
        start = time.time()
        while time.time() - start < timeout:
            self.update_state()
            if self.current_state["mode"] == "OFFBOARD":
                return True
            time.sleep(0.1)
        return False

    def _goto_px4_offboard(self, lat, lng, alt_rel, timeout_seconds, cmd_id):
        """
        PX4 严格 OFFBOARD GOTO
        """
        print("   📡 PX4 OFFBOARD GOTO 开始")

        if not self.current_state["armed"]:
            print("   🔓 无人机未解锁，先执行解锁...")
            if not self.arm(cmd_id=cmd_id):
                print("   ❌ PX4 解锁失败")
                return False
            time.sleep(1)

        if self.is_command_cancelled(cmd_id): return False

        # 预热也要检查中断
        if not self._px4_offboard_preheat_with_cancel(lat, lng, alt_rel, cmd_id):
            print("   ❌ PX4 OFFBOARD 预热失败或被中断")
            return False

        if self.current_state["mode"] != "OFFBOARD":
            print("   🔄 切换到 OFFBOARD...")
            if not self.set_mode("OFFBOARD", cmd_id=cmd_id):
                print("   ❌ 无法切换到 OFFBOARD")
                return False

        if not self._px4_wait_offboard_mode_with_cancel(cmd_id, timeout=5.0):
            print("   ❌ OFFBOARD 模式确认失败")
            return False

        print("   ✅ OFFBOARD 已激活，开始持续发送 setpoint")
        return self._wait_for_goto_complete(lat, lng, alt_rel, timeout_seconds, cmd_id, offboard=True)

    def _goto_ardupilot(self, lat, lng, alt_rel, speed, timeout_seconds, cmd_id):
        """ArduPilot GOTO"""
        command_id = self.goto_command or mavutil.mavlink.MAV_CMD_DO_REPOSITION
        target_system, target_component = self._get_command_target_ids()
        
        # 增加重试机制，防止飞控在切换模式瞬间拒绝指令
        for attempt in range(1, 3):
            if self.is_command_cancelled(cmd_id): return False
            
            self._clear_command_ack(command_id, target_system, target_component)
            try:
                print(f"   📤 发送 ArduPilot GOTO 命令 (尝试 {attempt}/2)")
                self.master.mav.command_int_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command_id,
                    0,
                    1,
                    float(speed),
                    0,
                    0,
                    0,
                    int(lat * 1e7),
                    int(lng * 1e7),
                    alt_rel,
                )
            except Exception as exc:
                print(f"   ❌ 发送 GOTO 命令失败: {exc}")
                return False

            ack_result = self.wait_command_ack(
                command_id,
                timeout=1.0, # 缩短单次 ACK 等待，以便快速重试
                target_system=target_system,
                target_component=target_component,
            )
            
            if self._is_ack_positive(ack_result):
                print("   ✅ GOTO 命令已被飞控接受")
                break
            
            if attempt < 2:
                print(f"   ⚠️ GOTO 被拒绝或超时 (result={ack_result})，准备重试...")
                time.sleep(0.5)
        else:
            print("   ❌ 飞控多次拒绝 GOTO 命令")
            return False

        if not self._wait_for_goto_complete(lat, lng, alt_rel, timeout_seconds, cmd_id, offboard=False):
            return False

        print("   ✅ 已到达目标位置")
        return True

    def goto(self, lat, lng, alt, speed=5.0, cmd_id=None):
        """定点飞行"""
        print("\n📍 执行 GOTO 定点飞行")
        
        if cmd_id is None:
            cmd_id = self.start_new_command()

        print(f"   目标: ({lat:.6f}, {lng:.6f})")
        print(f"   高度(相对): {alt}m, 速度: {speed}m/s")

        firmware_name = (
            "ArduPilot"
            if self.firmware_type == FirmwareType.ARDUPILOT
            else "PX4" if self.firmware_type == FirmwareType.PX4 else "Unknown"
        )
        print(f"   固件类型: {firmware_name}")

        start_lat = self.current_state["lat"]
        start_lng = self.current_state["lng"]
        distance_to_target = self._haversine_distance(start_lat, start_lng, lat, lng)
        print(f"   目标距离: {distance_to_target:.1f}米")

        current_alt = self.current_state["alt_rel"]
        if (
            distance_to_target < config.GOTO_REACHED_DISTANCE
            and abs(current_alt - alt) < config.GOTO_VERTICAL_THRESHOLD
        ):
            print("   ⚠️ 当前已在目标点附近，未执行 GOTO 命令")
            return False

        estimated_time = distance_to_target / speed if speed > 0 else distance_to_target / 5.0
        timeout_seconds = max(
            config.GOTO_MIN_TIMEOUT,
            min(config.GOTO_MAX_TIMEOUT, int(estimated_time * 1.5)),
        )
        print(f"   动态超时: {timeout_seconds}秒")

        target_flight_mode = self.default_mode
        print(f"   当前模式: {self.current_state['mode']}")

        if self.current_state["mode"] != target_flight_mode:
            print(f"   🔄 切换到 {target_flight_mode} 模式...")
            if not self.set_mode(target_flight_mode, cmd_id=cmd_id):
                print(f"   ❌ {target_flight_mode} 模式切换失败")
                return False

            if not self._wait_until_with_cancel(
                lambda: self.current_state["mode"] == target_flight_mode,
                3.0,
                cmd_id
            ):
                print("   ❌ 模式切换超时")
                return False
            print(f"   ✅ {target_flight_mode} 模式已就绪")
            
            # --- 新增：给飞控一点时间稳定位置控制器 ---
            time.sleep(0.2) 
            self.update_state()

        if self.is_command_cancelled(cmd_id): return False

        if self.firmware_type == FirmwareType.ARDUPILOT:
            return self._goto_ardupilot(lat, lng, alt, speed, timeout_seconds, cmd_id)
        if self.firmware_type == FirmwareType.PX4:
            return self._goto_px4_offboard(lat, lng, alt, timeout_seconds, cmd_id)
        return self._goto_ardupilot(lat, lng, alt, speed, timeout_seconds, cmd_id)

    def _wait_for_goto_complete(self, target_lat, target_lng, target_alt_rel, timeout_seconds, cmd_id, offboard=False):
        print("\n   🔍 开始监控到达目标...")
        print(f"      检查间隔: {self.offboard_setpoint_period:.2f}s")
        print(
            f"      到达阈值: 水平 < {config.GOTO_REACHED_DISTANCE}米, "
            f"垂直 < {config.GOTO_VERTICAL_THRESHOLD}米"
        )

        start_time = time.time()
        last_distance = float("inf")
        stable_count = 0
        check_count = 0
        horizontal_distance = float("inf")
        vertical_distance = float("inf")

        while time.time() - start_time < timeout_seconds:
            if self.is_command_cancelled(cmd_id):
                print("   🛑 GOTO 监控已被新指令中断")
                return False
                
            time.sleep(self.offboard_setpoint_period)
            check_count += 1
            self.update_state()

            if offboard:
                try:
                    self._send_px4_position_setpoint(target_lat, target_lng, target_alt_rel)
                except Exception as exc:
                    print(f"   ❌ OFFBOARD setpoint 发送失败: {exc}")
                    return False

            current_lat = self.current_state["lat"]
            current_lng = self.current_state["lng"]
            current_alt_rel = self.current_state["alt_rel"]

            horizontal_distance = self._haversine_distance(
                current_lat,
                current_lng,
                target_lat,
                target_lng,
            )
            vertical_distance = abs(current_alt_rel - target_alt_rel)

            if check_count % 10 == 0 or horizontal_distance < last_distance - 1:
                print(
                    f"   📍 [{check_count * self.offboard_setpoint_period:.1f}s] "
                    f"距离: {horizontal_distance:.1f}米, 高度差: {vertical_distance:.1f}米"
                )
                last_distance = horizontal_distance

            if (
                horizontal_distance < config.GOTO_REACHED_DISTANCE
                and vertical_distance < config.GOTO_VERTICAL_THRESHOLD
            ):
                stable_count += 1
                if stable_count >= 3:
                    print("\n   ✅ 到达目标!")
                    print(f"      水平距离: {horizontal_distance:.1f}米")
                    print(f"      垂直距离: {vertical_distance:.1f}米")
                    print(f"      耗时: {check_count * self.offboard_setpoint_period:.1f}秒")
                    return True
            else:
                stable_count = 0

        print("\n   ❌ GOTO 超时!")
        print(f"      最终水平距离: {horizontal_distance:.1f}米")
        print(f"      最终高度差: {vertical_distance:.1f}米")
        return False

    def _px4_offboard_preheat_with_cancel(self, lat, lng, alt_rel, cmd_id, duration=None):
        if duration is None:
            duration = self.offboard_preheat_seconds

        print(f"   📡 PX4 OFFBOARD 预热 {duration:.1f} 秒...")
        count = 0
        start = time.time()

        while time.time() - start < duration:
            if self.is_command_cancelled(cmd_id): return False
            self._send_px4_position_setpoint(lat, lng, alt_rel)
            count += 1
            time.sleep(self.offboard_setpoint_period)

        return count >= self.offboard_min_preheat_count

    def _px4_wait_offboard_mode_with_cancel(self, cmd_id, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            if self.is_command_cancelled(cmd_id): return False
            self.update_state()
            if self.current_state["mode"] == "OFFBOARD":
                return True
            time.sleep(0.1)
        return True
