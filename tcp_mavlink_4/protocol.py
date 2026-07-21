import struct
import math


class Protocol:
    """系统协议定义"""

    # 数据包起始标记
    MAGIC = b'\x55\xAA'

    class MsgType:
        # ===== 命令类型（客户端 → 服务器）=====
        ARM = 0x01
        DISARM = 0x02
        TAKEOFF = 0x03
        SET_MODE = 0x04
        LAND = 0x05
        RTH = 0x06
        GOTO = 0x07
        MISSION_WP = 0x08  # 航点任务飞行

        # ===== 成功反馈（服务器 → 客户端）=====
        ARM_ACK = 0x11
        DISARM_ACK = 0x12
        TAKEOFF_ACK = 0x13
        SET_MODE_ACK = 0x14
        LAND_ACK = 0x15
        RTH_ACK = 0x16
        GOTO_ACK = 0x17
        MISSION_WP_ACK = 0x18  # 航线完成
        MISSION_PROGRESS = 0x19

        # ===== 失败反馈（服务器 → 客户端）=====
        ARM_FAIL = 0xF1
        DISARM_FAIL = 0xF2
        TAKEOFF_FAIL = 0xF3
        SET_MODE_FAIL = 0xF4
        LAND_FAIL = 0xF5
        RTH_FAIL = 0xF6
        GOTO_FAIL = 0xF7
        MISSION_WP_FAIL = 0xF8  # 航线失败

        # ===== 状态上报（服务器 → 客户端）=====
        STATUS_REPORT = 0x20

    # 模式映射表
    MODE_MAP = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
        4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
        8: "POSITION", 9: "LAND", 10: "OF_LOITER", 11: "DRIFT",
        12: "RESERVED_12", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
        16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
        20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
        24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
        28: "TURTLE"
    }

    MODE_NAME_TO_ID = {v: k for k, v in MODE_MAP.items()}

    # 航点协议版本
    MISSION_VERSION_V1 = 0x01
    MISSION_VERSION_V2 = 0x02

    # 单个航点载荷大小（字节）
    WAYPOINT_PAYLOAD_SIZE = 28  # 7个float * 4字节

    @staticmethod
    def calculate_crc16(data):
        """CRC16-CCITT校验"""
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def build_packet(msg_type, drone_id, payload=b''):
        """构建协议数据包"""
        packet = Protocol.MAGIC
        packet += bytes([msg_type])
        packet += bytes([drone_id])
        packet += struct.pack('<H', len(payload))
        packet += payload
        crc = Protocol.calculate_crc16(packet[2:])
        packet += struct.pack('<H', crc)
        return packet

    @staticmethod
    def parse_packet(data):
        """解析协议数据包"""
        if len(data) < 8:
            return None, None, None

        if data[0] != 0x55 or data[1] != 0xAA:
            return None, None, None

        msg_type = data[2]
        drone_id = data[3]
        payload_len = struct.unpack('<H', data[4:6])[0]

        total_len = 8 + payload_len
        if len(data) < total_len:
            return None, None, None

        crc_recv = struct.unpack('<H', data[total_len - 2:total_len])[0]
        crc_calc = Protocol.calculate_crc16(data[2:total_len - 2])

        if crc_recv != crc_calc:
            return None, None, None

        payload = data[6:6 + payload_len]
        return msg_type, drone_id, payload

    @staticmethod
    def build_mission_payload_v2(waypoints, global_speed=0.0):
        """
        构建V2版本的航点任务payload

        参数:
            waypoints: 航点列表，每个航点为字典包含:
                - lat: 纬度
                - lng: 经度
                - alt: 相对高度
                - hold_time: 停留时间(秒)
                - accept_radius: 接受半径(米)
                - pass_radius: 过弯半径(米)
                - yaw: 偏航角(度)
            global_speed: 全局任务速度(m/s)，0表示使用飞控默认

        返回:
            bytes: payload数据
        """
        if not waypoints or len(waypoints) > 255:
            raise ValueError("航点数量必须在1-255之间")

        payload = bytearray()

        # 协议版本 (V2)
        payload.append(Protocol.MISSION_VERSION_V2)

        # 保留字节
        payload.append(0x00)

        # 航点数量
        payload.extend(struct.pack('<H', len(waypoints)))

        # 全局任务速度
        payload.extend(struct.pack('<f', global_speed))

        # 航点列表
        for wp in waypoints:
            payload.extend(struct.pack('<f', wp['lat']))
            payload.extend(struct.pack('<f', wp['lng']))
            payload.extend(struct.pack('<f', wp['alt']))
            payload.extend(struct.pack('<f', wp.get('hold_time', 0.0)))
            payload.extend(struct.pack('<f', wp.get('accept_radius', 5.0)))
            payload.extend(struct.pack('<f', wp.get('pass_radius', 0.0)))
            payload.extend(struct.pack('<f', wp.get('yaw', float('nan'))))

        return bytes(payload)

    @staticmethod
    def build_mission_payload_v1(waypoints):
        """
        构建V1版本的航点任务payload（兼容旧版）
        """
        if not waypoints or len(waypoints) > 255:
            raise ValueError("航点数量必须在1-255之间")

        payload = bytearray()

        # 协议版本 (V1)
        payload.append(Protocol.MISSION_VERSION_V1)

        # 保留字节
        payload.append(0x00)

        # 航点数量
        payload.extend(struct.pack('<H', len(waypoints)))

        # 航点列表（无全局速度）
        for wp in waypoints:
            payload.extend(struct.pack('<f', wp['lat']))
            payload.extend(struct.pack('<f', wp['lng']))
            payload.extend(struct.pack('<f', wp['alt']))
            payload.extend(struct.pack('<f', wp.get('hold_time', 0.0)))
            payload.extend(struct.pack('<f', wp.get('accept_radius', 5.0)))
            payload.extend(struct.pack('<f', wp.get('pass_radius', 0.0)))
            payload.extend(struct.pack('<f', wp.get('yaw', float('nan'))))

        return bytes(payload)

    @staticmethod
    def parse_mission_payload(payload):
        """
        解析航点任务payload

        返回:
            dict: {
                'version': 协议版本,
                'waypoints': 航点列表,
                'global_speed': 全局速度(V2才有)
            }
        """
        if len(payload) < 4:
            return None

        offset = 0
        version = payload[offset]
        offset += 1

        reserved = payload[offset]
        offset += 1

        num_waypoints = struct.unpack('<H', payload[offset:offset + 2])[0]
        offset += 2

        result = {
            'version': version,
            'num_waypoints': num_waypoints,
            'waypoints': [],
            'global_speed': None
        }

        # V2协议有全局速度字段
        if version == Protocol.MISSION_VERSION_V2:
            if len(payload) < offset + 4:
                return None
            result['global_speed'] = struct.unpack('<f', payload[offset:offset + 4])[0]
            offset += 4

        # 解析航点
        expected_len = offset + (num_waypoints * Protocol.WAYPOINT_PAYLOAD_SIZE)
        if len(payload) < expected_len:
            return None

        for i in range(num_waypoints):
            wp = {
                'lat': struct.unpack('<f', payload[offset:offset + 4])[0],
                'lng': struct.unpack('<f', payload[offset + 4:offset + 8])[0],
                'alt': struct.unpack('<f', payload[offset + 8:offset + 12])[0],
                'hold_time': struct.unpack('<f', payload[offset + 12:offset + 16])[0],
                'accept_radius': struct.unpack('<f', payload[offset + 16:offset + 20])[0],
                'pass_radius': struct.unpack('<f', payload[offset + 20:offset + 24])[0],
                'yaw': struct.unpack('<f', payload[offset + 24:offset + 28])[0],
            }
            result['waypoints'].append(wp)
            offset += Protocol.WAYPOINT_PAYLOAD_SIZE

        return result

    @staticmethod
    def build_status_payload(armed, battery, mode, lat, lng, alt,
                             groundspeed, verticalspeed, satellites, hdop, flying=False, 
                             has_compass=None, battery_voltage=0.0, battery_current=0.0):
        """构建状态上报payload (扩展版：增加电压和电流)"""
        payload = bytearray()

        # 状态标志位
        flags = 0
        if armed:
            flags |= 0x01  # bit0: armed
        if satellites >= 3:
            flags |= 0x02  # bit1: gps_fix
        if has_compass is not None:
            if has_compass:
                flags |= 0x04
        else:
            if hdop > 0 and hdop < 2.0:
                flags |= 0x04  # bit2: good_hdop
        if flying:
            flags |= 0x08  # bit3: flying
        payload.append(flags)

        # 电池百分比 (0-100)
        battery_value = max(0, min(100, int(battery)))
        payload.append(battery_value)

        # 模式字符串 (最大16字节)
        mode_name = str(mode)[:16]
        payload.append(len(mode_name))
        payload.extend(mode_name.encode('ascii'))
        payload.extend(b'\x00' * (16 - len(mode_name)))

        # 位置数据
        payload.extend(struct.pack('<f', float(lat)))
        payload.extend(struct.pack('<f', float(lng)))
        payload.extend(struct.pack('<f', float(alt)))

        # 地面速度: 0-65535 cm/s (0-655.35 m/s)
        groundspeed_cm = max(0, min(65535, int(abs(groundspeed) * 100)))
        payload.extend(struct.pack('<H', groundspeed_cm))

        # 垂直速度: -32768 到 32767 cm/s (-327.68 到 327.67 m/s)
        verticalspeed_cm = max(-32768, min(32767, int(verticalspeed * 100)))
        payload.extend(struct.pack('<h', verticalspeed_cm))

        # GPS数据
        satellites_value = max(0, min(255, int(satellites)))
        payload.append(satellites_value)

        hdop_value = max(0, min(100, int(hdop * 10)))
        payload.append(hdop_value)

        # --- 扩展数据：电池电压(mV)和电流(cA) ---
        voltage_mv = max(0, min(65535, int(battery_voltage * 1000)))
        current_ca = max(0, min(65535, int(battery_current * 100)))
        payload.extend(struct.pack('<H', voltage_mv))
        payload.extend(struct.pack('<H', current_ca))

        expected_len = 41  # 37 + 2 + 2 = 41
        if len(payload) != expected_len:
            print(f"⚠️ 状态payload长度异常: 期望 {expected_len}, 实际 {len(payload)}")

        return bytes(payload)

    @staticmethod
    def parse_status_payload(payload):
        """解析状态上报payload (扩展版)"""
        if len(payload) < 37:
            print(f"⚠️ payload长度不足: 期望 >=37, 实际 {len(payload)}")
            return None

        offset = 0
        flags = payload[offset]
        armed = (flags & 0x01) != 0
        gps_fix = (flags & 0x02) != 0
        has_compass = (flags & 0x04) != 0
        flying = (flags & 0x08) != 0
        offset += 1

        battery = payload[offset]
        offset += 1

        mode_len = payload[offset]
        offset += 1
        mode = payload[offset:offset + mode_len].decode('ascii', errors='ignore')
        offset += 16

        lat = struct.unpack('<f', payload[offset:offset + 4])[0]
        offset += 4
        lng = struct.unpack('<f', payload[offset:offset + 4])[0]
        offset += 4
        alt = struct.unpack('<f', payload[offset:offset + 4])[0]
        offset += 4

        groundspeed = struct.unpack('<H', payload[offset:offset + 2])[0] / 100.0
        offset += 2
        verticalspeed = struct.unpack('<h', payload[offset:offset + 2])[0] / 100.0
        offset += 2

        satellites = payload[offset]
        offset += 1
        hdop = payload[offset] / 10.0
        offset += 1
        
        # 解析扩展数据
        battery_voltage = 0.0
        battery_current = 0.0
        if len(payload) >= 41:
            battery_voltage = struct.unpack('<H', payload[offset:offset + 2])[0] / 1000.0
            battery_current = struct.unpack('<H', payload[offset + 2:offset + 4])[0] / 100.0

        return {
            'armed': armed,
            'gps_fix': gps_fix,
            'has_compass': has_compass,
            'flying': flying,
            'battery': battery,
            'mode': mode,
            'lat': lat,
            'lng': lng,
            'alt': alt,
            'groundspeed': groundspeed,
            'verticalspeed': verticalspeed,
            'satellites': satellites,
            'hdop': hdop,
            'battery_voltage': battery_voltage,
            'battery_current': battery_current
        }

    @staticmethod
    def get_ack_type(cmd_type):
        """获取对应的ACK类型"""
        mapping = {
            Protocol.MsgType.ARM: Protocol.MsgType.ARM_ACK,
            Protocol.MsgType.DISARM: Protocol.MsgType.DISARM_ACK,
            Protocol.MsgType.TAKEOFF: Protocol.MsgType.TAKEOFF_ACK,
            Protocol.MsgType.SET_MODE: Protocol.MsgType.SET_MODE_ACK,
            Protocol.MsgType.LAND: Protocol.MsgType.LAND_ACK,
            Protocol.MsgType.RTH: Protocol.MsgType.RTH_ACK,
            Protocol.MsgType.GOTO: Protocol.MsgType.GOTO_ACK,
            Protocol.MsgType.MISSION_WP: Protocol.MsgType.MISSION_WP_ACK,
        }
        return mapping.get(cmd_type, 0xFF)

    @staticmethod
    def get_fail_type(cmd_type):
        """获取对应的FAIL类型"""
        mapping = {
            Protocol.MsgType.ARM: Protocol.MsgType.ARM_FAIL,
            Protocol.MsgType.DISARM: Protocol.MsgType.DISARM_FAIL,
            Protocol.MsgType.TAKEOFF: Protocol.MsgType.TAKEOFF_FAIL,
            Protocol.MsgType.SET_MODE: Protocol.MsgType.SET_MODE_FAIL,
            Protocol.MsgType.LAND: Protocol.MsgType.LAND_FAIL,
            Protocol.MsgType.RTH: Protocol.MsgType.RTH_FAIL,
            Protocol.MsgType.GOTO: Protocol.MsgType.GOTO_FAIL,
            Protocol.MsgType.MISSION_WP: Protocol.MsgType.MISSION_WP_FAIL,
        }
        return mapping.get(cmd_type, 0xFF)