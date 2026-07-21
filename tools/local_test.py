import socket
import struct
import time
import threading
from queue import Queue, Empty
import os
from datetime import datetime
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

"""
【TCP-MAVLink 飞行辅助控制客户端】

协议说明：
1. 接收 TCP 连接 (MissionPlanner插件作为服务器)
2. 发送自定义协议包来控制无人机
3. 接收飞控反馈和实时状态上报

支持的命令 (MsgType):
  0x01 = ARM (解锁)      → 反馈: 0x11=成功, 0xF1=失败
  0x02 = DISARM (上锁)   → 反馈: 0x12=成功, 0xF2=失败
  0x03 = TAKEOFF (起飞)  → 反馈: 0x13=成功, 0xF3=失败
  0x04 = SET_MODE (模式) → 反馈: 0x14=成功, 0xF4=失败
  0x05 = LAND (降落)     → 反馈: 0x15=成功, 0xF5=失败
  0x06 = RTH (返航)      → 反馈: 0x16=成功, 0xF6=失败
  0x07 = GOTO (定点)     → 反馈: 0x17=成功, 0xF7=失败
  0x20 = STATUS (状态)   → 飞控主动推送

定点飞行 (GOTO 0x07) 详解:
  Payload (16字节):
    - Latitude (float, 小端序): 十进制度数 (-90 ~ 90)
    - Longitude (float, 小端序): 十进制度数 (-180 ~ 180)
    - Altitude (float, 小端序): 相对 Home 点的目标高度（米）
    - Speed (float, 小端序): 水平速度（m/s），0表示默认5m/s
  
  执行流程:
    1. 插件接收GOTO指令
    2. 自动判断飞行模式，必要时切换到GUIDED（最多等3秒）
    3. 计算当前位置到目标位置的实际距离（Haversine公式）
    4. 基于距离和速度动态计算超时时间（10秒~5分钟）
    5. 插件内部使用 MAV_CMD_DO_REPOSITION (192) 引导飞控飞往目标点
    6. 每100ms检查一次是否到达（水平<5米且高度误差<2米）
    7. 成功: 发送0x17，失败/超时: 发送0xF7

数据包格式:
  Magic (2字节)     : 0x55AA (固定)
  MsgType (1字节)   : 消息类型
  DroneID (1字节)   : 无人机ID (0=用当前,1-255=指定)
  PayloadLen (2字节): 载荷长度 (小端序)
  Payload (可变)    : 指令参数
  Checksum (2字节)  : CRC16-CCITT (小端序)
"""


class DroneControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mission Planner 无人机控制终端")
        self.geometry("1150x900")

        # --- 核心变量 ---
        self.sock = None
        self.running = False
        self.current_drone_id = 0
        self.log_queue = Queue()
        self.status_report_count = 0

        # --- 协议常量 ---
        self.FRAME_MAGIC = b'\x55\xAA'
        self.CmdType = type('CmdType', (), {
            'ARM': 0x01, 'DISARM': 0x02, 'TAKEOFF': 0x03,
            'SET_MODE': 0x04, 'LAND': 0x05, 'RTH': 0x06,
            'GOTO': 0x07
        })
        self.CMD_NAMES = {
            self.CmdType.ARM: "解锁", self.CmdType.DISARM: "上锁",
            self.CmdType.TAKEOFF: "起飞", self.CmdType.SET_MODE: "切换模式",
            self.CmdType.LAND: "降落", self.CmdType.RTH: "返航",
            self.CmdType.GOTO: "飞往定点",
        }
        self.FEEDBACK_MAP = {
            self.CmdType.ARM: (0x11, 0xF1), self.CmdType.DISARM: (0x12, 0xF2),
            self.CmdType.TAKEOFF: (0x13, 0xF3), self.CmdType.SET_MODE: (0x14, 0xF4),
            self.CmdType.LAND: (0x15, 0xF5), self.CmdType.RTH: (0x16, 0xF6),
            self.CmdType.GOTO: (0x17, 0xF7),
        }

        # --- 日志文件配置 ---
        self.LOG_FILE_PATH = os.path.join(os.path.dirname(__file__), "client_log.txt")
        self.LOG_LOCK = threading.Lock()
        self._init_log_file()

        # --- 构建UI ---
        self._create_widgets()
        
        # --- 启动消息队列循环 ---
        self._process_queue()

        # --- 窗口关闭事件 ---
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    # ================= 核心逻辑函数 (移植并封装) =================
    def _init_log_file(self):
        try:
            with open(self.LOG_FILE_PATH, 'w', encoding='utf-8') as f:
                f.write(f"TCP 客户端日志 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*60 + "\n\n")
        except Exception as e:
            messagebox.showerror("错误", f"日志初始化失败: {e}")

    def _log_message(self, message: str):
        """线程安全的日志记录，同时写入文件和UI队列"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] {message}"
        
        # 写入文件
        try:
            with self.LOG_LOCK:
                with open(self.LOG_FILE_PATH, 'a', encoding='utf-8') as f:
                    f.write(log_line + '\n')
        except Exception as e:
            print(f"[日志写入错误] {e}")
            
        # 发送到UI队列
        self.log_queue.put(('log', log_line))

    def _get_feedback_name(self, msg_type):
        for cmd, (succ, fail) in self.FEEDBACK_MAP.items():
            if msg_type == succ: return f"{self.CMD_NAMES[cmd]}成功"
            if msg_type == fail: return f"{self.CMD_NAMES[cmd]}失败/超时"
        return f"未知消息 (0x{msg_type:02X})"

    def _calculate_crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def _build_packet(self, msg_type: int, drone_id: int, payload: bytes = b'') -> bytes:
        header = self.FRAME_MAGIC + bytes([msg_type, drone_id]) + struct.pack('<H', len(payload))
        crc = self._calculate_crc16(header + payload)
        return header + payload + struct.pack('<H', crc)

    def _parse_packet(self, buffer: bytes):
        if len(buffer) < 8: return None, "Incomplete header", buffer
        magic_idx = buffer.find(self.FRAME_MAGIC)
        if magic_idx == -1: return None, "Magic not found", buffer[-2:]
        
        buffer = buffer[magic_idx:]
        if len(buffer) < 8: return None, "Incomplete header after sync", buffer
        
        msg_type, drone_id = buffer[2], buffer[3]
        payload_len = struct.unpack('<H', buffer[4:6])[0]
        total_len = 8 + payload_len
        
        if len(buffer) < total_len: return None, "Waiting for payload", buffer
        
        packet_data = buffer[:total_len]
        recv_crc = struct.unpack('<H', packet_data[-2:])[0]
        calc_crc = self._calculate_crc16(packet_data[:-2])
        
        if recv_crc != calc_crc: return None, "CRC Mismatch", buffer[2:]
        
        return {'msg_type': msg_type, 'drone_id': drone_id, 'payload': packet_data[6:-2]}, None, buffer[total_len:]

    def _parse_status_report(self, payload: bytes):
        if len(payload) < 34: return None
        try:
            offset = 0
            flags = payload[offset]; offset +=1
            status = {
                'armed': bool(flags & 0x01), 'gps_fix': bool(flags & 0x02),
                'has_compass': bool(flags & 0x04), 'flying': bool(flags & 0x08),
                'battery': payload[offset],
            }
            offset +=1
            mode_len = payload[offset]; offset +=1
            mode_str = payload[offset:offset+16][:mode_len].decode('ascii', errors='ignore'); offset +=16
            
            status.update({
                'mode': mode_str,
                'lat': struct.unpack('<f', payload[offset:offset+4])[0],
                'lng': struct.unpack('<f', payload[offset+4:offset+8])[0],
                'alt': struct.unpack('<f', payload[offset+8:offset+12])[0],
                'groundspeed': struct.unpack('<H', payload[offset+12:offset+14])[0] / 100.0,
                'verticalspeed': struct.unpack('<h', payload[offset+14:offset+16])[0] / 100.0,
                'sat_count': payload[offset+16],
                'gps_hdop': payload[offset+17] / 10.0
            })
            return status
        except Exception as e:
            self._log_message(f"[错误] 解析状态上报失败: {e}")
            return None

    # ================= UI 布局 =================
    def _create_widgets(self):
        # 1. 顶部连接栏
        top_frame = ttk.Frame(self, padding=10)
        top_frame.pack(fill=tk.X)
        
        conn_group = ttk.LabelFrame(top_frame, text="连接设置", padding=5)
        conn_group.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        
        ttk.Label(conn_group, text="Host:").pack(side=tk.LEFT)
        self.host_entry = ttk.Entry(conn_group, width=12)
        self.host_entry.insert(0, "127.0.0.1")
        self.host_entry.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(conn_group, text="Port:").pack(side=tk.LEFT)
        self.port_entry = ttk.Entry(conn_group, width=6)
        self.port_entry.insert(0, "6000")
        self.port_entry.pack(side=tk.LEFT, padx=2)
        
        self.conn_btn = ttk.Button(conn_group, text="连接", command=self._connect_server)
        self.conn_btn.pack(side=tk.LEFT, padx=5)

        # ID 设置
        id_group = ttk.LabelFrame(top_frame, text="无人机ID", padding=5)
        id_group.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Label(id_group, text="当前ID:").pack(side=tk.LEFT)
        self.id_label = ttk.Label(id_group, text="0", foreground="blue", font=("Arial", 10, "bold"))
        self.id_label.pack(side=tk.LEFT, padx=2)
        self.id_entry = ttk.Entry(id_group, width=5)
        self.id_entry.insert(0, "0")
        self.id_entry.pack(side=tk.LEFT, padx=2)
        ttk.Button(id_group, text="设置", command=self._set_id).pack(side=tk.LEFT)

        # 2. 中间控制按钮区
        ctrl_frame = ttk.LabelFrame(self, text="控制面板", padding=10)
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)

        # 第一排：常用动作
        row1 = ttk.Frame(ctrl_frame)
        row1.pack(fill=tk.X, pady=5)
        style = ttk.Style()
        style.configure("Action.TButton", font=("Arial", 10, "bold"))
        
        ttk.Button(row1, text="自动起飞序列 (54+110)", style="Action.TButton", command=self._auto_sequence).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Separator(row1, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)
        ttk.Button(row1, text="解锁 ARM", command=lambda: self._send_cmd(self.CmdType.ARM)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(row1, text="上锁 DISARM", command=lambda: self._send_cmd(self.CmdType.DISARM)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # 第二排：高级控制 (起飞、模式、降落、返航)
        row2 = ttk.Frame(ctrl_frame)
        row2.pack(fill=tk.X, pady=5)
        
        # 起飞
        ttk.Label(row2, text="起飞高度(m):").pack(side=tk.LEFT)
        self.alt_entry = ttk.Entry(row2, width=5)
        self.alt_entry.insert(0, "10")
        self.alt_entry.pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="起飞 TAKEOFF", command=self._takeoff_cmd).pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(row2, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # 模式
        ttk.Label(row2, text="模式ID:").pack(side=tk.LEFT)
        self.mode_entry = ttk.Entry(row2, width=5)
        self.mode_entry.insert(0, "4") # GUIDED
        self.mode_entry.pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="切换模式", command=self._set_mode_cmd).pack(side=tk.LEFT, padx=5)
        
        ttk.Separator(row2, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        ttk.Button(row2, text="降落 LAND", command=lambda: self._send_cmd(self.CmdType.LAND)).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="返航 RTH", command=lambda: self._send_cmd(self.CmdType.RTH)).pack(side=tk.LEFT, padx=2)

        # 第三排：新增 - 定点飞行 (GOTO)
        row3 = ttk.Frame(ctrl_frame)
        row3.pack(fill=tk.X, pady=5)
        
        ttk.Label(row3, text="★ 定点飞行 GOTO (0x07) ★", foreground="red", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(row3, text="纬度(Lat):").pack(side=tk.LEFT)
        self.goto_lat_entry = ttk.Entry(row3, width=12)
        # 示例：西安某坐标
        self.goto_lat_entry.insert(0, "34.213413") 
        self.goto_lat_entry.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(row3, text="经度(Lng):").pack(side=tk.LEFT)
        self.goto_lng_entry = ttk.Entry(row3, width=12)
        self.goto_lng_entry.insert(0, "108.762779")
        self.goto_lng_entry.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(row3, text="高度(m):").pack(side=tk.LEFT)
        self.goto_alt_entry = ttk.Entry(row3, width=6)
        self.goto_alt_entry.insert(0, "20")
        self.goto_alt_entry.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(row3, text="速度(m/s):").pack(side=tk.LEFT)
        self.goto_speed_entry = ttk.Entry(row3, width=5)
        self.goto_speed_entry.insert(0, "5")
        self.goto_speed_entry.pack(side=tk.LEFT, padx=2)
        
        # 飞往定点按钮 - 带帮助文本
        btn_goto = ttk.Button(row3, text="执行飞往定点", command=self._goto_cmd)
        btn_goto.pack(side=tk.LEFT, padx=10)
        
        # 添加一个帮助提示按钮
        def show_goto_help():
            help_text = """【定点飞行(GOTO) 使用说明】

用途: 让无人机飞往指定的GPS坐标

参数说明:
  • 纬度: -90 ~ 90 (南为负，北为正)
  • 经度: -180 ~ 180 (西为负，东为正)
  • 高度: 相对 Home 点的目标高度（米）
  • 速度: 水平移动速度（m/s），0表示使用默认5m/s

执行流程:
  1. 插件自动判断并切换到GUIDED模式（最多3秒）
  2. 根据当前位置计算最优路径
  3. 动态计算超时时间（基于距离和速度）
  4. 插件内部使用 MAV_CMD_DO_REPOSITION (192) 引导飞控
  5. 每100ms检查一次是否到达（水平<5米且高度误差<2米）
  6. 返回成功/失败反馈

注意:
  ✓ 需要先解锁(ARM)并在GUIDED模式下使用
  ✓ 需要GPS定位且信号稳定
  ✓ GOTO 载荷仍是 4 个 float，插件内部再转换为 COMMAND_INT
  ✓ 最大超时时间为5分钟
  
提示:
  • 可以使用【自动起飞序列】快速进入GUIDED模式
  • 在状态栏查看实时GPS和高度信息"""
            messagebox.showinfo("GOTO 定点飞行帮助", help_text)
        
        ttk.Button(row3, text="帮助", command=show_goto_help).pack(side=tk.LEFT, padx=2)

        # 3. 下部分：状态与日志
        paned_window = ttk.PanedWindow(self, orient=tk.VERTICAL)
        paned_window.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 状态框
        status_frame = ttk.LabelFrame(paned_window, text="实时状态 (Status Report)", padding=5)
        self.status_text = scrolledtext.ScrolledText(status_frame, height=10, state='disabled', font=("Consolas", 9))
        self.status_text.pack(fill=tk.BOTH, expand=True)
        paned_window.add(status_frame, weight=1)

        # 日志框
        log_frame = ttk.LabelFrame(paned_window, text="通信日志 (Log)", padding=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        paned_window.add(log_frame, weight=2)

    # ================= 交互逻辑 =================
    def _connect_server(self):
        if self.sock:
            messagebox.showinfo("提示", "已连接")
            return
        
        host = self.host_entry.get()
        try:
            port = int(self.port_entry.get())
        except:
            messagebox.showerror("错误", "端口无效")
            return

        try:
            self._log_message(f"正在连接 {host}:{port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            self.running = True
            self.conn_btn.config(state="disabled", text="已连接")
            self._log_message("连接成功！")
            
            # 启动接收线程
            threading.Thread(target=self._recv_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            self.sock = None

    def _set_id(self):
        try:
            new_id = int(self.id_entry.get())
            if 0 <= new_id <= 255:
                self.current_drone_id = new_id
                self.id_label.config(text=str(new_id))
                self._log_message(f"当前无人机ID设为: {new_id}")
            else:
                messagebox.showwarning("警告", "ID范围 0-255")
        except:
            pass

    def _send_cmd(self, cmd_type, drone_id=None, payload=b''):
        if not self.sock:
            messagebox.showwarning("警告", "请先连接服务器")
            return
        
        d_id = drone_id if drone_id is not None else self.current_drone_id
        try:
            pkt = self._build_packet(cmd_type, d_id, payload)
            name = self.CMD_NAMES.get(cmd_type, "未知")
            self._log_message(f"[发送] {name} -> ID:{d_id} | Data:{pkt.hex()}")
            self.sock.sendall(pkt)
        except Exception as e:
            self._log_message(f"[发送失败] {e}")

    def _get_takeoff_altitude(self):
        try:
            alt = float(self.alt_entry.get())
        except ValueError:
            raise ValueError("起飞高度必须是数字")

        if alt <= 0:
            raise ValueError("起飞高度必须大于 0")

        return alt

    def _takeoff_cmd(self):
        try:
            alt = self._get_takeoff_altitude()
            self._send_cmd(self.CmdType.TAKEOFF, payload=struct.pack('<f', alt))
        except ValueError as e:
            messagebox.showerror("错误", str(e))

    def _set_mode_cmd(self):
        try:
            mode_id = int(self.mode_entry.get())
            self._send_cmd(self.CmdType.SET_MODE, payload=bytes([mode_id]))
        except:
            messagebox.showerror("错误", "模式ID无效")

    def _goto_cmd(self):
        """发送定点飞行命令 (GOTO 0x07)"""
        try:
            lat = float(self.goto_lat_entry.get())
            lng = float(self.goto_lng_entry.get())
            alt = float(self.goto_alt_entry.get())
            speed = float(self.goto_speed_entry.get())
            
            if not (-90 <= lat <= 90):
                messagebox.showwarning("警告", "纬度范围: -90 ~ 90")
                return
            if not (-180 <= lng <= 180):
                messagebox.showwarning("警告", "经度范围: -180 ~ 180")
                return
            if alt < 0:
                messagebox.showwarning("警告", "高度不能为负数")
                return
            if speed < 0:
                messagebox.showwarning("警告", "速度不能为负数")
                return
            
            if speed == 0:
                self._log_message("[GOTO] 警告: 速度为0，插件将使用默认速度5m/s")
            
            payload = struct.pack('<ffff', lat, lng, alt, speed)
            
            self._log_message(f"[GOTO] 目标坐标: 纬={lat:.6f}, 经={lng:.6f}, 相对高度={alt:.1f}m, 速度={speed:.1f}m/s")
            self._log_message(f"[GOTO] Payload (hex): {payload.hex()}")
            self._send_cmd(self.CmdType.GOTO, payload=payload)
            self._log_message("[GOTO] 命令已发送，等待飞往定点...")
            
        except ValueError as e:
            messagebox.showerror("错误", f"坐标输入格式错误: {e}\n必须输入数字（可以是小数）")
        except Exception as e:
            messagebox.showerror("错误", f"发送GOTO命令失败: {e}")

    def _auto_sequence(self):
        if not self.sock: return
        def task():
            ids = [54, 110]
            try:
                alt = self._get_takeoff_altitude()
            except ValueError as e:
                self._log_message(f"[自动起飞] 参数错误: {e}")
                self.log_queue.put(('log', "[自动起飞] 已取消"))
                return

            self._log_message("========== 开始执行自动起飞序列 ==========")
            for d_id in ids:
                self._log_message(f"--- 处理无人机 {d_id} ---")
                self._send_cmd(self.CmdType.SET_MODE, d_id, bytes([4]))
                time.sleep(1)
                self._send_cmd(self.CmdType.ARM, d_id)
                time.sleep(1)
                self._send_cmd(self.CmdType.TAKEOFF, d_id, struct.pack('<f', alt))
                time.sleep(1)
            self._log_message("========== 序列指令发送完毕 ==========")
        threading.Thread(target=task, daemon=True).start()

    # ================= 后台接收 =================
    def _recv_loop(self):
        buffer = b''
        while self.running:
            try:
                if not self.sock: break
                self.sock.settimeout(0.5)
                data = self.sock.recv(4096)
                if not data:
                    self.log_queue.put(('log', "[系统] 服务器断开连接"))
                    break
                buffer += data
                
                while True:
                    pkt, err, buffer = self._parse_packet(buffer)
                    if pkt:
                        self._handle_pkt(pkt)
                    elif err and "Waiting" not in err and "Incomplete" not in err:
                        pass # 忽略非关键错误
                    else:
                        break
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.log_queue.put(('log', f"[接收错误] {e}"))
                break

    def _handle_pkt(self, pkt):
        msg_type = pkt['msg_type']
        drone_id = pkt['drone_id']
        
        if msg_type == 0x20: # 状态上报
            status = self._parse_status_report(pkt['payload'])
            self.status_report_count += 1
            if status:
                self.log_queue.put(('status', status, drone_id, self.status_report_count))
                # 简略日志
                self._log_message(f"[状态] ID:{drone_id} Mode:{status['mode']} Bat:{status['battery']}% Alt:{status['alt']:.1f}m")
        else:
            name = self._get_feedback_name(msg_type)
            
            if msg_type == 0x17:
                self._log_message(f"🎯 [定点成功] ID:{drone_id} | {name} | 无人机已到达目标位置!")
            elif msg_type == 0xF7:
                self._log_message(f"❌ [定点失败] ID:{drone_id} | {name} | 定点飞行失败或超时")
            else:
                self._log_message(f"[反馈] ID:{drone_id} | {name} | Payload:{pkt['payload'].hex()}")

    # ================= UI 更新队列 =================
    def _process_queue(self):
        try:
            while True:
                item = self.log_queue.get_nowait()
                if item[0] == 'log':
                    self._append_log(item[1])
                elif item[0] == 'status':
                    self._update_status(item[1], item[2], item[3])
        except Empty:
            pass
        self.after(100, self._process_queue)

    def _append_log(self, text):
        self.log_text.insert(tk.END, text + '\n')
        self.log_text.see(tk.END)

    def _update_status(self, status, d_id, count):
        txt = (
            f"━━━ 上报 #{count} | 无人机: {d_id} ━━━\n"
            f"  系统状态: [解锁:{'✅' if status['armed'] else '❌'}] "
            f"[飞行:{'✈️' if status['flying'] else '🏠'}] "
            f"[GPS:{'OK' if status['gps_fix'] else 'NO'}]\n"
            f"  模式: {status['mode']:10} | 电池: {status['battery']:3}%\n"
            f"  位置: 纬 {status['lat']:.6f}, 经 {status['lng']:.6f}\n"
            f"  高度: {status['alt']:.2f} m | 速度(H/V): {status['groundspeed']:.2f}/{status['verticalspeed']:.2f} m/s\n"
            f"  卫星数: {status['sat_count']} | HDOP: {status['gps_hdop']:.1f}\n"
        )
        self.status_text.config(state='normal')
        self.status_text.insert(tk.END, txt)
        self.status_text.see(tk.END)
        self.status_text.config(state='disabled')

    def _on_closing(self):
        self.running = False
        if self.sock:
            try: self.sock.close()
            except: pass
        self.destroy()

if __name__ == "__main__":
    app = DroneControlApp()
    app.mainloop()
