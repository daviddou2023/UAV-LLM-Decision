import socket
import struct
import time
import threading
import math
import copy
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from datetime import datetime


class TestProtocol:
    MAGIC = b'\x55\xAA'

    class MsgType:
        ARM = 0x01
        DISARM = 0x02
        TAKEOFF = 0x03
        SET_MODE = 0x04
        LAND = 0x05
        RTH = 0x06
        GOTO = 0x07
        MISSION_WP = 0x08

    MODE_OPTIONS = [
        ("STABILIZE", 0), ("ACRO", 1), ("ALT_HOLD", 2), ("AUTO", 3),
        ("GUIDED", 4), ("LOITER", 5), ("RTL", 6), ("CIRCLE", 7),
        ("POSITION", 8), ("LAND", 9), ("OF_LOITER", 10), ("DRIFT", 11),
        ("SPORT", 13), ("FLIP", 14), ("AUTOTUNE", 15), ("POSHOLD", 16),
        ("BRAKE", 17), ("THROW", 18), ("SMART_RTL", 21), ("FOLLOW", 23),
    ]

    @staticmethod
    def calculate_crc16(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def build_packet(msg_type, drone_id=1, payload=b''):
        packet = bytearray()
        packet += TestProtocol.MAGIC
        packet += bytes([msg_type])
        packet += bytes([drone_id & 0xFF])
        packet += struct.pack('<H', len(payload))
        packet += payload
        crc = TestProtocol.calculate_crc16(packet[2:])
        packet += struct.pack('<H', crc)
        return bytes(packet)

    @staticmethod
    def build_mission_payload(waypoints, global_speed=5.0):
        if not waypoints or len(waypoints) > 255:
            raise ValueError("航点数量必须在1-255之间")

        payload = bytearray()
        payload.append(0x02)
        payload.append(0x01)
        payload.extend(struct.pack('<H', len(waypoints)))
        payload.extend(struct.pack('<f', global_speed))

        for wp in waypoints:
            
            payload.extend(struct.pack('<f', wp['lat']))
            payload.extend(struct.pack('<f', wp['lng']))
           

            payload.extend(struct.pack('<f', wp['alt']))
            payload.extend(struct.pack('<f', wp.get('hold_time', 0.0)))
            payload.extend(struct.pack('<f', wp.get('accept_radius', 5.0)))
            payload.extend(struct.pack('<f', wp.get('pass_radius', 0.0)))
            payload.extend(struct.pack('<f', wp.get('yaw', 0.0)))

        return bytes(payload)

    @staticmethod
    def build_goto_payload(lat, lng, alt, speed):
        return struct.pack('<ffff', lat, lng, alt, speed)


class WaypointEditor:
    def __init__(self, parent, existing_waypoints=None):
        self.dialog = tk.Toplevel(parent)
        self.dialog.title("航点任务编辑器")
        self.dialog.geometry("900x650")
        self.dialog.minsize(850, 600)
        self.dialog.transient(parent)
        self.dialog.grab_set()

        self.waypoints = copy.deepcopy(existing_waypoints) if existing_waypoints else []
        self.result = None
        self._build_ui()
        self._update_list()

    def _build_ui(self):
        main_frame = ttk.Frame(self.dialog, padding="10")
        main_frame.pack(fill="both", expand=True)

    # 中间内容区
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(side="top", fill="both", expand=True)

    # 底部按钮区，必须单独 pack 到 bottom
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(side="bottom", fill="x", pady=(10, 0))

    # 左侧 - 航点列表
        left_frame = ttk.LabelFrame(content_frame, text="航点列表")
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        columns = ("seq", "lat", "lng", "alt", "hold_time", "accept_radius")
        self.tree = ttk.Treeview(left_frame, columns=columns, show="headings", height=15)

        self.tree.heading("seq", text="序号")
        self.tree.heading("lat", text="纬度")
        self.tree.heading("lng", text="经度")
        self.tree.heading("alt", text="高度(m)")
        self.tree.heading("hold_time", text="停留时间(s)")
        self.tree.heading("accept_radius", text="接受半径(m)")

        self.tree.column("seq", width=60, anchor="center")
        self.tree.column("lat", width=120, anchor="center")
        self.tree.column("lng", width=120, anchor="center")
        self.tree.column("alt", width=80, anchor="center")
        self.tree.column("hold_time", width=90, anchor="center")
        self.tree.column("accept_radius", width=90, anchor="center")

        scrollbar = ttk.Scrollbar(left_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)

        self.tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    # 右侧 - 编辑区域
        right_frame = ttk.LabelFrame(content_frame, text="编辑航点")
        right_frame.pack(side="right", fill="y", padx=(5, 0))

        form_frame = ttk.Frame(right_frame, padding="10")
        form_frame.pack(fill="x")

        ttk.Label(form_frame, text="纬度:").grid(row=0, column=0, sticky="w", pady=5)
        self.lat_var = tk.StringVar()
        ttk.Entry(form_frame, textvariable=self.lat_var, width=15).grid(row=0, column=1, padx=5)

        ttk.Label(form_frame, text="经度:").grid(row=1, column=0, sticky="w", pady=5)
        self.lng_var = tk.StringVar()
        ttk.Entry(form_frame, textvariable=self.lng_var, width=15).grid(row=1, column=1, padx=5)

        ttk.Label(form_frame, text="高度(m):").grid(row=2, column=0, sticky="w", pady=5)
        self.alt_var = tk.StringVar(value="20")
        ttk.Entry(form_frame, textvariable=self.alt_var, width=15).grid(row=2, column=1, padx=5)

        ttk.Label(form_frame, text="停留时间(s):").grid(row=3, column=0, sticky="w", pady=5)
        self.hold_time_var = tk.StringVar(value="1.0")
        ttk.Entry(form_frame, textvariable=self.hold_time_var, width=15).grid(row=3, column=1, padx=5)

        ttk.Label(form_frame, text="接受半径(m):").grid(row=4, column=0, sticky="w", pady=5)
        self.radius_var = tk.StringVar(value="5.0")
        ttk.Entry(form_frame, textvariable=self.radius_var, width=15).grid(row=4, column=1, padx=5)

        btn_frame = ttk.Frame(right_frame, padding="10")
        btn_frame.pack(fill="x")

        ttk.Button(btn_frame, text="添加/更新", command=self._add_or_update).pack(fill="x", pady=2)
        ttk.Button(btn_frame, text="删除选中", command=self._delete_selected).pack(fill="x", pady=2)
        ttk.Button(btn_frame, text="清空所有", command=self._clear_all).pack(fill="x", pady=2)

        io_frame = ttk.LabelFrame(right_frame, text="导入/导出", padding="10")
        io_frame.pack(fill="x", pady=10)

        ttk.Button(io_frame, text="导入 CSV", command=self._import_csv).pack(fill="x", pady=2)
        ttk.Button(io_frame, text="导出 CSV", command=self._export_csv).pack(fill="x", pady=2)

    # 底部按钮
        ttk.Label(bottom_frame, text="全局速度(m/s):").pack(side="left", padx=5)
        self.global_speed_var = tk.StringVar(value="5.0")
        ttk.Entry(bottom_frame, textvariable=self.global_speed_var, width=8).pack(side="left", padx=5)

        ttk.Button(bottom_frame, text="取消", command=self.dialog.destroy).pack(side="right", padx=5)
        ttk.Button(bottom_frame, text="确认上传", command=self._confirm).pack(side="right", padx=5)

    def _validate_waypoint(self, lat, lng, alt):
        if not (-90 <= lat <= 90):
            raise ValueError("纬度必须在 -90 到 90 之间")
        if not (-180 <= lng <= 180):
            raise ValueError("经度必须在 -180 到 180 之间")
        if lat == 0 or lng == 0:
            raise ValueError("经纬度不能为 0")
        if alt <= 0:
            raise ValueError("高度必须大于 0")

    def _update_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

        for i, wp in enumerate(self.waypoints):
            self.tree.insert("", "end", values=(
                i + 1,
                f"{wp['lat']:.7f}",
                f"{wp['lng']:.7f}",
                wp['alt'],
                wp.get('hold_time', 0),
                wp.get('accept_radius', 5),
            ))

    def _add_or_update(self):
        try:
            lat = float(self.lat_var.get())
            lng = float(self.lng_var.get())
            alt = float(self.alt_var.get())
            hold_time = float(self.hold_time_var.get())
            radius = float(self.radius_var.get())
            self._validate_waypoint(lat, lng, alt)

            wp = {
                'lat': lat,
                'lng': lng,
                'alt': alt,
                'hold_time': hold_time,
                'accept_radius': radius,
            }

            selected = self.tree.selection()
            if selected:
                idx = self.tree.index(selected[0])
                self.waypoints[idx] = wp
            else:
                self.waypoints.append(wp)

            self._update_list()
            self.lat_var.set("")
            self.lng_var.set("")
            self.alt_var.set("20")
            self.hold_time_var.set("1.0")
            self.radius_var.set("5.0")

        except ValueError as e:
            messagebox.showerror("错误", f"参数格式错误: {e}")

    def _delete_selected(self):
        selected = list(self.tree.selection())
        for item in reversed(selected):
            idx = self.tree.index(item)
            del self.waypoints[idx]
        self._update_list()

    def _clear_all(self):
        if messagebox.askyesno("确认", "确定要清空所有航点吗？"):
            self.waypoints = []
            self._update_list()

    def _import_csv(self):
        from tkinter import filedialog
        filename = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not filename:
            return

        try:
            waypoints = []
            with open(filename, 'r', encoding="utf-8") as f:
                lines = f.readlines()

            for i, line in enumerate(lines):
                line = line.strip()
                if not line:
                    continue
                if i == 0 and line.lower().startswith('lat'):
                    continue

                parts = line.split(',')
                if len(parts) < 3:
                    continue

                lat = float(parts[0])
                lng = float(parts[1])
                alt = float(parts[2])
                self._validate_waypoint(lat, lng, alt)

                waypoints.append({
                    'lat': lat,
                    'lng': lng,
                    'alt': alt,
                    'hold_time': float(parts[3]) if len(parts) > 3 else 1.0,
                    'accept_radius': float(parts[4]) if len(parts) > 4 else 5.0,
                })

            self.waypoints = waypoints
            self._update_list()
            messagebox.showinfo("成功", f"已导入 {len(self.waypoints)} 个航点")
        except Exception as e:
            messagebox.showerror("错误", f"导入失败: {e}")

    def _export_csv(self):
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")]
        )
        if not filename:
            return

        try:
            with open(filename, 'w', encoding="utf-8") as f:
                f.write("lat,lng,alt,hold_time,accept_radius\n")
                for wp in self.waypoints:
                    f.write(
                        f"{wp['lat']},{wp['lng']},{wp['alt']},"
                        f"{wp.get('hold_time', 0)},{wp.get('accept_radius', 5)}\n"
                    )
            messagebox.showinfo("成功", f"已导出 {len(self.waypoints)} 个航点")
        except Exception as e:
            messagebox.showerror("错误", f"导出失败: {e}")

    def _confirm(self):
        if not self.waypoints:
            messagebox.showwarning("警告", "请至少添加一个航点")
            return

        try:
            global_speed = float(self.global_speed_var.get())
        except ValueError:
            global_speed = 5.0

        self.result = {
            'waypoints': copy.deepcopy(self.waypoints),
            'global_speed': global_speed,
        }
        self.dialog.destroy()


class MissionStatusWindow:
    def __init__(self, parent, drone_id):
        self.window = tk.Toplevel(parent)
        self.window.title(f"无人机 {drone_id} - 航点任务状态")
        self.window.geometry("600x400")
        self.drone_id = drone_id
        self.parent = parent
        self.row_by_seq = {}
        self._build_ui()

    def _build_ui(self):
        self.status_label = ttk.Label(self.window, text="等待任务...", font=("Arial", 14))
        self.status_label.pack(pady=10)

        self.progress = ttk.Progressbar(self.window, length=400, mode='determinate')
        self.progress.pack(pady=10)

        columns = ("seq", "lat", "lng", "alt", "status")
        self.tree = ttk.Treeview(self.window, columns=columns, show="headings", height=8)

        for col, text in {
            "seq": "航点",
            "lat": "纬度",
            "lng": "经度",
            "alt": "高度",
            "status": "状态",
        }.items():
            self.tree.heading(col, text=text)
            self.tree.column(col, width=100, anchor="center")

        self.tree.pack(fill="both", expand=True, padx=10, pady=10)
        ttk.Button(self.window, text="关闭", command=self.window.destroy).pack(pady=10)

    def update_progress(self, current, total):
        self.progress['maximum'] = max(total, 1)
        self.progress['value'] = min(current, total)
        self.status_label.config(text=f"进度: {current}/{total} 航点")

    def mark_reached(self, seq):
        item = self.row_by_seq.get(seq)
        if item:
            values = list(self.tree.item(item)['values'])
            values[4] = "✅ 已完成"
            self.tree.item(item, values=values)

    def mark_current(self, seq):
        item = self.row_by_seq.get(seq)
        if item:
            values = list(self.tree.item(item)['values'])
            if values[4] != "✅ 已完成":
                values[4] = "🚁 执行中"
                self.tree.item(item, values=values)

    def set_waypoints(self, waypoints):
        for item in self.tree.get_children():
            self.tree.delete(item)

        self.row_by_seq.clear()

        for i, wp in enumerate(waypoints, 1):
            item = self.tree.insert("", "end", values=(
                i,
                f"{wp['lat']:.7f}",
                f"{wp['lng']:.7f}",
                wp['alt'],
                "⏳ 等待中",
            ))
            self.row_by_seq[i] = item

        self.update_progress(0, len(waypoints))


class TestClientGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("多无人机 TCP-MAVLink 控制面板")
        self.root.geometry("1500x950")

        self.sock = None
        self.running = False
        self.recv_thread = None
        self.recv_buffer = b""

        self.drone_states = {}
        self.home_alt_abs = {}
        self.current_drone_id = 1
        self.drone_checkboxes = {}
        self.mission_windows = {}
        self.current_waypoints = []

        self.sock_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        conn_frame = ttk.LabelFrame(self.root, text="连接设置")
        conn_frame.pack(fill="x", padx=10, pady=5)

        row = ttk.Frame(conn_frame)
        row.pack(fill="x", padx=5, pady=5)

        ttk.Label(row, text="服务器IP:").pack(side="left", padx=5)
        self.host_var = tk.StringVar(value="127.0.0.1")
        ttk.Entry(row, textvariable=self.host_var, width=15).pack(side="left", padx=5)

        ttk.Label(row, text="端口:").pack(side="left", padx=5)
        self.port_var = tk.StringVar(value="6001")
        ttk.Entry(row, textvariable=self.port_var, width=8).pack(side="left", padx=5)

        self.connect_btn = ttk.Button(row, text="连接", command=self.connect)
        self.connect_btn.pack(side="left", padx=10)

        self.disconnect_btn = ttk.Button(row, text="断开", command=self.disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(row, text="● 未连接", foreground="red")
        self.status_label.pack(side="left", padx=10)

        ttk.Button(row, text="刷新状态", command=self.refresh_status).pack(side="right", padx=5)
        ttk.Button(row, text="清空日志", command=self.clear_log).pack(side="right", padx=5)

        multi_frame = ttk.LabelFrame(self.root, text="多机选择与批量控制")
        multi_frame.pack(fill="x", padx=10, pady=5)

        left_frame = ttk.Frame(multi_frame)
        left_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        ttk.Label(left_frame, text="已发现无人机:").pack(anchor="w")

        drone_list_frame = ttk.Frame(left_frame)
        drone_list_frame.pack(fill="both", expand=True)

        self.drone_canvas = tk.Canvas(drone_list_frame, height=100)
        scrollbar = ttk.Scrollbar(drone_list_frame, orient="vertical", command=self.drone_canvas.yview)
        self.drone_frame = ttk.Frame(self.drone_canvas)
        self.drone_canvas.configure(yscrollcommand=scrollbar.set)

        self.drone_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.drone_canvas.create_window((0, 0), window=self.drone_frame, anchor="nw")
        self.drone_frame.bind(
            "<Configure>",
            lambda e: self.drone_canvas.configure(scrollregion=self.drone_canvas.bbox("all"))
        )

        select_frame = ttk.Frame(left_frame)
        select_frame.pack(fill="x", pady=5)
        ttk.Button(select_frame, text="全选", command=self.select_all_drones).pack(side="left", padx=2)
        ttk.Button(select_frame, text="取消全选", command=self.deselect_all_drones).pack(side="left", padx=2)

        right_frame = ttk.Frame(multi_frame)
        right_frame.pack(side="right", fill="both", expand=True, padx=10)

        ttk.Label(right_frame, text="批量命令", font=("Arial", 10, "bold")).pack(anchor="w")

        batch_btn_frame = ttk.Frame(right_frame)
        batch_btn_frame.pack(fill="x", pady=5)

        ttk.Button(batch_btn_frame, text="🔓 批量解锁", command=self.batch_arm).pack(side="left", padx=5)
        ttk.Button(batch_btn_frame, text="🔒 批量上锁", command=self.batch_disarm).pack(side="left", padx=5)
        ttk.Button(batch_btn_frame, text="🚁 批量起飞", command=self.batch_takeoff).pack(side="left", padx=5)
        ttk.Button(batch_btn_frame, text="🛬 批量降落", command=self.batch_land).pack(side="left", padx=5)
        ttk.Button(batch_btn_frame, text="🏠 批量返航", command=self.batch_rth).pack(side="left", padx=5)
        ttk.Button(batch_btn_frame, text="🔄 批量切换模式", command=self.batch_set_mode).pack(side="left", padx=5)

        single_frame = ttk.LabelFrame(self.root, text="单机控制")
        single_frame.pack(fill="x", padx=10, pady=5)

        target_frame = ttk.Frame(single_frame)
        target_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(target_frame, text="目标无人机ID:").pack(side="left", padx=5)
        self.target_drone_var = tk.StringVar(value="1")
        self.drone_combo = ttk.Combobox(
            target_frame,
            textvariable=self.target_drone_var,
            values=["1"],
            width=8,
            state="readonly",
        )
        self.drone_combo.pack(side="left", padx=5)
        ttk.Button(target_frame, text="设置目标", command=self.set_target_drone).pack(side="left", padx=5)

        cmd_frame = ttk.Frame(single_frame)
        cmd_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(cmd_frame, text="模式:").pack(side="left", padx=5)
        self.mode_var = tk.StringVar(value="GUIDED (4)")
        mode_combo = ttk.Combobox(
            cmd_frame,
            textvariable=self.mode_var,
            values=[f"{n} ({m})" for n, m in TestProtocol.MODE_OPTIONS],
            width=18,
            state="readonly",
        )
        mode_combo.pack(side="left", padx=5)
        ttk.Button(cmd_frame, text="切换模式", command=self.send_set_mode).pack(side="left", padx=5)
        ttk.Button(cmd_frame, text="解锁", command=self.send_arm).pack(side="left", padx=5)
        ttk.Button(cmd_frame, text="上锁", command=self.send_disarm).pack(side="left", padx=5)

        ttk.Label(cmd_frame, text="起飞高度(m):").pack(side="left", padx=5)
        self.takeoff_alt_var = tk.StringVar(value="10")
        ttk.Entry(cmd_frame, textvariable=self.takeoff_alt_var, width=8).pack(side="left", padx=2)
        ttk.Button(cmd_frame, text="起飞", command=self.send_takeoff).pack(side="left", padx=5)

        ttk.Button(cmd_frame, text="降落", command=self.send_land).pack(side="left", padx=5)
        ttk.Button(cmd_frame, text="返航", command=self.send_rth).pack(side="left", padx=5)

        row2 = ttk.Frame(single_frame)
        row2.pack(fill="x", padx=5, pady=5)

        ttk.Label(row2, text="绝对 GOTO:").pack(side="left", padx=5)
        ttk.Label(row2, text="纬度:").pack(side="left", padx=5)
        self.goto_lat_var = tk.StringVar(value="34.213413")
        ttk.Entry(row2, textvariable=self.goto_lat_var, width=12).pack(side="left", padx=2)

        ttk.Label(row2, text="经度:").pack(side="left", padx=5)
        self.goto_lng_var = tk.StringVar(value="108.762779")
        ttk.Entry(row2, textvariable=self.goto_lng_var, width=12).pack(side="left", padx=2)

        ttk.Label(row2, text="高度(m):").pack(side="left", padx=5)
        self.goto_alt_var = tk.StringVar(value="20")
        ttk.Entry(row2, textvariable=self.goto_alt_var, width=8).pack(side="left", padx=2)

        ttk.Label(row2, text="速度(m/s):").pack(side="left", padx=5)
        self.goto_speed_var = tk.StringVar(value="5")
        ttk.Entry(row2, textvariable=self.goto_speed_var, width=8).pack(side="left", padx=2)

        ttk.Button(row2, text="绝对坐标飞行", command=self.send_goto).pack(side="left", padx=10)

        row3 = ttk.Frame(single_frame)
        row3.pack(fill="x", padx=5, pady=5)

        ttk.Label(row3, text="相对 GOTO:").pack(side="left", padx=5)
        ttk.Label(row3, text="方位角(°):").pack(side="left", padx=5)
        self.rel_angle_var = tk.StringVar(value="0")
        ttk.Entry(row3, textvariable=self.rel_angle_var, width=8).pack(side="left", padx=2)

        ttk.Label(row3, text="距离(m):").pack(side="left", padx=5)
        self.rel_dist_var = tk.StringVar(value="10")
        ttk.Entry(row3, textvariable=self.rel_dist_var, width=8).pack(side="left", padx=2)

        ttk.Label(row3, text="高度(m):").pack(side="left", padx=5)
        self.rel_alt_var = tk.StringVar(value="20")
        ttk.Entry(row3, textvariable=self.rel_alt_var, width=8).pack(side="left", padx=2)

        ttk.Button(row3, text="相对位移飞行", command=self.send_relative_goto).pack(side="left", padx=10)

        mission_frame = ttk.LabelFrame(self.root, text="航点任务")
        mission_frame.pack(fill="x", padx=10, pady=5)

        mission_control = ttk.Frame(mission_frame)
        mission_control.pack(fill="x", padx=5, pady=5)

        ttk.Label(mission_control, text="全局速度(m/s):").pack(side="left", padx=5)
        self.mission_speed_var = tk.StringVar(value="5.0")
        ttk.Entry(mission_control, textvariable=self.mission_speed_var, width=8).pack(side="left", padx=5)

        ttk.Button(mission_control, text="📝 编辑航点任务", command=self.edit_mission).pack(side="left", padx=10)
        ttk.Button(mission_control, text="📤 上传航点任务", command=self.upload_mission_to_target).pack(side="left", padx=5)
        ttk.Button(mission_control, text="📊 查看任务状态", command=self.show_mission_status).pack(side="left", padx=5)

        ttk.Label(mission_control, text="正方形航点:").pack(side="left", padx=10)
        ttk.Label(mission_control, text="边长(m):").pack(side="left", padx=5)
        self.square_size_var = tk.StringVar(value="50")
        ttk.Entry(mission_control, textvariable=self.square_size_var, width=8).pack(side="left", padx=2)

        ttk.Label(mission_control, text="高度(m):").pack(side="left", padx=5)
        self.square_alt_var = tk.StringVar(value="20")
        ttk.Entry(mission_control, textvariable=self.square_alt_var, width=8).pack(side="left", padx=2)

        ttk.Button(mission_control, text="生成正方形航点", command=self.create_square_mission).pack(side="left", padx=10)

        preview_frame = ttk.LabelFrame(mission_frame, text="航点预览")
        preview_frame.pack(fill="both", expand=True, padx=5, pady=5)

        columns = ("seq", "lat", "lng", "alt", "hold_time", "radius")
        self.mission_preview = ttk.Treeview(preview_frame, columns=columns, show="headings", height=4)

        for col, text in {
            "seq": "序号",
            "lat": "纬度",
            "lng": "经度",
            "alt": "高度(m)",
            "hold_time": "停留(s)",
            "radius": "半径(m)",
        }.items():
            self.mission_preview.heading(col, text=text)
            self.mission_preview.column(col, width=120, anchor="center")

        self.mission_preview.pack(side="left", fill="both", expand=True)

        preview_scroll = ttk.Scrollbar(preview_frame, orient="vertical", command=self.mission_preview.yview)
        preview_scroll.pack(side="right", fill="y")
        self.mission_preview.configure(yscrollcommand=preview_scroll.set)

        status_frame = ttk.LabelFrame(self.root, text="无人机状态")
        status_frame.pack(fill="x", padx=10, pady=5)

        columns = (
            "drone_id", "mode", "armed", "flying", "battery", "alt",
            "lat", "lng", "speed", "gps", "mission_status", "updated"
        )
        self.status_tree = ttk.Treeview(status_frame, columns=columns, show="headings", height=5)

        headings = {
            "drone_id": "ID",
            "mode": "模式",
            "armed": "解锁",
            "flying": "飞行",
            "battery": "电池 (V/A)",
            "alt": "高度",
            "lat": "纬度",
            "lng": "经度",
            "speed": "速度",
            "gps": "GPS",
            "mission_status": "任务状态",
            "updated": "更新时间",
        }

        widths = {
            "drone_id": 50,
            "mode": 80,
            "armed": 60,
            "flying": 60,
            "battery": 120,
            "alt": 70,
            "lat": 110,
            "lng": 110,
            "speed": 100,
            "gps": 80,
            "mission_status": 100,
            "updated": 120,
        }

        for col in columns:
            self.status_tree.heading(col, text=headings[col])
            self.status_tree.column(col, width=widths.get(col, 80), anchor="center")

        self.status_tree.pack(fill="both", expand=True, side="left")
        status_scroll = ttk.Scrollbar(status_frame, orient="vertical", command=self.status_tree.yview)
        status_scroll.pack(side="right", fill="y")
        self.status_tree.configure(yscrollcommand=status_scroll.set)
        self.status_rows = {}

        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=15, wrap="word")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

        self.bottom_label = ttk.Label(self.root, text="就绪", relief="sunken")
        self.bottom_label.pack(fill="x", padx=10, pady=2)

        self.root.after(1000, self.update_ui)

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete(1.0, "end")
        self.log_text.configure(state="disabled")

    def refresh_status(self):
        self.update_drone_list()
        self.update_status_tree()

    def get_selected_drones(self):
        return [drone_id for drone_id, var in self.drone_checkboxes.items() if var.get()]

    def select_all_drones(self):
        for var in self.drone_checkboxes.values():
            var.set(True)

    def deselect_all_drones(self):
        for var in self.drone_checkboxes.values():
            var.set(False)

    def set_target_drone(self):
        try:
            drone_id = int(self.target_drone_var.get())
            self.current_drone_id = drone_id
            self.log(f"🎯 当前目标无人机已切换为 DroneID={drone_id}")
        except ValueError:
            self.log("❌ 无人机 ID 格式错误")

    def update_drone_list(self):
        old_selected = {
            drone_id: var.get()
            for drone_id, var in self.drone_checkboxes.items()
        }

        with self.state_lock:
            drone_ids = sorted(self.drone_states.keys())
            states_snapshot = copy.deepcopy(self.drone_states)

        values = [str(did) for did in drone_ids] if drone_ids else ["1"]
        self.drone_combo["values"] = values

        if drone_ids:
            if self.current_drone_id not in drone_ids:
                self.current_drone_id = drone_ids[0]
            self.target_drone_var.set(str(self.current_drone_id))
        else:
            self.target_drone_var.set(str(self.current_drone_id))

        for widget in self.drone_frame.winfo_children():
            widget.destroy()

        self.drone_checkboxes.clear()

        for i, drone_id in enumerate(drone_ids):
            var = tk.BooleanVar(value=old_selected.get(drone_id, True))
            self.drone_checkboxes[drone_id] = var

            state = states_snapshot.get(drone_id, {})
            mode = state.get('mode', 'UNKNOWN')
            armed = "🔓" if state.get('armed') else "🔒"

            cb = ttk.Checkbutton(
                self.drone_frame,
                text=f"无人机 {drone_id} [{mode}] {armed}",
                variable=var,
            )
            cb.grid(row=i // 4, column=i % 4, sticky="w", padx=10, pady=2)

    def update_status_tree(self):
        with self.state_lock:
            items = list(self.drone_states.items())

        for drone_id, state in items:
            updated = datetime.now().strftime("%H:%M:%S")
            mission_status = "🚁 执行中" if state.get('flying') and state.get('mode') == 'AUTO' else "⏳ 等待"

            # 格式化电池信息
            battery_pct = state.get('battery', 0)
            voltage = state.get('battery_voltage', 0.0)
            current = state.get('battery_current', 0.0)
            battery_str = f"{battery_pct}% ({voltage:.1f}V/{current:.1f}A)"

            values = (
                drone_id,
                state.get('mode', 'UNKNOWN'),
                "是" if state.get('armed') else "否",
                "是" if state.get('flying') else "否",
                battery_str,
                f"{state.get('alt', 0):.1f}",
                f"{state.get('lat', 0):.6f}",
                f"{state.get('lng', 0):.6f}",
                f"{state.get('groundspeed', 0):.1f}/{state.get('verticalspeed', 0):.1f}",
                f"{state.get('satellites', 0)}/{state.get('hdop', 0):.1f}",
                mission_status,
                updated,
            )

            if drone_id in self.status_rows:
                self.status_tree.item(self.status_rows[drone_id], values=values)
            else:
                item_id = self.status_tree.insert("", "end", values=values)
                self.status_rows[drone_id] = item_id

    def update_ui(self):
        self.update_status_tree()
        self.update_drone_list()
        self.root.after(2000, self.update_ui)

    def connect(self):
        host = self.host_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            self.log("❌ 端口格式错误")
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((host, port))
            self.sock.settimeout(0.5)

            self.running = True
            self.recv_buffer = b""
            self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.recv_thread.start()

            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.status_label.config(text="● 已连接", foreground="green")
            self.bottom_label.config(text=f"已连接到 {host}:{port}")
            self.log(f"✅ 连接成功: {host}:{port}")

        except Exception as e:
            self.log(f"❌ 连接失败: {e}")
            messagebox.showerror("连接失败", str(e))

    def disconnect(self):
        self.running = False
        with self.sock_lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None

        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self.status_label.config(text="● 未连接", foreground="red")
        self.bottom_label.config(text="已断开")
        self.log("🔌 连接已断开")

    def receive_loop(self):
        while self.running:
            with self.sock_lock:
                sock = self.sock

            if sock is None:
                break

            try:
                data = sock.recv(4096)
                if not data:
                    self.root.after(0, lambda: self.log("⚠️ 服务器已断开连接"))
                    self.root.after(0, self.disconnect)
                    break

                self.recv_buffer += data

                while True:
                    packet, err, self.recv_buffer = self.parse_packet(self.recv_buffer)
                    if packet:
                        self.root.after(0, lambda p=packet: self.handle_packet(p))
                        continue
                    if err == "crc_mismatch":
                        self.root.after(0, lambda: self.log("⚠️ CRC校验失败"))
                        continue
                    break

            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                if self.running:
                    self.root.after(0, lambda msg=str(e): self.log(f"接收错误: {msg}"))
                break

    def parse_packet(self, buffer):
        if len(buffer) < 8:
            return None, "incomplete", buffer

        magic_index = buffer.find(TestProtocol.MAGIC)
        if magic_index < 0:
            return None, "magic", b""

        if magic_index > 0:
            buffer = buffer[magic_index:]

        if len(buffer) < 8:
            return None, "incomplete", buffer

        msg_type = buffer[2]
        drone_id = buffer[3]
        payload_len = struct.unpack('<H', buffer[4:6])[0]
        total_len = 8 + payload_len

        if len(buffer) < total_len:
            return None, "waiting", buffer

        packet = buffer[:total_len]
        recv_crc = struct.unpack('<H', packet[-2:])[0]
        calc_crc = TestProtocol.calculate_crc16(packet[2:-2])

        if recv_crc != calc_crc:
            return None, "crc_mismatch", buffer[2:]

        return {
            "msg_type": msg_type,
            "drone_id": drone_id,
            "payload": packet[6:-2],
        }, None, buffer[total_len:]

    def handle_packet(self, packet):
        msg_type = packet["msg_type"]
        drone_id = packet["drone_id"]
        payload = packet["payload"]

        if msg_type == 0x20:
            self.parse_status(drone_id, payload)

        elif msg_type == 0x17:
            self.log(f"🎯 [定点成功] 无人机 {drone_id} 已到达目标位置")

        elif msg_type == 0xF7:
            self.log(f"❌ [定点失败] 无人机 {drone_id} 定点飞行失败")

        elif msg_type == 0x18:
            self.log(f"✅ [航点任务] 无人机 {drone_id} 任务已上传并启动")
            if drone_id in self.mission_windows:
                self.mission_windows[drone_id].set_waypoints(self.current_waypoints)

        elif msg_type == 0x19:
            if len(payload) >= 5:
                current_wp, total_wp, event_type = struct.unpack("<HHB", payload[:5])

                if event_type == 0:
                    self.log(f"📍 [航点任务] 正在执行航点 {current_wp}/{total_wp}")
                elif event_type == 1:
                    self.log(f"🎯 [航点任务] 航点 {current_wp}/{total_wp} 已到达")
                elif event_type == 2:
                    self.log(f"✅ [航点任务] 全部 {total_wp} 个航点执行完成")

            if drone_id in self.mission_windows:
                win = self.mission_windows[drone_id]
                win.update_progress(current_wp, total_wp)

                if event_type == 0:
                    win.mark_current(current_wp)
                elif event_type in (1, 2):
                    win.mark_reached(current_wp)
        elif msg_type == 0xF8:
            self.log(f"❌ [航点任务] 无人机 {drone_id} 任务上传失败")

        elif 0x11 <= msg_type <= 0x16:
            ack_names = {
                0x11: "解锁",
                0x12: "上锁",
                0x13: "起飞",
                0x14: "模式切换",
                0x15: "降落",
                0x16: "返航",
            }
            self.log(f"✅ [ACK] 无人机 {drone_id} {ack_names.get(msg_type, '命令')}成功")

        elif 0xF1 <= msg_type <= 0xF8:
            fail_names = {
                0xF1: "解锁失败",
                0xF2: "上锁失败",
                0xF3: "起飞失败",
                0xF4: "模式切换失败",
                0xF5: "降落失败",
                0xF6: "返航失败",
            }

            reason = payload.decode(
                "utf-8",
                errors="ignore",
            ).strip() if payload else ""

            print(
                f"DEBUG NACK: "
                f"len={len(payload)}, "
                f"reason={reason!r}"
            )

            if reason:
                self.log(
                    f"❌ [NACK] 无人机 {drone_id} "
                    f"{fail_names.get(msg_type, '命令失败')}：{reason}"
                )   
                
        else:
            self.log(
                f"❌ [NACK] 无人机 {drone_id} "
                f"{fail_names.get(msg_type, '命令失败')}"
            )

    def parse_status(self, drone_id, payload):
        if len(payload) < 37:
            return

        with self.state_lock:
            state = self.drone_states.setdefault(drone_id, {})
            offset = 0

            flags = payload[offset]
            offset += 1
            state['armed'] = (flags & 0x01) != 0
            state['gps_fix'] = (flags & 0x02) != 0
            state['has_compass'] = (flags & 0x04) != 0
            state['flying'] = (flags & 0x08) != 0

            state['battery'] = payload[offset]
            offset += 1

            mode_len = min(payload[offset], 16)
            offset += 1
            mode_raw = payload[offset:offset + 16]
            offset += 16
            state['mode'] = mode_raw[:mode_len].decode('ascii', errors='ignore')

            state['lat'] = struct.unpack('<f', payload[offset:offset + 4])[0]
            offset += 4
            state['lng'] = struct.unpack('<f', payload[offset:offset + 4])[0]
            offset += 4

            alt_abs = struct.unpack('<f', payload[offset:offset + 4])[0]
            offset += 4

            if drone_id not in self.home_alt_abs or abs(alt_abs) < 0.01:
                self.home_alt_abs[drone_id] = alt_abs

            state['alt_abs'] = alt_abs
            state['alt'] = alt_abs - self.home_alt_abs.get(drone_id, alt_abs)

            groundspeed_cm = struct.unpack('<H', payload[offset:offset + 2])[0]
            offset += 2
            state['groundspeed'] = groundspeed_cm / 100.0

            verticalspeed_cm = struct.unpack('<h', payload[offset:offset + 2])[0]
            offset += 2
            state['verticalspeed'] = verticalspeed_cm / 100.0

            state['satellites'] = payload[offset]
            offset += 1
            state['hdop'] = payload[offset] / 10.0
            offset += 1

            # 解析扩展数据 (电压和电流)
            if len(payload) >= 41:
                state['battery_voltage'] = struct.unpack('<H', payload[offset:offset + 2])[0] / 1000.0
                state['battery_current'] = struct.unpack('<H', payload[offset + 2:offset + 4])[0] / 100.0
            else:
                state['battery_voltage'] = 0.0
                state['battery_current'] = 0.0

    def send_packet(self, cmd_type, drone_id, payload=b''):
        packet = TestProtocol.build_packet(cmd_type, drone_id, payload)

        with self.sock_lock:
            if self.sock is None:
                self.log("❌ 未连接服务器")
                return False

            try:
                self.sock.sendall(packet)
                return True
            except Exception as e:
                self.log(f"❌ 发送失败: {e}")
                return False

    def batch_arm(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return
        self.log(f"🚀 批量解锁: {selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.ARM, drone_id)
            time.sleep(0.1)

    def batch_disarm(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return
        self.log(f"🔒 批量上锁: {selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.DISARM, drone_id)
            time.sleep(0.1)

    def batch_takeoff(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return
        try:
            alt = float(self.takeoff_alt_var.get())
        except ValueError:
            alt = 10
        payload = struct.pack('<f', alt)
        self.log(f"🚁 批量起飞 - 高度:{alt}m, 目标:{selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.TAKEOFF, drone_id, payload)
            time.sleep(0.1)

    def batch_land(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return
        self.log(f"🛬 批量降落: {selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.LAND, drone_id)
            time.sleep(0.1)

    def batch_rth(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return
        self.log(f"🏠 批量返航: {selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.RTH, drone_id)
            time.sleep(0.1)

    def batch_set_mode(self):
        selected = self.get_selected_drones()
        if not selected:
            self.log("⚠️ 请至少选择一架无人机")
            return

        mode_id = self._get_selected_mode_id()
        if mode_id is None:
            return

        payload = bytes([mode_id])
        self.log(f"🔄 批量切换模式 - mode_id:{mode_id}, 目标:{selected}")
        for drone_id in selected:
            self.send_packet(TestProtocol.MsgType.SET_MODE, drone_id, payload)
            time.sleep(0.1)

    def _get_selected_mode_id(self):
        mode_text = self.mode_var.get()
        left = mode_text.rfind("(")
        right = mode_text.rfind(")")
        if left >= 0 and right > left:
            try:
                return int(mode_text[left + 1:right])
            except ValueError:
                pass
        self.log("❌ 模式格式错误")
        return None

    def send_set_mode(self):
        mode_id = self._get_selected_mode_id()
        if mode_id is None:
            return
        payload = bytes([mode_id])
        self.send_packet(TestProtocol.MsgType.SET_MODE, self.current_drone_id, payload)
        self.log(f"🔄 发送模式切换命令: ID={self.current_drone_id}, mode_id={mode_id}")

    def send_arm(self):
        self.send_packet(TestProtocol.MsgType.ARM, self.current_drone_id)
        self.log(f"🔓 发送解锁命令: ID={self.current_drone_id}")

    def send_disarm(self):
        self.send_packet(TestProtocol.MsgType.DISARM, self.current_drone_id)
        self.log(f"🔒 发送上锁命令: ID={self.current_drone_id}")

    def send_takeoff(self):
        try:
            alt = float(self.takeoff_alt_var.get())
        except ValueError:
            alt = 10
        payload = struct.pack('<f', alt)
        self.send_packet(TestProtocol.MsgType.TAKEOFF, self.current_drone_id, payload)
        self.log(f"🚁 发送起飞命令: ID={self.current_drone_id}, 高度={alt}m")

    def send_land(self):
        self.send_packet(TestProtocol.MsgType.LAND, self.current_drone_id)
        self.log(f"🛬 发送降落命令: ID={self.current_drone_id}")

    def send_rth(self):
        self.send_packet(TestProtocol.MsgType.RTH, self.current_drone_id)
        self.log(f"🏠 发送返航命令: ID={self.current_drone_id}")

    def send_goto(self):
        try:
            lat = float(self.goto_lat_var.get())
            lng = float(self.goto_lng_var.get())
            alt = float(self.goto_alt_var.get())
            speed = float(self.goto_speed_var.get())
            payload = TestProtocol.build_goto_payload(lat, lng, alt, speed)
            self.send_packet(TestProtocol.MsgType.GOTO, self.current_drone_id, payload)
            self.log(f"📍 发送绝对坐标飞行: ID={self.current_drone_id}, ({lat}, {lng}, {alt}m)")
        except ValueError as e:
            self.log(f"❌ 参数格式错误: {e}")

    def send_relative_goto(self):
        """发送相对坐标飞行指令"""
        try:
            angle = float(self.rel_angle_var.get())
            distance = float(self.rel_dist_var.get())
            alt = float(self.rel_alt_var.get())
            speed = float(self.goto_speed_var.get()) # 复用绝对GOTO的速度设置

            # 获取当前坐标
            with self.state_lock:
                state = self.drone_states.get(self.current_drone_id)
                if not state or state.get('lat') == 0:
                    self.log("❌ 无法执行相对飞行：未获取到无人机当前坐标")
                    return
                curr_lat = state['lat']
                curr_lng = state['lng']

            # 计算目标坐标
            target_lat, target_lng = self._calculate_target_coords(curr_lat, curr_lng, angle, distance)
            
            payload = TestProtocol.build_goto_payload(target_lat, target_lng, alt, speed)
            self.send_packet(TestProtocol.MsgType.GOTO, self.current_drone_id, payload)
            self.log(f"📍 发送相对位移飞行: ID={self.current_drone_id}, 方向={angle}°, 距离={distance}m (目标: {target_lat:.6f}, {target_lng:.6f})")
            
            # 同时更新绝对坐标输入框，方便观察
            self.goto_lat_var.set(f"{target_lat:.6f}")
            self.goto_lng_var.set(f"{target_lng:.6f}")
            self.goto_alt_var.set(str(alt))

        except ValueError as e:
            self.log(f"❌ 参数格式错误: {e}")

    def _calculate_target_coords(self, lat1, lng1, bearing_deg, distance_m):
        """根据起点坐标、方位角和距离计算终点坐标"""
        R = 6371000.0  # 地球平均半径(米)
        brng = math.radians(bearing_deg)
        phi1 = math.radians(lat1)
        lam1 = math.radians(lng1)

        phi2 = math.asin(math.sin(phi1) * math.cos(distance_m/R) +
                        math.cos(phi1) * math.sin(distance_m/R) * math.cos(brng))
        lam2 = lam1 + math.atan2(math.sin(brng) * math.sin(distance_m/R) * math.cos(phi1),
                                math.cos(distance_m/R) - math.sin(phi1) * math.sin(phi2))

        return math.degrees(phi2), math.degrees(lam2)

    def edit_mission(self):
        editor = WaypointEditor(self.root, self.current_waypoints)
        self.root.wait_window(editor.dialog)

        if editor.result:
            self.current_waypoints = copy.deepcopy(editor.result['waypoints'])
            global_speed = editor.result['global_speed']
            self.mission_speed_var.set(str(global_speed))
            self.update_mission_preview()
            self.log(f"📝 航点任务已编辑: {len(self.current_waypoints)} 个航点, 全局速度={global_speed}m/s")
            if messagebox.askyesno("上传任务", f"是否立即上传 {len(self.current_waypoints)} 个航点到无人机 {self.current_drone_id}？"):
                self.upload_mission_to_target()

    def upload_mission_to_target(self):
        if not self.current_waypoints:
            self.log("❌ 请先编辑航点任务")
            messagebox.showwarning("警告", "请先编辑航点任务")
            return

        try:
            global_speed = float(self.mission_speed_var.get())
        except ValueError:
            global_speed = 5.0

        try:
            payload = TestProtocol.build_mission_payload(self.current_waypoints, global_speed)
        except ValueError as e:
            self.log(f"❌ 航点任务参数错误: {e}")
            messagebox.showerror("错误", str(e))
            return

        if self.send_packet(TestProtocol.MsgType.MISSION_WP, self.current_drone_id, payload):
            self.log(f"📤 上传航点任务: ID={self.current_drone_id}, {len(self.current_waypoints)}个用户航点")
            self.show_mission_status()

    def create_square_mission(self):
        try:
            size = float(self.square_size_var.get())
            alt = float(self.square_alt_var.get())
        except ValueError:
            self.log("❌ 参数格式错误")
            return

        if size <= 0 or alt <= 0:
            self.log("❌ 边长和高度必须大于 0")
            return

        current_state = self.drone_states.get(self.current_drone_id, {})
        center_lat = current_state.get('lat', 0)
        center_lng = current_state.get('lng', 0)

        if center_lat == 0 or center_lng == 0:
            self.log("❌ 当前无人机位置无效，无法生成正方形航点")
            messagebox.showwarning("警告", "请先等待无人机状态上报，获取有效 GPS 坐标")
            return

        meter_to_deg_lat = 1.0 / 111320.0
        cos_lat = math.cos(math.radians(center_lat))
        if abs(cos_lat) < 1e-6:
            self.log("❌ 当前纬度过高，无法生成正方形航点")
            return

        meter_to_deg_lng = 1.0 / (111320.0 * cos_lat)

        half_side_deg_lat = (size / 2) * meter_to_deg_lat
        half_side_deg_lng = (size / 2) * meter_to_deg_lng

        self.current_waypoints = [
            {'lat': center_lat - half_side_deg_lat, 'lng': center_lng - half_side_deg_lng, 'alt': alt, 'hold_time': 1.0, 'accept_radius': 5.0},
            {'lat': center_lat - half_side_deg_lat, 'lng': center_lng + half_side_deg_lng, 'alt': alt, 'hold_time': 1.0, 'accept_radius': 5.0},
            {'lat': center_lat + half_side_deg_lat, 'lng': center_lng + half_side_deg_lng, 'alt': alt, 'hold_time': 1.0, 'accept_radius': 5.0},
            {'lat': center_lat + half_side_deg_lat, 'lng': center_lng - half_side_deg_lng, 'alt': alt, 'hold_time': 1.0, 'accept_radius': 5.0},
            {'lat': center_lat - half_side_deg_lat, 'lng': center_lng - half_side_deg_lng, 'alt': alt, 'hold_time': 1.0, 'accept_radius': 5.0},
        ]

        self.update_mission_preview()
        self.log(f"📐 已生成正方形航点任务: 边长={size}m, 高度={alt}m, 共5个用户航点")

    def update_mission_preview(self):
        for item in self.mission_preview.get_children():
            self.mission_preview.delete(item)

        for i, wp in enumerate(self.current_waypoints, 1):
            self.mission_preview.insert("", "end", values=(
                i,
                f"{wp['lat']:.7f}",
                f"{wp['lng']:.7f}",
                wp['alt'],
                wp.get('hold_time', 0),
                wp.get('accept_radius', 5),
            ))

    def show_mission_status(self):
        if self.current_drone_id not in self.mission_windows:
            self.mission_windows[self.current_drone_id] = MissionStatusWindow(self.root, self.current_drone_id)

        window = self.mission_windows[self.current_drone_id]
        if not window.window.winfo_exists():
            window = MissionStatusWindow(self.root, self.current_drone_id)
            self.mission_windows[self.current_drone_id] = window

        window.set_waypoints(self.current_waypoints)
        window.window.lift()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    app = TestClientGUI()
    app.root.mainloop()


if __name__ == '__main__':
    main()