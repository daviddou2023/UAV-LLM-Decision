# mavlink_handler.py
import threading
import time

from mavlink_handler_base import FirmwareType, MavlinkHandlerBaseMixin
from mavlink_handler_commands import MavlinkCommandMixin
from mavlink_handler_connection import MavlinkConnectionMixin
from mavlink_handler_firmware import MavlinkFirmwareMixin
from mavlink_handler_mission import MavlinkMissionMixin


class MavlinkHandler(
    MavlinkMissionMixin,
    MavlinkCommandMixin,
    MavlinkFirmwareMixin,
    MavlinkConnectionMixin,
    MavlinkHandlerBaseMixin,
):
    def __init__(self, port, baud, sys_id=1, comp_id=1):
        self.port = port
        self.baud = baud
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.master = None
        self.connected = False

        self._mode_mapping = None
        self.firmware_type = FirmwareType.UNKNOWN
        self.firmware_version = ""
        self.firmware_vendor = ""

        self.goto_command = None
        self.goto_supports_speed = False
        self.default_mode = "GUIDED"
        self.arm_mode = None
        self.takeoff_mode = "GUIDED"

        self.current_state = {
            "armed": False,
            "mode": "UNKNOWN",
            "lat": 0.0,
            "lng": 0.0,
            "alt_abs": 0.0,
            "alt_rel": 0.0,
            "groundspeed": 0.0,
            "verticalspeed": 0.0,
            "battery": 100,
            "satellites": 0,
            "hdop": 0.0,
            "gps_fix_type": 0,
            "flying": False,
            "home_alt_abs": None,
            "home_lat": 0.0,
            "home_lng": 0.0,
            "home_alt": 0.0,
            "home_set": False,
        }
        self.last_update_time = time.time()
        self.update_interval = 0.1
        self.last_heartbeat = time.time()
        self.last_heartbeat_debug_time = 0.0
        self.last_statustext = None
        self.last_statustext_time = 0.0
        self.last_arm_status_text = None
        self.last_arm_status_time = 0.0
        self.last_armed_evidence_time = 0.0
        self.last_armed_evidence_source = None
        self.io_lock = threading.Lock()
        self.ack_lock = threading.Lock()
        self.command_ack_cache = {}
        self.command_ack_events = {}
        self.param_value_cache = {}
        
        self.mission_protocol_lock = threading.Lock()
        self.mission_request_cache = {}
        self.mission_request_events = {}
        self.mission_ack_cache = None
        self.mission_ack_event = None
        self.mission_count_cache = None
        self.mission_count_event = None
        self.autopilot_version_msg = None
        self.autopilot_version_time = 0.0
        self.last_fc_heartbeat_msg = None
        self.last_fc_heartbeat_time = 0.0
        self.flight_controller_ids = None

        self.offboard_setpoint_rate_hz = 10.0
        self.offboard_setpoint_period = 1.0 / self.offboard_setpoint_rate_hz
        self.offboard_preheat_seconds = 2.5
        self.offboard_min_preheat_count = int(
            self.offboard_preheat_seconds * self.offboard_setpoint_rate_hz
        )
        self.offboard_last_send_time = 0.0
        self.heartbeat_thread = None
        self.heartbeat_running = False
        self.rx_thread = None
        self.rx_running = False
        self.state_events = {
            "armed": threading.Event(),
            "mode": threading.Event(),
        }
    
        # --- 指令中断机制相关变量 ---
        self.current_cmd_id = 0          # 当前正在运行的指令版本 ID
        self.cmd_lock = threading.Lock() # 保护指令版本的锁
        
        self.mission_complete = False
        self._current_mission_count = None

        self._last_mission_current_log_time = 0.0
        self._last_mission_current_seq = -1
        self._mission_stuck_count = 0
        self._last_position_log_time = 0.0
        self._last_heartbeat_log_time = 0.0

        self.last_statustext = None
        self.last_statustext_time = 0.0