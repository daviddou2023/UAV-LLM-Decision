"""
Runtime publisher for Device2 decision frames returned to Device1.

    将 station/contracts.py 生成的完整 JSON 字典，按照设备一输入帧触发或预定频率，
    通过指定网络协议投递给外部的设备一。

simulation/main.py 会调用 PlanExportConfig.from_config() 加载 core/common.py 中的 PLAN_EXPORT 配置，并实例化本模块的 PlannerExporter 类
simulation/main.py 里的每帧循环中，通过 plan_exporter.maybe_publish(env) 不断驱动这套协议引擎
本文件并不直接操作 TCP 底层。它将网络连接（如重连、缓冲区粘包拆包）外包给了 PlannerSocketClient，并调用它的 send_json() 发送数据，调用 poll_feedback() 拉取对方的回应。
拉取到的反馈信息会送给 Device3FeedbackHandler.process()，让这个处理器去判定：如果飞控执行侧报错了，是不是该在系统里把对应的无人机标为“暂时不可用”


"""
import json
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from station.feedback import Device3FeedbackHandler
from station.contracts import PlanFrameBuilder, PlannerContractOptions
from station.debug_redis import PlanDebugRedisMirror
from station.interfaces import (
    PlanDebugMirrorPort,
    PlanFeedbackHandlerPort,
    PlanFrameBuilderPort,
    PlanFrameTransportPort,
    PlanSideBridgePort,
)
from station.query import decision_output_frame_from_plan_frame
from station.socket_client import PlannerSocketClient

try:
    from station.plan_bridge import StationPlanBridge
except Exception:
    StationPlanBridge = None


@dataclass
class PlanExportConfig(PlannerContractOptions):
    enabled: bool = False
    transport: str = "tcp_json_lines"
    publish_policy: str = "input_frame"
    output_frame_type: str = "decision_output_frame"
    publish_interval: float = 1.0
    socket_host: str = "127.0.0.1"
    socket_port: int = 7001
    socket_reconnect_sec: float = 1.0
    socket_connect_timeout_sec: float = 0.2
    debug_redis_enable: bool = False
    debug_redis_host: str = "127.0.0.1"
    debug_redis_port: int = 6379
    debug_redis_db: int = 0
    debug_redis_password: str = ""
    debug_redis_key: str = "d2:d3:plan_latest"
    failed_uav_cooldown_sec: float = 5.0
    kafka_enable: bool = False
    kafka_topic: str = "d2.d3.plan_frame"

    @classmethod
    def from_config(cls, settings=None):
        """从 core.common 读取规划发布配置。"""
        if settings is None:
            from core.common import PLAN_EXPORT as settings
        supported_modes = _cfg_list(settings, "supported_control_modes", ("mission_waypoints", "goto"))
        return cls(
            enabled=_cfg_bool(settings, "enabled", False),
            transport=_cfg_str(settings, "transport", "tcp_json_lines"),
            publish_policy=_cfg_str(settings, "publish_policy", "input_frame"),
            output_frame_type=_cfg_str(settings, "output_frame_type", "decision_output_frame"),
            # 最小发送周期限制为 0.03秒，防止网络被打挂。
            publish_interval=max(0.03, _cfg_float(settings, "publish_interval", 1.0)),
            valid_duration_ms=max(100, _cfg_int(settings, "valid_duration_ms", 1500)),
            waypoint_count=max(1, _cfg_int(settings, "waypoint_count", 3)),
            waypoint_spacing_m=max(1.0, _cfg_float(settings, "waypoint_spacing_m", 500.0)),
            primary_control_mode=_cfg_str(settings, "primary_control_mode", "mission_waypoints"),
            supported_control_modes=tuple(supported_modes),
            replace_policy=_cfg_str(settings, "replace_policy", "task_change_immediate_same_task_smooth"),
            smooth_replace_threshold_m=max(0.0, _cfg_float(settings, "smooth_replace_threshold_m", 30.0)),
            min_reupload_interval_sec=max(0.0, _cfg_float(settings, "min_reupload_interval_sec", 2.0)),
            uav_id_map=_cfg_str(settings, "uav_id_map", ""),
            preflight_takeoff_alt_m=max(0.0, _cfg_float(settings, "preflight_takeoff_alt_m", 15.0)),
            socket_host=_cfg_str(settings, "socket_host", "127.0.0.1"),
            socket_port=_cfg_int(settings, "socket_port", 7001),
            socket_reconnect_sec=max(0.1, _cfg_float(settings, "socket_reconnect_sec", 1.0)),
            socket_connect_timeout_sec=max(0.05, _cfg_float(settings, "socket_connect_timeout_sec", 0.2)),
            debug_redis_enable=_cfg_bool(settings, "debug_redis_enable", False),
            debug_redis_host=_cfg_str(settings, "debug_redis_host", "127.0.0.1"),
            debug_redis_port=_cfg_int(settings, "debug_redis_port", 6379),
            debug_redis_db=_cfg_int(settings, "debug_redis_db", 0),
            debug_redis_password=_cfg_str(settings, "debug_redis_password", ""),
            debug_redis_key=_cfg_str(settings, "debug_redis_key", "d2:d3:plan_latest"),
            failed_uav_cooldown_sec=max(0.0, _cfg_float(settings, "failed_uav_cooldown_sec", 5.0)),
            kafka_enable=_cfg_bool(settings, "kafka_enable", False),
            kafka_topic=_cfg_str(settings, "kafka_topic", "d2.d3.plan_frame"),
        )

    def summary(self) -> str:
        target = f"{self.socket_host}:{self.socket_port}" if self.transport in ("tcp", "tcp_jsonl", "tcp_json_lines") else self.transport
        return (
            f"设备2->设备一决策发布开启 | transport={self.transport} -> {target} "
            f"| policy={self.publish_policy} | frame={self.output_frame_type} "
            f"| interval={self.publish_interval:.2f}s | wp={self.waypoint_count}x{self.waypoint_spacing_m:.0f}m"
        )


class PlannerExporter:
    def __init__(
        self,
        config: PlanExportConfig,
        builder: Optional[PlanFrameBuilderPort] = None,
        transport: Optional[PlanFrameTransportPort] = None,
        feedback_handler: Optional[PlanFeedbackHandlerPort] = None,
        debug_mirror: Optional[PlanDebugMirrorPort] = None,
        side_bridge: Optional[PlanSideBridgePort] = None,
    ):
        """组装规划发布链路。

        默认仍使用现有实现；参数只用于测试或联调时替换某个端口。
        """
        self.config = config
        self.builder = builder or PlanFrameBuilder()
        self.feedback = feedback_handler or Device3FeedbackHandler(config.uav_id_map, config.failed_uav_cooldown_sec)
        self.plan_seq = 0
        self.last_publish_at = 0.0
        self.last_send_ok = False
        self.last_error_log_at = 0.0
        self.kafka_notice_sent = False
        self.last_published_input_key = None

        self.socket = transport
        if self.socket is None and config.transport in ("tcp", "tcp_jsonl", "tcp_json_lines"):
            self.socket = PlannerSocketClient(
                host=config.socket_host,
                port=config.socket_port,
                reconnect_sec=config.socket_reconnect_sec,
                connect_timeout_sec=config.socket_connect_timeout_sec,
            )

        self.station_bridge = side_bridge
        if self.station_bridge is None and StationPlanBridge:
            self.station_bridge = StationPlanBridge.from_config()

        self.debug_redis = debug_mirror
        if self.debug_redis is None and config.debug_redis_enable:
            self.debug_redis = PlanDebugRedisMirror(
                host=config.debug_redis_host,
                port=config.debug_redis_port,
                db=config.debug_redis_db,
                key=config.debug_redis_key,
                password=config.debug_redis_password,
            )

    def reset_assignments(self):
        """委托内部的构建器清空当前的任务绑定历史记录"""
        self.builder.reset_assignments()

    def close(self):
        """安全关闭底层的 TCP Socket 连接"""
        if self.socket:
            self.socket.close()
        if self.station_bridge:
            self.station_bridge.close()

    def maybe_publish(self, env, force: bool = False) -> List[Tuple[str, str, str]]:
        """设备2到设备一的发布入口。

        执行顺序固定为：先拉回执 -> 按输入帧或 publish_interval 判定是否发送
        -> 构建 PlanFrame -> 裁剪主链路载荷 -> 发送 -> 可选 station 转接/Redis 镜像
        -> 再拉一次回执。
        """
        if not self.config.enabled:
            return []

        logs: List[Tuple[str, str, str]] = []
        # 尝试接收对方发来的回执消息
        logs.extend(self._poll_feedback(env))

        now = time.time()
        input_key = self._input_frame_key(env)
        if not force and not self._should_publish(now, input_key):
            return logs

        # 增加包序列号，获取毫秒时间戳
        self.plan_seq += 1
        timestamp_ms = int(now * 1000)
        # 调用 station/contracts.py 里的构建器，生成庞大的字典
        frame = self.builder.build(env, self.config, self.plan_seq, timestamp_ms)
        payload = self._payload_from_frame(frame)
        # 将实际发送载荷序列化为紧凑的单行 JSON 字符串 (没有空格和换行)
        payload_text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

        # 调用底层的 Socket 发送出去
        sent = self._send_frame(payload, now, logs)
        # 可选旁路转接给地面站侧，复用同一个 PlanFrame，避免重新分配或重新规划。
        if self.station_bridge:
            logs.extend(self.station_bridge.forward(frame, now))
        # 如果开启了 Redis 镜像功能，同时把这段文本写入 Redis 供调试
        if self.debug_redis:
            self.debug_redis.publish_text(payload_text)
        if self.config.kafka_enable and not self.kafka_notice_sent:
            logs.append(("[PLAN]", f"Kafka日志方案仅预留，当前未实际发布 topic={self.config.kafka_topic}", "amber"))
            self.kafka_notice_sent = True

        # 记录本次发送是否成功
        self.last_send_ok = sent
        self.last_publish_at = now
        if sent and input_key is not None:
            self.last_published_input_key = input_key
        # 发送完毕后，再次尝试拉取一下有没有极速反馈的消息
        logs.extend(self._poll_feedback(env))
        return logs

    def _should_publish(self, now: float, input_key) -> bool:
        policy = str(self.config.publish_policy or "input_frame").strip().lower()
        if policy in ("input_frame", "radar_frame", "on_input"):
            return input_key is not None and input_key != self.last_published_input_key
        if policy in ("input_frame_or_interval", "radar_frame_or_interval"):
            if input_key is not None and input_key != self.last_published_input_key:
                return True
            return (now - self.last_publish_at) >= self.config.publish_interval
        return (now - self.last_publish_at) >= self.config.publish_interval

    def _input_frame_key(self, env):
        meta = getattr(env, "last_live_packet_meta", {}) or {}
        device_id = meta.get("device_id")
        for key in ("frame", "seq", "timestamp"):
            value = meta.get(key)
            if value not in (None, ""):
                return (device_id, key, value)
        return None

    def _payload_from_frame(self, frame):
        output_type = str(self.config.output_frame_type or "decision_output_frame").strip().lower()
        if output_type in ("decision_output_frame", "decision_frame", "compact"):
            return decision_output_frame_from_plan_frame(frame)
        return frame

    def _send_frame(self, frame, now: float, logs: List[Tuple[str, str, str]]) -> bool:
        """根据不同的传输协议（tcp_json_lines 或 redis），选择正确的底层发送器进行发包"""
        # 第一阶段主协议：使用 TCP 发送单行 JSON
        if self.config.transport in ("tcp", "tcp_jsonl", "tcp_json_lines"):
            if not self.socket:
                return False
            # 调用 station/socket_client.py 里的 send_json 发送
            sent = self.socket.send_json(frame)
            if sent:
                # 只有从“失败”变为“成功”的那一刻，打印一条绿色的成功日志
                if not self.last_send_ok:
                    logs.append(("[PLAN]", "设备一决策Socket已连接，决策帧开始发送", "green"))
                return True
            # 发送失败时的限流打印机制：每5秒才打印一次，防止终端疯狂刷屏崩溃
            if now - self.last_error_log_at >= 5.0:
                reason = self.socket.last_error or "waiting_for_device1"
                logs.append(("[PLAN]", f"设备一决策Socket暂未发送成功: {reason}", "amber"))
                self.last_error_log_at = now
            return False

        if self.config.transport == "redis":
            if self.debug_redis:
                return True
            if now - self.last_error_log_at >= 5.0:
                logs.append(("[PLAN]", "PLAN_EXPORT['transport']='redis' 仅作为调试镜像保留，请开启 PLAN_EXPORT['debug_redis_enable']", "amber"))
                self.last_error_log_at = now
            return False

        if now - self.last_error_log_at >= 5.0:
            logs.append(("[PLAN]", f"未知规划发布方式: {self.config.transport}", "amber"))
            self.last_error_log_at = now
        return False

    def _poll_feedback(self, env) -> List[Tuple[str, str, str]]:
        """调用底层的 TCP 接收器，拉取设备 3 传回的执行确认和状态回执"""
        if not self.socket:
            return []
        events = self.socket.poll_feedback()
        if not events:
            return []
        return self.feedback.process(env, events)


def _cfg_value(settings, key: str, default):
    if not isinstance(settings, dict):
        return default
    return settings.get(key, default)


def _cfg_str(settings, key: str, default: str) -> str:
    return str(_cfg_value(settings, key, default))


def _cfg_int(settings, key: str, default: int) -> int:
    try:
        return int(float(_cfg_value(settings, key, default)))
    except (TypeError, ValueError):
        return int(default)


def _cfg_float(settings, key: str, default: float) -> float:
    try:
        return float(_cfg_value(settings, key, default))
    except (TypeError, ValueError):
        return float(default)


def _cfg_bool(settings, key: str, default: bool) -> bool:
    value = _cfg_value(settings, key, default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "on", "true", "yes", "y", "enable", "enabled"):
        return True
    if text in ("0", "off", "false", "no", "n", "disable", "disabled"):
        return False
    return bool(default)


def _cfg_list(settings, key: str, default) -> List[str]:
    value = _cfg_value(settings, key, default)
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]
