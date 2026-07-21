"""
Small TCP JSON Lines client for Device2 -> Device1 decision-frame delivery.

    通过 TCP Socket 将组装好的决策帧发送给设备一，并实时将对方返回的反馈消息读取回来

station/exporter.py的PlannerExporter 类中会实例化 self.socket = PlannerSocketClient(...)
"""
import json
import select
import socket
import time
from typing import Any, Dict, List, Optional


class PlannerSocketClient:
    def __init__(
        self,
        host: str,
        port: int,
        reconnect_sec: float = 1.0,
        connect_timeout_sec: float = 0.2,
        recv_bytes: int = 65536,
    ):
        self.host = str(host)
        self.port = int(port)
        self.reconnect_sec = max(0.1, float(reconnect_sec))
        self.connect_timeout_sec = max(0.05, float(connect_timeout_sec))
        self.recv_bytes = max(1024, int(recv_bytes))
        self.sock: Optional[socket.socket] = None
        self.recv_buffer = b""
        self.next_reconnect_at = 0.0
        self.last_error = ""
        self.connected_once = False

    @property
    def connected(self) -> bool:
        """用于快速查询当前 Socket 是否处于活跃连接状态"""
        return self.sock is not None

    def close(self):
        """安全销毁 Socket 连接，避免句柄泄漏"""
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send_json(self, payload: Dict[str, Any]) -> bool:
        """(核心发送接口) 将 Python 字典序列化为 JSON 字符串，添加换行符，并发送给远程设备。包含自动重连机制"""
        # 1. 确保 Socket 已连通
        if not self._ensure_connected():
            return False
        # 2. 将数据转换为 UTF-8 编码的 JSON 字符串，结尾加 \n
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"
        try:
            # 全量发送
            self.sock.sendall(data)
            return True
        except OSError as exc:
            # 3. 如果发送过程中出错，记录错误，关闭 Socket，并设置下一次重连的时间点
            self.last_error = str(exc)
            self.close()
            self.next_reconnect_at = time.time() + self.reconnect_sec
            return False

    def poll_feedback(self) -> List[Dict[str, Any]]:
        """(核心接收接口) 非阻塞式轮询（使用 select 模型），从网络缓冲区中读取设备 3 返回的所有反馈消息（反馈也是 JSON Lines 格式）
            从 Socket 缓冲区读取并解析所有 JSON 行。
            使用 select 实现非阻塞轮询，保证仿真循环不会因为等待网络回应而挂起。
        """
        if not self.sock:
            return []
        messages: List[Dict[str, Any]] = []
        try:
            while True:
                # 检查 socket 是否有数据可读
                readable, _, _ = select.select([self.sock], [], [], 0.0)
                if not readable:
                    break
                # 读取数据块
                chunk = self.sock.recv(self.recv_bytes)
                if not chunk:
                    # 对方主动断开
                    self.last_error = "peer closed"
                    self.close()
                    break
                self.recv_buffer += chunk
                # 处理粘包：如果缓冲区里有多个 \n，循环拆包处理
                while b"\n" in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        parsed = json.loads(line.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if isinstance(parsed, dict):
                        messages.append(parsed)
        except OSError as exc:
            # 发生网络异常则触发重连逻辑
            self.last_error = str(exc)
            self.close()
            self.next_reconnect_at = time.time() + self.reconnect_sec
        return messages

    def _ensure_connected(self) -> bool:
        """(底层自动重连逻辑) 保证连接的一致性，如果连接断开了，它会自动尝试建立新连接，并设置非阻塞标志以保证仿真循环不被卡死"""
        if self.sock:
            return True
        now = time.time()
        if now < self.next_reconnect_at:
            return False
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_sec)
            sock.setblocking(False)
            self.sock = sock
            self.recv_buffer = b""
            self.last_error = ""
            self.connected_once = True
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self.next_reconnect_at = now + self.reconnect_sec
            self.close()
            return False
