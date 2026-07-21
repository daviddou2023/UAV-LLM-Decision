"""
Optional debug mirror for the latest Device2 -> Device1 decision payload.

The matched display Redis interface remains in integrations/middle_layer.py / integrations/redis_export.py.
This mirror is only for integration troubleshooting when enabled.

    当设备二向设备一发送决策帧时，如果设备一没有做出预期反应，调试人员往往很难直接观测到发送的具体内容。该文件通过将最新的实际发送载荷镜像写入 Redis，
    让你可以随时在 Redis 数据库中通过 GUI 工具查看当前系统到底发出了什么指令，极大地降低了联调成本

station/exporter.py 内部有一个 debug_redis 属性。
调用时机：当 PlannerExporter 在 maybe_publish 中构建好 JSON 报文后，如果配置开启了调试功能（config.debug_redis_enable 为 True），
它会立即调用本模块的 publish_text 方法，把 JSON 存入 Redis，确保发送给真实设备的规划与 Redis 里的镜像完全一致
"""
import shutil
import subprocess
from typing import Optional


class PlanDebugRedisMirror:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 6379,
        db: int = 0,
        key: str = "d2:d3:plan_latest",
        password: Optional[str] = None,
        timeout_sec: float = 0.3,
    ):
        """
        初始化 Redis 调试镜像的连接配置，包括 Redis 的地址、端口、数据库索引及关键的存储键名（Key）。它同时通过 shutil.which 检查系统中是否存在 redis-cli 工具，这是它运行的前提
        :param host:
        :param port:
        :param db:
        :param key:
        :param password:
        :param timeout_sec:
        """
        self.host = str(host)
        self.port = int(port)
        self.db = int(db)
        self.key = str(key)
        self.password = password or None
        self.timeout_sec = max(0.05, float(timeout_sec))
        self.redis_cli = shutil.which("redis-cli")
        self.last_error = "" if self.redis_cli else "redis-cli not found"

    def publish_text(self, text: str) -> bool:
        """将序列化好的 JSON 字符串文本通过系统命令 redis-cli 执行 SET 操作，写入指定的 Redis Key 中
        该函数采用异步调用的思想（通过 subprocess），不会阻塞仿真主循环的运行。
        """
        if not self.redis_cli:
            return False
        # 组装 redis-cli 命令参数
        cmd = [self.redis_cli, "-h", self.host, "-p", str(self.port), "-n", str(self.db)]
        if self.password:
            cmd.extend(["-a", self.password])
        # 将数据 SET 到指定的 Key 中
        cmd.extend(["SET", self.key, text])
        try:
            # 在独立进程中执行命令，设置了严格的超时时间，防止 Redis 阻塞仿真
            proc = subprocess.run(
                cmd,
                # 丢弃输出，减少干扰
                stdout=subprocess.DEVNULL,
                # 捕获错误信息
                stderr=subprocess.PIPE,
                text=True,
                timeout=self.timeout_sec,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.last_error = str(exc)
            return False
        # 检查是否写入成功，若失败则更新错误信息
        if proc.returncode != 0:
            self.last_error = (proc.stderr or "").strip()
            return False
        self.last_error = ""
        return True
