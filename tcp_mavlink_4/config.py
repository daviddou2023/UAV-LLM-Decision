# config.py
import os

# ===== 飞控连接配置 =====
# SITL仿真配置
#SERIAL_PORT = 'udp:172.19.32.1:14550'
DRONE_LINKS = {
    #11: {
       # "port": "udp:172.19.32.1:14550",
        #"baud": 57600,
        #"sys_id": 1,
        #"comp_id": 1,
        #"label": "uav-1",
    #},
    4: {
        "port": "udpin:0.0.0.0:14554",
        "baud": 57600,
        "sys_id": 54,
        "comp_id": 1,
        "label": "uav-4",
    },
    #4: {
        #"port": "udpin:0.0.0.0:14553",
        #"baud": 57600,
        #"sys_id": 53,
        #"comp_id": 1,
        #"label": "uav-4",
   #},
}
#SERIAL_PORT = 'udpin:0.0.0.0:14554'  # 本地UDP端口
# SERIAL_PORT = 'COM3'  # 真实无人机（Windows）
# SERIAL_PORT = '/dev/ttyUSB0'  # 真实无人机（Linux）
#BAUD_RATE = 57600

# ===== TCP服务器配置 =====
TCP_SERVER_IP = '0.0.0.0'
TCP_SERVER_PORT = 6001

# ===== 无人机配置 =====
#DRONE_ID = 1
#SYS_ID = 53
#COMP_ID = 1

# ===== 状态上报配置 =====
STATUS_BROADCAST_INTERVAL = 0.1  # 秒，状态广播间隔（100ms）

# ===== 飞行参数 =====
GOTO_REACHED_DISTANCE = 5.0      # 米，判定到达目标的距离阈值
GOTO_VERTICAL_THRESHOLD = 2.0    # 米，垂直方向到达阈值
TAKEOFF_ALT_THRESHOLD = 0.8      # 起飞成功阈值（目标高度的百分比）
LAND_ALT_THRESHOLD = 0.5         # 米，降落成功高度阈值
MODE_SWITCH_TIMEOUT = 3.0        # 秒，模式切换等待时间

# ===== 超时配置 =====
CMD_ACK_TIMEOUT = 5.0            # 秒，命令ACK等待超时
GOTO_MIN_TIMEOUT = 10.0          # 秒，GOTO最小超时时间
GOTO_MAX_TIMEOUT = 300.0         # 秒，GOTO最大超时时间（5分钟）
TAKEOFF_TIMEOUT = 30.0           # 秒，起飞超时时间
LAND_TIMEOUT = 60.0              # 秒，降落超时时间