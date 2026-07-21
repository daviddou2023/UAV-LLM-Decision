import json
import socket

def send_udp_json(data, host='127.0.0.1', port=12345):
    """
    :param data: 字典类型的数据，将被转换为JSON字符串
    :param host: 目标主机地址（默认本地回环地址）
    :param port: 目标端口（默认12345）
    :return:
    """

    json_str = json.dumps(data, separators=(',', ':'))
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sent = s.sendto(json_str.encode('utf-8'), (host, port))
        return sent
    finally:
        s.close()


if __name__ == '__main__':
    data = {
        "device_id": "RADAR_001",
        "timestamp": 1740523800000,
        "seq": 123,
        "drone_info": [
            {
                "drone_id": 1,
                "lon": 116.403874,
                "lat": 39.914885,
                "altitude": 503.2,
                "pitch": 15.6,
                "azimuth": 88.5,
                "speed": 5.0,
                "speedx": 3.0,
                "speedy": 4.0,
                "speedz": 0.0,
                "distance": 200.3,
                "status": 1
            },
            {
                "drone_id": 2,
                "lon": 117.403874,
                "lat": 34.914885,
                "altitude": 400.2,
                "pitch": 12.3,
                "azimuth": 45.5,
                "speed": 12.0,
                "speedx": 4.0,
                "speedy": 3.0,
                "speedz": 0.0,
                "distance": 340.2,
                "status": 1
            }
        ]
    }

    send_udp_json(data, host='192.168.1.100', port=8888)


# self.redis_client.set(f'{node_num}_x', node_x)
# self.redis_client.set(f'{node_num}_y', node_y)
# self.redis_client.set(f'{node_num}_z', node_z)
# self.redis_client.set(f'{node_num}_status', node_status)（normal/unnormal）
# self.redis_client.set(f'{node_num}_type', node_type)(ally/enemy)
# self.redis_client.set(f'{node_num}_frame', frame_num)
# self.redis_client.set(f'{node_num}_timestamp', time.time())
# redis ip 192.166.51.23  密码 uav123
