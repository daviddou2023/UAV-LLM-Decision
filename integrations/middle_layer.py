"""
无人集群反无项目
中间层
地面站-unity数据转换
1.经纬度转换为笛卡尔坐标系
2.地面站redis-hash数据转换为unity-key数据
3.做数据插值

"""

import math
import redis
import time
import threading
from typing import Dict, List, Tuple, Optional
import json
from enum import Enum, unique


@unique
class NodeType(Enum):
    Enemy = 0  # 敌方无人机
    AllyNet = 1  # 己方拦网无人机
    AllyAttack = 2  # 己方打击无人机

class Node:
    def __init__(self, node_id: int, node_type: NodeType) -> None:
        self.x = 0
        self.y = 0
        self.z = 0
        self.id = node_id  # 节点编号，己方与敌方合在一起统一编号
        self.type = node_type  # 节点类型
        self.battery = 100  # 电量，0-100


class MiddleLayer:
    def __init__(self, host='localhost', port=6379, password='', db=0, decode_responses=True,
                 socket_connect_timeout=3, socket_timeout=3, read_interval=2, framerate=20):
        self.host = host
        self.port = port
        self.password = password
        self.db = db
        self.decode_responses = decode_responses
        self.socket_connect_timeout = socket_connect_timeout
        self.socket_timeout = socket_timeout
        self.read_interval = read_interval  # 读取地面站数据间隔，单位秒
        self.framerate = framerate  # 插值帧率
        self.redis_client = None
        self.running = False

        # 插值相关
        self.total_frames_between_reads = read_interval * framerate  # 每次读取间隔内的总帧数
        self.current_frame_in_interval = 0  # 当前读取间隔内的帧数
        self.last_read_time = 0  # 上次读取数据的时间

        # 数据存储
        self.previous_data = None  # 上一次读取到的所有节点位置数据
        self.current_data = None   # 当前读取到的所有节点位置数据
        self.interpolation_buffer = []  # 插值缓冲，存储当前间隔内的所有插值点

        # 默认中心点坐标（西安市中心）
        self.center_lat = 34.2663
        self.center_lon = 108.9549

        # 性能统计
        self.stats = {
            'total_frames_processed': 0,
            'total_reads': 0,
            'last_read_size': 0
        }


    @staticmethod
    def lonlat2cartesian(lat: float, lon: float, center_lat: float = 34.2663, center_lon: float = 108.9549) -> list:
        """
        经纬度转换为笛卡尔坐标系
        基于WGS84椭球模型计算

        笛卡尔坐标系原点的经纬度为：
            1.西安市中心 lat:34.2663,lon:108.9549
        x轴正向为东，z轴正向为北
        :param lat: 需要转换的点纬度值
        :param lon: 需要转换的点经度值
        :param center_lat: 坐标系原点纬度值
        :param center_lon: 坐标系原点经度值
        :return: [x, z] unity坐标系中x轴和z轴坐标
        """
        # WGS84椭球参数
        a = 6378137.0  # 长半轴，单位：米
        f = 1 / 298.257223563  # 扁率

        # 将角度转换为弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        center_lat_rad = math.radians(center_lat)
        center_lon_rad = math.radians(center_lon)

        # 计算卯酉圈曲率半径N
        e2 = 2 * f - f * f  # 第一偏心率平方
        sin_lat = math.sin(lat_rad)
        N = a / math.sqrt(1 - e2 * sin_lat ** 2)

        # 计算中心点的N
        sin_center_lat = math.sin(center_lat_rad)
        N_center = a / math.sqrt(1 - e2 * sin_center_lat ** 2)

        # 计算东西方向距离（东为正）
        dx = (lon_rad - center_lon_rad) * (N_center * math.cos(center_lat_rad))

        # 计算南北方向距离（北为正）
        # 使用子午线曲率半径M
        M_center = a * (1 - e2) / (1 - e2 * sin_center_lat ** 2) ** (3 / 2)
        dz = (lat_rad - center_lat_rad) * M_center

        return [dx, dz]


    def create_redis(self) -> None:
        """
        创建redis对象
        """
        try:
            self.redis_client = redis.Redis(
                host=self.host,
                port=self.port,
                db=self.db,
                password=self.password,
                decode_responses=self.decode_responses,
                socket_connect_timeout=self.socket_connect_timeout,
                socket_timeout=self.socket_timeout
            )
            # 测试连接
            self.redis_client.ping()
            print(f"Redis连接成功: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Redis连接失败: {e}")
            self.redis_client = None
            return False


    def clear_redis_data(self):
        """
        清空redis数据
        """
        if self.redis_client:
            self.redis_client.flushall()
            print("Redis数据已清空")


    def get_redis_data(self) -> List[List]:
        """
        查询当前所有节点数据
        地面站使用的redis数据键名：'uav:status:uav_003'
        查询结果为字典：{
            'drone_id': 'uav_003',
            'timestamp': '2026-01-27 11:35:35',
            'longitude': '116.393956',
            'latitude': '39.908824',
            'altitude': '130.57',
            'roll': '-4.0',
            'pitch': '0.77',
            'yaw': '165.53',
            'battery': '66.77',
            'task_status': 'EXECUTING',
            'flight_path': 'RECTANGLE',
            'message_type': 'STATUS_REPORT'
            }
        """

        ally_pattern_str = 'uav:status:uav'
        enemy_pattern_str = 'enemy:status:enemy'

        if not self.redis_client:
            print("Redis客户端未初始化")
            return []

        try:
            keys = self.redis_client.keys()
            ally_keys = []  # 遍历检索符合要求的所有己方节点键名
            enemy_keys = []  # 遍历检索符合要求的所有敌方节点键名
            for key in keys:
                if key.startswith(ally_pattern_str):
                    ally_keys.append(key)
                if key.startswith(enemy_pattern_str):
                    enemy_keys.append(key)

            # 如果没有找到节点数据，返回空列表
            if not ally_keys + enemy_keys:
                print("未找到节点数据")
                return []

            result = []

            for key in ally_keys:
                try:
                    node_data = self.redis_client.hgetall(key)
                    if node_data:
                        # 提取节点编号
                        node_num = int(key[len(ally_pattern_str):])

                        # 解析坐标数据
                        lat = float(node_data['lat'])
                        lon = float(node_data['lon'])
                        alt = float(node_data['alt'])
                        status = 0 if node_data['status'] == 'destroyed' else 1

                        # 坐标转换
                        x, z = self.lonlat2cartesian(lat, lon, self.center_lat, self.center_lon)
                        y = alt

                        result.append([node_num, x, y, z, status, 'ally'])
                except Exception as e:
                    print(f"解析己方节点数据失败 {key}: {e}")
                    continue

            for key in enemy_keys:
                try:
                    node_data = self.redis_client.hgetall(key)
                    if node_data:
                        # 提取节点编号
                        node_num = int(key[len(enemy_pattern_str):]) + len(ally_keys)  # 敌方节点序号排在己方节点之后

                        # 解析坐标数据
                        lat = float(node_data['lat'])
                        lon = float(node_data['lon'])
                        alt = float(node_data['alt'])
                        # status = 0 if node_data['status'] == 'destroyed' else 1
                        status = node_data['status']

                        # 坐标转换
                        x, z = self.lonlat2cartesian(lat, lon, self.center_lat, self.center_lon)
                        y = alt

                        result.append([node_num, x, y, z, status, 'enemy'])
                except Exception as e:
                    print(f"解析敌方节点数据失败 {key}: {e}")
                    continue

            # 按节点编号排序
            result.sort(key=lambda x: x[0])

            self.stats['last_read_size'] = len(result)
            self.stats['total_reads'] += 1

            if result:
                print(f"读取到 {len(result)} 个节点数据")
            else:
                print("读取数据为空")

            return result
        except Exception as e:
            print(f"获取Redis数据失败: {e}")
            return []


    def write_redis(self, node_data_list: List[List], frame_num: int):
        """
        把节点数据按照unity读取的格式写入redis
        键名：'3_x', '3_y', '3_z'
        同时写入帧编号和时间戳

        参数:
        node_data_list: 节点数据列表 [[node_num, x, y, z], ...]
        frame_num: 当前帧号
        """
        if not self.redis_client:
            print("Redis客户端未初始化")
            return

        try:
            # 写入节点数据
            for node_data in node_data_list:
                node_num = int(node_data[0])
                node_x = float(node_data[1])
                node_y = float(node_data[2])
                node_z = float(node_data[3])
                node_status = node_data[4]
                node_type = node_data[5]

                # 写入Redis
                self.redis_client.set(f'{node_num}_x', node_x)
                self.redis_client.set(f'{node_num}_y', node_y)
                self.redis_client.set(f'{node_num}_z', node_z)
                self.redis_client.set(f'{node_num}_status', node_status)
                self.redis_client.set(f'{node_num}_type', node_type)
                self.redis_client.set(f'{node_num}_frame', frame_num)
                self.redis_client.set(f'{node_num}_timestamp', time.time())

            # 写入总帧数（供Unity端同步）
            self.redis_client.set('total_frame', frame_num)

        except Exception as e:
            print(f"写入Redis失败: {e}")
            raise e


    @staticmethod
    def linear_interpolation_3d(p1, p2, t: float) -> Tuple:
        """
        两点之间线性插值
        :param p1: 第一个点的坐标 (x1, y1, z1)
        :param p2: 第二个点的坐标 (x2, y2, z2)
        :param t: 插值参数，0到1之间

        返回:
        插值点的坐标 (x, y, z)
        """
        # 确保输入是数值
        p1 = [float(coord) for coord in p1]
        p2 = [float(coord) for coord in p2]

        # 线性插值公式: p = p1 + t * (p2 - p1)
        point = [
            p1[0] + t * (p2[0] - p1[0]),
            p1[1] + t * (p2[1] - p1[1]),
            p1[2] + t * (p2[2] - p1[2])
        ]

        return tuple(point)


    def interpolate_all_nodes(self, prev_data: List[List], curr_data: List[List], num_points: int) -> List[List[List]]:
        """
        对所有节点进行插值，生成指定数量的插值点

        :param prev_data: 前一时刻的所有节点数据
        :param curr_data: 当前时刻的所有节点数据
        :param num_points: 需要生成的插值点数量
        :return: 插值结果列表 [frame1_data, frame2_data, ...]，每个元素是[[node_num, x, y, z], ...]
        """
        if not prev_data or not curr_data:
            print("插值数据不足")
            return []

        # 按节点编号排序确保对应关系
        prev_data_sorted = sorted(prev_data, key=lambda x: x[0])
        curr_data_sorted = sorted(curr_data, key=lambda x: x[0])

        # 检查节点数量是否一致
        if len(prev_data_sorted) != len(curr_data_sorted):
            print(f"节点数量不一致: 前次{len(prev_data_sorted)}个, 本次{len(curr_data_sorted)}个")
            return []

        # 生成所有插值点
        interpolation_results = []

        for i in range(num_points):

            # 计算当前插值参数t (0到1之间)
            t = i / (num_points - 1) if num_points > 1 else 0

            frame_data = []
            for j in range(len(prev_data_sorted)):
                node_num = prev_data_sorted[j][0]
                p1 = prev_data_sorted[j][1:4]  # [x, y, z]
                p2 = curr_data_sorted[j][1:4]  # [x, y, z]
                # print(f'p1:{p1}, p2:{p2}')
                node_status = prev_data_sorted[j][-2]
                node_type = prev_data_sorted[j][-1]

                # 线性插值
                interpolated_point = self.linear_interpolation_3d(p1, p2, t)

                frame_data.append([node_num, interpolated_point[0], interpolated_point[1], interpolated_point[2], node_status, node_type])

            interpolation_results.append(frame_data)

        return interpolation_results


    def frame_run(self, current_time: float, total_frames: int):
        """
        每一帧需要执行的功能：
            1. 检查是否需要读取新数据（根据时间间隔）
            2. 如果需要读取，更新数据并生成新的插值缓冲
            3. 从插值缓冲中取出当前帧数据写入Redis
        """
        # 检查是否需要读取新数据
        time_since_last_read = current_time - self.last_read_time

        if time_since_last_read >= self.read_interval or self.last_read_time == 0:
            # 需要读取新数据
            print(f"读取新数据 (距离上次读取: {time_since_last_read:.2f}s)")

            # 读取数据
            new_data = self.get_redis_data()

            if new_data:
                # 更新数据
                self.previous_data = self.current_data
                self.current_data = new_data

                # 如果是第一次读取，初始化previous_data为current_data
                if self.previous_data is None:
                    self.previous_data = self.current_data
                    print("首次读取数据，初始化previous_data")

                # 清空旧的插值缓冲
                self.interpolation_buffer = []

                # 生成新的插值缓冲
                if self.previous_data and self.current_data:
                    # 生成read_interval*framerate个插值点（包括起点和终点）
                    self.interpolation_buffer = self.interpolate_all_nodes(
                        self.previous_data,
                        self.current_data,
                        self.total_frames_between_reads
                    )

                    if self.interpolation_buffer:
                        print(f"生成 {len(self.interpolation_buffer)} 个插值点")
                    else:
                        print("插值缓冲生成失败")

                # 重置当前间隔内的帧计数器
                self.current_frame_in_interval = 0

                # 更新最后读取时间
                self.last_read_time = current_time
            else:
                print("读取数据为空，保持原有数据")

        # 从插值缓冲中获取当前帧数据
        if self.interpolation_buffer:
            # 计算当前帧在缓冲中的位置
            buffer_index = self.current_frame_in_interval % len(self.interpolation_buffer) if self.interpolation_buffer else 0

            # 确保索引在有效范围内
            if 0 <= buffer_index < len(self.interpolation_buffer):
                frame_data = self.interpolation_buffer[buffer_index]

                # 写入Redis
                self.write_redis(frame_data, total_frames)

                # 更新统计
                self.stats['total_frames_processed'] += 1

                # 每100帧输出一次统计信息
                if total_frames % 100 == 0:
                    print(f"已处理 {total_frames} 帧，插值缓冲大小: {len(self.interpolation_buffer)}，当前索引: {buffer_index}")

                # 增加当前间隔内的帧计数器
                self.current_frame_in_interval += 1

                # 如果超过了插值缓冲大小，重置（这表示需要读取新数据了）
                if self.current_frame_in_interval >= len(self.interpolation_buffer):
                    print("插值缓冲已用完，等待下一次数据读取")
            else:
                print(f"插值缓冲索引越界: {buffer_index}/{len(self.interpolation_buffer)}")
        else:
            # 如果没有插值缓冲，直接使用当前数据
            if self.current_data:
                self.write_redis(self.current_data, total_frames)
                print(f"无插值缓冲，使用当前数据 (第{total_frames}帧)")
            else:
                print("无可用数据")


    def run(self, duration: Optional[int] = None):
        """
        主运行循环
        :param duration: 运行持续时间（秒），None表示无限运行
        """
        print(f"中间层服务启动...")
        print(f"读取间隔: {self.read_interval}秒, 帧率: {self.framerate}Hz")
        print(f"每次读取间隔内帧数: {self.total_frames_between_reads}")

        self.running = True
        start_time = time.time()
        frame_interval = 1.0 / self.framerate
        total_frames = 0

        while self.running:
            try:
                frame_start = time.time()
                current_time = time.time()

                # 执行帧处理
                self.frame_run(current_time, total_frames)

                total_frames += 1

                # 计算帧处理耗时
                process_time = time.time() - frame_start
                sleep_time = max(0, frame_interval - process_time)

                # 控制帧率
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # 如果处理时间超过帧间隔，发出警告
                    if process_time > frame_interval * 1.1:  # 超过10%
                        print(f"警告: 帧处理时间({process_time:.3f}s)超过帧间隔({frame_interval:.3f}s)")

                # 检查运行时间
                if duration and (current_time - start_time) > duration:
                    print(f"运行时间到达 {duration} 秒，停止运行")
                    self.running = False

            except KeyboardInterrupt:
                print("接收到中断信号，停止运行")
                self.running = False
                break
            except Exception as e:
                print(f"帧处理出错: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1)  # 出错后等待1秒继续

        # 输出统计信息
        print("\n运行统计:")
        print(f"总处理帧数: {self.stats['total_frames_processed']}")
        print(f"总读取次数: {self.stats['total_reads']}")
        print(f"最后读取节点数: {self.stats['last_read_size']}")
        print(f"运行时间: {time.time() - start_time:.2f}秒")
        print("中间层服务停止")


    def start_run_thread(self, duration: Optional[int] = None):
        """
        启动运行线程

        参数:
        duration: 运行持续时间（秒），None表示无限运行
        """
        if not self.redis_client:
            print("请先调用 create_redis() 初始化Redis连接")
            return None

        # 创建并启动线程
        run_thread = threading.Thread(
            target=self.run,
            args=(duration,),
            daemon=True  # 设置为守护线程，主程序退出时自动结束
        )
        run_thread.start()

        print(f"中间层服务已启动 - 帧率: {self.framerate}Hz, 读取间隔: {self.read_interval}秒")
        if duration:
            print(f"运行持续时间: {duration}秒")

        return run_thread


    def stop(self):
        """停止运行"""
        self.running = False
        print("正在停止中间层服务...")


if __name__ == '__main__':

    m = MiddleLayer(
        host='192.166.51.23',
        port=6379,
        password='uav123',
        read_interval=2,  # 每2秒从地面站读取一次数据
        framerate=20      # 20Hz插值帧率
    )

    # 创建Redis连接
    if m.create_redis():
        # 测试坐标转换
        test_lat = 39.908824
        test_lon = 116.393956
        result = m.lonlat2cartesian(test_lat, test_lon)
        print(f"坐标转换测试: ({test_lat}, {test_lon}) -> {result}")

        # 测试数据读取
        test_data = m.get_redis_data()
        print(f"读取到 {len(test_data)} 个节点数据")

        # 启动中间层服务（运行30秒测试）
        print("\n启动中间层服务...")
        thread = m.start_run_thread(duration=300)

        # 等待线程结束
        if thread:
            thread.join(timeout=35)

        print("中间层服务测试完成")
    else:
        print("Redis连接失败，无法启动服务")