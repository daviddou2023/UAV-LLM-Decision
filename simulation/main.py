"""
MARL主模块 - 空域拦截系统 v8.0
1. 优先接入老师的实时 Redis 数据
2. 路径规划升级为 3D，显示仍保持 2D 投影
3. 场景尺寸支持 1/2/3/4/5/10km 动态切换
4. 增加雷达丢失、识别不稳、目标失联后的保守重规划
"""
import argparse
import math
import os
import random
import re
import time

from core.common import (
    CFG,
    EState,
    EType,
    IRole,
    IState,
    angle_diff,
    compute_attitude_from_motion,
    create_enemy,
    create_interceptor,
    dist2d,
    dist3d,
    move_entity,
)
from decision.cooperation import InterceptionAssigner
from perception.radar_feed import FusionTrackFeed, TeacherDataFeed
from decision.deconfliction import DeconflictionController
from core.geo import GeoReference
from decision.llm_kit import BattlefieldAnalyst
from decision.llm_task_constraints import parse_llm_number, parse_llm_task_constraints_command
from integrations.redis_export import (
    RedisNodeWriter,
    build_geo_hash_payload,
    build_payload,
    build_teacher_redis_payload,
    enemy_rows,
    friendly_rows,
    planned_node_nums,
    stale_keys,
)
from perception.udp_gateway import TeacherUDPFeed, UDPFramePublisher
# 111
try:
    from ui.voice import VoiceEngine
    _HAS_VOICE = True
except ImportError:
    _HAS_VOICE = False

try:
    from station.exporter import PlanExportConfig, PlannerExporter
    _HAS_PLAN_EXPORT = True
except ImportError:
    PlanExportConfig = None
    PlannerExporter = None
    _HAS_PLAN_EXPORT = False


def _etype_from_track(track):
    """
        根据轨迹数据的文本信息，推断敌机的战术类型。

        参数:
            track (dict): 上游传来的单个目标轨迹字典数据，通常包含外部ID、坐标、速度、状态等。

        返回:
            EType: 定义在 core/common.py 中的目标类型枚举值。
    """
    """
        
        上游：perception/radar_feed.py 和 perception/udp_gateway.py：负责从Redis或者UDP端口拉去外部的原始JSON数据帧
            _create_live_enemy(self, track)：当雷达发现一个全新的敌机目标，调用此函数打标签
            _apply_track_to_enemy(self, enemy, track)：雷达持续刷新已存在目标状态时调用此函数更新敌机状态是否变化
        下游：ProNav (底层制导算法, simulation/main.py)：如果是EType.LOITER或者EType.DASH，gain会放大到8.0
            _desired_interceptor_speed (速度规划, simulation/main.py)：如果是EType.LOITER或者EType.DASH，己方拦截机自动解开限速，以 CFG.INTERCEPTOR_BOOST_SPEED (极速) 进行冲刺
            _check_net_capture (网捕判定逻辑, simulation/main.py)：如果是EType.LOITER巡飞弹，该函数会触发特殊的“紧急收网 (emergency_strike)”逻辑
            decision/cooperation.py：根据EType进行威胁度排序。
            _update_enemy_detection (simulation/main.py)：系统在UI日志上将etype的中文属性翻译给指挥员看
    """
    # 将track字典中的'external_id' (外部标识) 和 'status' (状态描述) 提取出来进行拼接
    # 拼接后的text “uav_dash_01 诱饵”或者 "target_103 高速突防"
    text = f"{track.get('external_id', '')} {track.get('status', '')}".lower()
    if "decoy" in text or "诱饵" in text:
        return EType.DECOY
    if "loiter" in text or "巡飞" in text:
        return EType.LOITER
    if "dash" in text or "高速" in text:
        return EType.DASH
    if "snake" in text or "s型" in text:
        return EType.SNAKE
    if "jink" in text or "闪避" in text:
        return EType.JINK
    # 默认返回常规目标
    return EType.NORMAL


class ProNav:
    """
        向上依赖：core/common.py：ProNav算法的基础组件库
            CFG.PRONAV_GAIN：提供默认比例导引系数。
            CFG.INTERCEPTOR_MAX_ANG：限制无人机的物理转弯能力。
            angle_diff：用于计算角度偏差。
            EType.LOITER / EType.DASH：用于识别高威胁目标并动态调整飞控参数。

        向下被调用：
            _execute_local_plan()：局部运动执行函数
                飞向静态节点时：会调用 ProNav.guide_point(it, plan.command_point)
                末端动态追踪：当plan.allow_terminal_direct 为真（进入最终撞击阶段），调用ProNav.command(it, enemy)
            _guide_xxx：各种特定的飞行状态向导函数

    """
    @staticmethod
    def command(intr, enemy):
        """
        动态目标拦截：经典的比例导引律 (Proportional Navigation Law)。
        功能：计算己方拦截机 (intr) 在追踪敌方移动目标 (enemy) 时，当前需要输出的转向角速度。
        """
        # 1. 计算相对位置和距离
        dx = enemy['x'] - intr['x']
        dy = enemy['y'] - intr['y']
        r = math.sqrt(dx * dx + dy * dy)
        if r < 1.0:
            return 0.0 # 距离极近时停止转向

        # 2. 将航向角速度转化为弧度计算相对速度
        er = math.radians(enemy['heading'])
        ir = math.radians(intr['heading'])
        # 相对速度 = 敌机速度分量 - 己方速度分量
        rvx = enemy['speed'] * math.cos(er) - intr['speed'] * math.cos(ir)
        rvy = enemy['speed'] * math.sin(er) - intr['speed'] * math.sin(ir)
        # 计算接近速度 = 两机距离缩小的速率
        vc = -(dx * rvx + dy * rvy) / r
        if vc < 0.1:
            vc = 0.1
        # 视线角速率
        los_rate = (dx * rvy - dy * rvx) / (r * r)


        gain = CFG.PRONAV_GAIN # 默认导引系数
        # 对于特殊高机动目标，在末端动态增加增益，使其转弯更加锐利
        if enemy['type'] in (EType.LOITER, EType.DASH) and r < max(600.0, 0.15 * CFG.INTERCEPT_FAIL_LINE):
            gain = 8.0

        # 计算比例导引（转向角速度 = 导引系数 * 接近速度 * 视线角速率）
        w = math.degrees(gain * vc * los_rate)
        # 限制最大转弯角速度，模拟固定翼无人机的物理极限
        return max(-CFG.INTERCEPTOR_MAX_ANG, min(CFG.INTERCEPTOR_MAX_ANG, w))

    @staticmethod
    def guide_point(intr, pt):
        """
            静态航点导引：纯追踪/按点飞行。
            功能：计算己方无人机 (intr) 飞向固定航点 (pt) 时，需要的转向角速度。
        """
        # 计算无人机到目标航点的XY偏差
        dx = pt[0] - intr['x']
        dy = pt[1] - intr['y']
        # 计算期望的绝对航向角（目标点在无人机的哪个方向）
        desired = math.degrees(math.atan2(dy, dx))
        # 计算期望航向与当前航向之间的夹角差
        d = angle_diff(desired, intr['heading'])
        # 简单乘以因子修正航向，限制在无人机最大的物理转弯能力范围内。
        return max(-CFG.INTERCEPTOR_MAX_ANG, min(CFG.INTERCEPTOR_MAX_ANG, d * 0.5))


def _planned_demo_node_nums(env, friendly_start, enemy_start):
    """
        计算当前演示(Demo)环境中所有预期活跃的实体节点编号集合。

        参数:
            env (InterceptionEnvironment): 当前的仿真环境实例，存储了所有实体数据和统计信息。
            friendly_start (int): 己方无人机对外发布的起始编号 (例如配置里常见的 1)。
            enemy_start (int): 敌方目标对外发布的起始编号 (例如配置里常见的 100，用来防冲突)。

        返回:
            set/list: 返回一个包含所有预期节点编号的集合。
    """
    # 实际逻辑委托给了integrations/redis_export.py 模块中的 planned_node_nums 函数
    return planned_node_nums(
        # 当前己方拦截机的实际数量
        interceptor_count=len(env.interceptors),
        # 提取系统运行至今，总共生成过的（含阵亡的）敌机总数
        # 让底层也能考虑到已经被击毁的敌机，从而通知外部清空它们的旧数据
        total_enemy_count=env.stats.get("total_enemies", 0),
        # 提取当前场上仍存活的敌机数量
        live_enemy_count=len(env.enemies),
        # 传入己方和敌方的起始偏移编号
        friendly_start=friendly_start,
        enemy_start=enemy_start,
    )


def _normalize_publish_side(publish_side):
    """
        将用户指定的发布侧参数规范化为系统标准格式。

        参数:
            publish_side: 外部传入的原始参数，可能为 None、字符串，空格等非法字符。

        返回:
            str: 规范化后的字符串，值限定为 'all'、'friendly' 或 'enemy' 之一。



        向上：对应启动配置传入的发布侧参数
        向下：_publish_numbered_rows 函数：该函数利用规范化后的publish_side来决定在组装负载时，是只提取己方数据还是敌方数据，还是all
            _planned_publish_node_nums 函数：在执行数据清理（清理 Redis 中过期节点编号）时，它会参考这个参数，确保只清理对应侧（己方或敌方）的节点，避免误删。
            主程序创建数据发布器实例xxxPublisher：初始化调用此函数将配置参数锁定为合法值
            对应启动配置传递的发布侧参数
    """

    # 统一将数据清洗为字符串-去除首尾空格-小写格式
    side = str(publish_side or "all").strip().lower()
    return side if side in ("all", "friendly", "enemy") else "all"


def _publish_numbered_rows(env, friendly_start, enemy_start, publish_side):
    """
        功能：根据发布侧要求，提取并编号己方和敌方数据行。

        参数:
            env: 当前拦截环境实例。
            friendly_start: 己方编号起始基数。
            enemy_start: 敌方编号起始基数。
            publish_side: 指定发布范围 ('all', 'friendly', 'enemy')。

        返回:
            tuple: (numbered_friendlies, numbered_enemies)
                   两个列表，包含带编号的实体行数据。


        向上依赖：_normalize_publish_side (simulation/main.py)：确保输入参数符合规范
                InterceptionEnvironment (simulation/main.py)：本函数处理的原始数据（env.interceptors 和 env.enemies）均来源于此
                friendly_rows 和 enemy_rows（integrations/redis_export.py）：将原始的飞机对象转化成下游所需要的字典或者列表格式
        向下被调用：DemoRedisPublisher 类 (simulation/main.py)：在maybe_publish 方法中调用 _publish_numbered_rows 获取带编号的数据，随后调用 build_payload 将其封装成 Redis 可写入的格式。
                TeacherFriendlyRedisPublisher 类 (simulation/main.py)：同样调用此函数来获取数据，以便将当前仿真快照同步给教师端系统。
                GeoHashRedisPublisher 类 (simulation/main.py)：在进行经纬度 Hash 映射发布时，也依赖此函数获取经过编号处理的实体列表
    """

    # 使用规范化函数
    publish_side = _normalize_publish_side(publish_side)
    # 获取己方数据列表
    friendlies = env.visible_interceptors() if hasattr(env, "visible_interceptors") else env.interceptors
    # 如果需要发布己方数据，调用friendly_rows进行编号转换
    numbered_friendlies = friendly_rows(friendlies, friendly_start) if publish_side in ("all", "friendly") else []
    # 如果需要发布敌方数据，调用enemy_rows进行编号转换
    numbered_enemies = enemy_rows(getattr(env, "enemies", []), enemy_start) if publish_side in ("all", "enemy") else []
    return numbered_friendlies, numbered_enemies


def _planned_publish_node_nums(env, friendly_start, enemy_start, publish_side):
    """
        计算当前指令下，预期发布的数据节点编号集合。



        向上依赖：_normalize_publish_side
                planned_node_nums（integrations/redis_export.py）
                启动配置中的 PUBLISH_SIDE 参数
        向下被调用：发布类接收_planned_publish_node_nums用来做垃圾回收
                
    """

    # 规范化参数，确保传入的是 "all","friendly"，"enemy"
    publish_side = _normalize_publish_side(publish_side)
    node_nums = set()
    # 如果要求发送我方数据
    if publish_side in ("all", "friendly"):
        # 将己方节点编号加入集合
        node_nums.update({friendly_start + idx for idx in range(len(env.interceptors))})
    # 如果要求发送敌方数据
    if publish_side in ("all", "enemy"):
        # 计算敌方节点编号
        node_nums.update(
            planned_node_nums(
                interceptor_count=0,
                total_enemy_count=env.stats.get("total_enemies", 0),
                live_enemy_count=len(env.enemies),
                friendly_start=friendly_start,
                enemy_start=enemy_start,
            )
        )
        # 数据过滤，确保只有符合enemy编号区间的ID才会保留
        node_nums = {num for num in node_nums if num >= enemy_start}
    return node_nums


def _enabled_flag(value, default=True):
    """
        将多种格式的输入转换为布尔值 (True/False)。

        参数:
            value: 待转换的值，可以是 None, bool, str, int 等。
            default: 当 value 无法识别时的默认返回结果。

        返回:
            bool: 转换后的布尔结果。
    """
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "on", "true", "yes", "y", "enable", "enabled"):
        return True
    if text in ("0", "off", "false", "no", "n", "disable", "disabled"):
        return False
    return bool(default)


class DemoRedisPublisher:
    """
    将当前仿真环境的最新“快照（状态帧）”实时同步到 Redis 数据库中，并且负责自动清理已经失效的节点，使得显示层只需要读取redis数据库就可以渲染出图像

    向上依赖：run_demo 主循环 (simulation/main.py)：主循环里，每一帧结束前都会调用 redis_publisher.maybe_publish(env)
            integrations/redis_export.py：RedisNodeWriter: 处理所有与 Redis 通信的底层 Socket 细节
                                build_payload: 把 Python 对象翻译成 Redis 认识的 JSON 字符串。
                                stale_keys: 一个小工具函数，执行集合运算（published_nodes - active_nodes），找出死亡的节点编号

    """
    def __init__(self, host="127.0.0.1", port=6379, db=0, publish_interval=0.5,
                 friendly_start=1, enemy_start=101, cleanup_node_nums=None, password=None,
                 publish_side="all"):
        # 实例化底层的Redis写入工具
        self.writer = RedisNodeWriter(host=host, port=port, db=db, password=password)
        # 发布时间间隔，限制最低为33帧/秒，防止刷爆Redis
        self.publish_interval = max(0.03, float(publish_interval))
        # 己方和敌方发布到Redis的起始编号ID
        self.friendly_start = int(friendly_start)
        self.enemy_start = int(enemy_start)
        # 规范发布侧参数，确保是“all”，“friendly”，或者“enemy”
        self.publish_side = _normalize_publish_side(publish_side)
        # 当前发布的帧序号
        self.frame_num = 0
        # 记录上一次发布的时间戳，用于控制发送频率
        self.last_publish_at = 0.0
        # 记录上一帧发布了哪些节点ID,用于做差异对比和垃圾回收
        self.published_nodes = set()
        # 启动时需要强制清理的节点编号集合
        self.cleanup_node_nums = set(cleanup_node_nums or set())
        # 初始化清理
        self.writer.cleanup_legacy_keys()
        self.cleanup()

    def cleanup(self):
        """
            彻底清理 Redis 中与当前实例相关的所有节点数据。
        """
        self.writer.cleanup_node_nums(self.cleanup_node_nums | self.published_nodes)
        self.published_nodes = set()

    def maybe_publish(self, env, force=False):
        """
                尝试发布当前环境帧数据。如果不满足发布频率或条件，则跳过。

                参数:
                    env: 当前的拦截仿真环境对象。
                    force: 是否无视时间间隔，强制发布。
                """
        # 如果是接入外部真实数据格式，但是目前还没收到数据，并且没在本地跑demo,就不发空数据
        if (
            getattr(env, "source", None) in ("udp", "redis", "auto", "fusion")
            and not getattr(env, "demo_mode", False)
            and not getattr(env, "has_live_data", False)
        ):
            return False
        now = time.time()

        # 频率控制：如果没开启强制发送，并且距离上次发送还没达到publish_interval，就直接返回
        if not force and (now - self.last_publish_at) < self.publish_interval:
            return False
        self.frame_num += 1

        # 提取数据
        numbered_friendlies, numbered_enemies = _publish_numbered_rows(
            env, self.friendly_start, self.enemy_start, self.publish_side
        )

        # 将行数据转化为 Redis 的 key-value 字典 (payload)，并返回当前帧所有活跃节点的集合 (active_nodes)
        payload, active_nodes = build_payload(numbered_friendlies, numbered_enemies, self.frame_num, now)
        # 使用mset命令一次性将所有数据并发写入Redis,效率极高
        self.writer.mset(payload)
        self.writer.delete(stale_keys(self.published_nodes, active_nodes))
        # 更新状态
        self.published_nodes = active_nodes
        self.last_publish_at = now
        return True


class TeacherFriendlyRedisPublisher:
    """
    和class DemoRedisPublisher功能类似，也是将当前环境中的实体状态打包并周期性写入redis,区别是
    调用build_teacher_redis_payload组装数据，并采用无条件的发布逻辑

    向上依赖：run_demo 函数或者run_fusion_custom.sh配置了publish_redis_mode="teacher-friendly“才会开启
            build_teacher_redis_payload和RedisNodeWriter（integrations/redis_export.py）


    """

    def __init__(self, host="127.0.0.1", port=6379, db=0, publish_interval=0.5,
                 friendly_start=1, enemy_start=101, password=None, publish_side="all",
                 cleanup_node_nums=None):
        # 实例化底层的redis写入工具
        self.writer = RedisNodeWriter(host=host, port=port, db=db, password=password)
        # 限制最高发布频率
        self.publish_interval = max(0.03, float(publish_interval))
        # 写入redis 的id编号偏移量
        self.friendly_start = int(friendly_start)
        self.enemy_start = int(enemy_start)
        # 规范化发布侧参数
        self.publish_side = _normalize_publish_side(publish_side)
        self.frame_num = 0
        self.last_publish_at = 0.0
        # 记录当前在Redis中存活的节点id
        self.published_nodes = set()
        self.cleanup_node_nums = set(cleanup_node_nums or set())
        self.cleanup()

    def cleanup(self):
        """
                清理函数：将指定的节点从 Redis 数据库中抹除。
        """
        self.writer.cleanup_node_nums(self.cleanup_node_nums | self.published_nodes)
        self.published_nodes = set()

    def maybe_publish(self, env, force=False):
        """
                尝试发布当前环境帧数据到 Redis。

                参数:
                    env: 当前的仿真环境 (InterceptionEnvironment)
                    force: 是否强制跳过时间间隔检查
                """
        now = time.time()
        if not force and (now - self.last_publish_at) < self.publish_interval:
            return False
        self.frame_num += 1
        # 提取带编号的己方和敌方编号
        numbered_friendlies, numbered_enemies = _publish_numbered_rows(
            env, self.friendly_start, self.enemy_start, self.publish_side
        )
        payload, active_nodes = build_teacher_redis_payload(numbered_friendlies, numbered_enemies, self.frame_num, now)
        self.writer.mset(payload)
        self.writer.delete(stale_keys(self.published_nodes, active_nodes))
        self.published_nodes = active_nodes
        self.last_publish_at = now
        return True


class GeoHashRedisPublisher:
    """
    将二维平面坐标转化为真实经纬度坐标

    向上依赖：core/geo.py：类初始化时实例化的 GeoReference 对象通常来自于项目中的 core/geo.py 文件
            integrations/redis_export.py：build_geo_hash_payload：这是 integrations/redis_export.py 提供的一个专属打包器
            RedisNodeWriter.bulk_hset:专门针对 Hash 结构封装的批量写入方法
            run_demo:当 publish_redis_mode="geo-hash" 时，系统才会激活这个发布器


    """
    def __init__(
        self,
        host="127.0.0.1",
        port=6379,
        db=0,
        publish_interval=0.5,
        friendly_start=1,
        enemy_start=101,
        password=None,
        # 设定地理参考原点
        geo_origin_lat=34.2663,
        geo_origin_lon=108.9549,
        publish_side="all",
    ):
        # 初始化Redis底层写入器
        self.writer = RedisNodeWriter(host=host, port=port, db=db, password=password)
        # 限制最高发布频率
        self.publish_interval = max(0.03, float(publish_interval))
        self.friendly_start = int(friendly_start)
        self.enemy_start = int(enemy_start)
        self.publish_side = _normalize_publish_side(publish_side)
        # 实例化地理坐标参考系转换器
        self.geo_reference = GeoReference(origin_lat=geo_origin_lat, origin_lon=geo_origin_lon)
        self.frame_num = 0
        self.last_publish_at = 0.0
        self.published_keys = set()

    def cleanup(self):
        """
                清理所有由此实例发布到 Redis 的键值。
                """
        if self.published_keys:
            # 删除所有记录在案的hash key
            self.writer.delete(sorted(self.published_keys))
            self.published_keys = set()

    def maybe_publish(self, env, force=False):
        """
                尝试执行发布操作。
                """
        # 如果是真实数据接入模式但还没收到数据，且不是本地 Demo 模式，则不发布空数据
        if (
            getattr(env, "source", None) in ("udp", "redis", "auto", "fusion")
            and not getattr(env, "demo_mode", False)
            and not getattr(env, "has_live_data", False)
        ):
            return False
        now = time.time()
        if not force and (now - self.last_publish_at) < self.publish_interval:
            return False
        self.frame_num += 1

        # 提取环境中的己方和敌方数据
        numbered_friendlies, numbered_enemies = _publish_numbered_rows(
            env, self.friendly_start, self.enemy_start, self.publish_side
        )
        # 调用专门的build_geo_hash_payload
        payload, active_keys = build_geo_hash_payload(
            numbered_friendlies,
            numbered_enemies,
            self.frame_num,
            now,
            geo_reference=self.geo_reference,
        )
        # 使用 bulk_hset 将数据以 Redis Hash 数据类型批量写入
        self.writer.bulk_hset(payload)
        stale = self.published_keys - active_keys
        if stale:
            self.writer.delete(sorted(stale))
        self.published_keys = active_keys
        self.last_publish_at = now
        return True


class WaveManager:
    """
    没有外部真实雷达数据接入时，系统随机化态势生成敌机
    向上依赖：core/common.py：CFG (全局配置)
            create_enemy（core/common.py）：此函数用于构建敌机的状态字典

    向下被调用：主循环驱动 (_spawn_demo_waves)：demo_mode（演示模式）且没收到外部真实数据，每帧会调用 WaveManager.update(self.time)，把新生成的敌机塞进 env.enemies 列表里参加战斗
            UI 交互 (add_enemy_target)：用户在UI界面点击“添加敌机”按钮，会专门去读取 WaveManager.nxt_id，确保手动生成的 ID 永远比波次管理器里的最大 ID 还要大

    """
    def __init__(self, rng, waves=None):
        """
                初始化波次管理器。

                参数:
                    rng: 随机数生成器 (Random Number Generator)，保证生成的随机具有可复现性(受seed控制)。
                    waves: 波次配置列表。如果未提供，则使用默认的两波次攻击。
                """
        self.rng = rng

        # 如果外部没有传入波次计划，默认生成两波攻击：
        # 第 0 秒生成 12 架；第 CFG.WAVE_INTERVAL 秒生成 13 架。总计 25 架。

        self.waves = [dict(wave) for wave in (waves or [
            {'time': 0, 'count': 12},
            {'time': CFG.WAVE_INTERVAL, 'count': 13},
        ])]
        # 当前已经触发到了第几个波次的进攻
        self.idx = 0
        # 下一个生成的敌机将分配到的ID编号
        self.nxt_id = 0

    def update(self, t):
        """
                每一帧被环境调用，检查是否需要释放新一波敌机。

                参数:
                    t: 当前的仿真时间 (秒)

                返回:
                    new: 刚刚生成的敌机对象列表
                """
        new = []
        while self.idx < len(self.waves) and t >= self.waves[self.idx]['time']:
            wave = self.waves[self.idx]
            # 根据波次配置的数量，循环生成敌机
            for _ in range(wave['count']):
                x = self.rng.uniform(CFG.AREA_WIDTH * 0.1, CFG.AREA_WIDTH * 0.9)
                # 随机生成速度：基础速度+正负扰动量
                speed = CFG.ENEMY_SPEED + self.rng.uniform(-CFG.ENEMY_SPEED_VAR, CFG.ENEMY_SPEED_VAR)
                # 随机生成航向：基础航向+角度扰动
                heading = CFG.ENEMY_HDG_BASE + self.rng.uniform(-CFG.ENEMY_HDG_VAR, CFG.ENEMY_HDG_VAR)
                # 调用工厂函数创建敌机字典，传入 ID、X位置、速度、航向、生成时间、以及一个用于决定其机动类型的随机因子
                enemy = create_enemy(self.nxt_id, x, speed, heading, t, self.rng.random())
                # 对这个系统自己demo的目标打上标签，标注并非外部真实雷达数据
                enemy['source'] = 'demo'
                new.append(enemy)
                self.nxt_id += 1
            self.idx += 1
        return new

    @property
    def total(self):
        """
                计算整个计划中总共会生成多少架敌机。
                用于给 UI 或统计模块提供一个“总数”参考。
                """
        return sum(w['count'] for w in self.waves)


class InterceptionEnvironment:
    """
    它负责维护所有对象的状态（无人机、敌机）、推进时间的流逝（每一帧的物理运动）、判定游戏规则（碰撞、突防、燃料耗尽），并负责协调各种外部插件（雷达数据源、大模型分析器、底层避碰算法）。
    上游输入：perception/radar_feed.py 和 perception/udp_gateway.py：环境类通过持有 TeacherDataFeed 或 TeacherUDPFeed 实例，在 step() 中不断调用 _pump_live_data()，将这些文件拉取到的外部网络 JSON 数据转换为内部的 self.enemies
    内部依赖：decision/cooperation.py (协同与分配)：在 step() 中，它提取存活的敌我列表，扔给 self.assigner.update()，由这个文件里的算法算出最优匹配。
            decision/deconfliction.py (避碰与航线)：当飞机起飞或返航时，环境会调用 self.deconfliction.plan_local_motion() 来确保无人机不会相撞或飞出走廊
            decision/llm_kit.py (大模型引擎)：持有 BattlefieldAnalyst，用来处理所有“人类语言”和智能态势播报。
            core/common.py (基础库)：大量使用其中的枚举（EState, IState）、数学公式（计算距离、角度）以及全局配置常量（CFG）


    """
    def __init__(self, seed=42, scene_km=10.0, source="auto",
                 redis_host="192.166.51.21", redis_port=6379, redis_db=0, redis_password=None,
                 intercept_mode="hybrid", demo_case=None,
                 udp_in_host="0.0.0.0", udp_in_port=8020,
                 enemy_redis_format="auto", friendly_return_source="udp",
                 geo_origin_lat=34.2663, geo_origin_lon=108.9549,
                 enemy_assoc="on", enemy_assoc_max_distance=450.0,
                 enemy_assoc_max_altitude=140.0, enemy_assoc_keep_sec=18.0,
                 enemy_hash_remap_mode="direct",
                 enemy_hash_center_x_ratio=0.5, enemy_hash_lateral_scale=1.0,
                 enemy_hash_range_scale=5.0, enemy_hash_start_range_m=0.0,
                 enemy_hash_y_offset_m=0.0,
                 enemy_hash_hide_outbound="off",
                 enemy_flat_remap_mode="legacy",
                 enemy_flat_rotate_deg=135.0, enemy_flat_flip_x="off",
                 enemy_flat_flip_y="off", enemy_flat_scale=1.0,
                 enemy_flat_center_x_ratio=0.5, enemy_flat_center_y_ratio=0.2,
                 demo_interference_enable=True, demo_interference_visible=True,
                 demo_scheme=0):
        """
            初始化环境。接收来自启动脚本 (run_fusion_custom.sh) 的几十个配置参数。
        """

        self.seed = seed
        self.source = source
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.redis_db = redis_db
        self.redis_password = redis_password
        self.udp_in_host = udp_in_host
        self.udp_in_port = udp_in_port
        self.enemy_redis_format = str(enemy_redis_format or "auto").strip().lower()
        self.friendly_return_source = str(friendly_return_source or "udp").strip().lower()
        self.enemy_assoc = str(enemy_assoc or "on").strip().lower()
        self.enemy_assoc_max_distance = float(enemy_assoc_max_distance)
        self.enemy_assoc_max_altitude = float(enemy_assoc_max_altitude)
        self.enemy_assoc_keep_sec = float(enemy_assoc_keep_sec)
        self.enemy_hash_remap_mode = str(enemy_hash_remap_mode or "direct").strip().lower()
        self.enemy_hash_center_x_ratio = float(enemy_hash_center_x_ratio)
        self.enemy_hash_lateral_scale = float(enemy_hash_lateral_scale)
        self.enemy_hash_range_scale = float(enemy_hash_range_scale)
        self.enemy_hash_start_range_m = float(enemy_hash_start_range_m)
        self.enemy_hash_y_offset_m = float(enemy_hash_y_offset_m)
        self.enemy_hash_hide_outbound = str(enemy_hash_hide_outbound or "off").strip().lower()
        self.enemy_flat_remap_mode = str(enemy_flat_remap_mode or "legacy").strip().lower()
        self.enemy_flat_rotate_deg = float(enemy_flat_rotate_deg)
        self.enemy_flat_flip_x = str(enemy_flat_flip_x or "off").strip().lower()
        self.enemy_flat_flip_y = str(enemy_flat_flip_y or "off").strip().lower()
        self.enemy_flat_scale = float(enemy_flat_scale)
        self.enemy_flat_center_x_ratio = float(enemy_flat_center_x_ratio)
        self.enemy_flat_center_y_ratio = float(enemy_flat_center_y_ratio)
        self.geo_origin_lat = float(geo_origin_lat)
        self.geo_origin_lon = float(geo_origin_lon)
        self.geo_reference = GeoReference(origin_lat=self.geo_origin_lat, origin_lon=self.geo_origin_lon)
        self.intercept_mode = intercept_mode
        self.demo_case = demo_case
        self.demo_interference_enabled = _enabled_flag(demo_interference_enable, True)
        self.demo_interference_visible = _enabled_flag(demo_interference_visible, True)
        self.demo_scheme = 0
        self.demo_scheme_name = "默认"
        self.demo_strategy_mode = "cooperative"
        self.demo_baseline_ready_count = None
        self.llm_interference_auto_replan_enabled = False
        self.llm_replan_boost_until = -1.0
        self.llm_decision_title = "LLM上层决策链"
        self.llm_decision_lines = []
        self.llm_decision_color = "pink"
        self.base_interceptor_count = int(CFG.NUM_INTERCEPTORS)
        self.demo_showcase_active = False
        self.demo_interference_zones = []
        self.demo_interference_limit = 0
        if int(demo_scheme or 0) in (1, 2, 3):
            self._apply_demo_scheme_settings(int(demo_scheme))

        # 实例化LLM战场情报分析员，用于后续的态势文字播报
        self.analyst = BattlefieldAnalyst()

        # 根据数据源配置，挂载不同的底层雷达监听器（TeacherUDPFeed / TeacherDataFeed）
        if source == "udp": # 实时监听并解析局域网中通过UDP广播出来的数据
            self.feed = TeacherUDPFeed(
                bind_host=udp_in_host,
                port=udp_in_port,
                geo_reference=self.geo_reference,
            )
        elif source in ("auto", "redis"): # 连接到指定的Redis内存数据库服务器，pub/sub读取
            self.feed = TeacherDataFeed(
                host=redis_host,
                port=redis_port,
                db=redis_db,
                password=redis_password,
                redis_format=self.enemy_redis_format,
                geo_reference=self.geo_reference,
                enemy_assoc=self.enemy_assoc,
                enemy_assoc_max_distance=self.enemy_assoc_max_distance,
                enemy_assoc_max_altitude=self.enemy_assoc_max_altitude,
                enemy_assoc_keep_sec=self.enemy_assoc_keep_sec,
                enemy_hash_remap_mode=self.enemy_hash_remap_mode,
                enemy_hash_center_x_ratio=self.enemy_hash_center_x_ratio,
                enemy_hash_lateral_scale=self.enemy_hash_lateral_scale,
                enemy_hash_range_scale=self.enemy_hash_range_scale,
                enemy_hash_start_range_m=self.enemy_hash_start_range_m,
                enemy_hash_y_offset_m=self.enemy_hash_y_offset_m,
                enemy_hash_hide_outbound=self.enemy_hash_hide_outbound,
                enemy_flat_remap_mode=self.enemy_flat_remap_mode,
                enemy_flat_rotate_deg=self.enemy_flat_rotate_deg,
                enemy_flat_flip_x=self.enemy_flat_flip_x,
                enemy_flat_flip_y=self.enemy_flat_flip_y,
                enemy_flat_scale=self.enemy_flat_scale,
                enemy_flat_center_x_ratio=self.enemy_flat_center_x_ratio,
                enemy_flat_center_y_ratio=self.enemy_flat_center_y_ratio,
            )
        elif source == "fusion": #  异构多源数据输入，将多源数据先进行fusion,再读取
            enemy_feed = TeacherDataFeed(
                host=redis_host,
                port=redis_port,
                db=redis_db,
                password=redis_password,
                redis_format=self.enemy_redis_format,
                side_filter="enemy",
                geo_reference=self.geo_reference,
                enemy_assoc=self.enemy_assoc,
                enemy_assoc_max_distance=self.enemy_assoc_max_distance,
                enemy_assoc_max_altitude=self.enemy_assoc_max_altitude,
                enemy_assoc_keep_sec=self.enemy_assoc_keep_sec,
                enemy_hash_remap_mode=self.enemy_hash_remap_mode,
                enemy_hash_center_x_ratio=self.enemy_hash_center_x_ratio,
                enemy_hash_lateral_scale=self.enemy_hash_lateral_scale,
                enemy_hash_range_scale=self.enemy_hash_range_scale,
                enemy_hash_start_range_m=self.enemy_hash_start_range_m,
                enemy_hash_y_offset_m=self.enemy_hash_y_offset_m,
                enemy_hash_hide_outbound=self.enemy_hash_hide_outbound,
                enemy_flat_remap_mode=self.enemy_flat_remap_mode,
                enemy_flat_rotate_deg=self.enemy_flat_rotate_deg,
                enemy_flat_flip_x=self.enemy_flat_flip_x,
                enemy_flat_flip_y=self.enemy_flat_flip_y,
                enemy_flat_scale=self.enemy_flat_scale,
                enemy_flat_center_x_ratio=self.enemy_flat_center_x_ratio,
                enemy_flat_center_y_ratio=self.enemy_flat_center_y_ratio,
            )
            if self.friendly_return_source == "redis":
                friendly_feed = TeacherDataFeed(
                    host=redis_host,
                    port=redis_port,
                    db=redis_db,
                    password=redis_password,
                    redis_format="auto",
                    side_filter="uav",
                    geo_reference=self.geo_reference,
                    enemy_assoc=self.enemy_assoc,
                    enemy_assoc_max_distance=self.enemy_assoc_max_distance,
                    enemy_assoc_max_altitude=self.enemy_assoc_max_altitude,
                    enemy_assoc_keep_sec=self.enemy_assoc_keep_sec,
                )
            elif self.friendly_return_source == "udp":
                friendly_feed = TeacherUDPFeed(
                    bind_host=udp_in_host,
                    port=udp_in_port,
                    side_filter="friendly",
                    geo_reference=self.geo_reference,
                )
            else:
                friendly_feed = None
            self.feed = FusionTrackFeed(enemy_feed=enemy_feed, friendly_feed=friendly_feed)
        else:
            self.feed = None
        self.scene_revision = 0
        # 配置场景比例尺
        self.configure_scene(scene_km, reset=False)
        self.reset()

    def configure_scene(self, scene_km, reset=True):
        CFG.apply_scene_scale(scene_km)
        self.scene_revision += 1
        if reset and hasattr(self, "logs"):
            self.reset()

    def reset(self):
        """
                重置/初始化整个仿真环境。
                当收到 '重置' 指令，或切换演示方案时被调用。
                """
        self.rng = random.Random(self.seed)
        self.time = 0.0
        self.step_count = 0
        self.done = False
        self.success = False
        self.demo_showcase_active = self.demo_case is None and self.source == "demo"
        desired_count = 30 if self.demo_showcase_active else self.base_interceptor_count
        self._set_interceptor_count(desired_count)
        self.demo_interference_zones = self._build_demo_interference_zones() if self.demo_showcase_active else []
        self.demo_interference_limit = self._demo_interference_capacity() if self.demo_showcase_active and self.demo_interference_enabled else 0
        # 初始化己方拦截机列表，并为每架飞机设置爬升率等初始属性
        self.interceptors = [create_interceptor(i) for i in range(CFG.NUM_INTERCEPTORS)]
        for it in self.interceptors:
            it['climb_rate'] = CFG.INTERCEPTOR_CLIMB_RATE

        # 清空敌机
        self.enemies = []

        self.wave_mgr = WaveManager(self.rng, waves=self._demo_showcase_wave_plan())

        # 实例化目标分配器
        self.assigner = InterceptionAssigner()
        # 清空大模型下发的战术约束
        self._reset_llm_task_constraints()
        self._sync_llm_task_constraints_to_assigner()
        self.stats = {
            'kills': 0,
            'penetrations': 0,
            'our_losses': 0,
            'total_enemies': self.wave_mgr.total,
            'intercept_alts': [],
            'waves_done': 0,
        }
        self.logs = [
            "系统初始化完成 - 智能情报分析已接入",
            f"[SCENE] 当前场景 {CFG.SCENE_KM:.0f}km",
            f"[MODE] 当前拦截模式 {self._mode_label()}",
            f"[HANGAR] 当前机巢模式 {CFG.HANGAR_MODE} | 数量 {len(CFG.HANGAR_POSITIONS)}",
        ]
        for msg in self.analyst.drain_status_events():
            self.logs.append(f"[LLM] {msg}")
        if self.feed:
            if self.source == "udp":
                self.logs.append(f"[DATA] 已配置数据源 udp://{self.udp_in_host}:{self.udp_in_port}")
            elif self.source == "fusion":
                self.logs.append(
                    f"[DATA] 已配置融合数据源 enemy=redis://{self.redis_host}:{self.redis_port}/{self.redis_db} "
                    f"+ friendly={self.friendly_return_source}"
                )
            else:
                self.logs.append(f"[DATA] 已配置数据源 redis://{self.redis_host}:{self.redis_port}/{self.redis_db}")
        else:
            self.logs.append("[DATA] 当前使用本地回放波次")
        if self.demo_showcase_active:
            jam_effect = "开启" if self.demo_interference_enabled else "关闭"
            jam_view = "显示" if self.demo_interference_visible else "隐藏"
            self.logs.append(f"[DEMO] 强干扰演示: 效果{jam_effect} / 圆形干扰范围{jam_view}")
        if getattr(self, "demo_scheme", 0):
            self.logs.append(
                f"[SCHEME] 方案{self.demo_scheme}: {self.demo_scheme_name} | "
                f"{self._demo_scheme_brief(self.demo_scheme)}"
            )
        self._refresh_llm_decision_card(emit_logs=bool(getattr(self, "demo_scheme", 0)))

        self.external_enemy_to_id = {}
        self.uav_id_map = {}
        self.next_enemy_id = 0
        self.has_live_data = False
        self.demo_mode = (self.source == "demo")
        self.live_switch_logged = False
        self.last_diag = ""
        self.last_live_seen_time = 0.0
        self.last_live_packet_meta = {}
        self.last_enemy_presence_time = 0.0
        self.enemy_track_flags = {}
        self.deconflict_cooldown = {}
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}

        # 实例化航线避碰与合规控制器
        self.deconfliction = DeconflictionController(self)
        self.last_assistant_dispatch_time = -999.0
        self.pending_confirmation = None
        self.command_posture = "normal"
        self.llm_interference_no_fly_active = False
        self.llm_interference_replan_time = -1.0
        self.llm_interference_replan_threshold = 2
        self.llm_interference_released_targets = set()
        self.llm_replan_boost_until = -1.0

        if self.demo_case:
            self._setup_demo_case(self.demo_case)

    def _mode_label(self):
        if self.intercept_mode == "hit":
            return "撞击拦截"
        if self.intercept_mode == "legacy-net":
            return "高级网阻(隐藏)"
        if self.intercept_mode == "net":
            return "列阵扯网"
        return "混合拦截"

    def _demo_scheme_preset(self, scheme_id):
        presets = {
            1: {
                'name': '传统最近邻',
                'brief': '最近邻分配 + 第一批战备机单机直追；无全局重分配、无随动备份',
                'strategy': 'baseline',
                'intercept_mode': 'hit',
                'jam': False,
                'jam_visible': False,
                'llm_auto': False,
                'baseline_ready': 15,
            },
            2: {
                'name': '威胁协同',
                'brief': '威胁驱动分配 + 主拦截/随动备份；无强干扰',
                'strategy': 'cooperative',
                'intercept_mode': 'hit',
                'jam': False,
                'jam_visible': False,
                'llm_auto': False,
                'baseline_ready': None,
            },
            3: {
                'name': '强干扰失联',
                'brief': '协同拦截遇到圆形强干扰；无人机受扰后失联悬停，暴露集中指挥边界',
                'strategy': 'cooperative',
                'intercept_mode': 'hit',
                'jam': True,
                'jam_visible': True,
                'llm_auto': False,
                'baseline_ready': None,
            },
        }
        return presets.get(int(scheme_id or 0))

    def _demo_scheme_brief(self, scheme_id):
        preset = self._demo_scheme_preset(scheme_id)
        return preset['brief'] if preset else "当前手动配置"

    def _apply_demo_scheme_settings(self, scheme_id):
        preset = self._demo_scheme_preset(scheme_id)
        if not preset:
            return None
        self.demo_scheme = int(scheme_id)
        self.demo_scheme_name = preset['name']
        self.demo_strategy_mode = preset['strategy']
        self.intercept_mode = preset['intercept_mode']
        self.demo_interference_enabled = bool(preset['jam'])
        self.demo_interference_visible = bool(preset['jam_visible'])
        self.llm_interference_auto_replan_enabled = bool(preset['llm_auto'])
        self.demo_baseline_ready_count = preset.get('baseline_ready')
        return preset

    def _refresh_llm_decision_card(self, emit_logs=False):
        scheme_id = int(getattr(self, "demo_scheme", 0) or 0)
        cards = {
            1: (
                "传统基线对照",
                [
                    "输入: 雷达点",
                    "处理: 最近邻单机追击",
                    "缺口: 无意图理解/无威胁排序/无备份",
                ],
                "txt2",
                "[BASE]",
            ),
            2: (
                "LLM任务意图 -> 协同分配",
                [
                    "意图: 防线前最大化拦截率",
                    "研判: 速度/距离/机动性综合威胁排序",
                    "输出: 主拦截+随动备份+保留机动余量",
                ],
                "pink",
                "[LLM]",
            ),
            3: (
                "LLM态势研判 -> 边界暴露",
                [
                    "意图: 协同拦截保持防线",
                    "研判: 强干扰造成链路中断与失联悬停",
                    "边界: 设备一/LLM无法控制失联无人机",
                ],
                "amber",
                "[LLM]",
            ),
        }
        title, lines, color, prefix = cards.get(scheme_id, (
            "LLM上层决策链",
            [
                "意图: 自然语言/态势输入",
                "输出: 任务约束与分配偏好",
                "执行: 下层控制器闭环飞行",
            ],
            "pink",
            "[LLM]",
        ))
        self.llm_decision_title = title
        self.llm_decision_lines = lines
        self.llm_decision_color = color
        if emit_logs:
            for idx, line in enumerate(lines):
                label = ("意图解析", "态势研判", "决策输出")[min(idx, 2)]
                self.logs.append((prefix, f"{label}: {line}", color))

    def _reset_llm_task_constraints(self):
        self.llm_task_constraints = {
            'reserve_count': 0,
            'target_priority': None,
            'preferred_sector': None,
            'avoid_jam': False,
            'max_active_count': 0,
            'reserve_locked': 0,
            'reserve_shortfall': 0,
        }
        self.llm_reserved_ids = set()
        for it in getattr(self, "interceptors", []):
            it['task_reserved'] = False
        self.llm_reasoning_state = {
            'mode': 'idle',
            'input_text': '',
            'intent': '等待语音/文本指令',
            'facts': '暂无显式约束',
            'situation': '等待战场态势输入',
            'risk': '暂无额外风险约束',
            'plan': '下层分配器按默认策略待命',
            'constraints_text': 'target_priority=default, preferred_sector=all, reserve=0, max_active=all, avoid_jam=false',
            'execution': '尚未下发新任务',
            'decision_object': {
                'target_priority': 'default',
                'preferred_sector': 'all',
                'reserve_count': 0,
                'max_active_count': 0,
                'avoid_jam': False,
            },
            'trace_steps': [
                {'index': '01', 'title': '输入指令', 'detail': '等待语音/文本指令'},
                {'index': '02', 'title': '关键信息抽取', 'detail': '暂无显式约束'},
                {'index': '03', 'title': '战场态势判断', 'detail': '等待战场态势输入'},
                {'index': '04', 'title': '风险约束评估', 'detail': '暂无额外风险约束'},
                {'index': '05', 'title': '任务方案生成', 'detail': '下层分配器按默认策略待命'},
                {'index': '06', 'title': '执行下发', 'detail': '尚未下发新任务'},
            ],
            'updated_at': 0.0,
        }

    def _sync_llm_task_constraints_to_assigner(self):
        if getattr(self, "assigner", None) is not None:
            self.assigner.task_constraints = getattr(self, "llm_task_constraints", {}) or {}

    def _llm_battlefield_summary(self):
        active_enemy_count = sum(
            1 for enemy in getattr(self, "enemies", [])
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING)
        )
        standby_count = sum(
            1 for item in getattr(self, "interceptors", [])
            if item['state'] == IState.STANDBY
        )
        active_friendly_count = sum(
            1 for item in getattr(self, "interceptors", [])
            if item['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
        )
        jammed_count = sum(
            1 for item in getattr(self, "interceptors", [])
            if item.get('jammed_by_interference') or item.get('lost')
        )
        penetrations = int((getattr(self, "stats", {}) or {}).get('penetrations', 0) or 0)
        return {
            'active_enemy_count': active_enemy_count,
            'standby_count': standby_count,
            'active_friendly_count': active_friendly_count,
            'jammed_count': jammed_count,
            'penetrations': penetrations,
        }

    def _record_llm_reasoning(self, input_text, intent, facts, situation, risk, plan, constraints_text, execution,
                              decision_object=None, trace_steps=None, mode="constraint"):
        if trace_steps is None:
            trace_steps = [
                {'index': '01', 'title': '输入指令', 'detail': str(input_text or '等待语音/文本指令')},
                {'index': '02', 'title': '关键信息抽取', 'detail': str(facts or '暂无')},
                {'index': '03', 'title': '战场态势判断', 'detail': str(situation or '暂无')},
                {'index': '04', 'title': '风险约束评估', 'detail': str(risk or '暂无')},
                {'index': '05', 'title': '任务方案生成', 'detail': str(plan or '暂无')},
                {'index': '06', 'title': '执行下发', 'detail': str(execution or '暂无')},
            ]
        self.llm_reasoning_state = {
            'mode': str(mode or 'constraint'),
            'input_text': str(input_text or ''),
            'intent': str(intent or ''),
            'facts': str(facts or ''),
            'situation': str(situation or ''),
            'risk': str(risk or ''),
            'plan': str(plan or ''),
            'constraints_text': str(constraints_text or ''),
            'execution': str(execution or ''),
            'decision_object': decision_object or {},
            'trace_steps': trace_steps,
            'updated_at': float(getattr(self, "time", 0.0) or 0.0),
        }

    def _parse_llm_number(self, token):
        return parse_llm_number(token)

    def _parse_llm_task_constraints_command(self, text):
        return parse_llm_task_constraints_command(text, int(CFG.NUM_INTERCEPTORS))

    def _release_interceptor_task_binding(self, it):
        iid = it.get('id')
        if iid is None:
            return set()
        released_targets = set()
        for enemy_id, asgn in list(self.assigner.assignments.items()):
            for role in ('primary', 'follower'):
                if asgn.get(role) == iid:
                    asgn[role] = None
                    released_targets.add(enemy_id)
            if asgn.get('primary') is None and asgn.get('follower') is None:
                asgn.pop('poi', None)
                asgn.pop('eta', None)
        for mapping in (self.barrier_team_assignments, self.net_team_assignments):
            for enemy_id, team in list(mapping.items()):
                if iid in team:
                    mapping[enemy_id] = [member for member in team if member != iid]
                    released_targets.add(enemy_id)
        return released_targets

    def _park_interceptor_for_llm_reserve(self, it):
        self._release_interceptor_task_binding(it)
        it['state'] = IState.STANDBY
        it['target_id'] = None
        it['role'] = IRole.RESERVE
        it['launch_time'] = -1.0
        it['speed'] = 0.0
        it['target_z'] = 0.0
        it['search_until'] = 0.0
        it['path_plan'] = []
        it['path_reason'] = ""
        it['poi'] = None
        it['search_point'] = None
        it['search_distance'] = 0.0
        it['net_slot'] = None
        it['barrier_slot'] = None
        it['barrier_center'] = None
        it['task_reserved'] = True
        it['mission_label'] = "LLM保留"
        it['target_label'] = "待命"
        self._clear_local_motion_state(it)
        if getattr(self, "deconfliction", None):
            self.deconfliction.reset_local_state(it['id'])

    def _llm_reserve_target_count(self):
        constraints = getattr(self, "llm_task_constraints", {}) or {}
        try:
            count = int(constraints.get('reserve_count') or 0)
        except (TypeError, ValueError):
            count = 0
        return max(0, min(len(getattr(self, "interceptors", [])), count))

    def _update_llm_reserved_pool(self, cancel_pending=False):
        reserve_count = self._llm_reserve_target_count()
        current_ids = set(getattr(self, "llm_reserved_ids", set()) or set())
        for it in self.interceptors:
            if it.get('task_reserved') and it['id'] not in current_ids:
                it['task_reserved'] = False
        if reserve_count <= 0:
            self.llm_reserved_ids = set()
            for it in self.interceptors:
                it['task_reserved'] = False
            self.llm_task_constraints['reserve_locked'] = 0
            self.llm_task_constraints['reserve_shortfall'] = 0
            self._sync_llm_task_constraints_to_assigner()
            return set()

        eligible = [
            it for it in self.interceptors
            if it['state'] in (IState.STANDBY, IState.LANDED)
            and it.get('target_id') is None
            and not it.get('jammed_by_interference')
            and it['state'] != IState.DESTROYED
        ]
        selected = [it for it in eligible if it['id'] in current_ids]
        selected.sort(key=lambda item: item['id'], reverse=True)
        selected_ids = {it['id'] for it in selected[:reserve_count]}

        if len(selected_ids) < reserve_count:
            new_candidates = [it for it in eligible if it['id'] not in selected_ids]
            new_candidates.sort(key=lambda item: item['id'], reverse=True)
            for cand in new_candidates:
                if len(selected_ids) >= reserve_count:
                    break
                selected_ids.add(cand['id'])

        if cancel_pending and len(selected_ids) < reserve_count:
            pending = [
                it for it in self.interceptors
                if it['state'] == IState.LAUNCHING
                and it['id'] not in selected_ids
                and not it.get('jammed_by_interference')
            ]
            pending.sort(key=lambda item: item['id'], reverse=True)
            for cand in pending:
                if len(selected_ids) >= reserve_count:
                    break
                self._park_interceptor_for_llm_reserve(cand)
                selected_ids.add(cand['id'])

        self.llm_reserved_ids = selected_ids
        for it in self.interceptors:
            if it['id'] in selected_ids:
                it['task_reserved'] = True
                if it['state'] in (IState.STANDBY, IState.LANDED):
                    it['mission_label'] = "LLM保留"
                    it['target_label'] = "待命"
            elif it.get('task_reserved'):
                it['task_reserved'] = False
        self.llm_task_constraints['reserve_locked'] = len(selected_ids)
        self.llm_task_constraints['reserve_shortfall'] = max(0, reserve_count - len(selected_ids))
        self._sync_llm_task_constraints_to_assigner()
        return selected_ids

    def _constraint_value_text(self, value):
        if value is True:
            return "true"
        if value is False or value is None:
            return "false" if value is False else "none"
        return str(value)

    def _sector_value_text(self, value):
        mapping = {None: "all", "left": "left", "right": "right", "center": "center"}
        return mapping.get(value, str(value))

    def _sector_label_text(self, value):
        mapping = {"left": "左翼", "right": "右翼", "center": "中路"}
        return mapping.get(value, "全域")

    def _priority_label_text(self, value):
        mapping = {None: "默认威胁排序", "speed": "高速优先", "breach": "突防线附近优先"}
        return mapping.get(value, str(value))

    def _constraint_summary_text(self, constraints):
        constraints = constraints or {}
        priority = constraints.get('target_priority') or "default"
        sector = self._sector_value_text(constraints.get('preferred_sector'))
        reserve = int(constraints.get('reserve_count') or 0)
        max_active = int(constraints.get('max_active_count') or 0)
        max_active_text = str(max_active) if max_active > 0 else "all"
        avoid_jam = self._constraint_value_text(bool(constraints.get('avoid_jam')))
        return (
            f"target_priority={priority}, preferred_sector={sector}, "
            f"reserve={reserve}, max_active={max_active_text}, avoid_jam={avoid_jam}"
        )

    def _max_active_limit(self):
        constraints = getattr(self, "llm_task_constraints", {}) or {}
        try:
            count = int(constraints.get('max_active_count') or 0)
        except (TypeError, ValueError):
            count = 0
        return max(0, min(len(getattr(self, "interceptors", [])), count))

    def _committed_interceptor_count(self):
        return sum(
            1 for it in getattr(self, "interceptors", [])
            if it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
            and not it.get('task_reserved')
            and self._interceptor_can_complete_task(it)
        )

    def _sorted_active_enemies_for_assignment(self):
        active = [
            enemy for enemy in self.enemies
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and enemy.get('detected')
            and not enemy.get('lost', False)
        ]
        return self.assigner.sort_active_enemies(active)

    def _force_task_reassignment(self, reason="LLM口令"):
        if self.command_posture == "guard":
            return {
                'ok': False,
                'message': "当前处于警戒态势，自动接敌已关闭；请先说“解除警戒”，再执行任务重分配。",
                'execution': "警戒态势下未执行任务重分配",
                'facts': "警戒态势会冻结自动新增派机",
                'plan': "解除警戒后再按当前约束重排任务",
            }

        active_enemies = self._sorted_active_enemies_for_assignment()
        if not active_enemies:
            return {
                'ok': False,
                'message': "当前无可重分配的活动目标。",
                'execution': "未发现活动目标，未触发任务重分配",
                'facts': "当前无活动且已探测目标",
                'plan': "保持当前待命状态",
            }

        self.assigner.assignments = {}
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}

        released_airborne = 0
        direct_reassigned = 0
        returned_waiting = 0
        released_targets = set()

        for it in self.interceptors:
            if it['state'] in (IState.DESTROYED, IState.LANDED):
                continue
            if it.get('device3_temporarily_unavailable') and not self._interceptor_can_complete_task(it):
                if bool(
                    it.get('target_id') is not None
                    or it.get('search_point')
                    or it.get('net_slot') is not None
                    or it.get('barrier_slot') is not None
                ):
                    released_targets.update(self._release_interceptor_task_binding(it))
                it['target_id'] = None
                it['role'] = IRole.RESERVE
                it['search_until'] = 0.0
                it['path_plan'] = []
                it['path_reason'] = ""
                it['poi'] = None
                it['search_point'] = None
                it['search_distance'] = 0.0
                it['net_slot'] = None
                it['barrier_slot'] = None
                it['barrier_center'] = None
                it['mission_label'] = "设备一执行失败"
                it['target_label'] = "暂不可用"
                continue

            if it.get('jammed_by_interference') or it.get('task_reserved'):
                continue

            had_binding = bool(
                it.get('target_id') is not None
                or it.get('search_point')
                or it.get('net_slot') is not None
                or it.get('barrier_slot') is not None
                or it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
            )
            if had_binding:
                released_targets.update(self._release_interceptor_task_binding(it))

            it['target_id'] = None
            it['role'] = IRole.RESERVE
            it['search_until'] = 0.0
            it['path_plan'] = []
            it['path_reason'] = ""
            it['poi'] = None
            it['search_point'] = None
            it['search_distance'] = 0.0
            it['net_slot'] = None
            it['barrier_slot'] = None
            it['barrier_center'] = None

            if it['state'] not in (IState.STANDBY, IState.RETURNING):
                released_airborne += 1

            if (
                had_binding
                and getattr(self, "demo_strategy_mode", "cooperative") != "baseline"
                and self.intercept_mode in ("hit", "hybrid")
                and it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
            ):
                new_target_id, new_role = self.assigner._try_hot_reassign(it, active_enemies)
                if new_target_id is not None:
                    enemy = self._get_enemy(new_target_id)
                    self.assigner._bind_assignment(new_target_id, it['id'], new_role)
                    it['state'] = IState.FOLLOWING if new_role == IRole.FOLLOWER else IState.INTERCEPTING
                    it['target_id'] = new_target_id
                    it['role'] = new_role
                    it['mission_label'] = "任务重分配"
                    it['target_label'] = f"F-{new_target_id+1}"
                    poi, eta = self.assigner.compute_poi(it, enemy) if enemy else (None, None)
                    it['poi'] = poi
                    it['poi_time'] = eta
                    if enemy:
                        it['target_z'] = poi[2] if poi else enemy.get('z', CFG.INTERCEPTOR_CRUISE_ALT)
                    asgn = self.assigner.assignments.setdefault(new_target_id, {'primary': None, 'follower': None})
                    asgn['poi'] = poi
                    asgn['eta'] = eta
                    if new_role == IRole.PRIMARY:
                        asgn['launch_anchor'] = min(self.time, it.get('launch_time', self.time))
                    direct_reassigned += 1
                    continue

            if it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                self._set_return(it, "任务重分配，等待新目标")
                returned_waiting += 1

        hit_targets, barrier_targets, net_targets = self._split_targets_by_mode(active_enemies)
        if self.intercept_mode in ("hit", "hybrid"):
            if getattr(self, "demo_strategy_mode", "cooperative") == "baseline":
                for msg in self._update_baseline_assignments(hit_targets):
                    self.logs.append(f"[BASE] {msg}")
            else:
                for msg in self.assigner.update(self.interceptors, hit_targets, self.time):
                    self.logs.append(f"[ASGN] {msg}")
        if self.intercept_mode in ("net", "hybrid"):
            for msg in self._update_barrier_assignments(barrier_targets):
                self.logs.append(f"[BARRIER] {msg}")
        if self.intercept_mode == "legacy-net":
            for msg in self._update_net_assignments(net_targets):
                self.logs.append(f"[NET] {msg}")

        target_names = "/".join(f"F-{enemy['id']+1}" for enemy in active_enemies[:6])
        if len(active_enemies) > 6:
            target_names += "/..."
        target_names = target_names or "无活动目标"
        execution = (
            f"已释放{released_airborne}架在空任务，直接改派{direct_reassigned}架，"
            f"其余由下层分配器按当前约束重排；当前目标队列 {target_names}"
        )
        return {
            'ok': True,
            'message': (
                f"已执行任务重分配：释放{released_airborne}架当前任务，"
                f"直接改派{direct_reassigned}架，其余目标重新进入分配队列。"
            ),
            'execution': execution,
            'facts': (
                f"活动目标{len(active_enemies)}个；释放在空任务{released_airborne}架；"
                f"直接热接力{direct_reassigned}架；返航等待{returned_waiting}架"
            ),
            'plan': "清空旧任务绑定，保留当前上层约束，由下层分配器重新排队并补位",
            'released_targets': sorted(released_targets),
        }

    def _apply_llm_task_constraints_from_command(self, text):
        parsed = self._parse_llm_task_constraints_command(text)
        if not parsed:
            return None
        if parsed.get('clear_all'):
            self._reset_llm_task_constraints()
            self._clear_llm_interference_replan()
            self._sync_llm_task_constraints_to_assigner()
            self.logs.append(("[LLM]", "意图解析: 恢复默认分配策略", "pink"))
            self.logs.append(("[LLM]", "约束生成: target_priority=default, preferred_sector=all, reserve=0, max_active=all, avoid_jam=false", "amber"))
            self.logs.append(("[CMD]", "下层分配器已清除LLM任务约束", "green"))
            summary = self._llm_battlefield_summary()
            self._record_llm_reasoning(
                input_text=text,
                intent="恢复默认分配策略",
                facts="清除保留、目标优先级和干扰绕避等上层约束",
                situation=(
                    f"当前活动目标{summary['active_enemy_count']}个，待命{summary['standby_count']}架，"
                    f"执行中{summary['active_friendly_count']}架"
                ),
                risk="恢复默认调度，不再附加保留/绕避限制",
                plan="回到系统默认的威胁驱动分配与资源调度",
                constraints_text="target_priority=default, preferred_sector=all, reserve=0, max_active=all, avoid_jam=false",
                execution="下层分配器已清除LLM任务约束",
                decision_object={
                    'target_priority': 'default',
                    'preferred_sector': 'all',
                    'reserve_count': 0,
                    'max_active_count': 0,
                    'avoid_jam': False,
                    'command': text,
                },
                mode="reset",
            )
            return "已恢复默认分配：不保留待命机，不强制高速优先，不启用干扰禁入。"

        constraints = getattr(self, "llm_task_constraints", {}) or {}
        constraints.update({
            key: parsed[key]
            for key in ('reserve_count', 'target_priority', 'preferred_sector', 'avoid_jam', 'max_active_count')
            if key in parsed
        })
        self.llm_task_constraints = constraints
        if constraints.get('avoid_jam'):
            self._mark_interference_no_fly()
            self.llm_replan_boost_until = self.time + 999.0
        elif parsed.get('avoid_jam') is False:
            self._clear_llm_interference_replan()
        selected_ids = self._update_llm_reserved_pool(cancel_pending=True)

        reserve_count = int(constraints.get('reserve_count') or 0)
        priority = constraints.get('target_priority') or "none"
        preferred_sector = constraints.get('preferred_sector')
        avoid_jam = bool(constraints.get('avoid_jam'))
        max_active_count = int(constraints.get('max_active_count') or 0)
        intent_parts = []
        if 'target_priority' in parsed:
            if priority == "speed":
                intent_parts.append("优先高速目标")
            elif priority == "breach":
                intent_parts.append("优先突防线附近目标")
            elif priority == "none":
                intent_parts.append("恢复默认目标优先级")
        if 'preferred_sector' in parsed:
            if preferred_sector in ("left", "right", "center"):
                intent_parts.append(f"优先{self._sector_label_text(preferred_sector)}目标")
            else:
                intent_parts.append("取消区域优先")
        if 'reserve_count' in parsed:
            intent_parts.append(f"保留{reserve_count}架待命")
        if 'avoid_jam' in parsed:
            intent_parts.append("绕避干扰区" if avoid_jam else "取消干扰禁入")
        if 'max_active_count' in parsed:
            if max_active_count > 0:
                intent_parts.append(f"最多出动{max_active_count}架")
            else:
                intent_parts.append("取消出动上限")
        if parsed.get('force_reassign'):
            intent_parts.append("任务重分配")
        intent = "，".join(intent_parts) or "更新任务约束"

        self.logs.append(("[LLM]", f"意图解析: {intent}", "pink"))
        self.logs.append((
            "[LLM]",
            f"约束生成: {self._constraint_summary_text(constraints)}",
            "amber",
        ))
        locked = len(selected_ids)
        shortfall = max(0, reserve_count - locked)
        if shortfall:
            exec_text = f"下层分配器按该约束重新分配任务；已锁定{locked}架待命机，仍缺{shortfall}架返回机巢后补足"
        else:
            exec_text = f"下层分配器按该约束重新分配任务；已锁定{locked}架待命机不可调用"
        reassign_result = None
        if parsed.get('force_reassign'):
            reassign_result = self._force_task_reassignment("LLM口令")
            exec_text = reassign_result.get('execution', exec_text)
        self.logs.append(("[CMD]", exec_text, "green"))
        summary = self._llm_battlefield_summary()
        jam_zone_names = "/".join(
            zone.get('label') or zone.get('name') or "干扰"
            for zone in getattr(self, "demo_interference_zones", []) or []
        ) or "无干扰区"
        fact_bits = [
            f"解析到目标优先级={priority}",
            f"解析到区域优先={self._sector_value_text(preferred_sector)}",
            f"解析到保留机={reserve_count}架",
            f"解析到最多出动={max_active_count if max_active_count > 0 else 'all'}",
            f"解析到绕避干扰={self._constraint_value_text(avoid_jam)}",
        ]
        risk_text = (
            f"存在 {jam_zone_names}，继续硬闯会带来链路中断风险"
            if avoid_jam else
            "当前不附加干扰绕避约束，按默认路径规划执行"
        )
        if max_active_count > 0:
            risk_text += f"；同时受兵力上限约束，最多仅允许{max_active_count}架无人机出动"
        plan_text = (
            f"保留{locked}架机巢待命，其余无人机按{self._priority_label_text(constraints.get('target_priority'))}"
            f"{'、'+self._sector_label_text(preferred_sector)+'优先' if preferred_sector in ('left', 'right', 'center') else ''}"
            f"重新分配；"
            f"{f'最大出动{max_active_count}架；' if max_active_count > 0 else ''}"
            f"{'等待返航补足保留缺口' if shortfall else '保留池已满足'}"
        )
        if reassign_result:
            risk_text = reassign_result.get('facts', risk_text) if not reassign_result.get('ok', True) else risk_text
            plan_text = reassign_result.get('plan', plan_text)
            fact_bits.append(reassign_result.get('facts', "已触发任务重分配"))
        self._record_llm_reasoning(
            input_text=text,
            intent=intent,
            facts="；".join(fact_bits),
            situation=(
                f"当前活动目标{summary['active_enemy_count']}个，待命{summary['standby_count']}架，"
                f"执行中{summary['active_friendly_count']}架，受扰/失联{summary['jammed_count']}架，"
                f"突防{summary['penetrations']}个"
            ),
            risk=risk_text,
            plan=plan_text,
            constraints_text=self._constraint_summary_text(constraints),
            execution=exec_text,
            decision_object={
                'command': text,
                'target_priority': priority,
                'preferred_sector': self._sector_value_text(preferred_sector),
                'reserve_count': reserve_count,
                'max_active_count': max_active_count,
                'avoid_jam': avoid_jam,
                'force_reassign': bool(parsed.get('force_reassign')),
                'reserve_locked': locked,
                'reserve_shortfall': shortfall,
                'active_enemy_count': summary['active_enemy_count'],
                'jammed_count': summary['jammed_count'],
                'jam_zones': jam_zone_names,
            },
            mode="constraint",
        )
        if reassign_result and not reassign_result.get('ok', True):
            return reassign_result.get('message', "任务重分配未执行。")
        action_text = reassign_result.get('message', exec_text).rstrip("。") if reassign_result else exec_text
        return (
            f"LLM意图解析：{intent}。已生成约束 "
            f"{self._constraint_summary_text(constraints)}；"
            f"{action_text}。"
        )

    def apply_demo_scheme(self, scheme_id, reason="UI按钮"):
        preset = self._apply_demo_scheme_settings(scheme_id)
        if not preset:
            self.logs.append(f"[SCHEME] 未识别的方案: {scheme_id}")
            return False
        self.reset()
        self.logs.append(f"[SCHEME] {reason}: 已切换方案{self.demo_scheme} - {self.demo_scheme_name}")
        self.logs.append(f"[SCHEME] {preset['brief']}")
        self._refresh_llm_decision_card(emit_logs=False)
        return True

    def _free_assignment_pool(self):
        return sum(
            1
            for it in self.interceptors
            if it['state'] in (IState.STANDBY, IState.RETURNING)
            and it['target_id'] is None
            and not it.get('jammed_by_interference')
            and not it.get('task_reserved')
        )

    def _launch_delay_for_interceptor(self, it, queue_offset=0.0):
        hangar_count = max(1, len(CFG.HANGAR_POSITIONS))
        hangar_idx = it.get('hangar_idx', it['id'] % hangar_count)
        stack_idx = it['id'] // hangar_count
        return 0.18 * float(queue_offset) + 0.08 * hangar_idx + 0.16 * stack_idx

    def _legacy_demo_behavior_enabled(self):
        return self.demo_mode and not self.has_live_data

    def _set_interceptor_count(self, count):
        count = max(1, int(count))
        if CFG.NUM_INTERCEPTORS == count:
            return False
        CFG.NUM_INTERCEPTORS = count
        CFG.apply_scene_scale(CFG.SCENE_KM)
        self.scene_revision += 1
        return True

    def _demo_showcase_wave_plan(self):
        if not self.demo_showcase_active:
            return None
        return [{'time': 0.0, 'count': 26}]

    def _build_demo_interference_zones(self):
        return [
            {
                'name': 'JAM-A',
                'label': '干扰A',
                'cx': CFG.AREA_WIDTH * 0.23,
                'cy': CFG.INTERCEPT_FAIL_LINE * 0.67,
                'radius': CFG.INTERCEPT_FAIL_LINE * 0.10,
            },
            {
                'name': 'JAM-B',
                'label': '干扰B',
                'cx': CFG.AREA_WIDTH * 0.68,
                'cy': CFG.INTERCEPT_FAIL_LINE * 0.52,
                'radius': CFG.INTERCEPT_FAIL_LINE * 0.10,
            },
        ]

    def _demo_interference_capacity(self):
        return max(1, int(CFG.NUM_INTERCEPTORS), len(getattr(self, "interceptors", [])))

    def _rebuild_interceptor_force(self):
        self.interceptors = [create_interceptor(i) for i in range(CFG.NUM_INTERCEPTORS)]
        for it in self.interceptors:
            it['climb_rate'] = CFG.INTERCEPTOR_CLIMB_RATE
        if hasattr(self, "deconfliction"):
            self.deconfliction = DeconflictionController(self)

    def add_friendly_target(self, reason="UI按钮"):
        next_id = max((it['id'] for it in self.interceptors), default=-1) + 1
        CFG.NUM_INTERCEPTORS = max(int(CFG.NUM_INTERCEPTORS), next_id + 1)
        it = create_interceptor(next_id)
        it['climb_rate'] = CFG.INTERCEPTOR_CLIMB_RATE
        it['mission_label'] = "新增待命"
        it['target_label'] = "-"
        self.interceptors.append(it)
        if self.demo_showcase_active and self.demo_interference_enabled:
            self.demo_interference_limit = self._demo_interference_capacity()
        reset_local_state = getattr(getattr(self, "deconfliction", None), "reset_local_state", None)
        if reset_local_state:
            reset_local_state(it['id'])
        self.logs.append(f"[UI] {reason}: 新增我方 I-{it['id']+1}，已加入待命序列")
        self._update_llm_reserved_pool()
        return it

    def add_enemy_target(self, reason="UI按钮"):
        existing_next = max((enemy['id'] for enemy in self.enemies), default=-1) + 1
        wave_next = int(getattr(getattr(self, "wave_mgr", None), "nxt_id", 0) or 0)
        live_next = int(getattr(self, "next_enemy_id", 0) or 0)
        next_id = max(existing_next, wave_next, live_next)
        x = self.rng.uniform(CFG.AREA_WIDTH * 0.12, CFG.AREA_WIDTH * 0.88)
        speed = CFG.ENEMY_SPEED + self.rng.uniform(-CFG.ENEMY_SPEED_VAR, CFG.ENEMY_SPEED_VAR)
        heading = CFG.ENEMY_HDG_BASE + self.rng.uniform(-CFG.ENEMY_HDG_VAR, CFG.ENEMY_HDG_VAR)
        enemy = create_enemy(next_id, x, speed, heading, self.time, self.rng.random())
        enemy['y'] = CFG.ENEMY_SPAWN_LINE + 50.0
        enemy['detected'] = True
        enemy['detect_time'] = self.time
        enemy['source'] = 'ui'
        enemy['last_update'] = self.time
        self.enemies.append(enemy)
        if getattr(self, "wave_mgr", None) is not None:
            self.wave_mgr.nxt_id = max(self.wave_mgr.nxt_id, next_id + 1)
        self.next_enemy_id = max(int(getattr(self, "next_enemy_id", 0) or 0), next_id + 1)
        self.stats['total_enemies'] = max(int(self.stats.get('total_enemies', 0)) + 1, len(self.enemies))
        if self.done:
            self.done = False
            self.success = False
        self.logs.append(
            f"[UI] {reason}: 新增敌方 F-{enemy['id']+1} | "
            f"({enemy['x']:.0f},{enemy['y']:.0f}) | 已进入探测态势"
        )
        return enemy

    def set_demo_interference(self, enabled, reason="UI按钮"):
        enabled = bool(enabled)
        self.demo_interference_enabled = enabled
        self.demo_interference_visible = enabled
        if getattr(self, "demo_scheme", 0):
            self.demo_scheme = 0
            self.demo_scheme_name = "手动配置"
            self._refresh_llm_decision_card(emit_logs=False)
        if self.demo_showcase_active and enabled and not self.demo_interference_zones:
            self.demo_interference_zones = self._build_demo_interference_zones()
        self.demo_interference_limit = self._demo_interference_capacity() if self.demo_showcase_active and enabled else 0
        if not enabled:
            for it in self.interceptors:
                if it.get('jammed_by_interference'):
                    self._clear_interference_state(it)
            self._clear_llm_interference_replan()
        state = "开启" if enabled else "关闭"
        self.logs.append(f"[UI] {reason}: 强干扰{state}，效果与圆形范围同步{state}")
        return enabled

    def toggle_demo_interference(self, reason="UI按钮"):
        return self.set_demo_interference(not self.demo_interference_enabled, reason=reason)

    def _activate_demo_showcase_from_auto_fallback(self):
        if self.demo_case is not None:
            return
        self.demo_showcase_active = True
        self._set_interceptor_count(30)
        self.demo_interference_zones = self._build_demo_interference_zones()
        self.demo_interference_limit = self._demo_interference_capacity() if self.demo_interference_enabled else 0
        self._rebuild_interceptor_force()
        self.enemies = []
        self.wave_mgr = WaveManager(self.rng, waves=self._demo_showcase_wave_plan())
        self.assigner = InterceptionAssigner()
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}
        self.stats.update({
            'kills': 0,
            'penetrations': 0,
            'our_losses': 0,
            'total_enemies': self.wave_mgr.total,
            'intercept_alts': [],
            'waves_done': 0,
        })
        self.logs.append("[DEMO] 已切换为强干扰演示场景")

    def _restore_base_interceptor_layout_for_live(self):
        if not self.demo_showcase_active:
            return
        self.demo_showcase_active = False
        self.demo_interference_zones = []
        self.demo_interference_limit = 0
        self._set_interceptor_count(self.base_interceptor_count)
        self._rebuild_interceptor_force()

    def _demo_interference_active(self):
        return (
            self.demo_mode
            and not self.has_live_data
            and self.demo_interference_enabled
            and bool(self.demo_interference_zones)
        )

    def _interference_zone_for(self, it):
        x, y = it.get('x', 0.0), it.get('y', 0.0)
        for zone in self.demo_interference_zones:
            if (
                zone.get('llm_no_fly')
                and getattr(self, "llm_interference_no_fly_active", False)
                and not it.get('jammed_by_interference')
            ):
                continue
            if 'cx' in zone and 'cy' in zone and 'radius' in zone:
                dx = x - zone['cx']
                dy = y - zone['cy']
                if dx * dx + dy * dy <= zone['radius'] * zone['radius']:
                    return zone
                continue
            if zone['x1'] <= x <= zone['x2'] and zone['y1'] <= y <= zone['y2']:
                return zone
        return None

    def _clear_interference_state(self, it):
        it['jammed_by_interference'] = False
        it['jam_zone'] = None
        it['jam_since'] = -1.0
        it['jam_loss_logged'] = False
        if it.get('local_hold_reason') in ("强干扰悬停", "航向紊乱"):
            self._clear_local_motion_state(it)

    def _interference_link_loss_delay(self):
        return 6.0

    def _interference_phase(self, it):
        if not it.get('jammed_by_interference'):
            return None
        jam_since = float(it.get('jam_since', -1.0))
        if jam_since < 0.0:
            return "degraded"
        return "lost" if self.time - jam_since >= self._interference_link_loss_delay() else "degraded"

    def _apply_interference_confusion_status(self, it):
        it['path_reason'] = "强干扰区内链路抖动，任务指令解析异常"
        it['local_avoid_mode'] = "链路干扰"
        it['local_hold_reason'] = "航向紊乱"
        it['mission_label'] = "链路干扰"
        it['target_label'] = it.get('jam_zone') or "强干扰区"

    def _apply_interference_confused_motion(self, it, dt):
        self._apply_interference_confusion_status(it)
        elapsed = max(0.0, self.time - float(it.get('jam_since', self.time)))
        phase_seed = (it['id'] + 1) * 1.731
        sway = math.sin(self.time * 5.2 + phase_seed) * CFG.INTERCEPTOR_MAX_ANG * 0.95
        kick = math.sin(self.time * 13.1 + phase_seed * 2.0) * CFG.INTERCEPTOR_MAX_ANG * 0.38
        drift = math.sin(elapsed * 1.15 + phase_seed) * CFG.INTERCEPTOR_MAX_ANG * 0.45
        it['heading'] = (it.get('heading', 270.0) + (sway + kick + drift) * dt) % 360

        base_speed = max(0.0, it.get('speed', 0.0))
        if base_speed > 0.05:
            factor = 0.52 + 0.24 * math.sin(self.time * 3.3 + phase_seed)
            if math.sin(self.time * 1.7 + phase_seed * 0.7) < -0.70:
                factor *= 0.25
            it['speed'] = max(3.0, min(base_speed * factor, CFG.INTERCEPTOR_SPEED * 0.9))

        alt_jitter = 5.0 * math.sin(self.time * 2.1 + phase_seed)
        it['target_z'] = self._cap_interceptor_altitude(
            it,
            it.get('target_z', it.get('z', 0.0)) + alt_jitter * dt,
        )
        if it.get('speed', 0.0) > 0.05:
            move_entity(it, dt * 0.28)
            it['flight_time'] += dt * 0.28

    def _apply_interference_hover(self, it):
        it['speed'] = 0.0
        it['vz'] = 0.0
        it['target_z'] = self._cap_interceptor_altitude(it, it.get('z', 0.0))
        it['path_plan'] = []
        it['path_reason'] = "强干扰区内失联悬停"
        it['local_avoid_mode'] = "通信受阻"
        it['local_hold_reason'] = "强干扰悬停"
        it['mission_label'] = "通信受阻"
        it['target_label'] = it.get('jam_zone') or "强干扰区"

    def _interceptor_can_complete_task(self, it):
        if not it or it.get('jammed_by_interference'):
            return False
        if it.get('device3_temporarily_unavailable'):
            failed_until = it.get('device3_failed_until')
            try:
                if failed_until is not None and self.time >= float(failed_until):
                    it['device3_temporarily_unavailable'] = False
                    it['device3_failure_reason'] = ""
                    return True
            except (TypeError, ValueError):
                pass
            return False
        return True

    def _llm_lost_interceptors(self):
        return [
            it for it in self.interceptors
            if it.get('jammed_by_interference')
            and self._interference_phase(it) == "lost"
            and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED)
        ]

    def _llm_jammed_interceptors(self):
        return [
            it for it in self.interceptors
            if it.get('jammed_by_interference')
            and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED)
        ]

    def _clear_llm_interference_replan(self):
        self.llm_interference_no_fly_active = False
        self.llm_interference_replan_time = -1.0
        self.llm_interference_released_targets = set()
        for zone in self.demo_interference_zones:
            zone.pop('llm_no_fly', None)

    def _mark_interference_no_fly(self):
        for zone in self.demo_interference_zones:
            zone['llm_no_fly'] = True
        self.llm_interference_no_fly_active = bool(self.demo_interference_zones)
        self.llm_interference_replan_time = self.time

    def _release_jammed_task_bindings(self):
        blocked = {it['id'] for it in self._llm_jammed_interceptors()}
        if not blocked:
            return set(), 0

        released_targets = set()
        released_slots = 0
        for enemy_id, asgn in list(self.assigner.assignments.items()):
            for role in ('primary', 'follower'):
                iid = asgn.get(role)
                if iid not in blocked:
                    continue
                asgn[role] = None
                released_targets.add(enemy_id)
                released_slots += 1
                it = self._get_interceptor(iid)
                if it:
                    it['target_id'] = None
                    it['role'] = IRole.RESERVE
                    it['path_plan'] = []
                    it['poi'] = None
            if asgn.get('primary') is None and asgn.get('follower') is None:
                asgn.pop('poi', None)
                asgn.pop('eta', None)

        for mapping in (self.barrier_team_assignments, self.net_team_assignments):
            for enemy_id, team in list(mapping.items()):
                new_team = [iid for iid in team if iid not in blocked]
                if len(new_team) == len(team):
                    continue
                mapping[enemy_id] = new_team
                released_targets.add(enemy_id)

        for iid in blocked:
            it = self._get_interceptor(iid)
            if not it:
                continue
            it['net_slot'] = None
            it['barrier_slot'] = None
            it['barrier_center'] = None
            it['target_id'] = None
            it['role'] = IRole.RESERVE
            it['path_plan'] = []
            it['poi'] = None

        self.llm_interference_released_targets.update(released_targets)
        return released_targets, released_slots

    def trigger_llm_interference_replan(self, reason="自动触发", force=False):
        if not self.demo_interference_zones:
            self.logs.append(("[LLM]", "未发现可标记的干扰区，暂不执行态势重构", "txtd"))
            return False
        lost = self._llm_lost_interceptors()
        if not force and len(lost) < self.llm_interference_replan_threshold:
            return False
        if self.llm_interference_no_fly_active and not force:
            return False

        self._mark_interference_no_fly()
        released_targets, released_slots = self._release_jammed_task_bindings()

        lost_names = "/".join(f"I-{it['id']+1}" for it in lost) or "暂无彻底失联机"
        zone_names = "/".join(zone.get('label') or zone.get('name') or "干扰" for zone in self.demo_interference_zones)
        target_names = "/".join(f"F-{eid+1}" for eid in sorted(released_targets)) or "现有威胁目标"
        isolated_count = len(self._llm_jammed_interceptors())
        available_count = sum(
            1 for it in self.interceptors
            if it['state'] not in (IState.DESTROYED, IState.LANDED)
            and not it.get('jammed_by_interference')
        )

        self.logs.append((
            "[LLM]",
            f"研判: {lost_names} 在干扰区出现链路中断，判定为区域通信压制",
            "pink",
        ))
        self.logs.append((
            "[LLM]",
            f"决策: 将 {zone_names} 标记为禁入区，失联/受扰无人机退出任务池",
            "amber",
        ))
        self.logs.append((
            "[LLM]",
            f"执行: 隔离{isolated_count}架受扰/失联无人机，释放{released_slots}个任务席位，{target_names} 重新排队，{available_count}架可通信无人机绕行补位",
            "green",
        ))
        self._record_llm_reasoning(
            input_text=f"{reason}: 干扰态势触发任务重构",
            intent="将强干扰区域标记为禁入区，并释放失联无人机占用的任务席位",
            facts=f"失联/受扰无人机={lost_names}；干扰区域={zone_names}；需重排目标={target_names}",
            situation=f"当前受扰/失联{isolated_count}架，可通信{available_count}架，释放席位{released_slots}个",
            risk="继续硬闯干扰区会导致链路中断、任务无法闭环完成",
            plan="把干扰区设为禁入区，失联机退出任务池，未受扰无人机绕飞补位",
            constraints_text=f"llm_no_fly=true, released_slots={released_slots}, available={available_count}",
            execution=f"已释放{released_slots}个任务席位，{target_names} 已重新进入分配队列",
            decision_object={
                'trigger': reason,
                'lost_interceptors': lost_names,
                'jam_zones': zone_names,
                'released_targets': sorted(released_targets),
                'released_slots': released_slots,
                'available_count': available_count,
                'isolated_count': isolated_count,
                'llm_no_fly': True,
            },
            mode="jam_replan",
        )
        self.llm_replan_boost_until = self.time + 45.0
        for it in self.interceptors:
            if it.get('jammed_by_interference'):
                continue
            if it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                it['path_reason'] = ""
        return True

    def _maybe_auto_llm_interference_replan(self):
        if not getattr(self, "llm_interference_auto_replan_enabled", True):
            return False
        if self.llm_interference_no_fly_active:
            return False
        if len(self._llm_lost_interceptors()) < self.llm_interference_replan_threshold:
            return False
        return self.trigger_llm_interference_replan("自动触发", force=False)

    def _update_demo_interference(self):
        if not self._demo_interference_active():
            for it in self.interceptors:
                if it.get('jammed_by_interference'):
                    self._clear_interference_state(it)
            return

        airborne_states = (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
        for it in self.interceptors:
            if it['state'] in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                if it.get('jammed_by_interference'):
                    self._clear_interference_state(it)
                continue

            if it.get('jammed_by_interference'):
                if self._interference_phase(it) == "lost":
                    if not it.get('jam_loss_logged'):
                        it['jam_loss_logged'] = True
                        self.logs.append(
                            f"[JAM] I-{it['id']+1} 在{it.get('jam_zone') or '强干扰区'}内链路彻底中断，转入悬停"
                        )
                    self._apply_interference_hover(it)
                else:
                    self._apply_interference_confusion_status(it)
                continue

            if it['state'] not in airborne_states:
                continue

            zone = self._interference_zone_for(it)
            if not zone:
                continue

            it['jammed_by_interference'] = True
            it['jam_zone'] = zone.get('label') or zone.get('name')
            it['jam_since'] = self.time
            it['jam_loss_logged'] = False
            self.deconfliction.reset_local_state(it['id'])
            self._apply_interference_confusion_status(it)
            self.logs.append(
                f"[JAM] I-{it['id']+1} 进入{it['jam_zone']}，链路强干扰，继续执行但航向开始紊乱"
            )

    def _setup_demo_case(self, case_name):
        if case_name not in ("net-single", "barrier-single"):
            return

        enemy = create_enemy(
            0,
            CFG.AREA_WIDTH * 0.5,
            30.0,
            90.0,
            0.0,
            0.62,
        )
        enemy['type'] = EType.SNAKE
        enemy['x'] = CFG.AREA_WIDTH * 0.5
        enemy['y'] = max(CFG.DETECTION_LINE + 120.0, CFG.AREA_HEIGHT * 0.08)
        enemy['z'] = min(CFG.ENEMY_MAX_ALT, 32.0)
        enemy['target_z'] = enemy['z']
        enemy['z_cap'] = CFG.ENEMY_MAX_ALT
        enemy['detected'] = True
        enemy['detect_time'] = 0.0
        enemy['classification_confidence'] = 0.98
        enemy['external_id'] = "case-barrier-01"
        enemy['source'] = 'demo'

        self.enemies = [enemy]
        self.wave_mgr.waves = []
        self.wave_mgr.idx = 0
        self.wave_mgr.nxt_id = 1
        self.next_enemy_id = 1
        self.stats['total_enemies'] = 1
        self.stats['waves_done'] = 1
        self.logs.append("[CASE] 最小验证集: 单目标四机列阵扯网")
        self.logs.append(
            f"[CASE] F-1 固定为机动目标 | 初始=({enemy['x']:.0f},{enemy['y']:.0f},{enemy['z']:.0f}) | "
            f"速度={enemy['speed']:.0f}m/s"
        )

    def _clear_feed_runtime_cache(self, feed):
        if not feed:
            return
        if hasattr(feed, "last_result"):
            feed.last_result = None
        if hasattr(feed, "last_poll"):
            feed.last_poll = 0.0
        if hasattr(feed, "track_cache"):
            feed.track_cache = {}
        if hasattr(feed, "enemy_hash_projection_cache"):
            feed.enemy_hash_projection_cache = {}
        if hasattr(feed, "last_diag"):
            feed.last_diag = ""
        if hasattr(feed, "last_error"):
            feed.last_error = ""
        associator = getattr(feed, "enemy_associator", None)
        if associator is not None and hasattr(associator, "track_table"):
            associator.track_table = {}
            associator.next_local_index = 1
        if hasattr(feed, "enemy_feed") or hasattr(feed, "friendly_feed"):
            self._clear_feed_runtime_cache(getattr(feed, "enemy_feed", None))
            self._clear_feed_runtime_cache(getattr(feed, "friendly_feed", None))

    def _prepare_fresh_live_start(self):
        if self.source not in ("udp", "redis", "auto", "fusion"):
            return

        self.enemies = []
        self.external_enemy_to_id = {}
        self.next_enemy_id = 0
        self.has_live_data = False
        self.live_switch_logged = False
        self.last_live_seen_time = 0.0
        self.last_live_packet_meta = {}
        self.last_enemy_presence_time = 0.0
        self.last_diag = ""
        self.enemy_track_flags = {}
        self.assigner = InterceptionAssigner()
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}

        self._clear_feed_runtime_cache(self.feed)

        for idx, it in enumerate(self.interceptors):
            if it['state'] in (IState.DESTROYED, IState.LANDED):
                continue
            fresh = create_interceptor(idx)
            for key in (
                'x', 'y', 'z', 'vz', 'heading', 'speed', 'fuel',
                'state', 'target_id', 'role', 'launch_time', 'search_until',
                'path_plan', 'path_reason', 'poi', 'return_fast',
                'net_slot', 'barrier_slot', 'mission_label', 'target_label',
                'search_point',
            ):
                it[key] = fresh.get(key, it.get(key))
            it['climb_rate'] = CFG.INTERCEPTOR_CLIMB_RATE
            it['external_controlled'] = False
            it['external_id'] = None
            it['track_quality'] = 1.0
            it['status_text'] = ""
            it['stale'] = False
            it['lost'] = False
            it['frame'] = None
            it['age'] = 0.0
            it['raw_track'] = {}
            it['reported_x'] = it['x']
            it['reported_y'] = it['y']
            it['reported_z'] = it['z']
            it['reported_speed'] = it['speed']
            it['reported_heading'] = it['heading']
            it['reported_vz'] = it['vz']
            it['reported_frame'] = None
            it['reported_at'] = -1.0
            self.deconfliction.reset_local_state(it['id'])
            self._clear_local_motion_state(it)

    def _select_engagement_mode(self, enemy, planned_barrier_targets=0, planned_hit_targets=0, remaining_targets=0):
        if self.intercept_mode == "hit":
            return "hit"
        if self.intercept_mode == "net":
            return "barrier"
        if self.intercept_mode == "legacy-net":
            return "net"
        if enemy['id'] in self.barrier_team_assignments and self.barrier_team_assignments.get(enemy['id']):
            return "barrier"
        if enemy['id'] in self.net_team_assignments and self.net_team_assignments.get(enemy['id']):
            return "net"
        free_pool = self._free_assignment_pool()
        free_pool = max(0, free_pool - planned_barrier_targets * CFG.BARRIER_GROUP_SIZE)
        active_barrier_targets = sum(1 for team in self.barrier_team_assignments.values() if team) + planned_barrier_targets
        hard_target = False
        if enemy.get('lost') or enemy.get('stale'):
            hard_target = True
        if enemy.get('classification_confidence', 1.0) < CFG.MISCLASSIFY_CONFIDENCE:
            hard_target = True
        if getattr(enemy.get('type'), 'name', None) in CFG.BARRIER_TYPE_NAMES:
            hard_target = True
        if (
            hard_target and
            active_barrier_targets < CFG.MAX_CONCURRENT_BARRIER_TARGETS and
            free_pool >= CFG.BARRIER_GROUP_SIZE + CFG.BARRIER_RESOURCE_BUFFER and
            self._barrier_window_feasible(enemy)
        ):
            return "barrier"
        return "hit"

    def _split_targets_by_mode(self, enemies):
        hit_targets, barrier_targets, net_targets = [], [], []
        planned_barrier_targets = 0
        for enemy in enemies:
            mode = self._select_engagement_mode(enemy, planned_barrier_targets=planned_barrier_targets)
            enemy['engagement_mode'] = mode
            if mode == "barrier":
                barrier_targets.append(enemy)
                planned_barrier_targets += 1
            elif mode == "net":
                net_targets.append(enemy)
            else:
                hit_targets.append(enemy)
        return hit_targets, barrier_targets, net_targets

    def _prune_hit_assignments(self, keep_ids, reason):
        for enemy_id, asgn in list(self.assigner.assignments.items()):
            if enemy_id in keep_ids:
                continue
            self.assigner.assignments.pop(enemy_id, None)
            for role in ('primary', 'follower'):
                iid = asgn.get(role)
                it = self._get_interceptor(iid)
                if it and it['target_id'] == enemy_id and it['state'] not in (IState.DESTROYED, IState.LANDED):
                    self._set_return(it, f"F-{enemy_id+1} {reason}")

    def _baseline_available_interceptors(self):
        ready_count = getattr(self, "demo_baseline_ready_count", None)
        available = [
            it for it in self.interceptors
            if it['state'] in (IState.STANDBY, IState.RETURNING)
            and it.get('target_id') is None
            and self._interceptor_can_complete_task(it)
            and not it.get('task_reserved')
            and (ready_count is None or it['id'] < int(ready_count))
        ]
        max_active = self._max_active_limit()
        if max_active > 0 and self._committed_interceptor_count() >= max_active:
            available = [it for it in available if it['state'] == IState.RETURNING]
        return available

    def _release_baseline_assignment(self, enemy_id, asgn, reason):
        for role in ('primary', 'follower'):
            iid = asgn.get(role)
            it = self._get_interceptor(iid)
            if not it or it.get('target_id') != enemy_id:
                continue
            if it.get('jammed_by_interference'):
                it['target_id'] = None
                it['role'] = IRole.RESERVE
                it['path_plan'] = []
                it['poi'] = None
            elif it['state'] not in (IState.DESTROYED, IState.LANDED):
                self._set_return(it, reason)

    def _update_baseline_assignments(self, targets):
        active = [
            enemy for enemy in targets
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and enemy.get('detected')
            and not enemy.get('lost', False)
        ]
        active.sort(key=lambda e: -e['y'])
        active_ids = {enemy['id'] for enemy in active}
        msgs = []

        for enemy_id, asgn in list(self.assigner.assignments.items()):
            if enemy_id not in active_ids:
                self.assigner.assignments.pop(enemy_id, None)
                self._release_baseline_assignment(enemy_id, asgn, f"F-{enemy_id+1} 已脱离传统方案任务池")
                continue
            follower_id = asgn.get('follower')
            if follower_id is not None:
                follower = self._get_interceptor(follower_id)
                if follower and follower.get('target_id') == enemy_id and follower['state'] not in (IState.DESTROYED, IState.LANDED):
                    self._set_return(follower, f"方案1不使用随动备份，释放 F-{enemy_id+1} 随动机")
                asgn['follower'] = None

        for enemy in active:
            enemy_id = enemy['id']
            asgn = self.assigner.assignments.setdefault(enemy_id, {'primary': None, 'follower': None})
            primary_id = asgn.get('primary')
            primary = self._get_interceptor(primary_id)
            if (
                primary
                and primary.get('target_id') == enemy_id
                and primary['state'] not in (IState.DESTROYED, IState.LANDED)
                and not primary.get('jammed_by_interference')
            ):
                asgn['follower'] = None
                continue

            if primary and primary.get('target_id') == enemy_id:
                self._release_baseline_assignment(enemy_id, asgn, f"F-{enemy_id+1} 传统单机不可用，释放重派")
            asgn['primary'] = None
            asgn['follower'] = None

            available = self._baseline_available_interceptors()
            if not available:
                continue
            chosen = min(available, key=lambda it: dist2d(it, enemy))
            chosen['state'] = IState.LAUNCHING
            chosen['target_id'] = enemy_id
            chosen['role'] = IRole.PRIMARY
            chosen['launch_time'] = self.time + self._launch_delay_for_interceptor(chosen)
            chosen['speed'] = 0.0
            chosen['target_z'] = enemy.get('z', 0.0)
            chosen['poi'] = (enemy['x'], enemy['y'], enemy.get('z', 0.0))
            chosen['poi_time'] = None
            chosen['path_plan'] = []
            chosen['path_reason'] = "方案1最近邻单机直追"
            chosen['mission_label'] = "最近邻拦截"
            chosen['target_label'] = f"F-{enemy_id+1}"
            asgn['primary'] = chosen['id']
            asgn['poi'] = chosen['poi']
            asgn['eta'] = None
            msgs.append(f"方案1最近邻: I-{chosen['id']+1} → F-{enemy_id+1} 单机直追")
        return msgs

    def step(self, dt):
        """设备2决策主循环：一次 tick 内完成输入同步、分配、路径和本地判定。

        这是设备2核心调用回路。外部模块通常不要跳过 step() 直接调用分配器，
        否则会丢掉雷达同步、LLM 约束、起降管制和避碰收尾。
        """
        self.time += dt
        self.step_count += 1

        # 1. 从Redis/UDP拉取最新的真实雷达数据，或者本地生成的假想敌
        self._pump_live_data()
        # 2. 更新被大模型保留的不可用的无人机池
        self._update_llm_reserved_pool()

        # 把当前敌我坐标喂给LLM
        active_enemies = [e for e in self.enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        if active_enemies:
            self.last_enemy_presence_time = self.time

        # 大模型态势感知（将当前敌我位置喂给BattlefieldAnalyst）生成解说日志或者副官建议
        self.analyst.update_situation(self.time, active_enemies, self.interceptors)

        ai_msg = self.analyst.get_analysis_log()
        if ai_msg:
            self.logs.append(("[AI情报]", ai_msg, "eloiter"))
        chat_msg = self.analyst.get_chat_reply()
        if chat_msg:
            self.logs.append(("[副官]", chat_msg, "pink"))
            self._apply_assistant_action(chat_msg)
        for msg in self.analyst.drain_status_events():
            self.logs.append(f"[LLM] {msg}")

        # 4. 判断敌机是否越过探测线被发现
        self._update_enemy_detection()
        # 5. 让外出寻敌的无人机接管刚发现的目标
        self._assign_idle_searchers_to_targets()
        # 6. 检查是否有无人机进入强干扰区域，触发llm重新规划
        self._maybe_auto_llm_interference_replan()

        # 7.任务分配核心（调用marl.cooperation里的算法）
        # 根据intercept_mode决定是执行撞击（hit）还是列阵（barrier）还是网阻（net）
        engageable_enemies = [enemy for enemy in active_enemies if enemy.get('detected')]
        if self.command_posture == "guard":
            assigned_enemy_ids = {
                it.get('target_id')
                for it in self.interceptors
                if it.get('target_id') is not None
                and it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
            }
            engageable_enemies = [enemy for enemy in engageable_enemies if enemy['id'] in assigned_enemy_ids]
        hit_targets, barrier_targets, net_targets = self._split_targets_by_mode(engageable_enemies)
        barrier_ids = {enemy['id'] for enemy in barrier_targets}
        net_ids = {enemy['id'] for enemy in net_targets}
        if barrier_ids:
            self._prune_hit_assignments({enemy['id'] for enemy in hit_targets}, "切换为列阵扯网任务，释放撞击编队")
        elif net_ids:
            self._prune_hit_assignments({enemy['id'] for enemy in hit_targets}, "切换为网阻任务，释放撞击编队")

        if self.intercept_mode in ("hit", "hybrid"):
            if getattr(self, "demo_strategy_mode", "cooperative") == "baseline":
                for msg in self._update_baseline_assignments(hit_targets):
                    self.logs.append(f"[BASE] {msg}")
            else:
                for msg in self.assigner.update(self.interceptors, hit_targets, self.time):
                    self.logs.append(f"[ASGN] {msg}")
        if self.intercept_mode in ("net", "hybrid"):
            for msg in self._update_barrier_assignments(barrier_targets):
                self.logs.append(f"[BARRIER] {msg}")
        if self.intercept_mode == "legacy-net":
            for msg in self._update_net_assignments(net_targets):
                self.logs.append(f"[NET] {msg}")

        # 8. 更新起飞状态（处理起飞排队，出机巢避碰）
        self._update_launch_states()
        # 9. 强干扰逻辑（更新位于干扰区内的无人机状态，使其失控）
        self._update_demo_interference()
        self._maybe_auto_llm_interference_replan()

        # 10. 根据底层分配的航点，结合ProNav算法，更新飞机坐标
        self._move_interceptors(dt)

        # 如果不是真实飞控接入，则在本地模拟敌机运动，并进行物理碰撞判定
        if not self._uses_authoritative_live_tracks():
            self._move_enemies(dt)
            # 碰撞判定：检查是否成功拦截/网捕
            if self.intercept_mode in ("hit", "hybrid"):
                self._check_intercepts()
            if self.intercept_mode in ("net", "hybrid"):
                self._check_barrier_capture()
            if self.intercept_mode == "legacy-net":
                self._check_net_capture()

            # 检查敌机是否越过防线导致突防失败
            self._check_penetration()
            # 扣减续航燃料
            self._consume_fuel(dt)
        # 更新UI显示的文本标签
        self._update_mission_labels()
        # 检查本地demo是否已经跑完所有波次
        if not self._uses_authoritative_live_tracks():
            self._check_done()

    def _pump_live_data(self):
        self._sync_teacher_data()
        if not self.demo_mode:
            self._age_live_tracks()
        else:
            self._spawn_demo_waves()

    def _uses_authoritative_live_tracks(self):
        if self.source not in ("udp", "redis", "auto", "fusion") or not self.has_live_data:
            return False
        return any(it.get('external_controlled') for it in self.interceptors)

    def _uses_authoritative_friendly_tracks(self):
        return self._uses_authoritative_live_tracks()

    def _snapshot_authoritative_friendlies(self):
        if not self._uses_authoritative_friendly_tracks():
            return {}
        snapshots = {}
        for it in self.interceptors:
            if not it.get('external_controlled'):
                continue
            snapshots[it['id']] = {
                'x': it.get('reported_x', it['x']),
                'y': it.get('reported_y', it['y']),
                'z': it.get('reported_z', it.get('z', 0.0)),
                'vz': it.get('reported_vz', it.get('vz', 0.0)),
                'speed': it.get('reported_speed', it.get('speed', 0.0)),
                'heading': it.get('reported_heading', it.get('heading', 270.0)),
            }
        return snapshots

    def _restore_authoritative_friendlies(self, snapshots):
        for iid, snap in snapshots.items():
            it = self._get_interceptor(iid)
            if not it:
                continue
            it['x'] = snap['x']
            it['y'] = snap['y']
            it['z'] = self._cap_interceptor_altitude(it, snap['z'])
            it['vz'] = snap['vz']
            it['speed'] = max(0.0, snap['speed'])
            it['heading'] = snap['heading'] % 360

    def visible_interceptors(self):
        if not self._uses_authoritative_friendly_tracks():
            return self.interceptors
        visible = [it for it in self.interceptors if it.get('external_controlled')]
        return visible or self.interceptors

    def _sync_teacher_data(self):
        """把设备1输入帧同步进设备2环境状态。

        feed.poll() 输出仍是输入层快照；本函数负责切换实时/演示模式、保存
        输入帧元数据，并把敌方/己方航迹分别写入 env.enemies/env.interceptors。
        """
        if not self.feed or self.done:
            return None

        snapshot = self.feed.poll(self.time)
        meta = snapshot.get("meta", {})
        diag = meta.get("diag", "")
        if diag and diag != self.last_diag:
            self.logs.append(f"[DATA] {diag}")
            self.last_diag = diag

        enemies = snapshot.get("enemies", [])
        friendlies = snapshot.get("friendlies", [])
        self.last_live_packet_meta = {
            "device_id": meta.get("device_id"),
            "seq": meta.get("seq"),
            "timestamp": meta.get("timestamp"),
            "frame": meta.get("frame"),
        }
        if enemies or friendlies:
            self.last_live_seen_time = self.time
            if not self.has_live_data:
                self.has_live_data = True
                if self.source == "auto":
                    self._restore_base_interceptor_layout_for_live()
                    self.demo_mode = False
                    self.enemies = []
                    self.external_enemy_to_id = {}
                    self.assigner = InterceptionAssigner()
                    for it in self.interceptors:
                        if it['state'] not in (IState.STANDBY, IState.LANDED, IState.DESTROYED):
                            self._set_return(it, "切换到老师实时数据，清空回放任务")
                if not self.live_switch_logged:
                    self.logs.append("[DATA] 已切换到老师实时数据驱动，随机敌机关闭")
                    self.live_switch_logged = True
            self._upsert_live_enemies(enemies)
            self._sync_live_friendlies(friendlies)
        elif self.source == "auto" and not self.has_live_data and self.time >= 2.0 and not self.demo_mode:
            self.demo_mode = True
            self._activate_demo_showcase_from_auto_fallback()
            self.logs.append("[DATA] 2秒内未收到老师数据，启用本地回放作为演示兜底")

        return snapshot

    def _age_live_tracks(self):
        for enemy in self.enemies:
            if enemy.get('source') != 'teacher':
                continue
            age = self.time - enemy.get('last_update', self.time)
            enemy['age'] = age
            was_stale = enemy.get('stale', False)
            was_lost = enemy.get('lost', False)
            enemy['stale'] = age > CFG.RADAR_STALE_SEC
            enemy['lost'] = age > CFG.RADAR_LOST_SEC

            if enemy['stale'] and not was_stale:
                self.logs.append(f"[DATA] F-{enemy['id']+1} 雷达延迟 {age:.1f}s，按最后航迹续算")
            if enemy['lost'] and not was_lost:
                self.logs.append(f"[LOST] F-{enemy['id']+1} 目标丢失，转入保守搜索/重规划")
            if was_lost and not enemy['lost']:
                self.logs.append(f"[REACQ] F-{enemy['id']+1} 目标重获，恢复闭环拦截")
        self._age_live_friendlies()

    def _age_live_friendlies(self):
        for it in self.interceptors:
            if not it.get('external_controlled'):
                continue
            if it.get('reported_at', -1.0) < 0.0:
                continue
            age = self.time - it.get('reported_at', self.time)
            it['age'] = age
            it['stale'] = age > CFG.RADAR_STALE_SEC
            it['lost'] = age > CFG.RADAR_LOST_SEC

    def _spawn_demo_waves(self):
        new_enemies = self.wave_mgr.update(self.time)
        if not new_enemies:
            return
        self.enemies.extend(new_enemies)
        self.stats['waves_done'] = self.wave_mgr.idx
        self.logs.append(f"[WAVE-{self.wave_mgr.idx}] {len(new_enemies)} 架回放目标进入场景")

    def _upsert_live_enemies(self, tracks):
        """按稳定 external_id 新建或刷新敌方目标。

        新目标会分配设备2内部 enemy['id']；已有目标只更新坐标、速度、
        stale/lost 等状态，保持任务分配引用不失效。
        """
        seen = set()
        for track in tracks:
            ext_id = track['external_id']
            seen.add(ext_id)
            enemy = self._get_enemy_by_external(ext_id)
            if enemy is None:
                enemy = self._create_live_enemy(track)
                self.enemies.append(enemy)
                self.external_enemy_to_id[ext_id] = enemy['id']
                self.logs.append(
                    f"[RADAR] F-{enemy['id']+1} 新建轨迹 | "
                    f"({enemy['x']:.0f},{enemy['y']:.0f},{enemy['z']:.0f}) | "
                    f"置信度 {track.get('classification_confidence', 1.0):.2f}"
                )
            else:
                self._apply_track_to_enemy(enemy, track)

        for enemy in self.enemies:
            if enemy.get('source') != 'teacher':
                continue
            if enemy['external_id'] not in seen:
                enemy['stale'] = True

    def _create_live_enemy(self, track):
        etype = _etype_from_track(track)
        z_value = self._cap_enemy_altitude(track.get('z', 0.0))
        track_age = max(0.0, float(track.get('age', 0.0) or 0.0))
        enemy = {
            'id': self.next_enemy_id,
            'x': track['x'],
            'y': track['y'],
            'z': z_value,
            'vz': track.get('vz', 0.0),
            'target_z': z_value,
            'heading': track.get('heading', 90.0),
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': track.get('yaw', track.get('heading', 90.0)),
            '_att_prev_heading': track.get('heading', 90.0),
            'speed': track.get('speed', CFG.ENEMY_SPEED),
            'state': EState.APPROACHING,
            'spawn_time': self.time,
            'maneuver_timer': 0.0,
            'detected': False,
            'detect_time': -1.0,
            'type': etype,
            'phase': 0.0,
            'target_heading': track.get('heading', 90.0),
            'loiter_center': None,
            'loiter_timer': CFG.LOITER_DURATION,
            'is_diving': False,
            'external_id': track['external_id'],
            'track_quality': track.get('track_quality', 1.0),
            'stale': track.get('stale', False),
            'lost': track.get('lost', False),
            'last_update': self.time - max(0.0, float(track.get('age', 0.0) or 0.0)),
            'classification_confidence': track.get('classification_confidence', 1.0),
            'source': 'teacher',
            'status_text': str(track.get('status', '')),
            'frame': track.get('frame'),
            'age': max(0.0, float(track.get('age', 0.0) or 0.0)),
            'climb_rate': CFG.ENEMY_CLIMB_RATE,
            'z_cap': CFG.ENEMY_MAX_ALT,
            'raw_track': dict(track.get('raw', {})),
        }
        compute_attitude_from_motion(
            enemy,
            dt=0.0,
            roll=track.get('roll'),
            pitch=track.get('pitch'),
            yaw=track.get('yaw'),
        )
        self.next_enemy_id += 1
        return enemy

    def _apply_track_to_enemy(self, enemy, track):
        prev_lost = enemy.get('lost', False)
        track_age = max(0.0, float(track.get('age', 0.0) or 0.0))
        prev_update = float(enemy.get('last_update', self.time - track_age) or 0.0)
        att_dt = max(1e-3, (self.time - track_age) - prev_update)
        enemy['x'] = track['x']
        enemy['y'] = track['y']
        enemy['z'] = self._cap_enemy_altitude(track.get('z', enemy.get('z', 0.0)))
        enemy['vz'] = track.get('vz', 0.0)
        enemy['target_z'] = enemy['z']
        enemy['heading'] = track.get('heading', enemy.get('heading', 90.0))
        enemy['speed'] = max(0.0, track.get('speed', enemy.get('speed', CFG.ENEMY_SPEED)))
        enemy['track_quality'] = track.get('track_quality', enemy.get('track_quality', 1.0))
        enemy['classification_confidence'] = track.get('classification_confidence', enemy.get('classification_confidence', 1.0))
        enemy['stale'] = track.get('stale', False)
        enemy['lost'] = track.get('lost', False)
        enemy['last_update'] = self.time - track_age
        enemy['frame'] = track.get('frame')
        enemy['age'] = track_age
        enemy['status_text'] = str(track.get('status', enemy.get('status_text', '')))
        enemy['type'] = _etype_from_track(track)
        enemy['raw_track'] = dict(track.get('raw', enemy.get('raw_track', {})))
        compute_attitude_from_motion(
            enemy,
            dt=att_dt,
            roll=track.get('roll'),
            pitch=track.get('pitch'),
            yaw=track.get('yaw'),
        )
        if enemy['state'] in (EState.DESTROYED, EState.PENETRATED):
            return
        enemy['state'] = EState.APPROACHING
        if prev_lost and not enemy['lost']:
            self.logs.append(f"[REACQ] F-{enemy['id']+1} 重获，按新雷达点刷新航迹")

    def _sync_live_friendlies(self, tracks):
        """把设备1上报的己方真实位置同步到 interceptor 状态。

        这里通过外部 UAV ID 映射内部 interceptor，并保留 reported_at/raw_track，
        后续路径规划和 PlanFrame 的 uav_snapshot 都读取这些状态。
        """
        for track in tracks:
            idx = self._map_uav_track(track['external_id'])
            it = self.interceptors[idx]
            track_age = max(0.0, float(track.get('age', 0.0) or 0.0))
            prev_reported_at = float(it.get('reported_at', self.time - track_age) or 0.0)
            att_dt = max(1e-3, (self.time - track_age) - prev_reported_at)
            reported_z = self._cap_interceptor_altitude(it, track.get('z', 0.0))
            reported_speed = max(0.0, track.get('speed', it.get('speed', 0.0)))
            reported_heading = track.get('heading', it.get('heading', 270.0))
            reported_vz = track.get('vz', it.get('vz', 0.0))
            reported_roll = track.get('roll')
            reported_pitch = track.get('pitch')
            reported_yaw = track.get('yaw', reported_heading)
            it['external_id'] = track['external_id']
            it['source'] = 'teacher'
            it['external_controlled'] = True
            it['track_quality'] = track.get('track_quality', it.get('track_quality', 1.0))
            it['status_text'] = str(track.get('status', it.get('status_text', '')))
            it['stale'] = track.get('stale', False)
            it['lost'] = track.get('lost', False)
            it['last_update'] = self.time
            it['frame'] = track.get('frame')
            it['age'] = track.get('age', 0.0)
            it['raw_track'] = dict(track.get('raw', {}))
            it['reported_x'] = track['x']
            it['reported_y'] = track['y']
            it['reported_z'] = reported_z
            it['reported_speed'] = reported_speed
            it['reported_heading'] = reported_heading
            it['reported_vz'] = reported_vz
            it['reported_roll'] = reported_roll if reported_roll is not None else it.get('roll', 0.0)
            it['reported_pitch'] = reported_pitch if reported_pitch is not None else it.get('pitch', 0.0)
            it['reported_yaw'] = reported_yaw
            it['reported_frame'] = track.get('frame')
            it['reported_at'] = self.time - track_age
            battery = track.get('battery')
            if battery is None:
                battery = track.get('raw', {}).get('battery', track.get('raw', {}).get('fuel'))
            if battery is not None:
                try:
                    it['fuel'] = max(0.0, float(battery))
                except (TypeError, ValueError):
                    pass

            # 外部己方轨迹作为权威态势输入，规划只在该位置上给出建议，不反向驱动真值。
            it['x'] = track['x']
            it['y'] = track['y']
            it['z'] = reported_z
            it['vz'] = reported_vz
            it['heading'] = reported_heading
            it['speed'] = reported_speed
            compute_attitude_from_motion(
                it,
                dt=att_dt,
                roll=reported_roll,
                pitch=reported_pitch,
                yaw=reported_yaw,
            )
            it['reported_roll'] = it.get('roll', 0.0)
            it['reported_pitch'] = it.get('pitch', 0.0)
            it['reported_yaw'] = it.get('yaw', reported_heading)

    def _map_uav_track(self, external_id):
        if external_id in self.uav_id_map:
            return self.uav_id_map[external_id]
        match = re.search(r"(\d+)$", external_id)
        if match:
            idx = max(0, min(CFG.NUM_INTERCEPTORS - 1, int(match.group(1)) - 1))
            if idx not in self.uav_id_map.values():
                self.uav_id_map[external_id] = idx
                return idx
        free = next((it['id'] for it in self.interceptors if it['id'] not in self.uav_id_map.values()), 0)
        self.uav_id_map[external_id] = free
        return free

    def _update_enemy_detection(self):
        names = {
            EType.NORMAL: "常规",
            EType.SNAKE: "机动",
            EType.JINK: "闪避",
            EType.DASH: "高速突防",
            EType.LOITER: "巡飞弹",
            EType.DECOY: "诱饵/疑似误识别",
        }
        for enemy in self.enemies:
            if enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                continue
            if enemy['detected']:
                continue
            if enemy['y'] >= CFG.DETECTION_LINE:
                enemy['detected'] = True
                enemy['detect_time'] = self.time
                detail = names.get(enemy['type'], "未知")
                engage_mode = self._select_engagement_mode(enemy)
                if enemy.get('classification_confidence', 1.0) < CFG.MISCLASSIFY_CONFIDENCE:
                    detail += " / 识别不稳"
                if engage_mode == "barrier":
                    detail += " / 列阵扯网"
                elif engage_mode == "net":
                    detail += " / 网阻"
                else:
                    detail += " / 撞击"
                self.logs.append(
                    f"[DET] F-{enemy['id']+1} 发现 ({detail}) | "
                    f"位置=({enemy['x']:.0f},{enemy['y']:.0f},{enemy.get('z', 0.0):.0f})"
                )

    def _update_launch_states(self):
        for it in self.interceptors:
            if it['state'] != IState.LAUNCHING:
                continue
            if it['launch_time'] < 0:
                it['launch_time'] = self.time
            it['speed'] = 0.0
            it['target_z'] = self._friendly_altitude(it, mission="launch", phase="initial")
            if self.time - it['launch_time'] < CFG.LAUNCH_DELAY:
                if self._legacy_demo_behavior_enabled():
                    self._clear_local_motion_state(it)
                else:
                    it['local_avoid_mode'] = "等待放行"
                    it['local_hold_reason'] = "起飞延迟"
                continue
            if self._legacy_demo_behavior_enabled():
                self._clear_local_motion_state(it)
                it['speed'] = CFG.INTERCEPTOR_SPEED
                if self.intercept_mode in ("net", "legacy-net") or it.get('net_slot') is not None or it.get('barrier_slot') is not None:
                    it['state'] = IState.INTERCEPTING
                else:
                    it['state'] = IState.FOLLOWING if it['role'] == IRole.FOLLOWER else IState.INTERCEPTING
                target = self._get_enemy(it['target_id'])
                if target:
                    self._update_route_plan(it, target, "起飞后进入设备一规划航线")
                continue
            clear_to_depart, reason = self.deconfliction.can_release_launch(it)
            if not clear_to_depart:
                it['local_avoid_mode'] = reason
                it['local_hold_reason'] = reason
                continue
            self._clear_local_motion_state(it)
            it['speed'] = CFG.INTERCEPTOR_SPEED
            if self.intercept_mode in ("net", "legacy-net") or it.get('net_slot') is not None or it.get('barrier_slot') is not None:
                it['state'] = IState.INTERCEPTING
            else:
                it['state'] = IState.FOLLOWING if it['role'] == IRole.FOLLOWER else IState.INTERCEPTING
            target = self._get_enemy(it['target_id'])
            if target:
                self._update_route_plan(it, target, "起飞后进入设备一规划航线")

    def _apply_deconflict_limits(self, it):
        self.deconfliction.apply_interceptor_limits(it)

    def _clear_local_motion_state(self, it):
        it['local_avoid_mode'] = ""
        it['local_hold_reason'] = ""
        it['local_plan_stamp'] = -1.0

    def _advance_interceptor_motion(self, it, dt, turn_rate, apply_limits=False):
        it['heading'] = (it['heading'] + turn_rate * dt) % 360
        if apply_limits:
            self._apply_deconflict_limits(it)
        move_entity(it, dt)
        it['flight_time'] += dt

    def _plan_local_motion(self, it, desired_point, desired_speed, mission_ctx):
        plan = self.deconfliction.plan_local_motion(it, desired_point, desired_speed, mission_ctx)
        if plan.hold_reason == "等待超时" and mission_ctx.get('kind') in ("hit", "follow", "search"):
            self._set_return(it, "绕网等待超时，释放目标")
            return None

        it['local_avoid_mode'] = plan.avoid_mode
        it['local_hold_reason'] = plan.hold_reason
        it['local_plan_stamp'] = self.time
        it['target_z'] = self._cap_interceptor_altitude(it, plan.target_z)
        it['speed'] = min(desired_speed, plan.speed_cap) if plan.speed_cap is not None else desired_speed
        return plan

    def _execute_local_plan(self, it, dt, plan, enemy=None):
        if enemy is not None and plan.allow_terminal_direct:
            turn_rate = ProNav.command(it, enemy)
        else:
            turn_rate = ProNav.guide_point(it, plan.command_point)
        self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=True)

    def _move_interceptors(self, dt):
        """根据已分配任务推进己方 UAV，并刷新路径规划结果。

        本函数读取 it['target_id']/search/return 等任务状态，生成 desired_point，
        再通过 plan_local_motion() 加上局部避碰和绕飞约束，最终更新位置、
        path_plan 和 path_reason；station/contracts.py 后续会采样这些路径。
        """
        snapshots = self._snapshot_authoritative_friendlies()
        jammed_snapshots = {}
        for it in self.interceptors:
            jam_phase = self._interference_phase(it)
            if jam_phase == "lost" and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                jammed_snapshots[it['id']] = {
                    'x': it['x'],
                    'y': it['y'],
                    'z': it.get('z', 0.0),
                    'heading': it.get('heading', 270.0),
                    'target_z': it.get('target_z', it.get('z', 0.0)),
                }
                self._apply_interference_hover(it)
                move_entity(it, dt)
                it['flight_time'] += dt
                continue
            elif it['state'] in (IState.INTERCEPTING, IState.FOLLOWING) and it.get('barrier_slot') is not None:
                self._guide_barrier_member(it, dt)
            elif it['state'] in (IState.INTERCEPTING, IState.FOLLOWING) and it.get('net_slot') is not None:
                self._guide_net_member(it, dt)
            elif it['state'] == IState.INTERCEPTING:
                self._guide_primary(it, dt)
            elif it['state'] == IState.FOLLOWING:
                self._guide_follower(it, dt)
            elif it['state'] == IState.RETURNING:
                self._guide_returning(it, dt)
            elif it['state'] == IState.LAUNCHING:
                if not self._legacy_demo_behavior_enabled():
                    self._apply_deconflict_limits(it)
                move_entity(it, dt)
                it['flight_time'] += dt
            if jam_phase == "degraded" and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                self._apply_interference_confused_motion(it, dt)
        self.deconfliction.deconflict_interceptors()
        for it in self.interceptors:
            snap = jammed_snapshots.get(it['id'])
            if not snap or not it.get('jammed_by_interference'):
                continue
            it['x'] = snap['x']
            it['y'] = snap['y']
            it['z'] = snap['z']
            it['heading'] = snap['heading']
            it['target_z'] = snap['target_z']
            self._apply_interference_hover(it)
        self._restore_authoritative_friendlies(snapshots)

    def _bind_search_interceptor_to_enemy(self, it):
        for enemy in self._dispatch_target_candidates():
            asgn = self.assigner.assignments.setdefault(enemy['id'], {'primary': None, 'follower': None})
            if asgn.get('primary') is None:
                role, role_key = IRole.PRIMARY, 'primary'
            elif asgn.get('follower') is None:
                role, role_key = IRole.FOLLOWER, 'follower'
            else:
                continue

            if not enemy.get('detected', False):
                enemy['detected'] = True
                enemy['detect_time'] = self.time

            it['target_id'] = enemy['id']
            it['role'] = role
            it['search_point'] = None
            it['poi'] = None
            it['path_plan'] = []
            it['path_reason'] = ""
            it['target_z'] = enemy.get('z', CFG.INTERCEPTOR_CRUISE_ALT)
            asgn[role_key] = it['id']
            if role_key == 'primary':
                asgn['launch_anchor'] = min(self.time, it.get('launch_time', self.time))
            poi, eta = self.assigner.compute_poi(it, enemy)
            asgn['poi'] = poi
            asgn['eta'] = eta
            role_text = "主拦截" if role == IRole.PRIMARY else "随动增援"
            self.logs.append(f"[SEARCH] I-{it['id']+1} 自主寻敌接入 F-{enemy['id']+1} | {role_text}")
            return True
        return False

    def _assign_idle_searchers_to_targets(self):
        for it in self.interceptors:
            if it['state'] not in (IState.LAUNCHING, IState.INTERCEPTING):
                continue
            if it.get('target_id') is not None or not it.get('search_point'):
                continue
            self._bind_search_interceptor_to_enemy(it)

    def _guide_idle_search(self, it, dt):
        if self._bind_search_interceptor_to_enemy(it):
            return

        point = it.get('search_point')
        if not point:
            self._set_return(it, "目标已不存在，返航")
            return

        target = {
            'x': point['x'],
            'y': point['y'],
            'z': self._cap_interceptor_altitude(it, point.get('z', CFG.INTERCEPTOR_CRUISE_ALT)),
        }
        it['target_z'] = target['z']
        it['poi'] = (target['x'], target['y'], target['z'])
        search_distance = float(it.get('search_distance') or 500.0)
        if dist2d(it, target) <= 2.0 or it['y'] <= target['y']:
            it['speed'] = 0.0
            it['y'] = target['y']
            it['path_plan'] = []
            it['path_reason'] = f"前出{search_distance:.0f}米等待目标"
            self._clear_local_motion_state(it)
            self._set_return(it, f"前出{search_distance:.0f}米未发现目标，返航")
            return

        self._update_route_plan(it, target, f"无目标前出{search_distance:.0f}米自主寻敌")
        desired_point = self._command_point(it, target)
        desired_speed = min(CFG.INTERCEPTOR_SPEED, 12.0)
        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            it['speed'] = desired_speed
            turn_rate = ProNav.guide_point(it, desired_point)
            self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
            return

        local_plan = self._plan_local_motion(
            it,
            desired_point,
            desired_speed,
            {
                'kind': 'search',
                'phase': 'idle',
                'threat_y': target['y'],
            },
        )
        if local_plan is None:
            return
        self._execute_local_plan(it, dt, local_plan)

    def _guide_primary(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            if it.get('search_point'):
                self._guide_idle_search(it, dt)
                return
            self._set_return(it, "目标已不存在，返航")
            return

        if getattr(self, "demo_strategy_mode", "cooperative") == "baseline":
            self._guide_baseline_primary(it, enemy, dt)
            return

        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            if enemy.get('lost'):
                if it['search_until'] <= 0:
                    it['search_until'] = self.time + CFG.TARGET_SEARCH_SEC
                    self.logs.append(f"[REPLAN] I-{it['id']+1} 对 F-{enemy['id']+1} 丢失目标，沿最后POI搜索 {CFG.TARGET_SEARCH_SEC:.0f}s")
                if self.time > it['search_until']:
                    self._set_return(it, f"F-{enemy['id']+1} 超时未重获，返航")
                    return
                ghost = self._ghost_target(enemy)
                ghost['z'] = self._friendly_altitude(it, ghost, mission="search", phase="search")
                it['speed'] = self._desired_interceptor_speed(ghost, mission="hit", phase="terminal")
                self._update_route_plan(it, ghost, "目标失联，沿预测点搜索")
                turn_rate = ProNav.guide_point(it, self._command_point(it, ghost))
            else:
                it['search_until'] = 0.0
                if dist2d(it, enemy) <= CFG.TERMINAL_GUIDE_RANGE:
                    plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="terminal")}
                    self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                    self._command_point(it, plan_target)
                    it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                    turn_rate = ProNav.command(it, enemy)
                else:
                    plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="cruise")}
                    self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                    turn_rate = ProNav.guide_point(it, self._command_point(it, plan_target))
                    it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="cruise")
            self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
            return

        direct_terminal = False
        if enemy.get('lost'):
            if it['search_until'] <= 0:
                it['search_until'] = self.time + CFG.TARGET_SEARCH_SEC
                self.logs.append(f"[REPLAN] I-{it['id']+1} 对 F-{enemy['id']+1} 丢失目标，沿最后POI搜索 {CFG.TARGET_SEARCH_SEC:.0f}s")
            if self.time > it['search_until']:
                self._set_return(it, f"F-{enemy['id']+1} 超时未重获，返航")
                return
            ghost = self._ghost_target(enemy)
            ghost['z'] = self._friendly_altitude(it, ghost, mission="search", phase="search")
            desired_speed = self._desired_interceptor_speed(ghost, mission="hit", phase="terminal")
            self._update_route_plan(it, ghost, "目标失联，沿预测点搜索")
            desired_point = self._command_point(it, ghost)
            local_plan = self._plan_local_motion(
                it,
                desired_point,
                desired_speed,
                {
                    'kind': 'search',
                    'phase': 'terminal',
                    'threat_y': ghost['y'],
                },
            )
        else:
            it['search_until'] = 0.0
            if dist2d(it, enemy) <= CFG.TERMINAL_GUIDE_RANGE:
                plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="terminal")}
                self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                desired_point = self._command_point(it, plan_target)
                desired_speed = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                direct_terminal = True
                local_plan = self._plan_local_motion(
                    it,
                    desired_point,
                    desired_speed,
                    {
                        'kind': 'hit',
                        'phase': 'terminal',
                        'threat_y': enemy['y'],
                        'allow_terminal_direct': True,
                    },
                )
            else:
                plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="cruise")}
                self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                desired_point = self._command_point(it, plan_target)
                desired_speed = self._desired_interceptor_speed(enemy, mission="hit", phase="cruise")
                local_plan = self._plan_local_motion(
                    it,
                    desired_point,
                    desired_speed,
                    {
                        'kind': 'hit',
                        'phase': 'cruise',
                        'threat_y': enemy['y'],
                    },
                )

        if local_plan is None:
            return
        self._execute_local_plan(it, dt, local_plan, enemy if direct_terminal else None)

    def _guide_follower(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            self._set_return(it, "随动目标消失，返航")
            return

        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            asgn = self.assigner.get_info(enemy['id'])
            guided = False
            if asgn and asgn.get('primary') is not None:
                primary = next((item for item in self.interceptors if item['id'] == asgn['primary']), None)
                if primary and primary['state'] in (IState.INTERCEPTING, IState.FOLLOWING):
                    pr = math.radians(primary['heading'])
                    fx = primary['x'] - math.cos(pr) * CFG.REDUNDANCY_OFFSET
                    fy = primary['y'] - math.sin(pr) * CFG.REDUNDANCY_OFFSET
                    pz = self._friendly_altitude(it, enemy, mission="hit", phase="terminal")
                    it['target_z'] = pz
                    it['poi'] = (fx, fy, pz)
                    self._update_route_plan(it, {'x': fx, 'y': fy, 'z': pz}, "随动保持备份拦截位置")
                    it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                    turn_rate = ProNav.guide_point(it, (fx, fy, pz))
                    self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
                    guided = True
            if not guided:
                self._guide_primary(it, dt)
            return

        asgn = self.assigner.get_info(enemy['id'])
        guided = False
        if asgn and asgn.get('primary') is not None:
            primary = next((item for item in self.interceptors if item['id'] == asgn['primary']), None)
            if primary and primary['state'] in (IState.INTERCEPTING, IState.FOLLOWING):
                pr = math.radians(primary['heading'])
                fx = primary['x'] - math.cos(pr) * CFG.REDUNDANCY_OFFSET
                fy = primary['y'] - math.sin(pr) * CFG.REDUNDANCY_OFFSET
                pz = self._friendly_altitude(it, enemy, mission="hit", phase="terminal")
                it['target_z'] = pz
                it['poi'] = (fx, fy, pz)
                self._update_route_plan(it, {'x': fx, 'y': fy, 'z': pz}, "随动保持备份拦截位置")
                desired_speed = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                desired_point = self._command_point(it, {'x': fx, 'y': fy, 'z': pz})
                local_plan = self._plan_local_motion(
                    it,
                    desired_point,
                    desired_speed,
                    {
                        'kind': 'follow',
                        'phase': 'terminal',
                        'threat_y': enemy['y'],
                    },
                )
                if local_plan is None:
                    return
                self._execute_local_plan(it, dt, local_plan)
                guided = True
        if not guided:
            self._guide_primary(it, dt)
            return

    def _route_lane_offset(self, it):
        hangar_count = max(1, len(CFG.HANGAR_POSITIONS))
        hangar_center = (hangar_count - 1) * 0.5
        hangar_bias = (it.get('hangar_idx', 0) - hangar_center) * CFG.FORMATION_SPACING * 0.55
        stack_idx = it['id'] // hangar_count
        stack_bias = (((stack_idx % 5) - 2.0) * CFG.FORMATION_SPACING * 0.12) if stack_idx else 0.0
        role_bias = -CFG.FORMATION_SPACING * 0.18 if it.get('role') == IRole.FOLLOWER else 0.0
        return hangar_bias + stack_bias + role_bias

    def _hangar_gate_point(self, it, outbound=True, z_value=None):
        h_idx = max(0, min(len(CFG.HANGAR_POSITIONS) - 1, it.get('hangar_idx', 0)))
        lane = self._route_lane_offset(it)
        gate_x = max(0.0, min(CFG.AREA_WIDTH, CFG.HANGAR_POSITIONS[h_idx] + lane * (0.55 if outbound else 0.35)))
        gate_y = (
            CFG.INTERCEPT_FAIL_LINE - max(140.0, CFG.FORMATION_SPACING * 0.75)
            if outbound
            else CFG.INTERCEPT_FAIL_LINE + max(70.0, CFG.FORMATION_SPACING * 0.3)
        )
        gate_z = self._cap_interceptor_altitude(it, z_value if z_value is not None else it.get('z', 0.0))
        return (gate_x, gate_y, gate_z)

    def _guide_returning(self, it, dt):
        h_idx = it.get('hangar_idx', 0)
        target_x = CFG.HANGAR_POSITIONS[h_idx]
        target_y = CFG.INTERCEPT_FAIL_LINE + 200
        target_z = self._friendly_altitude(it, mission="return", phase="return")
        if it.get('return_fast'):
            it['speed'] = min(CFG.INTERCEPTOR_BOOST_SPEED, max(CFG.INTERCEPTOR_SPEED * 1.4, it.get('speed', 0.0)))
        else:
            it['speed'] = max(CFG.INTERCEPTOR_SPEED, it.get('speed', 0.0))
        it['target_z'] = target_z
        it['poi'] = (target_x, target_y, target_z)
        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            return_plan, _ = self.deconfliction.apply_barrier_detours(
                it,
                [(it['x'], it['y'], it.get('z', 0.0)), (target_x, target_y, target_z)],
            )
            cmd_pt = (target_x, target_y, target_z)
            for point in return_plan[1:]:
                if dist2d(it, {'x': point[0], 'y': point[1]}) > max(35.0, CFG.FORMATION_SPACING * 0.25):
                    cmd_pt = point
                    break
            turn_rate = ProNav.guide_point(it, cmd_pt)
            self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
        else:
            inbound_gate = self._hangar_gate_point(it, outbound=False, z_value=target_z)
            return_plan, _ = self.deconfliction.apply_barrier_detours(
                it,
                [(it['x'], it['y'], it.get('z', 0.0)), inbound_gate, (target_x, target_y, target_z)],
            )
            cmd_pt = (target_x, target_y, target_z)
            for point in return_plan[1:]:
                if dist2d(it, {'x': point[0], 'y': point[1]}) > max(35.0, CFG.FORMATION_SPACING * 0.25):
                    cmd_pt = point
                    break
            desired_speed = min(CFG.INTERCEPTOR_BOOST_SPEED, max(CFG.INTERCEPTOR_SPEED * (1.4 if it.get('return_fast') else 1.0), it.get('speed', 0.0)))
            local_plan = self._plan_local_motion(
                it,
                cmd_pt,
                desired_speed,
                {
                    'kind': 'return',
                    'phase': 'corridor',
                },
            )
            if local_plan is None:
                return
            self._execute_local_plan(it, dt, local_plan)

        if dist2d(it, {'x': target_x, 'y': target_y}) < 100 and abs(it.get('z', 0.0) - target_z) < 20:
            it['state'] = IState.STANDBY
            it['speed'] = 0.0
            it['x'] = target_x
            it['y'] = target_y
            it['z'] = target_z
            it['target_id'] = None
            it['role'] = IRole.RESERVE
            it['fuel'] = CFG.INTERCEPTOR_ENDURANCE
            it['path_plan'] = []
            it['path_reason'] = ""
            it['poi'] = None
            it['return_fast'] = False
            self.deconflict_cooldown.pop(it['id'], None)
            self.deconfliction.reset_local_state(it['id'])
            self._clear_local_motion_state(it)

    def _guide_baseline_primary(self, it, enemy, dt):
        it['search_until'] = 0.0
        target_z = self._friendly_altitude(it, enemy, mission="hit", phase="cruise")
        current_point = (enemy['x'], enemy['y'], target_z)
        it['target_z'] = target_z
        it['poi'] = current_point
        it['path_plan'] = [(it['x'], it['y'], it.get('z', 0.0)), current_point]
        it['path_reason'] = "方案1传统基线: 最近邻直追当前点"
        it['speed'] = min(CFG.INTERCEPTOR_SPEED * 0.86, max(14.0, enemy.get('speed', CFG.ENEMY_SPEED) + 1.0))
        turn_rate = ProNav.guide_point(it, current_point)
        self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)

    def _move_enemies(self, dt):
        for enemy in self.enemies:
            if enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                continue
            if enemy.get('source') == 'teacher':
                move_entity(enemy, dt)
                continue
            self._move_demo_enemy(enemy, dt)

    def _move_demo_enemy(self, enemy, dt):
        desired_heading = enemy['heading']
        if enemy['type'] == EType.LOITER:
            if not enemy['is_diving']:
                if enemy['y'] > CFG.INTERCEPT_FAIL_LINE * 0.5 and enemy['loiter_center'] is None:
                    offset_dir = 1 if self.rng.random() > 0.5 else -1
                    enemy['loiter_center'] = (enemy['x'] + offset_dir * CFG.LOITER_RADIUS, enemy['y'])
                if enemy['loiter_center']:
                    cx, cy = enemy['loiter_center']
                    desired_heading = math.degrees(math.atan2(cy - enemy['y'], cx - enemy['x'])) + 90.0
                    enemy['loiter_timer'] -= dt
                    if enemy['loiter_timer'] <= 0:
                        enemy['is_diving'] = True
                        enemy['speed'] = CFG.ENEMY_SPEED * 1.8
                else:
                    desired_heading = 90.0
            else:
                desired_heading = 90.0
        elif enemy['type'] == EType.SNAKE:
            elapsed = self.time - enemy['spawn_time']
            desired_heading = 90.0 + 45.0 * math.sin(0.8 * elapsed + enemy['phase'])
        elif enemy['type'] == EType.JINK:
            enemy['maneuver_timer'] -= dt
            if enemy['maneuver_timer'] <= 0:
                enemy['target_heading'] = 90.0 + self.rng.uniform(-60.0, 60.0)
                enemy['maneuver_timer'] = self.rng.uniform(1.5, 3.0)
            desired_heading = enemy['target_heading']
        else:
            desired_heading = 90.0

        diff = angle_diff(desired_heading, enemy['heading'])
        turn_rate = CFG.ENEMY_MANEUVER_RATE * dt
        change = max(-turn_rate, min(turn_rate, diff))
        enemy['heading'] = (enemy['heading'] + change) % 360
        if enemy['x'] < 200:
            enemy['heading'] = 0
        elif enemy['x'] > CFG.AREA_WIDTH - 200:
            enemy['heading'] = 180
        move_entity(enemy, dt)

    def _check_intercepts(self):
        for it in self.interceptors:
            if it['state'] not in (IState.INTERCEPTING, IState.FOLLOWING):
                continue
            if not self._interceptor_can_complete_task(it):
                continue
            if it.get('net_slot') is not None or it.get('barrier_slot') is not None:
                continue
            for enemy in self.enemies:
                if enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                    continue
                d3 = dist3d(it, enemy)
                d2 = dist2d(it, enemy)
                dz = abs(it.get('z', 0.0) - enemy.get('z', 0.0))
                if d3 >= CFG.INTERCEPT_RADIUS and not (d2 <= CFG.HIT_2D_RADIUS and dz <= CFG.HIT_ALT_TOLERANCE):
                    continue

                dt_kill = self.time - it['launch_time'] if it['launch_time'] >= 0 else 0.0
                if enemy['type'] == EType.DECOY:
                    log_entry = ("[诱饵清除]", f"消耗弹药击毁疑似诱饵 (耗时:{dt_kill:.1f}s)", "amber")
                else:
                    log_entry = (
                        f"[耗时:{dt_kill:.1f}s]",
                        f"I-{it['id']+1} 击毁 F-{enemy['id']+1} "
                        f"(XYZ=({enemy['x']:.0f},{enemy['y']:.0f},{enemy.get('z', 0.0):.0f}))",
                        "red",
                    )
                enemy['state'] = EState.DESTROYED
                it['state'] = IState.DESTROYED
                self._clear_local_motion_state(it)
                self.deconfliction.reset_local_state(it['id'])
                self.stats['kills'] += 1
                self.stats['our_losses'] += 1
                self.stats['intercept_alts'].append(enemy.get('z', 0.0))
                self.logs.append(log_entry)

                asgn = self.assigner.get_info(enemy['id'])
                if asgn:
                    for role in ('primary', 'follower'):
                        pid = asgn.get(role)
                        if pid is None or pid == it['id']:
                            continue
                        partner = next((item for item in self.interceptors if item['id'] == pid), None)
                        if partner and partner['state'] not in (IState.DESTROYED, IState.LANDED, IState.STANDBY):
                            self._set_return(partner, f"F-{enemy['id']+1} 已被击毁，释放目标")
                break

    def _check_penetration(self):
        for enemy in self.enemies:
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING) and enemy['y'] >= CFG.INTERCEPT_FAIL_LINE:
                enemy['state'] = EState.PENETRATED
                self.stats['penetrations'] += 1
                self.logs.append(f"[FAIL] F-{enemy['id']+1} 突防成功")

    def _consume_fuel(self, dt):
        for it in self.interceptors:
            if it['state'] not in (IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING, IState.LAUNCHING):
                continue
            it['fuel'] -= dt
            if it['fuel'] <= 30 and it['state'] != IState.RETURNING:
                self._set_return(it, "燃料不足，返航")

    def _check_done(self):
        if self.demo_mode:
            all_resolved = all(e['state'] in (EState.DESTROYED, EState.PENETRATED) for e in self.enemies) if self.enemies else False
            all_spawned = self.wave_mgr.idx >= len(self.wave_mgr.waves)
            if all_resolved and all_spawned:
                self.done = True
                self.success = self.stats['penetrations'] == 0
                self.logs.append("任务成功！" if self.success else f"任务结束-{self.stats['penetrations']}架突防")
                return
        if math.isfinite(CFG.TIME_LIMIT) and self.time >= CFG.TIME_LIMIT:
            self.done = True
            self.success = False
            self.logs.append("时间耗尽")

    def _get_enemy(self, eid):
        if eid is None:
            return None
        return next((enemy for enemy in self.enemies if enemy['id'] == eid), None)

    def _get_interceptor(self, iid):
        if iid is None:
            return None
        return next((it for it in self.interceptors if it['id'] == iid), None)

    def _get_enemy_by_external(self, external_id):
        enemy_id = self.external_enemy_to_id.get(external_id)
        if enemy_id is None:
            return None
        return self._get_enemy(enemy_id)

    def _ghost_target(self, enemy):
        age = max(0.0, self.time - enemy.get('last_update', self.time))
        er = math.radians(enemy.get('heading', 90.0))
        ghost = {
            'x': max(0.0, min(CFG.AREA_WIDTH, enemy['x'] + math.cos(er) * enemy.get('speed', 0.0) * age)),
            'y': max(0.0, min(CFG.AREA_HEIGHT, enemy['y'] + math.sin(er) * enemy.get('speed', 0.0) * age)),
            'z': max(0.0, enemy.get('z', 0.0) + enemy.get('vz', 0.0) * age),
            'heading': enemy.get('heading', 90.0),
            'speed': enemy.get('speed', 0.0),
            'type': enemy.get('type', EType.NORMAL),
            'state': enemy.get('state', EState.APPROACHING),
        }
        return ghost

    def _cap_interceptor_altitude(self, it, z_value):
        return max(0.0, min(it.get('z_cap', CFG.INTERCEPTOR_MAX_ALT), float(z_value)))

    def _cap_enemy_altitude(self, z_value):
        return max(0.0, min(CFG.ENEMY_MAX_ALT, float(z_value)))

    def _friendly_altitude(self, it, enemy=None, mission="hit", phase="cruise"):
        role_offset = 0.0
        if it.get('role') == IRole.FOLLOWER:
            role_offset = -CFG.INTERCEPTOR_ALT_LAYER_STEP

        if mission == "launch":
            layer = (it['id'] % 4) * 1.2
            return self._cap_interceptor_altitude(it, CFG.INTERCEPTOR_LAUNCH_ALT + layer)

        if mission == "return":
            home_d = dist2d(it, {'x': CFG.HANGAR_POSITIONS[it.get('hangar_idx', 0)], 'y': CFG.INTERCEPT_FAIL_LINE + 200})
            base_alt = CFG.INTERCEPTOR_RETURN_ALT if home_d > 500.0 else max(0.0, home_d * 0.02)
            return self._cap_interceptor_altitude(it, base_alt)

        if mission == "net":
            slot = it.get('net_slot', 0) % 4
            base_layers = (
                CFG.INTERCEPTOR_CRUISE_ALT - 6.0,
                CFG.INTERCEPTOR_CRUISE_ALT - 1.5,
                CFG.INTERCEPTOR_CRUISE_ALT + 3.0,
                CFG.INTERCEPTOR_TERMINAL_ALT + 2.0,
            )
            alt = base_layers[slot]
            if phase == "close":
                alt += 2.0
            return self._cap_interceptor_altitude(it, alt)

        if mission == "barrier":
            slot = it.get('barrier_slot', 0) % 4
            alt = CFG.BARRIER_ALT_BASE + slot * CFG.BARRIER_ALT_STEP
            return self._cap_interceptor_altitude(it, alt)

        if mission == "search":
            return self._cap_interceptor_altitude(it, CFG.INTERCEPTOR_CRUISE_ALT + 2.0 + role_offset)

        alt = CFG.INTERCEPTOR_CRUISE_ALT + role_offset
        if phase == "terminal":
            alt = CFG.INTERCEPTOR_TERMINAL_ALT + role_offset
        elif phase == "cruise" and enemy is not None:
            enemy_d = dist2d(it, enemy)
            if enemy_d > CFG.TERMINAL_GUIDE_RANGE * 2.5:
                alt = CFG.INTERCEPTOR_CRUISE_ALT - 4.0 + role_offset
        if enemy is not None and enemy.get('type') in (EType.LOITER, EType.DASH):
            alt += 3.0
        return self._cap_interceptor_altitude(it, alt)

    def _desired_interceptor_speed(self, enemy, mission="hit", phase="cruise"):
        base = CFG.INTERCEPTOR_SPEED
        if not enemy:
            return base

        target_speed = enemy.get('speed', 0.0)
        fast_target = (
            enemy.get('type') in (EType.DECOY, EType.DASH) or
            target_speed >= base * 0.95 or
            (enemy.get('type') == EType.LOITER and enemy.get('is_diving'))
        )

        if mission == "net" and phase == "close":
            return min(CFG.INTERCEPTOR_NET_SPEED, max(base * 1.15, target_speed + CFG.INTERCEPTOR_FAST_MARGIN))
        if mission == "barrier":
            return CFG.INTERCEPTOR_BARRIER_SPEED
        if (
            mission == "hit"
            and getattr(self, "llm_interference_no_fly_active", False)
            and self.time <= getattr(self, "llm_replan_boost_until", -1.0)
        ):
            return min(CFG.INTERCEPTOR_BOOST_SPEED, max(base * 1.25, target_speed + CFG.INTERCEPTOR_FAST_MARGIN))
        if fast_target:
            return min(CFG.INTERCEPTOR_BOOST_SPEED, max(base, target_speed + CFG.INTERCEPTOR_FAST_MARGIN))
        return base

    def _lateral_shift(self, start_pt, end_pt, offset):
        dx = end_pt[0] - start_pt[0]
        dy = end_pt[1] - start_pt[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return end_pt
        px = -dy / norm
        py = dx / norm
        return (end_pt[0] + px * offset, end_pt[1] + py * offset, end_pt[2])

    def _command_point(self, it, target):
        for point in (it.get('path_plan') or [])[1:]:
            if dist2d(it, {'x': point[0], 'y': point[1]}) > max(35.0, CFG.FORMATION_SPACING * 0.25):
                it['target_z'] = point[2]
                return point
        poi = it.get('poi') or (target['x'], target['y'], target.get('z', 0.0))
        it['target_z'] = self._cap_interceptor_altitude(it, poi[2])
        return poi

    def _active_llm_no_fly_zones(self):
        if not self.llm_interference_no_fly_active:
            return []
        return [
            zone for zone in self.demo_interference_zones
            if zone.get('llm_no_fly') and all(key in zone for key in ('cx', 'cy', 'radius'))
        ]

    def _segment_hits_no_fly_zone(self, start, end, zone):
        margin = max(70.0, CFG.FORMATION_SPACING * 0.8)
        dist, seg_t = self._point_to_segment_distance(
            zone['cx'], zone['cy'],
            start[0], start[1],
            end[0], end[1],
        )
        if dist <= zone['radius'] + margin:
            return True, seg_t
        return False, seg_t

    def _no_fly_detour_point(self, it, start, end, zone, seg_t):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return None
        px = -dy / norm
        py = dx / norm
        clearance = zone['radius'] + max(110.0, CFG.FORMATION_SPACING * 1.15)
        base_x = zone['cx']
        base_y = zone['cy']
        side_order = (1.0, -1.0) if it['id'] % 2 == 0 else (-1.0, 1.0)
        candidates = []
        for side in side_order:
            cx = max(40.0, min(CFG.AREA_WIDTH - 40.0, base_x + px * clearance * side))
            cy = max(40.0, min(CFG.OUR_BASE_LINE - 40.0, base_y + py * clearance * side))
            candidates.append((cx, cy))
        candidates.sort(
            key=lambda pt: (
                0 if 40.0 <= pt[0] <= CFG.AREA_WIDTH - 40.0 and 40.0 <= pt[1] <= CFG.OUR_BASE_LINE - 40.0 else 1,
                math.hypot(pt[0] - start[0], pt[1] - start[1]) + math.hypot(end[0] - pt[0], end[1] - pt[1]),
            )
        )
        x, y = candidates[0]
        z = self._cap_interceptor_altitude(it, max(start[2], end[2], it.get('target_z', end[2])))
        return (x, y, z)

    def _apply_llm_no_fly_detours(self, it, plan):
        zones = self._active_llm_no_fly_zones()
        if not zones or len(plan) < 2:
            return plan, False
        detoured = False
        revised = list(plan)
        guard = 0
        while guard < 4:
            guard += 1
            inserted = False
            for idx in range(len(revised) - 1):
                start, end = revised[idx], revised[idx + 1]
                for zone in zones:
                    hits, seg_t = self._segment_hits_no_fly_zone(start, end, zone)
                    if not hits:
                        continue
                    detour = self._no_fly_detour_point(it, start, end, zone, seg_t)
                    if not detour:
                        continue
                    if dist2d({'x': detour[0], 'y': detour[1]}, {'x': start[0], 'y': start[1]}) <= 20.0:
                        continue
                    revised.insert(idx + 1, detour)
                    detoured = True
                    inserted = True
                    break
                if inserted:
                    break
            if not inserted:
                break
        return revised, detoured

    def _update_route_plan(self, it, target, reason):
        poi = it.get('poi')
        if not poi:
            poi = (target['x'], target['y'], target.get('z', 0.0))
            it['poi'] = poi
        poi = (poi[0], poi[1], self._cap_interceptor_altitude(it, poi[2]))
        it['poi'] = poi
        lane = self._route_lane_offset(it)
        start = (it['x'], it['y'], it.get('z', 0.0))
        anchor = start
        plan = [start]
        if it['y'] >= CFG.INTERCEPT_FAIL_LINE - max(40.0, CFG.FORMATION_SPACING * 0.2):
            anchor = self._hangar_gate_point(it, outbound=True, z_value=max(start[2], poi[2], it.get('target_z', start[2])))
            if dist2d(it, {'x': anchor[0], 'y': anchor[1]}) > max(30.0, CFG.FORMATION_SPACING * 0.15):
                plan.append(anchor)
        mid = (
            (anchor[0] + poi[0]) * 0.5,
            max(min((anchor[1] + poi[1]) * 0.5, CFG.INTERCEPT_FAIL_LINE + 150), target['y']),
            self._cap_interceptor_altitude(it, max(anchor[2], poi[2]) * 0.5),
        )
        mid = self._lateral_shift(anchor, mid, lane)
        plan.extend([mid, poi])
        plan, detoured = self.deconfliction.apply_barrier_detours(it, plan, ignore_enemy_id=it.get('target_id'))
        plan, no_fly_detoured = self._apply_llm_no_fly_detours(it, plan)
        prev_reason = it.get('path_reason', '')
        log_reason = reason + (" | 绕避列阵网带" if detoured else "")
        if no_fly_detoured:
            log_reason += " | LLM绕避干扰禁入区"
        quiet_route_reasons = ("闭环追踪当前雷达点", "随动保持备份拦截位置")
        should_log = prev_reason != log_reason and not any(token in reason for token in quiet_route_reasons)
        if should_log:
            self.logs.append(
                f"[PATH] I-{it['id']+1} {reason} | "
                f"W1=({plan[1][0]:.0f},{plan[1][1]:.0f},{plan[1][2]:.0f}) -> "
                f"W2=({poi[0]:.0f},{poi[1]:.0f},{poi[2]:.0f})"
            )
        it['path_plan'] = plan
        it['path_reason'] = log_reason
        it['target_z'] = poi[2]

    def _ground_station_point(self):
        center_x = sum(CFG.HANGAR_POSITIONS) / max(1, len(CFG.HANGAR_POSITIONS))
        return (center_x, CFG.OUR_BASE_LINE, 0.0)

    def _barrier_half_span(self):
        return max(0.0, CFG.BARRIER_SLOT_SPACING * max(0, CFG.BARRIER_GROUP_SIZE - 1) * 0.5)

    def _barrier_half_depth(self):
        return CFG.BARRIER_NET_RADIUS + CFG.BARRIER_CAPTURE_MARGIN + 30.0

    # Hidden backup: keep the old "four UAVs pull one shared net" geometry in code,
    # but do not expose it as the public barrier mode used in demos.
    def _legacy_barrier_slot_point(self, enemy, slot_idx, center=None):
        center_x, center_y, _ = center or self._barrier_intercept_center(enemy)
        slots = [
            (-CFG.BARRIER_HALF_WIDTH, -CFG.BARRIER_HALF_DEPTH),
            (-CFG.BARRIER_HALF_WIDTH, CFG.BARRIER_HALF_DEPTH),
            (CFG.BARRIER_HALF_WIDTH, CFG.BARRIER_HALF_DEPTH),
            (CFG.BARRIER_HALF_WIDTH, -CFG.BARRIER_HALF_DEPTH),
        ]
        dx, dy = slots[slot_idx % len(slots)]
        return (center_x + dx, center_y + dy)

    def _point_to_segment_distance(self, px, py, ax, ay, bx, by):
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-6:
            return math.hypot(px - ax, py - ay), 0.0
        t = ((px - ax) * abx + (py - ay) * aby) / ab2
        t = max(0.0, min(1.0, t))
        qx = ax + abx * t
        qy = ay + aby * t
        return math.hypot(px - qx, py - qy), t

    def _barrier_state(self, enemy_id):
        return self.barrier_states.setdefault(
            enemy_id,
            {
                'center': None,
                'ready_logged': False,
                'entry_logged': False,
                'center_stamp': -1.0,
                'reposition_log_time': -999.0,
            },
        )

    def _desired_barrier_center(self, enemy):
        station_x, station_y, _ = self._ground_station_point()
        barrier_y = station_y - CFG.BARRIER_STATION_OFFSET
        half_depth = self._barrier_half_depth()
        hangar_pressure = 4.0 + 0.3 * max(1, len(CFG.HANGAR_POSITIONS))
        base_clearance = half_depth + CFG.BARRIER_BUFFER_MARGIN + CFG.BASE_OUTBOUND_CORRIDOR_WIDTH * hangar_pressure
        barrier_y = max(
            CFG.DETECTION_LINE + half_depth,
            min(CFG.INTERCEPT_FAIL_LINE - max(half_depth + 40.0, base_clearance), barrier_y),
        )
        heading = math.radians(enemy.get('heading', 90.0))
        vx = enemy.get('speed', 0.0) * math.cos(heading)
        vy = enemy.get('speed', 0.0) * math.sin(heading)
        t_to_barrier = 0.0
        if abs(vy) > 1e-6:
            t_to_barrier = max(0.0, (barrier_y - enemy['y']) / vy)
        center_x = enemy['x'] + vx * t_to_barrier
        x_margin = self._barrier_half_span() + CFG.BARRIER_NET_RADIUS + 60.0
        center_x = max(x_margin, min(CFG.AREA_WIDTH - x_margin, center_x))
        return (
            center_x,
            barrier_y,
            self._cap_interceptor_altitude({'z_cap': CFG.INTERCEPTOR_MAX_ALT}, CFG.BARRIER_ALT_BASE + CFG.BARRIER_ALT_STEP * 1.5),
        )

    def _barrier_intercept_center(self, enemy):
        state = self._barrier_state(enemy['id'])
        desired = self._desired_barrier_center(enemy)
        center = state.get('center')
        if center is None:
            state['center'] = desired
            state['center_stamp'] = self.time
            return desired

        dx = desired[0] - center[0]
        dy = desired[1] - center[1]
        if abs(dx) <= CFG.BARRIER_REPOSITION_TRIGGER_X and abs(dy) <= CFG.BARRIER_REPOSITION_TRIGGER_Y:
            return center

        if state.get('center_stamp') != self.time:
            step = max(0.1, CFG.BARRIER_REPOSITION_RATE * CFG.DT)
            dist = math.hypot(dx, dy)
            if dist <= step:
                new_x, new_y = desired[0], desired[1]
            else:
                ratio = step / dist
                new_x = center[0] + dx * ratio
                new_y = center[1] + dy * ratio
            x_margin = self._barrier_half_span() + CFG.BARRIER_NET_RADIUS + 60.0
            half_depth = self._barrier_half_depth()
            state['center'] = (
                max(x_margin, min(CFG.AREA_WIDTH - x_margin, new_x)),
                max(CFG.DETECTION_LINE + half_depth, min(CFG.INTERCEPT_FAIL_LINE - half_depth - 40.0, new_y)),
                desired[2],
            )
            state['center_stamp'] = self.time
            if dist >= CFG.BARRIER_REPOSITION_TRIGGER_X and self.time - state.get('reposition_log_time', -999.0) >= 8.0:
                self.logs.append(f"[BARRIER] F-{enemy['id']+1} 偏离预测航迹，地面站平移列阵网带")
                state['reposition_log_time'] = self.time

        return state['center']

    def _barrier_slot_point(self, enemy, slot_idx, center=None):
        center_x, center_y, _ = center or self._barrier_intercept_center(enemy)
        slot_center = (CFG.BARRIER_GROUP_SIZE - 1) * 0.5
        dx = (slot_idx - slot_center) * CFG.BARRIER_SLOT_SPACING
        x = center_x + dx
        y = center_y
        return (
            max(0.0, min(CFG.AREA_WIDTH, x)),
            max(0.0, min(CFG.INTERCEPT_FAIL_LINE - 20.0, y)),
            self._friendly_altitude({'id': slot_idx, 'barrier_slot': slot_idx, 'z_cap': CFG.INTERCEPTOR_MAX_ALT}, mission="barrier"),
        )

    def _barrier_window_feasible(self, enemy):
        center = self._barrier_intercept_center(enemy)
        heading = math.radians(enemy.get('heading', 90.0))
        vy = enemy.get('speed', 0.0) * math.sin(heading)
        if vy <= 0.1:
            return False
        enemy_time = (center[1] - enemy['y']) / vy
        if enemy_time <= 0.0:
            return False
        team_travel = 0.0
        for slot_idx in range(CFG.BARRIER_GROUP_SIZE):
            slot_pt = self._barrier_slot_point(enemy, slot_idx, center)
            best_launch = min(
                dist2d({'x': hx, 'y': CFG.INTERCEPT_FAIL_LINE + 200.0}, {'x': slot_pt[0], 'y': slot_pt[1]})
                for hx in CFG.HANGAR_POSITIONS
            )
            team_travel = max(team_travel, best_launch)
        deploy_time = team_travel / max(CFG.INTERCEPTOR_BARRIER_SPEED, 0.1)
        return enemy_time >= deploy_time * CFG.BARRIER_TIME_MARGIN

    def _barrier_team_metrics(self, enemy, team, center):
        formed = []
        for iid in team:
            it = self._get_interceptor(iid)
            if not it or it['state'] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                continue
            if not self._interceptor_can_complete_task(it):
                continue
            if it.get('barrier_slot') is None:
                continue
            slot_pt = self._barrier_slot_point(enemy, it.get('barrier_slot', 0), center)
            if dist2d(it, {'x': slot_pt[0], 'y': slot_pt[1]}) <= CFG.BARRIER_SLOT_TOLERANCE:
                formed.append((it.get('barrier_slot', 0), it))

        formed.sort(key=lambda item: item[0])
        formed_members = [it for _, it in formed]
        cover_radius = CFG.BARRIER_NET_RADIUS + CFG.BARRIER_CAPTURE_MARGIN
        min_member_dist = float('inf')
        capture_members = []
        for it in formed_members:
            member_dist = dist2d(it, enemy)
            if member_dist < min_member_dist:
                min_member_dist = member_dist
            if member_dist <= cover_radius:
                capture_members.append(it['id'])

        min_belt_dist = min_member_dist
        if len(formed_members) >= 2:
            for left, right in zip(formed_members, formed_members[1:]):
                seg_dist, seg_t = self._point_to_segment_distance(
                    enemy['x'],
                    enemy['y'],
                    left['x'],
                    left['y'],
                    right['x'],
                    right['y'],
                )
                min_belt_dist = min(min_belt_dist, seg_dist)
                if seg_dist <= cover_radius and not capture_members:
                    capture_members = [left['id']] if seg_t <= 0.35 else ([right['id']] if seg_t >= 0.65 else [left['id'], right['id']])

        in_net = min_belt_dist <= cover_radius
        return {
            'formed_count': len(formed_members),
            'in_net': in_net,
            'min_belt_dist': min_belt_dist if min_belt_dist != float('inf') else 9999.0,
            'capture_members': capture_members,
        }

    def _update_barrier_assignments(self, targets):
        msgs = []
        active = sorted(targets, key=lambda enemy: (-enemy['y'], enemy.get('stale', False)))
        valid_targets = set()
        for enemy in active:
            team = self.barrier_team_assignments.get(enemy['id'], [])
            team = [
                iid for iid in team
                if self._get_interceptor(iid) and self._get_interceptor(iid)['state'] != IState.DESTROYED
                and self._interceptor_can_complete_task(self._get_interceptor(iid))
            ]
            if self.intercept_mode == "hybrid" and not team and not self._barrier_window_feasible(enemy):
                continue
            valid_targets.add(enemy['id'])
            state = self._barrier_state(enemy['id'])
            center = self._barrier_intercept_center(enemy)
            state['center'] = center
            needed = CFG.BARRIER_GROUP_SIZE - len(team)
            if needed > 0:
                used_slots = {
                    self._get_interceptor(iid).get('barrier_slot')
                    for iid in team
                    if self._get_interceptor(iid) and self._get_interceptor(iid).get('barrier_slot') is not None
                }
                available_slots = [idx for idx in range(CFG.BARRIER_GROUP_SIZE) if idx not in used_slots]
                pool = [
                    it for it in self.interceptors
                    if it['state'] in (IState.STANDBY, IState.RETURNING)
                    and it['target_id'] is None
                    and it['id'] not in team
                    and self._interceptor_can_complete_task(it)
                    and not it.get('task_reserved')
                ]
                pool.sort(key=lambda item: dist2d(item, {'x': center[0], 'y': center[1]}))
                for cand, slot_idx in zip(pool[:needed], available_slots[:needed]):
                    cand['state'] = IState.LAUNCHING
                    cand['target_id'] = enemy['id']
                    cand['role'] = IRole.PRIMARY
                    if self._legacy_demo_behavior_enabled():
                        cand['launch_time'] = self.time + len(team) * 0.2
                    else:
                        cand['launch_time'] = self.time + self._launch_delay_for_interceptor(cand, queue_offset=len(team))
                    cand['speed'] = 0.0
                    cand['barrier_slot'] = slot_idx
                    cand['barrier_center'] = center
                    slot_pt = self._barrier_slot_point(enemy, cand['barrier_slot'], center)
                    cand['poi'] = slot_pt
                    cand['target_z'] = slot_pt[2]
                    team.append(cand['id'])
                    msgs.append(
                        f"I-{cand['id']+1} 加入 F-{enemy['id']+1} 列阵扯网编队 | "
                        f"槽位{cand['barrier_slot']+1} -> ({slot_pt[0]:.0f},{slot_pt[1]:.0f},{slot_pt[2]:.0f})"
                    )
            self.barrier_team_assignments[enemy['id']] = team

        for enemy_id in list(self.barrier_team_assignments):
            if enemy_id in valid_targets:
                continue
            team = self.barrier_team_assignments.pop(enemy_id)
            self.barrier_states.pop(enemy_id, None)
            for iid in team:
                it = self._get_interceptor(iid)
                if it and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                    self._set_return(it, f"F-{enemy_id+1} 已脱离列阵扯网任务")
        return msgs

    def _guide_barrier_member(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            self._set_return(it, "列阵扯网目标消失，返航")
            return
        center = self._barrier_intercept_center(enemy)
        self._barrier_state(enemy['id'])['center'] = center
        slot_pt = self._barrier_slot_point(enemy, it.get('barrier_slot', 0), center)
        slot_d = dist2d(it, {'x': slot_pt[0], 'y': slot_pt[1]})
        it['barrier_center'] = center
        it['poi'] = slot_pt
        self._update_route_plan(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}, f"列阵槽位 {it.get('barrier_slot', 0)+1}")

        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            if slot_d <= CFG.BARRIER_SLOT_TOLERANCE * 0.8:
                it['speed'] = 0.0
                it['target_z'] = slot_pt[2]
                desired_heading = math.degrees(math.atan2(enemy['y'] - it['y'], enemy['x'] - it['x']))
                turn = angle_diff(desired_heading, it['heading'])
                turn = max(-CFG.INTERCEPTOR_MAX_ANG * dt, min(CFG.INTERCEPTOR_MAX_ANG * dt, turn))
                it['heading'] = (it['heading'] + turn) % 360
                move_entity(it, dt)
                it['flight_time'] += dt
            else:
                it['speed'] = min(CFG.INTERCEPTOR_BARRIER_SPEED, max(6.0, slot_d * 0.32))
                turn_rate = ProNav.guide_point(it, self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}))
                self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
            return

        desired_speed = 0.0 if slot_d <= CFG.BARRIER_SLOT_TOLERANCE * 0.8 else min(CFG.INTERCEPTOR_BARRIER_SPEED, max(6.0, slot_d * 0.32))
        local_plan = self._plan_local_motion(
            it,
            self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}),
            desired_speed,
            {
                'kind': 'barrier',
                'phase': 'hold' if slot_d <= CFG.BARRIER_SLOT_TOLERANCE * 0.8 else 'form',
                'allow_barrier_enemy_id': enemy['id'],
                'threat_y': enemy['y'],
            },
        )
        if local_plan is None:
            return

        if slot_d <= CFG.BARRIER_SLOT_TOLERANCE * 0.8 and not local_plan.avoid_mode and local_plan.speed_cap is not None and local_plan.speed_cap <= 0.1:
            it['speed'] = 0.0
            it['target_z'] = local_plan.target_z
            desired_heading = math.degrees(math.atan2(enemy['y'] - it['y'], enemy['x'] - it['x']))
            turn = angle_diff(desired_heading, it['heading'])
            turn = max(-CFG.INTERCEPTOR_MAX_ANG * dt, min(CFG.INTERCEPTOR_MAX_ANG * dt, turn))
            it['heading'] = (it['heading'] + turn) % 360
            self._apply_deconflict_limits(it)
            move_entity(it, dt)
            it['flight_time'] += dt
        else:
            self._execute_local_plan(it, dt, local_plan)

    def _check_barrier_capture(self):
        for enemy_id, team in list(self.barrier_team_assignments.items()):
            enemy = self._get_enemy(enemy_id)
            if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                continue
            state = self._barrier_state(enemy_id)
            center = self._barrier_intercept_center(enemy)
            state['center'] = center
            metrics = self._barrier_team_metrics(enemy, team, center)
            if metrics['formed_count'] >= 3 and not state.get('ready_logged'):
                self.logs.append(
                    f"[BARRIER] F-{enemy['id']+1} 列阵网带展开完成 | 到位{metrics['formed_count']}/4 | "
                    f"线心=({center[0]:.0f},{center[1]:.0f})"
                )
                state['ready_logged'] = True
            if metrics['in_net'] and metrics['formed_count'] >= 3:
                enemy['state'] = EState.DESTROYED
                self.stats['kills'] += 1
                capture_by = "/".join(f"I-{iid+1}" for iid in metrics.get('capture_members', [])[:2]) or "列阵网带"
                self.logs.append(
                    f"[BARRIER-KILL] F-{enemy['id']+1} 进入列阵网带，被 {capture_by} 捕获成功 | "
                    f"线心=({center[0]:.0f},{center[1]:.0f},{center[2]:.0f})"
                )
                for iid in team:
                    it = self._get_interceptor(iid)
                    if it and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                        self._set_return(it, f"F-{enemy['id']+1} 列阵扯网完成，返航")
                self.barrier_team_assignments.pop(enemy_id, None)
                self.barrier_states.pop(enemy_id, None)

    def _net_state(self, enemy_id):
        return self.net_capture_states.setdefault(
            enemy_id,
            {'phase': 'form', 'hold': 0.0, 'lock_logged': False, 'emergency_strike': False},
        )

    def _predict_enemy_point(self, enemy, lead_sec):
        heading = math.radians(enemy.get('heading', 90.0))
        lead_sec = max(0.0, lead_sec)
        x = enemy['x'] + math.cos(heading) * enemy.get('speed', 0.0) * lead_sec
        y = enemy['y'] + math.sin(heading) * enemy.get('speed', 0.0) * lead_sec
        z = enemy.get('z', 0.0) + enemy.get('vz', 0.0) * lead_sec
        y_cap = max(0.0, CFG.INTERCEPT_FAIL_LINE - max(40.0, CFG.POI_MARGIN * 0.5))
        return (
            max(0.0, min(CFG.AREA_WIDTH, x)),
            max(0.0, min(y_cap, y)),
            max(0.0, z),
        )

    def _net_slot_point(self, enemy, slot_idx, radius=None, lead_sec=0.0):
        radius = CFG.NET_CAPTURE_RADIUS if radius is None else radius
        center_x, center_y, center_z = self._predict_enemy_point(enemy, lead_sec)
        heading = math.radians(enemy.get('heading', 90.0))
        fx = math.cos(heading)
        fy = math.sin(heading)
        lx = -fy
        ly = fx
        slots = [
            (-lx * radius, -ly * radius),
            (lx * radius, ly * radius),
            (-fx * radius, -fy * radius),
            (fx * radius, fy * radius),
        ]
        ox, oy = slots[slot_idx % len(slots)]
        return (
            max(0.0, min(CFG.AREA_WIDTH, center_x + ox)),
            max(0.0, min(CFG.INTERCEPT_FAIL_LINE, center_y + oy)),
            center_z,
        )

    def _angular_span(self, angles):
        if len(angles) < 2:
            return 0.0
        ordered = sorted((a + 360.0) % 360.0 for a in angles)
        gaps = [ordered[idx + 1] - ordered[idx] for idx in range(len(ordered) - 1)]
        gaps.append(ordered[0] + 360.0 - ordered[-1])
        return 360.0 - max(gaps)

    def _net_team_metrics(self, enemy, team, phase):
        radius = CFG.NET_CAPTURE_RADIUS if phase == 'form' else CFG.NET_CLOSE_RADIUS
        tolerance = CFG.NET_SLOT_TOLERANCE if phase == 'form' else CFG.NET_CLOSE_TOLERANCE
        lead_sec = CFG.NET_LEAD_TIME if phase == 'form' else 0.0
        close_limit = CFG.NET_CLOSE_RADIUS + CFG.NET_CLOSE_TOLERANCE
        if enemy.get('type') == EType.LOITER:
            close_limit = max(close_limit, CFG.NET_CAPTURE_RADIUS * 1.05)
        formed = []
        close = []
        angles = []
        close_distances = []

        for iid in team:
            it = self._get_interceptor(iid)
            if not it or it['state'] not in (IState.INTERCEPTING, IState.FOLLOWING, IState.LAUNCHING):
                continue
            if not self._interceptor_can_complete_task(it):
                continue
            if it.get('net_slot') is None:
                continue
            slot_pt = self._net_slot_point(enemy, it.get('net_slot', 0), radius=radius, lead_sec=lead_sec)
            slot_d = dist2d(it, {'x': slot_pt[0], 'y': slot_pt[1]})
            enemy_d = dist2d(it, enemy)
            dz = abs(it.get('z', 0.0) - enemy.get('z', 0.0))

            if slot_d <= tolerance or enemy_d <= radius + tolerance * 0.8:
                formed.append(it)
            if enemy_d <= close_limit and dz <= CFG.HIT_ALT_TOLERANCE:
                close.append(it)
                close_distances.append(enemy_d)
                angles.append(math.degrees(math.atan2(it['y'] - enemy['y'], it['x'] - enemy['x'])))

        return {
            'formed': formed,
            'formed_count': len(formed),
            'close': close,
            'close_count': len(close),
            'span_deg': self._angular_span(angles),
            'avg_close_dist': (sum(close_distances) / len(close_distances)) if close_distances else 0.0,
        }

    def _update_net_assignments(self, targets):
        msgs = []
        active = sorted(targets, key=lambda e: (-e['y'], e.get('stale', False)))

        valid_targets = set()
        for enemy in active:
            valid_targets.add(enemy['id'])
            self._net_state(enemy['id'])
            team = self.net_team_assignments.get(enemy['id'], [])
            team = [
                iid for iid in team
                if self._get_interceptor(iid)
                and self._get_interceptor(iid)['state'] != IState.DESTROYED
                and self._interceptor_can_complete_task(self._get_interceptor(iid))
            ]
            needed = CFG.NET_GROUP_SIZE - len(team)
            if needed > 0:
                pool = [
                    it for it in self.interceptors
                    if it['state'] in (IState.STANDBY, IState.RETURNING)
                    and it['target_id'] is None
                    and it['id'] not in team
                    and self._interceptor_can_complete_task(it)
                    and not it.get('task_reserved')
                ]
                pool.sort(key=lambda item: dist2d(item, enemy))
                for cand in pool[:needed]:
                    cand['state'] = IState.LAUNCHING
                    cand['target_id'] = enemy['id']
                    cand['role'] = IRole.PRIMARY
                    if self._legacy_demo_behavior_enabled():
                        cand['launch_time'] = self.time + len(team) * 0.45
                    else:
                        cand['launch_time'] = self.time + self._launch_delay_for_interceptor(cand, queue_offset=len(team) + 1.0)
                    cand['speed'] = 0.0
                    cand['net_slot'] = len(team)
                    team.append(cand['id'])
                    slot_pt = self._net_slot_point(enemy, cand['net_slot'], radius=CFG.NET_CAPTURE_RADIUS, lead_sec=CFG.NET_LEAD_TIME)
                    slot_pt = (slot_pt[0], slot_pt[1], self._friendly_altitude(cand, enemy, mission="net", phase="form"))
                    cand['poi'] = slot_pt
                    cand['target_z'] = slot_pt[2]
                    msgs.append(
                        f"I-{cand['id']+1} 加入 F-{enemy['id']+1} 网阻编队 | "
                        f"槽位{cand['net_slot']+1} -> ({slot_pt[0]:.0f},{slot_pt[1]:.0f},{slot_pt[2]:.0f})"
                    )
            self.net_team_assignments[enemy['id']] = team

        for enemy_id in list(self.net_team_assignments):
            if enemy_id not in valid_targets:
                team = self.net_team_assignments.pop(enemy_id)
                self.net_capture_states.pop(enemy_id, None)
                for iid in team:
                    it = self._get_interceptor(iid)
                    if it and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                        self._set_return(it, f"F-{enemy_id+1} 已脱离网阻任务")
                        it['net_slot'] = None
        return msgs

    def _guide_net_member(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            it['net_slot'] = None
            self._set_return(it, "网阻目标消失，返航")
            return
        net_state = self._net_state(enemy['id'])
        if net_state.get('phase') == 'close':
            slot_xy = self._net_slot_point(enemy, it.get('net_slot', 0), radius=CFG.NET_CLOSE_RADIUS, lead_sec=0.0)
            slot_pt = (slot_xy[0], slot_xy[1], self._friendly_altitude(it, enemy, mission="net", phase="close"))
            desired_speed = self._desired_interceptor_speed(enemy, mission="net", phase="close")
            reason = f"执行收网槽位 {it.get('net_slot', 0)+1}"
        else:
            slot_xy = self._net_slot_point(enemy, it.get('net_slot', 0), radius=CFG.NET_CAPTURE_RADIUS, lead_sec=CFG.NET_LEAD_TIME)
            slot_pt = (slot_xy[0], slot_xy[1], self._friendly_altitude(it, enemy, mission="net", phase="form"))
            desired_speed = self._desired_interceptor_speed(enemy, mission="net", phase="form")
            reason = f"网阻成形槽位 {it.get('net_slot', 0)+1}"
        it['poi'] = slot_pt
        self._update_route_plan(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}, reason)
        if self._legacy_demo_behavior_enabled():
            self._clear_local_motion_state(it)
            it['speed'] = desired_speed
            turn_rate = ProNav.guide_point(it, self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}))
            self._advance_interceptor_motion(it, dt, turn_rate, apply_limits=False)
            return
        desired_point = self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]})
        local_plan = self._plan_local_motion(
            it,
            desired_point,
            desired_speed,
            {
                'kind': 'net',
                'phase': net_state.get('phase', 'form'),
                'threat_y': enemy['y'],
            },
        )
        if local_plan is None:
            return
        self._execute_local_plan(it, dt, local_plan)

    def _check_net_capture(self):
        for enemy_id, team in list(self.net_team_assignments.items()):
            enemy = self._get_enemy(enemy_id)
            if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                continue
            net_state = self._net_state(enemy_id)
            metrics = self._net_team_metrics(enemy, team, net_state.get('phase', 'form'))
            if (
                enemy.get('type') == EType.LOITER and
                enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.55 and
                metrics['formed_count'] < 3 and
                not net_state.get('emergency_strike')
            ):
                striker = None
                candidates = []
                for iid in team:
                    it = self._get_interceptor(iid)
                    if not it or it['state'] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                        continue
                    if not self._interceptor_can_complete_task(it):
                        continue
                    candidates.append(it)
                if candidates:
                    striker = min(candidates, key=lambda item: dist2d(item, enemy))
                if striker:
                    striker['net_slot'] = None
                    striker['role'] = IRole.PRIMARY
                    striker['state'] = IState.INTERCEPTING
                    striker['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                    striker['poi'] = None
                    striker['path_reason'] = ""
                    self.net_team_assignments[enemy_id] = [iid for iid in team if iid != striker['id']]
                    net_state['emergency_strike'] = True
                    self.logs.append(
                        f"[NET-EMG] F-{enemy['id']+1} 巡飞突防，I-{striker['id']+1} 切换终端撞击兜底"
                    )
                    team = self.net_team_assignments[enemy_id]
                    metrics = self._net_team_metrics(enemy, team, net_state.get('phase', 'form'))
            emergency_close = enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.7 and metrics['formed_count'] >= 3
            if enemy.get('type') == EType.LOITER and enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.62 and metrics['formed_count'] >= 2:
                emergency_close = True

            if net_state.get('phase') != 'close' and (metrics['formed_count'] >= 3 or emergency_close):
                net_state['phase'] = 'close'
                net_state['hold'] = 0.0
                net_state['lock_logged'] = False
                self.logs.append(
                    f"[NET] F-{enemy['id']+1} 网阻编队成形，开始收网 | "
                    f"到位{metrics['formed_count']}/4"
                )
                metrics = self._net_team_metrics(enemy, team, 'close')

            span_req = CFG.NET_MIN_SPAN_DEG
            hold_req = CFG.NET_CAPTURE_HOLD
            if enemy.get('type') == EType.LOITER and (enemy.get('is_diving') or enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.72):
                span_req = 140.0
                hold_req = 0.18
            force_loiter_capture = (
                enemy.get('type') == EType.LOITER and
                net_state.get('phase') == 'close' and
                metrics['formed_count'] >= 3 and
                enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.68
            )

            if force_loiter_capture:
                net_state['hold'] = hold_req
                if not net_state.get('lock_logged'):
                    self.logs.append(
                        f"[NET] F-{enemy['id']+1} 巡飞目标触发紧急收网 | "
                        f"到位{metrics['formed_count']}/4"
                    )
                    net_state['lock_logged'] = True
            elif net_state.get('phase') == 'close' and metrics['close_count'] >= 3 and metrics['span_deg'] >= span_req:
                net_state['hold'] += CFG.DT
                if not net_state.get('lock_logged'):
                    self.logs.append(
                        f"[NET] F-{enemy['id']+1} 收网锁定 | "
                        f"覆盖角{metrics['span_deg']:.0f}° | 平均半径{metrics['avg_close_dist']:.0f}m"
                    )
                    net_state['lock_logged'] = True
            else:
                net_state['hold'] = 0.0
                net_state['lock_logged'] = False

            if net_state.get('phase') == 'close' and net_state.get('hold', 0.0) >= hold_req:
                enemy['state'] = EState.DESTROYED
                self.stats['kills'] += 1
                self.logs.append(
                    f"[NET-KILL] F-{enemy['id']+1} 被 {metrics['close_count']} 架无人机网阻成功 | "
                    f"中心=({enemy['x']:.0f},{enemy['y']:.0f},{enemy.get('z', 0.0):.0f}) | "
                    f"覆盖角{metrics['span_deg']:.0f}°"
                )
                for iid in team:
                    it = self._get_interceptor(iid)
                    if it:
                        it['net_slot'] = None
                        if it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                            self._set_return(it, f"F-{enemy['id']+1} 网阻完成，返航")
                self.net_team_assignments.pop(enemy_id, None)
                self.net_capture_states.pop(enemy_id, None)

    def _update_mission_labels(self):
        for it in self.interceptors:
            if (
                it.get('reported_at', -1.0) < 0
                or (
                    not it.get('external_controlled')
                    and self.time - it.get('reported_at', -1.0) > CFG.RADAR_STALE_SEC
                )
            ):
                it['reported_x'] = it['x']
                it['reported_y'] = it['y']
                it['reported_z'] = it.get('z', 0.0)
                it['reported_speed'] = it.get('speed', 0.0)
                it['reported_heading'] = it.get('heading', 270.0)
                it['reported_vz'] = it.get('vz', 0.0)
                it['reported_roll'] = it.get('roll', 0.0)
                it['reported_pitch'] = it.get('pitch', 0.0)
                it['reported_yaw'] = it.get('yaw', it.get('heading', 270.0))
                it['reported_frame'] = it.get('frame')
                it['reported_at'] = self.time

            target = self._get_enemy(it.get('target_id'))
            if it['state'] == IState.STANDBY:
                if it.get('task_reserved'):
                    it['mission_label'] = "LLM保留"
                    it['target_label'] = "待命"
                else:
                    it['mission_label'] = "待命"
                    it['target_label'] = "-"
            elif it.get('jammed_by_interference'):
                it['mission_label'] = "通信受阻" if self._interference_phase(it) == "lost" else "链路干扰"
                it['target_label'] = it.get('jam_zone') or "强干扰区"
            elif it['state'] == IState.RETURNING:
                it['mission_label'] = "返航"
                it['target_label'] = "-"
            elif it.get('net_slot') is not None:
                target_state = self.net_capture_states.get(it.get('target_id'), {})
                prefix = "收网槽" if target_state.get('phase') == 'close' else "网阻槽"
                it['mission_label'] = f"{prefix}{it['net_slot']+1}"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            elif it.get('barrier_slot') is not None:
                it['mission_label'] = f"列阵槽{it['barrier_slot']+1}"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            elif it['state'] == IState.LAUNCHING:
                it['mission_label'] = "前出寻敌" if it.get('search_point') and not target else "起飞中"
                it['target_label'] = f"F-{target['id']+1}" if target else ("寻敌" if it.get('search_point') else "-")
            elif it['state'] == IState.INTERCEPTING:
                it['mission_label'] = "前出寻敌" if it.get('search_point') and not target else "主拦截"
                it['target_label'] = f"F-{target['id']+1}" if target else ("寻敌" if it.get('search_point') else "-")
            elif it['state'] == IState.FOLLOWING:
                it['mission_label'] = "随动"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            else:
                it['mission_label'] = "待命"
                it['target_label'] = "-"

            local_mode = it.get('local_avoid_mode', '')
            local_hold = it.get('local_hold_reason', '')
            if local_mode and it['state'] not in (IState.STANDBY, IState.DESTROYED, IState.LANDED):
                if local_hold == "等待超时":
                    it['mission_label'] = "等待超时"
                elif local_mode == "等待放行":
                    it['mission_label'] = "等待放行"
                elif local_mode == "紧急脱网":
                    it['mission_label'] = "脱网"
                else:
                    it['mission_label'] = local_mode

    def _set_return(self, it, reason):
        fast_return = it.get('barrier_slot') is not None or it.get('net_slot') is not None
        it['state'] = IState.RETURNING
        it['target_id'] = None
        it['role'] = IRole.RESERVE
        it['search_until'] = 0.0
        it['path_plan'] = []
        it['path_reason'] = ""
        it['poi'] = None
        it['search_point'] = None
        it['search_distance'] = 0.0
        it['return_fast'] = fast_return
        it['speed'] = min(CFG.INTERCEPTOR_BOOST_SPEED, max(CFG.INTERCEPTOR_SPEED * (1.4 if fast_return else 1.0), it.get('speed', 0.0)))
        it['net_slot'] = None
        it['barrier_slot'] = None
        it['barrier_center'] = None
        self._clear_local_motion_state(it)
        self.deconfliction.reset_local_state(it['id'])
        self.logs.append(f"[RTB] I-{it['id']+1} {reason}")
        self.deconflict_cooldown.pop(it['id'], None)

    def _dispatch_command_requested(self, text):
        text = str(text or "")
        lower = text.lower()
        direct_words = (
            "派出", "出动", "增援", "支援", "起飞", "放飞",
            "派一架", "派1架", "新无人机", "新的无人机", "再来一架",
            "左翼出动", "右翼出动", "前出",
        )
        if any(word in text for word in direct_words):
            return True
        if "最近" in text and any(word in text for word in ("拦截", "接敌", "处置")):
            return True
        if "dispatch" in lower or "scramble" in lower:
            return True
        if "发射" in text and any(word in text for word in ("一架", "1架", "新", "无人机", "增援", "支援")):
            return True
        return False

    def _parse_command_count(self, text, default=1):
        text = str(text or "")
        match = re.search(r"(\d+)\s*架", text)
        if match:
            return max(1, min(CFG.NUM_INTERCEPTORS, int(match.group(1))))
        chinese_nums = {
            "一": 1, "两": 2, "二": 2, "三": 3, "四": 4, "五": 5,
            "六": 6, "七": 7, "八": 8, "九": 9, "十": 10,
        }
        for word, value in chinese_nums.items():
            if f"{word}架" in text or f"{word}个" in text:
                return max(1, min(CFG.NUM_INTERCEPTORS, value))
        return default

    def _parse_search_distance(self, text, default=500.0):
        text = str(text or "")
        match = re.search(r"(?:前出|前推|推进|外推)\s*(\d+(?:\.\d+)?)\s*(公里|千米|km|米|m)?", text, re.IGNORECASE)
        if not match:
            return float(default)
        value = float(match.group(1))
        unit = (match.group(2) or "米").lower()
        if unit in ("公里", "千米", "km"):
            value *= 1000.0
        return max(20.0, min(CFG.AREA_HEIGHT, value))

    def _parse_wing(self, text):
        text = str(text or "")
        if "左翼" in text or "左侧" in text or "左边" in text:
            return "left"
        if "右翼" in text or "右侧" in text or "右边" in text:
            return "right"
        return None

    def _standby_candidates(self, wing=None):
        max_active = self._max_active_limit()
        if max_active > 0 and self._committed_interceptor_count() >= max_active:
            return []
        standby = [
            it for it in self.interceptors
            if it['state'] == IState.STANDBY
            and it.get('target_id') is None
            and self._interceptor_can_complete_task(it)
            and not it.get('task_reserved')
        ]
        if wing == "left":
            filtered = [it for it in standby if it['x'] <= CFG.AREA_WIDTH * 0.5]
            return filtered or standby
        if wing == "right":
            filtered = [it for it in standby if it['x'] >= CFG.AREA_WIDTH * 0.5]
            return filtered or standby
        return standby

    def _assistant_dispatch_suggested(self, text):
        text = str(text or "")
        if any(word in text for word in ("未派出", "暂不出动", "保持", "无待命机")):
            return False
        if self._dispatch_command_requested(text):
            return True
        has_named_pair = bool(re.search(r"I-\d+.*F-\d+|F-\d+.*I-\d+", text, re.IGNORECASE))
        action_words = ("执行", "拦截", "撞击", "列阵", "扯网", "接敌", "处置")
        return has_named_pair and any(word in text for word in action_words)

    def _dispatch_target_candidates(self):
        candidates = [
            enemy for enemy in self.enemies
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and not enemy.get('lost', False)
        ]
        candidates = self.assigner.sort_active_enemies(candidates)
        candidates.sort(key=lambda enemy: not enemy.get('detected', False))
        return candidates

    def _dispatch_one_interceptor(self, reason="指挥员语音", record_log=True, search_distance=500.0, wing=None):
        max_active = self._max_active_limit()
        if max_active > 0 and self._committed_interceptor_count() >= max_active:
            return f"已达到LLM最多出动{max_active}架限制"
        standby = self._standby_candidates(wing=wing)
        if not standby:
            return "无可用待命机"

        chosen = None
        for enemy in self._dispatch_target_candidates():
            asgn = self.assigner.assignments.setdefault(enemy['id'], {'primary': None, 'follower': None})
            if asgn.get('primary') is None:
                chosen = (enemy, IRole.PRIMARY, 'primary', 0.0)
                break
            if asgn.get('follower') is None:
                chosen = (enemy, IRole.FOLLOWER, 'follower', 1.0)
                break

        if chosen is None:
            it = min(standby, key=lambda item: item['id'])
            search_point = {
                'x': it['x'],
                'y': max(0.0, it['y'] - float(search_distance)),
                'z': self._friendly_altitude(it, mission="search", phase="idle"),
            }
            it['state'] = IState.LAUNCHING
            it['target_id'] = None
            it['role'] = IRole.PRIMARY
            it['speed'] = 0.0
            it['launch_time'] = self.time + self._launch_delay_for_interceptor(it)
            it['target_z'] = search_point['z']
            it['search_until'] = 0.0
            it['search_point'] = search_point
            it['search_distance'] = float(search_distance)
            it['poi'] = (search_point['x'], search_point['y'], search_point['z'])
            it['path_plan'] = []
            it['path_reason'] = ""
            it['mission_label'] = "前出寻敌"
            it['target_label'] = "寻敌"
            self._clear_local_motion_state(it)
            self.deconfliction.reset_local_state(it['id'])
            result = f"已派出I-{it['id']+1}，无目标前出{search_distance:.0f}米自主寻敌"
            if record_log:
                self.logs.append(f"[CMD] {reason}: {result}")
            return result

        enemy, role, role_key, queue_offset = chosen
        it = min(standby, key=lambda item: dist2d(item, enemy))

        if not enemy.get('detected', False):
            enemy['detected'] = True
            enemy['detect_time'] = self.time
            if record_log:
                self.logs.append(f"[DET] F-{enemy['id']+1} 按副官/语音指令提前接敌")

        it['state'] = IState.LAUNCHING
        it['target_id'] = enemy['id']
        it['role'] = role
        it['speed'] = 0.0
        it['launch_time'] = self.time + self._launch_delay_for_interceptor(it, queue_offset=queue_offset)
        it['target_z'] = enemy.get('z', CFG.INTERCEPTOR_CRUISE_ALT)
        it['search_until'] = 0.0
        it['search_point'] = None
        it['mission_label'] = "副官派出"
        it['target_label'] = f"F-{enemy['id']+1}"
        self._clear_local_motion_state(it)
        self.deconfliction.reset_local_state(it['id'])

        asgn = self.assigner.assignments.setdefault(enemy['id'], {'primary': None, 'follower': None})
        asgn[role_key] = it['id']
        poi, eta = self.assigner.compute_poi(it, enemy)
        asgn['poi'] = poi
        asgn['eta'] = eta
        if role_key == 'primary':
            asgn['launch_anchor'] = it['launch_time']

        role_text = "主拦截" if role == IRole.PRIMARY else "随动增援"
        result = f"已派出I-{it['id']+1}，对F-{enemy['id']+1}执行{role_text}"
        if record_log:
            self.logs.append(f"[CMD] {reason}: {result}")
        return result

    def _dispatch_interceptors(self, count=1, reason="指挥员语音", search_distance=500.0, wing=None):
        results = []
        for _ in range(max(1, int(count))):
            result = self._dispatch_one_interceptor(
                reason=reason,
                record_log=False,
                search_distance=search_distance,
                wing=wing,
            )
            results.append(result)
            if result == "无可用待命机":
                break
        return "；".join(results)

    def _resume_auto_tasking(self):
        self._update_enemy_detection()
        engageable_enemies = [
            enemy for enemy in self.enemies
            if enemy['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and enemy.get('detected')
            and not enemy.get('lost', False)
        ]
        if not engageable_enemies:
            return 0, len(engageable_enemies)
        hit_targets, barrier_targets, net_targets = self._split_targets_by_mode(engageable_enemies)
        barrier_ids = {enemy['id'] for enemy in barrier_targets}
        net_ids = {enemy['id'] for enemy in net_targets}
        if barrier_ids:
            self._prune_hit_assignments({enemy['id'] for enemy in hit_targets}, "解除警戒后恢复协同任务，释放撞击编队")
        elif net_ids:
            self._prune_hit_assignments({enemy['id'] for enemy in hit_targets}, "解除警戒后恢复网阻任务，释放撞击编队")

        before_launching = sum(1 for item in self.interceptors if item['state'] == IState.LAUNCHING)
        if self.intercept_mode in ("hit", "hybrid"):
            if getattr(self, "demo_strategy_mode", "cooperative") == "baseline":
                for msg in self._update_baseline_assignments(hit_targets):
                    self.logs.append(f"[BASE] {msg}")
            else:
                for msg in self.assigner.update(self.interceptors, hit_targets, self.time):
                    self.logs.append(f"[ASGN] {msg}")
        if self.intercept_mode in ("net", "hybrid"):
            for msg in self._update_barrier_assignments(barrier_targets):
                self.logs.append(f"[BARRIER] {msg}")
        if self.intercept_mode == "legacy-net":
            for msg in self._update_net_assignments(net_targets):
                self.logs.append(f"[NET] {msg}")
        after_launching = sum(1 for item in self.interceptors if item['state'] == IState.LAUNCHING)
        new_launches = max(0, after_launching - before_launching)
        return new_launches, len(engageable_enemies)

    def _launch_all_interceptors(self):
        n = 0
        reserved = self.get_reserved_interceptor_count()
        max_active = self._max_active_limit()
        remaining_capacity = None if max_active <= 0 else max(0, max_active - self._committed_interceptor_count())
        for it in self.interceptors:
            if remaining_capacity is not None and remaining_capacity <= 0:
                break
            if it['state'] == IState.STANDBY and not it.get('task_reserved'):
                it['state'] = IState.LAUNCHING
                it['speed'] = 0.0
                it['launch_time'] = self.time
                it['search_point'] = None
                it['search_distance'] = 0.0
                n += 1
                if remaining_capacity is not None:
                    remaining_capacity -= 1
        if max_active > 0:
            limit_text = f"，受LLM上限约束最多出动{max_active}架"
        else:
            limit_text = ""
        if reserved:
            return f"已确认执行：全体发射，已发射{n}架{limit_text}，LLM保留{reserved}架待命"
        return f"已确认执行：全体发射，已发射{n}架{limit_text}" if n else "已确认执行：无可用拦截机"

    def _recall_all_interceptors(self):
        n = 0
        for it in self.interceptors:
            if it['state'] in (IState.INTERCEPTING, IState.FOLLOWING, IState.LAUNCHING):
                self._set_return(it, "收到设备一返航指令")
                n += 1
        return f"已确认执行：全体返航，已召回{n}架" if n else "已确认执行：无在空拦截机"

    def _set_intercept_mode(self, mode):
        self.intercept_mode = mode
        self.assigner = InterceptionAssigner()
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}
        return f"已确认执行：切换为{self._mode_label()}，任务分配已重新初始化"

    def _request_confirmation(self, label, action):
        self.pending_confirmation = {'label': label, 'action': action}
        return f"收到，是否确认{label}？请说“确认”执行，或说“取消”。"

    def _handle_confirmation_reply(self, cmd):
        text = str(cmd or "").strip().lower()
        confirm_words = ("确认", "执行", "同意", "可以", "yes", "ok", "okay")
        cancel_words = ("取消", "不要", "停止", "否", "no", "cancel")
        if any(word in text for word in cancel_words):
            if not self.pending_confirmation:
                return "当前没有待取消的危险动作"
            label = self.pending_confirmation.get('label', '危险动作')
            self.pending_confirmation = None
            return f"已取消：{label}"
        if any(word in text for word in confirm_words):
            if not self.pending_confirmation:
                return "当前没有待确认动作"
            pending = self.pending_confirmation
            self.pending_confirmation = None
            action = pending.get('action', {})
            kind = action.get('kind')
            if kind == "launch_all":
                return self._launch_all_interceptors()
            if kind == "recall_all":
                return self._recall_all_interceptors()
            if kind == "scene":
                scene_km = float(action.get('scene_km', CFG.SCENE_KM))
                self.configure_scene(scene_km, reset=True)
                return f"已确认执行：切换到 {scene_km:.0f}km 场景并重置闭环规划"
            if kind == "mode":
                return self._set_intercept_mode(action.get('mode', self.intercept_mode))
            return "已确认，但动作类型无法识别"
        return None

    def _parse_mode_command(self, text):
        text = str(text or "")
        if "模式" not in text and "mode" not in text.lower():
            return None
        if "撞击" in text or "hit" in text.lower():
            return "hit"
        if "混合" in text or "hybrid" in text.lower():
            return "hybrid"
        if "高级网阻" in text or "legacy" in text.lower():
            return "legacy-net"
        if "列阵" in text or "扯网" in text or "网阻" in text or "net" in text.lower():
            return "net"
        return None

    def _apply_assistant_action(self, chat_msg):
        if not self._assistant_dispatch_suggested(chat_msg):
            return None
        if self.time - self.last_assistant_dispatch_time < 2.0:
            return None
        self.last_assistant_dispatch_time = self.time
        return self._dispatch_one_interceptor("副官建议", record_log=True)

    def get_rate(self):
        total = self.stats['kills'] + self.stats['penetrations']
        return self.stats['kills'] / total if total > 0 else 0.0

    def get_lost_interceptor_count(self):
        return len(self._llm_lost_interceptors())

    def get_reserved_interceptor_count(self):
        self._update_llm_reserved_pool()
        return sum(
            1 for item in self.interceptors
            if item.get('task_reserved') and item['state'] in (IState.STANDBY, IState.LANDED)
        )

    def get_effectiveness_rate(self):
        total = max(1, int(self.stats.get('total_enemies', 0) or 0))
        lost_penalty = self.get_lost_interceptor_count() * 0.5
        return max(0.0, min(1.0, (self.stats.get('kills', 0) - lost_penalty) / total))

    def get_avg_alt(self):
        alts = self.stats['intercept_alts']
        return sum(alts) / len(alts) if alts else 0.0

    def get_standby(self):
        return sum(1 for item in self.interceptors if item['state'] == IState.STANDBY)

    def process_command(self, cmd):
        """
        处理用户的输入语音或者通过 UI 文本框打字输入的文本指令
        :param cmd:
        :return:
        """
        cmd = cmd.strip()
        if not cmd:
            return ""
        cmd_lower = cmd.lower()
        # 危险动作确认拦截机制（防止误触）
        if self.pending_confirmation:
            confirmation_result = self._handle_confirmation_reply(cmd)
            if confirmation_result is not None:
                return confirmation_result
            label = self.pending_confirmation.get('label', '危险动作')
            return f"当前待确认：{label}。请先说“确认”或“取消”。"
        if cmd_lower in ("确认", "执行", "同意", "可以", "yes", "ok", "okay", "取消", "不要", "停止", "否", "no", "cancel"):
            confirmation_result = self._handle_confirmation_reply(cmd)
            if confirmation_result is not None:
                return confirmation_result

        # 解析大模型分配约束
        # 比如指令说"保留两架无人机，绕开干扰"，就会在 _apply_llm_task_constraints_from_command
        # 里解析出 reserve_count=2, avoid_jam=True
        llm_constraint_result = self._apply_llm_task_constraints_from_command(cmd)
        if llm_constraint_result is not None:
            return llm_constraint_result

        # 解析系统命令：切换场景大小
        scene_match = re.search(r"(?:场景|scene)\s*([1-9]|10)(?:km)?", cmd_lower)
        if not scene_match:
            scene_match = re.search(r"\b([1-9]|10)\s*km\b", cmd_lower)
        if scene_match:
            scene_km = float(scene_match.group(1))
            return self._request_confirmation(
                f"切换到{scene_km:.0f}km场景并重置闭环规划",
                {'kind': 'scene', 'scene_km': scene_km},
            )

        mode = self._parse_mode_command(cmd)
        if mode:
            old_mode = self._mode_label()
            old_value = self.intercept_mode
            self.intercept_mode = mode
            new_mode = self._mode_label()
            self.intercept_mode = old_value
            return self._request_confirmation(
                f"从{old_mode}切换为{new_mode}",
                {'kind': 'mode', 'mode': mode},
            )

        # 解析具体动作：出动一架或者全体返航等
        is_launch = ("发射" in cmd_lower or "launch" in cmd_lower
                     or bool(re.search(r'(?<!\w)f1(?!\d)', cmd_lower)))
        is_dispatch_one = self._dispatch_command_requested(cmd)
        is_recall = ("撤退" in cmd_lower or "返航" in cmd_lower
                     or "recall" in cmd_lower
                     or bool(re.search(r'(?<!\w)f2(?!\d)', cmd_lower)))
        is_status = ("状态" in cmd_lower or "status" in cmd_lower)
        is_all = any(word in cmd for word in ("全体", "全部", "所有", "全都"))

        # 警戒态势控制（冻结自动派出机制）
        if "保持警戒" in cmd or "警戒" == cmd.strip():
            self.command_posture = "guard"
            return "已进入警戒态势：不再自动新增派机，已派出无人机会继续执行；可下令“派出一架/派出两架/全部返航”。"


        if "解除警戒" in cmd or "恢复自动" in cmd or "自动接敌" in cmd:
            self.command_posture = "normal"
            new_launches, active_target_count = self._resume_auto_tasking()
            if new_launches > 0:
                return f"已解除警戒态势：恢复自动接敌与任务分配；已立即补派{new_launches}架。"
            if active_target_count > 0:
                max_active = self._max_active_limit()
                if max_active > 0 and self._committed_interceptor_count() >= max_active:
                    return f"已解除警戒态势：恢复自动接敌与任务分配；但当前仍受LLM最多出动{max_active}架限制。"
                available = len(self._standby_candidates())
                if available <= 0:
                    return "已解除警戒态势：恢复自动接敌与任务分配；但当前无可用待命机。"
                return "已解除警戒态势：恢复自动接敌与任务分配；当前目标已恢复到自动分配队列。"
            return "已解除警戒态势：恢复自动接敌与任务分配。"

        if is_recall:
            label = "全体返航" if is_all or "f2" in cmd_lower else "返航当前在空无人机"
            return self._request_confirmation(label, {'kind': 'recall_all'})

        if (is_launch and is_all) or (is_dispatch_one and is_all):
            return self._request_confirmation("全体发射", {'kind': 'launch_all'})

        if is_dispatch_one:
            count = self._parse_command_count(cmd, default=1)
            search_distance = self._parse_search_distance(cmd, default=500.0)
            wing = self._parse_wing(cmd)
            return self._dispatch_interceptors(
                count=count,
                reason="指挥员语音",
                search_distance=search_distance,
                wing=wing,
            )

        if is_launch:
            return self._request_confirmation("全体发射", {'kind': 'launch_all'})

        if is_status:
            standby = self.get_standby()
            reserved = self.get_reserved_interceptor_count()
            active = sum(1 for item in self.interceptors if item['state'] in (IState.INTERCEPTING, IState.FOLLOWING))
            max_active = self._max_active_limit()
            preferred_sector = self._sector_value_text((getattr(self, "llm_task_constraints", {}) or {}).get('preferred_sector'))
            return (
                f"场景:{CFG.SCENE_KM:.0f}km "
                f"待命:{standby} 保留:{reserved} 拦截中:{active} "
                f"击毁:{self.stats['kills']} 突防:{self.stats['penetrations']} "
                f"区域优先:{preferred_sector} 上限:{max_active if max_active > 0 else 'all'} "
                f"模式:{self._mode_label()} "
                f"态势:{'警戒' if self.command_posture == 'guard' else '自动'} "
                f"数据源:{'实时' if self.has_live_data else ('本地回放' if self.demo_mode else '等待实时数据')}"
            )

        # 如果都不是以上动作，则把自然语言发送给大模型analyst,让LLM决定怎么回复
        active_enemies = [e for e in self.enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        self.analyst.chat(cmd, active_enemies, self.interceptors)
        return "▧ 正在同步至战术数据链..."

# 解析 run_fusion_custom.sh 传进来的长串参数，初始化 Pygame 渲染器、网络通信（Redis/UDP）、语音引擎和大模型面板，并维持一个按 FPS 刷新的主循环

def run_demo(seed=42, test_wav=None, scene_km=10.0, source="auto",
             redis_host="127.0.0.1", redis_port=6379, redis_db=0, redis_password=None,
             intercept_mode="hybrid", demo_case=None, fullscreen=False, ui_style="arc",
             publish_redis=False, publish_interval=0.5, friendly_start=1, enemy_start=101,
             publish_redis_mode="default", publish_redis_side="all", publish_redis_host=None,
             publish_redis_port=None, publish_redis_db=None, publish_redis_password=None,
             publish_udp=False, udp_out_host="127.0.0.1", udp_out_port=9999, udp_enemy_only=False,
             publish_udp_mode="teacher", udp_in_host="0.0.0.0", udp_in_port=8020,
             hangar_mode="multi", enemy_redis_format="auto", friendly_return_source="udp",
             geo_origin_lat=34.2663, geo_origin_lon=108.9549,
             radar_stale_sec=None, radar_lost_sec=None, target_search_sec=None,
             enemy_assoc="on", enemy_assoc_max_distance=450.0,
             enemy_assoc_max_altitude=140.0, enemy_assoc_keep_sec=18.0,
             enemy_hash_remap_mode="direct",
             enemy_hash_center_x_ratio=0.5, enemy_hash_lateral_scale=1.0,
             enemy_hash_range_scale=5.0, enemy_hash_start_range_m=0.0,
             enemy_hash_y_offset_m=0.0,
             enemy_hash_hide_outbound="off",
             enemy_flat_remap_mode="legacy",
             enemy_flat_rotate_deg=135.0, enemy_flat_flip_x="off",
             enemy_flat_flip_y="off", enemy_flat_scale=1.0,
             enemy_flat_center_x_ratio=0.5, enemy_flat_center_y_ratio=0.2,
             demo_interference_enable=True, demo_interference_visible=True,
             demo_scheme=0, llm_dashboard=False, llm_dashboard_host="127.0.0.1",
             llm_dashboard_port=8765, llm_dashboard_open=False):

    """
    把项目中分散的各个组件（物理环境、网络、大模型面板、渲染引擎等）实例化，并拉入一个无限循环的仿真时间线中，驱动它们相互配合运转
    1. 统一初始化：它充当了系统启动的“黏合剂”，负责实例化物理环境（InterceptionEnvironment）、大模型语音引擎（VoiceEngine）、大模型看板、Redis 发布器、UDP 发布器以及 Pygame 图形渲染器。
    2. 驱动主仿真循环（时间轴推进）：建立一个基于固定帧率（CFG.FPS，通常是 30 或 60 帧）的 while running: 循环。在每一帧中，它让环境推进 dt 秒，更新全场飞机的状态。
    3. 网络数据发布控制：在主循环的每一个周期，它负责驱动各个网络发布器（如前面讲过的 DemoRedisPublisher、UDPFramePublisher 等），将最新的敌我坐标广播出去。
    4. 管理全局事件与生命周期：监听键盘、鼠标操作（如用户在界面上点选飞机、按键暂停、退出），并在程序关闭或被用户中断（Ctrl+C）时，安全地调用各发布器的 cleanup() 函数，抹除 Redis 缓存中的废弃数据，防止污染下一次运行。


    :param seed:
    :param test_wav:
    :param scene_km:
    :param source:
    :param redis_host:
    :param redis_port:
    :param redis_db:
    :param redis_password:
    :param intercept_mode:
    :param demo_case:
    :param fullscreen:
    :param ui_style:
    :param publish_redis:
    :param publish_interval:
    :param friendly_start:
    :param enemy_start:
    :param publish_redis_mode:
    :param publish_redis_side:
    :param publish_redis_host:
    :param publish_redis_port:
    :param publish_redis_db:
    :param publish_redis_password:
    :param publish_udp:
    :param udp_out_host:
    :param udp_out_port:
    :param udp_enemy_only:
    :param publish_udp_mode:
    :param udp_in_host:
    :param udp_in_port:
    :param hangar_mode:
    :param enemy_redis_format:
    :param friendly_return_source:
    :param geo_origin_lat:
    :param geo_origin_lon:
    :param radar_stale_sec:
    :param radar_lost_sec:
    :param target_search_sec:
    :param enemy_assoc:
    :param enemy_assoc_max_distance:
    :param enemy_assoc_max_altitude:
    :param enemy_assoc_keep_sec:
    :param enemy_hash_remap_mode:
    :param enemy_hash_center_x_ratio:
    :param enemy_hash_lateral_scale:
    :param enemy_hash_range_scale:
    :param enemy_hash_start_range_m:
    :param enemy_hash_y_offset_m:
    :param enemy_hash_hide_outbound:
    :param enemy_flat_remap_mode:
    :param enemy_flat_rotate_deg:
    :param enemy_flat_flip_x:
    :param enemy_flat_flip_y:
    :param enemy_flat_scale:
    :param enemy_flat_center_x_ratio:
    :param enemy_flat_center_y_ratio:
    :param demo_interference_enable:
    :param demo_interference_visible:
    :param demo_scheme:
    :param llm_dashboard:
    :param llm_dashboard_host:
    :param llm_dashboard_port:
    :param llm_dashboard_open:
    :return:
    """

    import pygame

    CFG.set_hangar_mode(hangar_mode)
    if radar_stale_sec is not None:
        CFG.RADAR_STALE_SEC = max(0.5, float(radar_stale_sec))
    if radar_lost_sec is not None:
        CFG.RADAR_LOST_SEC = max(CFG.RADAR_STALE_SEC + 0.5, float(radar_lost_sec))
    if target_search_sec is not None:
        CFG.TARGET_SEARCH_SEC = max(1.0, float(target_search_sec))


    # 实例化核心的物理世界和拦截环境，将雷达数据，防区大小和拦截模式等核心参数喂入
    env = InterceptionEnvironment(
        seed=seed,
        scene_km=scene_km,
        source=source,
        redis_host=redis_host,
        redis_port=redis_port,
        redis_db=redis_db,
        redis_password=redis_password,
        intercept_mode=intercept_mode,
        demo_case=demo_case,
        udp_in_host=udp_in_host,
        udp_in_port=udp_in_port,
        enemy_redis_format=enemy_redis_format,
        friendly_return_source=friendly_return_source,
        geo_origin_lat=geo_origin_lat,
        geo_origin_lon=geo_origin_lon,
        enemy_assoc=enemy_assoc,
        enemy_assoc_max_distance=enemy_assoc_max_distance,
        enemy_assoc_max_altitude=enemy_assoc_max_altitude,
        enemy_assoc_keep_sec=enemy_assoc_keep_sec,
        enemy_hash_remap_mode=enemy_hash_remap_mode,
        enemy_hash_center_x_ratio=enemy_hash_center_x_ratio,
        enemy_hash_lateral_scale=enemy_hash_lateral_scale,
        enemy_hash_range_scale=enemy_hash_range_scale,
        enemy_hash_start_range_m=enemy_hash_start_range_m,
        enemy_hash_y_offset_m=enemy_hash_y_offset_m,
        enemy_hash_hide_outbound=enemy_hash_hide_outbound,
        enemy_flat_remap_mode=enemy_flat_remap_mode,
        enemy_flat_rotate_deg=enemy_flat_rotate_deg,
        enemy_flat_flip_x=enemy_flat_flip_x,
        enemy_flat_flip_y=enemy_flat_flip_y,
        enemy_flat_scale=enemy_flat_scale,
        enemy_flat_center_x_ratio=enemy_flat_center_x_ratio,
        enemy_flat_center_y_ratio=enemy_flat_center_y_ratio,
        demo_interference_enable=demo_interference_enable,
        demo_interference_visible=demo_interference_visible,
        demo_scheme=demo_scheme,
    )
    from ui.renderer import DemoRenderer
    # 初始化本地GUI图像渲染器
    renderer = DemoRenderer(env, fullscreen=fullscreen, ui_style=ui_style)
    clock = pygame.time.Clock()
    started, paused, sim_speed, running = False, False, 1, True
    time_accumulator = 0.0
    llm_dashboard_server = None
    redis_publisher = None
    udp_publisher = None
    plan_exporter = None
    redis_publish_host = publish_redis_host or redis_host
    redis_publish_port = int(publish_redis_port if publish_redis_port is not None else redis_port)
    redis_publish_db = int(publish_redis_db if publish_redis_db is not None else redis_db)
    redis_publish_password = publish_redis_password if publish_redis_password is not None else redis_password
    publish_redis_side = _normalize_publish_side(publish_redis_side)
    teacher_compat_mode = (publish_redis_mode == "teacher-friendly")
    geo_hash_mode = (publish_redis_mode == "geo-hash")

    # 根据命令行开关有选择地初始化Redis数据发布通道
    if teacher_compat_mode:
        if publish_udp:
            udp_enemy_only = True

    if publish_redis:
        try:
            if teacher_compat_mode:
                redis_publisher = TeacherFriendlyRedisPublisher(
                    host=redis_publish_host,
                    port=redis_publish_port,
                    db=redis_publish_db,
                    password=redis_publish_password,
                    publish_interval=publish_interval,
                    friendly_start=friendly_start,
                    enemy_start=enemy_start,
                    publish_side=publish_redis_side,
                    cleanup_node_nums=_planned_publish_node_nums(env, friendly_start, enemy_start, publish_redis_side),
                )
                renderer.add_log(
                    "[SYNC]",
                    (
                        f"老师Redis兼容发布已开启 | {redis_publish_host}:{redis_publish_port}/{redis_publish_db} "
                        f"| side={publish_redis_side} | Friendly {friendly_start}-{friendly_start + len(env.interceptors) - 1} "
                        f"| Enemy {enemy_start}+"
                    ),
                    "green",
                )
            elif geo_hash_mode:
                redis_publisher = GeoHashRedisPublisher(
                    host=redis_publish_host,
                    port=redis_publish_port,
                    db=redis_publish_db,
                    password=redis_publish_password,
                    publish_interval=publish_interval,
                    friendly_start=friendly_start,
                    enemy_start=enemy_start,
                    geo_origin_lat=geo_origin_lat,
                    geo_origin_lon=geo_origin_lon,
                    publish_side=publish_redis_side,
                )
                renderer.add_log(
                    "[SYNC]",
                    (
                        f"Geo Hash发布已开启 | {redis_publish_host}:{redis_publish_port}/{redis_publish_db} "
                        f"| side={publish_redis_side} | Origin=({geo_origin_lat:.4f},{geo_origin_lon:.4f})"
                    ),
                    "green",
                )
            else:
                redis_publisher = DemoRedisPublisher(
                    host=redis_publish_host,
                    port=redis_publish_port,
                    db=redis_publish_db,
                    password=redis_publish_password,
                    publish_interval=publish_interval,
                    friendly_start=friendly_start,
                    enemy_start=enemy_start,
                    cleanup_node_nums=_planned_publish_node_nums(env, friendly_start, enemy_start, publish_redis_side),
                    publish_side=publish_redis_side,
                )
                renderer.add_log(
                    "[SYNC]",
                    f"同帧发布已开启 | side={publish_redis_side} | Friendly {friendly_start}-{friendly_start + len(env.interceptors) - 1} | Enemy {enemy_start}+",
                    "green",
                )
            redis_publisher.maybe_publish(env, force=True)
        except Exception as exc:
            renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
            redis_publisher = None

    if publish_udp:
        try:
            udp_publisher = UDPFramePublisher(
                host=udp_out_host,
                port=udp_out_port,
                publish_interval=publish_interval,
                enemy_only=udp_enemy_only,
                friendly_start=friendly_start,
                enemy_start=enemy_start,
                publish_mode=publish_udp_mode,
                geo_origin_lat=geo_origin_lat,
                geo_origin_lon=geo_origin_lon,
            )
            mode_text = "敌方" if udp_enemy_only else "敌我"
            renderer.add_log(
                "[UDP]",
                f"UDP发布已开启 -> {udp_out_host}:{udp_out_port} | {mode_text} | mode={publish_udp_mode}",
                "green",
            )
            udp_publisher.maybe_publish(env, force=True)
        except Exception as exc:
            renderer.add_log("[UDP]", f"UDP发布失败: {exc}", "red")
            udp_publisher = None

    if _HAS_PLAN_EXPORT and PlanExportConfig and PlannerExporter:
        try:
            plan_export_config = PlanExportConfig.from_config()
            if plan_export_config.enabled:
                plan_exporter = PlannerExporter(plan_export_config)
                renderer.add_log("[PLAN]", plan_export_config.summary(), "green")
                for tag, message, color in plan_exporter.maybe_publish(env, force=True):
                    renderer.add_log(tag, message, color)
        except Exception as exc:
            renderer.add_log("[PLAN]", f"设备2到设备一决策发布启动失败: {exc}", "red")
            plan_exporter = None
    else:
        try:
            from core.common import PLAN_EXPORT as _plan_export_settings
            if bool(_plan_export_settings.get("enabled", False)):
                renderer.add_log("[PLAN]", "规划发布模块未加载，core.common.PLAN_EXPORT enabled 被忽略", "amber")
        except Exception:
            pass

    # 初始化大模型专用的网页端看板
    if llm_dashboard:
        try:
            from ui.llm_dashboard import LLMDashboardServer
            llm_dashboard_server = LLMDashboardServer(
                env,
                host=llm_dashboard_host,
                port=llm_dashboard_port,
                open_browser=llm_dashboard_open,
            )
            # 在后台线程跑大模型的“思维面板”
            url = llm_dashboard_server.start()
            renderer.add_log("[LLM-WEB]", f"LLM决策链网页已开启: {url}", "green")
            print(f"LLM dashboard: {url}")
        except Exception as exc:
            renderer.add_log("[LLM-WEB]", f"LLM决策链网页启动失败: {exc}", "red")
            llm_dashboard_server = None

    # 初始化大模型的语音引擎
    voice_engine = None
    if _HAS_VOICE:
        try:
            voice_engine = VoiceEngine(test_wav=test_wav)
            if test_wav:
                renderer.add_log("[SYS]", f"语音测试模式: {test_wav}", "amber")
            else:
                renderer.add_log("[SYS]", "语音模块已加载 (按住V说话)", "green")
        except Exception as exc:
            renderer.add_log("[SYS]", f"语音模块不可用: {exc}", "amber")
    else:
        renderer.add_log("[SYS]", "语音模块未安装 (pip install pyaudio faster-whisper)", "txtd")

    print("=" * 60)
    print("空域拦截防御系统 v8.0")
    print(
        f"Scene={CFG.SCENE_KM:.0f}km Source={source} Mode={intercept_mode} "
        f"UI={ui_style} Hangar={CFG.HANGAR_MODE} Case={demo_case or 'default'} "
        f"Redis={redis_host}:{redis_port}/{redis_db}"
    )
    print("Enter-开始 Space-暂停 P-显隐预测线 F11-全屏切换")
    print("UI右上角 1/2/3: 传统 / 协同 / 干扰失联")
    print("1/2/4/8-常规速度 0-极速(20x)")
    print("T-指令 V-按住语音 R-同场景重置 Shift+R-新seed F1-发射 F2-返航 ESC-退出")
    print("=" * 60)


    while running:
        """
            以固定的帧率（如60FPS）捕获用户输入、推进物理引擎的时间、更新 UI 画面，并将最新的位置数据实时分发给 Redis 和外部机箱。
            """
        raw_dt = clock.tick(CFG.FPS) / 1000.0
        # UI 游标闪烁逻辑：累加时间，每0.5秒切换一次光标的可见状态 (用于聊天输入框)
        renderer.cursor_timer += raw_dt
        if renderer.cursor_timer > 0.5:
            renderer.cursor_timer = 0.0
            renderer.cursor_vis = not renderer.cursor_vis

        # 第一阶段：事件处理（按键，鼠标和窗口操作）
        for event in pygame.event.get():
            # 处理窗口关闭按钮
            if event.type == pygame.QUIT:
                running = False
            # 处理鼠标滚轮缩放和鼠标点击拖拽场景
            elif event.type in (pygame.MOUSEWHEEL, pygame.MOUSEBUTTONDOWN):
                renderer.handle_event(event)
            elif event.type == pygame.VIDEORESIZE and not renderer.fullscreen:
                renderer._set_display_mode(size=(event.w, event.h))
                renderer._init_fonts()
            # 处理键盘按下一个键
            elif event.type == pygame.KEYDOWN:
                # 状态A：用户正在输入文字聊天指令
                if renderer.chat_active:
                    if event.key == pygame.K_RETURN:
                        if renderer.chat_input.strip():
                            text = renderer.chat_input.strip()
                            # 在屏幕上打印用户输入的内容
                            renderer.add_log("[USR]", text, "txt_h")
                            # 将文本送入环境，获取系统回复
                            resp = env.process_command(text)
                            # 在屏幕上打印系统回复
                            renderer.add_log("[CMD]", resp, "green")
                        # 清空输入框并退出聊天模式
                        renderer.chat_input = ""
                        renderer.chat_active = False
                    elif event.key == pygame.K_ESCAPE:
                        renderer.chat_input = ""
                        renderer.chat_active = False
                    elif event.key == pygame.K_BACKSPACE:
                        renderer.chat_input = renderer.chat_input[:-1]

                # --- 状态 B: 正常战术指令快捷键 ---
                else:
                    if event.key == pygame.K_ESCAPE: # 退出系统
                        running = False
                    elif event.key == pygame.K_RETURN and not started:
                        env._prepare_fresh_live_start()
                        started = True
                        renderer.add_log("[SYS]", f"任务开始({sim_speed}x)", "green")
                    elif event.key == pygame.K_r: # R键重置场景
                        if event.mod & pygame.KMOD_SHIFT:
                            env.seed = random.randint(0, 10000)
                        env.reset()
                        if redis_publisher:
                            try:
                                redis_publisher.maybe_publish(env, force=True)
                            except Exception as exc:
                                renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
                                redis_publisher = None
                        # 同步重置底层任务规划分配器的状态
                        if plan_exporter:
                            try:
                                plan_exporter.reset_assignments()
                                for tag, message, color in plan_exporter.maybe_publish(env, force=True):
                                    renderer.add_log(tag, message, color)
                            except Exception as exc:
                                renderer.add_log("[PLAN]", f"规划发布重置失败: {exc}", "red")
                        # 重置 UI 和时间状态
                        started, paused = False, False
                        renderer._elo = 0
                        if event.mod & pygame.KMOD_SHIFT:
                            renderer.add_log("[SYS]", f"已随机重置(seed={env.seed})", "amber")
                        else:
                            renderer.add_log("[SYS]", f"已按当前seed重置(seed={env.seed})", "amber")
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                        renderer.add_log("[SYS]", "暂停" if paused else "继续", "amber")
                    elif event.key == pygame.K_p:
                        renderer.show_poi = not renderer.show_poi
                        renderer.add_log("[SYS]", "预测线: " + ("显示" if renderer.show_poi else "隐藏"), "cyan")
                    elif event.key == pygame.K_t:
                        renderer.chat_active = True
                        renderer.chat_input = ""
                    elif event.key == pygame.K_F11:
                        renderer._set_display_mode(fullscreen=not renderer.fullscreen)
                        renderer._init_fonts()
                        renderer.add_log("[SYS]", "全屏" if renderer.fullscreen else "窗口模式", "cyan")
                    # 1/2/4/8/0 键控制时间流逝倍速 (20x极速常用于大模型快速推演)
                    elif event.key == pygame.K_1:
                        sim_speed = 1
                    elif event.key == pygame.K_2:
                        sim_speed = 2
                    elif event.key == pygame.K_4:
                        sim_speed = 4
                    elif event.key == pygame.K_8:
                        sim_speed = 8
                    elif event.key == pygame.K_0:
                        sim_speed = 20

                    elif event.key == pygame.K_F1: # F1下令全体发射
                        renderer.add_log("[CMD]", env.process_command("全体发射"), "green")
                    elif event.key == pygame.K_F2: # F2下令全体返航
                        renderer.add_log("[CMD]", env.process_command("全体返航"), "amber")
                    elif event.key == pygame.K_v: # V键对讲机模式：按下开始录音
                        if voice_engine and renderer.voice_state == 0:
                            renderer.voice_state = 1
                            voice_engine.start_recording()

            # 键盘松开事件
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_v and renderer.voice_state == 1: # 松开V键：停止录音并处理语音
                    renderer.voice_state = 2
                    if voice_engine:
                        voice_engine.stop_recording()
            # 纯文本输入事件 (用于捕获用户打字时的中文或英文字符)
            elif event.type == pygame.TEXTINPUT:
                if renderer.chat_active and len(renderer.chat_input) < 50:
                    renderer.chat_input += event.text

        # 第二阶段：物理世界步进计算 (Fixed-Timestep)
        if started and not paused and not env.done:
            time_accumulator += raw_dt * sim_speed
            while time_accumulator >= CFG.DT:
                env.step(CFG.DT)
                time_accumulator -= CFG.DT
                if env.done:
                    break
        # 如果处于没按回车开始的待机状态，且接入了外部真实雷达，依然要保持后台抽水(拉取数据)，防止数据积压
        elif source in ("udp", "redis", "auto", "fusion") and not env.done:
            env._pump_live_data()

        # 第三阶段：日志与外部交互同步
        # 1. 增量同步底层环境产生的日志到前端渲染器
        if renderer._elo > len(env.logs):
            renderer._elo = 0
        new_logs = env.logs[renderer._elo:]
        for log in new_logs:
            if isinstance(log, tuple):
                renderer.add_log(log[0], log[1], log[2])
            else:
                renderer.add_log("[ENV]", log, "blue")
        renderer._elo = len(env.logs) # 更新已读取日志的游标位置

        # 2. 处理大模型语音识别引擎返回的异步结果
        if voice_engine:
            vr = voice_engine.get_result()
            if vr is not None:
                if vr.startswith("[SYS]") or vr.startswith("[错误]"):
                    color = "green" if "加载完毕" in vr else ("red" if "错误" in vr else "amber")
                    renderer.add_log("[VOICE]", vr, color)
                    if renderer.voice_state == 2:
                        renderer.voice_state = 0
                else: # 成功识别出的用户指令
                    renderer.add_log("[VOICE]", vr, "pink")
                    # 将识别出的文字当成打字一样交给环境处理
                    renderer.add_log("[CMD]", env.process_command(vr), "green")
                    renderer.voice_state = 0

        # ==========================================
        # 第四阶段：数据向外发布
        # ==========================================
        # 将最新算出来的坐标发布给联调机箱或导调系统
        if redis_publisher:
            try:
                redis_publisher.maybe_publish(env)
            except Exception as exc:
                renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
                redis_publisher = None
        if udp_publisher:
            try:
                udp_publisher.maybe_publish(env)
            except Exception as exc:
                renderer.add_log("[UDP]", f"UDP发布失败: {exc}", "red")
                udp_publisher = None
        if plan_exporter:
            try:
                for tag, message, color in plan_exporter.maybe_publish(env):
                    renderer.add_log(tag, message, color)
            except Exception as exc:
                renderer.add_log("[PLAN]", f"规划发布失败: {exc}", "red")

        # ==========================================
        # 第五阶段：画面绘制
        # ==========================================
        # 结合目前的真实帧率(clock.get_fps)和倍速，画出这一帧
        renderer.render(sim_speed, clock.get_fps())

    # ==========================================
    # 退出后的善后工作 (Cleanup)
    # ==========================================
    # 当 running 变成 False (按了ESC或关了窗口)，跳出 while 循环，开始清理战场资源

    if voice_engine:
        voice_engine.shutdown() # 关掉录音麦克风线程
    if redis_publisher:
        try:
            redis_publisher.cleanup() # 向Redis发Delete命令，清空我们刚才造的虚假飞机坐标
        except Exception:
            pass
    if udp_publisher:
        udp_publisher.close()
    if plan_exporter:
        plan_exporter.close()
    if llm_dashboard_server:
        llm_dashboard_server.stop() # 关掉大模型网页看板的后台端口
    if getattr(env.feed, "close", None):
        try:
            env.feed.close() # 断开与真实雷达UDP的连接
        except Exception:
            pass
    pygame.quit() # 彻底销毁渲染窗口


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', default='menu', choices=['menu', 'demo'])
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--scene-km', type=float, default=10.0, help='场景尺度: 1/2/3/4/5/10 km')
    parser.add_argument('--source', default='auto', choices=['auto', 'redis', 'udp', 'fusion', 'demo'])
    parser.add_argument('--intercept-mode', default='hybrid', choices=['hybrid', 'hit', 'net', 'legacy-net'])
    parser.add_argument('--demo-case', default=None, choices=['net-single', 'barrier-single'])
    parser.add_argument('--redis-host', default='127.0.0.1')
    parser.add_argument('--redis-port', type=int, default=6379)
    parser.add_argument('--redis-db', type=int, default=0)
    parser.add_argument('--redis-password', default=None, help='输入Redis密码(可选)')
    parser.add_argument('--enemy-redis-format', default='auto', choices=['auto', 'flat', 'hash'], help='敌方Redis输入格式: auto/flat/hash')
    parser.add_argument('--friendly-return-source', default='udp', choices=['udp', 'redis', 'none'], help='己方回传来源: udp/redis/none')
    parser.add_argument('--geo-origin-lat', type=float, default=34.2663, help='固定参考原点纬度')
    parser.add_argument('--geo-origin-lon', type=float, default=108.9549, help='固定参考原点经度')
    parser.add_argument('--enemy-assoc', default='on', choices=['on', 'off'], help='敌方本地关联器: on/off')
    parser.add_argument('--enemy-assoc-max-distance', type=float, default=450.0, help='敌方本地关联的平面匹配门限(米)')
    parser.add_argument('--enemy-assoc-max-altitude', type=float, default=140.0, help='敌方本地关联的高度匹配门限(米)')
    parser.add_argument('--enemy-assoc-keep-sec', type=float, default=18.0, help='敌方本地轨迹保留时长(秒)')
    parser.add_argument('--enemy-hash-remap-mode', default='direct', choices=['direct', 'inbound'], help='敌方hash映射模式: direct=按原点直接投影, inbound=只显示向原点进攻的那一段')
    parser.add_argument('--enemy-hash-center-x-ratio', type=float, default=0.5, help='敌方hash入侵段映射的横向中心比例: 0=贴左, 0.5=居中, 1=贴右')
    parser.add_argument('--enemy-hash-lateral-scale', type=float, default=1.0, help='敌方hash入侵段映射的横向缩放倍数')
    parser.add_argument('--enemy-hash-range-scale', type=float, default=5.0, help='敌方hash入侵段映射的纵深推进倍率, 越大越容易从下方向上推进')
    parser.add_argument('--enemy-hash-start-range-m', type=float, default=0.0, help='敌方hash仅显示距原点最后N米的进攻段; 0=关闭该窗口')
    parser.add_argument('--enemy-hash-y-offset-m', type=float, default=0.0, help='敌方hash入侵段纵向平移(米): 负值整体下移, 正值整体上移')
    parser.add_argument('--enemy-hash-hide-outbound', default='off', choices=['on', 'off'], help='敌方hash是否在越过原点后隐藏: on/off')
    parser.add_argument('--enemy-flat-remap-mode', default='legacy', choices=['legacy', 'direct'], help='敌方flat上层映射模式: legacy=旧经纬度/旋转映射, direct=关闭上层映射直接轴对应')
    parser.add_argument('--enemy-flat-rotate-deg', type=float, default=135.0, help='敌方flat坐标的平面旋转角度(度), 用于把来袭方向对齐到我方方向')
    parser.add_argument('--enemy-flat-flip-x', default='off', choices=['on', 'off'], help='敌方flat坐标是否镜像X轴: on/off')
    parser.add_argument('--enemy-flat-flip-y', default='off', choices=['on', 'off'], help='敌方flat坐标是否镜像Y轴: on/off')
    parser.add_argument('--enemy-flat-scale', type=float, default=1.0, help='敌方flat坐标平面缩放倍数, 比例不一致时调大/调小')
    parser.add_argument('--enemy-flat-center-x-ratio', type=float, default=0.5, help='敌方flat坐标横向平移比例: 0=贴左, 0.5=居中, 1=贴右')
    parser.add_argument('--enemy-flat-center-y-ratio', type=float, default=0.2, help='敌方flat坐标纵向平移比例: 0=贴底, 0.2=抬进场内, 1=贴顶部')
    parser.add_argument('--radar-stale-sec', type=float, default=None, help='敌方轨迹超过该时间未更新则记为stale')
    parser.add_argument('--radar-lost-sec', type=float, default=None, help='敌方轨迹超过该时间未更新则记为lost')
    parser.add_argument('--target-search-sec', type=float, default=None, help='目标lost后沿最后航迹搜索时长')
    parser.add_argument('--publish-redis', action='store_true', help='演示时将当前同一仿真帧同步写入 Redis')
    parser.add_argument('--publish-interval', type=float, default=0.5, help='Redis同步周期(秒)，最小0.03')
    parser.add_argument('--publish-redis-mode', default='default', choices=['default', 'teacher-friendly', 'geo-hash'], help='Redis输出格式: 默认格式 / 老师兼容扁平键 / 经纬度Hash')
    parser.add_argument('--publish-redis-side', default='all', choices=['all', 'friendly', 'enemy'], help='Redis输出对象: all/friendly/enemy')
    parser.add_argument('--publish-redis-host', default=None, help='输出Redis主机，默认沿用 --redis-host')
    parser.add_argument('--publish-redis-port', type=int, default=None, help='输出Redis端口，默认沿用 --redis-port')
    parser.add_argument('--publish-redis-db', type=int, default=None, help='输出Redis库，默认沿用 --redis-db')
    parser.add_argument('--publish-redis-password', default=None, help='输出Redis密码(可选)')
    parser.add_argument('--friendly-start', type=int, default=1, help='己方写入 Redis 的起始编号')
    parser.add_argument('--enemy-start', type=int, default=101, help='敌方写入 Redis 的起始编号')
    parser.add_argument('--publish-udp', action='store_true', help='演示时将当前同一仿真帧同步写入 UDP')
    parser.add_argument('--publish-udp-mode', default='teacher', choices=['teacher', 'geo'], help='UDP输出格式: teacher/geo')
    parser.add_argument('--udp-out-host', default='127.0.0.1')
    parser.add_argument('--udp-out-port', type=int, default=9999)
    parser.add_argument('--udp-enemy-only', action='store_true', help='UDP只发送敌方')
    parser.add_argument('--udp-in-host', default='0.0.0.0')
    parser.add_argument('--udp-in-port', type=int, default=8020)
    parser.add_argument('--hangar-mode', default='multi', choices=['single', 'multi'])
    parser.add_argument('--demo-interference-enable', default='on', help='demo强干扰效果开关: on/off/1/0')
    parser.add_argument('--demo-interference-visible', default='on', help='demo干扰圆区可视化开关: on/off/1/0')
    parser.add_argument('--demo-scheme', type=int, default=0, choices=[0, 1, 2, 3], help='demo演示方案: 0=手动当前配置, 1=传统最近邻, 2=威胁协同, 3=强干扰失联')
    parser.add_argument('--llm-dashboard', action='store_true', help='启动单独网页显示LLM可解释决策链')
    parser.add_argument('--llm-dashboard-host', default='127.0.0.1', help='LLM决策链网页监听地址')
    parser.add_argument('--llm-dashboard-port', type=int, default=8765, help='LLM决策链网页端口')
    parser.add_argument('--llm-dashboard-open', action='store_true', help='启动后尝试自动打开浏览器')
    parser.add_argument('--test-wav', type=str, default=None, help='WAV文件路径，用于远程调试语音 (跳过麦克风)')
    parser.add_argument('--fullscreen', action='store_true')
    parser.add_argument('--ui-style', default='arc', choices=['arc', 'rect', 'omni'])
    args = parser.parse_args()

    if args.mode == 'menu':
        from ui.menu import run_with_menu
        run_with_menu()
    else:
        run_demo(
            seed=args.seed,
            test_wav=args.test_wav,
            scene_km=args.scene_km,
            source=args.source,
            redis_host=args.redis_host,
            redis_port=args.redis_port,
            redis_db=args.redis_db,
            redis_password=args.redis_password,
            enemy_redis_format=args.enemy_redis_format,
            friendly_return_source=args.friendly_return_source,
            geo_origin_lat=args.geo_origin_lat,
            geo_origin_lon=args.geo_origin_lon,
            enemy_assoc=args.enemy_assoc,
            enemy_assoc_max_distance=args.enemy_assoc_max_distance,
            enemy_assoc_max_altitude=args.enemy_assoc_max_altitude,
            enemy_assoc_keep_sec=args.enemy_assoc_keep_sec,
            enemy_hash_remap_mode=args.enemy_hash_remap_mode,
            enemy_hash_center_x_ratio=args.enemy_hash_center_x_ratio,
            enemy_hash_lateral_scale=args.enemy_hash_lateral_scale,
            enemy_hash_range_scale=args.enemy_hash_range_scale,
            enemy_hash_start_range_m=args.enemy_hash_start_range_m,
            enemy_hash_y_offset_m=args.enemy_hash_y_offset_m,
            enemy_hash_hide_outbound=args.enemy_hash_hide_outbound,
            enemy_flat_remap_mode=args.enemy_flat_remap_mode,
            enemy_flat_rotate_deg=args.enemy_flat_rotate_deg,
            enemy_flat_flip_x=args.enemy_flat_flip_x,
            enemy_flat_flip_y=args.enemy_flat_flip_y,
            enemy_flat_scale=args.enemy_flat_scale,
            enemy_flat_center_x_ratio=args.enemy_flat_center_x_ratio,
            enemy_flat_center_y_ratio=args.enemy_flat_center_y_ratio,
            radar_stale_sec=args.radar_stale_sec,
            radar_lost_sec=args.radar_lost_sec,
            target_search_sec=args.target_search_sec,
            intercept_mode=args.intercept_mode,
            demo_case=args.demo_case,
            fullscreen=args.fullscreen,
            ui_style=args.ui_style,
            publish_redis=args.publish_redis,
            publish_interval=args.publish_interval,
            friendly_start=args.friendly_start,
            enemy_start=args.enemy_start,
            publish_redis_mode=args.publish_redis_mode,
            publish_redis_side=args.publish_redis_side,
            publish_redis_host=args.publish_redis_host,
            publish_redis_port=args.publish_redis_port,
            publish_redis_db=args.publish_redis_db,
            publish_redis_password=args.publish_redis_password,
            publish_udp=args.publish_udp,
            publish_udp_mode=args.publish_udp_mode,
            udp_out_host=args.udp_out_host,
            udp_out_port=args.udp_out_port,
            udp_enemy_only=args.udp_enemy_only,
            udp_in_host=args.udp_in_host,
            udp_in_port=args.udp_in_port,
            hangar_mode=args.hangar_mode,
            demo_interference_enable=args.demo_interference_enable,
            demo_interference_visible=args.demo_interference_visible,
            demo_scheme=args.demo_scheme,
            llm_dashboard=args.llm_dashboard,
            llm_dashboard_host=args.llm_dashboard_host,
            llm_dashboard_port=args.llm_dashboard_port,
            llm_dashboard_open=args.llm_dashboard_open,
        )


if __name__ == "__main__":
    main()
