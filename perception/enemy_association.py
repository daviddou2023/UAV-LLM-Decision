"""
    在实际外场联调中，雷达/外部设备传来的敌方无人机 ID 经常会“闪烁”或“换号”（比如一架飞机由于雷达盲区短暂消失，再次出现时雷达给了一个新 ID）。
    如果直接把这种数据喂给系统，会导致底层分配算法（decision/cooperation.py）疯狂地取消任务并重新指派。
    这个文件就是通过位置、速度等物理特征，把这些“换号”的目标强行关联回原来的身份，从而保证系统的稳定性。

敌方本地关联器:
1. 当上游不给稳定编号时, 在本地为敌方轨迹生成稳定ID
2. 根据位置/高度/航向/速度做轻量关联
3. 只作用于敌方输入层, 不改变后续任务分配与UI逻辑

    1. 上游：当从 Redis 或 UDP 抓取到外部传来的雷达快照时，如果在run_fusion_custom.sh中配置了 ENEMY_ASSOC="on"，perception/radar_feed.py，先将其传入 LocalEnemyAssociator.associate() 进行洗牌
    清洗完之后外部雷达的 ID（如 uav_1, uav_2）会被隐藏在 source_external_id 中，而吐出来的数据会被强制赋予内部稳定 ID（如 local_enemy_001）
    2. 下游：底层分配器 InterceptionAssigner 不会因为雷达杂波导致目标 ID 闪变，提高程序抗干扰能力
"""
import math
import time


def _coerce_timestamp(value, fallback):
    """时间戳清洗函数，处理异常或不同单位（毫秒/秒）的时间戳输入，防止系统时间错乱"""
    if value is None:
        return fallback
    try:
        stamp = float(value)
    except (TypeError, ValueError):
        return fallback
    if stamp > 1e12:
        stamp /= 1000.0
    if stamp < 1e9:
        return fallback
    return stamp


def _angle_diff_deg(a, b):
    """计算两个航向角之间的最小夹角差值（处理 360 度循环问题）"""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


class LocalEnemyAssociator:
    def __init__(
        self,
        enabled=True,
        max_distance_m=450.0,
        max_altitude_diff_m=140.0,
        keep_sec=18.0,
        max_heading_diff_deg=95.0,
    ):
        self.enabled = bool(enabled)
        self.max_distance_m = max(50.0, float(max_distance_m))
        self.max_altitude_diff_m = max(10.0, float(max_altitude_diff_m))
        self.keep_sec = max(1.0, float(keep_sec))
        self.max_heading_diff_deg = max(15.0, float(max_heading_diff_deg))
        self.track_table = {}
        self.next_local_index = 1

    # （核心入口） 接收一批新的雷达观测数据，过滤、匹配并返回带有稳定 local_id 的观测列表
    def associate(self, observations):
        """为一批敌方观测分配稳定本地 ID。

        设备1输入的目标 ID 可能闪烁；这里先清理过期轨迹，再按空间距离、
        高度差和航向差把新观测匹配到历史轨迹。输出会被雷达归一化层继续
        写成稳定 external_id，避免设备2重复创建敌方目标。
        """
        # 如果未开启关联功能，原样返回上游数据
        if not self.enabled:
            return observations

        now = time.time()
        # 踢掉那些长时间没更新的“死航迹”
        self._prune_dead_tracks(now)
        # 对传进来的观测点按 Y 坐标（距离我方防线的远近）进行排序确定处理的优先级
        candidates = sorted(observations, key=lambda item: float(item.get("y", 0.0)), reverse=True)
        if not candidates:
            return []

        # 核心分配：计算雷达点与本地历史轨迹的最佳配对关系，返回格式{观测点索引: 本地轨迹ID}
        assignments = self._compute_assignments(candidates, now)
        stabilized = []
        for obs_index, obs in enumerate(candidates):
            local_id = assignments.get(obs_index)
            # 如果这个雷达点没有匹配到任何历史轨迹，说明是新出现的敌人
            if local_id is None:
                # 颁发一个新的稳定ID
                local_id = self._new_local_id() 
            # 将该雷达点的数据与稳定ID绑定，并更新到本地缓存中
            stabilized.append(self._bind_observation(obs, local_id, now))
        return stabilized

    # （核心逻辑） 计算新观测点与历史轨迹的最佳匹配对（使用贪心算法匹配最低得分/最短距离）
    def _compute_assignments(self, observations, now):
        """
                计算雷达观测点与历史轨迹的最佳分配（贪心分配法）。
                """
        track_ids = list(self.track_table.keys())
        scored_pairs = []
        # 暴力遍历：计算所有 [新观测点] 和所有 [历史轨迹] 之间的得分
        for obs_index, obs in enumerate(observations):
            for local_id in track_ids:
                score = self._match_score(obs, self.track_table[local_id], now)
                # 只有通过了硬性门限（未返回 None）的组合，才有资格进入候选名单
                if score is not None:
                    scored_pairs.append((score, obs_index, local_id))

        # 全局排序：将所有组合按得分从小到大排序（最接近、最完美的匹配排在最前面）
        scored_pairs.sort(key=lambda item: item[0])
        assignments = {}
        used_tracks = set()
        # 贪心锁定：从最优匹配开始，逐一绑定
        for _, obs_index, local_id in scored_pairs:
            # 如果这个观测点已经分配了，或者这个历史轨迹已经被抢占了，跳过
            if obs_index in assignments or local_id in used_tracks:
                continue
            # 绑定它们
            assignments[obs_index] = local_id
            used_tracks.add(local_id)
        return assignments

    # （核心算法） 综合评分函数。计算某个雷达点与历史航迹的相似度“代价分”（得分越低越匹配）
    def _match_score(self, obs, track_state, now):
        """
                代价函数/打分机制：评估一个新的雷达观测点 (obs) 是否属于某个已知的历史航迹 (track_state)。
                返回值越小，说明相似度越高；返回 None 说明差距过大，直接否决。
                """
        # 根据历史速度和航向，推算该目标在当前时间戳理论上应该飞到了哪里
        pred_x, pred_y, pred_z = self._predict_track(track_state, now)
        # 计算物理平面距离误差
        dx = float(obs.get("x", 0.0)) - pred_x
        dy = float(obs.get("y", 0.0)) - pred_y
        planar = math.hypot(dx, dy)
        # 如果距离跳变超过了容忍门限 (如 450米)，判定不是同一架飞机，否决
        if planar > self.max_distance_m:
            return None

        alt_diff = abs(float(obs.get("z", 0.0)) - pred_z)
        if alt_diff > self.max_altitude_diff_m:
            return None

        # 计算航向角惩罚（飞机不可能瞬间掉头，如果航向突变过大则否决）
        heading = obs.get("heading")
        prev_heading = track_state.get("heading")
        heading_penalty = 0.0
        if heading is not None and prev_heading is not None:
            diff = _angle_diff_deg(heading, prev_heading)
            if diff > self.max_heading_diff_deg:
                return None
            # 放大航向误差在总分中的比重
            heading_penalty = diff * 1.2 

        # 计算速度突变惩罚
        speed_penalty = 0.0
        if obs.get("speed") is not None and track_state.get("speed") is not None:
            speed_penalty = abs(float(obs.get("speed", 0.0)) - float(track_state.get("speed", 0.0))) * 6.0

        # 计算时间老化惩罚（如果一个轨迹消失了很久才出现，匹配的置信度降低）
        age_penalty = max(0.0, now - track_state.get("last_seen", now)) * 20.0
        # 综合得分 = 平面距离 + 高度差(权重大) + 航向惩罚 + 速度惩罚 + 时间惩罚
        return planar + alt_diff * 1.8 + heading_penalty + speed_penalty + age_penalty

    # 航迹推演。利用历史目标最后出现的位置、速度和航向，推测它现在“应该”在哪个坐标
    def _predict_track(self, track_state, now):
        dt = max(0.0, now - track_state.get("last_seen", now))
        heading = track_state.get("heading")
        speed = float(track_state.get("speed", 0.0) or 0.0)
        vz = float(track_state.get("vz", 0.0) or 0.0)
        x = float(track_state.get("x", 0.0))
        y = float(track_state.get("y", 0.0))
        z = float(track_state.get("z", 0.0))
        if heading is None:
            return x, y, z
        hr = math.radians(float(heading))
        return (
            x + math.cos(hr) * speed * dt,
            y + math.sin(hr) * speed * dt,
            z + vz * dt,
        )

    # 将成功匹配的雷达数据与本地稳定 ID 绑定，并更新本地维护的轨迹状态表 (track_table)
    def _bind_observation(self, obs, local_id, now):
        source_external_id = str(obs.get("external_id", local_id))
        stabilized = dict(obs)
        stabilized["source_external_id"] = source_external_id
        stabilized["external_id"] = local_id
        raw = dict(stabilized.get("raw", {}))
        raw["source_external_id"] = source_external_id
        raw["assoc_track_id"] = local_id
        stabilized["raw"] = raw

        stamp = _coerce_timestamp(stabilized.get("stamp"), now)
        self.track_table[local_id] = {
            "x": float(stabilized.get("x", 0.0)),
            "y": float(stabilized.get("y", 0.0)),
            "z": float(stabilized.get("z", 0.0)),
            "heading": stabilized.get("heading"),
            "speed": float(stabilized.get("speed", 0.0) or 0.0),
            "vz": float(stabilized.get("vz", 0.0) or 0.0),
            # 本地关联保留时长按“本机收到观测的时间”算，
            # 不直接依赖上游时钟，避免多机时钟偏差导致误删轨迹。
            "last_seen": now,
            "source_stamp": stamp,
            "source_external_id": source_external_id,
        }
        return stabilized

    # 为系统从未见过的新目标生成一个递增的本地稳定编号（如 local_enemy_001）
    def _new_local_id(self):
        local_id = f"local_enemy_{self.next_local_index:03d}"
        self.next_local_index += 1
        return local_id

    # 定期清理缓存。把超过指定时间（keep_sec）没有再出现过的历史轨迹从内存中删掉
    def _prune_dead_tracks(self, now):
        expired = [
            local_id
            for local_id, state in self.track_table.items()
            if (now - state.get("last_seen", now)) > self.keep_sec
        ]
        for local_id in expired:
            self.track_table.pop(local_id, None)
