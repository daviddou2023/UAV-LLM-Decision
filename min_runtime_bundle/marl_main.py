"""
MARL主模块 - 空域拦截系统 v8.0
1. 优先接入老师的实时 Redis 数据
2. 路径规划升级为 3D，显示仍保持 2D 投影
3. 场景尺寸支持 1/2/3/4/5/10km 动态切换
4. 增加雷达丢失、识别不稳、目标失联后的保守重规划
"""
import argparse
import math
import random
import re
import time

from marl_common import (
    CFG,
    EState,
    EType,
    IRole,
    IState,
    angle_diff,
    create_enemy,
    create_interceptor,
    dist2d,
    dist3d,
    move_entity,
)
from marl_cooperation import InterceptionAssigner
from marl_data import TeacherDataFeed
from marl_deconfliction import DeconflictionController
from marl_llm_kit import BattlefieldAnalyst
from marl_redis_export import (
    RedisNodeWriter,
    build_payload,
    enemy_rows,
    friendly_rows,
    planned_node_nums,
    stale_keys,
)

try:
    from marl_voice import VoiceEngine
    _HAS_VOICE = True
except ImportError:
    _HAS_VOICE = False


def _etype_from_track(track):
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
    return EType.NORMAL


class ProNav:
    @staticmethod
    def command(intr, enemy):
        dx = enemy['x'] - intr['x']
        dy = enemy['y'] - intr['y']
        r = math.sqrt(dx * dx + dy * dy)
        if r < 1.0:
            return 0.0

        er = math.radians(enemy['heading'])
        ir = math.radians(intr['heading'])
        rvx = enemy['speed'] * math.cos(er) - intr['speed'] * math.cos(ir)
        rvy = enemy['speed'] * math.sin(er) - intr['speed'] * math.sin(ir)
        vc = -(dx * rvx + dy * rvy) / r
        if vc < 0.1:
            vc = 0.1
        los_rate = (dx * rvy - dy * rvx) / (r * r)

        gain = CFG.PRONAV_GAIN
        if enemy['type'] in (EType.LOITER, EType.DASH) and r < max(600.0, 0.15 * CFG.INTERCEPT_FAIL_LINE):
            gain = 8.0
        w = math.degrees(gain * vc * los_rate)
        return max(-CFG.INTERCEPTOR_MAX_ANG, min(CFG.INTERCEPTOR_MAX_ANG, w))

    @staticmethod
    def guide_point(intr, pt):
        dx = pt[0] - intr['x']
        dy = pt[1] - intr['y']
        desired = math.degrees(math.atan2(dy, dx))
        d = angle_diff(desired, intr['heading'])
        return max(-CFG.INTERCEPTOR_MAX_ANG, min(CFG.INTERCEPTOR_MAX_ANG, d * 0.5))


def _planned_demo_node_nums(env, friendly_start, enemy_start):
    return planned_node_nums(
        interceptor_count=len(env.interceptors),
        total_enemy_count=env.stats.get("total_enemies", 0),
        live_enemy_count=len(env.enemies),
        friendly_start=friendly_start,
        enemy_start=enemy_start,
    )


def _enabled_flag(value, default=True):
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
    def __init__(self, host="127.0.0.1", port=6379, db=0, publish_interval=0.5,
                 friendly_start=1, enemy_start=101, cleanup_node_nums=None):
        self.writer = RedisNodeWriter(host=host, port=port, db=db)
        self.publish_interval = max(0.03, float(publish_interval))
        self.friendly_start = int(friendly_start)
        self.enemy_start = int(enemy_start)
        self.frame_num = 0
        self.last_publish_at = 0.0
        self.published_nodes = set()
        self.cleanup_node_nums = set(cleanup_node_nums or set())
        self.writer.cleanup_legacy_keys()
        self.cleanup()

    def cleanup(self):
        self.writer.cleanup_node_nums(self.cleanup_node_nums | self.published_nodes)
        self.published_nodes = set()

    def maybe_publish(self, env, force=False):
        now = time.time()
        if not force and (now - self.last_publish_at) < self.publish_interval:
            return False
        self.frame_num += 1
        numbered_friendlies = friendly_rows(env.interceptors, self.friendly_start)
        numbered_enemies = enemy_rows(env.enemies, self.enemy_start)
        payload, active_nodes = build_payload(numbered_friendlies, numbered_enemies, self.frame_num, now)
        self.writer.mset(payload)
        self.writer.delete(stale_keys(self.published_nodes, active_nodes))
        self.published_nodes = active_nodes
        self.last_publish_at = now
        return True


class WaveManager:
    def __init__(self, rng, waves=None):
        self.rng = rng
        self.waves = [dict(wave) for wave in (waves or [
            {'time': 0, 'count': 3},
            {'time': CFG.WAVE_INTERVAL, 'count': 4},
        ])]
        self.idx = 0
        self.nxt_id = 0

    def update(self, t):
        new = []
        while self.idx < len(self.waves) and t >= self.waves[self.idx]['time']:
            wave = self.waves[self.idx]
            for _ in range(wave['count']):
                x = self.rng.uniform(CFG.AREA_WIDTH * 0.1, CFG.AREA_WIDTH * 0.9)
                speed = CFG.ENEMY_SPEED + self.rng.uniform(-CFG.ENEMY_SPEED_VAR, CFG.ENEMY_SPEED_VAR)
                heading = CFG.ENEMY_HDG_BASE + self.rng.uniform(-CFG.ENEMY_HDG_VAR, CFG.ENEMY_HDG_VAR)
                enemy = create_enemy(self.nxt_id, x, speed, heading, t, self.rng.random())
                enemy['source'] = 'demo'
                new.append(enemy)
                self.nxt_id += 1
            self.idx += 1
        return new

    @property
    def total(self):
        return sum(w['count'] for w in self.waves)


class InterceptionEnvironment:
    def __init__(self, seed=42, scene_km=10.0, source="auto",
                 redis_host="192.166.51.21", redis_port=6379, redis_db=0,
                 intercept_mode="hybrid", demo_case=None,
                 demo_interference_enable=True, demo_interference_visible=True,
                 demo_scheme=0):
        self.seed = seed
        self.source = source
        self.redis_host = redis_host
        self.redis_port = redis_port
        self.redis_db = redis_db
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
        self.analyst = BattlefieldAnalyst()
        self.feed = TeacherDataFeed(host=redis_host, port=redis_port, db=redis_db) if source in ("auto", "redis") else None
        self.scene_revision = 0
        self.configure_scene(scene_km, reset=False)
        self.reset()

    def configure_scene(self, scene_km, reset=True):
        CFG.apply_scene_scale(scene_km)
        self.scene_revision += 1
        if reset and hasattr(self, "logs"):
            self.reset()

    def reset(self):
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
        self.interceptors = [create_interceptor(i) for i in range(CFG.NUM_INTERCEPTORS)]
        for it in self.interceptors:
            it['climb_rate'] = CFG.INTERCEPTOR_CLIMB_RATE

        self.enemies = []
        self.wave_mgr = WaveManager(self.rng, waves=self._demo_showcase_wave_plan())
        self.assigner = InterceptionAssigner()
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
        ]
        for msg in self.analyst.drain_status_events():
            self.logs.append(f"[LLM] {msg}")
        if self.feed:
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
        self.last_enemy_presence_time = 0.0
        self.enemy_track_flags = {}
        self.deconflict_cooldown = {}
        self.net_team_assignments = {}
        self.net_capture_states = {}
        self.barrier_team_assignments = {}
        self.barrier_states = {}
        self.deconfliction = DeconflictionController(self)
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
                    "边界: 地面站LLM无法控制失联无人机",
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
        token = str(token or "").strip()
        if not token:
            return None
        if token.isdigit():
            return int(token)
        digits = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
                  "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
        if token == "十":
            return 10
        if "十" in token:
            left, right = token.split("十", 1)
            tens = digits.get(left, 1 if left == "" else None)
            ones = digits.get(right, 0 if right == "" else None)
            if tens is not None and ones is not None:
                return tens * 10 + ones
        if len(token) == 1 and token in digits:
            return digits[token]
        return None

    def _parse_llm_task_constraints_command(self, text):
        text = str(text or "").strip()
        lower = text.lower()
        parsed = {}
        if any(word in text for word in ("恢复默认约束", "清除任务约束", "取消LLM约束", "取消llm约束", "默认分配")):
            parsed['clear_all'] = True
            return parsed
        if any(word in text for word in ("取消区域优先", "全域优先", "不区分左右", "取消左翼优先", "取消右翼优先", "取消中路优先")):
            parsed['preferred_sector'] = None
        if any(word in text for word in ("取消出动上限", "不限出动", "取消兵力上限", "取消数量限制")):
            parsed['max_active_count'] = 0
        if any(word in text for word in ("取消高速优先", "取消突防优先", "恢复默认优先级")):
            parsed['target_priority'] = None
        if any(
            word in text
            for word in (
                "重新分配任务", "任务重新分配", "任务重分配", "重新分配", "重分配",
                "重新调度", "重调度", "任务重排", "重新规划任务", "重新部署任务",
                "对任务进行重新分配",
            )
        ):
            parsed['force_reassign'] = True
        if any(word in text for word in ("取消保留", "解除保留", "不保留", "清空保留")):
            parsed['reserve_count'] = 0
        reserve_match = re.search(
            r"(?:至少)?(?:保留|预留|留出|留下|留)\s*([0-9一二两三四五六七八九十]+)\s*(?:架|个)?(?:无人机|拦截机|飞机|机)?",
            text,
        )
        if reserve_match:
            count = self._parse_llm_number(reserve_match.group(1))
            if count is not None:
                parsed['reserve_count'] = max(0, min(int(CFG.NUM_INTERCEPTORS), count))
        max_active_match = re.search(
            r"(?:最多|至多|上限|限制|只允许|只可|最多只)?(?:出动|派出|发射)\s*([0-9一二两三四五六七八九十]+)\s*(?:架|个)?(?:无人机|拦截机|飞机|机)?",
            text,
        )
        if max_active_match:
            count = self._parse_llm_number(max_active_match.group(1))
            if count is not None:
                parsed['max_active_count'] = max(0, min(int(CFG.NUM_INTERCEPTORS), count))
        if (
            ("高速" in text or "速度快" in text or "速度最快" in text or "fast" in lower)
            and any(word in text for word in ("优先", "先拦", "先打", "先处理", "重点"))
        ):
            parsed['target_priority'] = "speed"
        if any(word in text for word in ("左翼", "左侧", "左边")) and any(
            word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
        ):
            parsed['preferred_sector'] = "left"
        elif any(word in text for word in ("右翼", "右侧", "右边")) and any(
            word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
        ):
            parsed['preferred_sector'] = "right"
        elif any(word in text for word in ("中路", "中间", "中央")) and any(
            word in text for word in ("优先", "先处理", "先拦", "先打", "重点")
        ):
            parsed['preferred_sector'] = "center"
        if any(word in text for word in ("突防线", "接近突防", "靠近突防", "临近突防")) and any(
            word in text for word in ("优先", "先处理", "先拦", "先打", "重点", "附近")
        ):
            parsed['target_priority'] = "breach"
        if "干扰" in text and any(
            word in text
            for word in ("避开", "绕开", "绕避", "不要进", "不要进入", "不要继续派机硬闯", "不要硬闯", "禁入", "避让")
        ):
            parsed['avoid_jam'] = True
        if "取消绕避干扰" in text or "取消干扰禁入" in text:
            parsed['avoid_jam'] = False
        return parsed if parsed else None

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
        it['net_slot'] = None
        it['barrier_slot'] = None
        it['barrier_center'] = None
        it['task_reserved'] = True
        it['mission_label'] = "LLM保留"
        it['target_label'] = "待命"
        clear_local = getattr(self, "_clear_local_motion_state", None)
        if clear_local:
            clear_local(it)
        reset_local_state = getattr(getattr(self, "deconfliction", None), "reset_local_state", None)
        if reset_local_state:
            reset_local_state(it['id'])

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
        if priority == "speed":
            intent_parts.append("优先高速目标")
        elif priority == "breach":
            intent_parts.append("优先突防线附近目标")
        if 'preferred_sector' in parsed:
            if preferred_sector in ("left", "right", "center"):
                intent_parts.append(f"优先{self._sector_label_text(preferred_sector)}目标")
            else:
                intent_parts.append("取消区域优先")
        if 'reserve_count' in parsed:
            intent_parts.append(f"保留{reserve_count}架待命")
        if 'max_active_count' in parsed:
            if max_active_count > 0:
                intent_parts.append(f"最多出动{max_active_count}架")
            else:
                intent_parts.append("取消出动上限")
        if 'avoid_jam' in parsed:
            intent_parts.append("绕避干扰区" if avoid_jam else "取消干扰禁入")
        if parsed.get('force_reassign'):
            intent_parts.append("任务重分配")
        intent = "，".join(intent_parts) or "更新任务约束"

        self.logs.append(("[LLM]", f"意图解析: {intent}", "pink"))
        self.logs.append(("[LLM]", f"约束生成: {self._constraint_summary_text(constraints)}", "amber"))
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
            f"保留{locked}架机巢待命，其余无人机按{self._priority_label_text(priority if priority != 'none' else None)}"
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

    def _set_interceptor_count(self, count):
        count = max(1, int(count))
        if CFG.NUM_INTERCEPTORS == count:
            return False
        CFG.NUM_INTERCEPTORS = count
        CFG.apply_scene_scale(CFG.SCENE_KM)
        self.scene_revision += 1
        return True

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

    def _demo_showcase_wave_plan(self):
        if not self.demo_showcase_active:
            return None
        return [{'time': 0.0, 'count': 20}]

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
                'cy': CFG.INTERCEPT_FAIL_LINE * 0.59,
                'radius': CFG.INTERCEPT_FAIL_LINE * 0.09,
            },
        ]

    def _demo_interference_capacity(self):
        return max(1, int(CFG.NUM_INTERCEPTORS), len(getattr(self, "interceptors", [])))

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
            it['local_avoid_mode'] = ""
            it['local_hold_reason'] = ""

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
        return bool(it) and not it.get('jammed_by_interference')

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
            reset_local_state = getattr(self.deconfliction, 'reset_local_state', None)
            if reset_local_state:
                reset_local_state(it['id'])
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

    def _select_engagement_mode(self, enemy, planned_barrier_targets=0):
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
        free_pool = sum(
            1 for it in self.interceptors
            if it['state'] in (IState.STANDBY, IState.RETURNING) and it['target_id'] is None
            and not it.get('jammed_by_interference')
            and not it.get('task_reserved')
        )
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
            mode = self._select_engagement_mode(enemy, planned_barrier_targets)
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
            and not it.get('jammed_by_interference')
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
            chosen['launch_time'] = self.time + 0.2 * (chosen['id'] % 4)
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
        self.time += dt
        self.step_count += 1

        live_snapshot = self._sync_teacher_data()
        if not self.demo_mode:
            self._age_live_tracks()
        else:
            self._spawn_demo_waves()
        self._update_llm_reserved_pool()

        active_enemies = [e for e in self.enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        if active_enemies:
            self.last_enemy_presence_time = self.time

        self.analyst.update_situation(self.time, active_enemies, self.interceptors)
        ai_msg = self.analyst.get_analysis_log()
        if ai_msg:
            self.logs.append(("[AI情报]", ai_msg, "eloiter"))
        chat_msg = self.analyst.get_chat_reply()
        if chat_msg:
            self.logs.append(("[副官]", chat_msg, "pink"))
        for msg in self.analyst.drain_status_events():
            self.logs.append(f"[LLM] {msg}")

        self._update_enemy_detection()
        self._maybe_auto_llm_interference_replan()

        engageable_enemies = [enemy for enemy in active_enemies if enemy.get('detected')]
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

        self._update_launch_states()
        self._update_demo_interference()
        self._maybe_auto_llm_interference_replan()
        self._move_interceptors(dt)
        self._move_enemies(dt)
        if self.intercept_mode in ("hit", "hybrid"):
            self._check_intercepts()
        if self.intercept_mode in ("net", "hybrid"):
            self._check_barrier_capture()
        if self.intercept_mode == "legacy-net":
            self._check_net_capture()
        self._check_penetration()
        self._consume_fuel(dt)
        self._update_mission_labels()
        self._check_done()

    def _sync_teacher_data(self):
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
        if enemies or friendlies:
            self.last_live_seen_time = self.time
            if not self.has_live_data:
                self.has_live_data = True
                if self.source == "auto":
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

    def _spawn_demo_waves(self):
        new_enemies = self.wave_mgr.update(self.time)
        if not new_enemies:
            return
        self.enemies.extend(new_enemies)
        self.stats['waves_done'] = self.wave_mgr.idx
        self.logs.append(f"[WAVE-{self.wave_mgr.idx}] {len(new_enemies)} 架回放目标进入场景")

    def _upsert_live_enemies(self, tracks):
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
        enemy = {
            'id': self.next_enemy_id,
            'x': track['x'],
            'y': track['y'],
            'z': z_value,
            'vz': track.get('vz', 0.0),
            'target_z': z_value,
            'heading': track.get('heading', 90.0),
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
            'last_update': self.time,
            'classification_confidence': track.get('classification_confidence', 1.0),
            'source': 'teacher',
            'status_text': str(track.get('status', '')),
            'frame': track.get('frame'),
            'age': track.get('age', 0.0),
            'climb_rate': CFG.ENEMY_CLIMB_RATE,
            'z_cap': CFG.ENEMY_MAX_ALT,
        }
        self.next_enemy_id += 1
        return enemy

    def _apply_track_to_enemy(self, enemy, track):
        prev_lost = enemy.get('lost', False)
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
        enemy['last_update'] = self.time
        enemy['frame'] = track.get('frame')
        enemy['age'] = track.get('age', 0.0)
        enemy['status_text'] = str(track.get('status', enemy.get('status_text', '')))
        enemy['type'] = _etype_from_track(track)
        if enemy['state'] in (EState.DESTROYED, EState.PENETRATED):
            return
        enemy['state'] = EState.APPROACHING
        if prev_lost and not enemy['lost']:
            self.logs.append(f"[REACQ] F-{enemy['id']+1} 重获，按新雷达点刷新航迹")

    def _sync_live_friendlies(self, tracks):
        for track in tracks:
            idx = self._map_uav_track(track['external_id'])
            it = self.interceptors[idx]
            it['reported_x'] = track['x']
            it['reported_y'] = track['y']
            it['reported_z'] = self._cap_interceptor_altitude(it, track.get('z', 0.0))
            it['reported_speed'] = track.get('speed', it.get('speed', 0.0))
            it['reported_heading'] = track.get('heading', it.get('heading', 270.0))
            it['reported_at'] = self.time

            dx = track['x'] - it['x']
            dy = track['y'] - it['y']
            dz = track.get('z', 0.0) - it.get('z', 0.0)
            corr_xy = math.hypot(dx, dy)
            if corr_xy <= CFG.TELEMETRY_MAX_CORRECTION:
                it['x'] += dx * CFG.TELEMETRY_BLEND
                it['y'] += dy * CFG.TELEMETRY_BLEND
                it['z'] += dz * CFG.TELEMETRY_BLEND
                it['z'] = self._cap_interceptor_altitude(it, it['z'])

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
                continue
            it['speed'] = CFG.INTERCEPTOR_SPEED
            if self.intercept_mode in ("net", "legacy-net") or it.get('net_slot') is not None or it.get('barrier_slot') is not None:
                it['state'] = IState.INTERCEPTING
            else:
                it['state'] = IState.FOLLOWING if it['role'] == IRole.FOLLOWER else IState.INTERCEPTING
            target = self._get_enemy(it['target_id'])
            if target:
                self._update_route_plan(it, target, "起飞后进入地面站规划航线")

    def _move_interceptors(self, dt):
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

    def _guide_primary(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            self._set_return(it, "目标已不存在，返航")
            return

        if getattr(self, "demo_strategy_mode", "cooperative") == "baseline":
            self._guide_baseline_primary(it, enemy, dt)
            return

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
            cmd_pt = self._command_point(it, ghost)
            w = ProNav.guide_point(it, cmd_pt)
        else:
            it['search_until'] = 0.0
            if dist2d(it, enemy) <= CFG.TERMINAL_GUIDE_RANGE:
                plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="terminal")}
                self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                cmd_pt = self._command_point(it, plan_target)
                it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                w = ProNav.command(it, enemy)
            else:
                plan_target = {'x': enemy['x'], 'y': enemy['y'], 'z': self._friendly_altitude(it, enemy, mission="hit", phase="cruise")}
                self._update_route_plan(it, plan_target, "闭环追踪当前雷达点")
                cmd_pt = self._command_point(it, plan_target)
                it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="cruise")
                w = ProNav.guide_point(it, cmd_pt)

        it['heading'] = (it['heading'] + w * dt) % 360
        move_entity(it, dt)
        it['flight_time'] += dt

    def _guide_baseline_primary(self, it, enemy, dt):
        it['search_until'] = 0.0
        target_z = self._friendly_altitude(it, enemy, mission="hit", phase="cruise")
        current_point = (enemy['x'], enemy['y'], target_z)
        it['target_z'] = target_z
        it['poi'] = current_point
        it['path_plan'] = [(it['x'], it['y'], it.get('z', 0.0)), current_point]
        it['path_reason'] = "方案1传统基线: 最近邻直追当前点"
        it['speed'] = min(CFG.INTERCEPTOR_SPEED * 0.86, max(14.0, enemy.get('speed', CFG.ENEMY_SPEED) + 1.0))
        w = ProNav.guide_point(it, current_point)
        it['heading'] = (it['heading'] + w * dt) % 360
        move_entity(it, dt)
        it['flight_time'] += dt

    def _guide_follower(self, it, dt):
        enemy = self._get_enemy(it['target_id'])
        if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
            self._set_return(it, "随动目标消失，返航")
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
                it['speed'] = self._desired_interceptor_speed(enemy, mission="hit", phase="terminal")
                w = ProNav.guide_point(it, (fx, fy, pz))
                it['heading'] = (it['heading'] + w * dt) % 360
                guided = True
        if not guided:
            self._guide_primary(it, dt)
            return

        move_entity(it, dt)
        it['flight_time'] += dt

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
        return_plan, _ = self.deconfliction.apply_barrier_detours(
            it,
            [(it['x'], it['y'], it.get('z', 0.0)), (target_x, target_y, target_z)],
        )
        cmd_pt = (target_x, target_y, target_z)
        for point in return_plan[1:]:
            if dist2d(it, {'x': point[0], 'y': point[1]}) > max(35.0, CFG.FORMATION_SPACING * 0.25):
                cmd_pt = point
                break
        w = ProNav.guide_point(it, cmd_pt)
        it['heading'] = (it['heading'] + w * dt) % 360
        move_entity(it, dt)
        it['flight_time'] += dt

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
        if self.time >= CFG.TIME_LIMIT:
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
        side_order = (1.0, -1.0) if it['id'] % 2 == 0 else (-1.0, 1.0)
        candidates = []
        for side in side_order:
            cx = max(40.0, min(CFG.AREA_WIDTH - 40.0, zone['cx'] + px * clearance * side))
            cy = max(40.0, min(CFG.OUR_BASE_LINE - 40.0, zone['cy'] + py * clearance * side))
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
        lane = ((it['id'] % 4) - 1.5) * CFG.FORMATION_SPACING * 0.35
        mid = (
            (it['x'] + poi[0]) * 0.5,
            max(min((it['y'] + poi[1]) * 0.5, CFG.INTERCEPT_FAIL_LINE + 150), target['y']),
            self._cap_interceptor_altitude(it, max(it.get('z', 0.0), poi[2]) * 0.5),
        )
        mid = self._lateral_shift((it['x'], it['y'], it.get('z', 0.0)), mid, lane)
        plan = [
            (it['x'], it['y'], it.get('z', 0.0)),
            mid,
            poi,
        ]
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
        return (CFG.HANGAR_POSITIONS[0], CFG.OUR_BASE_LINE, 0.0)

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
        barrier_y = max(
            CFG.DETECTION_LINE + half_depth,
            min(CFG.INTERCEPT_FAIL_LINE - half_depth - 40.0, barrier_y),
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
        launch_pt = {'x': CFG.HANGAR_POSITIONS[0], 'y': CFG.INTERCEPT_FAIL_LINE + 200.0}
        team_travel = 0.0
        for slot_idx in range(CFG.BARRIER_GROUP_SIZE):
            slot_pt = self._barrier_slot_point(enemy, slot_idx, center)
            team_travel = max(team_travel, dist2d(launch_pt, {'x': slot_pt[0], 'y': slot_pt[1]}))
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
                and not self._get_interceptor(iid).get('jammed_by_interference')
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
                    and not it.get('jammed_by_interference')
                    and not it.get('task_reserved')
                ]
                pool.sort(key=lambda item: dist2d(item, {'x': center[0], 'y': center[1]}))
                for cand, slot_idx in zip(pool[:needed], available_slots[:needed]):
                    cand['state'] = IState.LAUNCHING
                    cand['target_id'] = enemy['id']
                    cand['role'] = IRole.PRIMARY
                    cand['launch_time'] = self.time + len(team) * 0.2
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

        if slot_d <= CFG.BARRIER_SLOT_TOLERANCE * 0.8:
            it['speed'] = 0.0
            it['target_z'] = slot_pt[2]
            desired_heading = math.degrees(math.atan2(enemy['y'] - it['y'], enemy['x'] - it['x']))
            turn = angle_diff(desired_heading, it['heading'])
            turn = max(-CFG.INTERCEPTOR_MAX_ANG * dt, min(CFG.INTERCEPTOR_MAX_ANG * dt, turn))
            it['heading'] = (it['heading'] + turn) % 360
        else:
            it['speed'] = min(CFG.INTERCEPTOR_BARRIER_SPEED, max(6.0, slot_d * 0.32))
            w = ProNav.guide_point(it, self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}))
            it['heading'] = (it['heading'] + w * dt) % 360
        move_entity(it, dt)
        it['flight_time'] += dt

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
                and not self._get_interceptor(iid).get('jammed_by_interference')
            ]
            needed = CFG.NET_GROUP_SIZE - len(team)
            if needed > 0:
                pool = [
                    it for it in self.interceptors
                    if it['state'] in (IState.STANDBY, IState.RETURNING)
                    and it['target_id'] is None
                    and it['id'] not in team
                    and not it.get('jammed_by_interference')
                    and not it.get('task_reserved')
                ]
                pool.sort(key=lambda item: dist2d(item, enemy))
                for cand in pool[:needed]:
                    cand['state'] = IState.LAUNCHING
                    cand['target_id'] = enemy['id']
                    cand['role'] = IRole.PRIMARY
                    cand['launch_time'] = self.time + len(team) * 0.45
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
            it['speed'] = self._desired_interceptor_speed(enemy, mission="net", phase="close")
            reason = f"执行收网槽位 {it.get('net_slot', 0)+1}"
        else:
            slot_xy = self._net_slot_point(enemy, it.get('net_slot', 0), radius=CFG.NET_CAPTURE_RADIUS, lead_sec=CFG.NET_LEAD_TIME)
            slot_pt = (slot_xy[0], slot_xy[1], self._friendly_altitude(it, enemy, mission="net", phase="form"))
            it['speed'] = self._desired_interceptor_speed(enemy, mission="net", phase="form")
            reason = f"网阻成形槽位 {it.get('net_slot', 0)+1}"
        it['poi'] = slot_pt
        self._update_route_plan(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}, reason)
        w = ProNav.guide_point(it, self._command_point(it, {'x': slot_pt[0], 'y': slot_pt[1], 'z': slot_pt[2]}))
        it['heading'] = (it['heading'] + w * dt) % 360
        move_entity(it, dt)
        it['flight_time'] += dt

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
            if it.get('reported_at', -1.0) < 0 or self.time - it.get('reported_at', -1.0) > CFG.RADAR_STALE_SEC:
                it['reported_x'] = it['x']
                it['reported_y'] = it['y']
                it['reported_z'] = it.get('z', 0.0)
                it['reported_speed'] = it.get('speed', 0.0)
                it['reported_heading'] = it.get('heading', 270.0)
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
                it['mission_label'] = "起飞中"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            elif it['state'] == IState.INTERCEPTING:
                it['mission_label'] = "主拦截"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            elif it['state'] == IState.FOLLOWING:
                it['mission_label'] = "随动"
                it['target_label'] = f"F-{target['id']+1}" if target else "-"
            else:
                it['mission_label'] = "待命"
                it['target_label'] = "-"

    def _set_return(self, it, reason):
        fast_return = it.get('barrier_slot') is not None or it.get('net_slot') is not None
        it['state'] = IState.RETURNING
        it['target_id'] = None
        it['role'] = IRole.RESERVE
        it['search_until'] = 0.0
        it['path_plan'] = []
        it['path_reason'] = ""
        it['poi'] = None
        it['return_fast'] = fast_return
        it['speed'] = min(CFG.INTERCEPTOR_BOOST_SPEED, max(CFG.INTERCEPTOR_SPEED * (1.4 if fast_return else 1.0), it.get('speed', 0.0)))
        it['net_slot'] = None
        it['barrier_slot'] = None
        it['barrier_center'] = None
        self.logs.append(f"[RTB] I-{it['id']+1} {reason}")

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

    def process_command(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return ""
        cmd_lower = cmd.lower()

        llm_constraint_result = self._apply_llm_task_constraints_from_command(cmd)
        if llm_constraint_result is not None:
            return llm_constraint_result

        scene_match = re.search(r"(?:场景|scene)\s*([1-9]|10)(?:km)?", cmd_lower)
        if not scene_match:
            scene_match = re.search(r"\b([1-9]|10)\s*km\b", cmd_lower)
        if scene_match:
            scene_km = float(scene_match.group(1))
            self.configure_scene(scene_km, reset=True)
            return f"已切换到 {scene_km:.0f}km 场景并重置闭环规划"

        is_launch = ("发射" in cmd_lower or "launch" in cmd_lower
                     or bool(re.search(r'(?<!\w)f1(?!\d)', cmd_lower)))
        is_recall = ("撤退" in cmd_lower or "返航" in cmd_lower
                     or "recall" in cmd_lower
                     or bool(re.search(r'(?<!\w)f2(?!\d)', cmd_lower)))
        is_status = ("状态" in cmd_lower or "status" in cmd_lower)

        if "保持警戒" in cmd or "警戒" == cmd.strip():
            self.command_posture = "guard"
            return "已进入警戒态势：不再自动新增派机，已派出无人机会继续执行。"

        if "解除警戒" in cmd or "恢复自动" in cmd or "自动接敌" in cmd:
            self.command_posture = "normal"
            new_launches, active_target_count = self._resume_auto_tasking()
            if new_launches > 0:
                return f"已解除警戒态势：恢复自动接敌与任务分配；已立即补派{new_launches}架。"
            if active_target_count > 0:
                max_active = self._max_active_limit()
                if max_active > 0 and self._committed_interceptor_count() >= max_active:
                    return f"已解除警戒态势：恢复自动接敌与任务分配；但当前仍受LLM最多出动{max_active}架限制。"
                available = sum(
                    1 for item in self.interceptors
                    if item['state'] == IState.STANDBY and not item.get('task_reserved')
                )
                if available <= 0:
                    return "已解除警戒态势：恢复自动接敌与任务分配；但当前无可用待命机。"
                return "已解除警戒态势：恢复自动接敌与任务分配；当前目标已恢复到自动分配队列。"
            return "已解除警戒态势：恢复自动接敌与任务分配。"

        if is_launch:
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
                    n += 1
                    if remaining_capacity is not None:
                        remaining_capacity -= 1
            limit_text = f"，受LLM上限约束最多出动{max_active}架" if max_active > 0 else ""
            if reserved:
                return f"已发射{n}架{limit_text}，LLM保留{reserved}架待命"
            return f"已发射{n}架{limit_text}" if n else "无可用拦截机"

        if is_recall:
            n = 0
            for it in self.interceptors:
                if it['state'] in (IState.INTERCEPTING, IState.FOLLOWING, IState.LAUNCHING):
                    self._set_return(it, "收到地面站返航指令")
                    n += 1
            return f"已召回{n}架" if n else "无在空拦截机"

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

        active_enemies = [e for e in self.enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        self.analyst.chat(cmd, active_enemies, self.interceptors)
        return "▧ 正在同步至战术数据链..."


def run_demo(seed=42, test_wav=None, scene_km=10.0, source="auto",
             redis_host="127.0.0.1", redis_port=6379, redis_db=0,
             intercept_mode="hybrid", demo_case=None, fullscreen=False, ui_style="arc",
             publish_redis=False, publish_interval=0.5, friendly_start=1, enemy_start=101,
             demo_interference_enable=True, demo_interference_visible=True,
             demo_scheme=0, llm_dashboard=False, llm_dashboard_host="127.0.0.1",
             llm_dashboard_port=8765, llm_dashboard_open=False):
    import pygame

    env = InterceptionEnvironment(
        seed=seed,
        scene_km=scene_km,
        source=source,
        redis_host=redis_host,
        redis_port=redis_port,
        redis_db=redis_db,
        intercept_mode=intercept_mode,
        demo_case=demo_case,
        demo_interference_enable=demo_interference_enable,
        demo_interference_visible=demo_interference_visible,
        demo_scheme=demo_scheme,
    )
    from marl_ui import DemoRenderer
    renderer = DemoRenderer(env, fullscreen=fullscreen, ui_style=ui_style)
    clock = pygame.time.Clock()
    started, paused, sim_speed, running = False, False, 4, True
    time_accumulator = 0.0
    redis_publisher = None
    llm_dashboard_server = None

    if publish_redis:
        try:
            redis_publisher = DemoRedisPublisher(
                host=redis_host,
                port=redis_port,
                db=redis_db,
                publish_interval=publish_interval,
                friendly_start=friendly_start,
                enemy_start=enemy_start,
                cleanup_node_nums=_planned_demo_node_nums(env, friendly_start, enemy_start),
            )
            renderer.add_log(
                "[SYNC]",
                f"同帧发布已开启 | Friendly {friendly_start}-{friendly_start + len(env.interceptors) - 1} | Enemy {enemy_start}+",
                "green",
            )
            redis_publisher.maybe_publish(env, force=True)
        except Exception as exc:
            renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
            redis_publisher = None

    if llm_dashboard:
        try:
            from marl_llm_dashboard import LLMDashboardServer
            llm_dashboard_server = LLMDashboardServer(
                env,
                host=llm_dashboard_host,
                port=llm_dashboard_port,
                open_browser=llm_dashboard_open,
            )
            url = llm_dashboard_server.start()
            renderer.add_log("[LLM-WEB]", f"LLM决策链网页已开启: {url}", "green")
            print(f"LLM dashboard: {url}")
        except Exception as exc:
            renderer.add_log("[LLM-WEB]", f"LLM决策链网页启动失败: {exc}", "red")
            llm_dashboard_server = None

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
        f"UI={ui_style} Case={demo_case or 'default'} Redis={redis_host}:{redis_port}/{redis_db}"
    )
    print("Enter-开始 Space-暂停 P-显隐预测线 F11-全屏切换")
    print("1/2/4/8-常规速度 0-极速(20x)")
    print("T-指令 V-按住语音 R-同场景重置 Shift+R-新seed F1-发射 F2-返航 ESC-退出")
    print("=" * 60)

    while running:
        raw_dt = clock.tick(CFG.FPS) / 1000.0
        renderer.cursor_timer += raw_dt
        if renderer.cursor_timer > 0.5:
            renderer.cursor_timer = 0.0
            renderer.cursor_vis = not renderer.cursor_vis

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type in (pygame.MOUSEWHEEL, pygame.MOUSEBUTTONDOWN):
                renderer.handle_event(event)
            elif event.type == pygame.VIDEORESIZE and not renderer.fullscreen:
                renderer._set_display_mode(size=(event.w, event.h))
                renderer._init_fonts()
            elif event.type == pygame.KEYDOWN:
                if renderer.chat_active:
                    if event.key == pygame.K_RETURN:
                        if renderer.chat_input.strip():
                            text = renderer.chat_input.strip()
                            renderer.add_log("[USR]", text, "txt_h")
                            resp = env.process_command(text)
                            renderer.add_log("[CMD]", resp, "green")
                        renderer.chat_input = ""
                        renderer.chat_active = False
                    elif event.key == pygame.K_ESCAPE:
                        renderer.chat_input = ""
                        renderer.chat_active = False
                    elif event.key == pygame.K_BACKSPACE:
                        renderer.chat_input = renderer.chat_input[:-1]
                else:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_RETURN and not started:
                        started = True
                        renderer.add_log("[SYS]", f"任务开始({sim_speed}x)", "green")
                    elif event.key == pygame.K_r:
                        if event.mod & pygame.KMOD_SHIFT:
                            env.seed = random.randint(0, 10000)
                        env.reset()
                        if redis_publisher:
                            try:
                                redis_publisher.maybe_publish(env, force=True)
                            except Exception as exc:
                                renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
                                redis_publisher = None
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
                    elif event.key == pygame.K_F1:
                        renderer.add_log("[CMD]", env.process_command("全体发射"), "green")
                    elif event.key == pygame.K_F2:
                        renderer.add_log("[CMD]", env.process_command("全体返航"), "amber")
                    elif event.key == pygame.K_v:
                        if voice_engine and renderer.voice_state == 0:
                            renderer.voice_state = 1
                            voice_engine.start_recording()
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_v and renderer.voice_state == 1:
                    renderer.voice_state = 2
                    if voice_engine:
                        voice_engine.stop_recording()
            elif event.type == pygame.TEXTINPUT:
                if renderer.chat_active and len(renderer.chat_input) < 50:
                    renderer.chat_input += event.text

        if started and not paused and not env.done:
            time_accumulator += raw_dt * sim_speed
            while time_accumulator >= CFG.DT:
                env.step(CFG.DT)
                time_accumulator -= CFG.DT
                if env.done:
                    break

            if renderer._elo > len(env.logs):
                renderer._elo = 0
            new_logs = env.logs[renderer._elo:]
            for log in new_logs:
                if isinstance(log, tuple):
                    renderer.add_log(log[0], log[1], log[2])
                else:
                    renderer.add_log("[ENV]", log, "blue")
            renderer._elo = len(env.logs)

        if voice_engine:
            vr = voice_engine.get_result()
            if vr is not None:
                if vr.startswith("[SYS]") or vr.startswith("[错误]"):
                    color = "green" if "加载完毕" in vr else ("red" if "错误" in vr else "amber")
                    renderer.add_log("[VOICE]", vr, color)
                    if renderer.voice_state == 2:
                        renderer.voice_state = 0
                else:
                    renderer.add_log("[VOICE]", vr, "pink")
                    renderer.add_log("[CMD]", env.process_command(vr), "green")
                    renderer.voice_state = 0

        if redis_publisher:
            try:
                redis_publisher.maybe_publish(env)
            except Exception as exc:
                renderer.add_log("[SYNC]", f"Redis同帧发布失败: {exc}", "red")
                redis_publisher = None

        renderer.render(sim_speed)

    if voice_engine:
        voice_engine.shutdown()
    if redis_publisher:
        try:
            redis_publisher.cleanup()
        except Exception:
            pass
    if llm_dashboard_server:
        llm_dashboard_server.stop()
    pygame.quit()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', default='menu', choices=['menu', 'demo'])
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--scene-km', type=float, default=10.0, help='场景尺度: 1/2/3/4/5/10 km')
    parser.add_argument('--source', default='auto', choices=['auto', 'redis', 'demo'])
    parser.add_argument('--intercept-mode', default='hybrid', choices=['hybrid', 'hit', 'net', 'legacy-net'])
    parser.add_argument('--demo-case', default=None, choices=['net-single', 'barrier-single'])
    parser.add_argument('--redis-host', default='127.0.0.1')
    parser.add_argument('--redis-port', type=int, default=6379)
    parser.add_argument('--redis-db', type=int, default=0)
    parser.add_argument('--publish-redis', action='store_true', help='演示时将当前同一仿真帧同步写入 Redis')
    parser.add_argument('--publish-interval', type=float, default=0.5, help='Redis同步周期(秒)，最小0.03')
    parser.add_argument('--friendly-start', type=int, default=1, help='己方写入 Redis 的起始编号')
    parser.add_argument('--enemy-start', type=int, default=101, help='敌方写入 Redis 的起始编号')
    parser.add_argument('--test-wav', type=str, default=None, help='WAV文件路径，用于远程调试语音 (跳过麦克风)')
    parser.add_argument('--fullscreen', action='store_true')
    parser.add_argument('--ui-style', default='arc', choices=['arc', 'rect'])
    parser.add_argument('--demo-interference-enable', default='on', help='demo强干扰效果开关: on/off/1/0')
    parser.add_argument('--demo-interference-visible', default='on', help='demo干扰圆区可视化开关: on/off/1/0')
    parser.add_argument('--demo-scheme', type=int, default=0, choices=[0, 1, 2, 3], help='demo演示方案: 0=手动当前配置, 1=传统最近邻, 2=威胁协同, 3=强干扰失联')
    parser.add_argument('--llm-dashboard', action='store_true', help='启动单独网页显示LLM可解释决策链')
    parser.add_argument('--llm-dashboard-host', default='127.0.0.1', help='LLM决策链网页监听地址')
    parser.add_argument('--llm-dashboard-port', type=int, default=8765, help='LLM决策链网页端口')
    parser.add_argument('--llm-dashboard-open', action='store_true', help='启动后尝试自动打开浏览器')
    args = parser.parse_args()

    if args.mode == 'menu':
        from marl_menu import run_with_menu
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
            intercept_mode=args.intercept_mode,
            demo_case=args.demo_case,
            fullscreen=args.fullscreen,
            ui_style=args.ui_style,
            publish_redis=args.publish_redis,
            publish_interval=args.publish_interval,
            friendly_start=args.friendly_start,
            enemy_start=args.enemy_start,
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
