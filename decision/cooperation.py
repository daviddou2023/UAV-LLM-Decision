"""
拦截指派模块 - MPC动态拦截 v8.0
"""
import math
from typing import Dict

from core.common import CFG, IState, EState, IRole, dist2d, EType
from decision.reassignment import StableRetaskPolicy

class InterceptionAssigner:
    """
    向上：simulation/main.py 将解析出来的硬性约束赋值给 self.assigner.task_constraints，本文件的simulation/main.py 将解析出来的硬性约束赋值给 self.assigner.task_constraints和_max_active_count读取这些配置

    实例化了decision/reassignment.py 中的 StableRetaskPolicy，在每次分配新任务前，先让它去检查“是否有敌机已被击毁/消失？是否有无人机进入了强干扰区失联？

    decision/cooperation.py 不负责让无人机真正移动，它只是打上了标签。打完标签后返回主循环，后续是simulation/main.py 里面的 _move_interceptors 方法会接手
    """
    def __init__(self):
        # 记录当前的分配关系：字典格式 {敌机ID: {'primary': 拦截机ID, 'follower': 备份机ID}}
        self.assignments: Dict[int, dict] = {}
        # 重分配：清洗已经失效的分配
        self.retask_policy = StableRetaskPolicy(self)
        self.last_active_count = 0
        # 接收来自大模型的战术约束
        self.task_constraints = {}

    def _sector_of_enemy(self, enemy):
        """判断敌机所在的防区（左翼、右翼、中路）"""
        x = float(enemy.get('x', 0.0) or 0.0)
        if x < CFG.AREA_WIDTH * 0.33:
            return "left"
        if x > CFG.AREA_WIDTH * 0.67:
            return "right"
        return "center"

    def _sector_rank(self, enemy):
        """根据大模型下发的防区偏好，给敌机打分。如果是优先防区返回0(最高级)，否则返回1"""
        preferred = (getattr(self, "task_constraints", {}) or {}).get("preferred_sector")
        if preferred not in ("left", "right", "center"):
            return 0
        return 0 if self._sector_of_enemy(enemy) == preferred else 1

    def _max_active_count(self):
        """读取大模型设置的己方最大允许出动数量限制"""
        constraints = getattr(self, "task_constraints", {}) or {}
        try:
            count = int(constraints.get("max_active_count") or 0)
        except (TypeError, ValueError):
            count = 0
        return max(0, count)

    def _device3_available(self, it, now=None):
        """检查下游设备(如真实飞控)是否报告该无人机暂时故障不可用"""
        if not it.get("device3_temporarily_unavailable"):
            return True
        failed_until = it.get("device3_failed_until")
        if now is not None and failed_until is not None:
            try:
                if float(now) >= float(failed_until):
                    it["device3_temporarily_unavailable"] = False
                    it["device3_failure_reason"] = ""
                    return True
            except (TypeError, ValueError):
                pass
        return False

    def _committed_interceptor_count(self, interceptors):
        return sum(
            1 for it in interceptors
            if it["state"] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
            and not it.get("task_reserved")
            and self._device3_available(it)
        )

    def _target_sort_key(self, enemy):
        """
                核心排序函数：决定哪架敌机应该最先被拦截！
                优先级逻辑：
                1. 敌机是否脱网/丢失 (stale)
                2. 是否在 LLM 指定的优先防区 (_sector_rank)
                3. 根据 LLM 偏好(优先高速 speed、优先突防 breach) 和系统内置的威胁度(_priority) 综合打分
                """
        priority_mode = (getattr(self, "task_constraints", {}) or {}).get("target_priority")
        base = (enemy.get('stale', False), self._sector_rank(enemy))
        if priority_mode == "speed":
            # 高速优先
            return base + (-enemy.get('speed', 0.0), self._priority(enemy), -enemy['y'])
        if priority_mode == "breach":
            # 突防优先
            return base + (-enemy['y'], self._priority(enemy), -enemy.get('speed', 0.0))
        return base + (self._priority(enemy), -enemy['y'], -enemy.get('speed', 0.0))

    def sort_active_enemies(self, enemies):
        """对外暴露的敌机排序接口"""
        return sorted(list(enemies), key=self._target_sort_key)

    def _interceptor_speed_for_target(self, enemy):
        if enemy['type'] in (EType.DECOY, EType.DASH):
            return CFG.INTERCEPTOR_BOOST_SPEED
        if enemy['type'] == EType.LOITER and enemy.get('is_diving'):
            return CFG.INTERCEPTOR_BOOST_SPEED
        if enemy.get('speed', 0.0) >= CFG.INTERCEPTOR_SPEED * 0.95:
            return CFG.INTERCEPTOR_BOOST_SPEED
        return CFG.INTERCEPTOR_SPEED

    def _priority(self, enemy):
        """硬编码的敌机类型威胁度评级 (数字越小越危险)"""
        if enemy['type'] == EType.LOITER: # 巡飞弹
            return 1
        if enemy['type'] == EType.DASH: # 高速突防
            return 2
        if enemy['type'] in (EType.SNAKE, EType.JINK): # 机动/闪避目标
            return 3
        if enemy['type'] == EType.DECOY:
            dist_to_line = CFG.INTERCEPT_FAIL_LINE - enemy['y']
            if enemy.get('speed', 0.0) >= CFG.INTERCEPTOR_SPEED * 0.95 or dist_to_line <= CFG.INTERCEPT_FAIL_LINE * 0.35:
                return 4
            return 7
        return 5 # 常规目标

    def compute_poi(self, intr, enemy):
        """
                预测相遇点 (Point of Intercept, POI)
                已知敌机当前位置、速度向量，和己方拦截机的标量速度，求解它们在何时何地相遇。
                使用一元二次方程求解： a*t^2 + b*t + c = 0
                """
        is_loiter = (enemy['type'] == EType.LOITER) or (str(enemy['type']) == 'EType.LOITER')
        if is_loiter:
            sim_speed = 40.0
            sim_heading = math.radians(90.0)
            evx = sim_speed * math.cos(sim_heading)
            evy = sim_speed * math.sin(sim_heading)
            evz = enemy.get('vz', 0.0)
        else:
            heading = enemy.get('heading')
            if heading is None:
                heading = 90.0
            speed = enemy.get('speed', 0.0) or 0.0
            er = math.radians(float(heading))
            evx = speed * math.cos(er)
            evy = speed * math.sin(er)
            evz = enemy.get('vz', 0.0)

        dx = enemy['x'] - intr['x']
        dy = enemy['y'] - intr['y']
        dz = enemy.get('z', 0.0) - intr.get('z', 0.0)
        vi = self._interceptor_speed_for_target(enemy)

        a = evx**2 + evy**2 + evz**2 - vi**2
        b = 2 * (dx * evx + dy * evy + dz * evz)
        c = dx**2 + dy**2 + dz**2

        t = None
        if abs(a) < 1e-6:
            if abs(b) > 1e-6:
                t_cand = -c / b
                if t_cand > 0: t = t_cand
        else:
            disc = b*b - 4*a*c
            if disc >= 0:
                sq = math.sqrt(disc)
                t1 = (-b - sq) / (2*a)
                t2 = (-b + sq) / (2*a)
                cands = [x for x in [t1, t2] if x > 0.1]
                if cands: t = min(cands)

        if t is None:
            return None, None
        # 返回预测的相遇三维坐标和所需时间
        return (
            enemy['x'] + evx * t,
            enemy['y'] + evy * t,
            max(0.0, enemy.get('z', 0.0) + evz * t),
        ), t

    def is_feasible(self, poi):
        return poi is not None and poi[1] < CFG.INTERCEPT_FAIL_LINE + CFG.POI_MARGIN

    def _launch_delay(self, intr, queue_offset=0.0):
        hangar_count = max(1, len(CFG.HANGAR_POSITIONS))
        hangar_idx = intr.get('hangar_idx', intr['id'] % hangar_count)
        stack_idx = intr['id'] // hangar_count
        return 0.18 * float(queue_offset) + 0.08 * hangar_idx + 0.16 * stack_idx

    def _high_pressure(self):
        pressure_floor = max(4, int(math.ceil(CFG.NUM_INTERCEPTORS * CFG.HIGH_PRESSURE_THREAT_RATIO)))
        return self.last_active_count >= pressure_floor

    def _can_commit_follower(self, avail_count, enemy):
        if avail_count < CFG.MIN_FREE_INTERCEPTORS_FOR_FOLLOWER:
            return False
        if self._high_pressure() and enemy['type'] not in (EType.LOITER, EType.DASH):
            return False
        reserve_floor = max(CFG.RESERVE_INTERCEPTOR_BUFFER, int(math.ceil(self.last_active_count * 0.2)))
        return (avail_count - 1) >= reserve_floor

    def update(self, interceptors, enemies, current_time):
        """撞击拦截任务分配主入口。

        调用方是 InterceptionEnvironment.step()。本函数原地更新己方 UAV 的
        target_id/role/poi，并维护 assignments；返回值只用于 UI/日志展示。
        """
        msgs = []
        # 只把“已发现、未丢失、仍在威胁态”的敌机送入分配器。
        active = [
            e for e in enemies
            if e['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and e['detected']
            and not e.get('lost', False)
        ]
        self.last_active_count = len(active)

        # 排序并清理失效的旧分配
        active = self.sort_active_enemies(active)
        msgs.extend(self.retask_policy.reconcile(interceptors, enemies, active))

        # 分配主拦截：每个有效目标至少争取一个 primary，任务写回 interceptor。
        for en in active:
            # 只要没分配主拦截，就必须分配！不管是新目标还是刚才被踢掉的
            if en['id'] not in self.assignments or self.assignments[en['id']].get('primary') is None:
                if en['id'] not in self.assignments: self.assignments[en['id']] = {'primary': None, 'follower': None}
                r = self._assign_primary(interceptors, en, current_time)
                if r: msgs.append(r)

        # 分配随动备份机：只在候选、压力和目标类型允许时补 follower。
        for en in active:
            if self.assignments[en['id']].get('follower') is None:
                r = self._assign_follower(interceptors, en, self.assignments[en['id']])
                if r: msgs.append(r)

        # 更新现有分配关系的 POI/ETA，路径规划后续会读取 it['poi']。
        for en in active:
            if en['id'] in self.assignments:
                for role in ['primary','follower']:
                    iid = self.assignments[en['id']].get(role)
                    if iid is None: continue
                    it = next((i for i in interceptors if i['id']==iid), None)
                    if it and it['state'] in (IState.INTERCEPTING, IState.FOLLOWING):
                        poi, t = self.compute_poi(it, en)
                        it['poi']=poi; it['poi_time']=t
                        it['target_z'] = poi[2] if poi else en.get('z', 0.0)
                        if en['id'] in self.assignments:
                            self.assignments[en['id']]['poi'] = poi
                            self.assignments[en['id']]['eta'] = t
        return msgs

    def _bind_assignment(self, eid, iid, role):
        if eid not in self.assignments:
            self.assignments[eid] = {'primary': None, 'follower': None}
        if role == IRole.PRIMARY and self.assignments[eid]['primary'] is not None:
            role = IRole.FOLLOWER
        role_key = 'primary' if role == IRole.PRIMARY else 'follower'
        if self.assignments[eid][role_key] is None:
            self.assignments[eid][role_key] = iid

    def _try_hot_reassign(self, intr, active_enemies):
        if intr.get('task_reserved'):
            return None, None
        if not self._device3_available(intr):
            return None, None
        for en in active_enemies:
            eid = en['id']
            asgn = self.assignments.get(eid, {'primary': None, 'follower': None})
            if en.get('stale') or en.get('lost'):
                continue
            if asgn['primary'] is None: return eid, IRole.PRIMARY
            if (
                asgn['follower'] is None and
                en['type'] != EType.DECOY and
                en.get('classification_confidence', 1.0) >= CFG.MISCLASSIFY_CONFIDENCE and
                not self._high_pressure()
            ):
                return eid, IRole.FOLLOWER
        return None, None

    def _assign_primary(self, interceptors, enemy, ct):
        """为特定敌机寻找最优的【主拦截机】"""
        avail = [
            i for i in interceptors
            if i['state'] in (IState.STANDBY, IState.RETURNING)
            and i['target_id'] is None
            and not i.get('jammed_by_interference')
            and not i.get('task_reserved')
            and self._device3_available(i, ct)
        ]
        max_active = self._max_active_count()
        if max_active > 0 and self._committed_interceptor_count(interceptors) >= max_active:
            avail = [i for i in avail if i['state'] == IState.RETURNING]
        if not avail: return None

        best, bt, best_poi, best_mode = None, 1e9, None, "最近可达"
        # Stage A: 完美拦截：计算POI能追上，而且相遇点在防线之前
        for it in avail:
            poi, t = self.compute_poi(it, enemy)
            if t and t < bt and self.is_feasible(poi):
                best, bt, best_poi, best_mode = it, t, poi, "POI可达"
        # Stage B: 紧急拦截：能追上，但是相遇点已经过了防线
        if best is None:
            for it in avail:
                poi, t = self.compute_poi(it, enemy)
                if t and t < bt:
                    best, bt, best_poi, best_mode = it, t, poi, "紧急前出"
        # Stage C: 保底：派离敌机最近的直接去撞
        if best is None:
            best = min(avail, key=lambda i: dist2d(i, enemy))
            best_poi = (enemy['x'], enemy['y'], enemy.get('z', 0.0))
            best_mode = "保底直扑"

        # 绑定任务关系，并将该无人机状态设定为起飞
        best['state'] = IState.LAUNCHING
        best['target_id'] = enemy['id']
        best['role'] = IRole.PRIMARY
        best['launch_time'] = ct + self._launch_delay(best)
        best['speed'] = 0.0
        best['target_z'] = enemy.get('z', 0.0)

        self.assignments[enemy['id']]['primary'] = best['id']
        self.assignments[enemy['id']]['launch_anchor'] = best['launch_time']
        self.assignments[enemy['id']]['poi'] = best_poi
        self.assignments[enemy['id']]['eta'] = bt if bt < 1e9 else None
        if best_poi and bt < 1e9:
            return (
                f"I-{best['id']+1} → F-{enemy['id']+1} 主拦截 | "
                f"ETA {bt:.1f}s | POI=({best_poi[0]:.0f},{best_poi[1]:.0f},{best_poi[2]:.0f}) | {best_mode}"
            )
        return f"I-{best['id']+1} → F-{enemy['id']+1} 主拦截 | {best_mode}"

    def _assign_follower(self, interceptors, enemy, asgn):
        avail = [
            i for i in interceptors
            if i['state'] == IState.STANDBY
            and i['target_id'] is None
            and i['id'] != asgn.get('primary')
            and not i.get('jammed_by_interference')
            and not i.get('task_reserved')
            and self._device3_available(i)
        ]
        max_active = self._max_active_count()
        if max_active > 0 and self._committed_interceptor_count(interceptors) >= max_active:
            return None
        if not avail: return None
        if len(avail) < 3: return None
        if not self._can_commit_follower(len(avail), enemy):
            return None
        urgent_decoy = (
            enemy['type'] == EType.DECOY and
            (
                enemy.get('speed', 0.0) >= CFG.INTERCEPTOR_SPEED * 0.95 or
                enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.55
            )
        )
        if enemy['type'] == EType.DECOY and not urgent_decoy: return None
        if enemy.get('classification_confidence', 1.0) < CFG.MISCLASSIFY_CONFIDENCE:
            if not asgn.get('follower_blocked'):
                asgn['follower_blocked'] = True
                return f"F-{enemy['id']+1} 识别置信度低，暂不派随动机"
            return None

        best = min(avail, key=lambda i: dist2d(i, enemy))
        best['state'] = IState.LAUNCHING
        best['target_id'] = enemy['id']
        best['role'] = IRole.FOLLOWER
        best['speed'] = 0.0
        best['target_z'] = enemy.get('z', 0.0)
        if asgn.get('launch_anchor') is not None:
            best['launch_time'] = asgn['launch_anchor'] + 0.45 + self._launch_delay(best, queue_offset=1.0)
        else:
            best['launch_time'] = self._launch_delay(best, queue_offset=1.0)
        asgn['follower'] = best['id']
        poi, eta = self.compute_poi(best, enemy)
        if poi and eta:
            return (
                f"I-{best['id']+1} → F-{enemy['id']+1} 随动 | "
                f"ETA {eta:.1f}s | POI=({poi[0]:.0f},{poi[1]:.0f},{poi[2]:.0f}) | 备份拦截"
            )
        return f"I-{best['id']+1} → F-{enemy['id']+1} 随动"

    def get_info(self, eid):
        return self.assignments.get(eid)

    def get_status(self):
        return f"任务组: {len(self.assignments)}"
