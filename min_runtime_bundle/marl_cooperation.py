"""
拦截指派模块 - MPC动态拦截 v8.0
"""
import math
from typing import Dict

from marl_common import CFG, IState, EState, IRole, dist2d, EType
from marl_reassignment import StableRetaskPolicy


class InterceptionAssigner:
    def __init__(self):
        self.assignments: Dict[int, dict] = {}
        self.retask_policy = StableRetaskPolicy(self)
        self.last_active_count = 0
        self.task_constraints = {}

    def _sector_of_enemy(self, enemy):
        x = float(enemy.get('x', 0.0) or 0.0)
        if x < CFG.AREA_WIDTH * 0.33:
            return "left"
        if x > CFG.AREA_WIDTH * 0.67:
            return "right"
        return "center"

    def _sector_rank(self, enemy):
        preferred = (getattr(self, "task_constraints", {}) or {}).get("preferred_sector")
        if preferred not in ("left", "right", "center"):
            return 0
        return 0 if self._sector_of_enemy(enemy) == preferred else 1

    def _max_active_count(self):
        constraints = getattr(self, "task_constraints", {}) or {}
        try:
            count = int(constraints.get("max_active_count") or 0)
        except (TypeError, ValueError):
            count = 0
        return max(0, count)

    def _committed_interceptor_count(self, interceptors):
        return sum(
            1 for it in interceptors
            if it["state"] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
            and not it.get("task_reserved")
        )

    def _target_sort_key(self, enemy):
        priority_mode = (getattr(self, "task_constraints", {}) or {}).get("target_priority")
        base = (enemy.get('stale', False), self._sector_rank(enemy))
        if priority_mode == "speed":
            return base + (-enemy.get('speed', 0.0), self._priority(enemy), -enemy['y'])
        if priority_mode == "breach":
            return base + (-enemy['y'], self._priority(enemy), -enemy.get('speed', 0.0))
        return base + (self._priority(enemy), -enemy['y'], -enemy.get('speed', 0.0))

    def sort_active_enemies(self, enemies):
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
        if enemy['type'] == EType.LOITER:
            return 1
        if enemy['type'] == EType.DASH:
            return 2
        if enemy['type'] in (EType.SNAKE, EType.JINK):
            return 3
        if enemy['type'] == EType.DECOY:
            dist_to_line = CFG.INTERCEPT_FAIL_LINE - enemy['y']
            if enemy.get('speed', 0.0) >= CFG.INTERCEPTOR_SPEED * 0.95 or dist_to_line <= CFG.INTERCEPT_FAIL_LINE * 0.35:
                return 4
            return 7
        return 5

    def compute_poi(self, intr, enemy):
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
                if t_cand > 0:
                    t = t_cand
        else:
            disc = b * b - 4 * a * c
            if disc >= 0:
                sq = math.sqrt(disc)
                t1 = (-b - sq) / (2 * a)
                t2 = (-b + sq) / (2 * a)
                cands = [x for x in (t1, t2) if x > 0.1]
                if cands:
                    t = min(cands)

        if t is None:
            return None, None
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
        ratio = float(getattr(CFG, "HIGH_PRESSURE_THREAT_RATIO", 0.45))
        pressure_floor = max(4, int(math.ceil(CFG.NUM_INTERCEPTORS * ratio)))
        return self.last_active_count >= pressure_floor

    def _can_commit_follower(self, avail_count, enemy):
        min_free = int(getattr(CFG, "MIN_FREE_INTERCEPTORS_FOR_FOLLOWER", 4))
        if avail_count < min_free:
            return False
        if self._high_pressure() and enemy['type'] not in (EType.LOITER, EType.DASH):
            return False
        reserve_buffer = int(getattr(CFG, "RESERVE_INTERCEPTOR_BUFFER", 2))
        reserve_floor = max(reserve_buffer, int(math.ceil(self.last_active_count * 0.2)))
        return (avail_count - 1) >= reserve_floor

    def update(self, interceptors, enemies, current_time):
        msgs = []
        active = [
            e for e in enemies
            if e['state'] in (EState.APPROACHING, EState.MANEUVERING)
            and e['detected']
            and not e.get('lost', False)
        ]
        self.last_active_count = len(active)

        active = self.sort_active_enemies(active)
        msgs.extend(self.retask_policy.reconcile(interceptors, enemies, active))

        for en in active:
            if en['id'] not in self.assignments or self.assignments[en['id']].get('primary') is None:
                if en['id'] not in self.assignments:
                    self.assignments[en['id']] = {'primary': None, 'follower': None}
                r = self._assign_primary(interceptors, en, current_time)
                if r:
                    msgs.append(r)

        for en in active:
            if self.assignments[en['id']].get('follower') is None:
                r = self._assign_follower(interceptors, en, self.assignments[en['id']])
                if r:
                    msgs.append(r)

        for en in active:
            if en['id'] in self.assignments:
                for role in ('primary', 'follower'):
                    iid = self.assignments[en['id']].get(role)
                    if iid is None:
                        continue
                    it = next((i for i in interceptors if i['id'] == iid), None)
                    if it and it['state'] in (IState.INTERCEPTING, IState.FOLLOWING):
                        poi, t = self.compute_poi(it, en)
                        it['poi'] = poi
                        it['poi_time'] = t
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
        for en in active_enemies:
            eid = en['id']
            asgn = self.assignments.get(eid, {'primary': None, 'follower': None})
            if en.get('stale') or en.get('lost'):
                continue
            if asgn['primary'] is None:
                return eid, IRole.PRIMARY
            if (
                asgn['follower'] is None and
                en['type'] != EType.DECOY and
                en.get('classification_confidence', 1.0) >= CFG.MISCLASSIFY_CONFIDENCE and
                not self._high_pressure()
            ):
                return eid, IRole.FOLLOWER
        return None, None

    def _assign_primary(self, interceptors, enemy, ct):
        avail = [
            i for i in interceptors
            if i['state'] in (IState.STANDBY, IState.RETURNING)
            and i['target_id'] is None
            and not i.get('jammed_by_interference')
            and not i.get('task_reserved')
        ]
        max_active = self._max_active_count()
        if max_active > 0 and self._committed_interceptor_count(interceptors) >= max_active:
            avail = [i for i in avail if i['state'] == IState.RETURNING]
        if not avail:
            return None

        best, bt, best_poi, best_mode = None, 1e9, None, "最近可达"
        for it in avail:
            poi, t = self.compute_poi(it, enemy)
            if t and t < bt and self.is_feasible(poi):
                best, bt, best_poi, best_mode = it, t, poi, "POI可达"
        if best is None:
            for it in avail:
                poi, t = self.compute_poi(it, enemy)
                if t and t < bt:
                    best, bt, best_poi, best_mode = it, t, poi, "紧急前出"
        if best is None:
            best = min(avail, key=lambda i: dist2d(i, enemy))
            best_poi = (enemy['x'], enemy['y'], enemy.get('z', 0.0))
            best_mode = "保底直扑"

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
        ]
        max_active = self._max_active_count()
        if max_active > 0 and self._committed_interceptor_count(interceptors) >= max_active:
            return None
        if not avail:
            return None
        if len(avail) < 3:
            return None
        if not self._can_commit_follower(len(avail), enemy):
            return None
        urgent_decoy = (
            enemy['type'] == EType.DECOY and
            (
                enemy.get('speed', 0.0) >= CFG.INTERCEPTOR_SPEED * 0.95 or
                enemy['y'] >= CFG.INTERCEPT_FAIL_LINE * 0.55
            )
        )
        if enemy['type'] == EType.DECOY and not urgent_decoy:
            return None
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
