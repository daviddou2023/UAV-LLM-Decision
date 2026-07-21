import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from core.common import CFG, EState, IRole, IState, dist2d


@dataclass
class LocalPlan:
    command_point: Tuple[float, float, float]
    speed_cap: Optional[float]
    target_z: float
    hold_reason: str = ""
    avoid_mode: str = ""
    allow_terminal_direct: bool = False


class DeconflictionController:

    """
    与其他文件联系：
    simulation/main.py的_move_interceptors(dt) 中，在决定飞机往前飞多少之前，会先调用本模块plan_local_motion，经过本模块修改并返回的plan.command_point 才是飞机最终朝向的“安全航点

    """

    # 维护每架无人机的局部规划缓存
    def __init__(self, env):
        self.env = env
        self.pair_cooldown: Dict[Tuple[int, int], float] = {}
        self.local_states: Dict[int, dict] = {}

    def _planner_state(self, iid):
        return self.local_states.setdefault(
            iid,
            {
                "last_plan_time": -1e9,
                "last_plan": None,
                "last_desired": None,
                "last_command": None,
                "mission_key": None,
                "barrier_side": 0,
                "barrier_side_until": -1e9,
                "hold_reason": "",
                "hold_started_at": -1e9,
                "last_avoid_mode": "",
            },
        )

    def reset_local_state(self, iid):
        self.local_states.pop(iid, None)
        self.env.deconflict_cooldown.pop(iid, None)

    # 基础的2D距离计算和速度向量归一化
    def _distance_xy(self, a_pt, b_pt):
        return math.hypot(a_pt[0] - b_pt[0], a_pt[1] - b_pt[1])

    def _normalize(self, vx, vy):
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (vx / norm, vy / norm)

    def _velocity_vector(self, entity):
        heading = math.radians(entity.get("heading", 0.0))
        speed = max(0.0, entity.get("speed", 0.0))
        return (math.cos(heading) * speed, math.sin(heading) * speed)

    # 获取当前友军拉网的区域范围
    def _barrier_zone_margin(self, zone_type):
        if zone_type == "core":
            return CFG.BARRIER_CORE_KEEP_OUT_MARGIN
        if zone_type == "buffer":
            return CFG.BARRIER_BUFFER_MARGIN
        return max(CFG.FRIENDLY_SAFE_SEPARATION, 90.0)

    def _segment_hits_barrier_zone(self, start_pt, end_pt, zone):
        for idx in range(17):
            t = idx / 16.0
            px = start_pt[0] + (end_pt[0] - start_pt[0]) * t
            py = start_pt[1] + (end_pt[1] - start_pt[1]) * t
            if zone["xmin"] <= px <= zone["xmax"] and zone["ymin"] <= py <= zone["ymax"]:
                return True
        return False

    def active_barrier_zones(self, ignore_enemy_id=None, zone_type="buffer"):
        zones = []
        margin = self._barrier_zone_margin(zone_type)
        half_span = self.env._barrier_half_span()
        half_depth = self.env._barrier_half_depth()
        for enemy_id, team in self.env.barrier_team_assignments.items():
            if ignore_enemy_id is not None and enemy_id == ignore_enemy_id:
                continue
            if not team:
                continue
            center = self.env.barrier_states.get(enemy_id, {}).get("center")
            enemy = self.env._get_enemy(enemy_id)
            if center is None and enemy and enemy["state"] in (EState.APPROACHING, EState.MANEUVERING):
                center = self.env._barrier_intercept_center(enemy)
            if center is None:
                continue
            zones.append(
                {
                    "enemy_id": enemy_id,
                    "center": center,
                    "xmin": center[0] - half_span - CFG.BARRIER_NET_RADIUS - margin,
                    "xmax": center[0] + half_span + CFG.BARRIER_NET_RADIUS + margin,
                    "ymin": center[1] - half_depth - margin,
                    "ymax": center[1] + half_depth + margin,
                    "margin": margin,
                    "zone_type": zone_type,
                }
            )
        return zones

    def _point_in_zone(self, point, zone):
        return zone["xmin"] <= point[0] <= zone["xmax"] and zone["ymin"] <= point[1] <= zone["ymax"]

    def _point_to_zone_distance(self, point, zone):
        dx = max(zone["xmin"] - point[0], 0.0, point[0] - zone["xmax"])
        dy = max(zone["ymin"] - point[1], 0.0, point[1] - zone["ymax"])
        return math.hypot(dx, dy)

    def _first_zone_on_route(self, start_pt, end_pt, ignore_enemy_id=None, zone_type="buffer"):
        hit = None
        best_score = 1e12
        for zone in self.active_barrier_zones(ignore_enemy_id=ignore_enemy_id, zone_type=zone_type):
            if (
                self._point_in_zone(start_pt, zone)
                or self._point_in_zone(end_pt, zone)
                or self._segment_hits_barrier_zone(start_pt, end_pt, zone)
            ):
                score = self._point_to_zone_distance(start_pt, zone)
                if score < best_score:
                    hit = zone
                    best_score = score
        return hit

    # 判断无人机的直线航线是否会穿过禁飞区
    def route_hits_barrier_zone(self, start_pt, end_pt, ignore_enemy_id=None, zone_type="buffer"):
        return self._first_zone_on_route(start_pt, end_pt, ignore_enemy_id=ignore_enemy_id, zone_type=zone_type) is not None

    def _priority_rank(self, it):
        if it.get("barrier_slot") is not None:
            rank = 60
        elif it.get("net_slot") is not None:
            rank = 56
        elif it["state"] == IState.INTERCEPTING:
            rank = 48 if it.get("role") == IRole.PRIMARY else 42
        elif it["state"] == IState.FOLLOWING:
            rank = 38
        elif it["state"] == IState.LAUNCHING:
            rank = 30
        elif it["state"] == IState.RETURNING:
            rank = 22
        else:
            rank = 0
        if it.get("target_id") is not None:
            rank += 3
        if it.get("return_fast"):
            rank += 1
        return rank

    # 核心路权逻辑：判定两机相遇时谁给谁让路
    def _mission_distance(self, it):
        if it.get("target_id") is not None:
            enemy = self.env._get_enemy(it.get("target_id"))
            if enemy and enemy["state"] in (EState.APPROACHING, EState.MANEUVERING):
                return dist2d(it, enemy)
        if it["state"] == IState.RETURNING:
            h_idx = max(0, min(len(CFG.HANGAR_POSITIONS) - 1, it.get("hangar_idx", 0)))
            home = {"x": CFG.HANGAR_POSITIONS[h_idx], "y": CFG.INTERCEPT_FAIL_LINE + 200.0}
            return dist2d(it, home)
        return float("inf")

    def _priority_pair(self, a, b):
        rank_a = self._priority_rank(a)
        rank_b = self._priority_rank(b)
        if rank_a != rank_b:
            return (a, b) if rank_a > rank_b else (b, a)
        dist_a = self._mission_distance(a)
        dist_b = self._mission_distance(b)
        if abs(dist_a - dist_b) > 25.0:
            return (a, b) if dist_a < dist_b else (b, a)
        return (a, b) if a["id"] < b["id"] else (b, a)

    def _pair_key(self, a, b):
        return (min(a["id"], b["id"]), max(a["id"], b["id"]))

    # 计算无人机周围有多少架友军，评估局部空间的拥挤程度
    def _active_neighbors(self, it):
        neighbors = []
        for other in self.env.interceptors:
            if other["id"] == it["id"]:
                continue
            if other["state"] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING):
                continue
            if other["state"] == IState.LAUNCHING and self.env.time < other.get("launch_time", self.env.time):
                continue
            d = dist2d(it, other)
            if d > CFG.LOCAL_PLANNER_NEIGHBOR_DIST:
                continue
            neighbors.append((d, other))
        neighbors.sort(key=lambda item: item[0])
        return [other for _, other in neighbors[: CFG.LOCAL_PLANNER_MAX_NEIGHBORS]]

    def _neighbor_density_score(self, point, neighbors):
        radius = max(CFG.FRIENDLY_SAFE_SEPARATION, CFG.LOCAL_PLANNER_NEIGHBOR_DIST * 0.5)
        score = 0.0
        for other in neighbors:
            d = self._distance_xy(point, (other["x"], other["y"], other.get("z", 0.0)))
            if d >= radius:
                continue
            score += radius - d
        return score

    def _launch_wait_point(self, it):
        # 如果航道拥挤，计算无人机应该在什么坐标点悬停等待
        home_y = CFG.INTERCEPT_FAIL_LINE + 200.0
        lane = ((it["id"] % 3) - 1) * max(18.0, CFG.BASE_OUTBOUND_CORRIDOR_WIDTH * 0.18)
        return (
            max(0.0, min(CFG.AREA_WIDTH, CFG.HANGAR_POSITIONS[it.get("hangar_idx", 0)] + lane)),
            home_y,
            self.env._friendly_altitude(it, mission="launch", phase="initial"),
        )

    def _return_wait_point(self, it):
        gate = self.env._hangar_gate_point(it, outbound=False, z_value=self.env._friendly_altitude(it, mission="return", phase="return"))
        offset_x = ((it["id"] % 3) - 1) * max(20.0, CFG.BASE_INBOUND_CORRIDOR_WIDTH * 0.25)
        return (
            max(0.0, min(CFG.AREA_WIDTH, gate[0] + offset_x)),
            max(0.0, gate[1] - CFG.BASE_INBOUND_CORRIDOR_WIDTH * 1.2),
            gate[2],
        )

    # 检查起飞走廊是否空闲
    def can_release_launch(self, it):
        gate = self.env._hangar_gate_point(it, outbound=True, z_value=self.env._friendly_altitude(it, mission="launch", phase="initial"))
        half_width = CFG.BASE_OUTBOUND_CORRIDOR_WIDTH * 0.5
        release_y = gate[1] - max(50.0, CFG.BASE_OUTBOUND_CORRIDOR_WIDTH * 0.35)
        outbound_count = 0
        for other in self.env.interceptors:
            if other["id"] == it["id"]:
                continue
            if other.get("hangar_idx", -1) != it.get("hangar_idx", -2):
                continue
            if other["state"] == IState.RETURNING:
                if abs(other["x"] - gate[0]) <= half_width and other["y"] <= CFG.INTERCEPT_FAIL_LINE + CFG.BASE_INBOUND_CORRIDOR_WIDTH:
                    return False, "等待放行"
                continue
            if other["state"] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                continue
            if other.get("target_id") is None:
                continue
            if abs(other["x"] - gate[0]) > half_width:
                continue
            if other["y"] < release_y:
                continue
            outbound_count += 1
        return outbound_count < CFG.BASE_CORRIDOR_CAPACITY, "等待放行"

    # 检查降落走廊是否被占用
    def _return_gate_busy(self, it):
        gate = self.env._hangar_gate_point(it, outbound=False, z_value=self.env._friendly_altitude(it, mission="return", phase="return"))
        half_width = CFG.BASE_INBOUND_CORRIDOR_WIDTH * 0.5
        for other in self.env.interceptors:
            if other["id"] == it["id"]:
                continue
            if other.get("hangar_idx", -1) != it.get("hangar_idx", -2):
                continue
            if other["state"] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING):
                continue
            if other.get("target_id") is None:
                continue
            if abs(other["x"] - gate[0]) > half_width:
                continue
            if other["y"] < gate[1] - CFG.BASE_INBOUND_CORRIDOR_WIDTH * 0.7:
                continue
            return True
        return False

    def _candidate_barrier_points(self, it, zone, desired_point):
        z_value = self.env._cap_interceptor_altitude(it, max(it.get("z", 0.0), desired_point[2]))
        if self._point_in_zone((it["x"], it["y"], it.get("z", 0.0)), zone):
            side_y = it["y"]
        elif it["y"] >= zone["center"][1]:
            side_y = zone["ymax"] + CFG.BARRIER_BUFFER_MARGIN
        else:
            side_y = zone["ymin"] - CFG.BARRIER_BUFFER_MARGIN
        side_y = max(0.0, min(CFG.AREA_HEIGHT, side_y))
        return {
            -1: (
                max(0.0, zone["xmin"] - CFG.BARRIER_BUFFER_MARGIN),
                side_y,
                z_value,
            ),
            1: (
                min(CFG.AREA_WIDTH, zone["xmax"] + CFG.BARRIER_BUFFER_MARGIN),
                side_y,
                z_value,
            ),
        }

    # 计算当必须要穿过禁飞区时，从左边绕飞还是右边绕飞成本更低
    def _choose_barrier_side(self, it, zone, desired_point, neighbors, state, ignore_enemy_id=None):
        candidates = self._candidate_barrier_points(it, zone, desired_point)
        current_side = state.get("barrier_side", 0)
        side_lock = self.env.time < state.get("barrier_side_until", -1e9)
        best_side = 0
        best_point = desired_point
        best_cost = 1e12
        side_costs = {}

        start_pt = (it["x"], it["y"], it.get("z", 0.0))
        for side, point in candidates.items():
            density_cost = self._neighbor_density_score(point, neighbors)
            route_cost = self._distance_xy(start_pt, point) + 0.32 * self._distance_xy(point, desired_point)
            edge_penalty = 0.0
            if point[0] <= CFG.FRIENDLY_SAFE_SEPARATION or point[0] >= CFG.AREA_WIDTH - CFG.FRIENDLY_SAFE_SEPARATION:
                edge_penalty = CFG.ROUTE_SWITCH_MIN_GAIN_M * 0.4
            switch_penalty = CFG.ROUTE_SWITCH_MIN_GAIN_M if side_lock and current_side and current_side != side else 0.0
            route_penalty = 0.0
            if self.route_hits_barrier_zone(point, desired_point, ignore_enemy_id=ignore_enemy_id, zone_type="core"):
                route_penalty += CFG.ROUTE_SWITCH_MIN_GAIN_M * 2.2
            cost = route_cost + density_cost + edge_penalty + switch_penalty + route_penalty
            side_costs[side] = cost
            if cost < best_cost:
                best_cost = cost
                best_side = side
                best_point = point

        if current_side in side_costs and side_lock:
            keep_cost = side_costs[current_side]
            if keep_cost <= best_cost + CFG.ROUTE_SWITCH_MIN_GAIN_M:
                best_side = current_side
                best_point = candidates[current_side]
                best_cost = keep_cost

        if best_side:
            state["barrier_side"] = best_side
            state["barrier_side_until"] = self.env.time + CFG.ROUTE_HYSTERESIS_SEC

        return best_side, best_point, best_cost

    def barrier_safe_command_point(self, it, desired_point, ignore_enemy_id=None):
        zone = self._first_zone_on_route(
            (it["x"], it["y"], it.get("z", 0.0)),
            desired_point,
            ignore_enemy_id=ignore_enemy_id,
            zone_type="buffer",
        )
        if zone is None or it.get("barrier_slot") is not None:
            return desired_point
        state = self._planner_state(it["id"])
        side, point, _ = self._choose_barrier_side(it, zone, desired_point, self._active_neighbors(it), state, ignore_enemy_id=ignore_enemy_id)
        return point if side else desired_point

    # 为无人机生成绕开禁飞区的安全拐点
    def apply_barrier_detours(self, it, points, ignore_enemy_id=None):
        if it.get("barrier_slot") is not None:
            return points, False

        zones = self.active_barrier_zones(ignore_enemy_id=ignore_enemy_id, zone_type="buffer")
        if not zones or len(points) < 2:
            return points, False

        detoured = False
        route = [points[0]]
        for end_pt in points[1:]:
            start_pt = route[-1]
            hit_zone = None
            for zone in zones:
                if self._segment_hits_barrier_zone(start_pt, end_pt, zone):
                    hit_zone = zone
                    break
            if hit_zone is None:
                route.append(end_pt)
                continue

            side_y = hit_zone["ymax"] + CFG.BARRIER_BUFFER_MARGIN if start_pt[1] >= hit_zone["center"][1] else hit_zone["ymin"] - CFG.BARRIER_BUFFER_MARGIN
            side_x = hit_zone["xmin"] - CFG.BARRIER_BUFFER_MARGIN if start_pt[0] <= hit_zone["center"][0] else hit_zone["xmax"] + CFG.BARRIER_BUFFER_MARGIN
            side_x = max(0.0, min(CFG.AREA_WIDTH, side_x))
            side_y = max(0.0, min(CFG.AREA_HEIGHT, side_y))
            cruise_z = self.env._cap_interceptor_altitude(it, max(start_pt[2], end_pt[2], it.get("z", 0.0)))
            route.extend(
                [
                    (side_x, side_y, cruise_z),
                    (side_x, max(0.0, min(CFG.AREA_HEIGHT, (side_y + end_pt[1]) * 0.5)), cruise_z),
                ]
            )
            route.append(end_pt)
            detoured = True
        return route, detoured

    # 将避撞指令强制应用到无人机的速度和高度上，并清理过期指令
    def apply_interceptor_limits(self, it):
        directive = self.env.deconflict_cooldown.get(it["id"])
        if not directive:
            return
        if self.env.time >= directive.get("until", -1.0):
            self.env.deconflict_cooldown.pop(it["id"], None)
            return
        if directive.get("target_z") is not None:
            it["target_z"] = self.env._cap_interceptor_altitude(it, directive["target_z"])
        if directive.get("speed_cap") is not None:
            it["speed"] = min(it.get("speed", 0.0), directive["speed_cap"])

    def _cleanup_directives(self, active_ids):
        for iid, directive in list(self.env.deconflict_cooldown.items()):
            if iid not in active_ids or self.env.time >= directive.get("until", -1.0):
                self.env.deconflict_cooldown.pop(iid, None)
        for pair_key, until in list(self.pair_cooldown.items()):
            if self.env.time >= until:
                self.pair_cooldown.pop(pair_key, None)
        for iid, state in list(self.local_states.items()):
            if iid not in active_ids and self.env.time - state.get("last_plan_time", -1e9) > 12.0:
                self.local_states.pop(iid, None)

    def _enforce_barrier_keepout(self, active):
        edge_margin = max(12.0, CFG.BARRIER_CORE_KEEP_OUT_MARGIN * 1.5)
        for it in active:
            if it.get("barrier_slot") is not None:
                continue
            point = (it["x"], it["y"], it.get("z", 0.0))
            for zone in self.active_barrier_zones(ignore_enemy_id=it.get("target_id"), zone_type="core"):
                if not self._point_in_zone(point, zone):
                    continue
                dist_left = abs(it["x"] - zone["xmin"])
                dist_right = abs(zone["xmax"] - it["x"])
                push_left = dist_left <= dist_right
                push = min(
                    max(18.0, CFG.FRIENDLY_SAFE_SEPARATION * 0.38),
                    min(dist_left, dist_right) + edge_margin,
                )
                it["x"] += -push if push_left else push
                it["x"] = max(0.0, min(CFG.AREA_WIDTH, it["x"]))
                it["target_z"] = self.env._cap_interceptor_altitude(
                    it,
                    max(it.get("target_z", it.get("z", 0.0)), it.get("z", 0.0) + 4.0),
                )
                break

    def _predict_altitude(self, it, horizon):
        z_now = it.get("z", 0.0)
        target_z = it.get("target_z", z_now)
        climb_rate = it.get("climb_rate", 0.0)
        if climb_rate <= 0.0:
            return z_now + it.get("vz", 0.0) * horizon
        dz = target_z - z_now
        step = max(-climb_rate * horizon, min(climb_rate * horizon, dz))
        return z_now + step

    def _predict_point(self, it, horizon):
        heading = math.radians(it.get("heading", 0.0))
        speed = max(0.0, it.get("speed", 0.0))
        return (
            max(0.0, min(CFG.AREA_WIDTH, it["x"] + math.cos(heading) * speed * horizon)),
            max(0.0, min(CFG.AREA_HEIGHT, it["y"] + math.sin(heading) * speed * horizon)),
            self._predict_altitude(it, horizon),
        )

    # 预测两架无人机在未来1-3秒是否会发生碰撞
    def _predict_conflict(self, a, b):
        safe_sep = CFG.FRIENDLY_SAFE_SEPARATION
        current_d = dist2d(a, b)
        if current_d > safe_sep * 2.6:
            return None

        current_dz = abs(a.get("z", 0.0) - b.get("z", 0.0))
        if current_d <= safe_sep * 0.8 and current_dz <= CFG.INTERCEPTOR_ALT_LAYER_STEP * 2.0:
            return {"horizon": 0.0, "distance": current_d, "dz": current_dz}

        best = None
        vertical_limit = max(8.0, CFG.INTERCEPTOR_ALT_LAYER_STEP * 1.5)
        for horizon in (1.2, 2.1, 3.0):
            pa = self._predict_point(a, horizon)
            pb = self._predict_point(b, horizon)
            dxy = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            dz = abs(pa[2] - pb[2])
            if dxy > safe_sep or dz > vertical_limit:
                continue
            score = (safe_sep - dxy) + max(0.0, vertical_limit - dz) * 0.25
            if best is None or score > best["score"]:
                best = {"horizon": horizon, "distance": dxy, "dz": dz, "score": score}
        return best

    def _escape_altitude(self, it, other):
        step = max(6.0, CFG.INTERCEPTOR_ALT_LAYER_STEP * 1.6)
        current_target = it.get("target_z", it.get("z", 0.0))
        lower_room = current_target
        upper_room = it.get("z_cap", CFG.INTERCEPTOR_MAX_ALT) - current_target

        if it["state"] == IState.RETURNING and other["state"] != IState.RETURNING:
            direction = -1.0
        elif other["state"] == IState.RETURNING and it["state"] != IState.RETURNING:
            direction = 1.0
        elif it["state"] == IState.LAUNCHING:
            direction = 1.0
        elif it.get("role") == IRole.FOLLOWER:
            direction = -1.0
        else:
            direction = -1.0 if current_target > other.get("target_z", other.get("z", 0.0)) else 1.0

        if direction < 0.0 and lower_room < step * 0.75:
            direction = 1.0
        elif direction > 0.0 and upper_room < step * 0.75:
            direction = -1.0

        return self.env._cap_interceptor_altitude(it, current_target + direction * step)

    def _speed_cap(self, it, conflict):
        base_speed = max(0.0, it.get("speed", 0.0))
        if it["state"] == IState.LAUNCHING:
            return min(base_speed, max(4.0, CFG.INTERCEPTOR_SPEED * 0.45))
        if base_speed <= 0.0:
            return None
        factor = 0.78
        if conflict["horizon"] <= 1.2:
            factor = 0.58
        return max(6.0, base_speed * factor)

    def _store_directive(self, it, keep_it, conflict):
        hold_time = max(0.8, min(2.8, conflict["horizon"] + 0.8))
        self.env.deconflict_cooldown[it["id"]] = {
            "until": self.env.time + hold_time,
            "target_z": self._escape_altitude(it, keep_it),
            "speed_cap": self._speed_cap(it, conflict),
            "yield_to": keep_it["id"],
        }

    # 如果预测会相撞，强制生成一条指令
    def _predictive_resolve(self, a, b, conflict):
        pair_key = self._pair_key(a, b)
        if pair_key in self.pair_cooldown:
            return

        keep_it, yield_it = self._priority_pair(a, b)
        self._store_directive(yield_it, keep_it, conflict)

        side = 1.0 if (yield_it["id"] + keep_it["id"]) % 2 == 0 else -1.0
        lateral = max(12.0, CFG.FORMATION_SPACING * 0.08)
        sep_x = yield_it["x"] - keep_it["x"]
        sep_y = yield_it["y"] - keep_it["y"]
        norm = math.hypot(sep_x, sep_y)
        if norm < 1e-6:
            heading = math.radians(keep_it.get("heading", 0.0))
            px = -math.sin(heading)
            py = math.cos(heading)
        else:
            px = -sep_y / norm
            py = sep_x / norm
        yield_it["x"] = max(0.0, min(CFG.AREA_WIDTH, yield_it["x"] + px * side * lateral))
        yield_it["y"] = max(0.0, min(CFG.AREA_HEIGHT, yield_it["y"] + py * side * lateral))
        self.pair_cooldown[pair_key] = self.env.time + max(0.5, conflict["horizon"] * 0.6)

    def _local_plan_reuse_ok(self, state, desired_point, mission_key):
        plan = state.get("last_plan")
        if plan is None:
            return False
        if state.get("mission_key") != mission_key:
            return False
        if self.env.time - state.get("last_plan_time", -1e9) >= CFG.LOCAL_PLANNER_REPLAN_SEC:
            return False
        last_desired = state.get("last_desired")
        if last_desired is None:
            return False
        if self._distance_xy(last_desired, desired_point) > CFG.ROUTE_SWITCH_MIN_GAIN_M * 0.45:
            return False
        return True

    def _store_local_plan(self, it, desired_point, mission_key, plan):
        state = self._planner_state(it["id"])
        state["last_plan_time"] = self.env.time
        state["last_plan"] = plan
        state["last_desired"] = desired_point
        state["last_command"] = plan.command_point
        state["mission_key"] = mission_key
        if plan.hold_reason:
            if state.get("hold_reason") != plan.hold_reason:
                state["hold_started_at"] = self.env.time
            state["hold_reason"] = plan.hold_reason
        else:
            state["hold_reason"] = ""
            state["hold_started_at"] = -1e9
        state["last_avoid_mode"] = plan.avoid_mode
        return plan

    def _barrier_detour_plan(self, it, desired_point, mission_ctx, neighbors, state):
        if it.get("barrier_slot") is not None:
            return desired_point, "", ""
        ignore_enemy_id = mission_ctx.get("allow_barrier_enemy_id")
        start_pt = (it["x"], it["y"], it.get("z", 0.0))

        core_zone = self._first_zone_on_route(start_pt, desired_point, ignore_enemy_id=ignore_enemy_id, zone_type="core")
        if core_zone is not None:
            side, point, _ = self._choose_barrier_side(it, core_zone, desired_point, neighbors, state, ignore_enemy_id=ignore_enemy_id)
            if side:
                if self._point_in_zone(start_pt, core_zone):
                    return point, "紧急脱网", ""
                return point, "绕网", ""

        buffer_zone = self._first_zone_on_route(start_pt, desired_point, ignore_enemy_id=ignore_enemy_id, zone_type="buffer")
        if buffer_zone is None:
            return desired_point, "", ""

        side, point, cost = self._choose_barrier_side(it, buffer_zone, desired_point, neighbors, state, ignore_enemy_id=ignore_enemy_id)
        if not side:
            return desired_point, "", ""

        density_cost = self._neighbor_density_score(point, neighbors)
        wait_threshold = max(CFG.FRIENDLY_SAFE_SEPARATION * 0.9, 180.0)
        if density_cost > wait_threshold and mission_ctx.get("kind") in ("hit", "follow", "search"):
            wait_y = buffer_zone["ymax"] + CFG.BARRIER_BUFFER_MARGIN if it["y"] >= buffer_zone["center"][1] else buffer_zone["ymin"] - CFG.BARRIER_BUFFER_MARGIN
            wait_pt = (
                max(0.0, min(CFG.AREA_WIDTH, it["x"])),
                max(0.0, min(CFG.AREA_HEIGHT, wait_y)),
                point[2],
            )
            return wait_pt, "等待放行", "等待放行"

        return point, "绕网", ""

    # 基于人工势场法，微调无人机速度向量，使其滑过友军
    def _adjust_velocity(self, it, command_point, desired_speed, neighbors):
        if desired_speed <= 0.05:
            return command_point, 0.0, ""

        dx = command_point[0] - it["x"]
        dy = command_point[1] - it["y"]
        dist_to_goal = math.hypot(dx, dy)
        if dist_to_goal < 1e-6:
            return command_point, 0.0, ""

        dir_x, dir_y = self._normalize(dx, dy)
        vx = dir_x * desired_speed
        vy = dir_y * desired_speed
        speed_factor = 1.0
        avoid_active = False
        min_sep = max(CFG.FRIENDLY_SAFE_SEPARATION * 0.9, CFG.FRIENDLY_COLLISION_RADIUS * 2.2)

        for other in neighbors:
            rel_x = other["x"] - it["x"]
            rel_y = other["y"] - it["y"]
            rel_dist = math.hypot(rel_x, rel_y)
            if rel_dist > CFG.LOCAL_PLANNER_NEIGHBOR_DIST:
                continue

            ovx, ovy = self._velocity_vector(other)
            rvx = vx - ovx
            rvy = vy - ovy
            denom = rvx * rvx + rvy * rvy + 1e-6
            t_closest = -(rel_x * rvx + rel_y * rvy) / denom
            t_closest = max(0.0, min(t_closest, CFG.LOCAL_PLANNER_TIME_HORIZON_SEC))
            closest_x = rel_x + rvx * t_closest
            closest_y = rel_y + rvy * t_closest
            closest_d = math.hypot(closest_x, closest_y)

            if closest_d >= min_sep and rel_dist >= min_sep * 1.05:
                continue

            rep_x, rep_y = self._normalize(-closest_x if abs(closest_x) > 1e-6 else -rel_x, -closest_y if abs(closest_y) > 1e-6 else -rel_y)
            side = 1.0 if self._pair_key(it, other)[0] == it["id"] else -1.0
            tan_x, tan_y = -rep_y * side, rep_x * side
            yield_scale = 1.0 if self._priority_rank(it) <= self._priority_rank(other) else 0.6
            strength = min(1.2, max(0.2, (min_sep - min(closest_d, rel_dist)) / max(min_sep, 1.0) + 0.35))
            vx += (rep_x * 0.75 + tan_x * 0.25) * desired_speed * strength * yield_scale
            vy += (rep_y * 0.75 + tan_y * 0.25) * desired_speed * strength * yield_scale
            speed_factor = min(speed_factor, 0.58 if yield_scale > 0.9 else 0.8)
            avoid_active = True

        dir_x, dir_y = self._normalize(vx, vy)
        if abs(dir_x) < 1e-6 and abs(dir_y) < 1e-6:
            return command_point, 0.0, "避障" if avoid_active else ""

        lookahead = max(38.0, desired_speed * max(CFG.LOCAL_PLANNER_REPLAN_SEC, CFG.DT) * 3.6)
        step = min(dist_to_goal, lookahead)
        smoothed_point = (
            max(0.0, min(CFG.AREA_WIDTH, it["x"] + dir_x * step)),
            max(0.0, min(CFG.AREA_HEIGHT, it["y"] + dir_y * step)),
            command_point[2],
        )
        return smoothed_point, speed_factor, "避障" if avoid_active else ""

    # 确保生成的局部指令是平滑的，不会让飞机来回抖动
    def _smooth_command_point(self, state, command_point, avoid_mode, hold_reason):
        prev = state.get("last_command")
        if prev is None or hold_reason == "等待超时" or avoid_mode in ("紧急脱网", "绕网", "等待放行"):
            return command_point
        alpha = 0.72 if not avoid_mode else 0.52
        return (
            prev[0] * (1.0 - alpha) + command_point[0] * alpha,
            prev[1] * (1.0 - alpha) + command_point[1] * alpha,
            prev[2] * (1.0 - alpha) + command_point[2] * alpha,
        )

    # 单机局部规划总入口
    def plan_local_motion(self, it, desired_point, desired_speed, mission_ctx):
        """单架 UAV 局部运动规划入口。

        输入是任务层给出的 desired_point/desired_speed；输出 LocalPlan，包含
        实际可执行 command_point、speed_cap、等待原因和避让模式。这里集中处理
        起降管制、禁飞区绕飞、友机密度等待、RVO 微调和平滑，避免上层分配器
        关心低层航线约束。
        """
        # 约束高度不能超过系统上限
        desired_point = (
            max(0.0, min(CFG.AREA_WIDTH, desired_point[0])),
            max(0.0, min(CFG.AREA_HEIGHT, desired_point[1])),
            self.env._cap_interceptor_altitude(it, desired_point[2]),
        )
        desired_speed = max(0.0, float(desired_speed))
        mission_key = (
            mission_ctx.get("kind"),
            it.get("target_id"),
            it.get("barrier_slot"),
            it.get("net_slot"),
        )
        state = self._planner_state(it["id"])

        if not CFG.LOCAL_PLANNER_ENABLE:
            return LocalPlan(
                command_point=desired_point,
                speed_cap=desired_speed,
                target_z=desired_point[2],
                allow_terminal_direct=bool(mission_ctx.get("allow_terminal_direct")),
            )

        if self._local_plan_reuse_ok(state, desired_point, mission_key):
            # 目标点和任务签名没有明显变化时复用上一版，减少指令抖动。
            return state["last_plan"]

        neighbors = self._active_neighbors(it)
        plan = LocalPlan(
            command_point=desired_point,
            speed_cap=desired_speed,
            target_z=desired_point[2],
            allow_terminal_direct=bool(mission_ctx.get("allow_terminal_direct")),
        )

        # 如果存在全局避撞系统下发的“强制避让指令”，优先服从指令（如强制减速或改变高度）
        directive = self.env.deconflict_cooldown.get(it["id"])
        if directive and self.env.time < directive.get("until", -1.0):
            if directive.get("target_z") is not None:
                plan.target_z = self.env._cap_interceptor_altitude(it, directive["target_z"])
            if directive.get("speed_cap") is not None:
                plan.speed_cap = min(plan.speed_cap, directive["speed_cap"])

        # 起降交通管制
        if mission_ctx.get("kind") == "launch":
            clear, reason = self.can_release_launch(it) # 检查走廊是否拥挤
            if not clear:
                # 走廊堵塞，生成一个原地等待点，速度设为0
                plan.command_point = self._launch_wait_point(it)
                plan.speed_cap = 0.0
                plan.hold_reason = reason
                plan.avoid_mode = "等待放行"
                plan.allow_terminal_direct = False
                return self._store_local_plan(it, desired_point, mission_key, plan)

        if mission_ctx.get("kind") == "return" and self._return_gate_busy(it):
            plan.command_point = self._return_wait_point(it)
            plan.speed_cap = min(desired_speed, max(6.0, CFG.INTERCEPTOR_SPEED * 0.45))
            plan.hold_reason = "等待放行"
            plan.avoid_mode = "等待放行"
            plan.allow_terminal_direct = False
            return self._store_local_plan(it, desired_point, mission_key, plan)

        # 禁飞区(扯网区)绕飞规划
        barrier_point, barrier_mode, barrier_hold = self._barrier_detour_plan(it, desired_point, mission_ctx, neighbors, state)
        if barrier_point != desired_point:
            # 强制拐弯
            plan.command_point = barrier_point 
            plan.allow_terminal_direct = False
        if barrier_mode:
            plan.avoid_mode = barrier_mode
        if barrier_hold:
            plan.hold_reason = barrier_hold
            plan.speed_cap = 0.0

        # 对局部空域过密的情况先等待，避免多机同时挤向同一指令点。
        density_score = self._neighbor_density_score(plan.command_point, neighbors)
        dense_threshold = max(CFG.FRIENDLY_SAFE_SEPARATION * 1.1, 190.0)
        threat_y = mission_ctx.get("threat_y", 0.0)
        if (
            density_score >= dense_threshold
            and mission_ctx.get("kind") in ("hit", "follow", "search")
            and not plan.hold_reason
            and threat_y < CFG.INTERCEPT_FAIL_LINE * 0.72
        ):
            plan.command_point = (it["x"], it["y"], plan.target_z)
            plan.speed_cap = 0.0
            plan.hold_reason = "等待放行"
            plan.avoid_mode = "等待放行"
            plan.allow_terminal_direct = False

        # 微观速度调整 (RVO（相对速度）避障：通过计算周围飞机的速度向量，微微推开当前飞机的航向)
        if not plan.hold_reason or (plan.speed_cap is not None and plan.speed_cap > 0.0):
            adjusted_point, speed_factor, rvo_mode = self._adjust_velocity(it, plan.command_point, desired_speed, neighbors)
            plan.command_point = adjusted_point
            plan.speed_cap = min(plan.speed_cap, desired_speed * speed_factor) if plan.speed_cap is not None else desired_speed * speed_factor
            if rvo_mode:
                plan.avoid_mode = plan.avoid_mode or rvo_mode
                plan.allow_terminal_direct = False

        if plan.hold_reason and mission_ctx.get("kind") in ("hit", "follow", "search"):
            hold_started_at = state.get("hold_started_at", -1e9)
            if hold_started_at > -1e8 and self.env.time - hold_started_at >= CFG.BARRIER_WAIT_TIMEOUT_SEC:
                plan.hold_reason = "等待超时"
                plan.avoid_mode = "等待放行"
                plan.allow_terminal_direct = False

        # 轨迹平滑滤波 (防止指令突变导致飞机剧烈摇晃)
        plan.command_point = self._smooth_command_point(state, plan.command_point, plan.avoid_mode, plan.hold_reason)
        if self._first_zone_on_route(
            (it["x"], it["y"], it.get("z", 0.0)),
            plan.command_point,
            ignore_enemy_id=mission_ctx.get("allow_barrier_enemy_id"),
            zone_type="core",
        ) is not None:
            plan.command_point = self.barrier_safe_command_point(
                it,
                plan.command_point,
                ignore_enemy_id=mission_ctx.get("allow_barrier_enemy_id"),
            )
            plan.avoid_mode = plan.avoid_mode or "绕网"
            plan.allow_terminal_direct = False
        return self._store_local_plan(it, desired_point, mission_key, plan)

    # 全局强制避撞入口
    def deconflict_interceptors(self):
        """
                全局强制避撞与预测性冲突消解。
                处理紧急情况：如果两架飞机已经靠得极近，直接在物理坐标上将它们推开。
                """
        active = []
        for it in self.env.interceptors:
            if it["state"] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING):
                continue
            if it["state"] == IState.LAUNCHING and self.env.time < it.get("launch_time", self.env.time):
                continue
            active.append(it)

        active_ids = {it["id"] for it in active}
        self._cleanup_directives(active_ids)
        self._enforce_barrier_keepout(active)

        # 第一层防御：已经发生或即将发生硬重叠，强行物理推开
        for idx in range(len(active)):
            for jdx in range(idx + 1, len(active)):
                a = active[idx]
                b = active[jdx]
                d = dist2d(a, b)
                # 如果距离大于安全距离，不处理
                if d >= CFG.FRIENDLY_SAFE_SEPARATION - 1.0:
                    continue
                if d < 1e-6:
                    ux, uy = 1.0, 0.0
                else:
                    # 计算推开的排斥力向量
                    ux = (b["x"] - a["x"]) / d
                    uy = (b["y"] - a["y"]) / d
                push = (CFG.FRIENDLY_SAFE_SEPARATION - d) * 0.5
                # 强行修改物理坐标：a往反方向推，b往正方向推
                a["x"] = max(0.0, min(CFG.AREA_WIDTH, a["x"] - ux * push))
                a["y"] = max(0.0, min(CFG.AREA_HEIGHT, a["y"] - uy * push))
                b["x"] = max(0.0, min(CFG.AREA_WIDTH, b["x"] + ux * push))
                b["y"] = max(0.0, min(CFG.AREA_HEIGHT, b["y"] + uy * push))

                # 强行分配高度层：a往上飞，b往下飞，实现三维错开
                a["target_z"] = self.env._cap_interceptor_altitude(
                    a,
                    max(a.get("target_z", a.get("z", 0.0)), a.get("z", 0.0) + 8.0),
                )
                b["target_z"] = self.env._cap_interceptor_altitude(
                    b,
                    max(0.0, b.get("target_z", b.get("z", 0.0)) - 8.0),
                )
                self._predictive_resolve(a, b, {"horizon": 0.0, "distance": d, "dz": abs(a.get("z", 0.0) - b.get("z", 0.0))})

        # 第二层防御：未来轨迹预测。如果现在没撞，但预测 3 秒后会撞
        for idx in range(len(active)):
            for jdx in range(idx + 1, len(active)):
                a = active[idx]
                b = active[jdx]
                if dist2d(a, b) > CFG.FRIENDLY_SAFE_SEPARATION * 2.6:
                    continue
                # 预测未来冲突
                conflict = self._predict_conflict(a, b)
                if not conflict:
                    continue
                # 判定路权，生成减速或高度规避指令存入 self.env.deconflict_cooldown
                self._predictive_resolve(a, b, conflict)
