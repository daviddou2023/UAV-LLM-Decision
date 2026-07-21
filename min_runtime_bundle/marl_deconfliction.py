import math

from marl_common import CFG, EState, IState, dist2d


class DeconflictionController:
    def __init__(self, env):
        self.env = env

    def _segment_hits_barrier_zone(self, start_pt, end_pt, zone):
        for idx in range(13):
            t = idx / 12.0
            px = start_pt[0] + (end_pt[0] - start_pt[0]) * t
            py = start_pt[1] + (end_pt[1] - start_pt[1]) * t
            if zone["xmin"] <= px <= zone["xmax"] and zone["ymin"] <= py <= zone["ymax"]:
                return True
        return False

    def active_barrier_zones(self, ignore_enemy_id=None):
        zones = []
        margin = max(CFG.FRIENDLY_SAFE_SEPARATION, 90.0)
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
                }
            )
        return zones

    def apply_barrier_detours(self, it, points, ignore_enemy_id=None):
        if it.get("barrier_slot") is not None:
            return points, False

        zones = self.active_barrier_zones(ignore_enemy_id=ignore_enemy_id)
        if not zones or len(points) < 2:
            return points, False

        detoured = False
        route = [points[0]]
        for end_pt in points[1:]:
            start_pt = route[-1]
            for zone in zones:
                if not self._segment_hits_barrier_zone(start_pt, end_pt, zone):
                    continue
                side_x = zone["xmin"] - 40.0 if start_pt[0] <= zone["center"][0] else zone["xmax"] + 40.0
                side_x = max(0.0, min(CFG.AREA_WIDTH, side_x))
                cruise_z = self.env._cap_interceptor_altitude(it, max(start_pt[2], end_pt[2], it.get("z", 0.0)))
                detour_1 = (
                    side_x,
                    max(0.0, min(CFG.INTERCEPT_FAIL_LINE + 150.0, (start_pt[1] + zone["center"][1]) * 0.5)),
                    cruise_z,
                )
                detour_2 = (
                    side_x,
                    max(0.0, min(CFG.INTERCEPT_FAIL_LINE + 150.0, (zone["center"][1] + end_pt[1]) * 0.5)),
                    cruise_z,
                )
                route.extend([detour_1, detour_2])
                detoured = True
                break
            route.append(end_pt)
        return route, detoured

    def deconflict_interceptors(self):
        active = []
        for it in self.env.interceptors:
            if it["state"] not in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING):
                continue
            if it["state"] == IState.LAUNCHING and self.env.time < it.get("launch_time", self.env.time):
                continue
            active.append(it)

        for idx in range(len(active)):
            for jdx in range(idx + 1, len(active)):
                a = active[idx]
                b = active[jdx]
                d = dist2d(a, b)
                if d >= CFG.FRIENDLY_SAFE_SEPARATION - 1.0:
                    continue
                if d < 1e-6:
                    ux, uy = 1.0, 0.0
                else:
                    ux = (b["x"] - a["x"]) / d
                    uy = (b["y"] - a["y"]) / d
                push = (CFG.FRIENDLY_SAFE_SEPARATION - d) * 0.5
                a["x"] = max(0.0, min(CFG.AREA_WIDTH, a["x"] - ux * push))
                a["y"] = max(0.0, min(CFG.AREA_HEIGHT, a["y"] - uy * push))
                b["x"] = max(0.0, min(CFG.AREA_WIDTH, b["x"] + ux * push))
                b["y"] = max(0.0, min(CFG.AREA_HEIGHT, b["y"] + uy * push))
                a["target_z"] = self.env._cap_interceptor_altitude(
                    a,
                    max(a.get("target_z", a.get("z", 0.0)), a.get("z", 0.0) + 8.0),
                )
                b["target_z"] = self.env._cap_interceptor_altitude(
                    b,
                    max(0.0, b.get("target_z", b.get("z", 0.0)) - 8.0),
                )
