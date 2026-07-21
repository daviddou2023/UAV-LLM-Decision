from marl_common import CFG, EState, IRole, IState, dist2d


class StableRetaskPolicy:
    def __init__(self, assigner):
        self.assigner = assigner

    def reconcile(self, interceptors, enemies, active):
        msgs = []
        msgs.extend(self._audit_assignments(interceptors))
        msgs.extend(self._replace_infeasible_primaries(interceptors, enemies))
        msgs.extend(self._close_resolved_assignments(interceptors, enemies, active))
        msgs.extend(self._promote_followers(interceptors))
        return msgs

    def _audit_assignments(self, interceptors):
        msgs = []
        for eid, asgn in list(self.assigner.assignments.items()):
            for role in ("primary", "follower"):
                iid = asgn.get(role)
                if iid is None:
                    continue
                it = next((item for item in interceptors if item["id"] == iid), None)
                if (
                    it
                    and it["state"] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
                    and not it.get("jammed_by_interference")
                ):
                    continue
                asgn[role] = None
                if role == "primary":
                    msgs.append(f"警告: I-{iid+1} 脱离任务，F-{eid+1} 重新排队")
        return msgs

    def _replace_infeasible_primaries(self, interceptors, enemies):
        msgs = []
        avail_pool = [
            item for item in interceptors
            if item["state"] in (IState.STANDBY, IState.RETURNING) and item["target_id"] is None
            and not item.get("jammed_by_interference")
            and not item.get("task_reserved")
        ]

        for eid, asgn in list(self.assigner.assignments.items()):
            pid = asgn.get("primary")
            if pid is None:
                continue

            pri = next((item for item in interceptors if item["id"] == pid), None)
            en = next((enemy for enemy in enemies if enemy["id"] == eid), None)
            if not pri or not en or pri["state"] not in (IState.INTERCEPTING, IState.FOLLOWING):
                continue
            if pri.get("fuel", 0.0) <= 30.0:
                continue
            if dist2d(pri, en) <= CFG.TERMINAL_GUIDE_RANGE:
                continue

            curr_poi, _ = self.assigner.compute_poi(pri, en)
            if self.assigner.is_feasible(curr_poi):
                continue

            replacement = None
            for cand in avail_pool:
                c_poi, _ = self.assigner.compute_poi(cand, en)
                if self.assigner.is_feasible(c_poi):
                    replacement = cand
                    break

            if replacement is None:
                continue

            pri["state"] = IState.RETURNING
            pri["target_id"] = None
            pri["role"] = IRole.RESERVE
            asgn["primary"] = None
            msgs.append(f"I-{pid+1} 当前POI越界，改由替补接手 F-{eid+1}")
            avail_pool.append(pri)
            if replacement in avail_pool:
                avail_pool.remove(replacement)

        return msgs

    def _close_resolved_assignments(self, interceptors, enemies, active):
        msgs = []
        for eid in list(self.assigner.assignments):
            en = next((enemy for enemy in enemies if enemy["id"] == eid), None)
            if en and en["state"] not in (EState.DESTROYED, EState.PENETRATED):
                continue

            asgn = self.assigner.assignments.pop(eid)
            for role in ("primary", "follower"):
                iid = asgn.get(role)
                if iid is None:
                    continue
                it = next((item for item in interceptors if item["id"] == iid), None)
                if not it or it["state"] in (IState.DESTROYED, IState.LANDED):
                    continue
                if it["fuel"] > 50 and not it.get("jammed_by_interference"):
                    new_target_id, new_role = self.assigner._try_hot_reassign(it, active)
                    if new_target_id is not None:
                        self.assigner._bind_assignment(new_target_id, it["id"], new_role)
                        it["state"] = IState.INTERCEPTING
                        it["target_id"] = new_target_id
                        it["role"] = new_role
                        msgs.append(f"I-{iid+1} 热接力 -> F-{new_target_id+1}")
                        continue
                it["state"] = IState.RETURNING
                it["target_id"] = None
                it["role"] = IRole.RESERVE
                msgs.append(f"I-{iid+1} 返航")
        return msgs

    def _promote_followers(self, interceptors):
        msgs = []
        for eid, asgn in list(self.assigner.assignments.items()):
            if asgn["primary"] is not None or asgn["follower"] is None:
                continue

            fid = asgn["follower"]
            fi = next((item for item in interceptors if item["id"] == fid), None)
            if fi and fi["state"] not in (IState.DESTROYED, IState.LANDED, IState.RETURNING):
                asgn["primary"] = fid
                asgn["follower"] = None
                fi["role"] = IRole.PRIMARY
                fi["state"] = IState.INTERCEPTING
                msgs.append(f"I-{fid+1} 补位主拦截")
            else:
                asgn["follower"] = None
        return msgs
