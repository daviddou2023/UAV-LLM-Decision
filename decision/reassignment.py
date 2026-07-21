from core.common import CFG, EState, IRole, IState, dist2d


class StableRetaskPolicy:
    """
        稳定的任务重分配策略类。
        主要用于在每一帧任务分配前，清理无效任务、纠正错误分配、回收可用兵力。

        在 decision/cooperation.py 的 InterceptionAssigner 的构造函数 __init__ 中实例化StableRetaskPolicy
        在 simulation/main.py 开启强干扰（demo_interference_enable）时，会有大量无人机进入干扰区变成 jammed_by_interference = True
        decision/reassignment.py 里面的 _audit_assignments（第一步异常审计），一旦 simulation/main.py 给无人机打上受干扰标签，这个文件就会立刻察觉，把脱网的无人机踢出任务表，从而触发主程序的“大模型态势重构”逻辑


        """
    def __init__(self, assigner):
        self.assigner = assigner

    def reconcile(self, interceptors, enemies, active):
        """
            【主调度方法】大扫除。依次执行四步审计，收集并返回所有的操作日志。
        """
        msgs = []
        msgs.extend(self._audit_assignments(interceptors))
        msgs.extend(self._replace_infeasible_primaries(interceptors, enemies))
        msgs.extend(self._close_resolved_assignments(interceptors, enemies, active))
        msgs.extend(self._promote_followers(interceptors))
        return msgs

    def _audit_assignments(self, interceptors):
        """第一步：异常审计（踢出掉线/受扰的无人机）"""
        msgs = []
        # 遍历所有敌机的任务分配情况
        for eid, asgn in list(self.assigner.assignments.items()):
            for role in ("primary", "follower"):
                iid = asgn.get(role)
                if iid is None:
                    continue
                # 找到正在执行该任务的无人机
                it = next((item for item in interceptors if item["id"] == iid), None)
                # 如果无人机状态正常且没有受到强干扰，继续保持分配
                if (
                    it
                    and it["state"] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
                    and not it.get("jammed_by_interference")
                ):
                    continue
                # 否则，强行解除它的任务绑定
                asgn[role] = None
                if role == "primary":
                    msgs.append(f"警告: I-{iid+1} 脱离任务，F-{eid+1} 重新排队")
        return msgs

    def _replace_infeasible_primaries(self, interceptors, enemies):
        """第二步：越界纠错与动态替补（追不上就赶紧换人）"""
        msgs = []
        # 找出现有的空闲兵力（待命或正在返航，且未受扰、未被大模型预留）
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
            # 没油了就不考虑换人了，直接等它自己触发返航逻辑
            if pri.get("fuel", 0.0) <= 30.0:
                continue
            # 如果已经进入末端制导距离，死磕到底，不再换人
            if dist2d(pri, en) <= CFG.TERMINAL_GUIDE_RANGE:
                continue
            # 计算当前主拦截机的相遇点
            curr_poi, _ = self.assigner.compute_poi(pri, en)
            # 如果相遇点还在防线外（可行），继续执行
            if self.assigner.is_feasible(curr_poi):
                continue

            # 如果当前主拦截机已经追不上（相遇点越过防线），去待命池里找替补
            replacement = None
            for cand in avail_pool:
                c_poi, _ = self.assigner.compute_poi(cand, en)
                # 如果替补能成功拦截
                if self.assigner.is_feasible(c_poi):
                    replacement = cand
                    break

            if replacement is None:
                continue
            # 找到替补，将原来的无人机召回
            pri["state"] = IState.RETURNING
            pri["target_id"] = None
            pri["role"] = IRole.RESERVE
            asgn["primary"] = None
            msgs.append(f"I-{pid+1} 当前POI越界，改由替补接手 F-{eid+1}")
            # 维护待命池
            avail_pool.append(pri)
            if replacement in avail_pool:
                avail_pool.remove(replacement)

        return msgs

    def _close_resolved_assignments(self, interceptors, enemies, active):
        """第三步：任务结算与热接力（打完不要急着回家，看看周围还有没有敌人）"""
        msgs = []
        for eid in list(self.assigner.assignments):
            en = next((enemy for enemy in enemies if enemy["id"] == eid), None)
            # 如果敌机还在飞，跳过
            if en and en["state"] not in (EState.DESTROYED, EState.PENETRATED):
                continue

            # 敌机已被击毁或突防，清理分配表
            asgn = self.assigner.assignments.pop(eid)
            for role in ("primary", "follower"):
                iid = asgn.get(role)
                if iid is None:
                    continue
                it = next((item for item in interceptors if item["id"] == iid), None)
                if not it or it["state"] in (IState.DESTROYED, IState.LANDED):
                    continue
                # 【热接力逻辑】如果电量大于50且通信正常
                if it["fuel"] > 50 and not it.get("jammed_by_interference"):
                    # 尝试直接寻找下一个目标
                    new_target_id, new_role = self.assigner._try_hot_reassign(it, active)
                    if new_target_id is not None:
                        # 找到后，直接在空中调转枪头
                        self.assigner._bind_assignment(new_target_id, it["id"], new_role)
                        it["state"] = IState.INTERCEPTING
                        it["target_id"] = new_target_id
                        it["role"] = new_role
                        msgs.append(f"I-{iid+1} 热接力 -> F-{new_target_id+1}")
                        continue
                # 如果没油了，或者没找到新目标，则返航
                it["state"] = IState.RETURNING
                it["target_id"] = None
                it["role"] = IRole.RESERVE
                msgs.append(f"I-{iid+1} 返航")
        return msgs

    def _promote_followers(self, interceptors):
        """第四步：备胎转正（主拦截机掉了，备份机顶上）"""
        msgs = []
        for eid, asgn in list(self.assigner.assignments.items()):
            # 如果主拦截机还在，或者根本没有备份机，跳过
            if asgn["primary"] is not None or asgn["follower"] is None:
                continue

            fid = asgn["follower"]
            fi = next((item for item in interceptors if item["id"] == fid), None)
            # 如果备份机活着且在空中，提拔为主拦截机
            if fi and fi["state"] not in (IState.DESTROYED, IState.LANDED, IState.RETURNING):
                asgn["primary"] = fid
                asgn["follower"] = None
                fi["role"] = IRole.PRIMARY
                fi["state"] = IState.INTERCEPTING
                msgs.append(f"I-{fid+1} 补位主拦截")
            else:
                # 备份机也死了，清空
                asgn["follower"] = None
        return msgs
