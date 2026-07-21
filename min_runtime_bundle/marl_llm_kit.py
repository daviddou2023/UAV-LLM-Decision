"""
MARL LLM 工具包 - 战场态势分析器 v4.1 (小模型适配版)
修复:
1. [核心] 上下文精简为结构化短文本，防止小模型编造不存在的编号
2. [核心] system prompt 用硬性格式约束，降低废话概率
3. [核心] max_tokens 严格压低 + 超长强制截断兜底
4. 保留逐目标编号 (E-1, E-2...)，但只列最危险的前3个
"""
import threading
import json
import math
import time
import requests
from marl_common import CFG, IState, EState, IRole, EType

# === 敌机类型简称 ===
_ET = {
    EType.NORMAL: "常规", EType.SNAKE: "S机动", EType.JINK: "闪避",
    EType.DASH: "高速", EType.LOITER: "巡飞弹", EType.DECOY: "诱饵",
}

def _sector(x):
    if x < CFG.AREA_WIDTH * 0.3: return "左翼"
    if x > CFG.AREA_WIDTH * 0.7: return "右翼"
    return "中路"


class BattlefieldAnalyst:
    def __init__(self):
        # ==========================================
        # vLLM 本地连接
        # ==========================================
        self.api_key = "EMPTY"
        self.base_url = "http://localhost:8000/v1"
        self.model = "Qwen-Agent"
        # ==========================================

        # =============================================
        # System Prompt - 警报通道 (极简)
        # =============================================
        self.system_prompt_analysis = (
            "你是INET实验室研发的防空预警节点，代号Sentinel。\n"
            "规则：根据战场数据用一句话警报，限40字，必须点名最危险目标编号，给一条行动建议。\n"
            "示例：紧急！F-5巡飞弹距防线800m无人拦截，建议调左翼待命机。"
        )

        # =============================================
        # System Prompt - 对话通道
        # =============================================
        self.system_prompt_chat = (
            "你是INET实验室专属研发的智能战术副官，代号Sentinel。\n"
            "身份规则：当被问到'你是谁'或'谁制作的'时，必须回答'我是INET实验室研发的战术副官Sentinel'。\n"
            "战术规则：\n"
            "1. 用数据中的真实编号(F-1,I-3等)回答，禁止编造不存在的编号\n"
            "2. 返航≠减员，交战≠损失，注意区分\n"
            "3. 限120字，不要重复数据原文\n"
            "4. 如被问建议：说调哪架机去拦哪个目标，一句话\n"
            "优先级常识：巡飞弹>高速>机动>常规>诱饵(别管)"
        )

        self.analysis_queue = []
        self.chat_queue = []
        self.analysis_results = []
        self.chat_results = []
        self.status_events = ["Sentinel战术副官通道已接入"]
        self.channel_state = "ATTACHED"
        self.last_error = ""

        self.running = True
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.last_analysis_time = -999.0
        self.analysis_interval = 40.0


    def update_situation(self, current_time, enemies, interceptors):
        if current_time - self.last_analysis_time < self.analysis_interval:
            return
        context = self._build_context(enemies, interceptors, simple=True)
        fallback = self._fallback_analysis(enemies, interceptors)
        self.analysis_queue.append((context, fallback))
        self.last_analysis_time = current_time

    def chat(self, user_text, enemies, interceptors):
        context = self._build_context(enemies, interceptors, simple=False)
        full_prompt = f"{context}\n指挥官问：{user_text}"
        fallback = self._fallback_chat(user_text, enemies, interceptors)
        self.chat_queue.append((full_prompt, fallback))

    def get_analysis_log(self):
        if self.analysis_results:
            return self.analysis_results.pop(0)
        return None

    def get_chat_reply(self):
        if self.chat_results:
            return self.chat_results.pop(0)
        return None

    def drain_status_events(self):
        events = self.status_events[:]
        self.status_events.clear()
        return events

    # =========================================================
    # 上下文构建 - 适配小模型：结构化、短、精确
    # =========================================================
    def _build_context(self, enemies, interceptors, simple=True):
        """
        设计原则 (针对 7B 级别本地模型):
        - 总文本控制在 200 字以内，避免模型注意力涣散
        - 只列最危险的前 3 个目标，每个一行
        - 我方只给汇总数字 + 未拦截警告
        - 用固定格式，模型容易模仿和引用
        """
        active = [e for e in enemies
                  if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]

        if not active:
            return "当前无活跃威胁。"

        # 按距防线距离排序 (最危险的在前)
        active.sort(key=lambda e: e['y'], reverse=True)

        # --- 威胁列表 (最多3个，对话模式最多5个) ---
        top_n = 3 if simple else min(5, len(active))
        threat_lines = []
        for e in active[:top_n]:
            eid = e['id'] + 1
            etype = _ET.get(e['type'], "?")
            dist = CFG.INTERCEPT_FAIL_LINE - e['y']
            sector = _sector(e['x'])

            # 找到拦截它的飞机
            chaser = None
            for it in interceptors:
                if (it['target_id'] == e['id'] and
                    it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)):
                    chaser = it
                    break

            if chaser:
                tag = f"I-{chaser['id']+1}拦截中"
            elif not e.get('detected', False):
                tag = "监视中(未过线)"
            else:
                tag = "⚠无人拦截"

            threat_lines.append(f"F-{eid}({etype}) {sector} 距防线{dist:.0f}m {tag}")

        remaining = len(active) - top_n
        if remaining > 0:
            threat_lines.append(f"另有{remaining}架略")

        # --- 我方汇总 ---
        n_standby = sum(1 for i in interceptors if i['state'] == IState.STANDBY)
        n_fighting = sum(1 for i in interceptors
                         if i['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING))
        n_returning = sum(1 for i in interceptors
                          if i['state'] in (IState.RETURNING, IState.LANDED))

        our_line = f"我方：交战{n_fighting} 待命{n_standby} 返航{n_returning}"

        # --- 未拦截目标警告 ---
        unguarded = []
        for e in active:
            has_chaser = any(
                it['target_id'] == e['id'] and
                it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
                for it in interceptors
            )
            if e.get('detected', False) and not has_chaser and e['type'] != EType.DECOY:
                unguarded.append(f"F-{e['id']+1}")

        # --- 对话模式额外信息：待命机编号 ---
        extra = ""
        if not simple:
            if unguarded:
                extra += f"\n⚠未拦截: {','.join(unguarded[:4])}"
            sb_ids = [f"I-{i['id']+1}" for i in interceptors if i['state'] == IState.STANDBY]
            if sb_ids:
                extra += f"\n可调配: {','.join(sb_ids[:6])}"

        # --- 组装 ---
        result = "威胁：\n" + "\n".join(threat_lines) + "\n" + our_line + extra
        return result

    # =========================================================
    # 后台工作线程
    # =========================================================
    def _worker_loop(self):
        while self.running:
            if self.chat_queue:
                prompt, fallback = self.chat_queue.pop(0)
                self.analysis_queue.clear()
                res = self._call_llm(prompt, is_chat=True, fallback_text=fallback)
                if res:
                    self.chat_results.append(res)
            elif self.analysis_queue:
                prompt, fallback = self.analysis_queue.pop(0)
                self.analysis_queue.clear()
                res = self._call_llm(prompt, is_chat=False, fallback_text=fallback)
                if res:
                    self.analysis_results.append(res)
            time.sleep(0.1)

    def _set_channel_state(self, new_state, message):
        if self.channel_state != new_state:
            self.channel_state = new_state
            self.status_events.append(message)

    def _fallback_mode(self, enemy):
        if enemy.get('lost') or enemy.get('stale'):
            return "列阵扯网"
        if enemy.get('classification_confidence', 1.0) < CFG.MISCLASSIFY_CONFIDENCE:
            return "列阵扯网"
        if enemy.get('type') in (EType.SNAKE, EType.JINK, EType.LOITER, EType.DASH):
            return "列阵扯网"
        return "撞击"

    def _fallback_analysis(self, enemies, interceptors):
        active = [e for e in enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        if not active:
            return "Sentinel在线：当前空域无活跃威胁，建议保持待命。"

        active.sort(key=lambda e: (e.get('type') == EType.DECOY, -e['y']))
        target = active[0]
        mode = self._fallback_mode(target)
        chasers = [
            it for it in interceptors
            if it['target_id'] == target['id']
            and it['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING)
        ]
        target_name = f"F-{target['id']+1}"
        dist = max(0.0, CFG.INTERCEPT_FAIL_LINE - target['y'])
        if not target.get('detected', False):
            return f"Sentinel监视：{target_name}未过警戒线，建议持续跟踪。"
        if chasers:
            return f"Sentinel告警：{target_name}距防线{dist:.0f}m，{mode}编队已接敌。"
        standby = next((it for it in interceptors if it['state'] == IState.STANDBY), None)
        if standby:
            return f"Sentinel告警：{target_name}距防线{dist:.0f}m，建议I-{standby['id']+1}立即{mode}。"
        return f"Sentinel告警：{target_name}持续逼近，建议保持当前编队闭环压制。"

    def _fallback_chat(self, user_text, enemies, interceptors):
        user_text = user_text.strip()
        lower = user_text.lower()
        if "你是谁" in user_text or "谁制作" in user_text or "who are you" in lower:
            return "我是INET实验室研发的战术副官Sentinel。"

        active = [e for e in enemies if e['state'] in (EState.APPROACHING, EState.MANEUVERING)]
        n_standby = sum(1 for i in interceptors if i['state'] == IState.STANDBY)
        n_fighting = sum(1 for i in interceptors if i['state'] in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING))

        if "状态" in user_text or "status" in lower:
            return f"当前活跃目标{len(active)}个，交战{n_fighting}架，待命{n_standby}架。"

        if "建议" in user_text or "怎么办" in user_text or "拦截" in user_text:
            if not active:
                return "当前无活跃威胁，建议保持单机巢待命。"
            active.sort(key=lambda e: (e.get('type') == EType.DECOY, -e['y']))
            target = active[0]
            if not target.get('detected', False):
                return f"F-{target['id']+1} 尚未过警戒线，建议持续监视，暂不出动。"
            mode = self._fallback_mode(target)
            standby = next((it for it in interceptors if it['state'] == IState.STANDBY), None)
            if standby:
                return f"建议优先由I-{standby['id']+1}对F-{target['id']+1}执行{mode}。"
            return f"建议保持当前编队对F-{target['id']+1}持续{mode}。"

        if not active:
            return "Sentinel在线，当前空域平静。"
        target = max(active, key=lambda e: e['y'])
        return f"当前最危险目标是F-{target['id']+1}，建议优先处置。"

    def _call_llm(self, prompt, is_chat, fallback_text):
        if "sk-xxx" in self.api_key:
            self._set_channel_state("FALLBACK", "Sentinel主通道关闭，已切换规则副官")
            return fallback_text

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        sys_prompt = self.system_prompt_chat if is_chat else self.system_prompt_analysis

        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": sys_prompt},
                {"role": "user", "content": prompt},
            ],
            "temperature": 0.3,   # 低温度 = 少废话少编造
            "max_tokens": 120 if is_chat else 60,  # 硬性截断兜底
        }

        try:
            resp = requests.post(
                f"{self.base_url}/chat/completions",
                headers=headers, json=payload, timeout=20,
            )
            if resp.status_code == 200:
                text = resp.json()['choices'][0]['message']['content'].strip()
                self._set_channel_state("ONLINE", "Sentinel大模型通道在线")

                if is_chat and len(text) > 150:
                    text = text[:147] + "..."
                elif not is_chat and len(text) > 80:
                    text = text[:77] + "..."
                return text
            self.last_error = f"HTTP {resp.status_code}"
            self._set_channel_state("FALLBACK", "Sentinel主通道异常，已切换规则副官")
            return fallback_text
        except Exception as exc:
            self.last_error = str(exc)
            self._set_channel_state("FALLBACK", "Sentinel主通道异常，已切换规则副官")
            return fallback_text

    def get_status(self):
        return f"LLM:{self.channel_state} 分析队列:{len(self.analysis_queue)} 对话队列:{len(self.chat_queue)}"
