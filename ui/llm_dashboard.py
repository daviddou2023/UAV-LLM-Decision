import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from core.common import CFG


HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>LLM 思维链 / 推理过程</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f3f5f8;
      --panel: #ffffff;
      --ink: #1b2430;
      --muted: #687587;
      --line: #d8e0eb;
      --teal: #007f85;
      --amber: #b76b00;
      --green: #17844a;
      --violet: #7b56c6;
      --blue: #2a69b8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--ink);
      font-family: "Microsoft YaHei", "PingFang SC", "Noto Sans CJK SC", Arial, sans-serif;
      letter-spacing: 0;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 18px;
      padding: 16px 24px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1, h2, h3, p { margin: 0; }
    h1 { font-size: 24px; font-weight: 780; }
    .sub { margin-top: 4px; color: var(--muted); font-size: 13px; }
    .status { display: flex; gap: 10px; flex-wrap: wrap; justify-content: flex-end; }
    .pill {
      padding: 7px 11px;
      border-radius: 999px;
      border: 1px solid var(--line);
      background: #fbfcfe;
      font-size: 13px;
      color: var(--muted);
      white-space: nowrap;
    }
    main {
      padding: 18px 24px 24px;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 340px;
      gap: 16px;
    }
    section, .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    section { padding: 18px; }
    .head {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 14px;
    }
    .head h2 { font-size: 16px; font-weight: 760; }
    .time { color: var(--muted); font-size: 12px; }
    .metrics {
      display: grid;
      grid-template-columns: repeat(5, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 16px;
    }
    .compare-strip {
      display: grid;
      grid-template-columns: minmax(0, 1.1fr) minmax(0, 1fr) minmax(0, 1.1fr);
      gap: 12px;
      margin-bottom: 16px;
    }
    .compare-card {
      min-height: 118px;
      padding: 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
    }
    .compare-card .k { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
    .compare-card .v {
      font-size: 15px;
      font-weight: 690;
      line-height: 1.55;
      color: #233142;
      word-break: break-word;
    }
    .metric {
      padding: 13px 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
    }
    .metric .label { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
    .metric .value { font-size: 24px; font-weight: 800; line-height: 1; }
    .reason-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 16px;
    }
    .reason-card {
      min-height: 150px;
      padding: 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #ffffff;
    }
    .reason-card .k { color: var(--muted); font-size: 12px; margin-bottom: 9px; }
    .reason-card .v { font-size: 15px; font-weight: 660; line-height: 1.5; word-break: break-word; }
    .teal { border-top: 3px solid var(--teal); }
    .amber { border-top: 3px solid var(--amber); }
    .violet { border-top: 3px solid var(--violet); }
    .green { border-top: 3px solid var(--green); }
    .dual {
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      gap: 14px;
      margin-bottom: 16px;
    }
    .box {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fbfcfe;
      min-height: 320px;
      overflow: auto;
    }
    .trace-item, .event {
      display: grid;
      gap: 8px;
      padding: 12px;
      border-bottom: 1px solid #e5ebf2;
    }
    .trace-item:last-child, .event:last-child { border-bottom: 0; }
    .trace-item {
      grid-template-columns: 42px minmax(0, 1fr);
      align-items: start;
    }
    .trace-num {
      width: 30px;
      height: 30px;
      border-radius: 999px;
      background: #e8f0fb;
      color: var(--blue);
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 12px;
      font-weight: 780;
    }
    .trace-title {
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 4px;
    }
    .trace-detail {
      font-size: 14px;
      color: #253240;
      line-height: 1.5;
      word-break: break-word;
    }
    .event {
      grid-template-columns: 82px minmax(0, 1fr);
      font-size: 13px;
      line-height: 1.45;
    }
    .tag { color: var(--blue); font-weight: 760; white-space: nowrap; }
    .msg { color: #253240; word-break: break-word; }
    pre {
      margin: 0;
      padding: 14px;
      white-space: pre-wrap;
      word-break: break-word;
      font-size: 12px;
      line-height: 1.55;
      color: #1f2937;
      min-height: 320px;
    }
    aside {
      display: flex;
      flex-direction: column;
      gap: 14px;
    }
    .card { padding: 16px; }
    .kv {
      display: grid;
      grid-template-columns: 124px minmax(0, 1fr);
      gap: 8px;
      padding: 8px 0;
      border-bottom: 1px solid #e5ebf2;
      font-size: 13px;
    }
    .kv:last-child { border-bottom: 0; }
    .name { color: var(--muted); }
    .data { font-weight: 700; word-break: break-word; }
    .hint {
      margin-top: 10px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.55;
    }
    @media (max-width: 1180px) {
      main { grid-template-columns: 1fr; }
      .metrics { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .compare-strip { grid-template-columns: 1fr; }
      .reason-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 760px) {
      header { flex-direction: column; align-items: flex-start; }
      main { padding: 12px; }
      .metrics, .compare-strip, .reason-grid, .dual { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>LLM 思维链 / 推理过程</h1>
      <div class="sub">输入理解 -> 信息抽取 -> 态势判断 -> 风险约束 -> 方案生成 -> 执行下发</div>
    </div>
    <div class="status">
      <span class="pill" id="conn">连接中</span>
      <span class="pill" id="scene">Scene --</span>
      <span class="pill" id="scheme">方案 --</span>
      <span class="pill" id="clock">t=0.0s</span>
    </div>
  </header>
  <main>
    <section>
      <div class="head">
        <h2>外显推理过程</h2>
        <span class="time" id="updated">--</span>
      </div>
      <div class="metrics">
        <div class="metric"><div class="label">敌方总数</div><div class="value" id="m_enemy">0</div></div>
        <div class="metric"><div class="label">活动目标</div><div class="value" id="m_active_enemy">0</div></div>
        <div class="metric"><div class="label">已拦截</div><div class="value" id="m_kill">0</div></div>
        <div class="metric"><div class="label">失联/受扰</div><div class="value" id="m_lost">0</div></div>
        <div class="metric"><div class="label">LLM保留</div><div class="value" id="m_reserve">0</div></div>
      </div>
      <div class="compare-strip">
        <div class="compare-card teal">
          <div class="k">原始指令</div>
          <div class="v" id="cmp_input">等待语音/文本指令</div>
        </div>
        <div class="compare-card amber">
          <div class="k">LLM提取约束</div>
          <div class="v" id="cmp_constraints">target_priority=default, preferred_sector=all, reserve=0, max_active=all, avoid_jam=false</div>
        </div>
        <div class="compare-card green">
          <div class="k">下层执行结果</div>
          <div class="v" id="cmp_execution">下层分配器待命</div>
        </div>
      </div>
      <div class="reason-grid">
        <div class="reason-card teal"><div class="k">1. 输入指令</div><div class="v" id="r_input">等待语音/文本指令</div></div>
        <div class="reason-card amber"><div class="k">2. 关键信息抽取</div><div class="v" id="r_facts">暂无</div></div>
        <div class="reason-card violet"><div class="k">3. 战场态势判断</div><div class="v" id="r_situation">暂无</div></div>
        <div class="reason-card amber"><div class="k">4. 风险约束评估</div><div class="v" id="r_risk">暂无</div></div>
        <div class="reason-card teal"><div class="k">5. 任务方案生成</div><div class="v" id="r_plan">暂无</div></div>
        <div class="reason-card green"><div class="k">6. 执行下发</div><div class="v" id="r_exec">下层分配器待命</div></div>
      </div>
      <div class="dual">
        <div>
          <div class="head"><h2>推理时间线</h2></div>
          <div class="box" id="trace_list"></div>
        </div>
        <div>
          <div class="head"><h2>结构化决策对象</h2></div>
          <div class="box"><pre id="decision_json">{}</pre></div>
        </div>
      </div>
      <div class="dual">
        <div>
          <div class="head"><h2>LLM / 指挥链日志</h2></div>
          <div class="box" id="llm_events"></div>
        </div>
        <div>
          <div class="head"><h2>下层执行日志</h2></div>
          <div class="box" id="action_events"></div>
        </div>
      </div>
    </section>
    <aside>
      <div class="card">
        <div class="head"><h2>当前约束</h2></div>
        <div class="kv"><span class="name">目标优先级</span><span class="data" id="c_priority">default</span></div>
        <div class="kv"><span class="name">区域优先</span><span class="data" id="c_sector">all</span></div>
        <div class="kv"><span class="name">保留无人机</span><span class="data" id="c_reserve">0</span></div>
        <div class="kv"><span class="name">最多出动</span><span class="data" id="c_max_active">all</span></div>
        <div class="kv"><span class="name">干扰区绕避</span><span class="data" id="c_jam">false</span></div>
        <div class="kv"><span class="name">任务姿态</span><span class="data" id="c_posture">normal</span></div>
        <div class="hint">这里展示的是可对外讲清楚、能映射到底层动作的外显推理链，不是无法验证的黑箱自言自语。</div>
      </div>
      <div class="card">
        <div class="head"><h2>战场状态</h2></div>
        <div class="kv"><span class="name">己方待命</span><span class="data" id="s_standby">0</span></div>
        <div class="kv"><span class="name">己方执行中</span><span class="data" id="s_active">0</span></div>
        <div class="kv"><span class="name">突防目标</span><span class="data" id="s_pen">0</span></div>
        <div class="kv"><span class="name">干扰区域</span><span class="data" id="s_zones">--</span></div>
        <div class="kv"><span class="name">当前模式</span><span class="data" id="s_mode">--</span></div>
      </div>
    </aside>
  </main>
  <script>
    const $ = (id) => document.getElementById(id);
    function esc(s) {
      return String(s ?? "").replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
    }
    function eventsHtml(events) {
      if (!events || !events.length) return '<div class="event"><span class="tag">--</span><span class="msg">暂无输出</span></div>';
      return events.map(e => `<div class="event"><span class="tag">${esc(e.tag)}</span><span class="msg">${esc(e.message)}</span></div>`).join('');
    }
    function traceHtml(trace) {
      if (!trace || !trace.length) return '<div class="trace-item"><span class="trace-num">--</span><div><div class="trace-title">等待输入</div><div class="trace-detail">暂无推理过程</div></div></div>';
      return trace.map(step => `
        <div class="trace-item">
          <span class="trace-num">${esc(step.index || '--')}</span>
          <div>
            <div class="trace-title">${esc(step.title || '步骤')}</div>
            <div class="trace-detail">${esc(step.detail || '')}</div>
          </div>
        </div>`).join('');
    }
    async function refresh() {
      try {
        const res = await fetch('/state?t=' + Date.now());
        const data = await res.json();
        const reasoning = data.reasoning || {};
        const compare = data.compare || {};
        $('conn').textContent = '已连接';
        $('scene').textContent = `Scene ${data.scene_km}km`;
        $('scheme').textContent = data.scheme || '手动配置';
        $('clock').textContent = `t=${Number(data.sim_time || 0).toFixed(1)}s`;
        $('updated').textContent = new Date().toLocaleTimeString();
        $('m_enemy').textContent = data.metrics.total_enemies;
        $('m_active_enemy').textContent = data.metrics.active_enemies;
        $('m_kill').textContent = data.metrics.kills;
        $('m_lost').textContent = data.metrics.lost_or_jammed;
        $('m_reserve').textContent = data.metrics.reserved;
        $('cmp_input').textContent = compare.input_text || reasoning.input_text || '等待语音/文本指令';
        $('cmp_constraints').textContent = compare.constraints_text || reasoning.constraints_text || 'target_priority=default, preferred_sector=all, reserve=0, max_active=all, avoid_jam=false';
        $('cmp_execution').textContent = compare.execution || reasoning.execution || '下层分配器待命';
        $('r_input').textContent = reasoning.input_text || '等待语音/文本指令';
        $('r_facts').textContent = reasoning.facts || '暂无';
        $('r_situation').textContent = reasoning.situation || '暂无';
        $('r_risk').textContent = reasoning.risk || '暂无';
        $('r_plan').textContent = reasoning.plan || '暂无';
        $('r_exec').textContent = reasoning.execution || '下层分配器待命';
        $('trace_list').innerHTML = traceHtml(reasoning.trace_steps);
        $('decision_json').textContent = JSON.stringify(reasoning.decision_object || {}, null, 2);
        $('c_priority').textContent = data.constraints.target_priority || 'default';
        $('c_sector').textContent = data.constraints.preferred_sector || 'all';
        $('c_reserve').textContent = `${data.constraints.reserve_count || 0} / locked ${data.constraints.reserve_locked || 0}`;
        $('c_max_active').textContent = data.constraints.max_active_count > 0 ? String(data.constraints.max_active_count) : 'all';
        $('c_jam').textContent = String(Boolean(data.constraints.avoid_jam));
        $('c_posture').textContent = data.command_posture || 'normal';
        $('s_standby').textContent = data.metrics.standby;
        $('s_active').textContent = data.metrics.active_friendlies;
        $('s_pen').textContent = data.metrics.penetrations;
        $('s_zones').textContent = data.jam_zones || '--';
        $('s_mode').textContent = reasoning.mode || '--';
        $('llm_events').innerHTML = eventsHtml(data.llm_events);
        $('action_events').innerHTML = eventsHtml(data.action_events);
      } catch (err) {
        $('conn').textContent = '连接中断';
      }
    }
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
"""


def _enum_name(value):
    return getattr(value, "name", str(value))


def _log_parts(entry):
    if isinstance(entry, tuple):
        tag = str(entry[0]) if len(entry) > 0 else "[LOG]"
        message = str(entry[1]) if len(entry) > 1 else ""
        color = str(entry[2]) if len(entry) > 2 else ""
        return tag, message, color
    text = str(entry)
    if text.startswith("[") and "]" in text:
        idx = text.find("]")
        return text[:idx + 1], text[idx + 1:].strip(), ""
    return "[ENV]", text, ""


def _last_matching(events, keywords, fallback):
    for entry in reversed(events):
        tag, message, _ = _log_parts(entry)
        text = f"{tag} {message}"
        if any(key in text for key in keywords):
            return message
    return fallback


def _event_items(events, tags, limit=18):
    items = []
    for entry in reversed(events):
        tag, message, color = _log_parts(entry)
        if tag in tags or any(tag.startswith(prefix) for prefix in tags):
            items.append({"tag": tag, "message": message, "color": color})
        if len(items) >= limit:
            break
    return list(reversed(items))


def _constraint_summary(constraints):
    constraints = dict(constraints or {})
    priority = constraints.get("target_priority") or "default"
    preferred_sector = constraints.get("preferred_sector") or "all"
    reserve_count = int(constraints.get("reserve_count") or 0)
    max_active_count = int(constraints.get("max_active_count") or 0)
    max_active_text = str(max_active_count) if max_active_count > 0 else "all"
    avoid_jam = str(bool(constraints.get("avoid_jam"))).lower()
    return (
        f"target_priority={priority}, preferred_sector={preferred_sector}, "
        f"reserve={reserve_count}, max_active={max_active_text}, avoid_jam={avoid_jam}"
    )


def build_dashboard_snapshot(env):
    logs = list(getattr(env, "logs", []) or [])
    constraints = dict(getattr(env, "llm_task_constraints", {}) or {})
    reasoning_state = dict(getattr(env, "llm_reasoning_state", {}) or {})
    active_friendly_states = {"LAUNCHING", "INTERCEPTING", "FOLLOWING", "RETURNING"}
    active_enemy_states = {"APPROACHING", "MANEUVERING"}
    interceptors = list(getattr(env, "interceptors", []) or [])
    enemies = list(getattr(env, "enemies", []) or [])

    reserved_count = 0
    get_reserved = getattr(env, "get_reserved_interceptor_count", None)
    if get_reserved:
        try:
            reserved_count = int(get_reserved())
        except Exception:
            reserved_count = sum(1 for item in interceptors if item.get("task_reserved"))
    else:
        reserved_count = sum(1 for item in interceptors if item.get("task_reserved"))

    lost_or_jammed = sum(1 for item in interceptors if item.get("jammed_by_interference") or item.get("lost"))
    active_friendlies = sum(1 for item in interceptors if _enum_name(item.get("state")) in active_friendly_states)
    standby = sum(1 for item in interceptors if _enum_name(item.get("state")) == "STANDBY")
    active_enemies = sum(1 for item in enemies if _enum_name(item.get("state")) in active_enemy_states)

    stats = getattr(env, "stats", {}) or {}
    zones = []
    for zone in getattr(env, "demo_interference_zones", []) or []:
        label = zone.get("label") or zone.get("name") or "干扰"
        suffix = "禁入" if zone.get("llm_no_fly") else "可视"
        zones.append(f"{label}({suffix})")

    priority = constraints.get("target_priority") or "default"
    reserve_count = int(constraints.get("reserve_count") or 0)
    avoid_jam = bool(constraints.get("avoid_jam"))
    chain = {
        "intent": _last_matching(logs, ("意图解析", "[USR]", "[VOICE]"), "等待语音/文本指令"),
        "assessment": _last_matching(logs, ("态势研判", "研判:", "[AI情报]"), getattr(env, "llm_decision_title", "等待态势更新")),
        "constraints": _constraint_summary(constraints),
        "execution": _last_matching(logs, ("[CMD]", "下层分配器", "[ASGN]", "[BASE]"), "下层分配器待命"),
    }
    reasoning = {
        "mode": reasoning_state.get("mode", "derived"),
        "input_text": reasoning_state.get("input_text") or _last_matching(logs, ("[USR]", "[VOICE]", "意图解析"), "等待语音/文本指令"),
        "intent": reasoning_state.get("intent") or chain["intent"],
        "facts": reasoning_state.get("facts") or chain["constraints"],
        "situation": reasoning_state.get("situation") or chain["assessment"],
        "risk": reasoning_state.get("risk") or ("干扰区存在链路中断风险" if avoid_jam else "当前无额外风险约束"),
        "plan": reasoning_state.get("plan") or chain["constraints"],
        "constraints_text": reasoning_state.get("constraints_text") or chain["constraints"],
        "execution": reasoning_state.get("execution") or chain["execution"],
        "decision_object": reasoning_state.get("decision_object") or {
            "target_priority": priority,
            "preferred_sector": constraints.get("preferred_sector") or "all",
            "reserve_count": reserve_count,
            "max_active_count": int(constraints.get("max_active_count") or 0),
            "avoid_jam": avoid_jam,
        },
        "trace_steps": reasoning_state.get("trace_steps") or [
            {"index": "01", "title": "输入指令", "detail": _last_matching(logs, ("[USR]", "[VOICE]"), "等待语音/文本指令")},
            {"index": "02", "title": "关键信息抽取", "detail": chain["intent"]},
            {"index": "03", "title": "战场态势判断", "detail": chain["assessment"]},
            {"index": "04", "title": "风险约束评估", "detail": "根据当前态势生成任务约束"},
            {"index": "05", "title": "任务方案生成", "detail": chain["constraints"]},
            {"index": "06", "title": "执行下发", "detail": chain["execution"]},
        ],
        "updated_at": float(reasoning_state.get("updated_at", 0.0) or 0.0),
    }

    scheme_id = int(getattr(env, "demo_scheme", 0) or 0)
    scheme_name = getattr(env, "demo_scheme_name", "") or "手动配置"
    return {
        "timestamp": time.time(),
        "sim_time": float(getattr(env, "time", 0.0) or 0.0),
        "scene_km": f"{float(CFG.SCENE_KM):.0f}",
        "scheme": f"S{scheme_id} {scheme_name}" if scheme_id else scheme_name,
        "command_posture": getattr(env, "command_posture", "normal"),
        "constraints": constraints,
        "chain": chain,
        "reasoning": reasoning,
        "compare": {
            "input_text": reasoning.get("input_text") or "等待语音/文本指令",
            "constraints_text": reasoning.get("constraints_text") or chain["constraints"],
            "execution": reasoning.get("execution") or chain["execution"],
        },
        "metrics": {
            "total_enemies": int(stats.get("total_enemies", len(enemies)) or len(enemies)),
            "active_enemies": active_enemies,
            "kills": int(stats.get("kills", 0) or 0),
            "penetrations": int(stats.get("penetrations", 0) or 0),
            "lost_or_jammed": lost_or_jammed,
            "reserved": reserved_count,
            "standby": standby,
            "active_friendlies": active_friendlies,
        },
        "jam_zones": " / ".join(zones),
        "llm_events": _event_items(logs, ("[LLM]", "[CMD]", "[AI情报]", "[副官]", "[VOICE]", "[USR]"), limit=22),
        "action_events": _event_items(logs, ("[ASGN]", "[BASE]", "[JAM]", "[PATH]", "[BARRIER]", "[NET]", "[RTB]", "[SCHEME]"), limit=22),
    }


class LLMDashboardServer:
    def __init__(self, env, host="127.0.0.1", port=8765, open_browser=False):
        self.env = env
        self.host = str(host or "127.0.0.1")
        self.port = int(port or 8765)
        self.open_browser = bool(open_browser)
        self.httpd = None
        self.thread = None
        self.url = None

    def start(self):
        env = self.env

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def _send(self, status, content_type, body):
                if isinstance(body, str):
                    body = body.encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", HTML_PAGE)
                    return
                if path == "/state":
                    try:
                        payload = build_dashboard_snapshot(env)
                        self._send(200, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False))
                    except Exception as exc:
                        self._send(500, "application/json; charset=utf-8", json.dumps({"error": str(exc)}, ensure_ascii=False))
                    return
                self._send(404, "text/plain; charset=utf-8", "not found")

        last_error = None
        for offset in range(20):
            try_port = self.port + offset
            try:
                self.httpd = ThreadingHTTPServer((self.host, try_port), Handler)
                self.port = try_port
                break
            except OSError as exc:
                last_error = exc
                self.httpd = None
        if self.httpd is None:
            raise RuntimeError(f"LLM dashboard port unavailable: {last_error}")

        self.url = f"http://{self.host}:{self.port}/"
        self.thread = threading.Thread(target=self.httpd.serve_forever, name="llm-dashboard", daemon=True)
        self.thread.start()
        if self.open_browser:
            threading.Timer(0.35, lambda: webbrowser.open(self.url)).start()
        return self.url

    def stop(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.httpd = None
