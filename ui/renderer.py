"""
MARL UI模块 - 空域拦截雷达态势显示 v8.0
1. 保持 2D 界面不变，但支持 3D 规划结果投影显示
2. 场景尺寸可动态切换，比例尺与分界线自动刷新
3. 侧栏显示目标高度 Z 与数据源状态
"""
import math, time
import os
from typing import Tuple, List, Dict
from core.common import CFG, IState, EState, IRole, EType, dist2d, entity_is_destroyed, friendly_view

class LLMConfig:
    API_TYPE="openai"; OPENAI_API_KEY="EMPTY"
    OPENAI_BASE_URL="http://localhost:8000/v1"; OPENAI_MODEL="Qwen-Agent"
    TIMEOUT=30; MAX_TOKENS=500; TEMPERATURE=0.7

LLM_CFG=LLMConfig()

class DemoRenderer:
    def __init__(self, env, fullscreen=False, ui_style="arc"):
        os.environ.setdefault("SDL_RENDER_DRIVER", "software")
        os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        os.environ.setdefault("SDL_VIDEO_X11_FORCE_EGL", "0")
        import pygame; self.pg=pygame; pygame.init()
        self.env=env
        self._scene_revision = -1
        self.fullscreen = fullscreen
        self.ui_style = ui_style if ui_style in ("arc", "rect", "omni") else "arc"
        self.windowed_size = (CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT)
        self._bg_cache = None
        self._bg_cache_key = None
        self._radar_cache = None
        self._radar_cache_key = None
        self._log_cache = []
        self._log_cache_key = None
        self._log_revision = 0
        self.cmd_y = 0
        self.cmd_h = 90
        self.enemy_panel_y = 0
        self.enemy_panel_h = 160
        self.add_enemy_button_rect = None
        self.add_friendly_button_rect = None
        self.toggle_interference_button_rect = None
        self.cancel_reserve_button_rect = None
        self.scheme_button_rects = []
        self._set_display_mode(fullscreen=self.fullscreen)
        pygame.display.set_caption("空域拦截防御系统 v8.0 | AIR DEFENSE COMMAND")

        # === 字体初始化 ===
        self._init_fonts()

        self.C = {
            'bg': (8, 12, 18), 'bg2': (15, 22, 35), 'pnl': (12, 18, 28),
            'gd': (20, 35, 50), 'gdd': (15, 25, 38),
            'cyan': (0, 255, 255), 'cyd': (0, 130, 160),
            'blue': (30, 144, 255), 'pink': (255, 20, 147),
            'amber': (255, 191, 0), 'orange': (255, 140, 0),
            'red': (255, 50, 50), 'rdd': (180, 30, 30),
            'green': (0, 255, 128), 'grd': (0, 160, 80),
            'txt': (220, 230, 240), 'txt2': (120, 140, 160),
            'txtd': (70, 85, 100), 'txt_h': (0, 255, 255),
            'ia': (0, 200, 255), 'if': (100, 180, 255),
            'is': (60, 80, 100), 'ir': (100, 160, 100), 'ib': (255, 215, 90),
            'ef': (255, 100, 100), 'em': (255, 50, 50), 'ec': (255, 0, 0), 'ed': (80, 40, 40),
            'eloiter': (255, 0, 255), 'edecoy': (180, 180, 200), 'edash': (255, 140, 0),
        }

        self.chat_input=""; self.chat_active=False
        self.cursor_timer=0; self.cursor_vis=True
        self.logs=[]; self._elo=0
        self.fc=0; self.pulse=0.0
        self.lsy=0; self.lth=0; self.lvh=500; self.fscr=True
        self.p2_scroll = 0
        self.p2_total_h = 0
        self.p2_view_h = 0
        self.enemy_scroll = 0
        self.enemy_total_h = 0
        self.enemy_view_h = 0
        self.show_poi = False

        self.enemy_trails: Dict[int, List[Tuple[float, float]]] = {}
        self.trail_length = 24

        self._fps_t=time.time(); self._fps_c=0; self._fps=60

        # 语音状态 ===
        self.voice_state = 0   # 0:待命  1:录音中(按住V)  2:识别中(松开V后)
        self.voice_text = ""   # 临时展示识别文本 (可选)

        self._init_logs()

    def _set_display_mode(self, fullscreen=None, size=None):
        if fullscreen is not None:
            self.fullscreen = fullscreen
        if size is not None:
            self.windowed_size = size
        pg = self.pg
        if self.fullscreen:
            display_size = pg.display.get_desktop_sizes()[0]
            flags = pg.FULLSCREEN
        else:
            display_size = size or self.windowed_size
            flags = pg.RESIZABLE
        self.screen = pg.display.set_mode(display_size, flags)
        CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT = self.screen.get_size()
        self._apply_layout()

    def _apply_layout(self):
        self.ox, self.oy = 20, max(76, int(CFG.SCREEN_HEIGHT * 0.09))
        usable_w = CFG.SCREEN_WIDTH - 40
        self.p1_width = int(usable_w * 0.44)
        self.p2_width = int(usable_w * 0.20)
        self.p3_width = usable_w - self.p1_width - self.p2_width - 35
        self.p1x = 20
        self.p2x = self.p1x + self.p1_width + 20
        self.p3x = self.p2x + self.p2_width + 15
        right_h = CFG.SCREEN_HEIGHT - self.oy - 20
        self.cmd_h = max(110, min(136, int(right_h * 0.14)))
        reserve_h = max(150, min(240, int(right_h * 0.23)))
        self.lvh = max(340, right_h - self.cmd_h - reserve_h - 24)
        self.cmd_y = self.oy + self.lvh + 12
        self.enemy_panel_y = self.cmd_y + self.cmd_h + 12
        self.enemy_panel_h = max(120, CFG.SCREEN_HEIGHT - self.enemy_panel_y - 20)
        if self.enemy_panel_h < 120:
            shrink = 120 - self.enemy_panel_h
            self.lvh = max(300, self.lvh - shrink)
            self.cmd_y = self.oy + self.lvh + 12
            self.enemy_panel_y = self.cmd_y + self.cmd_h + 12
            self.enemy_panel_h = max(120, CFG.SCREEN_HEIGHT - self.enemy_panel_y - 20)
        btn_w, btn_h, gap = 96, 32, 8
        right = CFG.SCREEN_WIDTH - 20
        self.add_friendly_button_rect = self.pg.Rect(right - btn_w, 18, btn_w, btn_h)
        self.add_enemy_button_rect = self.pg.Rect(right - btn_w * 2 - gap, 18, btn_w, btn_h)
        self.cancel_reserve_button_rect = self.pg.Rect(right - btn_w * 3 - gap * 2, 18, btn_w, btn_h)
        self.toggle_interference_button_rect = self.pg.Rect(right - btn_w * 4 - gap * 3, 18, btn_w, btn_h)
        scheme_w, scheme_h, scheme_gap = 66, 24, 6
        scheme_labels = (1, 2, 3)
        scheme_total_w = scheme_w * len(scheme_labels) + scheme_gap * (len(scheme_labels) - 1)
        scheme_x = right - scheme_total_w
        self.scheme_button_rects = [
            (sid, self.pg.Rect(scheme_x + idx * (scheme_w + scheme_gap), 54, scheme_w, scheme_h))
            for idx, sid in enumerate(scheme_labels)
        ]
        self._invalidate_caches()
        self._refresh_scene_geometry()

    def _init_fonts(self):
        scale = max(1.0, min(CFG.SCREEN_WIDTH / 1400.0, CFG.SCREEN_HEIGHT / 850.0))
        self.fcn = self._lf(int(21 * scale))
        self.fcs = self._lf(int(19 * scale))
        self.fcm = self._lf(int(25 * scale))
        self.fcl = self._lf(int(30 * scale))
        self.fm  = self._mf(int(16 * scale))
        self.fms = self._mf(int(16 * scale))
        self.f_status = self._lf(int(23 * scale))
        self._invalidate_caches()

    def _lf(self, sz):
        pg = self.pg
        font_paths = [
            "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf", "C:/Windows/Fonts/simsun.ttc",
            "/System/Library/Fonts/PingFang.ttc", "/System/Library/Fonts/STHeiti Light.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"
        ]
        for p in font_paths:
            try:
                f = pg.font.Font(p, sz)
                if f.render("测", True, (0,0,0)).get_width() > 0: return f
            except: pass
        return pg.font.Font(None, sz)

    def _mf(self, sz):
        return self._lf(sz)

    def _init_logs(self):
        self.add_log("[SYS]","空域拦截终端 v8.0 上线","cyan")
        self.add_log("[SYS]",f"拦截机编队 {CFG.NUM_INTERCEPTORS}架待命","blue")

    def _visible_friendlies(self):
        if hasattr(self.env, "visible_interceptors"):
            return self.env.visible_interceptors()
        return self.env.interceptors

    def _invalidate_caches(self):
        self._bg_cache = None
        self._bg_cache_key = None
        self._radar_cache = None
        self._radar_cache_key = None
        self._log_cache = []
        self._log_cache_key = None

    def _refresh_scene_geometry(self):
        avail_h = CFG.SCREEN_HEIGHT - self.oy - 20
        avail_w = self.p1_width - 20
        self.radar_cx = self.p1x + self.p1_width // 2
        self.radar_cy = self.oy + 8
        if self.ui_style == "arc":
            self.radar_half_span_deg = 44.0
            self.radar_half_span = math.radians(self.radar_half_span_deg)
            max_r_by_w = avail_w / max(2.0 * math.sin(self.radar_half_span), 0.1)
            max_r_by_h = avail_h - 8
            self.radar_radius = int(min(max_r_by_w, max_r_by_h))
            self.scale = self.radar_radius / max(CFG.OUR_BASE_LINE, 1.0)
            self.sx = self.scale
            self.sy = self.scale
            self.mw = int(2 * self.radar_radius * math.sin(self.radar_half_span))
            self.mh = self.radar_radius
            self.map_offset_x = self.radar_cx - self.mw // 2 - self.ox
            self.map_offset_y = 0
        elif self.ui_style == "rect":
            self.radar_half_span_deg = 90.0
            self.radar_half_span = math.radians(self.radar_half_span_deg)
            self.radar_radius = 0
            self.map_rect = self.pg.Rect(self.p1x + 10, self.oy + 8, self.p1_width - 20, avail_h - 8)
            self.scale_x = self.map_rect.width / max(CFG.AREA_WIDTH, 1.0)
            self.scale_y = self.map_rect.height / max(CFG.OUR_BASE_LINE, 1.0)
            self.scale = min(self.scale_x, self.scale_y)
            self.sx = self.scale_x
            self.sy = self.scale_y
            self.mw = self.map_rect.width
            self.mh = self.map_rect.height
            self.map_offset_x = self.map_rect.x - self.ox
            self.map_offset_y = self.map_rect.y - self.oy
        else:
            self.radar_half_span_deg = 180.0
            self.radar_half_span = math.pi
            self.radar_radius = int(max(120, min(avail_w, avail_h - 8) * 0.47))
            self.radar_cx = self.p1x + self.p1_width // 2
            self.radar_cy = self.oy + max(12, avail_h // 2)
            self.radar_range_m = max(CFG.OUR_BASE_LINE, CFG.AREA_WIDTH, 1.0)
            self.scale = self.radar_radius / self.radar_range_m
            self.sx = self.scale
            self.sy = self.scale
            self.mw = self.radar_radius * 2
            self.mh = self.radar_radius * 2
            self.map_offset_x = self.radar_cx - self.radar_radius - self.ox
            self.map_offset_y = self.radar_cy - self.radar_radius - self.oy
        self._scene_revision = getattr(self.env, "scene_revision", self._scene_revision)
        self._invalidate_caches()

    def add_log(self,pfx,msg,ck="txt2"):
        quiet_tokens = ("追踪当前雷达点", "随动保持备份拦截位置")
        if isinstance(msg, str) and any(token in msg for token in quiet_tokens):
            return
        if pfx == "[DECONF]" or (isinstance(msg, str) and "[DECONF]" in msg):
            return
        self.logs.append((pfx,msg,ck))
        if len(self.logs)>80: self.logs.pop(0)
        self.fscr=True
        self._log_revision += 1
        self._log_cache_key = None

    def _ownship_world_center(self):
        friendlies = [
            friendly_view(it)
            for it in self._visible_friendlies()
            if self._friendly_render_state(it) not in (IState.DESTROYED,)
        ]
        airborne = [
            it for it in friendlies
            if it.get('state') not in (IState.STANDBY, IState.LANDED)
        ]
        basis = airborne or friendlies
        if basis:
            return (
                sum(float(it.get('x', 0.0)) for it in basis) / len(basis),
                sum(float(it.get('y', 0.0)) for it in basis) / len(basis),
            )
        return (CFG.AREA_WIDTH * 0.5, CFG.INTERCEPT_FAIL_LINE)

    def _w2s(self,x,y):
        if self.ui_style == "omni":
            ox, oy = self._ownship_world_center()
            dx = float(x) - ox
            dy = float(y) - oy
            rng = math.hypot(dx, dy)
            if rng > self.radar_range_m and rng > 1e-6:
                scale = self.radar_range_m / rng
                dx *= scale
                dy *= scale
            screen_x = int(self.radar_cx + dx * self.scale)
            screen_y = int(self.radar_cy - dy * self.scale)
            return (screen_x, screen_y)
        if self.ui_style == "rect":
            clamped_x = max(0.0, min(CFG.AREA_WIDTH, x))
            clamped_y = max(0.0, min(CFG.OUR_BASE_LINE, y))
            screen_x = int(self.map_rect.x + clamped_x * self.scale_x)
            screen_y = int(self.map_rect.y + self.map_rect.height - clamped_y * self.scale_y)
            return (screen_x, screen_y)
        x_ratio = (max(0.0, min(CFG.AREA_WIDTH, x)) / max(CFG.AREA_WIDTH, 1.0)) - 0.5
        angle = math.pi / 2 + x_ratio * (2.0 * self.radar_half_span)
        radius = max(0.0, min(CFG.OUR_BASE_LINE, CFG.OUR_BASE_LINE - max(0.0, min(CFG.OUR_BASE_LINE, y)))) * self.scale
        screen_x = int(self.radar_cx + math.cos(angle) * radius)
        screen_y = int(self.radar_cy + math.sin(angle) * radius)
        return (screen_x, screen_y)

    def _sector_label(self, x):
        if x < CFG.AREA_WIDTH * 0.33:
            return "左翼"
        if x > CFG.AREA_WIDTH * 0.67:
            return "右翼"
        return "中路"

    def _bearing_deg_from_ownship(self, x, y):
        ox, oy = self._ownship_world_center()
        dx = float(x) - ox
        dy = float(y) - oy
        if abs(dx) + abs(dy) < 1e-6:
            return 0.0
        return (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0

    def _bearing_label(self, x, y):
        if self.ui_style != "omni":
            return self._sector_label(x)
        bearing = self._bearing_deg_from_ownship(x, y)
        dirs = ("北", "东北", "东", "东南", "南", "西南", "西", "西北")
        label = dirs[int((bearing + 22.5) // 45.0) % len(dirs)]
        return f"{label} {bearing:03.0f}°"

    def _coerce_float(self, value):
        if value is None:
            return None
        try:
            return float(value)
        except Exception:
            return None

    def _enemy_true_range_m(self, enemy):
        raw = enemy.get('raw_track') or {}
        geo_reference = getattr(self.env, "geo_reference", None)
        if raw and geo_reference is not None:
            lon = self._coerce_float(raw.get("lon", raw.get("longitude")))
            lat = self._coerce_float(raw.get("lat", raw.get("latitude")))
            if lon is not None and lat is not None:
                x_local, y_local = geo_reference.lonlat_to_xy(lat, lon)
                return math.hypot(x_local, y_local)

        raw_x = self._coerce_float(raw.get("x"))
        raw_z = self._coerce_float(raw.get("z"))
        if raw_x is not None and raw_z is not None:
            return math.hypot(raw_x, raw_z)

        if self.ui_style == "omni":
            ox, oy = self._ownship_world_center()
            return math.hypot(float(enemy.get('x', 0.0)) - ox, float(enemy.get('y', 0.0)) - oy)

        return max(0.0, CFG.INTERCEPT_FAIL_LINE - enemy['y'])

    def _line_radius(self, y_world):
        clipped = max(0.0, min(CFG.OUR_BASE_LINE, y_world))
        return max(0.0, (CFG.OUR_BASE_LINE - clipped) * self.scale)

    def _screen_heading(self, x, y, heading_deg, lead=180.0):
        sx, sy = self._w2s(x, y)
        r = math.radians(heading_deg)
        tx, ty = self._w2s(x + math.cos(r) * lead, y + math.sin(r) * lead)
        dx = tx - sx
        dy = ty - sy
        if abs(dx) < 1e-3 and abs(dy) < 1e-3:
            return r
        return math.atan2(-dy, dx)

    def _ecol(self,e):
        if entity_is_destroyed(e): return self.C['ed']
        if e['state']==EState.PENETRATED: return self.C['red']
        etype = e.get('type')
        if etype == EType.LOITER: return self.C['eloiter']
        if etype == EType.DECOY: return self.C['edecoy']
        if etype == EType.DASH: return self.C['edash']
        p = e['y'] / CFG.INTERCEPT_FAIL_LINE
        if p > 0.8: return self.C['ec']
        if p > 0.5: return self.C['em']
        return self.C['ef']

    def _icol(self,i):
        s=self._friendly_render_state(i)
        if s==IState.DESTROYED: return self.C['ed']
        if i.get('jammed_by_interference'): return self.C['orange']
        if i.get('task_reserved'): return self.C['amber']
        if i.get('barrier_slot') is not None: return self.C['ib']
        if s==IState.STANDBY: return self.C['is']
        if s==IState.RETURNING: return self.C['ir']
        if s==IState.FOLLOWING: return self.C['if']
        return self.C['ia']

    def _friendly_render_state(self, it):
        state = it.get('state', IState.STANDBY)
        if state == IState.DESTROYED or entity_is_destroyed(it):
            return IState.DESTROYED
        if not it.get('external_controlled') or it.get('reported_at', -1.0) < 0.0:
            return state

        status_text = str(it.get('status_text', '')).strip().lower()
        if any(token in status_text for token in ("destroy", "dead", "损毁", "坠毁", "unnormal")):
            return IState.DESTROYED
        if any(token in status_text for token in ("return", "rtb", "返航")):
            return IState.RETURNING
        if any(token in status_text for token in ("follow", "wing", "随动")):
            return IState.FOLLOWING
        if any(token in status_text for token in ("launch", "起飞")):
            return IState.LAUNCHING
        if any(token in status_text for token in ("intercept", "active", "engage", "拦截", "作战", "飞行")):
            return IState.INTERCEPTING

        reported_speed = float(it.get('reported_speed', it.get('speed', 0.0)) or 0.0)
        reported_z = float(it.get('reported_z', it.get('z', 0.0)) or 0.0)
        if reported_speed > 0.5 or reported_z > 1.0:
            if state in (IState.STANDBY, IState.LANDED):
                return IState.INTERCEPTING
        return state

    def _draw_destroyed_marker(self, sx, sy, color, size=10):
        pg = self.pg
        pg.draw.line(self.screen, color, (sx - size, sy - size), (sx + size, sy + size), 3)
        pg.draw.line(self.screen, color, (sx - size, sy + size), (sx + size, sy - size), 3)
        pg.draw.circle(self.screen, (255, 255, 255), (sx, sy), max(4, size // 3), 1)

    def handle_event(self,ev):
        pg=self.pg
        if ev.type == pg.MOUSEBUTTONDOWN and getattr(ev, "button", None) == 1:
            mx, my = getattr(ev, "pos", pg.mouse.get_pos())
            for scheme_id, rect in self.scheme_button_rects:
                if rect.collidepoint(mx, my):
                    apply_scheme = getattr(self.env, "apply_demo_scheme", None)
                    if apply_scheme:
                        ok = apply_scheme(scheme_id, "右上角方案按钮")
                        name = getattr(self.env, "demo_scheme_name", "")
                        self.add_log("[UI]", f"切换方案{scheme_id}: {name}" if ok else f"方案{scheme_id}切换失败", "amber" if ok else "red")
                    return True
            if self.toggle_interference_button_rect and self.toggle_interference_button_rect.collidepoint(mx, my):
                toggle_interference = getattr(self.env, "toggle_demo_interference", None)
                if toggle_interference:
                    enabled = toggle_interference("右上角按钮")
                    self.add_log("[UI]", "强干扰已开启" if enabled else "强干扰已关闭", "amber" if enabled else "txtd")
                return True
            if self.cancel_reserve_button_rect and self.cancel_reserve_button_rect.collidepoint(mx, my):
                reserve_fn = getattr(self.env, "get_reserved_interceptor_count", None)
                reserved = reserve_fn() if reserve_fn else 0
                if reserved <= 0:
                    self.add_log("[UI]", "当前无LLM保留无人机", "txtd")
                else:
                    resp = self.env.process_command("取消保留")
                    if resp:
                        self.add_log("[CMD]", resp, "amber")
                return True
            if self.add_enemy_button_rect and self.add_enemy_button_rect.collidepoint(mx, my):
                add_enemy = getattr(self.env, "add_enemy_target", None)
                if add_enemy:
                    enemy = add_enemy("右上角按钮")
                    self.add_log("[UI]", f"新增敌方 F-{enemy['id']+1}", "red")
                return True
            if self.add_friendly_button_rect and self.add_friendly_button_rect.collidepoint(mx, my):
                add_friendly = getattr(self.env, "add_friendly_target", None)
                if add_friendly:
                    it = add_friendly("右上角按钮")
                    self.add_log("[UI]", f"新增我方 I-{it['id']+1}", "green")
                return True
            return False
        if ev.type==pg.MOUSEWHEEL:
            mx,my=pg.mouse.get_pos()
            p2_h = CFG.SCREEN_HEIGHT - self.oy
            if pg.Rect(self.p2x,self.oy,self.p2_width,p2_h).collidepoint(mx,my):
                self.p2_scroll-=ev.y*26
                self.p2_scroll=max(0,min(self.p2_scroll,max(0,self.p2_total_h-self.p2_view_h)))
            elif pg.Rect(self.p3x,self.enemy_panel_y,self.p3_width,self.enemy_panel_h).collidepoint(mx,my):
                self.enemy_scroll-=ev.y*24
                self.enemy_scroll=max(0,min(self.enemy_scroll,max(0,self.enemy_total_h-self.enemy_view_h)))
            elif pg.Rect(self.p3x,self.oy,self.p3_width,self.lvh).collidepoint(mx,my):
                self.lsy-=ev.y*22; self.lsy=max(0,min(self.lsy,max(0,self.lth-max(1,self.lvh-34))))
        return False

    def _dashed(self,x1,y1,x2,y2,c,w=1):
        pg=self.pg; dx,dy=x2-x1,y2-y1
        d=math.sqrt(dx*dx+dy*dy)
        if d<2: return
        segs=max(4,int(d/10)); ph=(self.fc*0.08)%1
        for i in range(segs):
            if(i+int(ph*segs))%2==0:
                t1,t2=i/segs,min((i+0.6)/segs,1.0)
                pg.draw.line(self.screen,c,(int(x1+dx*t1),int(y1+dy*t1)),(int(x1+dx*t2),int(y1+dy*t2)),w)

    def _wrap(self,text,font,mw):
        lines=[]; cur=""
        for para in text.replace('\r','').split('\n'):
            if para=="": lines.append(""); continue
            cur=""
            for ch in para:
                t=cur+ch
                if font.render(t,True,(0,0,0)).get_width()<=mw: cur=t
                else:
                    if cur: lines.append(cur); cur=ch
            if cur: lines.append(cur)
        return lines or [""]

    def _fit_text(self, text, font, max_width):
        if font.render(text, True, (0, 0, 0)).get_width() <= max_width:
            return text
        suffix = "..."
        trimmed = text
        while trimmed:
            trimmed = trimmed[:-1]
            candidate = trimmed + suffix
            if font.render(candidate, True, (0, 0, 0)).get_width() <= max_width:
                return candidate
        return suffix

    def _wrapped_logs(self, max_width):
        key = (self._log_revision, max_width, self.fcs.get_height(), self.fms.get_height())
        if key == self._log_cache_key:
            return self._log_cache

        rendered = []
        for pfx, msg, ck in self.logs:
            color = self.C.get(ck, self.C['txt2'])
            prefix = self.fms.render(pfx, True, color) if pfx else None
            prefix_w = prefix.get_width() if prefix else 0
            lines = self._wrap(msg, self.fcs, max_width)
            for idx, line in enumerate(lines):
                rendered.append((
                    prefix if idx == 0 else None,
                    prefix_w if idx == 0 else 0,
                    self.fcs.render(line, True, self.C['txt2']),
                ))

        self._log_cache = rendered
        self._log_cache_key = key
        return rendered

    def render(self, spd=4, fps_value=None):
        pg=self.pg; self.fc+=1; self.pulse=(self.pulse+0.025)%(2*math.pi)
        fps_numeric = None
        if fps_value is not None:
            try:
                fps_numeric = float(fps_value)
            except Exception:
                fps_numeric = None
        if fps_numeric is not None and fps_numeric > 0.1:
            self._fps = int(round(fps_numeric))
            self._fps_c = 0
            self._fps_t = time.time()
        else:
            self._fps_c += 1
            now = time.time()
            if now - self._fps_t >= 0.5:
                self._fps = max(1, int(self._fps_c / (now - self._fps_t)))
                self._fps_c = 0
                self._fps_t = now
        if getattr(self.env, "scene_revision", self._scene_revision) != self._scene_revision:
            self._refresh_scene_geometry()
        self._draw_bg(); self._draw_radar(); self._draw_entities()
        self._draw_hdr(spd); self._draw_p2(); self._draw_p3(); self._draw_cmd()
        if self.env.done: self._draw_result()
        pg.display.flip()

    def _draw_bg(self):
        key = (CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT)
        if self._bg_cache is None or self._bg_cache_key != key:
            pg = self.pg
            surf = pg.Surface((CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT))
            surf.fill(self.C['bg'])
            grid_c = (18, 22, 30)
            for x in range(0, CFG.SCREEN_WIDTH, 140):
                pg.draw.line(surf, grid_c, (x, 0), (x, CFG.SCREEN_HEIGHT), 1)
            for y in range(0, CFG.SCREEN_HEIGHT, 140):
                pg.draw.line(surf, grid_c, (0, y), (CFG.SCREEN_WIDTH, y), 1)
            self._bg_cache = surf
            self._bg_cache_key = key
        self.screen.blit(self._bg_cache, (0, 0))

    def _draw_radar(self):
        key = (
            CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT, self.ui_style, self.p1x, self.oy,
            self.p1_width, self._scene_revision, self.fcs.get_height(), self.fms.get_height(),
        )
        if self._radar_cache is None or self._radar_cache_key != key:
            pg = self.pg
            surf = pg.Surface((CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT), pg.SRCALPHA)
            panel_rect = (self.p1x, self.oy-2, self.p1_width, CFG.SCREEN_HEIGHT - self.oy + 4)
            pg.draw.rect(surf, self.C['pnl'], panel_rect, border_radius=4)
            pg.draw.rect(surf, self.C['gdd'], panel_rect, 1, border_radius=4)
            if self.ui_style == "rect":
                self._draw_radar_rect_static(surf)
            elif self.ui_style == "omni":
                self._draw_radar_omni_static(surf)
            else:
                self._draw_radar_arc_static(surf)
            self._radar_cache = surf
            self._radar_cache_key = key
        self.screen.blit(self._radar_cache, (0, 0))
        if self.ui_style == "arc":
            self._draw_arc_sweep()
        elif self.ui_style == "omni":
            self._draw_omni_sweep()
            self._draw_omni_center_status()
        self._draw_hangar_counts()
        self._draw_enemy_hud()

    def _draw_arc_zone(self, target, y_world, label, color, dash=True):
        pg = self.pg
        rr = int(self._line_radius(y_world))
        if rr <= 2:
            pg.draw.circle(target, color, (self.radar_cx, self.radar_cy), 3, 1)
            txt = self.fcs.render(label, True, color)
            target.blit(txt, (self.radar_cx + 8, self.radar_cy + 2))
            return
        rect = pg.Rect(self.radar_cx - rr, self.radar_cy - rr, rr * 2, rr * 2)
        base_color = tuple(max(12, min(255, int(c * 0.35))) for c in color)
        if dash:
            for seg in range(40):
                start = (math.pi / 2 - self.radar_half_span) + (seg / 40.0) * (2.0 * self.radar_half_span)
                end = start + 0.033
                if seg % 2 == 0:
                    pg.draw.arc(target, base_color, rect, start, min(end, math.pi / 2 + self.radar_half_span), 5)
                    pg.draw.arc(target, color, rect, start, min(end, math.pi / 2 + self.radar_half_span), 3)
        else:
            pg.draw.arc(target, base_color, rect, math.pi / 2 - self.radar_half_span, math.pi / 2 + self.radar_half_span, 5)
            pg.draw.arc(target, color, rect, math.pi / 2 - self.radar_half_span, math.pi / 2 + self.radar_half_span, 3)
        lx = self.radar_cx + int(math.cos(math.pi / 2 + self.radar_half_span) * rr) + 12
        ly = self.radar_cy + int(math.sin(math.pi / 2 + self.radar_half_span) * rr) - 8
        target.blit(self.fcs.render(label, True, color), (lx, ly))

    def _draw_hangar_frames(self, target):
        pg = self.pg
        for idx, hx in enumerate(CFG.HANGAR_POSITIONS):
            sx, sy = self._w2s(hx, CFG.INTERCEPT_FAIL_LINE + 200)
            rect = (sx - 30, sy - 45, 60, 40)
            pg.draw.rect(target, (10, 20, 30), rect, border_radius=4)
            pg.draw.rect(target, self.C['cyd'], rect, 2, border_radius=4)
            lbl = self.fms.render("HANGAR", True, self.C['cyan'])
            target.blit(lbl, (sx - lbl.get_width() // 2, sy - 40))

    def _draw_hangar_counts(self):
        if self.ui_style == "omni":
            return
        counts = [0 for _ in CFG.HANGAR_POSITIONS]
        for it in self._visible_friendlies():
            if self._friendly_render_state(it) in (IState.STANDBY, IState.LANDED):
                counts[it.get('hangar_idx', 0)] += 1
        for idx, hx in enumerate(CFG.HANGAR_POSITIONS):
            sx, sy = self._w2s(hx, CFG.INTERCEPT_FAIL_LINE + 200)
            cnt = self.fcs.render(
                f"x{counts[idx]}",
                True,
                self.C['green'] if counts[idx] > 0 else self.C['red'],
            )
            self.screen.blit(cnt, (sx - cnt.get_width() // 2, sy - 23))

    def _draw_radar_arc_static(self, target):
        pg = self.pg
        sector_pts = [(self.radar_cx, self.radar_cy)]
        for idx in range(36):
            t = idx / 35.0
            ang = (math.pi / 2 - self.radar_half_span) + t * (2.0 * self.radar_half_span)
            sector_pts.append((
                int(self.radar_cx + math.cos(ang) * self.radar_radius),
                int(self.radar_cy + math.sin(ang) * self.radar_radius),
            ))
        pg.draw.polygon(target, (5, 12, 18, 245), sector_pts, 0)

        for i in range(1, 7):
            ratio = i / 6.0
            rr = int(self.radar_radius * ratio)
            rect = pg.Rect(self.radar_cx - rr, self.radar_cy - rr, rr * 2, rr * 2)
            pg.draw.arc(target, self.C['gdd'], rect, math.pi / 2 - self.radar_half_span, math.pi / 2 + self.radar_half_span, 1)
            range_km = (CFG.OUR_BASE_LINE * ratio) / 1000.0
            tx = self.radar_cx + int(math.cos(math.pi / 2 + self.radar_half_span) * rr) + 10
            ty = self.radar_cy + int(math.sin(math.pi / 2 + self.radar_half_span) * rr) - 10
            target.blit(self.fms.render(f"{range_km:.1f}km", True, self.C['txtd']), (tx, ty))

        for i in range(7):
            ratio = i / 6.0
            ang = (math.pi / 2 - self.radar_half_span) + ratio * (2.0 * self.radar_half_span)
            ex = int(self.radar_cx + math.cos(ang) * self.radar_radius)
            ey = int(self.radar_cy + math.sin(ang) * self.radar_radius)
            pg.draw.line(target, self.C['gdd'], (self.radar_cx, self.radar_cy), (ex, ey), 1)

        self._draw_arc_zone(target, CFG.DETECTION_LINE, "警戒线", self.C['amber'])
        self._draw_arc_zone(target, CFG.INTERCEPT_FAIL_LINE, "防线", self.C['green'])
        self._draw_arc_zone(target, CFG.OUR_BASE_LINE, "基地", self.C['blue'], False)
        self._draw_hangar_frames(target)

        cl = 18
        cc = self.C['cyan']
        left_arc = (
            int(self.radar_cx + math.cos(math.pi / 2 - self.radar_half_span) * self.radar_radius),
            int(self.radar_cy + math.sin(math.pi / 2 - self.radar_half_span) * self.radar_radius),
        )
        right_arc = (
            int(self.radar_cx + math.cos(math.pi / 2 + self.radar_half_span) * self.radar_radius),
            int(self.radar_cy + math.sin(math.pi / 2 + self.radar_half_span) * self.radar_radius),
        )
        for cx, cy, dx in ((left_arc[0], left_arc[1], 1), (right_arc[0], right_arc[1], -1)):
            pg.draw.line(target, cc, (cx, cy), (cx + dx * cl, cy), 2)
            pg.draw.line(target, cc, (cx, cy), (cx, cy - cl), 2)
        pg.draw.circle(target, cc, (self.radar_cx, self.radar_cy), 5, 1)

    def _draw_radar_rect_static(self, target):
        pg = self.pg
        ox, oy = self.map_rect.x, self.map_rect.y
        w, h = self.map_rect.width, self.map_rect.height
        pg.draw.rect(target, (5, 8, 14), self.map_rect)

        for i in range(1, 7):
            yy = oy + int(i * h / 7.0)
            pg.draw.line(target, self.C['gdd'], (ox, yy), (ox + w, yy), 1)
            world_y = CFG.OUR_BASE_LINE * (1 - i / 7.0)
            target.blit(self.fms.render(f"{world_y/1000:.1f}km", True, self.C['txtd']), (ox + 4, yy - 12))

        for i in range(1, 10):
            xx = ox + int(i * w / 10)
            pg.draw.line(target, self.C['gdd'], (xx, oy), (xx, oy + h), 1)

        def zl_arc(y_world, label, color):
            sy = oy + h - int(max(0.0, min(CFG.OUR_BASE_LINE, y_world)) * self.scale_y)
            curve_h = max(16, int(h * 0.078))
            low_y = sy - max(10, int(h * 0.015))
            pts = []
            for idx in range(61):
                t = idx / 60.0
                xx = ox + int(t * w)
                bow = 1.0 - ((t * 2.0 - 1.0) ** 2)
                yy = int(low_y - curve_h * (1.0 - bow))
                pts.append((xx, yy))
            base_color = tuple(max(12, min(255, int(c * 0.35))) for c in color)
            pg.draw.lines(target, base_color, False, pts, 5)
            pg.draw.lines(target, color, False, pts, 3)
            txt = self.fcs.render(label, True, color)
            target.blit(txt, (ox + w - txt.get_width() - 8, low_y + 4))

        def zl_line(y_world, label, color):
            sy = oy + h - int(max(0.0, min(CFG.OUR_BASE_LINE, y_world)) * self.scale_y)
            pg.draw.line(target, color, (ox, sy), (ox + w, sy), 2)
            txt = self.fcs.render(label, True, color)
            target.blit(txt, (ox + w - txt.get_width() - 8, sy + 4))

        zl_arc(CFG.DETECTION_LINE, "警戒线", self.C['amber'])
        zl_arc(CFG.INTERCEPT_FAIL_LINE, "防线", self.C['green'])
        zl_line(CFG.OUR_BASE_LINE, "基地", self.C['blue'])
        self._draw_hangar_frames(target)

        cl = 24
        for cx, cy, dx, dy in [(ox, oy, 1, 1), (ox + w, oy, -1, 1), (ox, oy + h, 1, -1), (ox + w, oy + h, -1, -1)]:
            pg.draw.line(target, self.C['cyan'], (cx, cy), (cx + dx * cl, cy), 2)
            pg.draw.line(target, self.C['cyan'], (cx, cy), (cx, cy + dy * cl), 2)

    def _draw_radar_omni_static(self, target):
        pg = self.pg
        cx, cy, rr = self.radar_cx, self.radar_cy, self.radar_radius
        pg.draw.circle(target, (5, 10, 14), (cx, cy), rr, 0)
        pg.draw.circle(target, self.C['cyd'], (cx, cy), rr, 2)

        for i in range(1, 7):
            ring_r = int(rr * i / 6.0)
            pg.draw.circle(target, self.C['gdd'], (cx, cy), ring_r, 1)
            range_km = (self.radar_range_m * i / 6.0) / 1000.0
            txt = self.fms.render(f"{range_km:.1f}km", True, self.C['txtd'])
            tx = min(self.p1x + self.p1_width - txt.get_width() - 12, cx + ring_r + 6)
            target.blit(txt, (tx, cy - txt.get_height() // 2))

        for deg in range(0, 360, 15):
            ang = math.radians(90.0 - deg)
            length = rr if deg % 45 == 0 else int(rr * 0.96)
            color = self.C['gdd'] if deg % 45 else self.C['cyd']
            ex = cx + int(math.cos(ang) * length)
            ey = cy - int(math.sin(ang) * length)
            pg.draw.line(target, color, (cx, cy), (ex, ey), 1)

        cardinal = [
            ("N", 0, 0, -1), ("E", 90, 1, 0), ("S", 180, 0, 1), ("W", 270, -1, 0),
        ]
        for label, _deg, vx, vy in cardinal:
            surf = self.fcn.render(label, True, self.C['cyan'])
            tx = cx + int(vx * (rr - 24)) - surf.get_width() // 2
            ty = cy + int(vy * (rr - 24)) - surf.get_height() // 2
            target.blit(surf, (tx, ty))

        pg.draw.line(target, self.C['green'], (cx - 18, cy), (cx + 18, cy), 2)
        pg.draw.line(target, self.C['green'], (cx, cy - 18), (cx, cy + 18), 2)
        pg.draw.circle(target, self.C['green'], (cx, cy), 24, 1)

        title = self.fcn.render("◆ 360°全向雷达", True, self.C['cyan'])
        target.blit(title, (self.p1x + 12, self.oy + 8))

    def _draw_omni_sweep(self):
        pg = self.pg
        sweep_deg = (self.fc * 1.7) % 360.0
        ang = math.radians(90.0 - sweep_deg)
        sx = int(self.radar_cx + math.cos(ang) * self.radar_radius)
        sy = int(self.radar_cy - math.sin(ang) * self.radar_radius)
        pg.draw.line(self.screen, (0, 185, 150), (self.radar_cx, self.radar_cy), (sx, sy), 2)
        pg.draw.circle(self.screen, (0, 140, 120), (self.radar_cx, self.radar_cy), self.radar_radius, 1)

    def _draw_omni_center_status(self):
        pg = self.pg
        cx, cy = self.radar_cx, self.radar_cy
        friendlies = self._visible_friendlies()
        airborne = sum(
            1 for it in friendlies
            if self._friendly_render_state(it) in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
        )
        label = self.fcs.render(f"我方 I:{len(friendlies)} 空:{airborne}", True, self.C['green'])
        pg.draw.circle(self.screen, (0, 255, 128), (cx, cy), 6, 0)
        pg.draw.circle(self.screen, (0, 255, 128), (cx, cy), 18 + int(2 * math.sin(self.pulse * 2.0)), 1)
        self.screen.blit(label, (cx - label.get_width() // 2, cy + 24))

    def _draw_arc_sweep(self):
        pg = self.pg
        sweep_ang = (math.pi / 2 - self.radar_half_span) + ((self.fc % 260) / 259.0) * (2.0 * self.radar_half_span)
        sweep_x = int(self.radar_cx + math.cos(sweep_ang) * self.radar_radius)
        sweep_y = int(self.radar_cy + math.sin(sweep_ang) * self.radar_radius)
        pg.draw.line(self.screen, (0, 165, 135), (self.radar_cx, self.radar_cy), (sweep_x, sweep_y), 1)

    def _draw_enemy_hud(self):
        pg = self.pg
        px, py, pw, ph = self.p3x, self.enemy_panel_y, self.p3_width, self.enemy_panel_h
        header_h = 36
        pg.draw.rect(self.screen, self.C['pnl'], (px, py, pw, ph), border_radius=4)
        pg.draw.rect(self.screen, self.C['cyd'], (px, py, pw, ph), 1, border_radius=4)
        self.screen.blit(self.fcn.render("◆ 入侵目标列表", True, self.C['cyan']), (px + 10, py + 8))

        active = [
            e for e in self.env.enemies
            if e['state'] in (EState.APPROACHING, EState.MANEUVERING) and not entity_is_destroyed(e)
        ]
        total_enemies = len([e for e in self.env.enemies if not entity_is_destroyed(e)])
        count_text = f"{len(active)}/{total_enemies}"
        count_surf = self.fms.render(count_text, True, self.C['amber'] if active else self.C['txtd'])
        self.screen.blit(count_surf, (px + pw - count_surf.get_width() - 12, py + 12))
        pg.draw.line(self.screen, self.C['cyd'], (px + 8, py + header_h), (px + pw - 8, py + header_h), 1)

        active.sort(key=lambda item: item['y'], reverse=True)
        view_y = py + header_h + 4
        self.enemy_view_h = ph - header_h - 8
        clip_rect = pg.Rect(px + 4, view_y, pw - 8, self.enemy_view_h)
        if not active:
            self.enemy_total_h = 0
            self.enemy_scroll = 0
            self.screen.blit(self.fcs.render("当前无入侵目标", True, self.C['txtd']), (px + 12, py + header_h + 12))
            return

        cols = 2
        gap_x = 6
        gap_y = 6
        inner_x = px + 8
        card_w = (pw - 16 - gap_x) // cols
        card_h = max(52, self.fcs.get_linesize() * 2 + 18)
        total_rows = (len(active) + cols - 1) // cols
        self.enemy_total_h = total_rows * (card_h + gap_y)
        self.enemy_scroll = max(0, min(self.enemy_scroll, max(0, self.enemy_total_h - self.enemy_view_h)))
        prev_clip = self.screen.get_clip()
        self.screen.set_clip(clip_rect)
        for idx, enemy in enumerate(active):
            row = idx // cols
            col = idx % cols
            cx = inner_x + col * (card_w + gap_x)
            cy = view_y + row * (card_h + gap_y) - self.enemy_scroll
            if cy + card_h < view_y or cy > view_y + self.enemy_view_h:
                continue
            color = self._ecol(enemy)
            sector = self._bearing_label(enemy['x'], enemy['y'])
            dist = self._enemy_true_range_m(enemy)
            pg.draw.rect(self.screen, (12, 16, 24), (cx, cy, card_w, card_h), border_radius=3)
            pg.draw.rect(self.screen, color, (cx, cy, card_w, card_h), 1, border_radius=3)
            self.screen.blit(self.fm.render(f"F-{enemy['id']+1:02d}", True, color), (cx + 6, cy + 4))
            dist_surf = self.fm.render(f"{dist:.0f}m", True, self.C['txt2'])
            self.screen.blit(dist_surf, (cx + card_w - dist_surf.get_width() - 6, cy + 4))
            self.screen.blit(self.fcs.render(sector, True, self.C['txt']), (cx + 6, cy + 24))
            alt_surf = self.fcs.render(f"Z {enemy.get('z', 0.0):.0f}", True, self.C['txtd'])
            self.screen.blit(alt_surf, (cx + card_w - alt_surf.get_width() - 6, cy + 24))
        self.screen.set_clip(prev_clip)
        if self.enemy_total_h > self.enemy_view_h:
            bar_x = px + pw - 6
            bar_y = view_y + 3
            bar_h = self.enemy_view_h - 6
            pg.draw.rect(self.screen, (20, 26, 34), (bar_x, bar_y, 3, bar_h), border_radius=2)
            knob_h = max(30, int(bar_h * self.enemy_view_h / max(self.enemy_total_h, 1)))
            max_scroll = max(1, self.enemy_total_h - self.enemy_view_h)
            knob_y = bar_y + int((bar_h - knob_h) * self.enemy_scroll / max_scroll)
            pg.draw.rect(self.screen, self.C['cyan'], (bar_x, knob_y, 3, knob_h), border_radius=2)

    def _draw_net_overlay(self):
        pg = self.pg
        overlay = pg.Surface((CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT), pg.SRCALPHA)
        labels = []

        barrier_map = getattr(self.env, 'barrier_team_assignments', {})
        for enemy_id, team in barrier_map.items():
            enemy = next((e for e in self.env.enemies if e['id'] == enemy_id), None)
            if not enemy or enemy['state'] not in (EState.APPROACHING, EState.MANEUVERING):
                continue
            members = []
            for iid in team:
                it = next((item for item in self.env.interceptors if item['id'] == iid), None)
                if not it or it['state'] in (IState.STANDBY, IState.LANDED, IState.DESTROYED):
                    continue
                if it.get('barrier_slot') is None:
                    continue
                members.append((it.get('barrier_slot', 0), it))
            if len(members) < 2:
                continue
            members.sort(key=lambda item: item[0])
            points = [self._w2s(friendly_view(it)['x'], friendly_view(it)['y']) for _, it in members]
            center = getattr(self.env, 'barrier_states', {}).get(enemy_id, {}).get('center')
            if center is None:
                center = (enemy['x'], enemy['y'], enemy.get('z', 0.0))
            cx, cy = self._w2s(center[0], center[1])
            scale = min(self.sx, self.sy)
            net_r = max(14, int(CFG.BARRIER_NET_RADIUS * scale))
            band_w = max(10, int(CFG.BARRIER_NET_RADIUS * scale * 0.72))

            if len(points) >= 2:
                pg.draw.line(overlay, (255, 198, 96, 42), points[0], points[-1], band_w)
                pg.draw.lines(overlay, (255, 170, 70, 120), False, points, 7)
                pg.draw.lines(overlay, (255, 232, 150, 190), False, points, 3)
            for px, py in points:
                pg.draw.circle(overlay, (255, 190, 90, 34), (px, py), net_r, 0)
                pg.draw.circle(overlay, (255, 228, 148, 150), (px, py), net_r, 2)
                pg.draw.circle(overlay, (255, 245, 210, 220), (px, py), 4, 0)
            pg.draw.circle(overlay, (255, 210, 120, 80), (cx, cy), 5, 0)
            labels.append(("NET BELT", self.C['ib'], cx + 10, cy - 22))

        self.screen.blit(overlay, (0, 0))
        for text, color, tx, ty in labels:
            text_surf = self.fms.render(text, True, color)
            self.screen.blit(text_surf, (tx, ty))

        if getattr(self.env, 'intercept_mode', 'hybrid') != 'legacy-net':
            return

    def _zone_polygon(self, zone, steps=12):
        if self.ui_style == "rect":
            x1, y1 = self._w2s(zone['x1'], zone['y1'])
            x2, y2 = self._w2s(zone['x2'], zone['y2'])
            left, right = min(x1, x2), max(x1, x2)
            top, bottom = min(y1, y2), max(y1, y2)
            return [(left, top), (right, top), (right, bottom), (left, bottom)]

        points = []
        for idx in range(steps + 1):
            t = idx / float(steps)
            x = zone['x1'] + (zone['x2'] - zone['x1']) * t
            points.append(self._w2s(x, zone['y1']))
        for idx in range(steps + 1):
            t = idx / float(steps)
            y = zone['y1'] + (zone['y2'] - zone['y1']) * t
            points.append(self._w2s(zone['x2'], y))
        for idx in range(steps, -1, -1):
            t = idx / float(steps)
            x = zone['x1'] + (zone['x2'] - zone['x1']) * t
            points.append(self._w2s(x, zone['y2']))
        for idx in range(steps, -1, -1):
            t = idx / float(steps)
            y = zone['y1'] + (zone['y2'] - zone['y1']) * t
            points.append(self._w2s(zone['x1'], y))
        return points

    def _draw_interference_overlay(self):
        if not getattr(self.env, 'demo_interference_visible', True):
            return
        zones = getattr(self.env, 'demo_interference_zones', []) or []
        if not zones:
            return
        pg = self.pg
        overlay = pg.Surface((CFG.SCREEN_WIDTH, CFG.SCREEN_HEIGHT), pg.SRCALPHA)
        labels = []
        fill_colors = [(255, 82, 32, 34), (255, 180, 30, 30)]
        border = (255, 190, 80, 150)
        for idx, zone in enumerate(zones):
            no_fly = bool(zone.get('llm_no_fly'))
            zone_fill = (255, 24, 24, 62) if no_fly else fill_colors[idx % len(fill_colors)]
            zone_border = (255, 70, 70, 230) if no_fly else border
            label = zone.get('label', '干扰')
            if no_fly:
                label = f"{label} 禁入"
            if 'cx' in zone and 'cy' in zone and 'radius' in zone:
                cx, cy = self._w2s(zone['cx'], zone['cy'])
                rr = max(10, int(zone['radius'] * min(self.sx, self.sy)))
                pg.draw.circle(overlay, zone_fill, (cx, cy), rr, 0)
                pg.draw.circle(overlay, zone_border, (cx, cy), rr, 3 if no_fly else 2)
                pg.draw.circle(overlay, (255, 230, 150, 230), (cx, cy), 5, 0)
                pg.draw.circle(overlay, (35, 20, 12, 235), (cx, cy), 8, 2)
                labels.append((label, cx + rr + 8, cy - 14))
                continue
            pts = self._zone_polygon(zone)
            if len(pts) < 3:
                continue
            pg.draw.polygon(overlay, zone_fill, pts, 0)
            pg.draw.polygon(overlay, zone_border, pts, 3 if no_fly else 2)
            lx, ly = self._w2s((zone['x1'] + zone['x2']) * 0.5, zone['y2'])
            labels.append((label, lx + 8, ly - 18))
        self.screen.blit(overlay, (0, 0))
        for text, tx, ty in labels:
            self.screen.blit(self.fms.render(text, True, self.C['amber']), (tx, ty))

    def _draw_entities(self):
        pg = self.pg; now = self.env.time
        scan_period = 3.0; scan_decay = 2.5

        # 绘制拦截线
        for e in self.env.enemies:
            if e['state'] not in (EState.APPROACHING,EState.MANEUVERING): continue
            asgn=self.env.assigner.get_info(e['id'])
            if not asgn: continue
            pid,fid=asgn.get('primary'),asgn.get('follower')
            if self.show_poi:
                for role,iid in[('p',pid),('f',fid)]:
                    if iid is None: continue
                    it=next((i for i in self.env.interceptors if i['id']==iid),None)
                    if not it or it['state'] in (IState.DESTROYED,IState.STANDBY): continue
                    if it.get('poi'):
                        itv = friendly_view(it)
                        ix,iy=self._w2s(itv['x'],itv['y']); px,py=self._w2s(it['poi'][0],it['poi'][1])
                        c=self.C['amber'] if role=='p' else self.C['cyd']
                        self._dashed(ix,iy,px,py,c,2 if role=='p' else 1)
                        pg.draw.circle(self.screen,c,(px,py),4,1)
            if pid is not None and fid is not None:
                pi=next((i for i in self.env.interceptors if i['id']==pid),None)
                fi=next((i for i in self.env.interceptors if i['id']==fid),None)
                if pi and fi and pi['state'] not in (IState.DESTROYED,IState.STANDBY) and fi['state'] not in (IState.DESTROYED,IState.STANDBY):
                    piv = friendly_view(pi); fiv = friendly_view(fi)
                    p1,p2=self._w2s(piv['x'],piv['y']),self._w2s(fiv['x'],fiv['y'])
                    self._dashed(p1[0],p1[1],p2[0],p2[1],self.C['blue'],1)

        self._draw_net_overlay()
        self._draw_interference_overlay()

        # === 绘制敌机 (含轨迹，无文字) ===
        for e in self.env.enemies:
            is_destroyed = entity_is_destroyed(e)
            if e['id'] not in self.enemy_trails: self.enemy_trails[e['id']] = []
            if (not is_destroyed) and e['state'] in (EState.APPROACHING, EState.MANEUVERING):
                if self.fc % 8 == 0:
                    self.enemy_trails[e['id']].append((e['x'], e['y']))
                    if len(self.enemy_trails[e['id']]) > self.trail_length:
                        self.enemy_trails[e['id']].pop(0)

            trail = self.enemy_trails[e['id']]
            if len(trail) > 1 and not is_destroyed:
                pts = [self._w2s(p[0], p[1]) for p in trail]
                trail_color = (150, 150, 50) if e.get('type') in (EType.SNAKE, EType.JINK) else (60, 70, 80)
                if e.get('type') == EType.LOITER: trail_color = (120, 60, 120)
                pg.draw.lines(self.screen, trail_color, False, pts, 1)

            if self.ui_style == "omni":
                sweep_deg = (self.fc * 1.7) % 360.0
                bearing = self._bearing_deg_from_ownship(e['x'], e['y'])
                diff_deg = abs((bearing - sweep_deg + 180.0) % 360.0 - 180.0)
                brightness = math.exp(-diff_deg / 85.0)
            else:
                scan_progress = (now % scan_period) / scan_period
                rel_x = e['x'] / CFG.AREA_WIDTH
                diff = scan_progress - rel_x
                if diff < 0: diff += 1.0
                brightness = math.exp(-scan_decay * diff)
            brightness = max(0.15, brightness)
            if e['y'] > 8000 or is_destroyed: brightness = 1.0
            base_c = self._ecol(e)
            c = (int(base_c[0]*brightness), int(base_c[1]*brightness), int(base_c[2]*brightness))
            sx,sy=self._w2s(e['x'],e['y'])

            if e.get('type') == EType.LOITER and e.get('loiter_center'):
                lc = e['loiter_center']; lsx, lsy = self._w2s(lc[0], lc[1])
                lr = int(CFG.LOITER_RADIUS * self.sx)
                track_c = (int(80*brightness), 0, int(80*brightness))
                pg.draw.circle(self.screen, track_c, (lsx, lsy), lr, 1)
                if brightness > 0.4: self.screen.blit(self.fms.render("Loiter", True, (int(200*brightness), 0, int(200*brightness))), (lsx - 15, lsy - 6))

            if is_destroyed:
                continue

            if e['state'] in (EState.APPROACHING, EState.MANEUVERING):
                sz = 8
                if e.get('type') == EType.DECOY: sz = 8
                pts = [(sx, sy - sz), (sx + sz, sy), (sx, sy + sz), (sx - sz, sy)]
                pg.draw.polygon(self.screen, c, pts, 0)
                pg.draw.polygon(self.screen, (int(255*brightness), int(255*brightness), int(255*brightness)), pts, 1)

                if brightness > 0.3:
                    if self.show_poi:
                        er = math.radians(e['heading']); pl = e['speed'] * 8
                        ex = sx + int(math.cos(er) * pl * self.sx); ey = sy - int(math.sin(er) * pl * self.sy)
                        self._dashed(sx, sy, ex, ey, c, 1)
                    lbl = f"F-{e['id'] + 1} {self._bearing_label(e['x'], e['y'])}"
                    self.screen.blit(self.fms.render(lbl, True, c), (sx + 10, sy - 6))

        # 绘制拦截机
        for it in self._visible_friendlies():
            render_state = self._friendly_render_state(it)
            if render_state in (IState.STANDBY, IState.LANDED): continue

            view = friendly_view(it)
            sx,sy=self._w2s(view['x'],view['y']); c=self._icol(it)
            if render_state == IState.DESTROYED:
                continue
            r=self._screen_heading(view['x'], view['y'], view['heading'])
            sz=14
            tip = (sx + int(math.cos(r)*sz), sy - int(math.sin(r)*sz))
            left = (sx + int(math.cos(r+2.5)*sz*0.7), sy - int(math.sin(r+2.5)*sz*0.7))
            right = (sx + int(math.cos(r-2.5)*sz*0.7), sy - int(math.sin(r-2.5)*sz*0.7))
            pg.draw.polygon(self.screen,c,[tip,left,right],0)
            pg.draw.polygon(self.screen,(255,255,255),[tip,left,right],1)
            if it.get('jammed_by_interference'):
                ring = 18 + int(3 * math.sin(self.pulse * 3.0))
                pg.draw.circle(self.screen, self.C['amber'], (sx, sy), ring, 2)
                self.screen.blit(self.fms.render("JAM", True, self.C['amber']), (sx + 10, sy + 10))
            rs="P" if it['role']==IRole.PRIMARY else("F" if it['role']==IRole.FOLLOWER else "R")
            self.screen.blit(self.fms.render(f"I-{it['id']+1}{rs}",True,c),(sx+10,sy-6))

            # === 锁敌连线 ===
            if render_state in (IState.INTERCEPTING, IState.FOLLOWING) and it['target_id'] is not None:
                tgt = next((e for e in self.env.enemies if e['id'] == it['target_id']), None)
                if tgt and tgt['state'] not in (EState.DESTROYED, EState.PENETRATED):
                    d = math.hypot(view['x'] - tgt['x'], view['y'] - tgt['y'])
                    if d < 3000:
                        tx, ty = self._w2s(tgt['x'], tgt['y'])
                        intensity = max(0, min(1, 1 - d/3000))
                        line_c = (int(255*intensity), int(150*intensity), 50)
                        width = 2 if d < 1000 else 1
                        pg.draw.line(self.screen, line_c, (sx, sy), (tx, ty), width)

    def _draw_hdr(self,spd):
        pg=self.pg
        self.screen.blit(self.fcl.render("空域拦截防御系统",True,self.C['cyan']),(20,12))
        pc=int(150+50*math.sin(self.pulse*2))
        src = "实时" if getattr(self.env, "has_live_data", False) else ("本地回放" if getattr(self.env, "demo_mode", False) else "等待数据")
        mode_map = {"net": "BARRIER", "hit": "HIT", "hybrid": "HYBRID", "legacy-net": "L-NET"}
        mode = mode_map.get(getattr(self.env, "intercept_mode", "hybrid"), "HYBRID")
        ui_mode = {"arc": "ARC", "rect": "RECT", "omni": "OMNI"}.get(self.ui_style, "ARC")
        llm_state = getattr(getattr(self.env, "analyst", None), "channel_state", "ATTACHED")
        jam_count = sum(1 for it in getattr(self.env, "interceptors", []) if it.get('jammed_by_interference'))
        reserve_fn = getattr(self.env, "get_reserved_interceptor_count", None)
        reserved_count = reserve_fn() if reserve_fn else 0
        jam_text = f" | JAM {jam_count}" if getattr(self.env, "demo_interference_enabled", False) else ""
        reserve_text = f" | RSV {reserved_count}" if reserved_count else ""
        scheme_id = int(getattr(self.env, "demo_scheme", 0) or 0)
        scheme_name = getattr(self.env, "demo_scheme_name", "")
        scheme_text = f" | S{scheme_id} {scheme_name}" if scheme_id else ""
        self.screen.blit(
            self.fm.render(
                f"AIR DEFENSE COMMAND v8.0 | Scene {CFG.SCENE_KM:.0f}km | {src} | {mode} | UI {ui_mode} | LLM {llm_state}{jam_text}{reserve_text}{scheme_text}",
                True,
                (0,pc,pc),
            ),
            (20,46),
        )
        bx=self.p2x
        if math.isfinite(CFG.TIME_LIMIT):
            rem=max(0,CFG.TIME_LIMIT-self.env.time)
            m,s=int(rem//60),int(rem%60)
            time_text = f"T-{m:02d}:{s:02d}"
            tc=self.C['red'] if rem<60 else self.C['txt']
        else:
            time_text = "T-\u221e"
            tc=self.C['txt']
        pg.draw.rect(self.screen,self.C['pnl'],(bx,12,self.p2_width,60),border_radius=3)
        pg.draw.rect(self.screen,self.C['cyd'],(bx,12,self.p2_width,60),1,border_radius=3)
        self.screen.blit(self.fcn.render(time_text,True,tc),(bx+10,20))
        self.screen.blit(self.fms.render(f"{spd}x Speed",True,self.C['cyan']),(bx+10,45))
        fc=self.C['green'] if self._fps>=30 else self.C['red']
        self.screen.blit(self.fms.render(f"FPS:{self._fps}",True,fc),(bx+100,45))
        jam_enabled = getattr(self.env, "demo_interference_enabled", False)
        self._draw_quick_button(
            self.toggle_interference_button_rect,
            "关闭干扰" if jam_enabled else "开启干扰",
            self.C['amber'] if jam_enabled else self.C['txtd'],
        )
        reserve_color = self.C['amber'] if reserved_count else self.C['txtd']
        self._draw_quick_button(self.cancel_reserve_button_rect, "取消保留", reserve_color)
        self._draw_quick_button(self.add_enemy_button_rect, "+敌方", self.C['red'])
        self._draw_quick_button(self.add_friendly_button_rect, "+我方", self.C['green'])
        scheme_labels = {
            1: "1传统",
            2: "2协同",
            3: "3干扰",
        }
        active_scheme = int(getattr(self.env, "demo_scheme", 0) or 0)
        for sid, rect in self.scheme_button_rects:
            color = self.C['amber'] if sid == active_scheme else self.C['txt2']
            self._draw_quick_button(rect, scheme_labels.get(sid, str(sid)), color)

    def _draw_quick_button(self, rect, label, color):
        if rect is None:
            return
        pg = self.pg
        hover = rect.collidepoint(pg.mouse.get_pos())
        fill = (24, 31, 42) if not hover else (34, 43, 56)
        border = color if hover else tuple(max(35, int(c * 0.65)) for c in color)
        pg.draw.rect(self.screen, fill, rect, border_radius=4)
        pg.draw.rect(self.screen, border, rect, 2 if hover else 1, border_radius=4)
        text = self.fms.render(label, True, color)
        self.screen.blit(
            text,
            (
                rect.x + (rect.width - text.get_width()) // 2,
                rect.y + (rect.height - text.get_height()) // 2,
            ),
        )

    def _draw_p2(self):
        pg = self.pg; px, py, pw = self.p2x, self.oy, self.p2_width
        panel_h = CFG.SCREEN_HEIGHT - self.oy
        header_h = 34
        pg.draw.rect(self.screen, self.C['pnl'], (px, py, pw, panel_h), border_radius=4)
        pg.draw.rect(self.screen, self.C['cyd'], (px, py, pw, panel_h), 1, border_radius=4)
        self.screen.blit(self.fcn.render("◆ 己方无人机态势", True, self.C['cyan']), (px + 10, py + 7))
        visible_friendlies = self._visible_friendlies()
        standby_count = sum(1 for it in visible_friendlies if self._friendly_render_state(it) in (IState.STANDBY, IState.LANDED))
        airborne_count = sum(
            1
            for it in visible_friendlies
            if self._friendly_render_state(it) in (IState.LAUNCHING, IState.INTERCEPTING, IState.FOLLOWING, IState.RETURNING)
        )
        reserve_fn = getattr(self.env, "get_reserved_interceptor_count", None)
        reserved_count = reserve_fn() if reserve_fn else 0
        count_surf = self.fms.render(
            f"{len(visible_friendlies)} | 待{standby_count} 空{airborne_count} 保{reserved_count}",
            True,
            self.C['green'],
        )
        self.screen.blit(count_surf, (px + pw - count_surf.get_width() - 8, py + 11))
        pg.draw.line(self.screen, self.C['cyd'], (px + 8, py + header_h), (px + pw - 8, py + header_h), 1)
        cols = 2; card_w = (pw - 20) // 2; card_h = 112
        gap_y = 4
        view_y = py + header_h + 4
        self.p2_view_h = panel_h - header_h - 8
        clip_rect = pg.Rect(px + 4, view_y, pw - 8, self.p2_view_h)
        prev_clip = self.screen.get_clip()
        self.screen.set_clip(clip_rect)
        stm = {IState.STANDBY:("待命",self.C['is']),IState.LAUNCHING:("发射",self.C['amber']),
             IState.INTERCEPTING:("拦截",self.C['red']),IState.FOLLOWING:("随动",self.C['blue']),
             IState.RETURNING:("返航",self.C['grd']),IState.DESTROYED:("损毁",self.C['ed']),
             IState.LANDED:("降落",self.C['is'])}
        for idx, it in enumerate(visible_friendlies):
            row = idx // cols; col = idx % cols
            cx = px + 8 + col * (card_w + 4); cy = view_y + row * (card_h + gap_y) - self.p2_scroll
            if cy + card_h < view_y or cy > view_y + self.p2_view_h:
                continue
            c = self._icol(it)
            pg.draw.rect(self.screen, (12, 16, 24), (cx, cy, card_w, card_h), border_radius=3)
            pg.draw.rect(self.screen, c, (cx, cy, card_w, card_h), 1, border_radius=3)
            self.screen.blit(self.fcn.render(f"I-{it['id']+1:02d}", True, c), (cx+5, cy+4))
            if it.get('jammed_by_interference'):
                phase_fn = getattr(self.env, "_interference_phase", None)
                jam_phase = phase_fn(it) if phase_fn else "lost"
                st, sc = ("失联", self.C['amber']) if jam_phase == "lost" else ("受扰", self.C['orange'])
            else:
                st, sc = stm.get(self._friendly_render_state(it), ("--", self.C['txtd']))
            st_surf = self.f_status.render(st, True, sc)
            self.screen.blit(st_surf, (cx + card_w - st_surf.get_width() - 5, cy+4))
            rx = it.get('reported_x', it['x'])
            ry = it.get('reported_y', it['y'])
            rz = it.get('reported_z', it.get('z', 0.0))
            self.screen.blit(self.fms.render(f"X:{rx:.0f}", True, self.C['txt2']), (cx+5, cy+24))
            self.screen.blit(self.fms.render(f"Y:{ry:.0f}", True, self.C['txt2']), (cx+72, cy+24))
            self.screen.blit(self.fms.render(f"Z:{rz:.0f}", True, self.C['txt2']), (cx+5, cy+42))
            roll = float(it.get('reported_roll', it.get('roll', 0.0)) or 0.0)
            pitch = float(it.get('reported_pitch', it.get('pitch', 0.0)) or 0.0)
            yaw = float(it.get('reported_yaw', it.get('yaw', it.get('heading', 0.0))) or 0.0) % 360.0
            att_text = self._fit_text(f"R:{roll:+.0f} P:{pitch:+.0f} Y:{yaw:03.0f}", self.fms, card_w - 10)
            self.screen.blit(self.fms.render(att_text, True, self.C['txt2']), (cx+5, cy+60))
            mission_text = self._fit_text(f"{it.get('mission_label', '--')} {it.get('target_label', '-')}", self.fms, card_w - 12)
            self.screen.blit(self.fms.render(mission_text, True, c), (cx+5, cy+78))
            fp = it['fuel'] / CFG.INTERCEPTOR_ENDURANCE * 100
            fc = self.C['green'] if fp > 30 else self.C['red']
            bar_w = card_w - 10; bar_h = 4; bar_y = cy + card_h - 8
            pg.draw.rect(self.screen, (30,30,40), (cx+5, bar_y, bar_w, bar_h))
            if fp > 0: pg.draw.rect(self.screen, fc, (cx+5, bar_y, int(bar_w * fp/100), bar_h))
        self.screen.set_clip(prev_clip)
        total_rows = (len(visible_friendlies) + cols - 1) // cols
        self.p2_total_h = total_rows * (card_h + gap_y)
        self.p2_scroll = max(0, min(self.p2_scroll, max(0, self.p2_total_h - self.p2_view_h)))
        if self.p2_total_h > self.p2_view_h:
            bar_x = px + pw - 6
            bar_y = view_y + 4
            bar_h = self.p2_view_h - 8
            pg.draw.rect(self.screen, (20, 26, 34), (bar_x, bar_y, 3, bar_h), border_radius=2)
            knob_h = max(40, int(bar_h * self.p2_view_h / self.p2_total_h))
            max_scroll = max(1, self.p2_total_h - self.p2_view_h)
            knob_y = bar_y + int((bar_h - knob_h) * self.p2_scroll / max_scroll)
            pg.draw.rect(self.screen, self.C['cyan'], (bar_x, knob_y, 3, knob_h), border_radius=2)

    def _draw_p3(self):
        pg = self.pg; px, py, pw, ph = self.p3x, self.oy, self.p3_width, self.lvh
        pg.draw.rect(self.screen, (10, 12, 18), (px, py, pw, ph), border_radius=4)
        pg.draw.rect(self.screen, self.C['cyd'], (px, py, pw, ph), 1, border_radius=4)
        hh = 34
        pg.draw.rect(self.screen, self.C['pnl'], (px, py, pw, hh), border_radius=4)
        pg.draw.line(self.screen, self.C['cyd'], (px, py + hh), (px + pw, py + hh), 1)
        self.screen.blit(self.fcn.render("◆ 系统日志", True, self.C['cyan']), (px + 10, py + 7))
        cr = pg.Rect(px + 4, py + hh + 2, pw - 8, ph - hh - 4)
        prev = self.screen.get_clip(); self.screen.set_clip(cr)
        rendered = self._wrapped_logs(pw - 70)
        line_h = max(24, self.fcs.get_linesize() + 4)
        self.lth = len(rendered) * line_h
        if self.fscr: self.lsy = max(0, self.lth - cr.height); self.fscr = False
        sty = py + hh + 4 - self.lsy
        for idx, (pfx_surf, pfx_w, text_surf) in enumerate(rendered):
            yy = sty + idx * line_h
            if yy + line_h < py + hh or yy > py + ph: continue
            if pfx_surf:
                self.screen.blit(pfx_surf, (px + 5, yy))
            text_x = px + max(45, pfx_w + 8)
            self.screen.blit(text_surf, (text_x, yy))
        self.screen.set_clip(prev)
        if self.lth > cr.height:
            bar_x = px + pw - 6
            bar_y = cr.y + 3
            bar_h = cr.height - 6
            pg.draw.rect(self.screen, (20, 26, 34), (bar_x, bar_y, 3, bar_h), border_radius=2)
            knob_h = max(36, int(bar_h * cr.height / max(self.lth, 1)))
            max_scroll = max(1, self.lth - cr.height)
            knob_y = bar_y + int((bar_h - knob_h) * self.lsy / max_scroll)
            pg.draw.rect(self.screen, self.C['cyan'], (bar_x, knob_y, 3, knob_h), border_radius=2)

    # ============================================================
    # 指令区 - 含语音状态可视化
    # ============================================================
    def _draw_cmd(self):
        pg = self.pg
        cx, cy, cw, ch = self.p3x, self.cmd_y, self.p3_width, self.cmd_h
        pg.draw.rect(self.screen, self.C['pnl'], (cx, cy, cw, ch), border_radius=4)

        # 边框颜色随语音状态变化
        if self.voice_state == 1:
            bc = self.C['red']       # 录音中 → 红色
        elif self.voice_state == 2:
            bc = self.C['amber']     # 识别中 → 黄色
        else:
            bc = self.C['pink'] if self.chat_active else self.C['cyd']

        bw = 2 if (self.chat_active or self.voice_state > 0) else 1
        pg.draw.rect(self.screen, bc, (cx, cy, cw, ch), bw, border_radius=4)

        # 标题栏
        title = "◆ 指令"
        if self.voice_state == 1:
            title += " [MIC 录音中...]"
        elif self.voice_state == 2:
            title += " [AI 识别中...]"
        self.screen.blit(self.fcm.render(title, True, bc), (cx + 12, cy + 6))

        # 输入框
        ix, iy, iw, ih = cx + 12, cy + 40, cw - 24, max(36, ch - 52)
        pg.draw.rect(self.screen, (15, 18, 25), (ix, iy, iw, ih), border_radius=3)
        pg.draw.rect(self.screen, bc, (ix, iy, iw, ih), 1, border_radius=3)

        # 文本内容
        if self.voice_state == 1:
            self.screen.blit(
                self.fcs.render("保持按住 V 键说话...", True, bc), (ix + 6, iy + 6))
        elif self.voice_state == 2:
            self.screen.blit(
                self.fcs.render("正在处理语音...", True, bc), (ix + 6, iy + 6))
        elif self.chat_active:
            disp = self.chat_input[-15:]
            if self.cursor_vis:
                disp += "▎"
            self.screen.blit(
                self.fcn.render(disp, True, self.C['txt']), (ix + 6, iy + 4))
        else:
            hint = "按T指令 | 按住V语音 | P切换预测线"
            self.screen.blit(self.fms.render(hint, True, self.C['txtd']), (ix + 6, iy + 8))

    def _draw_result(self):
        pg=self.pg
        ov=pg.Surface((CFG.SCREEN_WIDTH,CFG.SCREEN_HEIGHT),pg.SRCALPHA)
        ov.fill((0,0,0,200)); self.screen.blit(ov,(0,0))
        bw,bh=560,315; bx,by=(CFG.SCREEN_WIDTH-bw)//2,(CFG.SCREEN_HEIGHT-bh)//2
        pg.draw.rect(self.screen,self.C['pnl'],(bx,by,bw,bh),border_radius=8)
        lost_count = self.env.get_lost_interceptor_count() if hasattr(self.env, "get_lost_interceptor_count") else 0
        if self.env.success and lost_count == 0: ti,c="任务成功",self.C['green']
        elif self.env.success: ti,c="任务受损",self.C['orange']
        else: ti,c="任务失败",self.C['red']
        pg.draw.rect(self.screen,c,(bx,by,bw,bh),3,border_radius=8)
        tt=self.fcl.render(ti,True,c)
        self.screen.blit(tt,(CFG.SCREEN_WIDTH//2-tt.get_width()//2,by+30))
        pg.draw.line(self.screen,c,(bx+30,by+80),(bx+bw-30,by+80),2)

        avg_y = self.env.get_avg_alt()
        safe_dist = CFG.INTERCEPT_FAIL_LINE - avg_y

        effectiveness = self.env.get_effectiveness_rate() if hasattr(self.env, "get_effectiveness_rate") else self.env.get_rate()
        stats=[
            (f"任务效能: {effectiveness*100:.1f}% | 拦截率: {self.env.get_rate()*100:.1f}%",self.C['cyan']),
            (f"击毁: {self.env.stats['kills']} | 突防: {self.env.stats['penetrations']} | 失联: {lost_count}",self.C['amber']),
            (f"我方战损: {self.env.stats['our_losses']}架",self.C['orange']),
            (f"平均拦截安全距: {safe_dist:.0f}m",self.C['txt']),
            (f"用时: {self.env.time:.1f}秒",self.C['txt']),
        ]
        for i,(s,sc) in enumerate(stats):
            st=self.fcn.render(s,True,sc)
            self.screen.blit(st,(CFG.SCREEN_WIDTH//2-st.get_width()//2,by+95+i*38))
        h=self.fcn.render("按 R 重新开始 | 按 ESC 退出",True,self.C['txt2'])
        self.screen.blit(h,(CFG.SCREEN_WIDTH//2-h.get_width()//2,by+bh-35))
