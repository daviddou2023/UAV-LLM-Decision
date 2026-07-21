"""
MARL 任务选择终端 - iPad风格界面
"""

import pygame
import math
import time
import sys
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List


class MenuState(Enum):
    MAIN_MENU = 0  # 主菜单
    MISSION_DETAIL = 1  # 任务详情
    LOADING = 2  # 加载中
    IN_GAME = 3  # 游戏中


class MissionType(Enum):
    DECAPITATION = 0  # 斩首行动
    VIP_PROTECT = 1  # 保护VIP
    COOP_STRIKE = 2  # 协同搜打


@dataclass
class Mission:
    type: MissionType
    name: str
    subtitle: str
    description: str
    icon: str
    available: bool
    color: Tuple[int, int, int]


class iPadTerminal:
    """iPad风格战术终端界面"""

    def __init__(self):
        pygame.init()

        # 屏幕设置
        self.width = 1400
        self.height = 850
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("战术指挥终端 | TACTICAL COMMAND TERMINAL")

        # 字体加载
        self.fonts = self._load_fonts()

        # 颜色方案 - 深色科技风
        self.colors = {
            'bg_dark': (8, 12, 18),
            'bg_panel': (15, 22, 32),
            'bg_card': (20, 28, 40),
            'bg_card_hover': (28, 38, 55),
            'bg_card_selected': (25, 45, 70),
            'border': (40, 60, 80),
            'border_highlight': (0, 200, 255),
            'cyan': (0, 255, 255),
            'cyan_dim': (0, 150, 180),
            'green': (0, 255, 128),
            'amber': (255, 191, 0),
            'red': (255, 60, 60),
            'purple': (148, 100, 255),
            'text_primary': (220, 230, 240),
            'text_secondary': (120, 140, 160),
            'text_dim': (70, 85, 100),
            'unavailable': (60, 65, 75),
        }

        # 状态
        self.state = MenuState.MAIN_MENU
        self.selected_mission: Optional[MissionType] = None
        self.hover_mission: Optional[MissionType] = None
        self.frame_count = 0
        self.loading_progress = 0
        self.loading_start_time = 0

        # 动画参数
        self.pulse_phase = 0
        self.card_animations = {m: 0 for m in MissionType}

        # 任务定义
        self.missions = [
            Mission(
                type=MissionType.DECAPITATION,
                name="斩首行动",
                subtitle="DECAPITATION STRIKE",
                description="精确打击高价值目标，快速突入敌方指挥中心，消灭敌方首脑人物。",
                icon="⚔",
                available=False,
                color=(255, 60, 60)
            ),
            Mission(
                type=MissionType.VIP_PROTECT,
                name="VIP护送",
                subtitle="VIP ESCORT MISSION",
                description="保护重要人员安全撤离危险区域，确保护送路线安全。",
                icon="🛡",
                available=False,
                color=(255, 191, 0)
            ),
            Mission(
                type=MissionType.COOP_STRIKE,
                name="协同搜打",
                subtitle="COOPERATIVE SEARCH & STRIKE",
                description="多无人机协同作战，搜索并消灭区域内所有敌方目标。",
                icon="🎯",
                available=True,
                color=(0, 255, 255)
            ),
        ]

        # 系统日志
        self.logs = [
            ("[SYS]", "战术终端已启动", "cyan"),
            ("[NET]", "卫星链路已建立", "green"),
            ("[UAV]", "无人机编队待命中", "text_secondary"),
        ]

        # 按钮区域
        self.execute_btn_rect = pygame.Rect(0, 0, 0, 0)
        self.back_btn_rect = pygame.Rect(0, 0, 0, 0)

        self.clock = pygame.time.Clock()

    def _load_fonts(self):
        """加载字体"""
        fonts = {}
        cn_paths = [
            "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/simhei.ttf",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/System/Library/Fonts/PingFang.ttc"
        ]

        for size_name, size in [('huge', 48), ('large', 32), ('medium', 24),
                                ('normal', 18), ('small', 14), ('tiny', 12)]:
            font = None
            for path in cn_paths:
                try:
                    font = pygame.font.Font(path, size)
                    test = font.render("测试", True, (255, 255, 255))
                    if test.get_width() > 10:
                        break
                except:
                    continue
            fonts[size_name] = font or pygame.font.Font(None, size)

        # 等宽字体
        for name in ['consolas', 'monaco', 'courier']:
            try:
                fonts['mono'] = pygame.font.SysFont(name, 14)
                fonts['mono_large'] = pygame.font.SysFont(name, 18)
                break
            except:
                continue
        if 'mono' not in fonts:
            fonts['mono'] = pygame.font.Font(None, 14)
            fonts['mono_large'] = pygame.font.Font(None, 18)

        return fonts

    def add_log(self, prefix: str, message: str, color: str = "text_secondary"):
        """添加日志"""
        self.logs.append((prefix, message, color))
        if len(self.logs) > 8:
            self.logs.pop(0)

    def run(self) -> Optional[MissionType]:
        """运行菜单，返回选择的任务类型"""
        running = True

        while running:
            self.frame_count += 1
            self.pulse_phase = (self.pulse_phase + 0.03) % (2 * math.pi)

            # 事件处理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type == pygame.MOUSEMOTION:
                    self._handle_mouse_motion(event.pos)

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    result = self._handle_click(event.pos)
                    if result == "START_GAME":
                        return MissionType.COOP_STRIKE
                    elif result == "QUIT":
                        running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        if self.state == MenuState.MISSION_DETAIL:
                            self.state = MenuState.MAIN_MENU
                            self.selected_mission = None
                        else:
                            running = False

            # 加载状态处理
            if self.state == MenuState.LOADING:
                elapsed = time.time() - self.loading_start_time
                self.loading_progress = min(elapsed / 2.0, 1.0)  # 2秒加载
                if self.loading_progress >= 1.0:
                    return MissionType.COOP_STRIKE

            # 渲染
            self._render()
            pygame.display.flip()
            self.clock.tick(60)

        return None

    def _handle_mouse_motion(self, pos):
        """处理鼠标移动"""
        self.hover_mission = None

        if self.state == MenuState.MAIN_MENU:
            # 检测悬停在哪个任务卡片上
            card_width = 380
            card_height = 480
            start_x = (self.width - (card_width * 3 + 60)) // 2

            for i, mission in enumerate(self.missions):
                x = start_x + i * (card_width + 30)
                y = 180
                rect = pygame.Rect(x, y, card_width, card_height)
                if rect.collidepoint(pos):
                    self.hover_mission = mission.type
                    break

    def _handle_click(self, pos) -> Optional[str]:
        """处理点击"""
        if self.state == MenuState.MAIN_MENU:
            # 点击任务卡片
            card_width = 380
            card_height = 480
            start_x = (self.width - (card_width * 3 + 60)) // 2

            for i, mission in enumerate(self.missions):
                x = start_x + i * (card_width + 30)
                y = 180
                rect = pygame.Rect(x, y, card_width, card_height)
                if rect.collidepoint(pos):
                    self.selected_mission = mission.type
                    self.state = MenuState.MISSION_DETAIL
                    self.add_log("[SYS]", f"已选择: {mission.name}", "cyan")
                    return None

        elif self.state == MenuState.MISSION_DETAIL:
            # 返回按钮
            if self.back_btn_rect.collidepoint(pos):
                self.state = MenuState.MAIN_MENU
                self.selected_mission = None
                return None

            # 执行按钮
            if self.execute_btn_rect.collidepoint(pos):
                mission = self._get_mission(self.selected_mission)
                if mission and mission.available:
                    self.add_log("[CMD]", "任务指令已下达", "green")
                    self.add_log("[UAV]", "无人机编队已接收指令", "cyan")
                    self.add_log("[SYS]", "正在初始化任务...", "amber")
                    self.state = MenuState.LOADING
                    self.loading_start_time = time.time()
                    self.loading_progress = 0
                else:
                    self.add_log("[ERR]", "该任务模块正在开发中", "red")
                return None

        return None

    def _get_mission(self, mission_type: MissionType) -> Optional[Mission]:
        """获取任务信息"""
        for m in self.missions:
            if m.type == mission_type:
                return m
        return None

    def _render(self):
        """渲染界面"""
        # 背景
        self.screen.fill(self.colors['bg_dark'])
        self._render_background_grid()

        # 顶部标题栏
        self._render_header()

        # 根据状态渲染
        if self.state == MenuState.MAIN_MENU:
            self._render_main_menu()
        elif self.state == MenuState.MISSION_DETAIL:
            self._render_mission_detail()
        elif self.state == MenuState.LOADING:
            self._render_loading()

        # 底部状态栏
        self._render_footer()

    def _render_background_grid(self):
        """渲染背景网格"""
        pulse = int(15 + 5 * math.sin(self.pulse_phase))
        color = (pulse, pulse + 5, pulse + 10)

        # 垂直线
        for x in range(0, self.width, 80):
            pygame.draw.line(self.screen, color, (x, 0), (x, self.height), 1)
        # 水平线
        for y in range(0, self.height, 80):
            pygame.draw.line(self.screen, color, (0, y), (self.width, y), 1)

    def _render_header(self):
        """渲染顶部标题"""
        # 背景条
        pygame.draw.rect(self.screen, self.colors['bg_panel'], (0, 0, self.width, 70))
        pygame.draw.line(self.screen, self.colors['border_highlight'], (0, 70), (self.width, 70), 2)

        # 左侧标题
        title = self.fonts['large'].render("战术指挥终端", True, self.colors['cyan'])
        self.screen.blit(title, (40, 18))

        subtitle = self.fonts['mono_large'].render("TACTICAL COMMAND TERMINAL v4.0", True,
                                                   self.colors['text_secondary'])
        self.screen.blit(subtitle, (40, 48))

        # 右侧时间
        current_time = time.strftime("%H:%M:%S")
        current_date = time.strftime("%Y-%m-%d")

        time_text = self.fonts['medium'].render(current_time, True, self.colors['text_primary'])
        date_text = self.fonts['small'].render(current_date, True, self.colors['text_secondary'])

        self.screen.blit(time_text, (self.width - 120, 18))
        self.screen.blit(date_text, (self.width - 120, 45))

        # 状态指示灯
        for i, (label, color) in enumerate([("SYS", "green"), ("NET", "green"), ("UAV", "cyan")]):
            x = self.width - 280 - i * 80
            pygame.draw.circle(self.screen, self.colors[color], (x, 35), 6)
            lbl = self.fonts['tiny'].render(label, True, self.colors['text_secondary'])
            self.screen.blit(lbl, (x + 12, 28))

    def _render_main_menu(self):
        """渲染主菜单"""
        # 副标题
        subtitle = self.fonts['medium'].render("选择任务模式 | SELECT MISSION", True, self.colors['text_secondary'])
        subtitle_x = (self.width - subtitle.get_width()) // 2
        self.screen.blit(subtitle, (subtitle_x, 110))

        # 任务卡片
        card_width = 380
        card_height = 480
        start_x = (self.width - (card_width * 3 + 60)) // 2

        for i, mission in enumerate(self.missions):
            x = start_x + i * (card_width + 30)
            y = 180
            self._render_mission_card(mission, x, y, card_width, card_height)

    def _render_mission_card(self, mission: Mission, x: int, y: int, w: int, h: int):
        """渲染任务卡片"""
        is_hover = self.hover_mission == mission.type
        is_selected = self.selected_mission == mission.type

        # 背景色
        if is_selected:
            bg_color = self.colors['bg_card_selected']
        elif is_hover:
            bg_color = self.colors['bg_card_hover']
        else:
            bg_color = self.colors['bg_card']

        # 边框色
        if mission.available:
            border_color = mission.color if (is_hover or is_selected) else self.colors['border']
        else:
            border_color = self.colors['unavailable']

        # 绘制卡片
        pygame.draw.rect(self.screen, bg_color, (x, y, w, h), border_radius=12)
        pygame.draw.rect(self.screen, border_color, (x, y, w, h), 2, border_radius=12)

        # 顶部装饰线
        if mission.available:
            pygame.draw.rect(self.screen, mission.color, (x, y, w, 4), border_radius=2)

        # 图标区域
        icon_y = y + 40
        icon_bg_rect = pygame.Rect(x + w // 2 - 50, icon_y, 100, 100)
        pygame.draw.rect(self.screen, self.colors['bg_dark'], icon_bg_rect, border_radius=50)

        if mission.available:
            pygame.draw.rect(self.screen, mission.color, icon_bg_rect, 2, border_radius=50)
            # 脉冲效果
            if is_hover:
                pulse_size = int(52 + 3 * math.sin(self.frame_count * 0.1))
                pygame.draw.circle(self.screen, (*mission.color[:3], 100),
                                   (x + w // 2, icon_y + 50), pulse_size, 2)
        else:
            pygame.draw.rect(self.screen, self.colors['unavailable'], icon_bg_rect, 2, border_radius=50)

        # 图标文字（使用大字体模拟图标）
        icon_color = mission.color if mission.available else self.colors['unavailable']
        icon_text = self.fonts['huge'].render(mission.icon, True, icon_color)
        icon_x = x + w // 2 - icon_text.get_width() // 2
        self.screen.blit(icon_text, (icon_x, icon_y + 25))

        # 任务名称
        name_color = self.colors['text_primary'] if mission.available else self.colors['unavailable']
        name_text = self.fonts['large'].render(mission.name, True, name_color)
        name_x = x + (w - name_text.get_width()) // 2
        self.screen.blit(name_text, (name_x, y + 160))

        # 英文副标题
        sub_color = mission.color if mission.available else self.colors['unavailable']
        sub_text = self.fonts['small'].render(mission.subtitle, True, sub_color)
        sub_x = x + (w - sub_text.get_width()) // 2
        self.screen.blit(sub_text, (sub_x, y + 200))

        # 分隔线
        pygame.draw.line(self.screen, self.colors['border'],
                         (x + 30, y + 240), (x + w - 30, y + 240), 1)

        # 描述文字（自动换行）
        desc_color = self.colors['text_secondary'] if mission.available else self.colors['text_dim']
        self._render_wrapped_text(mission.description, x + 30, y + 260, w - 60,
                                  self.fonts['normal'], desc_color, line_height=28)

        # 底部状态
        status_y = y + h - 60
        if mission.available:
            status_text = "● 可执行"
            status_color = self.colors['green']
        else:
            status_text = "○ 开发中"
            status_color = self.colors['text_dim']

        status = self.fonts['normal'].render(status_text, True, status_color)
        status_x = x + (w - status.get_width()) // 2
        self.screen.blit(status, (status_x, status_y))

        # 点击提示
        if mission.available and is_hover:
            hint = self.fonts['small'].render("点击查看详情", True, mission.color)
            hint_x = x + (w - hint.get_width()) // 2
            self.screen.blit(hint, (hint_x, status_y + 25))

    def _render_mission_detail(self):
        """渲染任务详情页"""
        mission = self._get_mission(self.selected_mission)
        if not mission:
            return

        # 左侧 - 任务信息面板
        panel_x, panel_y = 50, 100
        panel_w, panel_h = 800, 650

        pygame.draw.rect(self.screen, self.colors['bg_panel'],
                         (panel_x, panel_y, panel_w, panel_h), border_radius=12)
        pygame.draw.rect(self.screen, mission.color if mission.available else self.colors['border'],
                         (panel_x, panel_y, panel_w, panel_h), 2, border_radius=12)

        # 任务标题
        title = self.fonts['huge'].render(mission.name, True,
                                          mission.color if mission.available else self.colors['unavailable'])
        self.screen.blit(title, (panel_x + 40, panel_y + 30))

        subtitle = self.fonts['mono_large'].render(mission.subtitle, True, self.colors['text_secondary'])
        self.screen.blit(subtitle, (panel_x + 40, panel_y + 85))

        # 分隔线
        pygame.draw.line(self.screen, self.colors['border'],
                         (panel_x + 40, panel_y + 120), (panel_x + panel_w - 40, panel_y + 120), 1)

        if mission.available:
            # 任务简报
            self._render_section_title("任务简报", panel_x + 40, panel_y + 140, mission.color)
            self._render_wrapped_text(
                "多架无人机协同执行区域搜索与打击任务。通过智能分配探索区域、"
                "协同追踪目标、多机包围攻击等战术，在规定时间内完成区域覆盖并消灭所有目标。",
                panel_x + 40, panel_y + 180, panel_w - 80, self.fonts['normal'],
                self.colors['text_secondary'], line_height=30
            )

            # 任务参数
            self._render_section_title("任务参数", panel_x + 40, panel_y + 290, mission.color)
            params = [
                ("无人机数量", "3 架"),
                ("目标数量", "4 个"),
                ("任务时限", "300 秒"),
                ("覆盖目标", "≥75%"),
                ("默认速度", "1x"),
            ]
            for i, (label, value) in enumerate(params):
                y_pos = panel_y + 330 + i * 35
                self.screen.blit(self.fonts['normal'].render(f"◆ {label}:", True,
                                                             self.colors['text_secondary']), (panel_x + 50, y_pos))
                self.screen.blit(self.fonts['normal'].render(value, True,
                                                             self.colors['cyan']), (panel_x + 200, y_pos))

            # 操作说明
            self._render_section_title("操作说明", panel_x + 40, panel_y + 520, mission.color)
            controls = ["Enter-开始  Space-暂停  1/2/4-速度", "T-指令输入  F1~F3-快捷指令  R-重置"]
            for i, ctrl in enumerate(controls):
                self.screen.blit(self.fonts['small'].render(ctrl, True, self.colors['text_dim']),
                                 (panel_x + 50, panel_y + 560 + i * 25))
        else:
            # 开发中提示
            dev_y = panel_y + panel_h // 2 - 60

            # 大图标
            icon = self.fonts['huge'].render("🚧", True, self.colors['amber'])
            icon_x = panel_x + (panel_w - icon.get_width()) // 2
            self.screen.blit(icon, (icon_x, dev_y - 60))

            dev_text = self.fonts['large'].render("功能正在完善中", True, self.colors['amber'])
            dev_x = panel_x + (panel_w - dev_text.get_width()) // 2
            self.screen.blit(dev_text, (dev_x, dev_y + 20))

            dev_sub = self.fonts['normal'].render("UNDER DEVELOPMENT", True, self.colors['text_dim'])
            dev_sub_x = panel_x + (panel_w - dev_sub.get_width()) // 2
            self.screen.blit(dev_sub, (dev_sub_x, dev_y + 60))

            dev_hint = self.fonts['small'].render("敬请期待后续更新", True, self.colors['text_secondary'])
            dev_hint_x = panel_x + (panel_w - dev_hint.get_width()) // 2
            self.screen.blit(dev_hint, (dev_hint_x, dev_y + 100))

        # 右侧 - 日志和操作面板
        right_x = 880
        right_w = 480

        # 系统日志
        log_y = 100
        log_h = 300
        pygame.draw.rect(self.screen, self.colors['bg_panel'],
                         (right_x, log_y, right_w, log_h), border_radius=8)
        pygame.draw.rect(self.screen, self.colors['cyan_dim'],
                         (right_x, log_y, right_w, log_h), 1, border_radius=8)

        # 日志标题
        pygame.draw.rect(self.screen, self.colors['bg_card'],
                         (right_x, log_y, right_w, 35), border_radius=8)
        log_title = self.fonts['normal'].render("◆ 系统日志 | SYSTEM LOG", True, self.colors['cyan'])
        self.screen.blit(log_title, (right_x + 15, log_y + 8))

        # 日志内容
        for i, (prefix, msg, color_key) in enumerate(self.logs[-7:]):
            ly = log_y + 50 + i * 32
            color = self.colors.get(color_key, self.colors['text_secondary'])
            self.screen.blit(self.fonts['mono'].render(prefix, True, color), (right_x + 15, ly))
            self.screen.blit(self.fonts['small'].render(msg, True, self.colors['text_secondary']),
                             (right_x + 70, ly))

        # 操作按钮区域
        btn_y = 430
        btn_h = 320
        pygame.draw.rect(self.screen, self.colors['bg_panel'],
                         (right_x, btn_y, right_w, btn_h), border_radius=8)
        pygame.draw.rect(self.screen, self.colors['border'],
                         (right_x, btn_y, right_w, btn_h), 1, border_radius=8)

        # 返回按钮
        self.back_btn_rect = pygame.Rect(right_x + 20, btn_y + 20, right_w - 40, 50)
        back_hover = self.back_btn_rect.collidepoint(pygame.mouse.get_pos())
        back_color = self.colors['bg_card_hover'] if back_hover else self.colors['bg_card']
        pygame.draw.rect(self.screen, back_color, self.back_btn_rect, border_radius=8)
        pygame.draw.rect(self.screen, self.colors['text_secondary'], self.back_btn_rect, 1, border_radius=8)
        back_text = self.fonts['medium'].render("← 返回任务列表", True, self.colors['text_secondary'])
        back_text_x = self.back_btn_rect.centerx - back_text.get_width() // 2
        self.screen.blit(back_text, (back_text_x, self.back_btn_rect.centery - 12))

        # 执行按钮
        self.execute_btn_rect = pygame.Rect(right_x + 20, btn_y + 180, right_w - 40, 80)
        exec_hover = self.execute_btn_rect.collidepoint(pygame.mouse.get_pos())

        if mission.available:
            # 可执行 - 高亮按钮
            if exec_hover:
                exec_bg = (*mission.color[:3],)
                exec_border = mission.color
                text_color = self.colors['bg_dark']
                # 发光效果
                glow_rect = self.execute_btn_rect.inflate(10, 10)
                pygame.draw.rect(self.screen, (*mission.color[:3], 50), glow_rect, border_radius=12)
            else:
                exec_bg = self.colors['bg_card']
                exec_border = mission.color
                text_color = mission.color

            pygame.draw.rect(self.screen, exec_bg, self.execute_btn_rect, border_radius=10)
            pygame.draw.rect(self.screen, exec_border, self.execute_btn_rect, 2, border_radius=10)

            exec_text = self.fonts['large'].render("一键执行任务", True, text_color)
            exec_sub = self.fonts['small'].render("EXECUTE MISSION", True,
                                                  self.colors['bg_dark'] if exec_hover else self.colors['text_dim'])
        else:
            # 不可执行 - 灰色按钮
            pygame.draw.rect(self.screen, self.colors['bg_card'], self.execute_btn_rect, border_radius=10)
            pygame.draw.rect(self.screen, self.colors['unavailable'], self.execute_btn_rect, 2, border_radius=10)

            exec_text = self.fonts['large'].render("功能开发中", True, self.colors['unavailable'])
            exec_sub = self.fonts['small'].render("COMING SOON", True, self.colors['text_dim'])

        exec_text_x = self.execute_btn_rect.centerx - exec_text.get_width() // 2
        exec_sub_x = self.execute_btn_rect.centerx - exec_sub.get_width() // 2
        self.screen.blit(exec_text, (exec_text_x, self.execute_btn_rect.y + 18))
        self.screen.blit(exec_sub, (exec_sub_x, self.execute_btn_rect.y + 52))

        # 提示文字
        if mission.available:
            hint = self.fonts['small'].render("点击上方按钮开始任务", True, self.colors['green'])
        else:
            hint = self.fonts['small'].render("该模块尚未开放", True, self.colors['text_dim'])
        hint_x = right_x + (right_w - hint.get_width()) // 2
        self.screen.blit(hint, (hint_x, btn_y + 280))

    def _render_loading(self):
        """渲染加载界面"""
        # 半透明遮罩
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 200))
        self.screen.blit(overlay, (0, 0))

        # 加载框
        box_w, box_h = 500, 250
        box_x = (self.width - box_w) // 2
        box_y = (self.height - box_h) // 2

        pygame.draw.rect(self.screen, self.colors['bg_panel'],
                         (box_x, box_y, box_w, box_h), border_radius=12)
        pygame.draw.rect(self.screen, self.colors['cyan'],
                         (box_x, box_y, box_w, box_h), 2, border_radius=12)

        # 标题
        title = self.fonts['large'].render("正在初始化任务...", True, self.colors['cyan'])
        title_x = box_x + (box_w - title.get_width()) // 2
        self.screen.blit(title, (title_x, box_y + 40))

        # 进度条
        bar_x = box_x + 50
        bar_y = box_y + 110
        bar_w = box_w - 100
        bar_h = 20

        pygame.draw.rect(self.screen, self.colors['bg_dark'],
                         (bar_x, bar_y, bar_w, bar_h), border_radius=4)

        fill_w = int(bar_w * self.loading_progress)
        if fill_w > 0:
            pygame.draw.rect(self.screen, self.colors['cyan'],
                             (bar_x, bar_y, fill_w, bar_h), border_radius=4)

        pygame.draw.rect(self.screen, self.colors['cyan'],
                         (bar_x, bar_y, bar_w, bar_h), 1, border_radius=4)

        # 百分比
        percent = self.fonts['medium'].render(f"{int(self.loading_progress * 100)}%", True, self.colors['text_primary'])
        percent_x = box_x + (box_w - percent.get_width()) // 2
        self.screen.blit(percent, (percent_x, bar_y + 35))

        # 加载提示
        tips = ["加载地图数据...", "初始化无人机编队...", "建立通信链路...", "任务就绪"]
        tip_idx = min(int(self.loading_progress * len(tips)), len(tips) - 1)
        tip = self.fonts['small'].render(tips[tip_idx], True, self.colors['text_secondary'])
        tip_x = box_x + (box_w - tip.get_width()) // 2
        self.screen.blit(tip, (tip_x, box_y + 180))

    def _render_footer(self):
        """渲染底部状态栏"""
        footer_y = self.height - 40
        pygame.draw.rect(self.screen, self.colors['bg_panel'], (0, footer_y, self.width, 40))
        pygame.draw.line(self.screen, self.colors['border'], (0, footer_y), (self.width, footer_y), 1)

        # 左侧提示
        hint = self.fonts['small'].render("ESC-退出  |  使用鼠标选择任务", True, self.colors['text_dim'])
        self.screen.blit(hint, (40, footer_y + 12))

        # 右侧版本
        version = self.fonts['small'].render("TACTICAL TERMINAL v4.0 | MARL SYSTEM", True, self.colors['text_dim'])
        self.screen.blit(version, (self.width - version.get_width() - 40, footer_y + 12))

    def _render_section_title(self, text: str, x: int, y: int, color):
        """渲染章节标题"""
        pygame.draw.rect(self.screen, color, (x, y + 8, 4, 20))
        title = self.fonts['medium'].render(text, True, color)
        self.screen.blit(title, (x + 15, y + 5))

    def _render_wrapped_text(self, text: str, x: int, y: int, max_width: int,
                             font, color, line_height: int = 25):
        """渲染自动换行文本"""
        words = list(text)
        lines = []
        current_line = ""

        for char in words:
            test_line = current_line + char
            if font.render(test_line, True, color).get_width() <= max_width:
                current_line = test_line
            else:
                if current_line:
                    lines.append(current_line)
                current_line = char

        if current_line:
            lines.append(current_line)

        for i, line in enumerate(lines):
            text_surface = font.render(line, True, color)
            self.screen.blit(text_surface, (x, y + i * line_height))


def run_with_menu():
    terminal = iPadTerminal()
    selected = terminal.run()
    if selected == MissionType.COOP_STRIKE:
        pygame.quit()
        from simulation.main import run_demo
        run_demo()
    else:
        pygame.quit()


if __name__ == "__main__":
    run_with_menu()
