#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"          # Python解释器: 一般不用改; 可改 python3 / 指定虚拟环境里的python
export SDL_RENDER_DRIVER="${SDL_RENDER_DRIVER:-software}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export SDL_VIDEO_X11_FORCE_EGL="${SDL_VIDEO_X11_FORCE_EGL:-0}"

# =========================
# 基础运行参数
# =========================
MODE="${MODE:-demo}"                        # 启动模式: demo=直接进仿真主界面, menu=先进菜单; 一般保持 demo, 改成 menu 会多一步菜单选择
SCENE_KM="${SCENE_KM:-3}"                     # 场景大小(公里): 当前强干扰demo默认3km; 可改 1/2/3/4/5/10
INTERCEPT_MODE="${INTERCEPT_MODE:-hit}"    # 拦截模式: hit=主打随从, hybrid=主打+扯网混合, net=纯扯网, legacy-net=旧版网捕; 改模式会直接改变任务分配和演示观感
UI_STYLE="${UI_STYLE:-rect}"                # 界面样式: rect=方形, arc=扇形, omni=360全向雷达; 只影响显示风格, 不影响规划和数据
FULLSCREEN="${FULLSCREEN:-0}"               # 是否全屏: 0=窗口便于排错, 1=全屏适合演示; 只影响显示, 不影响逻辑
HANGAR_MODE="${HANGAR_MODE:-multi}"        # 机巢模式: multi=多机巢发射更分散, single=单机巢更集中; 会影响起飞节奏和演示观感

#  需要实装/外场联调时：
   #
   #找“设备1”的同事要他们的 Redis IP、经纬度原点，填入第 2 和 第 4 节。并将 SOURCE 改为 fusion。
   #
   # 找设备一接收端同事要 TCP 监听 IP、无人机真实 ID 映射表，填入 core/common.py 的 PLAN_EXPORT。


# =========================
# 强干扰演示配置
# =========================
DEMO_INTERFERENCE_ENABLE="${DEMO_INTERFERENCE_ENABLE:-1}"   # 干扰效果: 1=开启, 无人机进圆区后先受扰再失联悬停; 0=关闭, 只做正常拦截
DEMO_INTERFERENCE_VISIBLE="${DEMO_INTERFERENCE_VISIBLE:-1}" # 干扰可视化: 1=在UI上画圆形干扰范围; 0=隐藏圆区, 但若效果开启仍会实际干扰
DEMO_SCHEME="${DEMO_SCHEME:-0}"                             # UI演示方案: 0=按当前参数启动后手动点按钮; 1=传统最近邻; 2=威胁协同; 3=强干扰失联

# LLM决策链网页
LLM_DASHBOARD="${LLM_DASHBOARD:-1}"                         # 1=同时启动一个网页显示LLM决策链; 0=关闭
LLM_DASHBOARD_HOST="${LLM_DASHBOARD_HOST:-127.0.0.1}"       # 网页监听地址: 本机演示用127.0.0.1; 局域网给老师看可改0.0.0.0
LLM_DASHBOARD_PORT="${LLM_DASHBOARD_PORT:-8765}"            # 网页端口: 默认 http://127.0.0.1:8765/
LLM_DASHBOARD_OPEN="${LLM_DASHBOARD_OPEN:-1}"               # 1=启动后尝试自动打开浏览器; 0=只打印网址

# =========================
# 输入源配置
# =========================
SOURCE="${SOURCE:-demo}"                    # 数据源: demo=本机强干扰回放演示; 要接外部数据时可用 SOURCE=fusion 覆盖
ENEMY_REDIS_FORMAT="${ENEMY_REDIS_FORMAT:-hash}"        # 敌方Redis格式: auto=自动识别, flat=读 101_x/101_y, hash=读 enemy:status:*; 格式选错会导致读不到敌方或位置异常
FRIENDLY_RETURN_SOURCE="${FRIENDLY_RETURN_SOURCE:-none}" # 己方回传来源: none=完全不用外部己方回传、只让本机己方自己跑; udp=若有外部己方回传则吃UDP; redis=己方也从Redis读; 你当前需求建议保持 none

# 外部 Redis（敌方主源）
REDIS_HOST="${REDIS_HOST:-192.168.1.2}"   # 输入Redis地址: 这里填23号机IP; 改错会导致敌方读不到并出现卡顿/读取失败
REDIS_PORT="${REDIS_PORT:-6379}"            # 输入Redis端口: 大多数Redis是 6379; 如果对方改了端口, 这里也必须同步改
REDIS_DB="${REDIS_DB:-0}"                   # 输入Redis库号: 常用 0; 如果对方把数据写在别的db, 这里不改就等于读空库
REDIS_PASSWORD="${REDIS_PASSWORD:-uav123}"        # 输入Redis密码: 无密码留空, 有密码就写; 密码不对会报 Redis数据读取失败

# UAV 回传 UDP（己方主源）
UDP_IN_HOST="${UDP_IN_HOST:-0.0.0.0}"       # 本机监听地址: 0.0.0.0=监听所有网卡, 一般别改; 改成127.0.0.1会只接收本机UDP
UDP_IN_PORT="${UDP_IN_PORT:-6000}"          # 本机监听端口: 只有你真接己方UDP回传时才重要; 端口不对就收不到己方回传

# =========================
# 断续敌方数据兜底时间窗
# =========================
RADAR_STALE_SEC="${RADAR_STALE_SEC:-4.0}"   # stale阈值(秒): 超过该时间没新点, 目标先不删、按最后航迹续算; 调大=更稳但更迟钝, 调小=更灵敏但更容易抖
RADAR_LOST_SEC="${RADAR_LOST_SEC:-12.0}"    # lost阈值(秒): 超过该时间才判定真丢失; 调大=更不容易丢目标, 调小=更快释放错误目标
TARGET_SEARCH_SEC="${TARGET_SEARCH_SEC:-10.0}" # 丢失后搜索时长(秒): 沿最后方向继续搜多久; 调大=更执着追旧目标, 调小=更快放弃重分配

# =========================
# 敌方本地关联器
# =========================
ENEMY_ASSOC="${ENEMY_ASSOC:-off}"                            # 敌方本地关联器: on=开启本地“重新认人”, off=完全信上游编号; 23号机编号不稳时务必保持 on
ENEMY_ASSOC_MAX_DISTANCE="${ENEMY_ASSOC_MAX_DISTANCE:-450.0}" # 关联平面门限(米): 大一点=更容易把换号但位置接近的目标认成同一架, 也更容易误并轨; 小一点=更保守
ENEMY_ASSOC_MAX_ALTITUDE="${ENEMY_ASSOC_MAX_ALTITUDE:-140.0}" # 关联高度门限(米): 大一点=容忍高度波动, 小一点=避免把不同高度层目标合并
ENEMY_ASSOC_KEEP_SEC="${ENEMY_ASSOC_KEEP_SEC:-18.0}"          # 本地敌方轨迹保留时长(秒): 大一点=短时断流后还能沿用旧编号, 小一点=更快丢弃旧轨迹防止串轨

# 敌方 hash 战术显示映射
ENEMY_HASH_REMAP_MODE="${ENEMY_HASH_REMAP_MODE:-direct}"     # 敌方hash映射模式: direct=直接按原点投影, inbound=只显示从远处向原点进攻的那一段
ENEMY_HASH_CENTER_X_RATIO="${ENEMY_HASH_CENTER_X_RATIO:-0.5}" # 敌方hash入侵段横向中心: 0=贴左, 0.5=居中, 1=贴右; 现在建议先居中
ENEMY_HASH_LATERAL_SCALE="${ENEMY_HASH_LATERAL_SCALE:-1.0}"   # 敌方hash横向缩放: 大一点横向摆动更明显, 小一点更像直插来袭
ENEMY_HASH_RANGE_SCALE="${ENEMY_HASH_RANGE_SCALE:-2.0}"       # 敌方hash纵深推进倍率: 当前先用 5.0, 让目标先稳定地从场内下方向上推进; 太贴底可减到 4.0, 太靠上可加到 6.0
ENEMY_HASH_START_RANGE_M="${ENEMY_HASH_START_RANGE_M:-0.0}"   # 敌方hash入侵显示窗口(米): 0=关闭“最后500m窗口”, 恢复之前更自然的整段 inbound 映射; 需要再裁最后一段时再改成 500/800
ENEMY_HASH_Y_OFFSET_M="${ENEMY_HASH_Y_OFFSET_M:--1000.0}"     # 敌方hash纵向平移(米): 当前先整体下移1000米, 把入侵段起点从约1.5km压到UI约500m附近; 还偏高就再减小, 太低就往0收
ENEMY_HASH_HIDE_OUTBOUND="${ENEMY_HASH_HIDE_OUTBOUND:-on}"    # 敌方hash越过原点后是否隐藏: on=只看入侵段, off=通过原点后还继续显示

# 敌方 flat 坐标旋转/翻转映射
ENEMY_FLAT_REMAP_MODE="${ENEMY_FLAT_REMAP_MODE:-direct}"    # 敌方flat上层映射: direct=关闭上层映射, 直接按 x/-z/y 对应; legacy=保留旧经纬度+旋转+平移映射
ENEMY_FLAT_ROTATE_DEG="${ENEMY_FLAT_ROTATE_DEG:-130.0}"      # 敌方flat平面旋转角(度): 当前先收在 130, 让轨迹更接近竖直来袭又不过度转头; 若仍左右偏, 再在 125/135 之间微调
ENEMY_FLAT_FLIP_X="${ENEMY_FLAT_FLIP_X:-off}"                # 敌方flat是否镜像X轴: off=不翻, on=左右翻; 看到目标整体左右颠倒时改 on
ENEMY_FLAT_FLIP_Y="${ENEMY_FLAT_FLIP_Y:-on}"                # 敌方flat是否镜像Y轴: off=不翻, on=上下翻; 看到目标明明来袭却离你越来越远时改 on
ENEMY_FLAT_SCALE="${ENEMY_FLAT_SCALE:-1}"                  # 敌方flat平面缩放倍数: 当前回退到之前较稳定的 1.1; 轨迹太短可加到 1.5/2.0, 太长可退到 0.8
ENEMY_FLAT_CENTER_X_RATIO="${ENEMY_FLAT_CENTER_X_RATIO:-2}" # 敌方flat横向平移比例: 保持 0.7, 让目标先落在场景中间偏右; 太靠左加大, 太靠右减小
ENEMY_FLAT_CENTER_Y_RATIO="${ENEMY_FLAT_CENTER_Y_RATIO:-0.0003}" # 敌方flat纵向平移比例: 0=贴底, 0.2=先抬进场内; 如果目标一直趴底边, 优先把这里加到 0.3/0.4

# =========================
# 固定经纬度原点
# =========================
GEO_ORIGIN_LAT="${GEO_ORIGIN_LAT:-34.158780}" # 固定参考原点纬度: 现在按你给的雷达中心点; 目标经纬度会变, 但这个原点必须固定不动
GEO_ORIGIN_LON="${GEO_ORIGIN_LON:-108.692348}" # 固定参考原点经度: 和上面的纬度成对使用; 统一后 x/y 表示“相对雷达中心偏了多少米”

# =========================
# 对外发布配置
# =========================
PUBLISH_INTERVAL="${PUBLISH_INTERVAL:-0.07}" # 对外发布周期(秒): 越小刷新越快但更占资源; 常用 0.03/0.07/0.1/0.5/1, 录屏演示可适当放大
FRIENDLY_START="${FRIENDLY_START:-1}"       # 己方编号起点: 老师读取本机Redis时默认从 1_x/1_y/1_z 开始
ENEMY_START="${ENEMY_START:-100}"           # 敌方编号起点: 老师读取本机Redis时默认从 100_x/100_y/100_z 开始

# Redis 输出
ENABLE_PUBLISH_REDIS="${ENABLE_PUBLISH_REDIS:-1}"       # 是否开启Redis输出: 默认把敌我都写入本机Redis供老师读取
PUBLISH_REDIS_MODE="${PUBLISH_REDIS_MODE:-teacher-friendly}"     # Redis输出格式: 1_x/1_y/1_z/1_type... 扁平键, type=ally/enemy, y统一50m, z为地图纵向坐标
PUBLISH_REDIS_SIDE="${PUBLISH_REDIS_SIDE:-all}"    # Redis输出对象: all=己方和敌方都写; friendly=只写己方; enemy=只写敌方
PUBLISH_REDIS_HOST="${PUBLISH_REDIS_HOST:-127.0.0.1}"   # 输出Redis地址: 默认写你本机Redis; 如果你要写到别的电脑Redis, 就把这里改成对方IP
PUBLISH_REDIS_PORT="${PUBLISH_REDIS_PORT:-6379}"        # 输出Redis端口: 一般 6379; 对方Redis换端口时这里也要一起改
PUBLISH_REDIS_DB="${PUBLISH_REDIS_DB:-0}"               # 输出Redis库号: 通常 0; 如果你想和别的数据隔离, 可改成 1/2/... 但外部也必须读同一个db
PUBLISH_REDIS_PASSWORD="${PUBLISH_REDIS_PASSWORD:-}"    # 输出Redis密码: 目标Redis有密码就写, 没有就留空; 密码错了会导致你写不进去
#
# 本地Redis一次性清理旧敌方键(手动按钮):
# for i in $(seq 1 140); do
#   redis-cli -h 127.0.0.1 -p 6379 -n 0 DEL ${i}_x ${i}_y ${i}_z ${i}_status ${i}_type ${i}_frame ${i}_timestamp ${i}_battery
# done
# redis-cli -h 127.0.0.1 -p 6379 -n 0 KEYS '100_*'
# 说明: 默认己方从1开始、敌方从100开始; 如改数量, 清理范围也相应放大

# UDP 输出
ENABLE_PUBLISH_UDP="${ENABLE_PUBLISH_UDP:-0}"           # 是否开启UDP输出: 1=额外发UDP给外部, 0=不发; 当前建议保持 0, 先把Redis链路跑稳
PUBLISH_UDP_MODE="${PUBLISH_UDP_MODE:-teacher}"         # UDP输出格式: teacher=外部当前包格式, geo=经纬度格式; 只有开UDP输出时才有意义
UDP_OUT_HOST="${UDP_OUT_HOST:-127.0.0.1}"               # 输出UDP目标IP: 只有开UDP输出时才有意义; 改成接收端电脑IP
UDP_OUT_PORT="${UDP_OUT_PORT:-9999}"                    # 输出UDP目标端口: 只有开UDP输出时才有意义; 改错就发不到对方程序
UDP_ENEMY_ONLY="${UDP_ENEMY_ONLY:-0}"                   # UDP是否只发敌方: 1=只发敌方, 0=敌我都发; 如果外部那边容易把己方误识别成敌方, 可以改成 1

# =========================
# 设备二 -> 设备一 决策帧发布
# =========================
# 主链路: TCP Socket JSON Lines, 一行一个 decision_output_frame JSON; 设备一按最新有效帧执行。
# 设备二/设备一接口配置已统一放到 core/common.py，不再从启动脚本导出 PLAN_* 环境变量。
# 联调时修改 core.common.PLAN_EXPORT：
#   PLAN_EXPORT["enabled"] = True
#   PLAN_EXPORT["socket_host"] = "设备一接收IP"
#   PLAN_EXPORT["uav_id_map"] = "uav_01:1,uav_02:2,uav_03:3,uav_04:4"
#   STATION_BRIDGE["enabled"] = True  # 仅历史旁路调试需要时开启


cd "${SCRIPT_DIR}"

CMD=(
  "${PYTHON_BIN}" -m simulation.main
  --mode "${MODE}"
  --source "${SOURCE}"
  --scene-km "${SCENE_KM}"
  --intercept-mode "${INTERCEPT_MODE}"
  --ui-style "${UI_STYLE}"
  --hangar-mode "${HANGAR_MODE}"
  --redis-host "${REDIS_HOST}"
  --redis-port "${REDIS_PORT}"
  --redis-db "${REDIS_DB}"
  --enemy-redis-format "${ENEMY_REDIS_FORMAT}"
  --friendly-return-source "${FRIENDLY_RETURN_SOURCE}"
  --udp-in-host "${UDP_IN_HOST}"
  --udp-in-port "${UDP_IN_PORT}"
  --geo-origin-lat "${GEO_ORIGIN_LAT}"
  --geo-origin-lon "${GEO_ORIGIN_LON}"
  --radar-stale-sec "${RADAR_STALE_SEC}"
  --radar-lost-sec "${RADAR_LOST_SEC}"
  --target-search-sec "${TARGET_SEARCH_SEC}"
  --enemy-assoc "${ENEMY_ASSOC}"
  --enemy-assoc-max-distance "${ENEMY_ASSOC_MAX_DISTANCE}"
  --enemy-assoc-max-altitude "${ENEMY_ASSOC_MAX_ALTITUDE}"
  --enemy-assoc-keep-sec "${ENEMY_ASSOC_KEEP_SEC}"
  --enemy-hash-remap-mode "${ENEMY_HASH_REMAP_MODE}"
  --enemy-hash-center-x-ratio "${ENEMY_HASH_CENTER_X_RATIO}"
  --enemy-hash-lateral-scale "${ENEMY_HASH_LATERAL_SCALE}"
  --enemy-hash-range-scale "${ENEMY_HASH_RANGE_SCALE}"
  --enemy-hash-start-range-m "${ENEMY_HASH_START_RANGE_M}"
  --enemy-hash-y-offset-m "${ENEMY_HASH_Y_OFFSET_M}"
  --enemy-hash-hide-outbound "${ENEMY_HASH_HIDE_OUTBOUND}"
  --enemy-flat-remap-mode "${ENEMY_FLAT_REMAP_MODE}"
  --enemy-flat-rotate-deg "${ENEMY_FLAT_ROTATE_DEG}"
  --enemy-flat-flip-x "${ENEMY_FLAT_FLIP_X}"
  --enemy-flat-flip-y "${ENEMY_FLAT_FLIP_Y}"
  --enemy-flat-scale "${ENEMY_FLAT_SCALE}"
  --enemy-flat-center-x-ratio "${ENEMY_FLAT_CENTER_X_RATIO}"
  --enemy-flat-center-y-ratio "${ENEMY_FLAT_CENTER_Y_RATIO}"
  --demo-interference-enable "${DEMO_INTERFERENCE_ENABLE}"
  --demo-interference-visible "${DEMO_INTERFERENCE_VISIBLE}"
  --demo-scheme "${DEMO_SCHEME}"
  --llm-dashboard-host "${LLM_DASHBOARD_HOST}"
  --llm-dashboard-port "${LLM_DASHBOARD_PORT}"
  --publish-interval "${PUBLISH_INTERVAL}"
  --friendly-start "${FRIENDLY_START}"
  --enemy-start "${ENEMY_START}"
  --publish-redis-mode "${PUBLISH_REDIS_MODE}"
  --publish-redis-side "${PUBLISH_REDIS_SIDE}"
  --publish-redis-host "${PUBLISH_REDIS_HOST}"
  --publish-redis-port "${PUBLISH_REDIS_PORT}"
  --publish-redis-db "${PUBLISH_REDIS_DB}"
  --publish-udp-mode "${PUBLISH_UDP_MODE}"
  --udp-out-host "${UDP_OUT_HOST}"
  --udp-out-port "${UDP_OUT_PORT}"
)

if [[ -n "${REDIS_PASSWORD}" ]]; then
  CMD+=(--redis-password "${REDIS_PASSWORD}")
fi

if [[ -n "${PUBLISH_REDIS_PASSWORD}" ]]; then
  CMD+=(--publish-redis-password "${PUBLISH_REDIS_PASSWORD}")
fi

if [[ "${FULLSCREEN}" == "1" ]]; then
  CMD+=(--fullscreen)
fi

if [[ "${ENABLE_PUBLISH_REDIS}" == "1" ]]; then
  CMD+=(--publish-redis)
fi

if [[ "${ENABLE_PUBLISH_UDP}" == "1" ]]; then
  CMD+=(--publish-udp)
fi

if [[ "${LLM_DASHBOARD}" == "1" ]]; then
  CMD+=(--llm-dashboard)
fi

if [[ "${LLM_DASHBOARD_OPEN}" == "1" ]]; then
  CMD+=(--llm-dashboard-open)
fi

if [[ "${UDP_ENEMY_ONLY}" == "1" ]]; then
  CMD+=(--udp-enemy-only)
fi

CMD+=("$@")

printf 'Running command:\n'
printf ' SDL_RENDER_DRIVER=%q LIBGL_ALWAYS_SOFTWARE=%q SDL_VIDEO_X11_FORCE_EGL=%q\n' \
  "${SDL_RENDER_DRIVER}" "${LIBGL_ALWAYS_SOFTWARE}" "${SDL_VIDEO_X11_FORCE_EGL}"
printf ' %q' "${CMD[@]}"
printf '\n'

exec "${CMD[@]}"
