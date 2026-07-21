#### 新增文件

config/jammer_spawn.sdf          # Gazebo 干扰源视觉模型（加入一个橙色和一个红色干扰源）
config/jammer_config.yaml        # 配置干扰参数配置（对应jammer_spawn.sdf）
adapters/link_jammer.py          # 通信干扰核心逻辑（根据无人机位置和干扰源参数，计算实时信号强度、丢包率、延迟，并提供统一查询接口。）


#### 修改现有文件

adapters/adapter_manager.py      # 新增 _link_jammer 全局变量及 get_link_jammer() 函数，供外部获取干扰模块。

                                  新增 _init_link_jammer() —— 加载配置并启动，将适配器位置回调传入。

                                  新增 _shutdown_link_jammer() —— 安全关闭干扰线程。

                                  修改 init_adapter()：在适配器连接成功后调用 _init_link_jammer()。

                                  修改 switch_adapter()：切换前先调用 _shutdown_link_jammer()。
-----------------------------------------------------------------------------------------------------------------------------

config/sim_config.yaml

                                  # 新增 simulation.world 节点：将 static_models 列表统一管理，包含原有的camera_spawn. sdf 和新的 jammer_spawn.sdf。

                                  新增 jammer 配置段（第 93-103 行）：提供全局开关 enabled，以及 config_file 和 spawn_file 路径声明，方便所有模块统一读取。

                                  新增 ROS2 话题 link_status（第 120 行）：为通信链路状态流预留内部话题。

-----------------------------------------------------------------------------------------------------------------------------


scripts/start_sim.sh            # 1. 新增 GZ_SIM_RESOURCE_PATH
                                      export GZ_SIM_RESOURCE_PATH="...:${PROJECT_DIR}/config"
                                      将 config 目录添加到 Gazebo 资源搜索路径，确保能找到 jammer_spawn.sdf。

                                  2. 干扰源配置读取（第 64-93 行）
                                  从 sim_config.yaml 读取干扰配置：

                                  JAMMER_ENABLED：是否启用干扰仿真

                                  JAMMER_SPAWN_FILE：干扰源 SDF 文件路径

                                  增加容错处理，Python/yaml 不可用时使用默认值

                                  3. spawn_static_models 函数（第 102-142 行）
                                  通用的静态模型加载函数：

                                  检查 Gazebo 是否运行

                                  解析 SDF 中的模型名称

                                  逐个加载模型到仿真场景

                                  4. 加载顺序调整
                                  原来的 [3/3] PX4 SITL 现在变为：

                                  [*] 加载相机模型

                                  [*] 加载干扰源模型（如果启用）

                                  [3/3] PX4 SITL

                                  这样可以确保干扰源在无人机起飞前就存在于场景中。


perception/passive_perception.py 

#### 加载通信干扰源逻辑

1. 操作员启动: ./scripts/start_sim.sh
    
    ↓
    
2. start_sim.sh 读取 sim_config.yaml
    ├─ 解析 jammer.enabled = true
    ├─ 解析 jammer.spawn_file = "config/jammer_spawn.sdf"
    └─ 解析 jammer.config_file = "config/jammer_config.yaml"
    
    ↓ (如果 enabled=true)
    
3. start_sim.sh 加载 jammer_spawn.sdf 到 Gazebo
    ├─ 读取 XML，提取模型名称 (jammer_1, jammer_2)
    ├─ 调用 gz model --spawn-file 命令
    └─ Gazebo 场景中出现红色/橙色半透明球体
    
    ↓ (后续)
    
4. Python 服务启动 (server.py)
    ↓
5. adapter_manager.py 加载 link_jammer.py中的LinkJammer类
    ↓
6. LinkJammer 读取 jammer_config.yaml (干扰参数)
    ├─ 获取干扰源位置、半径、功率
    ├─ 计算实时信号质量
    └─ 提供链路状态给感知/决策层







