# 厕所行人系统 HuNav 迁移计划

日期：2026-07-26

## 1. 目标

构建一套自然、稳定、可复现、可评估的厕所行人 benchmark，解决当前链路中的主要问题：

- 行人静态 voxel guard 容易在门口、隔板和小便池附近误阻挡。
- director、voxel planner、Isaac NavigationManager 和 AnimGraph 同时影响运动。
- 多行人局部避让不真实，容易重叠、互锁、闪现或恢复失败。
- staging waypoint 的离散转向影响行走自然度。
- episode 成功状态不能充分反映行人过程是否有效。

目标架构：

```text
Scenario / Episode
        |
Toilet Semantic Director
        |
Behavior Task / Behavior Tree
        |
2D Global Routing (Theta*)
        |
HuNavSim Local Social Motion
        |
Isaac State Mirror + AnimGraph
        |
Recorder / Evaluator
```

## 2. 设计原则

1. 每层只有一个运动所有者。
2. director 只管理厕所语义、资源、事件和任务阶段。
3. 全局规划只处理静态地图上的几何可行性。
4. HuNavSim 只处理局部社会运动和行为树动作。
5. Isaac People 只负责角色、动画和传感器可见性，不再自行选路。
6. hard safety 只阻止非法几何状态，不能替代局部社会规划。
7. 所有随机行为必须受 seed 控制，并保存完整运行快照。
8. 每阶段必须有独立验收场景和清晰回滚点。

## 3. 保留与替换范围

### 保留

- `toilet_semantics.yaml` 中的入口、出口、portal、资源和 queue rule。
- `ResourceManager` 的资源占用与排队逻辑。
- director 的服务时长、状态事件、激活和回收生命周期。
- `manual_collection_node`、`EpisodeRecorder` 和 rosbag 数据格式。
- 机器人侧 hard guard。迁移初期可继续保留机器人 voxel guard。
- Isaac 角色资产和 AnimGraph walk/idle 动画。

### 替换

- director 中的 `VoxelPathPlanner` 行人路径。
- `Person` 对行人的 static voxel veto 和 voxel lateral avoidance。
- `MoveCharacters` 的隐式 NavMesh fallback。
- director 中与局部避障重复的 robot-yield 和 motion recovery。
- 强制 staging yaw、瞬移吸附和运动途中 direct-pose 修正。

## 4. 分阶段实施

### Phase 0：HuNav-Isaac 隔离验证

目的：先验证依赖和执行模型，不接厕所状态机。

场景：

1. 单行人直线行走、停止、继续。
2. 两行人迎面交叉。
3. 两行人狭窄通道相遇。
4. 静止机器人阻挡行人。
5. 移动机器人与行人交汇。
6. 行人接近终点后平滑减速并保持正确朝向。

验收：

- 连续运行 20 个固定 seed episode，无崩溃、穿墙、闪现或永久卡死。
- 行人间不发生明显视觉重叠。
- HuNav 位姿、Isaac root 位姿和 ROS state 的误差可记录且有界。
- walk/idle 与实际速度一致，停止后无行走动画。

回滚点：不修改现有 toilet director，只新增隔离测试入口。

### Phase 1：抽取 Motion Backend

在 director 下定义稳定接口：

```text
spawn(agent)
activate(agent, pose)
set_route(agent, goals, final_yaw)
set_activity(agent, activity)
stop(agent)
retire(agent)
read_states()
```

先实现 `IsaacPeopleBackend`，封装当前 ROS service/topic，使行为保持不变。director 不再直接了解
`MovePed`、`path_points_flat`、AnimGraph 或 HuNav message。

验收：

- 现有单行人流程与迁移前一致。
- director 状态机单元测试全部通过。
- backend 可使用 mock state stream 做无 Isaac 测试。

### Phase 2：标准 2D 几何地图与全局规划

- 从场景静态碰撞几何生成 ROS occupancy map。
- 对墙、门框、隔板、洁具做统一二维投影。
- 按行人 footprint 做 inflation。
- 使用 Theta* 生成少量、平滑、可视化 waypoint。
- semantic pose 必须投影到附近可行区域，但保留原始语义 anchor。

验收：

- 入口到五个小便池及出口的路径全部可达。
- 路径不穿越墙、隔板和洁具。
- 门口没有多余横向 staging 转折。
- 地图、路径和 seed 可保存并离线复查。

### Phase 3：HuNavBackend

每个 simulation tick：

1. 读取所有行人和机器人当前状态。
2. 组装 `hunav_msgs/Agents` 和 robot `Agent`。
3. 调用 `/compute_agents`。
4. 将返回的 pose、velocity、yaw 写入 Isaac mirror。
5. 发布统一 pedestrian state 供 director 和 recorder 使用。

约束：

- HuNav 是唯一局部运动所有者。
- Isaac NavigationManager 不再生成或修改行人路径。
- Isaac 角色 yaw 正常由速度方向产生。
- 只有停止活动状态才使用显式 final yaw。

验收：

- Phase 0 场景在正式 backend 下重复通过。
- 两到四名行人在厕所内可以稳定避让。
- 机器人阻挡后行人能自然等待或绕行，并在清空后恢复。

### Phase 4：厕所 Behavior Tree

将厕所事件映射为可审计行为节点：

```text
EnterPortal
AcquireResourceOrQueue
FollowGlobalRoute
ApproachActivityPose
UseUrinal
ReleaseResource
ExitPortal
Retire
```

director 继续拥有资源锁和阶段转换；BT 负责 GoTo、Wait、LookAt、Follow 和局部反应。

验收：

- 五个小便池可按 seed 复现。
- 多人排队、资源晋升和退出顺序稳定。
- portal token 只用于事件级容量控制，不修改逐帧行人位姿。

### Phase 5：评估与数据有效性

每个 episode 保存：

- scenario、seed、人数、角色和目标资源。
- occupancy map hash、planner 参数和生成路径。
- HuNav agents YAML、BT XML 和 SFM 参数。
- Isaac、HuNav、arena-rosnav 和 arena-isaac commit。
- director 事件、HuNav 状态、ROS 状态和关键日志。

新增有效性检查：

- 行人任务完成率和异常 retire 原因。
- 行人-行人、行人-机器人最小距离和碰撞次数。
- 卡住时间、路径回退、位姿跳变和动画状态不一致。
- portal、queue、resource 的等待与占用时间。
- TTC、personal-space intrusion、路径平滑度和社会导航指标。

### Phase 6：GROVE 风格场景生成

基础链路稳定后再增加：

- semantic RoI。
- normal、queue、emergency 等 preset。
- 可复用 BT subtree。
- 行为与场景参数的受控随机化。
- 可选自然语言到场景配置生成。

第一版不引入 LLM、RAG 或学习式轨迹生成。

## 5. 自动化日志与调试方案

### 日志目录

每次测试创建独立目录：

```text
/tmp/toilet_hunav_debug/<run_id>/
  manifest.yaml
  bridge.log
  hunav.log
  director.log
  collector.log
  rosout.log
  topics.txt
  services.txt
  events.jsonl
  rosbag2/
  result.json
```

### 统一启动器

后续新增一个测试启动脚本，职责为：

1. 创建 `run_id` 和日志目录。
2. 记录 git commit、dirty status、配置文件和环境变量。
3. 启动 bridge、HuNav、测试 director 和 recorder。
4. 等待必需 topic/service ready。
5. 执行指定场景和 seed。
6. 设置超时并收集退出码。
7. 正常发送 SIGINT，必要时再终止残留进程。
8. 生成机器可读的 `result.json`。

### 可直接读取的证据

- PTY 中 bridge、HuNav 和 director 的实时 stdout/stderr。
- `ros2 topic echo/hz/info` 和 `ros2 service list/type`。
- `/rosout`、director status 和 HuNav evaluator 输出。
- rosbag2 中的 odom、scan、pedestrian state、cmd_vel 和事件。
- episode `metadata.yaml`、`events.jsonl` 和轨迹可视化。
- Isaac 崩溃栈、core dump 摘要和进程退出码。

### 自动判定

每个 smoke test 至少输出：

```yaml
passed: true
termination_reason: completed
pedestrian_count: 2
collision_count: 0
pose_jump_count: 0
max_state_mirror_error_m: 0.0
max_stationary_walk_duration_sec: 0.0
stalled_agents: []
```

这样调试不再依赖人工描述 UI 现象。UI 观察仍用于视觉真实性确认，但代码正确性必须由日志和轨迹
证据支撑。

## 6. 下一步

立即执行 Phase 0，不改现有 toilet director：

1. 核对当前 workspace 中 HuNavSim、`hunav_msgs` 和 BehaviorTree.CPP 版本。
2. 拉起现有 `hunav_agent_manager` 并验证 `/compute_agents` 接口。
3. 对照官方 HuNav Isaac Wrapper，做最小的两行人 state mirror 原型。
4. 新增固定 seed 的空场景和狭窄通道 smoke test。
5. 建立统一日志目录、超时和结果摘要。
6. Phase 0 通过后，再抽取 director motion backend。

## 7. 明确不做

- 不继续调行人 voxel guard 的半径和阈值。
- 不在 `Person.update()` 中加入新的行人间 guard。
- 不让 HuNav 直接接管厕所资源和 episode 生命周期。
- 不同时保留 HuNav、Isaac NavMesh 和 director voxel A* 三套路径所有者。
- 不在隔离验证通过前修改现有数采主链路。
