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
Semantic Region / Portal Corridor
        |
2D Global Routing (Theta*, sparse guidance)
        |
HuNavSim Local Social Motion (single motion authority)
        |
Isaac Pose/Velocity Adapter + AnimGraph
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
9. 门、队列和小便池必须用有面积的语义区域表达，不能把单个点同时作为路径目标、状态完成条件和最终姿态目标。
10. waypoint 只能是几何引导，不能拥有事件状态；director 不得根据 waypoint 是否精确命中来重发局部运动命令。
11. RTX 雷达只服务机器人观测、RViz 检查和数据录制，不参与行人地图生成、全局规划或 hard safety。

## 3. 控制权边界

当前链路的根本问题不是某一个阈值，而是同一名行人的运动同时受以下模块控制：

- director 生成路径、判断到达并在停滞时重发路径。
- Isaac NavigationManager / AnimGraph 解释 waypoint、目标半径和朝向。
- voxel guard 或安全投影暂停、改写或拒绝运动。
- mirror 再用另一套 HuNav 状态计算候选轨迹。

迁移后的约束是：

```text
director: 任务意图、资源、portal token、开始/完成事件
global router: 静态几何上的稀疏 route/corridor
HuNav backend: 每个 tick 唯一的期望 pose/velocity/yaw
hard safety: 只裁剪本 tick 非法速度，不生成新路径
Isaac adapter: 应用状态并驱动 walk/idle，不判断到达
```

任何时刻只允许一个模块产生下一时刻的行人运动状态。参数调优只能在该边界建立后用于校准速度、个人空间和行为差异，不能继续用于协调多个到达判定器。

## 4. 保留与替换范围

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

## 5. 分阶段实施

### Phase 0：HuNav-Isaac 隔离验证

目的：先验证依赖和执行模型，不接厕所状态机。

当前状态（2026-07-26）：

- 纯 ROS 隔离 runner、固定 seed 轨迹、结果摘要和 hard safety 已完成。
- bottleneck 确定性让行和纯 yaw terminal alignment 已通过反事实与 20-seed 定向验证。
- 默认七场景 seed 42 为 7/7；三个高风险场景 seed 42 到 61 为 60/60。
- 单行人 shadow mirror、motion intent 旁路和 Isaac root yaw/速度观测已经实现。
- 已完成多轮 live Isaac shadow episode，并采集实际轨迹、阻挡和终点稳定性数据。
- shadow 仍不发送 Isaac 运动命令；门后偏转和小便池附近微动说明当前 point-to-point
  执行不能直接升级为 takeover，应先建立 corridor/region 契约和单一运动权威。

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

回滚点：不修改 director 状态机或动作请求，只新增隔离入口和只读状态事件。

### Phase 0.5：Portal Corridor 隔离验证

当前状态（2026-07-26）：已完成。

- 新增 backend-neutral `PortalCorridor` 和 motion intent 元数据。
- 第一次将 locomotion goal 放在 clear plane 时 5/5 提前停止，确认事件平面不能复用为停止目标。
- 将 locomotion goal 放到 clear plane 后 `0.35 m`，seed 42 到 61 为 20/20 通过。
- 最大横向偏差约 `0.0177 m`，门中停顿 `0 s`，反向进度 `0 m`。

门口不再表示为必须依次到达的：

```text
outside -> center -> inside -> inside_staging
```

而是表示为一个有方向、有宽度和容量的 portal：

```text
approach region -> portal corridor -> clear plane
```

字段语义：

- `outside_pose` 和 `inside_pose` 定义通行轴线和方向。
- `half_width_m` 定义允许通过的横向范围。
- `clearance_m` 定义身体越过 inside/outside plane 后才释放 portal 的距离。
- `capacity` 第一版固定为 1。
- `center_pose` 与 `inside_staging_pose` 暂时保留为旧链路兼容字段，但不再作为 corridor 模式下的到达目标。

状态流：

```text
APPROACH_PORTAL
  -> ACQUIRE_PORTAL
  -> TRAVERSE_PORTAL
  -> CLEAR_PORTAL
  -> RELEASE_PORTAL
```

穿越期间不得在门中心停顿或施加终态 yaw。完成条件是根节点沿 portal 方向越过 clear plane 且仍位于 corridor 横向范围内。

验收：

- 单行人以连续速度穿过门洞，中间没有 waypoint stop/turn。
- 临近门框但未进入 corridor 的位姿不能误判完成。
- 越过 clear plane 后立即释放 portal，不依赖到 staging 点的欧氏距离。
- 正反方向使用同一几何定义和互斥容量。

### Phase 1：建立 Motion Backend 和单一运动权威

当前状态（2026-07-27）：单行人 takeover 已接通，已新增可观测性和机器人几何 hard safety。

- 新增 backend-neutral `MotionCommand` 和 `MotionBackend` protocol。
- 新增 `IsaacPeopleBackend`，集中负责 `MovePed/NavPed` 构造、service readiness 和异步 callback。
- director 不再导入 `MovePed` 或 `NavPed`，现有路径、速度、direct pose 和 motion intent
  通过兼容 backend 原样发送。
- 字段级契约测试覆盖显式零速度、flattened path、direct pose、constrained path 和 callback。
- 新增 backend-neutral `RouteRequest/RouteProvider`，当前 `VoxelRouteProvider` 保留旧链路的
  动态机器人障碍、静态可达性复核和 fail-closed 行为；director 不再实现 A* 失败分类。
- 新增 `PedestrianObservation` state stream，将 `people/pedestrians`、三种 agent id、
  多层 pose 字段和 guard tags 统一后再交给 director。
- 已新增 `HuNavMotionBackend`：普通行走指令由 HuNav `reset_agents/compute_agents`
  连续推进，并通过 external-motion 命令写回 Isaac；激活和停止仍走兼容 adapter。
- Isaac `Person` 已新增 `external_motion` authority：physics tick 内按最近一次 world
  velocity 平滑推进，超时冻结并切换 Idle，跳过原 People path integrator 和 voxel guard。
- 当前显式限制为单行人；正式开放多人前仍需 live Isaac 验证以及共享 HuNav world state。
- RViz 诊断发布到 `/toilet_benchmark/hunav/route` 与
  `/toilet_benchmark/hunav/markers`：前者是 director 下发给 HuNav 的语义 route，后者显示
  HuNav tick 参考箭头与 portal corridor。generation、phase、目标 yaw、速度和裁剪状态也写入
  backend 日志，便于把视觉现象和控制层来源对应起来。
- 已删除实验性的离线 voxel tick 裁剪。该地图覆盖范围、膨胀模型和厕所交互 anchor 不一致，
  会把门外出生点、小便池终点及合法通道误判为占据，造成冻结和反复转向。
- HuNav backend 明确拒绝 director 的 legacy stall recovery；局部停顿不再触发路径重发和
  `reset_agents`。兼容 `IsaacPeopleBackend` 仍保留旧恢复机制用于 A/B 回滚。
- 当前 hard safety 仅处理行人与机器人几何接触。静态几何约束将在 Phase 2 由标准 walkable
  map 和 Isaac PhysX shape sweep 分别承担，二者不能复用 RTX scan。

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

先实现 `IsaacPeopleBackend` 作为兼容后端，封装当前 ROS service/topic。随后实现
`HuNavMotionBackend`，由它在固定 tick 内输出连续 pose/velocity/yaw。director 不再直接了解
`MovePed`、`path_points_flat`、AnimGraph 或 HuNav message，也不再根据局部停顿重发路径。

`HuNavMotionBackend` 不复用现有 `use_direct_pose` 作为连续运动接口。该接口面向一次性位姿
设置，无法表达稳定的外部运动权威和 Walk/Idle 生命周期。takeover 必须配套 Isaac
`external_motion` adapter：

- 每个 physics tick 接收 HuNav pose、world velocity 和 yaw。
- HuNav 独占根位姿推进，Isaac People 不再同时积分 path points。
- 速度阈值只控制 Walk/Idle 动画，不反向改变 HuNav 轨迹。
- 碰撞 proxy 跟随根位姿，但不能把角色切换成刚体或 kinematic locomotion。
- 指令超时自动冻结并进入 Idle，禁止继续沿旧速度漂移。

兼容后端只用于 A/B 和回滚；正式迁移完成后，不能让 `IsaacPeopleBackend` 与
`HuNavMotionBackend` 同时拥有同一 agent。

### Isaac Sim 可视化门槛

- 当前兼容 backend 阶段可以在 Isaac Sim 做回归验证，但视觉效果应与旧链路一致。
- `HuNavMotionBackend + Isaac external_motion adapter` 已达到首次单行人 Isaac
  可视化门槛，启动与回滚命令见 `docs/hunav_takeover_cn.md`。
- takeover 首轮只启用一个固定角色、一个固定 portal 和一个固定小便池，并提供开关在
  `IsaacPeopleBackend` 与 `HuNavMotionBackend` 间二选一。
- 单行人完成连续穿门、机器人阻挡恢复、到达 interaction region 和 walk/idle 一致性后，
  才进入多人可视化与正式数采。

验收：

- 现有单行人流程与迁移前一致。
- director 状态机单元测试全部通过。
- backend 可使用 mock state stream 做无 Isaac 测试。

### Phase 1.1：Interaction Region 与终态对齐

当前状态（2026-07-27）：已开始实现单行人小便池终态契约。

不能把“小便池前的一个点”同时用作全局路径终点、HuNav goal、事件完成条件和最终朝向。
这会让 director 在较宽的距离阈值内进入 `USING_URINAL`，而 HuNav 仍在消费最后一个
goal，导致局部 SFM 继续小幅修正、yaw 继续跟随瞬时速度。

正式契约为：

```text
FollowGlobalRoute
  -> EnterInteractionRegion
  -> HuNav consumes terminal goal
  -> FINAL_ALIGN_URINAL
  -> stable position + zero speed + final yaw
  -> USING_URINAL
```

- HuNav takeover 下，`WALK_TO_URINAL` 不再由纯几何距离直接完成；必须先收到 backend
  terminal-goal settled handshake。
- `FINAL_ALIGN_URINAL` 不发送新路线。Isaac external-motion adapter 保持零平移，并以受限
  角速度平滑转向资源语义 yaw。
- 只有位置已 settle、速度低于阈值、yaw 误差在容差内且持续稳定一段时间，director 才发送
  stop 并开始 `USING_URINAL` 服务计时。
- 此处禁止 direct-pose 吸附；它只能用于 spawn、park 和不可恢复故障的显式恢复。

第一版参数：

```yaml
director:
  urinal_final_yaw_tolerance_rad: 0.20
  urinal_final_alignment_stable_sec: 0.50
  urinal_final_alignment_max_speed_mps: 0.05
```

验收：进入服务状态后行人必须 Idle、位置不再漂移、朝向稳定指向小便池；机器人或其他行人
阻挡时，仍留在导航/等待阶段，不能提前进入使用状态。

### Phase 2：标准 Walkable Map 与全局规划

当前状态（2026-07-27）：静态地图导出与只读校验已完成，HuNav takeover 已切换到
walkable-map global router。

- bridge 新增 `/isaac/export_walkable_map`，使用 Isaac Sim 4.5 官方
  `isaacsim.asset.gen.omap` 从 PhysX collision geometry 生成地图。
- 导出期间排除 scene root 外的机器人、Characters 和 debug collision，输出 JSON、PGM、
  ROS map YAML 与场景碰撞指纹。
- `walkable_map_publisher` 发布 transient-local `OccupancyGrid` 与语义 anchor 标记。
- `walkable_map_validation.yaml` 只是验收快照，不驱动路径或覆盖 `toilet_semantics.yaml`。
- `WalkableMapPlanner` 将 occupied 和 unknown 视为不可通行，按行人 footprint 构造 clearance，
  经过确定性 A* 和 line-of-sight 简化后输出稀疏 route。
- 仅 `--motion-backend hunav` 使用新 provider；默认 `isaac_people` 继续使用旧 voxel provider
  作为兼容回滚。
- 入口到五个小便池及全部返回路线已离线通过，portal 线段可行，路径为 4–7 个点。
- 每条 route 日志记录完整 waypoint、scene fingerprint；director 启动日志记录随机 seed。

地图输入边界：

- 唯一几何来源是加载后的 USD 静态碰撞几何或由它烘焙出的 walkable surface/NavMesh。
- 允许导出 ROS `OccupancyGrid` 作为调试和 Theta* 输入，但它必须由指定静态 prim 生成，
  不能由 `/front_scan`、`/rear_scan` 或历史 RTX 点云拼图产生。
- RTX 雷达保持独立：它可以在 RViz 与 rosbag 中和行人路线同时显示，但不会反馈给行人系统。

实现顺序：

1. 从场景静态碰撞 prim 生成带版本和场景 hash 的 walkable map。
2. 对墙、门框、隔板、洁具做统一二维投影，并显式保留 portal 开口。
3. 用行人 footprint 配置空间或 clearance field，而不是对语义 anchor 一律做圆形膨胀。
4. 将 entrance、urinal interaction region、exit 投影到最近合法 walkable region，同时保留原始语义 anchor。
5. 使用 Theta* 或 funnel/string-pulling 生成少量平滑 route；门口输出 corridor 约束，不输出门中心 stop waypoint。
6. 运行时用 Isaac PhysX shape sweep 做短时域硬安全复核，不重新启用旧 voxel guard。

职责明确为：`global router` 仅使用静态 walkable map 生成路径；HuNav 不负责静态全局寻路，
只跟随 route lookahead 并处理局部人与人、人与机器人反应。现有 voxel A* 仅保留在
`IsaacPeopleBackend` 兼容链路中做 A/B 对照，不得进入 HuNav tick。

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
- HuNav 每个 tick 接收当前真实状态；不能将整段 HuNav 候选轨迹重新编码成 Isaac `GoTo` 列表。
- Isaac 角色 yaw 正常由速度方向产生。
- 只有停止活动状态才使用显式 final yaw。
- 当前单行人 backend 在每次 director 阶段命令时会重置 HuNav agent。进入多人前必须替换为
  持续运行的 shared HuNav world：所有 active agents 与机器人每 tick 一起提交给
  `compute_agents`，阶段变化只更新该 agent 的 route/corridor/semantic goal，不能重建其
  SFM 和 BT 状态。

验收：

- Phase 0 场景在正式 backend 下重复通过。
- 两到四名行人在厕所内可以稳定避让。
- 机器人阻挡后行人能自然等待或绕行，并在清空后恢复。

### Phase 4：厕所 Behavior Tree 与语义区域

将厕所事件映射为可审计行为节点：

```text
ApproachPortal
AcquirePortal
TraversePortal
ClearPortal
AcquireResourceOrQueue
FollowGlobalRoute
EnterActivityRegion
SmoothFinalAlignment
UseUrinal
ReleaseResource
TraverseExitPortal
Retire
```

director 继续拥有资源锁和阶段转换；BT 负责 GoTo、Wait、LookAt、Follow 和局部反应。
小便池完成条件由 interaction region、朝向容差和稳定时间共同构成，不再使用单一目标点距离。

BT 不应为每一对行人与机器人硬编码状态，而是读取黑板事实：`semantic_region`、`phase`、
`portal_token`、`resource_owner`、`route_blocked`、`TTC` 和 `blocking_actor`。推荐策略为：

```text
开放区域: FollowRoute + HuNav local avoidance
狭窄 portal: AcquireToken -> TraverseCorridor -> Wait/Yield
interaction region: FinalApproach -> FinalAlign -> Use
机器人阻挡: open area 可绕行；portal/小便池 approach 优先停留等待
```

director 仍是资源与 episode 的唯一真相来源；HuNav BT 只决定当前如何走、等、看或恢复，
不能自行占用小便池、释放队列位置或决定 episode 成败。

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

## 6. 自动化日志与调试方案

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

## 7. 下一步

按以下顺序推进：

1. 实现 USD 静态 prim 到 walkable map 的离线导出与可视化校验，覆盖门外入口、五个小便池 interaction region 和出口。
2. 用新 map router 替换 HuNav takeover 当前收到的 voxel route，并记录 map hash、route 和 seed。
3. 接入 Isaac PhysX shape sweep 作为静态短时域 hard safety，验证不会在合法 anchor 冻结。
4. 单行人通过后，再实现共享 HuNav world state 与多行人 portal capacity。

## 8. 明确不做

- 不继续调行人 voxel guard 的半径和阈值。
- 不在 `Person.update()` 中加入新的行人间 guard。
- 不让 HuNav 直接接管厕所资源和 episode 生命周期。
- 不同时保留 HuNav、Isaac NavMesh 和 director voxel A* 三套路径所有者。
- 不将 RTX scan 或机器人在线雷达地图作为行人规划输入。
- 不在隔离验证通过前修改现有数采主链路。
