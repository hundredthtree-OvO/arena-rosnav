# 路线编辑器与运行链路清理计划

## 2026-08-04 主线决策

本计划不再冻结旧 People Navigation、HuNav/director 或双轨 compatibility 实现。Git 提交
历史是唯一代码回退机制；仓库内不为回退保留第二套运行时、shadow 分支或旧状态机。

第一交付目标收敛为一个可自动验收的 vertical slice：

```text
toilet_benchmark_ui/examples/narrow_head_on_001.json
  -> authored ScenarioRuntime
  -> 每个 agent 独立的路线游标和 episode generation
  -> GlobalRouter + LocalMotionBackend
  -> EmbodimentAdapter
  -> Isaac AnimGraph（仅动画）/ 后续 SMPL-H
```

该 JSON 中的两个行人必须能在同一 bridge 内跨 episode 批量复现。事件、路线、速度和
朝向的真值由 benchmark runtime 持有；不得再读取或复用 Isaac People 的 GoTo command、
NavigationManager 路径游标或内部目标状态。

## 目标

本计划为厕所社会导航 benchmark 增加一个独立的平面图路线编辑器，同时把行人事件、全局路线、局部运动和 Isaac 表现层的责任收敛到稳定接口。

UI 不是新的运动控制器，也不直接写 Isaac Prim。它只产生可记录、可校验、可回放的路线编辑和局部 subgoal 命令。

```text
route editor UI
  -> ScenarioRuntime / AgentExecutive
  -> RoutePlan / SubgoalCommand
  -> LocalMotionBackend
  -> Isaac external_motion / AnimGraph / SMPL-H adapter
```

## 阶段 0：接口与验收夹具

### 冻结的操作面

场景和机器人主操作面保持不变：

```bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

首次导入资产和机器人：

```bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  spawn --phase physx_diff_contact
```

默认验收入口是 `manual_collection_node` 加载 `narrow_head_on_001.json`。旧 director、
HuNav smoke 和 Isaac People GoTo 不再属于必须保持的操作面。

### 基线记录内容

每次结构清理前至少记录：

- 固定 seed、目标 Smart Object、人数和 behavior profile；
- agent 的 phase 顺序、目标到达和退出终态；
- route version/generation、路径点和局部 subgoal；
- 行人和机器人最小距离、视觉包络重叠、静态碰撞裁剪；
- 轨迹跳变、走停次数、路线进度倒退和恢复次数；
- bridge、director 和 recorder 的退出状态。

### 当前基线证据

2026-08-03 在当前 worktree 上运行了不依赖 ROS 生成消息的纯 Python 回归集合：

```text
74 passed, 1 skipped
```

全量测试尚未作为通过条件：当前 shell 缺少 `isaacsim_msgs` 和 `hunav_msgs` 的 Python 运行时导入；工作区 `install/setup.bash` 还引用不存在的 `arena-rosnav/local_setup.sh`。这属于环境/构建阻断，不能误报成代码通过。

## 阶段 1：清理运行链路

### 保留为 canonical runtime

这些模块属于新结构的稳定边界，不能在 UI 接入前删除：

- `scenario/`：`ScenarioRuntime`、`AgentExecutive`、`SmartObjectRegistry` 和 `EnterUseExit`；
- `episodes/`：EpisodeSpec、manifest、validator、splits 和 Replay；
- `motion/contracts.py`、`motion/ports.py`、`motion/pipeline.py`；
- `motion_backend.py`：`MotionCommand`、latest-only external motion 和 Isaac service adapter；
- `local_motion_backend.py`：后续路线编辑器的第一接收端；
- `motion/swept_envelope.py` 和几何安全接口；
- `manual_collection_node.py`、recorder 和现有 benchmark CLI。

### 直接删除的旧运行链

新 vertical slice 通过后，直接删除而不是封存：

- `toilet_director_node` 的 People/HuNav/portal/voxel 运动分支；
- `IsaacPeopleBackend` 的 GoTo/NavigationManager 路径所有权；
- HuNav mirror、takeover 和 phase0 生产入口；
- 为旧路径恢复服务的 stall recovery、direct-pose 对齐和重复 hard guard；
- 不再被 setup entry point、authored runtime 或测试引用的配置与实验代码。

删除前只要求：新链路完成下述 `narrow_head_on_001` 验收、当前改动形成 Git 提交、引用
扫描通过。无需 shadow 对照，也不在源码中保留回退开关。

历史文档可以删除；仍有设计价值的结论应先压缩进当前规范，而不是保留整套旧操作说明。

## 阶段 2：路线和 subgoal 契约

先在 simulator-neutral 层新增纯 Python 数据结构：

```text
RouteEdit
- agent_id
- base_generation
- route_version
- phase
- waypoints
- commit

SubgoalCommand
- agent_id
- route_version
- target_xy
- ttl_sec
- speed_limit
- priority
- reason
- resume_policy
```

规则：

- UI 不能逐帧发布 `direct_pose`；
- generation/version 不匹配的命令必须丢弃；
- subgoal 只覆盖局部目标，不改变厕所语义目标；
- `USING_URINAL` 等活动阶段不能被普通路线编辑覆盖；
- subgoal 完成或过期后必须重新接回全局 RoutePlan；
- 所有 waypoint 必须经过静态几何和人体扫掠包络验证；
- 每次编辑都要写入 episode event/manifest，保证 Replay 可复现。

## 阶段 3：独立平面图 UI

UI 放在同一 Git 仓库的独立 ROS 2 包 `toilet_benchmark_ui`，不混入核心 `toilet_benchmark` 节点，也不成为 headless benchmark 的依赖。

本轮把阶段 3 从只读观察面收敛为独立 Qt 2D 编辑器；旧的 `route_view_node` 和
专用 RViz 配置不再保留。RViz 继续作为 LiDAR、TF 和 3D 场景诊断工具。

编辑器第一切片的清理计划：

1. 删除 `route_view_node`、旧 wrapper 和 `route_editor_view.rviz`，避免两个 UI 入口
   重复显示同一批状态；
2. 保留 `toilet_benchmark_ui` 作为独立进程，新增 `route_editor_node`；
3. UI 主线程只绘制和处理鼠标，ROS executor 在独立线程中采用 latest-only 状态缓存；
4. 鼠标编辑先形成本地 draft，静态占据验证通过后才允许保存或 shadow 发布；
5. shadow 命令只发布版本化 JSON，不直接调用 Isaac service；live commit 留在后续
   director 接入阶段。

阶段 3 第一切片的实现目标：

- 独立包：`toilet_benchmark_ui`；
- 节点：`route_editor_node`；
- Qt 2D 窗口，不要求 RViz 启动；
- 不调用 director、Isaac service 或 HuNav 控制接口。

当前显示：

- `map` 坐标系下的静态地图、墙体、门、portal 和 Smart Object；
- 行人身体包络、朝向箭头和名称标签；
- 当前 HuNav 路线和 portal/诊断 marker；
- walkable map 和语义 anchor。

People 消息中若带有 `yaw_rad`/`heading_rad` 标签则使用该朝向，否则使用速度方向；
静止行人保持上一规则的零朝向显示。地图和 People 的 frame 不一致时节点会发出
warning，而不是静默地把坐标变换成另一个 frame。

当前支持的操作：

- 选择 agent；
- 添加、删除、拖动 waypoint；
- 设置 subgoal、hold、resume、clear-subgoal；
- 保存本地 route/session JSON；
- 发布显式 shadow `RouteEdit`/`SubgoalCommand` JSON。

当前仍不允许 live commit；必须先增加 director 端 ack、generation/version 校验和
回接路线测试。

## 验收门槛

- 同一 bridge 连续执行至少 30 个 `narrow_head_on_001` episode；
- 两名行人每轮出生误差、初始 yaw 和首段路线一致，不出现第二轮漂移；
- 每名行人拥有独立 generation、路线游标、速度状态和终态，禁止共享 People command；
- 行人先到先 retire，下一轮不继承上一轮路径、朝向或动画状态；
- 机器人每轮 reset 后手柄可控，wheel actual、tire force 和 odom 均有效；
- 固定 seed 的事件序列和路径在约定容差内可复现；
- UI route edit 不产生瞬移或动画脱节；
- 非法路线被拒绝并给出原因；
- subgoal 结束后能回接原全局路线；
- 同一个 seed、路线版本和 UI 操作时间线可以 Replay；
- UI 关闭、重启或断开不会使 bridge 或 authored runtime 崩溃。

### 首轮量化阈值

- episode 完成率：`30/30`；
- agent 激活位置误差：`<= 0.05 m`；
- 激活 yaw 误差：`<= 0.10 rad`；
- 跨 episode 非命令位移跳变：`<= 0.03 m`；
- 行人静态几何穿透：`0`；
- 行人间重叠：`0`；
- robot reset 后命令非零但 odom 零响应：`0`；
- `reset_apply_failed`、People cursor reuse 和 AnimGraph root reuse：`0`。

## 后续实施顺序

### P1：切断 People 的任务状态所有权

1. `toilet_authored_scenario` 直接从 EpisodeSpec 建立每个 agent 的 `RoutePlan`；
2. 每个 agent 持有独立的 generation、route cursor、phase 和 terminal state；
3. benchmark 每个 phase 只提交一次冻结的完整 `PathPoints`；
4. People/MotionMatching 只负责低层路径消费和动画，不生成语义目标、事件或终态；
5. benchmark 使用实际 pose 判定 hold、下一 phase 和 retire，不读取 People 内部到达状态。

当前实现状态（2026-08-04）：

- `toilet_authored_scenario` 已为每个 agent 独立维护 generation、phase boundary、hold 和
  terminal state；
- authored runner 将 benchmark 规划出的整段 `PathPoints` 交给 People/MotionMatching，保留
  Isaac 原生连续步态和转弯；
- `/isaac/pedestrian_states` 用于出生确认和 benchmark-owned waypoint 到达判定；不读取
  `NavigationManager` 的目标、路径游标或终态；
- spawn/reactivate 和最终 park 仍复用 bridge 服务，这部分属于 embodiment 生命周期，不拥有路线；
- P1 纯 Python 契约和 phase 状态测试已通过。跨 episode 复位握手与 30 轮 Isaac 验收属于 P2/P3，
  不能用本阶段单测替代。

### P2：确定性 episode reset

1. 预生成固定数量角色，不增删 USD prim；
2. 结束本轮后停止 motion、关闭碰撞代理并移至 parking pose；
3. 清空 benchmark-owned motion buffer、route cursor 和 adapter 动画状态；
4. 下一轮写入 spawn pose/yaw，等待固定 warmup frame，再提交 generation 对应的首条命令；
5. reset 全程不 pause/play timeline，不重建机器人 PhysX。

当前实现状态（2026-08-04）：

- pooled `Person` 每次 reactivate 都递增独立 `embodiment_generation`；
- bridge 在 `/isaac/pedestrian_states` 中发布 `embodiment_generation` 和
  `reactivation_ready`，后者只有 AnimGraph ready、pose valid 且 4 帧 stabilization 完成后
  才为 true；
- authored runner 在看到有效 embodiment generation 和出生误差 `<= 0.15 m` 后，将第一段
  完整 `PathPoints` 预提交到 Person 的 command generation；该命令在 4 帧 stabilization 内
  只缓存、不执行；
- warmup 结束后 bridge 派发缓存命令，状态话题必须回报同一 embodiment generation、预期
  command generation 且 `motion_state=executing`，runner 才发布 `pedestrian_active`；
- active/retired 状态携带同一个 embodiment generation，旧进程或旧 episode 的事件不能冒充
  本轮角色；
- manual collection 每轮清空 generation roster，并等待全部预期行人 active 后才释放机器人手柄；
- park 仍要求 People command/path 完全 drain；整个过程不 pause timeline、不重建机器人 PhysX。

上述握手和单元契约已完成；同一 bridge 的 5/30 轮 Isaac 量化验证仍分别属于 P2 收尾和 P3。

### P3：双行人批量数采闭环

1. `manual_collection_node` 默认加载 `narrow_head_on_001.json`；
2. authored runtime 发布每名行人的 activated/retired/failed 明确终态；
3. recorder 将 EpisodeSpec hash、seed、generation 和两条实际轨迹写入 metadata；
4. 增加连续 30 轮 runner 和上述量化 gate；
5. gate 通过后才允许扩展 UI 场景和人数。

### P4：删除旧链路

按 `rg -> setup.py -> tests -> docs` 顺序删除旧 director、HuNav、People Navigation、
portal/voxel recovery 和失效文档；随后运行完整测试和一次 30 轮 Isaac 回归。P4 不建立
compatibility package，也不保留禁用代码块。

## 本轮执行状态

- 阶段 0：已建立纯 Python 回归基线，环境阻断已记录；
- 阶段 1：完成运行链路分类和文档冻结，backend 选择已从 director 提取到独立工厂；
- 阶段 2：已实现 `RouteEdit`、`SubgoalCommand` 和版本/TTL 纯 Python 契约测试；
- 阶段 3：已替换为独立 Qt `route_editor_node`；旧只读 `route_view_node` 和专用
  RViz 配置已删除；live commit 仍未实现。
