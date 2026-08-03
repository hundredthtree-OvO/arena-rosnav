# 路线编辑器与运行链路清理计划

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

## 阶段 0：基线冻结

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

当前基线运动入口：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --initial-agents 1
```

`manual_collection_node`、`hunav_interactive_smoke` 和 Replay 入口继续作为已有 Track 的验证入口，不因 UI 清理而改变命令语义。

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

### 暂存为 compatibility backend

以下内容仍被现有入口或测试引用，第一轮只隔离职责，不直接删除：

- `hunav_motion_backend.py`：当前 HuNav Interactive 兼容实现；
- `hunav_adapter.py`：director 兼容状态镜像；
- `hunav_isaac_mirror*.py`：HuNav/Isaac 对照工具；
- `hunav_phase0_*.py`：隔离实验和安全投影工具；
- `voxel_path_planner.py`：旧 `isaac_people` compatibility 路径；
- portal recovery 和 `use_direct_pose`：只保留在 compatibility 生命周期边界。

当前连续运动主链使用 `walkable_map` 路线和 external motion；voxel 路径不能再作为新 UI 或新局部运动的隐式依赖。只有兼容 backend 明确选择时才允许使用它。

### 后续可删除项

只有满足以下条件后，才删除历史实验脚本、旧 portal/voxel 分支和重复配置：

1. 固定 seed 单人、双人和 2-4 人 smoke 通过；
2. Replay 轨迹和 Interactive 事件终态通过 validator；
3. 新 backend 已覆盖旧入口需要的 spawn、hold、resume、terminal align 和 retire；
4. 旧模块没有被 `rg`、setup entry point、测试或文档操作面引用；
5. 完成一次 shadow 对照并保留可回退 tag。

因此本阶段禁止使用 `git reset`、批量删除或覆盖用户未提交修改。

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

- UI 不启动时，现有固定 seed 行为不改变；
- UI route edit 不产生瞬移或动画脱节；
- 非法路线被拒绝并给出原因；
- subgoal 结束后能回接原全局路线；
- 同一个 seed、路线版本和 UI 操作时间线可以 Replay；
- UI 关闭、重启或断开不会使 bridge 或 director 崩溃。

## 本轮执行状态

- 阶段 0：已建立纯 Python 回归基线，环境阻断已记录；
- 阶段 1：完成运行链路分类和文档冻结，backend 选择已从 director 提取到独立工厂；
- 阶段 2：已实现 `RouteEdit`、`SubgoalCommand` 和版本/TTL 纯 Python 契约测试；
- 阶段 3：已替换为独立 Qt `route_editor_node`；旧只读 `route_view_node` 和专用
  RViz 配置已删除；live commit 仍未实现。
