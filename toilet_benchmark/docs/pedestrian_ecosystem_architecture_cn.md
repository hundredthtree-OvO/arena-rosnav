# 厕所行人事件生态与冻结接口

状态：Architecture Freeze v0.1
冻结日期：2026-08-01
适用范围：后续事件系统重构、Interactive Track、Replay Track 和表现层替换

## 1. 目标

厕所行人系统不是随机 waypoint crowd。它模拟具有明确意图、资源约束和事件生命周期的
行人：进入厕所、选择并占用资源、等待或排队、执行活动、释放资源并离开。

本架构采用游戏 AI 中常见的 `Scenario + Smart Object + Agent Executive + Navigation +
Embodiment` 分层。冻结的是职责、数据方向和外部兼容面，不冻结当前大文件内部实现。

## 2. 唯一控制权

```text
EpisodeSpec / seed
        |
ScenarioRuntime ---- emits ----> BenchmarkEvent ----> Evaluator/Recorder
        |
        +--> SmartObjectRegistry <---- claim/release ---- AgentExecutive
                                                    |
                                                    v
                                              MotionIntent
                                                    |
                               GlobalRouter -> LocalMotionBackend
                                                    |
                                                    v
                                              MotionCommand
                                                    |
                                            EmbodimentAdapter
                                                    |
                                             Isaac / SMPL-H
```

每类决定只能有一个权威：

| 决定 | 唯一权威 | 明确禁止 |
| --- | --- | --- |
| episode 内容和随机采样 | Episode Generator | backend 隐式随机目标或人数 |
| 行人当前想做什么 | Agent Executive | HuNav 改写资源和任务阶段 |
| 资源占用、队列和 portal 容量 | SmartObjectRegistry | motion backend 自行抢占或释放 |
| 静态全局路线 | GlobalRouter | HuNav 或硬安全重写最终任务目标 |
| 下一时刻局部速度 | LocalMotionBackend | director 逐帧修正位姿 |
| 非法穿透判定 | Geometry Safety | safety 主动选绕行侧或社会策略 |
| 骨骼和动画表现 | EmbodimentAdapter | AnimGraph 反向改变任务状态 |
| 成败和指标 | Evaluator | evaluator 反向控制仿真 |

## 3. 领域模型

### 3.1 Smart Object

第一版对象类型：

- `portal`：大门或狭窄通道，包含容量和 traversal slots；
- `urinal`：站位、interaction region、最终朝向和服务时间分布；
- `toilet_stall`：入口、内部交互位、占用状态和使用动作；
- `queue`：有序 slots、晋升规则和最大容量；
- `waiting_area`：资源不可用或通道不可通行时的合法等待区域；
- `spawn` / `retire`：激活与退出边界，不属于可竞争资源。

每个 Smart Object 至少包含：

```yaml
id: urinal_1
kind: urinal
capacity: 1
tags: [toilet.resource.urinal]
interaction_region:
  center: [x, y]
  radius_m: 0.22
  yaw_rad: 1.57
  yaw_tolerance_rad: 0.20
slots:
  - id: use
    pose: [x, y, yaw]
queue_id: urinal_1_queue
```

Smart Object 只描述机会、slots 和预约状态，不包含路径规划或动画执行代码。

### 3.2 Agent Executive

任务意图使用层次状态机或行为树表达。主任务状态和短时反应状态必须正交：

```text
Task state:
  INACTIVE -> SEEK_OBJECT -> WAIT_OBJECT -> APPROACH_OBJECT
  -> INTERACT -> RELEASE_OBJECT -> SEEK_EXIT -> RETIRED

Reaction state:
  NORMAL | YIELD | STOP_AND_LOOK | AVOID | RECOVER
```

`Reaction state` 不得清除 `Task state`、资源 claim 或全局目标。反应结束后从保存的任务上下文
恢复，而不是重新随机选择目标。

### 3.3 Scenario Graph

Scenario 由可组合事件节点构成，而不是为每个 case 写一套 director 分支：

```text
SpawnAgent
ChooseResource
ClaimOrQueue
TraversePortal
FollowRoute
EnterInteractionRegion
PerformActivity
ReleaseResource
ExitScene
```

事件节点可由条件触发：时间、区域进入、资源状态、机器人区域、前序事件完成或显式 marker。
`DoorwayConflict`、`GoalOccupied` 等任务是这些节点和触发条件的模板组合。

## 4. 冻结接口

### 4.1 已存在且保持兼容

以下 Python 契约在第一轮拆分中保持 import 和字段兼容：

- `domain.agent.AgentSnapshot`；
- `domain.events.BenchmarkEvent`；
- `domain.task.MotionCommand`；
- `domain.task.TaskPhase`；
- `episodes.schema.EpisodeSpec`；
- `episodes.schema.RobotEpisodeSpec`；
- `episodes.schema.PedestrianEpisodeSpec`。

以下运行入口和 ROS 面保持兼容：

- `arena_scene_profile.py ... spawn --phase physx_diff_contact`；
- `arena_scene_profile.py ... bridge physx_diff_contact`；
- `ros2 run toilet_benchmark toilet_director_node ...`；
- `ros2 run toilet_benchmark manual_collection_node ...`；
- `/isaac/spawn_pedestrian`；
- `/isaac/move_pedestrians`；
- `/toilet_benchmark/director_status`。

允许新增字段，但必须提供默认值；删除或改名需要 schema/version 升级和迁移说明。

### 4.2 新的内部端口

下列是冻结语义，不要求一次性创建全部 Python 文件：

```python
class ScenarioRuntimePort(Protocol):
    def reset(self, episode: EpisodeSpec) -> None: ...
    def tick(self, snapshot: "WorldSnapshot", dt: float) -> tuple["TaskDirective", ...]: ...

class SmartObjectPort(Protocol):
    def query(self, request: "ObjectQuery") -> tuple["ObjectCandidate", ...]: ...
    def claim(self, object_id: str, slot_id: str, agent_id: str) -> "ClaimResult": ...
    def release(self, claim_id: str, reason: str) -> None: ...

class AgentExecutivePort(Protocol):
    def reset(self, spec: "PedestrianEpisodeSpec") -> None: ...
    def tick(self, world: "WorldSnapshot", task: "TaskContext") -> "MotionIntent": ...

class GlobalRouterPort(Protocol):
    def plan(self, request: "RouteRequest") -> "RoutePlan": ...

class LocalMotionPort(Protocol):
    def step(self, request: "LocalMotionRequest") -> "LocalMotionResult": ...

class EmbodimentPort(Protocol):
    def spawn(self, request: "SpawnRequest") -> None: ...
    def apply(self, command: MotionCommand) -> None: ...
    def retire(self, agent_id: str) -> None: ...
```

### 4.3 端口数据约束

- `WorldSnapshot` 是一个 tick 的不可变快照，包含机器人、全部 active 行人、Smart Object
  状态和仿真时间；不得在一次 tick 内混用不同时间戳。
- `TaskDirective` 只描述任务动作，例如 claim、wait、interact、release，不含局部速度。
- `MotionIntent` 包含 semantic goal、preferred speed、当前 activity/reaction 和 route policy，
  不含 Isaac prim path。
- `RoutePlan` 是静态/语义可行路线，带 map/version hash，不包含动画状态。
- `LocalMotionResult` 返回速度、heading、可行性和结构化诊断，不直接写场景。
- `MotionCommand` 是 motion 到 embodiment 的唯一运动命令。

## 5. 随机性与复现

随机性只能在 episode 生成时采样并落盘：

- 人数、角色、激活时间；
- 目标资源和备选资源；
- 服务时间、耐心和速度 profile；
- behavior profile 和群组关系；
- 事件模板参数与优先权策略。

逐帧算法如需随机数，必须使用由 `episode_seed + agent_id + subsystem` 派生的独立随机流，
并把算法版本和参数 hash 写入 manifest。相同 episode、初态和机器人动作序列必须在容差内
复现相同行人状态序列。

## 6. 事件模板

v0.1 优先实现：

1. `EnterUseExit`：进入、占用小便池、服务、离开；
2. `DoorwayConflict`：机器人与一至两名行人争用 portal；
3. `GoalOccupied`：机器人目标区域被行人占用；
4. `QueueAndPromote`：资源占用、排队和释放晋升；
5. `StallExitBlocked`：机器人从隔间出来，出口走廊存在行人流。

模板只约束语义和触发条件，不硬编码逐帧轨迹。每个模板同时提供 Replay 和 Interactive
实例时，两者共用 EpisodeSpec、机器人接口和 evaluator，仅行人 motion source 不同。

## 7. Embodiment 边界

Isaac AnimGraph 和 SMPL-H 必须消费同一 `MotionCommand`：

- `IsaacAnimGraphAdapter` 是当前兼容基线；
- `SmplHAdapter` 是高保真候选；
- root pose、yaw、速度和活动 phase 来自 motion truth；
- animation phase、foot locking 和骨骼姿态属于表现层；
- 碰撞评估使用版本化的几何 proxy/视觉扫掠包络，不使用渲染 mesh 顶点逐帧参与规划。

SMPL-H 的引入不能改变 episode、资源、路由或 evaluator 语义。

## 8. 迁移不变量

每个代码拆分切片必须满足：

1. 现有 CLI、service 和 topic 不变；
2. 固定 seed 的单行人路径、事件顺序和终态不变，除非该切片明确是行为修改；
3. 结构迁移和行为调参分开提交；
4. 新旧实现可 shadow 对比后再切换；
5. 替代实现通过单元测试和 Isaac smoke 后才删除旧分支；
6. 不新增第四套隐式局部规划或碰撞控制逻辑。

## 9. 长目标路线图

### E0：接口冻结与回归夹具

- 完成本文件和文档索引；
- 固定现有单/双行人日志和事件 trace；
- 为资源 claim/release、阶段转换和 MotionCommand 增加纯 Python golden tests。

### E1：事件内核

- 提取 `SmartObjectRegistry`、`AgentExecutive` 和 `ScenarioRuntime`；
- 用 `EnterUseExit` 兼容复现现有流程；
- director 收缩为 ROS、配置和生命周期 facade。

截至 2026-08-01，E1 的第一代码切片已经完成：

- `scenario/smart_objects.py` 提供 simulator-neutral 的资源、交互槽、FIFO 队列、
  claim/release 和晋升语义；
- `scenario/executive.py` 以每个行人一个 executive 的方式消费语义反馈并输出
  `TaskDirective`；
- `scenario/runtime.py` 按 agent id 稳定排序推进所有 executive；
- `scenario/templates.py` 提供首个 `EnterUseExit` 模板；
- golden tests 已冻结双行人占用、排队、服务、释放、晋升和退出的事件顺序，以及 reset
  后的确定性。

E1 director adapter 也已实现，提供三个显式模式：

- `legacy`：完全关闭新事件 runtime，用于紧急回退；
- `shadow`：旧 director 继续控制运动，新 runtime 消费相同到达/活动反馈，比较 phase、
  owner 和 FIFO queue，不写运动命令；
- `takeover`：`SmartObjectRegistry` 成为资源唯一真相，director 只同步兼容镜像并执行
  `TaskDirective` 对应的旧 motion 子阶段。

固定 seed 的单行人 `shadow/takeover` 以及双行人不同目标 `shadow` 已完成 live 验证，
未出现 E1 phase/owner/FIFO mismatch。默认配置仍保持 `shadow`，因为双行人同资源排队会在
旧 HuNav corridor safety 中互锁，且视觉包络仍检测到门框/墙体重叠；这两个问题属于 E2
运动与几何层，而不是事件内核。可继续使用 `--scenario-runtime takeover` 显式验证；
HuNav、路径、碰撞和动画配置不因该开关变化。

### E2：运动边界

- 提取 `GlobalRouter`、`LocalMotionBackend`、`GeometrySafety`；
- HuNav、ORCA/HRVO 和 Replay 实现统一 LocalMotionPort；
- 使用微场景矩阵选择 Interactive 默认 backend，而非凭单次观感调参。

E2-A 已完成契约和首批生产接线：全局 planner 通过不可变 `RoutePlan` 输出，现有
`SweptEnvelope` 通过 `GeometrySafetyPort` 执行连续静态裁剪；`BehaviorPolicyPort` 和
`LocalMotionPort` 的输入输出也已冻结，但尚未从 HuNav 内部接管逐帧行为与速度。该顺序是
刻意的：先保证路由和几何只有一个权威，再在 E2-B 以 ORCA/HRVO 候选替换局部速度核心，
避免形成第四套隐式避障逻辑。

E2-B 已加入确定性的 `ContextualBehaviorPolicy` 和 sampled velocity-obstacle local
backend，并在 HuNav 原始输出之后、GeometrySafety 之前建立默认关闭的 shadow 调用点。
shadow 只比较相同世界快照下的速度结果，不改变 Isaac motion authority。该候选目前是
dependency-free 几何基线，不宣称实现完整 ORCA；只有通过固定微场景和 Isaac visual
envelope gate 后，才允许以显式配置进入 takeover。

### E3：表现层

- 提取 Isaac AnimGraph adapter；
- 接入 SMPL-H A/B；
- 冻结 root/yaw、foot sliding 和视觉包络指标。

### E4：事件库与生成器

- 实现五个 v0.1 模板；
- 受控随机生成、validator、split 和 oracle 可解性检查；
- 单 bridge 连续 30 episode reset 回归。

### E5：Evaluator 与发布

- 统一 Replay/Interactive runner；
- 冻结 metrics、baseline、结果格式和 benchmark card；
- 通过有效性门槛后再增加 dense flow、群组和自然语言场景生成。
