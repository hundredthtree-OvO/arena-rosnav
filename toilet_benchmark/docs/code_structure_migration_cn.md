# Toilet Benchmark 代码结构迁移计划

状态：S0/S1/S2-A/S2-B completed；E0 frozen；E1 contract verified；E2-B shadow ready
最后更新：2026-08-01

## 1. 目的

本计划服务于两条并行开发线：

1. 修正 HuNav 原生 behavior/BT 接入和多人稳定性；
2. 建设 episode schema、validator、标准接口、Replay Track 和 evaluator。

迁移目标不是为了目录美观，而是避免两条线继续同时修改：

- `toilet_benchmark/toilet_director_node.py`；
- `toilet_benchmark/hunav_motion_backend.py`。

这两个文件当前分别承担过多职责，是后续并行开发、测试隔离和安全删除旧逻辑的主要障碍。

本文只定义代码边界和迁移顺序。完整 benchmark 规范见
`benchmark_design_cn.md`，HuNav 行为迁移细节见
`hunav_pedestrian_pipeline_migration_plan_cn.md`。2026-08-01 后新增代码以
`pedestrian_ecosystem_architecture_cn.md` 的控制权和端口定义为准；旧 HuNav 计划保留为
实验与迁移证据，不再单独定义架构。

## 2. 当前基线

结构审计时的主要文件规模：

| 文件 | 主要职责 |
| --- | --- |
| `hunav_motion_backend.py` | HuNav client、多人 runtime、route tracking、绕行、机器人反应、硬安全、终端对齐、诊断、Isaac external motion |
| `toilet_director_node.py` | 配置、场景 bootstrap、资源、portal、生命周期、恢复、事件和 CLI |
| `manual_collection_node.py` | Dataset Track orchestration、reset、录制握手、director 子进程和终态 |
| `collection_scenarios.py` | 人工数采配置 schema、解析和选择 |
| `episode_recorder.py` | session/episode manifest、event 和 rosbag 生命周期 |

审计时测试基线：

```text
197 passed
```

后续每次结构迁移都必须保持或有意识更新该测试基线，不允许在重构时顺便改变 Isaac
运动表现、HuNav 参数或 episode 语义。

### 2.1 S0/S1 执行记录

执行日期：2026-07-30

```text
S0 source checkpoint: 253ea2607d2271238d9a9a83897c5c920dd16e0b
S0 test baseline:      197 passed
S1 test baseline:      207 passed
Build:                 toilet_benchmark finished
Install import:        domain/task and episodes/schema passed
```

执行 S0 时 worktree 已包含 HuNav、配置、文档和测试的未提交修改，因此没有把这些用户
修改混入一个基线 commit。checkpoint 由上述 HEAD、`git status` 审计和测试结果共同
定义；后续提交应只暂存本阶段文件和兼容接线。

S1 已完成：

- 新增 simulator-neutral 的 `AgentSnapshot`、task phase、termination reason 和事件契约；
- 将既有 `MotionCommand`、`DirectedAgentState` 改为兼容重导出；
- director 和 manual collector 共用 canonical JSON event codec；
- 新增版本化 `EpisodeSpec`，并提供从现有 manual scenario 转换的 adapter；
- 未修改 HuNav 参数、Isaac 控制、路径规划或 episode 运行时语义。

S1 暂不包含 validator、manifest 和 split；这些属于 S2-B。

冻结的主操作面：

| 类型 | 当前接口 |
| --- | --- |
| Isaac asset/robot spawn | `arena_scene_profile.py --profile scripts/profiles/shenxinfu_841837.yaml spawn --phase physx_diff_contact` |
| Isaac bridge | `arena_scene_profile.py ... bridge physx_diff_contact` |
| Interactive director | `ros2 run toilet_benchmark toilet_director_node --motion-backend hunav --initial-agents N` |
| Dataset collector | `ros2 run toilet_benchmark manual_collection_node --config .../manual_collection.yaml` |
| Isaac services | `/isaac/spawn_pedestrian`、`/isaac/move_pedestrians` |
| Director event/intent | `/toilet_benchmark/director_status`，JSON over `std_msgs/String` |
| HuNav diagnostics | `/toilet_benchmark/hunav/route`、`/toilet_benchmark/hunav/markers` |

S1 不修改上述 CLI、service 或 topic 名称。

### 2.2 S2-A/S2-B 执行记录

执行日期：2026-07-30

S2-A：

- 新增 `motion/hunav/behavior.py`，统一六类原生 HuNav behavior 及完整参数映射；
- 新增 `motion/hunav/runtime.py`，隔离每名 agent 的 generation、goal、phase、hold、
  route 和 external motion 状态；
- 新增 `motion/hunav/client.py`，集中管理 compute/reset service endpoint；
- `MotionCommand.behavior` 由 `hunav_profile.<name>.behavior_type` 显式传入 backend；
- 旧 backend 字段和 service client alias 保留，避免一次性重写热路径。

S2-B：

- 新增结构化 `EpisodeSpec` validator；
- 新增确定性 episode/manifest hash 和原子 JSON 保存；
- 新增固定 seed、无 episode 重叠的 split 分配；
- `assets` 保持资产/config hash 语义，semantic goal 通过显式 ID 集合校验；
- 仓库现有五个 manual scenario 已通过确定性批量转换和校验测试。

```text
S2 unit-test baseline: 223 passed
S3 prep test baseline: 230 passed
S3 prep build:         toilet_benchmark finished
Live Isaac/HuNav run:  single-agent and dual-agent smoke passed
```

Live smoke 使用冻结主操作面中的 `physx_diff_contact` bridge，并分别执行
`--initial-agents 1` 与 `--initial-agents 2`。用户确认结构提取后的可视运动表现与修改
前一致；该结论只覆盖基础流程回归，不替代后续 behavior/BT 统计验证。

### 2.3 S3 准备工具

- HuNav 周期诊断现在同时输出 native behavior profile/configuration/参数和自研
  reaction state/reaction，用于识别双重行为权威；
- 新增 `toilet_manifest generate` 和 `toilet_manifest validate`；
- manifest bundle 保存独立 episode payload、relative path、episode hash 和 manifest
  content hash；
- 相同配置与 seed 的 manifest 和 episode payload 受字节级一致性测试保护；
- 新增 `hunav_interactive_smoke`，用服务 readiness 和机器人 settling 标志自动执行
  `bridge -> spawn -> director -> cleanup`，并按运行目录归档诊断；
- 本阶段只增加可观测性与工具，没有关闭任何旧 reaction/avoidance。

### 2.4 S3-A 首轮原生 Behavior 对照

2026-07-30 已使用固定 `arrival.seed=12345` 自动运行
`regular/surprised/scared`。结论如下：

- 原生-only regular 在关闭旧 reaction 和 `regular_avoidance` 后完成单行人流程；
- surprised 的原生 active state 可观察，但定时器结束后会退回 regular；
- scared 的零秒 timer 配置已修复；上游实现会在响应前清零 behavior state，因此不能
  使用同一 active-state 指标判断 scared 是否执行；
- 当前只满足一类策略的单行人关闭实验，不满足删除门槛。

完整证据见 `hunav_behavior_characterization_20260730_cn.md`。

## 3. 设计原则

1. 先锁定行为，再移动代码。
2. 每次只迁移一个职责簇。
3. 旧 import、CLI 和 ROS topic 通过 facade 保持兼容。
4. 新代码优先依赖数据契约，不依赖 ROS Node 具体实现。
5. Director 只管理任务语义，不管理 HuNav 局部策略。
6. Motion backend 不管理 episode、split、录制或 evaluator。
7. Track runner 不直接控制厕所资源，统一通过 task/director port。
8. Evaluator 只读取 ground truth/event，不反向修改仿真。
9. 删除必须发生在替代实现已接通并完成回归之后。
10. 不新增第三套行人路径或碰撞控制逻辑。

## 4. 目标结构

```text
toilet_benchmark/
├── domain/
│   ├── __init__.py
│   ├── agent.py
│   ├── events.py
│   ├── task.py
│   ├── world.py
│   └── intent.py
│
├── scenario/
│   ├── runtime.py
│   ├── executive.py
│   ├── smart_objects.py
│   ├── templates.py
│   └── triggers.py
│
├── motion/
│   ├── __init__.py
│   ├── base.py
│   ├── safety.py
│   └── hunav/
│       ├── __init__.py
│       ├── backend.py
│       ├── client.py
│       ├── behavior.py
│       ├── runtime.py
│       ├── route_tracker.py
│       ├── safety.py
│       └── diagnostics.py
│
├── routing/
│   ├── __init__.py
│   ├── base.py
│   ├── grid_search.py
│   ├── walkable_map.py
│   └── voxel_legacy.py
│
├── embodiment/
│   ├── base.py
│   ├── isaac_animgraph.py
│   └── smpl_h.py
│
├── episodes/
│   ├── __init__.py
│   ├── schema.py
│   ├── validator.py
│   ├── manifest.py
│   └── splits.py
│
├── tracks/
│   ├── __init__.py
│   ├── base.py
│   ├── replay.py
│   ├── interactive.py
│   └── dataset.py
│
├── evaluation/
│   ├── __init__.py
│   ├── metrics.py
│   ├── evaluator.py
│   └── aggregate.py
│
├── tools/
│   ├── hunav_phase0_smoke.py
│   ├── hunav_isaac_mirror.py
│   └── walkable_map_publisher.py
│
├── toilet_director_node.py
└── manual_collection_node.py
```

这是一条渐进目标，不要求一次性创建所有目录。没有实现内容的空目录不应提前创建。

`scenario/` 只拥有任务意图、Smart Object 和事件生命周期；`motion/` 只产生局部运动；
`routing/` 只产生全局路线；`embodiment/` 只把 MotionCommand 映射到角色。四者不得直接
互相写内部状态，统一通过冻结端口和不可变快照通信。

## 5. 共享数据契约

两条开发线开始前，先定义四个最小契约。

### 5.1 AgentSnapshot

表示仿真中某一时刻的行人或机器人状态：

```python
@dataclass(frozen=True)
class AgentSnapshot:
    agent_id: str
    timestamp_sec: float
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    wz: float
    radius_m: float
    source: str
```

要求：

- 不包含 ROS message；
- 不包含 HuNav message；
- frame 和时间语义固定；
- ROS/HuNav/Replay 分别通过 adapter 转换。

### 5.2 MotionCommand

现有 `MotionCommand` 作为初始兼容来源，逐步补齐：

```text
agent_id
phase
route
semantic_goal
target_yaw
desired_speed
generation
mode
```

Director 只发布意图，motion backend 决定如何执行，不应把 HuNav 内部 avoidance
waypoint 写回 task phase。

### 5.3 BenchmarkEvent

替代各处手写 JSON key：

```python
@dataclass(frozen=True)
class BenchmarkEvent:
    event_type: str
    timestamp_sec: float
    episode_id: str | None
    agent_id: str | None
    phase: str | None
    generation: int | None
    payload: Mapping[str, Any]
```

第一阶段仍通过 `std_msgs/String` 发布 JSON，但序列化和解析必须由同一模块负责。未来可
平滑迁移为 ROS msg，不要求本阶段新增接口包。

### 5.4 EpisodeSpec

正式 episode schema 的内存模型：

```text
schema_version
benchmark_version
episode_id
scene_id
task_type
track
seed
robot
pedestrians
termination
difficulty
assets
```

人工数采的 `ScenarioConfig` 通过 adapter 转换为 `EpisodeSpec`，不直接扩展成所有
Evaluation Track 配置的总模型。

## 6. 模块职责

### 6.1 domain

只放 simulator-neutral 数据结构和枚举：

- agent snapshot；
- motion intent；
- task phase；
- resource/semantic goal ID；
- benchmark event；
- termination reason。

禁止：

- `rclpy.Node`；
- ROS client/subscription；
- HuNav/Isaac message；
- 文件系统写入；
- planner 实现。

### 6.2 scenario

只处理目的性任务和事件：

- `runtime.py`：按统一 world snapshot 推进 episode 事件；
- `executive.py`：维护每名 agent 的 task state 和 reaction state；
- `smart_objects.py`：query、claim、occupy、release 和 queue promotion；
- `templates.py`：组合 EnterUseExit、DoorwayConflict 等事件节点；
- `triggers.py`：时间、区域、资源和前序事件触发条件。

禁止直接调用 HuNav、规划网格、Isaac prim API 或逐帧写行人位姿。

### 6.3 motion

负责根据 MotionIntent、RoutePlan 和 WorldSnapshot 产生局部运动结果：

- `base.py`：`MotionBackend` protocol；
- `safety.py`：只约束非法穿透，不搜索绕行方向；
- `hunav/backend.py`：组合各 HuNav 子模块；
- `hunav/client.py`：服务调用和请求/响应；
- `hunav/behavior.py`：原生行为类型和 BT 配置；
- `hunav/runtime.py`：每名 agent 的 shadow、generation、goal、hold；
- `hunav/route_tracker.py`：lookahead、splice、progress；
- `hunav/safety.py`：最后一层几何合法性；
- `hunav/diagnostics.py`：日志、marker、trace。

`backend.py` 应逐步缩减为生命周期协调器，不再包含大量几何和评分函数。
它不得占用资源、改变 task state 或直接操作 Isaac prim。

### 6.4 routing

只处理静态/动态几何上的全局路线：

- `base.py`：`RouteRequest`、`RouteProvider`；
- `grid_search.py`：与输入格式无关的 A*/Theta*；
- `walkable_map.py`：正式 walkable map loader、planner、publisher adapter；
- `voxel_legacy.py`：旧 voxel loader/provider。

当前 `voxel_path_planner.py` 不能直接删除，因为 walkable-map planner 复用了其搜索
内核。应先把通用搜索抽到 `grid_search.py`，再隔离 voxel loader。

### 6.5 embodiment

负责把 `MotionCommand` 转换为角色运行时操作：

- `base.py`：spawn、apply、retire 和状态查询协议；
- `isaac_animgraph.py`：当前 Isaac external-motion/AnimGraph 兼容实现；
- `smpl_h.py`：后续 SMPL-H root motion、动作 phase 和骨骼更新实现。

表现层不能修改 route、task state、资源状态或 episode 成败。角色碰撞 proxy 和视觉扫掠
包络必须版本化，并通过 truth/diagnostics 输出给 evaluator。

### 6.6 episodes

负责：

- schema；
- YAML/JSON load/dump；
- 静态字段校验；
- 起终点/目标引用校验；
- split manifest；
- hash 和版本。

不负责启动 ROS、reset 仿真、录 bag 或计算 policy action。

### 6.7 tracks

统一生命周期接口：

```python
class TrackRunner(Protocol):
    def prepare(self, episode: EpisodeSpec) -> None: ...
    def reset(self) -> None: ...
    def step(self, action) -> TrackStep: ...
    def close(self) -> None: ...
```

- `replay.py`：消费冻结行人轨迹；
- `interactive.py`：连接 director + HuNav；
- `dataset.py`：连接人工控制和 EpisodeRecorder。

### 6.8 evaluation

只消费 step truth/event：

- 每步更新指标；
- episode 终态计算；
- JSONL 输出；
- split/task/difficulty 聚合；
- 结果完整性检查。

Evaluator 不能控制机器人或行人。

## 7. 现有文件迁移映射

| 当前文件 | 迁移方向 | 本阶段动作 |
| --- | --- | --- |
| `motion_backend.py` | `motion/base.py` + `embodiment/isaac_animgraph.py` | 保留 facade |
| `hunav_motion_backend.py` | `motion/hunav/*` | 分职责渐进抽取 |
| `robot_reaction.py` | 原生 `motion/hunav/behavior.py` 替代 | 暂保留 |
| `interaction_policy.py` | native behavior/hard safety 替代 | 暂保留 |
| `route_provider.py` | `routing/base.py` | 保留 re-export |
| `voxel_path_planner.py` | `routing/grid_search.py` + `voxel_legacy.py` | 先拆搜索与 loader |
| `walkable_map.py` | `routing/walkable_map.py` + tools publisher | 暂不搬 |
| `walkable_map_planner.py` | `routing/walkable_map.py` | 后续合并 |
| `hunav_adapter.py` | `domain/agent.py` 或 spawn model | 后续重命名 |
| `collection_scenarios.py` | Dataset config adapter | 不作为正式 schema |
| `episode_recorder.py` | `episodes/manifest.py` + Dataset writer | 保留 facade |
| `manual_collection_node.py` | `tracks/dataset.py` 的 ROS facade | 暂不拆 Node |
| `resource_manager.py` | `scenario/smart_objects.py` | 先以 adapter 保持旧 API |
| `toilet_director_node.py` | `scenario/*` + ROS facade | E1 渐进拆分，保持 CLI/topic |
| `hunav_phase0_*` | `tools/` 或 tests support | 保留 |
| `hunav_isaac_mirror*` | `tools/` | 稳定前保留 |

## 8. 删除门槛

### 8.1 robot_reaction.py

只有同时满足以下条件才删除：

- 六类 HuNav behavior 已有显式 YAML 映射；
- 固定/随机行为均由 seed 控制；
- `regular/surprised/scared` 至少完成单元和 Isaac 回归；
- 不再有生产 import；
- 旧 `robot_proximity_reaction` 配置有迁移说明。

删除范围：

- `robot_reaction.py`；
- 对应单元测试；
- `regular_avoidance` 中与 HuNav BT 重复的策略；
- 已废弃 YAML 字段。

### 8.2 interaction_policy.py

只有 robot-yield/stall 判断全部被原生 behavior 或统一 safety contract 替代后删除。
不能因文件很小就先删，否则 director 的恢复计时可能再次把正常让行误判为 stall。

### 8.3 mirror

满足以下条件后删除：

- Interactive Track 连续回归稳定；
- 新 HuNav adapter 的状态、速度、yaw 和终态诊断覆盖 mirror 现有用途；
- 不再需要旧 IsaacPeople vs HuNav 对照；
- 至少保留一份最终对照报告。

删除：

- `hunav_isaac_mirror.py`；
- `hunav_isaac_mirror_core.py`；
- mirror config、script、tests 和操作文档。

### 8.4 voxel legacy

满足以下条件后删除：

- benchmark 正式 runner 不再支持旧 voxel backend；
- IsaacPeople compatibility 明确停止；
- walkable-map 只依赖通用 `grid_search`；
- voxel loader/provider 无生产 import；
- Replay/Interactive Track 均通过 walkable map 或冻结轨迹运行。

保留：

- 通用 A*/Theta*；
- grid inflation；
- line-of-sight/supercover；
- 通用路径简化。

### 8.5 IsaacPeopleBackend

不因 HuNav 成为主线立即删除。只有：

- Replay Track 不依赖它；
- Interactive Track 已稳定；
- 回滚窗口结束；
- README/CLI 明确移除兼容模式；
- IsaacPeople 专属测试完成归档；

才删除实现和 CLI choice。

### 8.6 Phase 0

Phase 0 是 HuNav 版本升级、BT 修改和参数回归的快速测试工具，长期保留。可以移动到
`tools/`，但不应作为“实验完成后即可删除”的临时代码。

## 9. 并行开发边界

### Lane A：HuNav Interactive

主要写入：

```text
domain/agent.py
domain/task.py
motion/hunav/
config/toilet_benchmark.yaml 的 behavior 部分
test/motion/hunav/
```

目标：

- 接通原生 behavior/BT；
- 统一 agent runtime；
- 减少重复 reaction/avoidance；
- 多人 roster 不跳变；
- safety 降级为薄兜底。

### Lane B：Benchmark/Replay

主要写入：

```text
domain/events.py
episodes/
tracks/replay.py
tracks/dataset.py
evaluation/
config/episodes/
test/episodes/
test/tracks/
test/evaluation/
```

目标：

- EpisodeSpec 和 validator；
- 固定 manifest/split；
- Replay trajectory schema；
- standard observation/action contract；
- evaluator 和结果格式。

### 共享修改规则

以下文件属于共享冲突区：

```text
toilet_director_node.py
manual_collection_node.py
README.md
package.xml
CMakeLists.txt
setup.py
```

并行阶段不允许两条 lane 同时重写这些文件。需要接线时使用小型 adapter commit，
并在合入前运行全量测试。

## 10. 分阶段实施

### S0：冻结当前基线

状态：完成（2026-07-30）

动作：

1. 保存当前 git 状态；
2. 运行全量单元测试；
3. 记录主 CLI、配置和 ROS topic；
4. 为未覆盖的关键行为补 characterization tests；
5. 形成可回滚 checkpoint。

验收：

- `197 passed` 或更新后的明确基线；
- 不改变运行表现；
- dirty worktree 中用户修改被单独识别。

### S1：共享契约

状态：完成（2026-07-30）

新增：

```text
domain/agent.py
domain/events.py
domain/task.py
episodes/schema.py
```

动作：

- 从现有 dataclass 和 JSON payload 提取；
- 增加序列化 round-trip 测试；
- 旧模块 re-export，旧 import 不失效；
- 不接入 Isaac 控制。

验收：

- 旧测试全过；
- 新契约不依赖 ROS/HuNav；
- 同一 event 只有一个 canonical serializer。

### S2-A：HuNav behavior/runtime

状态：完成（2026-07-30）

新增：

```text
motion/hunav/behavior.py
motion/hunav/runtime.py
motion/hunav/client.py
```

动作：

- 映射六类 `AgentBehavior`；
- 解析 `configuration/duration/once/vel/dist/force factors`；
- 每名 agent 保存稳定 runtime 和 BT generation；
- backend 通过新模块构造 request；
- 先不删除自研 reaction，支持 shadow 对比。

验收：

- manager 日志能加载对应 BT；
- fixed seed 行为类型和参数一致；
- roster 更新不重置无关 agent；
- Phase 0 和 unit tests 通过。

### S2-B：Episode schema/validator

状态：完成（2026-07-30）

新增：

```text
episodes/validator.py
episodes/manifest.py
episodes/splits.py
```

动作：

- 将 `manual_collection.yaml` 场景转换为 EpisodeSpec；
- 校验 ID、pose、资源、人数、目标和 hash；
- 生成固定 manifest；
- 先建立 seen/validation/test_behavior/test_density。

验收：

- 相同 seed 生成字节级一致 manifest；
- 无效 episode 有明确错误码；
- 不启动 Isaac 也能运行 schema/validator 测试。

### S3-A：删除重复 HuNav 策略

动作：

- 对比原生 BT 与旧 reaction 输出；
- 逐个关闭旧 yielding/impatient/regular avoidance；
- 将 hard safety 收缩为碰撞/重叠兜底；
- 每关闭一类逻辑运行单人、多人、门口和机器人阻挡回归。

验收：

- 无双重行为权威；
- 不再出现因两层策略竞争造成的回跳；
- 删除门槛满足后才删除文件和 YAML。

### S3-B：Replay Track

动作：

- 定义轨迹格式和时间基准；
- 从合格人工/HuNav episode 导出候选轨迹；
- 实现 replay actor；
- 为 live replay 提供可选的 `--spawn-character` 行人生成入口；
- 禁止 policy 读取未来轨迹；
- 接入统一 BenchmarkEvent。

验收：

- 同轨迹重复运行误差在容差内；
- replay 不调用实时 HuNav；
- future trajectory 只在 evaluator/oracle 可见；
- 碰撞按正式 policy 终止。

### S4：标准接口和 evaluator

动作：

- 冻结 LiDAR observation；
- 冻结 `[vx, vy,wz]` action；
- 实现 TrackRunner；
- 实现 v0.1 metrics；
- 输出 episode JSONL 和聚合结果。

验收：

- Replay/Interactive 使用同一 policy interface；
- ground truth 不进入 observation；
- metric 有手算测试；
- 丢失/重复 episode 会被 evaluator 拒绝。

### S5：清理和收口

动作：

- 按删除门槛清除旧策略、mirror 或 voxel legacy；
- 整理 CLI；
- 审计 CMake scripts 与 `setup.py` entry_points；
- 更新 package description/version/license；
- 删除兼容 facade 前提供迁移说明。

验收：

- 无死 import、无未使用配置；
- 文档只描述当前可运行路径；
- 全量单元、构建和 Isaac smoke test 通过；
- git 历史中每类删除都有独立可回滚 commit。

## 11. 测试策略

结构迁移最低验证：

```bash
source /home/stardust/resources/arena_ws/install/setup.zsh
PYTHONDONTWRITEBYTECODE=1 \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q -p no:cacheprovider toilet_benchmark/test
```

每个阶段还应运行：

- `colcon build --packages-select toilet_benchmark --symlink-install`；
- Phase 0 固定 seed；
- 单行人 HuNav takeover；
- 双行人 roster/退出；
- 同一 bridge 连续 episode；
- Replay deterministic trace；
- schema/validator 纯 Python 测试。

结构迁移和行为修改不得放在同一个 commit。若必须同时调整接口，commit 至少拆成：

1. characterization tests；
2. pure move/extraction；
3. caller switch；
4. old code deletion。

## 12. 下一步

立即执行顺序：

1. 已完成固定 seed 静态阻挡，并新增通过 `/cmd_vel_gamepad_diff` 的确定性动态 crossing；
2. 已增加独立 HuNav behavior transition JSONL，并修复 scared state 被提前清零；
3. 已接入只读动画骨骼外包络与静态 PhysX overlap 指标，下一次固定矩阵采集其基线；
4. 修复 surprised/scared 的可见性保持语义后，运行同脚本原生-only 对照；
5. 已固化 `crossing_clear`/`crossing_conflict` profile 和顺序矩阵运行器；下一步实际运行
   固定 seed 的三行为、单/双行人矩阵；每个 live 子任务先确认当前 bridge 的
   `Simulation App Startup Complete`，再检查服务并进入 spawn；
6. 通过后默认关闭第一类重复 `regular_avoidance`，再评估删除；
7. Replay bundle validator、封存和后端中立单行人 actor foundation 已完成；
8. 已接入 Isaac `external_motion` adapter，并从合格 episode 导出首条冻结轨迹；下一步
   已完成首次 live replay：空间横向 RMSE 0.078 m，但时间对齐 RMSE 1.031 m，需增加
   Replay 专用追赶、终态确认和 validator 阈值后再冻结回放容差。
9. Replay 专用单调局部追赶和终态 pose/yaw 稳定握手已实现并通过纯 Python 回归；
   下一步是在 Isaac 可稳定启动后复测 live 时间 RMSE 和终点误差。
10. HuNav 原生 behavior 返回状态已与本地 reaction 解耦并保留；下一次
    `surprised/scared + crossing_conflict` live smoke 必须观察真实 active 样本和动作，
    不能只以 hard-safety contact 作为通过证据。

### 2026-07-30 S3 基线门槛结果

- Replay 已输出实际下发时间轴上的全程 RMSE、路径横向误差和终点误差；冻结样本的
  time-aligned RMSE 为 `0.0159 m`，终点位置误差为 `0.00684 m`。
- 单行人 `regular/surprised/scared x clear/conflict` 固定 seed 矩阵 6/6 完成，
  stall/recovery exhausted 均为 0。
- 已进入双行人 characterization，而不是直接改多人策略；首轮固定
  `urinal_1,urinal_5` 完成，代理重叠、路线倒退、共同冻结和 behavior 错配均为 0。
- smoke summary 已提供 per-agent 和 pair 指标，并已接入独立的高频 raw pose 与视觉
  骨骼外包络 recorder；它在复用 bridge 时也不依赖 bridge 的落盘环境变量。

### 2026-07-31 双行人高频门槛结果

- 固定 `regular/surprised/scared x clear/conflict`、seed `12345`、目标
  `urinal_1,urinal_5` 的 6 轮流程全部完成，stall/recovery exhausted 均为 0；
- raw pose 实测频率为 `9.29-9.50 Hz`，六轮 peer activation jump excess 均为 0；
- 高频数据推翻了低频“完全无重叠”的判断：`regular-clear` 最小中心距 `0.403 m`，
  `<0.60 m` 累计约 `2.13 s`；
- 六轮均记录到视觉骨骼外包络与静态几何 overlap，主要位于大门和墙，少量位于隔板
  与小便池；
- surprised/scared conflict 仅有 1/3 个原生 active diagnostics 样本，BT 状态保持
  仍未达到稳定行为验收标准。

决策：Replay 单行人基线继续冻结；Interactive Lane A 暂不扩展到更多人数或正式
benchmark split，先修复双人中心重叠、静态视觉穿模和 behavior active 保持，再用
至少 3 个固定 seed 重跑同一矩阵。

### 2026-07-31 不可通行走廊边界

- `interactive_smoke_runner` 新增 `occupied_passage` 场景契约，固定双人目标、机器人
  阻塞/撤离 marker、激活间隔和服务时间；
- `HuNavMotionBackend` 增加固定行人封闭走廊检测，只有左右候选同时不可用时才进入
  `YIELDING_TO_PEDESTRIAN`；
- 让行状态复用 external-motion freeze 与 yaw hold，释放后重建剩余全局路线；
- raw recorder summary 增加让行窗口专用位移、速度和 yaw 稳定性指标；
- seed `12345` 的 Isaac 回归两人均完成，且让行区间位置跨度 `0.028 m`、yaw 漂移
  `0.0029 rad`。

该边界解决的是“不可通行却持续挤压”的错误，不应扩展成通用多人硬冻结。后续结构上
继续把可通行会车交给 HuNav 和走廊选择器，并用独立 profile 验证，避免一个场景同时
承担“应等待”和“应绕行”两种相反的验收语义。

### 2026-07-31 可通行走廊反例

- 新增 `passable_passage_u1/u2`，固定 agent 01 占用 `urinal_3`、机器人位于原点，
  agent 02 分别前往 `urinal_1/2`；
- 两轮流程均完成，但 agent 02 都没有锁定绕行侧，而是被固定行人封闭判定冻结；
- `urinal_2` 轮对 `Partition_0001` 累计出现 151 次视觉外包络 overlap，且全局最长
  连续 overlap 为 149 帧；
- 这证明 `pedestrian_yield` 不能继续独立扩展成另一套局部规划器。

该结构切片已完成：

- robot/fixed/moving pedestrian 共用候选生成和静态/动态评分；
- 局部直线候选失败时复用 walkable Theta*，不新增另一套栅格规划器；
- 可行候选锁定单侧并重接全局路线，无路候选锁定让行；
- 无路等待不重复同步搜索，避免阻塞 ROS executor 和制造 stale pose；
- `PEDESTRIAN_AVOIDANCE` stall 会带机器人和其他行人动态占据重建全局路线；
- HuNav 继续负责连续社会运动，hard safety 仍是最后一层。

固定 seed 的 `passable_passage_u1/u2` 均完成。`u1` 实现单侧绕行且不换边；`u2`
实现等待者保持、离开者动态重建和后续释放，raw 行人中心距没有低于 `0.60 m`。

下一结构切片不再增加局部状态机，而是：

1. 将 visual-envelope overlap 帧按门、墙、隔板分类回放，收敛骨骼外包络与 walkable
   map 的几何不一致；
2. 用 3 个固定 seed 和移动机器人重跑两个 profile，确认选边与动态预测稳定；
3. 收敛让行 yaw span 和可恢复 stall 数，再回到 surprised/scared BT 保持语义；
4. 继续保持 director 不感知具体位置特判，避免事件层重新承担运动规划。

### 2026-07-31 连续几何硬安全边界

静态 hard safety 已从“root 圆端点检查 + 左右投影搜索”替换为独立的
`motion/swept_envelope.py`：

- 使用经动画骨骼观测标定的有向胶囊；
- 连续检查平移和旋转扫掠，而非只检查下一 root 点；
- 碰撞前二分裁剪；起点已在边界内时只允许净空单调增加的原命令；
- 不产生绕行 waypoint，不选择通行侧，不承担脱困规划；
- 全局 Theta*/动态走廊选择器仍是唯一的几何选路层，HuNav 仍是连续社会运动层。

自动 Interactive smoke 已将视觉骨骼静态 overlap 提升为硬失败条件。角色穿模不再
以“流程最终完成”或“根节点未进入 occupied cell”掩盖。

## 13. 2026-08-01 接口冻结与后续长目标

E0 已完成文档级接口冻结：

- 新增 `pedestrian_ecosystem_architecture_cn.md`，冻结唯一控制权、Smart Object、
  Scenario Runtime、Agent Executive、LocalMotion 和 Embodiment 端口；
- 新增 `motion_backend_evaluation_cn.md`，冻结 HuNav、ORCA/HRVO、Replay、Isaac
  AnimGraph 和 SMPL-H 的能力边界与微场景矩阵；
- 现有 `AgentSnapshot`、`BenchmarkEvent`、`MotionCommand`、`EpisodeSpec`、CLI、ROS
  service/topic 保持兼容；
- 本阶段没有拆代码或修改行人运动行为。

后续长目标采用 E1-E5，而不是继续在 S3-A 热路径叠加策略：

1. **E1 事件内核**：提取 SmartObjectRegistry、AgentExecutive、ScenarioRuntime；
2. **E2 运动边界**：提取 GlobalRouter、LocalMotionBackend、GeometrySafety；
3. **E3 表现层**：提取 Isaac adapter，并以同一 MotionCommand 接入 SMPL-H A/B；
4. **E4 事件库**：实现五类冻结模板、受控随机生成和连续 reset；
5. **E5 发布闭环**：统一 runner、evaluator、baseline 和 benchmark card。

第一个代码切片仅实现 E1 的纯 Python 领域对象和 golden tests。它必须通过 adapter 复现
现有 `EnterUseExit`，不得同时调 HuNav、路径、碰撞半径或动画。旧 director 分支在新旧
event trace shadow 对比一致前保留。

### 13.1 E1 第一切片结果

已新增且通过纯 Python 回归：

- `domain/intent.py`：`TaskDirective`，只表达任务意图，不包含路径或速度；
- `domain/world.py`：`WorldSnapshot` 与语义级 `AgentTaskFeedback`；
- `scenario/smart_objects.py`：Smart Object 注册、槽位、容量、FIFO 队列和原子释放晋升；
- `scenario/executive.py`：每 agent 的 `EnterUseExit` 状态推进；
- `scenario/runtime.py`：确定性多 agent tick、事件聚合和完成判定；
- `test_smart_object_registry.py`、`test_scenario_runtime.py`：资源不变量与双行人 golden
  event trace。

后续切片已建立 `DirectorScenarioAdapter` 并接入 director：

- 配置面新增 `scenario_runtime.mode: legacy|shadow|takeover`；
- shadow 比较阶段、owner 和 FIFO queue，差异日志按 key 去重；
- takeover 下 `SmartObjectRegistry` 是唯一资源写入方，`ResourceManager` 仅是旧 motion
  代码读取的兼容镜像；
- 到达、服务完成和异常退休都通过 `AgentTaskFeedback` 推进 runtime；
- owner 异常退休时，同一 tick 内完成队首晋升和指令刷新，避免资源已晋升但行人仍等待；
- activation、portal、route、stall recovery、despawn 和 `MotionCommand` 仍属于 director
  facade，因此没有修改 HuNav、路径、碰撞或动画。

E1 代码实现已完成，默认仍是 `shadow`。2026-08-01 的固定 seed Isaac 验证结果如下：

- 单行人 `shadow` 与显式 `takeover` 均完成 `EnterUseExit`，各有一次 urinal/exit arrival，
  无 stall、无 recovery exhausted、无 E1 mismatch；
- 双行人、不同目标小便池的 `shadow` 完成两次 urinal/exit arrival，
  `scenario_shadow_mismatch_events=0`，无激活跳变、无行人重叠；
- 双行人、同一小便池能正确产生 FIFO 晋升，但旧 HuNav queue motion 在占用者附近进入
  corridor freeze，最终 300 秒超时；这是 E2 运动层门槛，不是 Smart Object owner/queue
  漂移；
- smoke 的非零进程码来自现有 visual-envelope 门框/墙体重叠 gate，不能解释为 E1 失败。

因此 E1 语义契约已通过 live shadow 验证，但 YAML 默认值暂不切到 `takeover`。下一步先在
E2 消除同资源 queue/exit 互锁并通过视觉包络 gate，再执行默认值切换；命令行仍可用
`--scenario-runtime takeover` 显式验证，也可用 `legacy` 一键回退。

### 13.2 E2-A 运动边界结果

截至 2026-08-01，E2-A 已完成第一组可执行边界，而不是只新增未使用的 Protocol：

- `motion/contracts.py` 冻结不可变 `RoutePlan`、`BehaviorPolicyRequest/Decision`、
  `LocalMotionRequest/Result` 和 `GeometrySafetyRequest/Result`；
- `motion/ports.py` 冻结 `GlobalRouterPort`、`BehaviorPolicyPort`、`LocalMotionPort` 和
  `GeometrySafetyPort`；
- `VoxelRouteProvider` 与 `WalkableMapRouteProvider` 真实返回带 planner/map provenance 的
  `RoutePlan`，director 在唯一路由调用点转换为兼容 path points；
- `SweptEnvelopeGeometrySafety` 成为 HuNav 静态扫掠裁剪的生产端口，仍调用原有
  `SweptEnvelope`，没有修改采样、裁剪比例或 overlap recovery 数值；
- 保留 `MotionBackend` 的 `register_agents`、direct-pose、settled handshake、
  `allows_director_stall_recovery` 和 Isaac external-motion 生命周期，不把观察用途的
  `HunavAdapter`/motion-intent 错当控制端口。

验证结果：纯 Python 全套 `344 passed`；固定 seed 单行人 Isaac shadow 完成
`EnterUseExit`，director `exit_code=0`，urinal/exit arrival 各 1 次，无 stall、无 recovery
exhausted、无 E1 mismatch。smoke 外层仍因既有门框 visual-envelope overlap 返回非零，
该问题不属于 E2-A 接口回归。

E2-B 不再继续扩大 `HuNavMotionBackend`。下一切片在 HuNav 内部提取实际
`BehaviorPolicyPort` 与 `LocalMotionPort` 调用点，并增加一个确定性 ORCA/HRVO 候选后端；
先通过纯 2D 微场景矩阵，再接 Isaac external-motion。HuNav 在 A/B gate 通过前保留为可回退
实现。

### 13.3 E2-B 局部运动候选与 shadow

截至 2026-08-01，E2-B 的第一阶段已经落地：

- `motion/behavior_policy.py` 提供 simulator-neutral 的上下文策略，显式区分
  `WALKING/YIELDING/PASSING_LEFT/PASSING_RIGHT/FOLLOWING/WAITING/GROUPING`；
- `motion/sampled_rvo.py` 提供无第三方依赖、确定性的 sampled velocity-obstacle 基线；
  它通过固定速度/方向候选和有限时间最近距离检查选择局部速度；
- `preferred_side=+1` 固定表示相对当前路线方向的左侧，`-1` 表示右侧，避免世界坐标和
  行进坐标混用；
- 纯 2D 测试覆盖无障碍、迎面、交叉、无可行通道、显式 hold、确定性重复和 50 tick
  连续会车；连续会车测试同时冻结“通过且不重叠”的要求；
- HuNav 原始输出与 hard safety 之间新增默认关闭的 `local_motion_shadow` 调用点。启用后
  candidate 读取相同 agent/peer/robot/lookahead 快照，只记录可行性和速度差，绝不修改
  HuNav 发往 Isaac 的命令。

配置入口位于 `motion_backend.hunav.local_motion_shadow`。当前默认 `enabled: false`，因为该
实现是用于验证端口和几何基线的 sampled velocity-obstacle，并不是严格线性规划 ORCA，
也尚未通过 doorway、narrow-passable、occupied-goal 和 queue-release 的 Isaac gate。禁止
仅凭开放区域会车测试将其切为默认控制权。

固定 seed Isaac shadow characterization 已完成第一轮：

- 单人、无机器人：`54/54` 个 candidate 样本可行，速度差均值 `0.493 m/s`；
- 双人、不同目标：`96/96` 个 candidate 样本可行，原始轨迹最小行人间距
  `1.003 m`，无重叠、无激活跳变；
- 单人 crossing：`60/65` 个 candidate 样本可行，可行率 `92.3%`，但只有 2 个样本真正
  选择了避让速度；机器人接触窗口内连续 5 个样本不可行，随后仍由既有 HuNav 与 hard
  safety 完成任务。

因此 E2-B **未达到 takeover 门槛**。当前 sampled_rvo 在开放空间可作为确定性几何对照，
但在紧接触 crossing 中仍表现为“保持首选速度，直到候选集全部失效”。下一切片优先改进
候选生成而非放宽安全距离：加入相对速度障碍边界附近的自适应候选，并在评分前查询
`GeometrySafetyPort` 的静态可行性。只有 crossing、narrow-passable 和 doorway gate 均通过后，
才增加 `local_motion_backend: hunav|sampled_rvo` takeover 开关。

重复 bridge 的视觉包络汇总已按 director 实际激活 agent 过滤，停车池中的预生成角色不再
污染 overlap 指标。GeometrySafety 始终保留为最终非法穿透约束，不负责选边或侧向脱困。

### 13.4 E2-B 独立 takeover 骨架

E2-B 第二阶段不再修改 HuNav 热路径，已新增独立控制链：

```text
director RoutePlan
  -> ContextualBehaviorPolicy
  -> SampledRvoLocalMotion
  -> SweptEnvelopeGeometrySafety
  -> IsaacPeopleBackend external-motion
```

- `motion/pipeline.py` 组合 `BehaviorPolicyPort` 和 `LocalMotionPort`，不依赖 ROS、HuNav 或
  Isaac；
- `local_motion_backend.py` 实现现有 `MotionBackend` 协议，复用 director 路径、实时
  agent/robot 快照和 Isaac external-motion adapter；
- director 新增显式 `--motion-backend local_motion`，并把 portal、walkable route、final
  alignment 和 settled handshake 的能力判断从 `hunav` 字符串扩展为连续运动 backend；
- 默认 `motion_backend.type` 未改变。HuNav 保留为冻结基线，`local_motion` 仅用于 E2 gate，
  不能据此宣称已经替换主链路。

候选器新增相对速度障碍边界采样、候选级静态扫掠预筛、上一帧速度连续性和低速 heading
锁定。前向或侧向存在可行解时优先保持路线进度；若已进入无法侧向通过的紧急窗口，后退
候选仍然保留，不把“禁止后退”写成错误硬规则。

纯 2D gate 已覆盖并通过：动态边界候选、静态侧预筛、跨 tick 侧向连续性、静止 heading
和 doorway 前向通过。下一步必须使用 `--motion-backend local_motion` 跑固定 seed 的
open crossing、doorway、narrow-passable 和 EnterUseExit Isaac smoke；全部通过视觉包络与
stop/yaw gate 后，才讨论修改 YAML 默认 backend。
