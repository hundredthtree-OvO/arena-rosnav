# Replay 轨迹 Schema 与 Actor Foundation

状态：Schema frozen / actor foundation implemented v0.1  
最后更新：2026-07-30

本文定义 `Replay Track` 消费的冻结行人轨迹资产，并记录首个后端中立 replay actor
foundation。目标是把 replay 轨迹和当前 `EpisodeSpec`、`AgentSnapshot`、
`BenchmarkEvent` 契约对齐。当前 actor 负责严格校验、确定性插值和 dry-run，不调用
实时社会运动规划；live 模式通过 Isaac `external_motion` 写入运动，并可用
`--spawn-character` 在首帧位姿自动生成目标行人。

## 1. 设计目标

Replay 轨迹资产必须满足以下要求：

1. 能从现有 `EpisodeSpec` 派生或回写到现有 episode/manifest 体系；
2. 轨迹字段可以被当前 `domain.agent.AgentSnapshot` 承载，不依赖 ROS 类型；
3. 坐标系、时间基准、采样规则、碰撞终止规则都显式记录；
4. policy 不能看到未来轨迹，evaluator/oracle 可以看到冻结真值；
5. 轨迹文件可独立校验、可哈希、可回放、可重算。

本草案只覆盖行人参考轨迹。机器人动作、评分结果和最终 episode 结果仍属于
`EpisodeSpec` / evaluator 范畴，不写进 replay 轨迹的主体数据结构里。

## 2. 与现有契约的对齐

当前仓库里已经有三组可复用契约：

1. `episodes/schema.py`
   - `EpisodeSpec`
   - `RobotEpisodeSpec`
   - `PedestrianEpisodeSpec`
   - `TerminationSpec`
   - `CollisionPolicy`
2. `domain/agent.py`
   - `AgentSnapshot`
3. `domain/events.py` 和 `domain/task.py`
   - `BenchmarkEvent`
   - `TaskPhase`
   - `TerminationReason`

Replay 轨迹应优先复用这些字段语义，而不是另起一套新的坐标或事件体系。
特别是 `AgentSnapshot` 已经提供了与后端无关的状态载体：

```text
agent_id, x, y, z, yaw, vx, vy, wz, radius_m, timestamp_sec, source
```

这组字段适合作为 replay sample 的基础状态，不应再引入 ROS 专属字段。

## 3. 建议文件外壳

建议 replay 轨迹文件使用单个 JSON 或 YAML bundle，最少包含以下顶层字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `schema_version` | string | replay 轨迹 schema 版本，建议独立于 episode schema |
| `benchmark_version` | string | 与 episode 一致的 benchmark 版本 |
| `episode_id` | string | 对应的 episode ID |
| `scene_id` | string | 场景 ID，例如 `shenxinfu_841837` |
| `task_type` | string | 任务族，例如 `enter_exit` |
| `track` | string | 固定为 `replay` |
| `seed` | int | 与 episode 保持一致的生成 seed |
| `reference_episode_hash` | string | 绑定来源 episode，防止错配 |
| `coordinate_frame` | string | 轨迹所使用的世界坐标系 |
| `time_base` | object | 时间基准定义 |
| `agents` | array | 冻结行人轨迹列表 |
| `annotations` | object | 终止、碰撞、质量检查等附加信息 |
| `provenance` | object | 生成来源、构建版本、哈希信息 |

### 3.1 `time_base` 建议字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `origin` | string | 建议值：`episode_reset` |
| `clock` | string | 建议值：`physics_monotonic_sec` |
| `unit` | string | 建议值：`sec` |
| `zero_sec` | number | 第一帧可回放状态对应的时间，通常为 `0.0` |
| `monotonic` | bool | 必须为 `true` |
| `sample_policy` | string | 建议值：`strictly_increasing` |

### 3.2 `provenance` 建议字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `source_episode_manifest` | string | 来源 manifest 路径或相对标识 |
| `source_episode_hash` | string | 来源 episode hash |
| `source_capture` | string | 原始采集来源，例如 `manual_collection` |
| `generated_by` | string | 生成器标识或工具名 |
| `generated_at_sec` | number | 生成时间，使用 wall-clock 秒也可以，但不得参与轨迹回放时序 |
| `content_hash` | string | bundle 内容哈希 |

## 4. 行人轨迹字段

每个 `agents[]` 元素表示一个被冻结的行人参考轨迹，建议结构如下：

```yaml
agent_id: toilet_agent_01
semantic_goal: urinal_3
character: original_female_adult_business_02
start_reference: entrance_main
start_pose: [x, y, z, yaw]
sample_rate_hz: 10
trajectory:
  - sample_index: 0
    timestamp_sec: 0.0
    x: 1.23
    y: 0.45
    z: 0.03
    yaw: -1.57
    vx: 0.0
    vy: 0.0
    wz: 0.0
    radius_m: 0.30
    source: replay_reference
```

### 4.1 建议字段说明

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `agent_id` | string | 与 `EpisodeSpec.pedestrians[].agent_id` 一致 |
| `semantic_goal` | string | 与当前 episode contract 一致的小便池资源 ID |
| `character` | string? | 可选，保留当前角色选择信息 |
| `start_reference` | string? | 可选，保留入口/锚点语义 |
| `start_pose` | float[4]? | 参考起点，和 episode 的生成来源保持一致 |
| `sample_rate_hz` | number? | 可选的名义采样频率 |
| `trajectory[]` | array | 按时间排序的冻结状态序列 |

### 4.2 单条轨迹样本建议字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `sample_index` | int | 从 0 开始的单调索引 |
| `timestamp_sec` | number | 轨迹的主时间轴，必须单调递增 |
| `x, y, z` | number | 世界坐标系位置，单位米 |
| `yaw` | number | 绕 `+z` 轴的朝向，单位弧度 |
| `vx, vy` | number | 世界坐标系平面速度，单位 m/s |
| `wz` | number | 角速度，单位 rad/s |
| `radius_m` | number | 代理半径，用于接触和安全判定 |
| `source` | string | 样本来源标签，建议固定为 `replay_reference` |
| `phase` | string? | 可选，若要表达任务阶段，可复用 `TaskPhase` 语义 |
| `confidence` | number? | 可选，表示样本生成质量或插值可信度 |

### 4.3 允许的补充字段

如果后续需要更强的验证能力，可以增加只读附加字段，但不要改变基础样本主结构：

- `heading_rate_rps`
- `path_length_m`
- `arc_length_m`
- `contact_state`
- `occlusion_flag`
- `surface_frame`

这些字段都应视为 evaluator/oracle 辅助信息，不作为 policy 输入的必要字段。

## 5. 坐标与时间基准

### 5.1 坐标基准

Replay 轨迹使用当前场景的世界坐标系，不引入新的局部坐标系作为主坐标。

约定如下：

- `x`、`y`、`z` 的单位都是米；
- `yaw` 使用右手系绕 `+z` 轴的弧度值；
- `z` 应保持与当前场景约定一致，通常是地面附近的固定高度；
- `x/y` 的定义必须和现有 episode 里 `robot.start_pose`、`robot.goal_pose` 的世界语义一致；
- 如果要输出局部参考系，只能作为附加字段，不能替代世界坐标。

### 5.2 时间基准

轨迹时间必须是 episode 内部的单调时间，不得依赖 wall-clock 作为回放主时钟。

约定如下：

- `timestamp_sec=0.0` 对应 episode reset 后的第一帧可回放状态；
- 时间单位固定为秒；
- 轨迹样本必须严格单调递增，不允许回退；
- 允许固定步长，也允许稀疏采样，但必须记录采样策略；
- 若源数据来自更高频率的 capture，重采样后仍应保留单调序列和内容哈希。

## 6. policy / evaluator 可见性

Replay Track 的核心约束不是“轨迹文件里有什么”，而是“哪些字段允许被 policy 看到”。

### 6.1 Policy 可见

Policy 只允许看到当前时刻及其之前已经发生的状态，不允许读取未来轨迹。

允许项：

- 当前时刻的机器人 observation；
- 当前时刻的局部行人状态；
- 当前时刻之前已经发布的 `BenchmarkEvent`；
- 当前时刻允许暴露的地图、目标或历史帧。

不允许项：

- `trajectory[]` 中未来样本；
- `annotations` 里的终止原因、碰撞真值和后验质量标签；
- `provenance.content_hash` 以外的敏感真值；
- 任何以 `future_`、`oracle_`、`ground_truth_` 命名的未来字段。

### 6.2 Evaluator / Oracle 可见

Evaluator 或 oracle 可以读取完整 replay bundle，用于：

- 逐步注入行人状态；
- 校验轨迹完整性；
- 判定终止原因；
- 生成 episode JSONL；
- 计算回放误差和基准指标。

允许项：

- 完整 `trajectory[]`；
- `annotations.collisions`；
- `annotations.termination_reason`；
- `annotations.coverage`；
- `provenance.*`；
- `reference_episode_hash`；
- `content_hash`。

### 6.3 推荐的可见性切分原则

如果字段是否可见不确定，默认按 evaluator-only 处理。
Replay Track 的可复现性优先于便利性，宁可少给 policy，也不要把未来状态泄漏给 policy。

## 7. 碰撞终止规则

Replay Track 的终止规则应和当前 `TerminationSpec`、`TerminationReason` 对齐。

### 7.1 Release 级规则

1. 若 `collision_policy=terminate`，第一次真实几何碰撞即终止 episode；
2. 碰撞类别至少分为：
   - `robot_human_collision`
   - `robot_scene_collision`
3. 终止原因必须是单值且可复现，不能在同一 episode 里同时给出多个终止主因；
4. 终止事件必须写入 `annotations.termination_reason`，并保留碰撞时间戳和参与方；
5. 如果碰撞发生在同一时刻的多个候选接触之间，应按最早接触帧和最小时间戳稳定打破平局。

### 7.2 建议的碰撞注释结构

```yaml
annotations:
  termination_reason: robot_human_collision
  termination_timestamp_sec: 12.34
  collisions:
    - kind: robot_human_collision
      timestamp_sec: 12.34
      actor_ids: [robot, toilet_agent_01]
      contact_frame: world
      geometry_source: physx_contact
      action: terminate
```

### 7.3 Debug-only 容忍模式

`collision_policy=contain` 可以作为调试态保留，但不应进入正式 Replay benchmark 统计。
如果保留该模式，必须显式标记为非 release 轨迹，并和正式评测分开存放。

## 8. 验收标准

Replay 轨迹 bundle 进入正式使用前，建议至少通过以下检查：

1. `schema_version`、`benchmark_version`、`episode_id`、`scene_id` 与来源 episode 一致；
2. 每个 `agent_id` 在来源 episode 中唯一，且轨迹样本按时间严格单调；
3. 所有数值字段有限且可序列化；
4. `reference_episode_hash` 与来源 episode hash 匹配；
5. `content_hash` 可重算且稳定；
6. `trajectory[]` 中不存在未来泄漏给 policy 的字段；
7. `collision_policy=terminate` 的 episode 在第一次真实几何碰撞时能稳定终止；
8. 同一输入 episode、同一 seed、同一机器人动作序列下，轨迹重放误差落在定义容差内；
9. Replay bundle 和 episode manifest 不出现重复、缺失或交叉污染；
10. evaluator 可独立重算终止原因和统计结果。

## 9. 当前实现边界

已实现：

- 单行人 bundle 的严格字段、时间、有限值、hash 和来源校验；
- `yaw` 跨越 `+pi/-pi` 时的最短角度插值；
- 固定步长的确定性状态序列和 sequence hash；
- `--seal-output`、`--validate-only` 和 dry-run CLI；
- 后端中立 `ReplayActorSink` 接口。

当前不实现：

- 不根据两个点在线规划；
- 不定义 metrics 公式；
- 不替换当前 Dataset Track 的采集流程；
- 不支持多人 bundle。

新增实现：

- 可从 manual collection episode 目录内的 rosbag2
  `/isaac/pedestrian_states` 导出单行人轨迹；
- 自动剔除远处 parking 帧；
- 旧 bag 的 `Person.velocity` 不可信时，以位置有限差分恢复 `vx/vy/yaw/wz`；
- `ReplayStateFrame` 已映射到 Isaac `MotionCommand` 的
  `direct_pose + orientation + external_velocity`；
- 终帧使用 `EXTERNAL_MOTION_TERMINAL_ALIGN` 或显式 freeze。

## 10. 自定义起终点与冻结规则

`start_reference`、`start_pose` 和 `semantic_goal` 是生成阶段的可编辑输入。后续轨迹生成器
可以接收例如 `entrance_main -> urinal_3` 的语义起终点，离线运行全局规划，
通过穿模和碰撞 validator 后输出候选 bundle。

正式 Replay 只消费已经封存的 `trajectory[]`，不会根据起终点重新规划。改变起点、终点、
角色、速度、激活时刻或多人相对时序，都必须生成新 bundle 和新 `content_hash`，不能
直接拉伸或平移旧轨迹。

当前 CLI：

```bash
ros2 run toilet_benchmark toilet_replay authoring.yaml \
  --seal-output sealed.yaml
ros2 run toilet_benchmark toilet_replay sealed.yaml --validate-only
ros2 run toilet_benchmark toilet_replay sealed.yaml --step-sec 0.1
```

第一条真实候选轨迹已从
`session_20260723_175315_seed42/episode_000001` 导出并通过 bundle 校验。

## 11. 首次 Isaac Live Replay 结果

2026-07-30 使用 `physx_diff_contact` 主链路和
`original_female_adult_business_02` 完成首次完整 live replay：

- bundle：`/tmp/toilet_replay_lane_b_episode_000001.yaml`
- bundle hash：`81206c5edbe42e64ec63aaef4df11b70e94285bbab9d8d648843690d546deb17`
- reference：974 个样本，98.482 s
- live bag：`/tmp/toilet_replay_lane_b_live_states_retry1`
- tracking metrics：`/tmp/toilet_replay_lane_b_tracking_metrics.json`

结果：

- actor 自动 spawn 成功，`animgraph_setup=True`；
- `/isaac/move_pedestrians` external-motion 完整执行并返回 `finished=true`；
- 路径横向 RMSE 为 0.078 m，P95 为 0.163 m，最大值为 0.290 m；
- 按原始时间轴对齐的位置 RMSE 为 1.031 m，P95 为 2.796 m；
- live 最终位置仍比 reference 终点滞后约 0.26 m。

结论：当前 adapter 已打通 Isaac 写入和动画路径跟踪，但只达到“空间路径可用”，尚未
达到 Replay Track 的确定性时间回放标准。主要原因是 Replay 仍复用了面向自然动画的
软 external-motion tracking：角色允许落后 reference，runner 又在 reference 时间轴
结束时立即退出，没有等待终点追赶和终态确认。

下一切片：

1. Replay 使用独立于 authored runtime 的追赶策略和误差上限；
2. terminal frame 必须等待 live pose 进入位置/yaw 容差后才报告完成；
3. validator 增加时间对齐 RMSE、路径横向误差和终点误差阈值；
4. 未通过上述阈值前，不把 `finished=true` 视为正式 Replay 合格。

## 12. Replay 追赶与终态握手

live runner 已增加只作用于 Replay Track 的闭环，不修改 authored runtime 或 Isaac
全局 external-motion 行为：

1. 从 `/isaac/pedestrian_states` 读取位置、速度以及 tags 中的
   `yaw_valid/yaw_rad`；
2. 将 live pose 投影到 reference 的局部前向时间窗口，进度只允许单调前进，避免
   入口和出口复用同一走廊时匹配到错误时间段；
3. reference 最多领先 live 进度 `catch_up_sec`，减少动画角色持续落后；
4. reference 到终帧后继续发送 terminal-align，直到真实位置、yaw、速度连续满足
   `terminal_stable_samples` 次；
5. 超过 `terminal_timeout_sec` 时输出明确 timeout 结果并返回非零退出码。

### Isaac 实时 Replay 验收

冻结 Replay 与 authored runtime 使用不同运动权威：

- Authored `LOCOMOTION`：路径执行器给连续参考，AnimGraph 根运动负责自然行走；
- Replay `REPLAY_TRACK`：冻结轨迹插值负责根位姿，AnimGraph 只负责 Walk/Idle；
- `FREEZE` 和 `TERMINAL_ALIGN` 继续负责回放末端静止与朝向收敛。

这是必要的 Track 隔离。原先让 AnimGraph 独占 Replay 根运动时，角色会在冻结轨迹
约 `t=6 s` 的减速反向段停止，live-progress 与 reference catch-up 形成死锁。

2026-07-30 使用 `toilet_replay_lane_b_episode_000001.yaml` 的实时验收结果：

- 完整轨迹回放成功；
- terminal position error：`0.0203 m`；
- terminal yaw error：约 `5.9e-8 rad`；
- 连续 `3` 个稳定样本，`0.60 s` 内完成终态握手。

dry-run 仍完全由 bundle 和 replay clock 决定，不订阅 ROS，也不受 live feedback
影响。单元回归覆盖 yaw tag、往返轨迹单调匹配、非等速段时间投影、终态稳定/抖动和
超时。实时验收已经通过；下一步补齐整段时间对齐 RMSE 和路径横向误差统计，而不是
只依赖终点收敛指标。

### 全程误差冻结

live runner 现在把每个 `/isaac/pedestrian_states` 样本与当时实际下发的
`playback_sec` 配对。不能使用绝对 ROS header 时间，也不能在启用 catch-up 后简单用
墙钟乘 `time_scale`，否则会把参考进度错误钳到终点。

2026-07-30 使用同一冻结 bundle 的复测结果：

```text
/tmp/toilet_replay_single_baseline_20260730_v2.jsonl
```

- live samples：1606；
- time-aligned position RMSE：`0.0159 m`；
- path lateral RMSE：`0.00041 m`；
- path lateral max：`0.00684 m`；
- terminal position error：`0.00684 m`；
- terminal yaw error：约 `5.9e-8 rad`；
- terminal handshake：连续 3 帧收敛，结果为 `finished=true`。

该结果冻结为当前单行人 Replay 实现基线。后续 validator 阈值应在多条冻结轨迹上确定，
不能只按这一条轨迹直接固化。
