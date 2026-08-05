# Toilet Benchmark 当前架构

## 1. 主链路

```text
Route Editor
  -> EpisodeSpec JSON
  -> toilet_authored_scenario
  -> WalkableMapPlanner + IsaacPeopleBackend
  -> Isaac People/AnimGraph

EpisodeSpec JSON
  -> manual_collection_node
  -> robot reset + authored runner + rosbag + collision verdict
  -> session/episode dataset

Replay bundle
  -> toilet_replay / toilet_replay_isaac
  -> frozen trajectory playback
```

当前代码只有一个在线行人运动所有者：`IsaacPeopleBackend`。它把 authored 路径作为
`path_points_flat` 发送给 Isaac People；People/AnimGraph 负责步态和 root motion。
benchmark 不再维护第二套 Director、SmartObject executive 或 local-motion pipeline。

## 2. 包职责

### `toilet_benchmark_ui`

- 编辑机器人和行人出生点；
- 编辑行人 waypoint、速度和 hold；
- 使用 walkable map 做离线静态验证；
- 保存标准 EpisodeSpec JSON。

UI 不直接驱动 Isaac，也不持有运行时状态。

### `toilet_benchmark.tracks.authored_scenario`

- 加载并验证 EpisodeSpec；
- 将 authored waypoint 连接为静态可行走路径；
- spawn/reactivate 行人；
- 分段发送路径、处理 hold、到达和退休；
- 配置人机接触策略并发布终态。

### `toilet_benchmark.manual_collection_node`

- 选择 authored episode；
- reset 机器人并启动独立 authored runner；
- 管理 rosbag 生命周期；
- 根据机器人目标、接触事件和超时判定 episode；
- 写入 manifest、配置快照和结果。

它不实现行人规划，也不复用跨 episode 的行人导航状态。

### Replay

- `tracks/replay.py`：冻结轨迹 schema、验证、插值和 Replay 快照；
- `tracks/replay_isaac.py`：将 Replay 状态接入 Isaac external motion；
- `tracks/replay_export.py`：导出可复现 bundle。

Replay 与 authored runtime 共享事件和任务终态枚举，但不共享在线路径执行状态。

### 数据与策略

- `dataset/`：session 审计、索引和训练样本准备；
- `toilet_policy`：训练、推理和 Isaac 策略评测；
- `episode_recorder.py`：数采落盘；
- `episodes/`：EpisodeSpec、validator、manifest 和 split。

## 3. 保留契约

- `domain/task.py`：`MotionCommand`、`TaskPhase`、`TerminationReason`；
- `domain/events.py`：benchmark 事件编码和解码；
- `motion_backend.py`：`IsaacPeopleBackend`；
- `geometry.py`：Replay/诊断共用的无 backend 几何函数；
- `walkable_map.py`、`walkable_map_planner.py`、`voxel_path_planner.py`：静态地图与路径；
- `pedestrian_state_stream.py`：Isaac 行人观测转换；
- `external_motion_stream.py`：Replay external-motion 数据流。

## 4. ROS/Isaac 边界

`arena-isaac` 负责场景、机器人 PhysX、传感器、People/AnimGraph 和接触事件。
`toilet_benchmark` 只通过 ROS service/topic 使用这些能力，不复制 Isaac 内部状态。

正式主操作面：

```text
bridge physx_diff_contact
spawn --phase physx_diff_contact
toilet_authored_scenario
manual_collection_node
toilet_replay_isaac
```

## 5. 删除边界

旧 HuNav、Director、E1 SmartObject runtime 和 E2 local-motion 实验已由 Git 历史保存。
当前代码不提供兼容入口、shadow backend 或 legacy 目录。未来新增行人模型时必须复用
EpisodeSpec、事件、数据和评测契约，并以独立 backend 接入，不得同时争夺同一 actor 的
运动所有权。
