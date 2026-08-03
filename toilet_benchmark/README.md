# Toilet Benchmark

`toilet_benchmark` 是厕所社会导航场景的任务编排、行人运行实验和人工数据采集包。
当前项目处于 benchmark 原型阶段，不应把“场景和行人能够运行”等同于正式公开
benchmark 已完成。

总体规范和开发阶段以
[`docs/benchmark_design_cn.md`](docs/benchmark_design_cn.md) 为准。
完整文档入口见 [`docs/README.md`](docs/README.md)，内部控制权和冻结接口以
[`pedestrian_ecosystem_architecture_cn.md`](docs/pedestrian_ecosystem_architecture_cn.md)
为准。
路线编辑器和运行链路清理的执行边界见
[`docs/route_editor_runtime_cleanup_plan_cn.md`](docs/route_editor_runtime_cleanup_plan_cn.md)。
阶段 3 的独立 Qt 2D Route Editor 位于同一仓库的 sibling 包
`toilet_benchmark_ui`，不属于核心 benchmark 运行时依赖；RViz 继续用于 LiDAR、TF
和 3D 场景诊断。

## 当前边界

```text
arena-isaac
  scene / robot / PhysX / sensors / Isaac character runtime

toilet director
  current compatibility facade for lifecycle and ROS

scenario runtime / smart objects / agent executive
  entrance / queue / resource / service / exit intent and events

global router
  static walkable route

local motion backend
  HuNav for Interactive social motion; Replay and future ORCA/HRVO share the port

embodiment adapter
  Isaac AnimGraph now; SMPL-H is an A/B candidate

recorder / evaluator
  dataset recording now; formal evaluator is planned
```

现有最小厕所事件流：

```text
ENTERING -> WALK_TO_URINAL / QUEUEING -> USING_URINAL -> EXITING
```

默认不传 `--motion-backend hunav` 时仍使用兼容的 `IsaacPeopleBackend`。HuNav
takeover 是当前 Interactive Track 的实验主线；它不是 Replay Track，也不负责厕所
资源状态、episode split 或最终评测。

## 配置

- `config/toilet_semantics.yaml`
  - 入口、出口、portal、小便池和排队语义；
  - `pose` / `queue_slots` 是实际行人锚点；
  - `scene_prim` 用于场景对象对应。
- `config/toilet_benchmark.yaml`
  - director 生命周期、服务时间、运动 backend、HuNav 和交互参数。
- `config/manual_collection.yaml`
  - 人工 Dataset Track 的 session、episode、人数、场景组合和录制话题。

小便池 `pose` 应是人实际站立的位置，不是 mesh 原点。入口、出口和队列点必须先在
当前 USD 中验证无碰撞和可达。

## 文档

| 文档 | 用途 |
| --- | --- |
| [`benchmark_design_cn.md`](docs/benchmark_design_cn.md) | Benchmark Card、任务、Track、episode、接口、指标、baseline 和路线图 |
| [`pedestrian_ecosystem_architecture_cn.md`](docs/pedestrian_ecosystem_architecture_cn.md) | 目的性行人生态、Smart Object、Scenario Runtime、Agent Executive 和冻结接口 |
| [`code_structure_migration_cn.md`](docs/code_structure_migration_cn.md) | 两条并行开发线的目标结构、共享契约、迁移顺序和删除门槛 |
| [`motion_backend_evaluation_cn.md`](docs/motion_backend_evaluation_cn.md) | HuNav、ORCA/HRVO、Replay、Isaac AnimGraph 与 SMPL-H 的统一评估门槛 |
| [`hunav_behavior_characterization_20260730_cn.md`](docs/hunav_behavior_characterization_20260730_cn.md) | S3-A 原生 behavior 固定 seed Isaac 对照与诊断限制 |
| [`replay_trajectory_schema_cn.md`](docs/replay_trajectory_schema_cn.md) | Replay Track 冻结轨迹的字段、时间基准和可见性草案 |
| [`manual_collection_cn.md`](docs/manual_collection_cn.md) | 当前人工数据采集配置、启动、有效性和 reset 验收 |
| [`hunav_takeover_cn.md`](docs/hunav_takeover_cn.md) | 当前 HuNav + Isaac 主链路运行与诊断 |
| [`hunav_pedestrian_pipeline_migration_plan_cn.md`](docs/hunav_pedestrian_pipeline_migration_plan_cn.md) | Interactive Track 的 HuNav 实现迁移记录 |
| [`hunav_phase0_smoke_cn.md`](docs/hunav_phase0_smoke_cn.md) | 不启动 Isaac 的 HuNav 隔离验证 |
| [`hunav_isaac_mirror_cn.md`](docs/hunav_isaac_mirror_cn.md) | 旧执行器与 HuNav shadow 对照工具 |
| [`pedestrian_pair_guard_failure_review_20260724.md`](docs/pedestrian_pair_guard_failure_review_20260724.md) | 行人间逐帧 guard 失败复盘和禁止回归约束 |

## 构建

HuNav takeover 同时依赖消息、bridge 和 benchmark：

```bash
cd /home/stardust/resources/arena_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select isaacsim_msgs ros2isaacsim toilet_benchmark \
  --symlink-install
source install/setup.bash
```

## 快速运行

首次导入资产和机器人：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  spawn --phase physx_diff_contact
```

终端 1：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

终端 2，单行人 HuNav 回归：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --initial-agents 1
```

E2-B 独立局部运动实验不启动 HuNav，可在同一 bridge 上显式运行：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend local_motion \
  --initial-agents 1
```

该入口目前只用于 doorway/crossing/视觉包络 gate，尚未替换默认 backend。

E1 事件内核默认以 `shadow` 运行：它比较新旧阶段和资源归属，但不改变现有运动。完成
固定 seed 的 shadow smoke 后，可显式验证语义接管：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --scenario-runtime takeover \
  --initial-agents 1
```

需要快速回退时使用 `--scenario-runtime legacy`。运行中不应出现
`E1 scenario shadow mismatch`。

多人属于 Interactive Track 实验入口：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --initial-agents 2
```

自动执行 bridge、spawn 和单次 HuNav smoke：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark hunav_interactive_smoke \
  --behavior surprised \
  --reaction disabled \
  --legacy-avoidance disabled \
  --seed 12345 \
  --target-resource urinal_1
```

runner 先等待当前 bridge 日志中的 `Simulation App Startup Complete`，再检查
Isaac ROS 服务，避免上一轮残留 ROS graph 造成错误握手；服务就绪并完成机器人
settling 后才启动 director。每次运行的 bridge、spawn、director、HuNav manager、
hold diagnostics 和汇总结果保存在
`/tmp/toilet_pipeline_runs/<timestamp>_<behavior>_<reaction>/`。

调试时如果 bridge、场景和机器人已经稳定运行，可复用当前实例，避免反复初始化
Isaac Sim：

```bash
ros2 run toilet_benchmark hunav_interactive_smoke \
  --reuse-existing-bridge \
  --skip-spawn \
  --behavior surprised \
  --reaction disabled \
  --legacy-avoidance disabled \
  --robot-intervention dynamic_crossing \
  --dynamic-crossing-profile crossing_conflict
```

复用模式只检查 Isaac ROS 服务，不创建、重启或终止 bridge；`--skip-spawn` 表示
场景和机器人已经通过固定 `spawn --phase physx_diff_contact` 操作面导入。

使用和 gamepad 相同的差速控制 topic 执行可复现动态 crossing：

```bash
ros2 run toilet_benchmark hunav_interactive_smoke \
  --behavior regular \
  --reaction disabled \
  --legacy-avoidance disabled \
  --seed 12345 \
  --target-resource urinal_1 \
  --robot-intervention dynamic_crossing \
  --dynamic-crossing-profile crossing_conflict
```

该模式先固定机器人起始位姿，在行人达到触发位置后通过
`/cmd_vel_gamepad_diff` 执行 `cmd_vel_pulse`，结束时显式补发零速命令；profile
配置了 `post_pose` 时还会把机器人移到清场位姿，避免继续影响后续阶段。
`robot_intervention.jsonl` 保存触发、运动命令、停止和清场结果。
`crossing_clear` 使用更早触发和更短运动窗口作为无接触候选基线，
`crossing_conflict` 保留强制交汇配置；各参数仍可通过原
`--dynamic-crossing-*` 参数覆盖。

固定 seed 的单、双行人行为矩阵按顺序运行 Isaac，避免多个 bridge 争用 GPU：

```bash
ros2 run toilet_benchmark hunav_behavior_matrix \
  --behaviors regular,surprised,scared \
  --agent-counts 1,2 \
  --intervention-profiles crossing_clear,crossing_conflict \
  --seeds 12345 \
  --output-root /tmp/toilet_behavior_matrix
```

总结果写入 `<timestamp>_behavior_matrix/matrix_summary.json`；先加 `--dry-run`
可只检查 12 个子任务及其命令，不启动 Isaac。

人工数采：

```bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml
```

生成和校验固定 episode manifest：

```bash
ros2 run toilet_benchmark toilet_manifest generate \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml \
  --scene-id shenxinfu_841837 \
  --seed 42 \
  --split-ratios train=0.6,validation=0.2,test=0.2 \
  --output /tmp/toilet_manifest_seed42.json

ros2 run toilet_benchmark toilet_manifest validate \
  /tmp/toilet_manifest_seed42.json
```

冻结 Replay bundle 的最小工作流：

```bash
# authoring YAML 中填写完整冻结轨迹后，校验并写入稳定 content_hash。
ros2 run toilet_benchmark toilet_replay \
  /tmp/replay_authoring.yaml \
  --seal-output /tmp/replay_sealed.yaml

ros2 run toilet_benchmark toilet_replay \
  /tmp/replay_sealed.yaml \
  --validate-only

# 以固定 0.1 s 步长执行后端中立 dry-run。
ros2 run toilet_benchmark toilet_replay \
  /tmp/replay_sealed.yaml \
  --step-sec 0.1
```

当前 Replay Track 只支持单行人且不调用 HuNav。语义起点和目标可在生成 bundle 时
自定义；评测回放期间轨迹保持冻结。

从已有合格 episode 的 `/isaac/pedestrian_states` 直接导出冻结轨迹：

```bash
ros2 run toilet_benchmark toilet_replay_export \
  /home/stardust/resources/arena_ws/data/toilet_manual/session_20260723_175315_seed42/episode_000001 \
  --agent-id toilet_agent_01 \
  --scene-id shenxinfu_841837 \
  --output /tmp/toilet_replay_episode_000001.yaml

ros2 run toilet_benchmark toilet_replay_isaac \
  /tmp/toilet_replay_episode_000001.yaml \
  --dry-run
```

导出器会裁掉远处 parking 帧；由于旧 bag 的 `Person.velocity` 基本为零，速度和 yaw
由相邻位置差分恢复，并写入 provenance。去掉 `--dry-run` 后，actor 通过
`/isaac/move_pedestrians` 的 `external_motion` 契约写入 Isaac。目标 agent 已存在时
可直接回放；否则传入 `--spawn-character original_female_adult_business_02`，actor
会在首帧位姿调用 `/isaac/spawn_pedestrian` 后再开始回放。

live replay 会订阅 `/isaac/pedestrian_states`，用单调局部进度限制 reference
领先量，并在终帧等待真实位置、yaw 和速度连续稳定后才退出。常用调试参数为
`--catch-up-sec`、`--terminal-position-tolerance-m`、
`--terminal-yaw-tolerance-rad`、`--terminal-stable-samples` 和
`--terminal-timeout-sec`；终态超时返回非零退出码，不再把 reference 时间结束直接
视为回放成功。

Replay Track 使用独立的 `REPLAY_TRACK` external-motion 模式：冻结轨迹拥有根位姿
权威，Isaac AnimGraph 只负责 Walk/Idle 外观。Interactive Track 的 HuNav
`LOCOMOTION` 仍由 AnimGraph 根运动闭环驱动，两者不会互相改变控制语义。

生成结果包含 manifest 和同目录下的 `toilet_manifest_seed42.episodes/`。相同配置和
seed 应生成字节级一致结果。

具体启动顺序、日志位置和验收项见对应文档，不在 README 重复维护。
