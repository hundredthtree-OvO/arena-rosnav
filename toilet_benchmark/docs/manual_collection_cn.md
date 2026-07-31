# 厕所社会导航人工数采手册

状态：当前 Dataset Track 操作说明  
最后更新：2026-07-30

## 1. 定位

人工数采属于 `Dataset Track`，用于收集操作者在行人交互环境中的机器人示范。它不是
正式 Evaluation Track，也不直接产生 benchmark 排名。

每个 episode：

1. 机器人在预验证的安全位姿复位；
2. 一个或多个预生成行人从入口激活；
3. 行人前往配置的小便池并执行 Enter-Use-Exit 生命周期；
4. 操作者使用手柄控制机器人前往门口目标；
5. rosbag 保存机器人 observation/action、行人状态和事件；
6. 成功、碰撞、超时或人工终止后保存 manifest；
7. 机器人复位，行人 park，开始下一轮。

整个 session 不重启 bridge，不重复导入场景，也不频繁增删行人 prim。

## 2. 当前配置

配置文件：

```text
config/manual_collection.yaml
```

关键字段：

```yaml
session:
  seed: 42
  selection_mode: round_robin
  fixed_scenario_id: robot_urinal_3_ped_urinal_3
  max_episodes: 0

pedestrian:
  count: 2
  character_pool:
    - original_female_adult_business_02
    - original_female_adult_medical_01

scenarios:
  - id: robot_urinal_3_ped_urinal_3
    robot_start: [1.31, 0.75, 0.03, -1.5708]
    robot_goal: [-3.8, -0.91, 0.0]
    pedestrian_target_urinal_ids: [urinal_3, urinal_4]
```

`selection_mode`：

- `fixed`：固定一个场景，用于调试；
- `round_robin`：顺序均衡覆盖；
- `seeded_random`：固定 seed 的加权随机。

如需指定人数：

```yaml
pedestrian:
  count: 3
```

并为场景提供目标：

```yaml
pedestrian_target_urinal_ids: [urinal_1, urinal_3, urinal_5]
```

目标或角色数量少于 `count` 时当前实现会循环使用列表。正式数据集应避免无意循环，
并在 episode manifest 中核对最终分配。

## 3. 录制内容

当前持续话题：

- `/clock`；
- `/tf`、`/tf_static`；
- `/cmd_vel_gamepad_diff`；
- `/cmd_vel_applied`；
- `/odom`；
- `/front_scan`；
- `/rear_scan`；
- `/isaac/pedestrian_states`。

事件话题：

- `/toilet_benchmark/collection_status`；
- `/isaac/pedestrian_hard_guard_events`；
- `/isaac/scene_collision_events`。

只有持续可用话题参与录制 ready gate。事件话题可能在发生事件前没有 publisher，不应
阻止 episode 开始。

## 4. 启动

先启动 Isaac 主链路：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

测试流程但不录 bag：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml \
  --no-record
```

正式录制：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml
```

`Ctrl-C` 会中止当前 episode、停止 rosbag、park 行人并保存 session manifest。

## 5. 输出

默认目录：

```text
/home/stardust/resources/arena_ws/data/toilet_manual/
  session_YYYYMMDD_HHMMSS_seed42/
    session_manifest.yaml
    episode_000001/
      metadata.yaml
      events.jsonl
      bag/
```

每个 episode 至少应记录：

- session、episode、scenario 和 operator ID；
- seed 和抽样序号；
- 机器人起点/目标；
- 行人数、角色和目标小便池；
- 配置路径及 hash；
- git commit 和 dirty 状态；
- rosbag 开始/结束时间；
- 成功/失败/中止状态；
- termination reason；
- 碰撞和 hard guard 事件摘要。

## 6. 数据有效性

以下情况应标记为无效仿真，而不是直接作为失败示范：

- 行人瞬移、穿墙或持续原地旋转；
- 行人因 roster/reset 错误消失；
- robot reset 未完成；
- bag ready 前动作已经开始；
- 雷达或 odom 在主要区间缺失；
- bridge/runtime 崩溃；
- 配置与实际人数/目标不一致。

以下情况属于有效的任务失败：

- 操作者与行人发生真实碰撞；
- 操作者与静态场景碰撞；
- episode timeout；
- 机器人 deadlock；
- 操作者未能到达目标。

`pedestrian_hard_guard_events` 是控制介入诊断，不等同于 PhysX contact。正式数据需要
分别保存：

```text
social warning
hard-guard intervention
geometric overlap
PhysX contact
episode failure
```

## 7. Reset 验收

正式 session 前至少执行：

1. `--no-record` 连续 10 个 episode；
2. 每次机器人复位位姿、yaw 和速度一致；
3. 行人 park 后不再出现在雷达和状态流中；
4. 下一 episode 激活时不回跳；
5. 上一 episode action 不泄漏；
6. 行人和机器人无初始重叠；
7. bridge RSS 不持续异常增长；
8. `Ctrl-C` 能在当前状态安全退出。

长期门槛：

- 同一 bridge 连续完成至少 30 个 episode；
- 无 prim 高频增删导致的 Isaac 崩溃；
- 相同 seed 得到相同 scenario 和目标分配序列；
- recording ready 后第一帧动作和 observation 时序正确。

## 8. 与正式 Benchmark 的关系

人工数采可以提供：

- imitation-learning demonstrations；
- 行人交互失败案例；
- Replay Track 候选轨迹；
- 传感器和控制时序样本；
- 指标开发的手工标注样例。

它不能替代：

- 正式 episode dataset；
- train/validation/test split；
- 标准 policy runner；
- 官方 evaluator；
- baseline；
- leaderboard 协议。

总体规范见 `benchmark_design_cn.md`。

