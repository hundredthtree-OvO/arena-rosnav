# 厕所社会导航人工数采手册

状态：当前 Dataset Track 操作说明  
最后更新：2026-08-03

> 正式数采时不要预先单独启动 `toilet_authored_scenario`。
> `manual_collection_node` 会为每个 episode 启动同一个 authored runner，并额外负责
> 机器人复位、手柄门控、rosbag 和碰撞判负。两者同时运行会向同名行人重复下发路径，
> 导致首轮偏移或跨轮漂移。独立 runner 仅用于不录包的单次路线验证；数采节点若检测到
> 已有 runner，会以 `pedestrian_runtime_already_running` 拒绝本轮。

## 1. 定位

人工数采属于 `Dataset Track`，用于收集操作者在行人交互环境中的机器人示范。它不是
正式 Evaluation Track，也不直接产生 benchmark 排名。

每个 episode：

1. 机器人在预验证的安全位姿复位；
2. 数采节点加载 Route Editor 生成的 `EpisodeSpec`；
3. authored runtime 按 JSON 中的出生点、目标锚点和停留点执行一个或多个行人；
4. 操作者使用手柄控制机器人前往门口目标；
5. rosbag 保存机器人 observation/action、行人状态和事件；
6. 成功、碰撞、超时或人工终止后保存 manifest；
7. 机器人复位，行人 park，开始下一轮。

整个 session 不重启 bridge，不重复导入场景，也不频繁增删行人 prim。
每轮先停止 rosbag 并 hold 机器人，但不向 authored runtime 发送 `SIGINT`。数采节点等待
authored runtime 按完整路线自然停止 People 命令、park 全部行人并正常退出，之后才允许
下一轮。只有等待超过
`session.authored_runtime_exit_timeout_sec` 或 runner 异常退出时，才使用数采节点的强制
park 兜底；兜底 park 超过 `session.parking_timeout_sec` 会终止 session，防止复用脏运行态。

人机接触会在数采终端明确打印 `Robot-human contact detected`、行人 prim 和穿透深度，
随后当前 episode 立即判负并 hold 机器人。hold 后继续操作手柄不会移动，这是失败收尾而非
控制器失效；authored runtime 完成退场后会自动复位并进入下一轮。

authored runtime 自己维护每名行人的 generation、phase boundary、hold 和 terminal state。
每个 phase 的完整 `PathPoints` 交给 People/MotionMatching 执行动画；
`/isaac/pedestrian_states` 用于本轮激活和 waypoint 到达确认，People 内部状态不决定事件流。
每轮复用角色时，bridge 会发布新的 `embodiment_generation`。authored runtime 在出生 pose
有效后预提交路径，让 Person 在 4 帧 warmup 内缓存该 command；只有 bridge 回报同一
embodiment/command generation 已进入 `executing`，runtime 才发布 `pedestrian_active`。
数采节点等待本轮全部行人 generation 都 active 后再释放手柄。

## 2. 当前配置

配置文件：

```text
config/manual_collection.yaml
```

关键字段：

```yaml
session:
  seed: 42
  selection_mode: fixed
  fixed_scenario_id: narrow_head_on_001
  max_episodes: 10

collision_policy:
  pedestrian_robot: detect_and_fail
  hard_guard_service: /isaac/set_pedestrian_hard_guard

scenario_source:
  mode: authored_route

scenarios:
  - id: narrow_head_on_001
    episode_path: /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark_ui/examples/narrow_head_on_001.json
```

`selection_mode`：

- `fixed`：固定一个场景，用于调试；
- `round_robin`：顺序均衡覆盖；
- `seeded_random`：固定 seed 的加权随机。

`max_episodes` 默认是 `10`。每个 episode 会创建带 session 唯一后缀的新行人实例，结束后移到停车区，不在同一 session 内复用 AnimGraph；达到上限后数采节点自动退出。`0` 表示不限制，但长时间运行会持续保留停车实例，不推荐用于正式采集。每个 session 结束后应重启 bridge，以释放这些实例。

默认人数、角色、出生点、路线、停留和机器人起终点全部来自 `episode_path` 指向的 JSON。
要改变人数或路线，应在 Route Editor 中修改并保存 EpisodeSpec。人工数采不再读取
小便池目标、toilet semantics 或旧 director 配置。

每轮 manifest 会保存 EpisodeSpec 的绝对路径及 SHA-256；这样 JSON 被修改后不会与旧数据
静默混用。人工数采执行链路不启动 `toilet_director_node`。

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
- `/isaac/pedestrian_contact_events`；
- `/isaac/scene_collision_events`。

默认 `detect_and_fail` 会在 session 开始时关闭机器人与行人双方的预测性人机 guard，
不提前裁剪机器人命令，也不让行人因该 guard 提前停车或绕行。零裕量运行时人体包络首次
接触机器人时，当前 episode 立即记录为 `robot_human_collision` 并结束。行人间约束和
行人与静态场景约束不受该开关影响。

正常退出数采节点时会恢复 bridge 的预测性 hard guard。若进程被 `kill -9` 强制终止，
应重启 bridge，或调用 `/isaac/set_pedestrian_hard_guard` 将其显式恢复。

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

bridge 稳定后，在另一个终端导入场景和机器人：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  spawn --phase physx_diff_contact
```

不要在第三个终端预启动 `toilet_authored_scenario`；下面的数采命令会自行启动它。

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
- 行人数、角色、路线 EpisodeSpec 路径及 SHA-256；
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
