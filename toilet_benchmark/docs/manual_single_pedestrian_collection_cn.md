# 单行人连续人工数采方案（第一版）

## 1. 目标与边界

本方案用于厕所场景第一轮人工社会导航数据采集。每个 episode 中：

1. 机器人在一个经过碰撞检查的安全点复位。
2. 一个预生成行人从门外进入，并前往指定小便池。
3. 操作者使用手柄控制机器人前往门口，主动完成减速、等待、绕行或让行。
4. 机器人到达门口后，该 episode 成功结束。
5. 若机器人与静态场景发生有效碰撞，则该 episode 标记失败并自动复位。
6. 若触发行人硬安全约束，则记录事件标签，但不把它等同于 PhysX 实体碰撞。
7. bridge、场景、机器人和行人 prim 在整个采集 session 中只初始化一次。

第一版不追求多人社会行为、HuNav 接入或完整奖励设计，优先保证流程稳定、数据可复现、事件可追踪。

## 2. 场景随机化策略

### 2.1 不采用两个独立随机目标

机器人起点和行人目标小便池不应分别在五个小便池中独立随机。独立随机容易产生以下问题：

- 机器人出生点与隔板、小便池或行人目标重叠。
- 双方路线几乎不相交，采不到有效交互。
- 某些组合天然过难或被静态几何封死。
- 难以均衡覆盖五个小便池，也不利于复现实验。

### 2.2 使用预验证的场景组合

配置中定义 `scenarios`，每个场景显式绑定：

- 机器人安全起点和朝向。
- 行人目标小便池。
- 机器人终点（门口）。
- 采样权重和是否启用。

第一版建议先验证左、中、右区域的 3 个组合，再扩展到五个小便池。最终可以做到五个小便池均有覆盖，但机器人起点仍必须来自预验证位置，而不是直接使用小便池站位。

支持三种选择模式：

- `fixed`：始终使用一个场景，适合单点调试。
- `round_robin`：按顺序均衡采样，作为第一版默认值。
- `seeded_random`：使用固定 seed 的加权随机，适合后续正式采集。

每个 episode 都记录 `scenario_id`、`pedestrian_target_urinal_id`、随机 seed 和抽样序号，从而完整复现采集顺序。

## 3. 代码和配置边界

数采编排属于 benchmark，不应放进场景启动脚本或机器人底层控制器。建议目录如下：

```text
toilet_benchmark/
├── config/
│   ├── toilet_benchmark.yaml
│   ├── toilet_semantics.yaml
│   └── manual_collection.yaml
├── docs/
│   ├── data_collection_stability_plan.md
│   └── manual_single_pedestrian_collection_cn.md
├── toilet_benchmark/
│   ├── toilet_director_node.py
│   ├── manual_collection_node.py
│   ├── collection_scenarios.py
│   └── episode_recorder.py
└── test/
    ├── test_collection_scenarios.py
    └── test_episode_recorder.py
```

职责划分：

- `manual_collection_node.py`：episode 状态机、服务调用、成功/失败判定和人工退出。
- `collection_scenarios.py`：配置校验、场景选择、固定 seed 和覆盖统计。
- `episode_recorder.py`：rosbag 进程、元数据、事件和原子落盘。
- `manual_collection.yaml`：场景组合、阈值、话题、输出目录和手柄操作配置。
- `toilet_director_node.py`：继续负责厕所语义和行人生命周期，不承担数据文件管理。
- Isaac bridge：只提供机器人软重置和底层碰撞/硬约束诊断接口。

不建议为第一版继续拆分 `mecanum_teleop.py`。只在其中接一个很薄的 reset 生命周期入口；具体复位事务可以放入独立辅助类或 service 文件，避免把数采状态机写入控制器。

## 4. Session 与 Episode 状态机

```text
SESSION_START
  -> INITIALIZE_WORLD
  -> PRESPAWN_PEDESTRIAN
  -> EPISODE_RESETTING
  -> EPISODE_READY
  -> EPISODE_RUNNING
       -> EPISODE_SUCCEEDED
       -> EPISODE_FAILED
       -> SESSION_STOP_REQUESTED
  -> EPISODE_SAVING
  -> EPISODE_RESETTING / SESSION_FINISHED
```

### 4.1 初始化

- bridge 和厕所场景只启动一次。
- 机器人只导入一次。
- 单个行人只预生成一次，非活动期间停放在远处 parking pose。
- 等待时钟、TF、雷达、机器人控制和行人服务全部 ready 后才允许开始。

### 4.2 Episode 开始

1. 根据选择模式取得一个 `scenario`。
2. 停止上一轮录制并确认文件关闭。
3. 将行人 park 到远处。
4. 请求机器人执行 PhysX 软重置。
5. 等待机器人位置、速度和轮速稳定。
6. 清空上一轮碰撞和硬约束事件状态。
7. 保持底层控制门控，启动本轮 rosbag 与元数据记录。
8. 确认 rosbag2 已订阅 odom、执行命令和双雷达等关键连续话题。
9. 将行人 re-activate 到门外入口，并下发本轮目标。
10. 收到 `pedestrian_active` 后解除控制门控，进入 `EPISODE_RUNNING`。

门控期间仍可记录原始手柄输入用于诊断，但底层控制器必须丢弃这些输入、持续输出零运动命令，且解除门控时不得恢复门控前缓存的旧命令。

### 4.3 Episode 结束

- 成功：机器人进入门口目标容差范围并满足低速条件。
- 失败：机器人与非地面静态场景发生有效碰撞，或出现超时/传感器失联。
- 行人交互：硬约束介入和重叠企图只记录标签，默认不自动判失败。
- 人工终止：保存当前 episode，标记 `aborted_by_operator`，然后安全关闭 session。

## 5. 机器人软重置

机器人每轮复位是可行的，但不能调用 `world.reset()`，也不能删除并重新导入机器人。全世界 reset 会影响时间线、TF、传感器和已有服务，违背连续采集目标。

应为 `physx_diff_contact` 提供一个事务化软重置接口：

1. 进入 `RESETTING`，暂时忽略手柄速度命令。
2. 将四轮目标速度和施力清零。
3. 清空速度平滑器、上一条命令、硬安全层历史状态和轮胎力瞬态。
4. 将 articulation 根速度、角速度和关节速度清零。
5. 使用实时 PhysX articulation 接口设置机器人根位姿，不能只修改 USD 可视 xform。
6. 再次清零速度，避免 teleport 后残留动量造成跳动。
7. 复用已有 settling 流程维持机械臂姿态并等待物理稳定。
8. 重建 odom/实际位姿估计基准。
9. 位置误差、线速度、角速度和轮速全部满足阈值后返回 ready。

复位期间不录制专家轨迹。仿真 `/clock` 保持单调递增，每个 episode 在元数据中记录绝对起始时间和本轮相对时间零点。

机器人起点必须通过三类预检查：

- 2D 静态占据/voxel footprint 不重叠。
- PhysX overlap 检查确认上层机身和机械臂不与场景相交。
- 轮子下方存在稳定地面接触。

## 6. 行人生命周期

第一版只使用一个预生成行人：

- session 初始化时 spawn 一次。
- episode 间调用现有 `park()` 移到远处。
- episode 开始时调用现有 `reactivate()` 回到入口。
- 每轮重新下发固定的入口到小便池路线和目标。
- session 结束时可以保留 prim，随 bridge 一起退出，不做高频增删。

这样可以避免 Isaac Sim 长时间反复创建和删除 AnimGraph/SkelRoot 导致的不稳定和段错误。

## 7. 数据与标签

建议每轮单独保存：

```text
collection_root/
└── session_YYYYMMDD_HHMMSS_seed42/
    ├── session_manifest.yaml
    ├── episode_000001/
    │   ├── metadata.yaml
    │   ├── events.jsonl
    │   └── rosbag2/
    └── episode_000002/
        └── ...
```

`metadata.yaml` 至少记录：

- session、episode、场景和操作者 ID。
- 配置文件哈希、代码 commit、seed 和抽样序号。
- 机器人起点/终点、行人入口/目标小便池。
- 开始/结束仿真时间、持续时间和结果。
- 成功、场景碰撞、超时、人工中止等终止原因。
- 硬约束介入次数、最小人机距离和事件摘要。

第一版建议录制：

- `/clock`
- `/tf`、`/tf_static`
- 机器人原始手柄命令和实际执行命令
- `/odom` 或实际机器人位姿/速度状态
- 两路或融合后的 LaserScan
- `/isaac/pedestrian_states`
- 行人状态机/目标资源状态
- 机器人硬安全层诊断事件
- 机器人与静态场景碰撞事件
- episode 状态和人工标签

硬约束事件应至少包含输入速度、裁剪后速度、介入原因、关联行人、距离和时间戳。第一版可使用标准诊断消息，避免新增自定义 rosidl 接口。

需要区分：

- `scene_collision`：PhysX 检测到机器人与非地面静态场景接触，通常判失败。
- `hard_guard_intervention`：软件硬安全层修改或停止命令，是训练标签，不等同于物理碰撞。
- `pedestrian_overlap_attempt`：机器人输入试图继续进入行人占据区，是更具体的风险标签。

元数据先写临时文件，episode 完成后原子重命名。人工退出或异常时也要关闭 rosbag，并保留 `incomplete/aborted` 状态，不能静默丢弃已采数据。

## 8. 第一版配置示例

```yaml
session:
  output_root: /home/stardust/resources/arena_datasets/toilet_manual
  operator_id: operator_01
  seed: 42
  selection_mode: round_robin  # fixed | round_robin | seeded_random
  max_episodes: 0              # 0 表示直到人工退出

episode:
  timeout_sec: 120.0
  goal_tolerance_m: 0.35
  goal_stop_speed_mps: 0.08
  settle_timeout_sec: 8.0
  settle_linear_speed_mps: 0.02
  settle_angular_speed_rps: 0.03

pedestrian:
  agent_id: toilet_agent_01

scenarios:
  - id: robot_left_ped_urinal_3
    enabled: true
    weight: 1.0
    robot_start: [1.8, 0.25, 3.14159]
    robot_goal: [-3.8, -0.91, 3.14159]
    pedestrian_target_urinal_id: urinal_3

  - id: robot_center_ped_urinal_5
    enabled: true
    weight: 1.0
    robot_start: [0.7, 0.20, 3.14159]
    robot_goal: [-3.8, -0.91, 3.14159]
    pedestrian_target_urinal_id: urinal_5
```

示例坐标只表达配置结构，不能直接作为最终采集点。正式启用前必须在当前 USD 场景中逐一完成碰撞、地面支撑和路线验证。

## 9. 实施顺序

### 阶段 A：配置与纯逻辑

- 增加 `manual_collection.yaml` 和场景配置校验。
- 实现 fixed、round-robin、seeded-random 三种选择器。
- 实现 episode manifest 和事件文件的原子写入。
- 用单元测试锁定 seed 复现、均衡覆盖和异常恢复。

### 阶段 B：机器人软重置

- 在 Isaac bridge 增加 contact 模式专用 reset 服务。
- 接入控制清零、articulation pose reset、settling 和 ready 响应。
- 验证连续 50 次复位无跳动、无时间线重置、无 odom 残留速度。

### 阶段 C：采集状态机

- 增加 `manual_collection_node.py`。
- 接通单行人 park/reactivate、机器人 reset、episode 成败判定。
- 接入手柄开始、跳过、重试和退出操作。

### 阶段 D：录制与事件

- 每轮启动并干净关闭 rosbag2。
- 发布并记录硬安全层、静态碰撞和 episode 状态。
- 验证正常退出、碰撞 reset、Ctrl-C 和 bridge 异常时的数据完整性。

## 10. 第一版验收标准

- 同一 bridge 下连续完成至少 30 个 episode，无 prim 增删和进程崩溃。
- 相同配置和 seed 得到相同场景序列及行人目标序列。
- round-robin 模式下各启用场景覆盖次数之差不超过 1。
- 机器人复位后位置、速度和轮速稳定，且不记录复位瞬态为专家轨迹。
- 场景碰撞能自动结束当前 episode 并开始下一轮。
- 硬安全层介入有时间戳和原因标签，不被误记为 PhysX 碰撞。
- 成功、失败、人工退出和异常中断均能保存可读取的 metadata、events 和 rosbag。

## 11. 暂不包含

- 多行人同时运行和排队行为。
- HuNav 社会力或复杂局部避障。
- 自动生成任意机器人出生点。
- 自动评价操作者行为是否“足够社会化”。
- 将软社会规则用于自动裁剪控制。

这些能力应在第一版连续采集链路稳定、数据格式验证完成后再逐步加入。

## 12. 当前运行命令

完成构建后，每个终端都先执行：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
```

保持现有 `physx_diff_contact` bridge、机器人 spawn 和手柄节点运行。先用不录 bag 的模式验证五个场景起点：

```bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml \
  --no-record
```

确认机器人复位点均无碰撞后，正式采集去掉 `--no-record`：

```bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml
```

运行期间直接使用手柄控制机器人。按 `Ctrl-C` 会中止当前 episode、停止 rosbag、park 行人并保存 session manifest。输出默认写入：

```text
/home/stardust/resources/arena_ws/data/toilet_manual/
```

如需只验证一个组合，将 `session.selection_mode` 改为 `fixed`，并通过 `session.fixed_scenario_id` 指定场景。
