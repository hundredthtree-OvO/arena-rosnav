# 厕所社会导航 Benchmark 总体设计

状态：Draft v0.2 / Architecture Freeze v0.1
最后更新：2026-08-01

## 1. 文档目的

本文是 `toilet_benchmark` 的 benchmark 规范唯一来源，定义：

- benchmark 测量的能力与明确不测量的能力；
- Replay Track 与 Interactive Track 的边界；
- 任务族、episode、数据划分和难度轴；
- 机器人 observation/action 接口；
- 仿真特权真值、指标和终止协议；
- baseline、结果格式和可复现要求；
- 当前成熟度、开发阶段和发布门槛。

实现细节和运行命令不在本文重复维护：

- HuNav 主链路运行见 `hunav_takeover_cn.md`；
- HuNav 隔离测试见 `hunav_phase0_smoke_cn.md`；
- HuNav 迁移实现计划见 `hunav_pedestrian_pipeline_migration_plan_cn.md`；
- 两条开发线的代码结构与删除门槛见 `code_structure_migration_cn.md`；
- 目的性行人生态、Smart Object、事件管线和冻结接口见
  `pedestrian_ecosystem_architecture_cn.md`；
- 行人运动与表现 backend 的对照门槛见 `motion_backend_evaluation_cn.md`；
- 人工数采见 `manual_collection_cn.md`；
- 历史失败约束见 `pedestrian_pair_guard_failure_review_20260724.md`。

## 2. Benchmark 定位

### 2.1 研究问题

第一版研究问题限定为：

> 在具有动态行人、局部遮挡、窄通道和厕所资源占用的室内环境中，使用双
> 2D LiDAR、相对目标和机器人自身速度的移动机器人，能否安全、高效并尽量
> 少干扰行人地到达目标？

本 benchmark 与开放场景随机行走 crowd 的主要差异是**目的性和资源约束**。行人不是
随机 waypoint 动态障碍，而是执行可观察生命周期的任务主体：

```text
产生意图 -> 进入 -> 选择/等待资源 -> 交互活动 -> 释放资源 -> 离开
```

随机性来自 episode 级的受控采样，例如到达时间、目标资源、服务时间、耐心、群组和
behavior profile；不能依靠逐帧随机决策制造“多样性”。厕所资源、portal、队列和等待区
统一建模为可预约的 Smart Object，运动 backend 不拥有这些对象的状态。

第一版重点测量：

1. 静态几何避障；
2. 动态碰撞预测；
3. 行人短时意图响应；
4. 窄空间会车和门口协商；
5. 目标区域被占用时的处理；
6. 长时序 Enter-Exit 任务；
7. 避免冻结、死锁和高频振荡；
8. 机器人对行人轨迹和到达时间的影响。

第一版明确不宣称测量：

- 视觉语义理解；
- 开放词汇任务理解；
- 跨机器人形态泛化；
- 跨建筑布局泛化；
- 大规模拥挤人群仿真；
- 完整的人类心理、文化和群体规范；
- 行人行为的真实世界统计一致性。

### 2.2 Benchmark 与仿真环境的区别

完整 benchmark 必须同时包含：

```text
Task distribution
Episode dataset
Standard policy interface
Fixed splits
Runtime and reset protocol
Ground-truth evaluator
Metrics
Baselines
Reproducible runner
Versioned result format
```

Isaac Sim、厕所 USD、机器人、HuNav 和 director 只是运行底座。只有场景可运行、
行人可移动或 rosbag 可录制，不能证明 benchmark 已成立。

### 2.3 Dataset 与 Evaluation 分离

本项目包含两个相关但独立的交付物：

1. `Dataset Track`
   - 操作者通过手柄控制机器人；
   - 保存双雷达、odom、动作、行人状态、碰撞和任务事件；
   - 用于模仿学习、离线分析和策略初始化。
2. `Evaluation Track`
   - 被测 policy 通过固定 observation/action 接口闭环运行；
   - 官方 runtime 负责 reset、step、termination 和真值记录；
   - 官方 evaluator 计算指标并聚合结果。

人工示范数据的数量和质量不应直接作为 benchmark 排名；同样，一个评测 policy
不应被允许读取只为 Dataset 标注而保存的特权真值。

## 3. 两条行人评测 Track

### 3.1 Replay Track

Replay Track 使用冻结的行人参考轨迹。行人不因被测机器人策略而改变高层轨迹，
仅在真实几何碰撞时产生 episode 失败。

用途：

- 严格比较动态碰撞预测；
- 比较路径效率、反应时间和控制平滑性；
- 消除不同局部行人模型带来的分布变化；
- 提供确定性较高的回归和下界测试。

要求：

- 轨迹文件、时间基准和动画 phase 版本化；
- 相同 episode 重放时，轨迹误差必须在规定容差内；
- 被测策略不可读取未来轨迹；
- replay actor 的未来状态只属于 evaluator/oracle。

Replay Track 的局限是行人不会自然回应机器人，因此不能测量真实的双向协商。

### 3.2 Interactive Track

Interactive Track 使用 HuNavSim 行为树和社会运动模型。每个 physics/control
step 将当前行人、机器人和局部障碍状态输入 HuNav，再把输出交给 Isaac adapter。

用途：

- 测量人机双向影响；
- 测量窄空间协商和行人让行/绕行；
- 测量机器人导致的行人延误、急停和轨迹偏离；
- 测量不同 HuNav 行为类型下的鲁棒性。

Interactive Track 中，不同机器人策略会导致不同的行人轨迹，这是预期行为。
可复现要求是：

> 相同 benchmark 版本、episode、seed、初始状态和机器人动作序列，应产生容差内
> 一致的行人状态序列。

不要求不同机器人策略下的行人轨迹相同。

### 3.3 HuNav 的职责

HuNav 只负责：

- 局部社会运动；
- 人-人和人-机器人连续交互；
- `regular`、`impassive`、`surprised`、`scared`、`curious`、
  `threatening` 等机器人反应；
- 配置了 `group_id` 时的群组社会力；
- 局部速度和朝向趋势。

HuNav 不负责：

- 小便池资源所有权；
- 排队顺序；
- 门口 token 或显式通行优先级；
- Entering/Using/Exiting 生命周期；
- episode split、reset 和评测；
- 机器人 observation/action 接口；
- benchmark 成功条件。

这些仍由 Scenario Director、global router、runtime 和 evaluator 分别承担。

### 3.4 行为不稳定对两条 Track 的影响

行人不自然或不稳定会直接影响 benchmark 有效性，但影响方式不同：

- Replay Track：主要影响轨迹资产质量；轨迹冻结并通过验收后不再受运行时 HuNav
  调参影响。
- Interactive Track：直接改变任务难度和指标，因此 HuNav adapter、行为配置和
  Isaac 动画执行必须冻结版本并通过统计验证。
- Dataset Track：不稳定行为会污染示范分布，必须在 episode manifest 中标记并
  剔除无效样本。

不能通过不断扩大硬安全距离来掩盖行人模型问题，因为这会使被测机器人面对的任务
分布发生变化。正式评测建议：

- 社会交互由 HuNav 软行为处理；
- 碰撞首次发生时记录失败；
- `collision_policy=terminate` 时立即终止 episode；
- 调试时可使用 `collision_policy=contain`，只阻止重叠继续加深并允许脱离；
- 硬安全层不得提前替行人或机器人做社会决策。

## 4. 任务体系

任务由事件模板组合，而不是由一条不断扩大的 director 状态机硬编码。模板约束语义、
资源、触发器和成功条件；全局路线、局部社会运动与动画表现由独立 backend 实现。

### 4.1 v0.1 核心任务

第一版只冻结三类任务，形成可运行的 vertical slice：

| 任务 | 能力 | 行人 Track |
| --- | --- | --- |
| Crossing / Head-on | 动态预测、基本会车 | Replay + Interactive |
| Doorway Conflict | 窄空间协商、避免死锁 | Replay + Interactive |
| Goal Occupied + Enter-Exit | 资源语义、长时序导航 | Interactive 为主 |

对应的首批事件模板冻结为：

- `EnterUseExit`；
- `DoorwayConflict`；
- `GoalOccupied`；
- `QueueAndPromote`；
- `StallExitBlocked`。

模板接口和 Smart Object 语义见 `pedestrian_ecosystem_architecture_cn.md`。v0.1 不要求
一次性发布全部模板，但新增 case 必须复用这些组合节点，不能增加新的逐帧位姿控制器。

### 4.2 后续扩展任务

| 任务 | 前置条件 |
| --- | --- |
| Queue Passing | 多人资源/队列状态和群体指标稳定 |
| Dense Flow | 多人 HuNav 与 reset 稳定，至少 30 分钟持续运行 |
| Occluded Pedestrian | 遮挡真值、可见性判定和传感器时序冻结 |
| Group Crossing | HuNav `group_id` 与群体空间指标完成 |
| Behavior Generalization | 六类 HuNav 行为进入固定 train/test split |

### 4.3 难度轴

每个 episode 显式记录，不在运行时隐式随机：

- 行人数量；
- 行人速度；
- 人机初始距离；
- TTC；
- 通道有效宽度；
- 遮挡持续时间；
- 行人 behavior type；
- 行人群组关系；
- 机器人最大速度和加速度；
- 传感器噪声；
- 起终点最短路长度；
- 目标区域是否被占用。

难度标签由生成器根据固定规则计算，不能由人工主观填写 `easy/medium/hard` 后不保存
构成因素。

## 5. Episode 规范

### 5.1 最小 Schema

```yaml
schema_version: toilet-social-nav-0.1
benchmark_version: 0.1.0
episode_id: doorway_seen_000042
scene_id: shenxinfu_841837
task_type: doorway_conflict
track: interactive
seed: 42

robot:
  model: xms_mecanum
  start_pose: [x, y, z, yaw]
  goal_pose: [x, y, yaw]
  max_linear_speed_mps: 0.8
  max_angular_speed_rps: 1.0

pedestrians:
  - id: toilet_agent_01
    character: original_female_adult_business_02
    start_pose: [x, y, z, yaw]
    semantic_goal: urinal_3
    behavior:
      type: regular
      configuration: custom
      walking_speed_mps: 0.8
      group_id: -1

termination:
  timeout_sec: 120.0
  goal_tolerance_m: 0.35
  collision_policy: terminate

difficulty:
  pedestrian_count: 1
  doorway_width_m: 0.9
  initial_ttc_sec: 2.1
  occluded: false

assets:
  scene_hash: "..."
  semantics_hash: "..."
  behavior_config_hash: "..."
```

### 5.2 Episode Validator

发布前每个 episode 必须自动检查：

- schema 和字段类型正确；
- ID 唯一；
- 起点、目标和初始行人位姿处于可行区域；
- 初始状态无静态或动态重叠；
- robot goal 可达；
- 行人 semantic goal 存在；
- replay 轨迹完整且时间戳单调；
- interactive behavior 配置合法；
- oracle 在限定时间内可解决；
- episode 不与同 split 或其他 split 重复；
- 资产/config hash 与 benchmark manifest 一致。

### 5.3 数据划分

只有一个厕所布局时，不宣称 unseen-scene：

```text
train:
  seen geometry + seen behavior/density
validation:
  held-out seeds and initial configurations
test_seen:
  held-out episodes from known distributions
test_behavior:
  held-out HuNav behavior types or parameters
test_density:
  held-out pedestrian counts/arrival rates
```

增加至少 2～3 个独立厕所布局后，再增加 `test_unseen_scene`。

测试集 episode manifest 应冻结。测试时是否公开完整配置取决于是否建设 leaderboard；
本地研究阶段可以公开，但必须保留不可修改的 hash。

## 6. Observation、Action 与时序

### 6.1 v0.1 LiDAR Track

建议策略输入：

```text
front_scan
rear_scan
relative_goal = [distance, bearing]
robot_velocity = [vx, vy, wz]
```

需要冻结：

- LaserScan 点数、角度范围、最小/最大距离；
- scan frame 和机器人 base frame；
- 发布频率和时戳语义；
- 无回波、NaN 和 Inf 的编码；
- 是否提供历史帧；
- goal 的坐标系；
- observation 同步和最大延迟。

### 6.2 Action

统一高层 action：

```text
[vx, vy, wz]
```

所有方法通过同一机器人控制后端映射到 PhysX 轮地执行，不允许部分方法直接设置 root
pose 或 root velocity。

需要冻结：

- control frequency；
- action bounds；
- 速度、加速度和 jerk 限制；
- command timeout；
- 松手/零动作制动行为；
- 延迟和丢帧处理；
- episode reset 期间 action gate。

### 6.3 信息权限

`Policy Observation`：

- 双雷达；
- 相对目标；
- 机器人自身速度；
- track 明确允许的地图或历史帧。

`Evaluation Ground Truth`：

- 机器人和行人真实位姿/速度；
- PhysX contact；
- 最小距离；
- 行人目标、behavior 和任务阶段；
- 可见性/遮挡；
- 机器人和行人轨迹；
- 接触力。

`Oracle-only`：

- 行人未来轨迹；
- 完整动态地图；
- 未来行为状态；
- 全局最优动态解。

世界坐标目标不是绝对禁止项。是否允许由 track 定义；同一排名中的方法必须使用相同
目标表达。

## 7. Runtime 与 Reset 协议

标准 step：

```text
validate action
apply robot command
update scenario/director
update pedestrian model
step physics
sample sensors and truth
update metrics
evaluate termination
return observation and public info
```

Reset 必须恢复：

- 机器人 root、关节、轮速和控制器积分状态；
- 行人位姿、速度、动画 phase、HuNav BT blackboard 和随机状态；
- director 资源、队列、portal token 和生命周期；
- 动态物体状态；
- contact/cache 和上一 episode 的命令；
- sensor warm-up 和 observation ready gate；
- evaluator 累积状态。

Reset 验收：

1. 同一 episode 连续 reset 20 次；
2. reset 后首帧状态在规定容差内一致；
3. 无残留 action；
4. 无上一 episode 行人/碰撞事件；
5. 不频繁增删 USD prim；
6. bridge 长时间运行不崩溃；
7. 相同动作序列的状态摘要 hash 在容差内一致。

## 8. 终止与碰撞协议

标准终态：

- `success`：机器人进入目标区域且速度稳定；
- `robot_human_collision`；
- `robot_scene_collision`；
- `timeout`；
- `deadlock`；
- `invalid_simulation`；
- `operator_abort`，只用于 Dataset Track；
- `runtime_error`。

正式 Evaluation Track 推荐：

```yaml
collision_policy: terminate
```

首次有效碰撞写入真值事件并终止，不允许把碰撞后的穿模轨迹继续计入有效结果。

调试使用：

```yaml
collision_policy: contain
```

只阻止进一步加深重叠，并允许后退或侧向脱离。`contain` 结果不得与正式
`terminate` 排名混合。

## 9. 指标

### 9.1 v0.1 必选指标

任务完成：

- Success Rate；
- Completion Time；
- Navigation Error；
- SPL；
- Timeout Rate。

安全：

- Robot-Human Collision Rate；
- Robot-Scene Collision Rate；
- Minimum Human Distance。

运动质量：

- Path Length；
- Stationary Time；
- Deadlock Rate；
- Linear/Angular Jerk。

### 9.2 Interactive Track 指标

- Personal-space Violation Time；
- 行人急停次数；
- 行人速度变化；
- 行人轨迹偏离；
- 行人到达时间增加量；
- 人机互相冻结持续时间；
- HuNav behavior transition 次数。

行人扰动应使用 counterfactual：

```text
Delta T_human = T_with_robot - T_without_robot
```

同一 episode 先运行无机器人或远离机器人基线，保存行人的参考到达时间和路径；再运行
被测策略。counterfactual 的行为 seed、初始状态和配置必须一致。

### 9.3 聚合层级

必须同时输出：

```text
episode
task type
scene
difficulty bucket
track
overall
```

overall score 只能作为辅助排名，不能隐藏碰撞率、任务成功率和行人扰动之间的权衡。

报告均值、标准差和置信区间；失败 episode 不重试，除非明确判定为
`invalid_simulation/runtime_error`。

## 10. Baseline

v0.1 最小 baseline：

1. `Stop`：验证终止和安全指标下界；
2. `Straight/Reactive`：验证任务不是只靠直行即可完成；
3. `Nav2/DWA` 或当前经典导航主链路；
4. 一个社会局部规划 baseline；
5. `Oracle`：允许使用特权状态和未来轨迹，仅估计上界。

学习 baseline（PPO、IL、CrowdNav 等）在 runner、episode 和指标冻结后再加入。

有效 benchmark 应满足：

```text
simple baseline < strong baseline < oracle
```

若所有 baseline 都接近满分，任务区分度不足；若 oracle 仍频繁失败，应先修复 episode
可解性或 runtime，而不是继续增加难度。

## 11. 结果和复现

每个 episode 输出一条 JSONL：

```json
{
  "benchmark_version": "0.1.0",
  "method": "nav2_dwa",
  "episode_id": "doorway_seen_000042",
  "track": "interactive",
  "seed": 42,
  "success": true,
  "termination_reason": "success",
  "completion_time_sec": 14.2,
  "path_length_m": 9.81,
  "spl": 0.91,
  "robot_human_collisions": 0,
  "min_human_distance_m": 0.47,
  "deadlock": false
}
```

运行 manifest 至少保存：

- benchmark、episode schema 和 asset 版本；
- git commit 和 dirty diff hash；
- ROS、Isaac Sim、HuNav 版本；
- profile/config hash；
- GPU/CPU；
- policy 名称和参数；
- observation/action track；
- control/sensor frequency；
- 每个 episode 的 seed；
- 缺失、重复和 invalid episode。

官方聚合器必须检查：

- episode 是否完整；
- 是否重复或未知；
- benchmark/version/hash 是否匹配；
- 结果字段和数值是否合法；
- 是否使用正确 track；
- 是否超过计算预算；
- 聚合统计是否可重算。

## 12. 当前成熟度

| 组成 | 当前状态 | v0.1 缺口 |
| --- | --- | --- |
| 场景/机器人/传感器 | 已有主链路 | 资产碰撞和传感器验收表 |
| Director/厕所语义 | Enter-Exit 原型可用 | 任务 API 和事件 schema |
| HuNav | takeover 和测试入口已存在 | 原生 behavior/BT、障碍输入和统计稳定性 |
| 人工数采 | 可连续录制并保存 manifest | 数据有效性 validator 和冻结 schema |
| Episode | 有运行时 scenario 模板 | 正式枚举数据集和 split |
| Reset | robot reset、park/reactivate 已有 | 一致性与长时间回归 |
| Policy interface | ROS topic 已存在 | 版本化 observation/action contract |
| Metrics | 零散事件和终止判断 | step metric、官方 evaluator 和聚合器 |
| Baselines | 未形成统一结果 | 至少四级 baseline |
| 发布协议 | 未建立 | benchmark card、版本和结果格式 |

当前正确称谓是：

> 厕所社会导航仿真、交互行人和人工数采原型。

在 episode dataset、接口、evaluator、split 和 baseline 冻结前，不应宣称为正式公开
benchmark。

## 13. 推进阶段

### Phase B0：冻结 Benchmark Card

交付：

- 本文评审完成；
- v0.1 任务范围和不测范围冻结；
- Replay/Interactive/Dataset 三者边界冻结；
- collision policy 和信息权限冻结。

退出条件：

- 新功能可以明确归属到 task、runtime、pedestrian、evaluator 或 dataset；
- 不再用“让画面更自然”替代可测验收条件。

### Phase B1：Episode Schema 和 Validator

交付：

- schema；
- manifest；
- deterministic generator；
- validator；
- 初始 train/validation/test 划分；
- 至少每个核心任务 20 个可解 episode。

退出条件：

- oracle 可解决绝大多数 episode；
- 无初始碰撞、不可达和重复 episode；
- 相同 seed 生成相同 manifest。

### Phase B2：标准 Runtime 和 Reset

交付：

- policy adapter；
-统一 reset/step/termination；
- truth recorder；
- Replay Track runner；
- Interactive Track runner。

退出条件：

- 单 bridge 连续运行至少 30 个 episode；
- reset 一致性测试通过；
- 策略不能读取 evaluator truth。

### Phase B3：Metrics 和 Evaluator

交付：

- v0.1 必选指标；
- Interactive 扰动指标；
- episode JSONL；
- validator/aggregator；
- 自动结果表。

退出条件：

- 指标有单元测试和手算样例；
- 相同原始轨迹始终得到相同指标；
- 缺失、重复或非法结果会被拒绝。

### Phase B4：Baselines 和任务有效性

交付：

- Stop、Reactive、经典导航、social baseline、oracle；
- 每个 task/split 的结果；
- 失败分类和置信区间。

退出条件：

- benchmark 能区分方法；
- oracle 上界合理；
- 任务既不是普遍满分，也不是普遍不可解。

### Phase B5：扩展

在 B0-B4 完成后再增加：

- Queue Passing；
- Dense Flow；
- Occluded Pedestrian；
- 多厕所布局；
- unseen-scene；
- 学习 baseline；
- vectorized/distributed evaluation；
- leaderboard。

## 14. 近期工作优先级

当前优先顺序：

1. 保持外部 CLI、ROS topic/service 和现有 domain/episode 契约兼容；
2. 提取 `SmartObjectRegistry + AgentExecutive + ScenarioRuntime`，先兼容复现
   `EnterUseExit`；
3. 将 director 收缩为 ROS、配置和生命周期 facade，本阶段不修改运动表现；
4. 提取 GlobalRouter、LocalMotionBackend、GeometrySafety 和 EmbodimentAdapter；
5. 使用统一微场景矩阵比较 HuNav、ORCA/HRVO 候选和 Replay，不再向 HuNav 热路径增加
   事件特判；
6. 保持 Replay Track 的确定性基线，并从合格 episode 冻结轨迹资产；
7. 使用同一 MotionCommand A/B 对照 Isaac AnimGraph 与 SMPL-H；
8. 实现五类事件模板、runner、evaluator 和 baseline 后再扩展随机场景生成。

行人系统不需要达到“与真人不可区分”才开始 benchmark 工程，但进入正式测试集前必须达到：

- 无无原因瞬移、穿墙和长时间原地旋转；
- 相同输入可重放；
- 行为失败可通过事件和终态识别；
- 多人不会因 roster/reset 产生跳变；
- 行为参数和 HuNav/adapter 版本固定；
- 无效仿真不会被计为 policy 失败。

## 15. 参考

- HuNavSim 2.0: <https://arxiv.org/abs/2507.17317>
- GROVE: <https://arxiv.org/abs/2606.25504>
- Menge: <https://gamma.cs.unc.edu/Menge/>
- ORCA: <https://gamma-web.iacs.umd.edu/ORCA/>
- Isaac Sim People Simulation:
  <https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/ext_replicator-agent/ext_omni_anim_people.html>
- AMASS / SMPL-H: <https://amass.is.tue.mpg.de/>
- SocNavBench: <https://arxiv.org/abs/2103.00047>
- Arena-Bench: <https://arxiv.org/abs/2206.05728>
- Arena 3.0: <https://arxiv.org/abs/2406.00837>
