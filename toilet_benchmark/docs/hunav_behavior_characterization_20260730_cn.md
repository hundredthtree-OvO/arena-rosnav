# HuNav 原生 Behavior Isaac 对照记录

日期：2026-07-30  
状态：S3-A characterization

## 1. 目的

本轮使用自动化 `bridge -> spawn -> director -> cleanup` 流程，对比
`regular`、`surprised` 和 `scared`。所有有效对照均使用：

- scene/profile：`shenxinfu_841837 / physx_diff_contact`
- `arrival.seed=12345`
- `initial_agents=1`
- `target_resource=urinal_1`
- 自研 `robot_proximity_reaction=disabled`
- 原生-only 对照额外设置 `regular_avoidance=disabled`

日志保存在 `/tmp/toilet_pipeline_runs/`。

## 2. 结果

| Run | 配置 | 完成 | robot contact | static clip | stall | 关键证据 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `20260730_132542_regular_disabled` | regular，旧 avoidance 保留 | 是 | 0 | 9 | 0 | `BTRegularNav.xml` |
| `20260730_132806_surprised_disabled` | surprised，duration=60 | 是 | 30 | 31 | 0 | active 60 samples，随后 timer expired |
| `20260730_133141_scared_disabled` | scared，duration=0 | 是 | 0 | 26 | 0 | `Timer of 0 seconds has expired` |
| `20260730_133720_scared_disabled` | scared，duration=60，原生-only | 是 | 0 | 32 | 0 | `BTScaredNav.xml` 正确加载，无零秒 timer |
| `20260730_134048_regular_disabled` | regular，原生-only | 是 | 0 | 17 | 0 | 旧 reaction/avoidance 均关闭仍完成 |
| `20260730_144920_surprised_disabled` | surprised，原生-only，固定 crossing 干预 | 是 | 37 | 12 | 0 | 自动进入/保持/移开；urinal 与 exit 均完成 |
| `20260730_150247_scared_disabled` | scared，原生-only，固定 crossing 干预 | 是 | 54 | 14 | 0 | active 可观测 1 sample；urinal 与 exit 均完成 |
| `20260730_150752_regular_disabled` | regular，原生-only，固定 crossing 干预 | 是 | 34 | 3 | 0 | 同条件基线；urinal 与 exit 均完成 |

不同 run 的 static clip 数量会受到连续动力学和路径细节影响，不能单独作为 behavior
优劣指标。当前更可靠的退出门槛是完成、接触、stall、恢复耗尽和 BT 分支证据。

## 3. 结论

### 3.1 Regular

原生-only regular 单行人流程通过。说明自研 `regular_avoidance` 不是完成厕所
Enter-Exit 的必要条件，可以进入 shadow/逐步关闭候选。

当前证据不足以直接删除它。删除前仍需：

- 主动将机器人放在行人路径上；
- 门口冲突；
- 双行人；
- 至少连续 10 次固定 seed/扰动 seed 回归。

### 3.2 Surprised

`duration=4` 只能证明 BT 被激活；早期静止机器人实验中，`duration=60` 能维持
active。固定 crossing 干预进一步暴露了上游可见性语义：机器人进入后状态仅短暂
`inactive -> active -> inactive`。角色停止或转向后，HuNav 的前向视野判断会重新失败，
因此 duration 并不能保证行为持续。

因此定时器不是正式的“机器人可见期间持续 hold”语义。正式实现应使用：

```text
robot visible -> surprised hold
robot not visible -> regular
```

在修改 BT 前，可以用大于 episode timeout 的 duration 作为兼容方案，但自动测试必须在
有限时间后把机器人移开，否则“正确持续 hold”和“测试永不完成”无法区分。

### 3.3 Scared

零秒 duration 已修正为 60 秒。HuNav manager 能正确加载 `BTScaredNav.xml`，且不再输出
零秒 timer 过期。

旧版本中 `native_behavior_state` 始终显示 inactive。原因不是 YAML 或 adapter：上游
`AgentManager::avoidRobot()` 先设置 state，随后调用 `computeForces()`，而后者在返回消息
前将 state 清零。当前工作区已在 `avoidRobot()` 返回前恢复 active state，并增加独立
behavior transition JSONL。同一 crossing 干预已证明 scared active state 可观测，但也只
持续 1 个采样周期；它与 surprised 共享可见性保持缺陷。

因此 state 观测链路已经修通，但 behavior 语义尚未通过：

- active transition 可作为 BT 分支证据；
- 单个 active sample 不能视作完整 scared 反应；
- 修复触发保持后才能用 active duration 比较 surprised/scared。

## 4. S3-A 决策

本轮不删除旧策略，只完成第一类关闭实验：

- `robot_proximity_reaction`：关闭；
- `regular_avoidance`：在原生-only regular/scared run 中关闭；
- hard safety：保留；
- 旧文件和 YAML：保留到主动阻挡及多人回归完成。

当前最适合作为第一批删除候选的是 `regular_avoidance`，但必须先补齐受控机器人干预。

## 5. 下一步

1. 修复 surprised/scared 的触发保持语义，避免角色 yaw 变化令前向视野条件抖动。
2. 已固化 `crossing_clear` 与 `crossing_conflict` 两个动态 crossing profile，并新增
   顺序矩阵运行器；固定 seed 的单行人矩阵已完成，下一步修复原生 behavior runtime
   后复跑单行人，再扩展到双行人。
3. 将 urinal/exit 的真实 Isaac 位姿终点捕获作为固定回归，禁止提议位置假 settled。
4. 已增加中心 clearance 与视觉骨骼穿模的分离观测；下一步运行固定矩阵建立阈值。
5. 再运行双行人和门口冲突；通过后才默认关闭或删除 `regular_avoidance`。

矩阵命令：

```bash
ros2 run toilet_benchmark hunav_behavior_matrix \
  --behaviors regular,surprised,scared \
  --agent-counts 1,2 \
  --intervention-profiles crossing_clear,crossing_conflict \
  --seeds 12345 \
  --output-root /tmp/toilet_behavior_matrix
```

Isaac 子任务必须顺序执行；这里的“并行推进”是 Lane A 与 Replay Lane B 的开发并行，
不是同时启动多个 Isaac bridge。

## 6. 终点与静态几何复验

`20260730_144920_surprised_disabled` 修复了 HuNav 以较高速度越过 urinal 后进入隔板
区域的问题。backend 现在只在真实 Isaac 根位姿进入 `settled_arrival_tolerance_m`
后切换 `TERMINAL_ALIGN`，并在报告 settled 时再次校验位置。

本轮证据：

- urinal 到达 1 次，exit 到达 1 次；
- stall 0，recovery exhausted 0；
- 离开 urinal_1 后窄通道发生 12 次 tangent static projection，但未冻结；
- 中心轨迹完成不等于角色手臂网格绝不穿模，后者需要单独的视觉/骨骼包络指标。

## 7. 固定 crossing 三行为矩阵

三组 run 使用相同 seed、目标、机器人进入位置和 6 秒保持：

| Behavior | 完成 | active samples | robot contacts | static clips |
| --- | ---: | ---: | ---: | ---: |
| regular | 是 | 0 | 34 | 3 |
| surprised | 是 | 1 | 37 | 12 |
| scared | 是 | 1 | 54 | 14 |

该矩阵证明自动化流程和终点捕获稳定，但不能证明 surprised/scared 行为自然。两者没有在
6 秒干预窗口内持续 active，接触也没有优于 regular。S3-A 下一项必须修正 BT 的
“首次看见后保持，明确条件释放”语义，而不是继续调 social-force 权重。

## 8. 动画骨骼外包络观测

Isaac bridge 增加了只读 topic：

```text
/isaac/pedestrian_visual_envelopes
```

它不使用固定圆代替角色。USD `UsdSkel` 只负责提供关节顺序和父子拓扑，当前世界坐标
由 AnimGraph `character_graph.get_joint_transform()` 读取；随后沿父子骨段插值采样，
再用 PhysX `overlap_sphere` 查询静态碰撞。角色自身、机器人和地面碰撞体会被过滤。

每个 agent 的 JSON 记录包含：

- `joint_count`、`sample_count`；
- `xy_aabb` 和骨架采样点凸包 `hull`；
- `overlap_sample_count`、`overlap_ratio`；
- `overlap_paths`；
- 单帧骨段上连续命中数量 `max_consecutive_overlap_samples`。

自动 smoke runner 会将原始记录写入：

```text
<run_dir>/pedestrian_visual_envelope.jsonl
```

并在 `summary.json` 中输出跨时间的重叠帧数、最大连续重叠采样数和 prim 命中统计。
这些字段当前只用于 characterization，不参与路径、速度、HuNav force 或 hard safety。

当前限制：

- 纯 Python 测试已经覆盖骨段采样、凸包、过滤和汇总；
- Isaac 4.5 固定 seed 实测已确认 AnimGraph 运行时关节查询可用；仍需多角色、多 seed
  统计可用率与开销；
- 探测球半径是视觉表面近似参数，不是新的运动碰撞半径；
- 在获得 live 分布前，不把单帧接触直接定义为 benchmark 失败。

固定 seed `12345`、`regular`、单行人 `urinal_1` 的首个可用基线位于：

```text
/tmp/toilet_pipeline_runs/20260730_171156_regular_disabled
```

该运行正常完成入口、小便池和出口流程，163/163 个 agent 样本可用，无 stall 或恢复
耗尽。过滤脚底地面接触后，共有 5 帧环境重叠：大门 3 帧，`Partition_0003` 和
`Partition_0004` 各 1 帧；最大单帧重叠率为 `0.076923`。这组结果仅作为
characterization 基线，不是最终 benchmark 判定阈值。

## 9. 校准后的单行人动态 Crossing 矩阵

2026-07-30 重新校准了两个 profile：

- `crossing_clear`：机器人在行人走廊侧方穿越，完成脉冲后移到清场位姿；
- `crossing_conflict`：机器人从更靠近走廊的位置稍早出发，形成可复现强制交汇，
  完成后同样清场。

启动 runner 还增加了当前 bridge 的 `Simulation App Startup Complete` 日志门控，
避免上一轮 ROS graph 残留服务导致 spawn 提前发送。

固定 seed `12345`、单行人、`urinal_1` 的结果：

| Behavior | Profile | 完成 | robot contacts | hard safety | native active | stall |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| regular | clear | 是 | 0 | 4 | 0 | 0 |
| regular | conflict | 是 | 26 | 32 | 0 | 0 |
| surprised | clear | 是 | 0 | 1 | 0 | 0 |
| surprised | conflict | 是 | 13 | 18 | 0 | 0 |
| scared | clear | 是 | 0 | 9 | 0 | 0 |
| scared | conflict | 是 | 17 | 20 | 0 | 0 |

证据目录：

```text
/tmp/toilet_crossing_conflict_recalibrated/20260730_195053_regular_disabled
/tmp/toilet_behavior_matrix_single_calibrated_20260730/20260730_195338_483946_behavior_matrix
```

结论：

1. clear/conflict 已能稳定区分负对照和强制冲突，动态干预时机不再是主要变量；
2. 六轮都正常完成 urinal 和 exit，无 stall 或 recovery exhausted；
3. surprised/scared 即使发生真实机器人接触，`native_behavior_active_samples` 仍为 0；
4. 下一步应修正 HuNav behavior/BT 的 runtime 激活入口，不能再用硬安全接触数代替
   behavior 已生效的证据。

### 2026-07-30 复用 bridge 复测

- smoke runner 支持 `--reuse-existing-bridge --skip-spawn`，避免行为矩阵反复初始化
  Isaac Sim 4.5。
- dynamic crossing 改为先把机器人放在清场位姿，待行人全局路线建立后再进入交汇；
  避免 Theta* 在测试开始前就绕开机器人，使 behavior 刺激失效。
- `surprised + crossing_conflict + seed=12345` 完整结束：
  `native_behavior_active_samples=5`、`robot_contact_events=19`、
  `stall_events=0`、`recovery_exhausted_events=0`。
- `crossing_conflict` 是强制冲突压力测试，不应以零接触为验收条件；
  `crossing_clear` 继续承担无接触会车基线。

## 10. 原生 Behavior 状态回写修复

backend 现在缓存每个 agent 由 HuNav `ComputeAgents` 实际返回的
`AgentBehavior.state`，并仅在 behavior profile 未变化时带入下一次请求和诊断。
该状态不再由本地 `robot_proximity_reaction` 距离阈值合成；原生 BT 仍由
`IsRobotVisible(dist)`、behavior type 和 timer 决定是否激活。

纯 Python 回归已经覆盖：

- HuNav 返回 `BEH_ACTIVE_1` 后，diagnostics 观察到真实 active；
- 下一帧构造 agent 时不再无条件覆盖为 inactive；
- agent 移除后清除状态缓存，避免同名新实例继承旧行为。

2026-07-30 的两次 live smoke 重试均在 Isaac Sim 启动约 5 秒时崩溃，栈位于
URDF importer UI 与 OmniGraph 扩展扫描，尚未进入 bridge startup marker、spawn 或
HuNav。因此本节当前状态是“实现和单元回归完成，Isaac live 验收受环境启动段错误
阻塞”，不能据此宣称 surprised/scared 的原生动作效果已通过。

## 11. 冻结的单行人基线

同一 `physx_diff_contact` bridge、固定 seed `12345`、固定 `urinal_1`，顺序执行
`regular/surprised/scared x crossing_clear/crossing_conflict`：

```text
/tmp/toilet_behavior_matrix_single_frozen_20260730/
  20260730_231321_200528_behavior_matrix
```

结果：

- 6/6 完成 urinal 和 exit；
- 6/6 的 `stall_events=0`、`recovery_exhausted_events=0`；
- 三个 clear case 均 `robot_contact_events=0`；
- surprised/scared conflict 分别观察到 1/2 个原生 active diagnostics 样本；
- static projection 仍会发生，但本轮没有导致流程失败；
- 当前复用 bridge 未提供有效视觉骨骼外包络样本，该项仍是观测缺口。

因此单行人完成性满足进入双行人 characterization 的门槛，但不能据此宣称视觉穿模
已经通过。双行人首轮固定不同目标 `urinal_1,urinal_5`，只观测多人影响，不在同一轮
修改运动策略。

新增 summary 指标：

- 每个 agent 的激活位置偏差、停止次数/时长、路线进度倒退和 behavior 错配；
- 行人对最小中心距离、`0.60 m` characterization 代理重叠样本；
- 双方同时停止时长、行人间 hard-contact diagnostics 样本；
- 各自 urinal/exit 终态数量以及 stall/recovery。

### 首轮双行人结果

固定 `regular + crossing_clear + seed=12345`，目标按顺序固定为
`urinal_1,urinal_5`：

```text
/tmp/toilet_behavior_matrix_dual_characterization_20260730/
  20260730_232822_095622_behavior_matrix
```

结果：

- 两人均到达各自 urinal，均到达 exit，director 正常退出；
- 最小中心距离 `1.095 m`，`0.60 m` 代理重叠样本 0，行人间 hard contact 0；
- 两人路线进度倒退、停止段、behavior profile 错配均为 0；
- 双方共同停止时长 0，stall 和 recovery exhausted 均为 0；
- 激活落点最大误差 `0.040 m`；
- 第二人激活前后，第一人在 diagnostics 跨帧位移 `0.277 m`，按观测速度和时间间隔
  计算的 jump excess 为 0。

最后一项只能排除 1 Hz diagnostics 可见的跳变，不能排除一个采样周期内发生并恢复的
瞬态。需要更高频 raw pose 记录后才能把“激活绝不跳变”升级为正式验收项。

## 12. 高频双行人固定矩阵

2026-07-31 在同一 `physx_diff_contact` bridge 内固定 seed `12345`，目标固定为
`urinal_1,urinal_5`，顺序执行：

```text
regular/surprised/scared x crossing_clear/crossing_conflict

/tmp/toilet_behavior_matrix_dual_full_20260731_v2/
  20260731_012205_840801_behavior_matrix
```

本轮新增独立 ROS recorder，不依赖 bridge 启动时的日志环境变量：

- `/isaac/pedestrian_states` 原始位姿以约 `9.29-9.50 Hz` 写入
  `pedestrian_states.jsonl`；
- `/isaac/pedestrian_visual_envelopes` 写入
  `pedestrian_visual_envelope.jsonl`；
- raw pose 只统计 agent 激活至 EXITING 对齐之间的有效窗口，排除远端 parking pose；
- peer activation jump 使用激活时刻两侧不超过 `0.5 s` 的原始位姿样本计算。

| behavior | crossing | 完成 | raw 最小中心距 | `<0.60 m` 时长 | 激活 jump excess | 原生 active 样本 | 视觉重叠样本 |
|---|---|---:|---:|---:|---:|---:|---:|
| regular | clear | 是 | `0.403 m` | `2.13 s` | `0.000 m` | 0 | 13 |
| regular | conflict | 是 | `0.808 m` | `0.00 s` | `0.000 m` | 0 | 16 |
| surprised | clear | 是 | `1.211 m` | `0.00 s` | `0.000 m` | 0 | 29 |
| surprised | conflict | 是 | `0.798 m` | `0.00 s` | `0.000 m` | 1 | 15 |
| scared | clear | 是 | `0.622 m` | `0.00 s` | `0.000 m` | 0 | 16 |
| scared | conflict | 是 | `0.779 m` | `0.00 s` | `0.000 m` | 3 | 8 |

流程层结果：

- 6/6 均有两个 urinal arrival、两个 exit arrival，进程正常结束；
- 6/6 的 `stall_events=0`、`recovery_exhausted_events=0`；
- 所有 case 的 behavior profile mismatch 和高频 peer activation jump excess 均为 0；
- `regular-clear`、`regular-conflict`、`surprised-clear` 分别仍有 2、1、2 次低频路线
  进度回退，其余三轮为 0；
- `regular-conflict` 有 21 个 robot contact diagnostics；surprised/scared conflict
  避免了该接触，但原生 active 只持续到 1/3 个 diagnostics 样本，行为保持仍不足。

几何层结果：

- 先前首轮双行人仅依靠约 1 Hz diagnostics 得到的“无重叠”结论被高频数据推翻；
  `regular-clear` 实际出现 `0.403 m` 最小中心距和约 `2.13 s` 代理重叠；
- 六轮均获得有效视觉骨骼外包络，但六轮也均存在静态几何 overlap；
- overlap 主要集中在大门门板/门框，其次是墙体，少量位于隔板和
  `Edestalurinal_0001`；单轮最大连续 overlap 为 5 至 12 个 2 Hz 样本；
- 这不是简单扩大圆形 footprint 能正确解决的问题，需要按骨骼外包络命中的部位、
  门洞通行阶段和静态几何路径分别定位。

因此，本轮只通过“多人流程完成性、无激活跳变、无 stall/recovery”门槛，**没有通过
双行人 Interactive 稳定性门槛**。下一阶段仍停留在 S3-A：

1. 先定位 `regular-clear` 的高频同行重叠，区分共享入口时序、HuNav pair force 和
   external-motion 执行延迟；
2. 按 door/wall/partition/urinal 分类回放视觉 overlap 帧，确认真实骨骼穿模与
   conservative envelope 的比例；
3. 延长并验证 surprised/scared 原生 active 的保持/释放语义；
4. 修复后至少用 3 个固定 seed 重跑同一双人矩阵，再决定是否进入门口冲突和更多人数。

## 13. 不可通行走廊的单方让行回归

此前的双人矩阵没有覆盖一个重要反例：机器人静止在通道一侧、另一名行人固定占用
小便池时，后方行人面对的不是“左右任选一侧绕行”，而是联合外包络已经封闭的
不可通行走廊。正确行为应是提前稳定等待，而不是依靠社会力继续挤压，最终被推向
相邻小便池或在静态安全层之间反复换向。

新增 `hunav_interactive_smoke --scenario-profile occupied_passage`：

- agent 01 固定目标 `urinal_2`，agent 02 固定目标 `urinal_1`；
- agent 01 进入服务阶段后，机器人移到固定阻塞位；
- agent 02 延迟激活，形成“机器人 + 占位行人”联合封闭；
- agent 01 离开服务阶段时撤走机器人，验证让行释放和剩余路线重建；
- 服务时间固定为 `55 s`，seed、激活间隔和目标顺序全部写入隔离配置。

后端新增的几何覆盖只处理固定行人：

1. 固定行人位于当前前向走廊且距离小于触发阈值；
2. 左右候选同时通过 walkable map、机器人有向 footprint 和其他固定行人检查；
3. 两侧均不可用才进入 `YIELDING_TO_PEDESTRIAN`，保持位置和 yaw；
4. 走廊连续可用后释放，重新规划到原语义目标。

它不替换 HuNav 社会力，也不把正常会车改成一律停车。

固定 seed `12345` 的完整实测：

```text
/tmp/toilet_occupied_passage_final_20260731/
  20260731_113652_regular_disabled
```

结果：

- agent 02 在距占位者 `0.943 m` 时提前让行；
- 让行持续约 `24.4 s`，随后仅释放一次并到达 `urinal_1`；
- 两人均完成 urinal 和 exit，director 退出码为 0；
- `stall_events=0`、`recovery_exhausted_events=0`；
- 让行区间离线核验：位置包络跨度 `0.028 m`、平均速度
  `0.0024 m/s`、yaw 相对首帧最大漂移 `0.0029 rad`。

smoke summary 已增加只统计该状态窗口的
`raw_pose_pedestrian_yield_by_agent`，避免把正常服务等待混入让行稳定性指标。

剩余问题：

- 全流程仍记录到 2 个高频行人代理重叠帧，最小中心距 `0.411 m`；
- 视觉骨骼外包络仍有门、隔板和小便池 overlap；
- agent 02 全流程仍有一次路线进度回退。

因此本次只证明“不可通行时稳定等待”边界成立，不代表普通可通行会车的自然性已经
通过。下一步应增加独立的“可通行窄缝”profile，验收单侧承诺、换边次数和通过时间。
