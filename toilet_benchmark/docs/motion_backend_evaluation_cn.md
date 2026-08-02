# 行人运动与表现 Backend 评估规范

状态：Interface Freeze v0.1
冻结日期：2026-08-01

## 1. 目的

本文避免把 HuNav、几何避障、事件状态和动画混为一个“行人 backend”。评估对象分为：

1. Global Router；
2. Local Motion Backend；
3. Geometry Safety；
4. Embodiment Adapter。

## 2. 候选能力边界

| 组件 | 负责 | 不负责 | Track |
| --- | --- | --- | --- |
| Walkable Theta*/后续 router | 静态路线、语义 corridor | 人际反应、动画 | Replay + Interactive |
| HuNav SFM/BT | 社会速度、群组、人机反应 | 资源、全局可达、精确终态 | Interactive |
| ORCA/HRVO 候选 | reciprocal collision avoidance | 厕所意图、真实心理 | Interactive baseline |
| Replay actor | 冻结状态序列和确定性插值 | 双向交互 | Replay |
| Geometry safety | 检测/阻止非法穿透 | 选边、绕行、让行决策 | Debug/contain |
| Isaac AnimGraph | 角色资产、walk/idle/activity 表现 | 任务和路径规划 | 当前表现基线 |
| SMPL-H | 高保真骨骼、动作库和可控 root | 社会决策和资源逻辑 | 候选高保真表现层 |

HuNav 不再被定义为“厕所行人系统”，而是 `LocalMotionPort` 的一个实现。正式 Interactive
Track 可以冻结 HuNav 版本；ORCA/HRVO 用作几何基线和问题定位，不保证替代 HuNav。

当前 E2-B 实现的 `SampledRvoLocalMotion` 是确定性的 sampled velocity-obstacle 基线，
不是严格求解半平面约束的 ORCA。命名用于表达 reciprocal-avoidance 候选方向；评估结论中
必须使用 `sampled_rvo`，不得把结果表述为“已集成完整 ORCA”。它当前仅可通过
`motion_backend.hunav.local_motion_shadow.enabled` 进入只读对照，不拥有 Isaac 控制权。

2026-08-01 的固定 seed Isaac shadow 结果显示：无机器人单人 `54/54`、双人 `96/96`
样本可行；机器人 crossing 仅 `60/65` 样本可行，并且 65 个样本中只有 2 个产生显式避让。
这说明当前离散候选集能验证端口和开放空间互避，却不能在紧接触窗口稳定替代 HuNav。
在 crossing gate 通过前禁止启用 takeover，也不继续运行 narrow/queue 作为通过性结论。

E2-B 第二阶段已提供显式 `--motion-backend local_motion` 实验入口。该入口不调用 HuNav，
而是直接组合 `ContextualBehaviorPolicy + SampledRvoLocalMotion + GeometrySafety + Isaac
external-motion`。纯 2D doorway/动态边界/静态预筛/连续性 gate 已通过，但 Isaac live gate
尚未完成，因此默认 backend 仍保持不变。

固定 seed 的 Isaac live gate 复用现有 smoke runner，并显式选择实验 backend：

```bash
ros2 run toilet_benchmark hunav_interactive_smoke \
  --motion-backend local_motion \
  --behavior regular \
  --reaction disabled \
  --legacy-avoidance disabled \
  --seed 12345 \
  --target-resource urinal_1 \
  --robot-intervention parked_away
```

脚本名称暂时保留以兼容既有自动化；实际 backend 会同时写入 `metadata.json` 的
`motion_backend` 和 director 命令，不应再根据脚本名称推断运动实现。
`parked_away` 仅用于先隔离单行人运动与静态几何；机器人留在路线、crossing 和
occupied-passage 必须作为后续独立动态微场景测试，不能用该结果替代。

### 2.1 首次 Isaac live 隔离结果（2026-08-01）

固定 `seed=12345`、`urinal_1`、`parked_away` 的运行目录：

```text
/tmp/toilet_pipeline_runs/20260801_211621_regular_disabled
```

结果：

- `EnterUseExit` 完成，urinal/exit 各到达 1 次；
- 42 个 local-motion 诊断样本全部可行，剩余距离无倒退；
- 无 stall、recovery exhaustion 或 HuNav hard-safety 事件；
- 142 个有效骨骼视觉包络样本，静态重叠为 0，视觉 gate 通过；
- raw pose 的转向方向反转为 7 次，累计 heading 变化约 `21.95 rad`；
- 命令/实际速度误差均值约 `0.66 m/s`、最大约 `0.95 m/s`。

因此该结果只放行“单行人功能闭环和零视觉穿模”，尚未放行自然性、速度跟踪或默认
backend 切换。下一轮必须继续定位 AnimGraph 可实现速度与 local-motion 命令速度的差异，
并完成 doorway/dynamic crossing/narrow 场景的固定 seed 矩阵。

同 seed 的 `0.48 m/s` planner 限速实验位于：

```text
/tmp/toilet_pipeline_runs/20260801_212309_regular_disabled
```

该实验将命令/实际速度误差均值降至约 `0.25 m/s`，但视觉包络与大门连续重叠 4 帧，
heading 变化增至约 `26.20 rad`、方向反转增至 9 次、stop episode 增至 4 次，因此方案
已撤销。该实验没有先测量 `Walk` blend 对 root motion 的真实响应，因此不能作为
EmbodimentAdapter 参数。后续标定必须先得到 reference speed、Walk blend 和周期平均 root
speed 的对应关系，再将可实现速度上限反馈给 local planner；任务层 desired velocity 保持不变。

### 2.2 Walk blend 标定与 embodiment 边界（2026-08-01）

新增 `ros2 run ros2isaacsim pedestrian_speed_calibration`，在无障碍直线上闭环刷新
external-motion reference，并用完整采样窗口的位置-时间斜率估算 root speed，避免单帧步态
相位和 reference tracking correction 污染结果。独立角色结果保存在：

```text
/tmp/pedestrian_walk_speed_calibration_fresh.json
```

拟合得到 `root_speed = 0.3082 * Walk^1.7670`，RMSE `0.011 m/s`，最大误差
`0.018 m/s`。该结果只验证了当时采样的 Walk/reference 区间，不能据此把 `0.31 m/s`
解释成 stock People 的绝对速度上限。后续 UI 观察表明 `0.9-1.2 m/s` 时步幅会继续变化，
且未见明显 root/腿部脱节，但转弯过冲更明显。因此当前
`motion_backend.local_motion.embodiment_max_speed_mps=0.31` 仅作为多行人局部规划对照的
保守基线；提升速度前必须补测高速直线、转弯半径、heading 跟踪和扫掠包络，而不是只改
一个全局上限。

三个固定 seed 的首轮 doorway 运行均完成事件流，但候选映射尚未限速时，视觉包络均失败，
连续重叠最高为 61、83、93 帧。加入 embodiment 上限后的复测目录为：

```text
/tmp/toilet_pipeline_runs/20260801_223420_regular_disabled
```

速度跟踪误差均值降到 `0.114 m/s`，连续重叠降到 4 帧；仍有门框、隔板和目标小便池命中，
所以视觉 gate 仍保持失败，不能晋升为默认 backend。

### 2.3 动态 crossing 时序与净空评分（2026-08-01）

旧 smoke runner 只解析 HuNav pose，导致 local-motion crossing 没有按行人位置触发；修复后又
发现原触发点与未标定的快 reference 绑定。现在 local diagnostics 显式记录 pose、机器人新鲜度、
机器人距离和选中候选的预测动态净空，`crossing_conflict` 按标定速度在 `x=-2.0` 触发。

Sampled RVO 不再只做动态碰撞二值过滤：安全候选还会惩罚低于 `0.30 m` 的预测动态净空，
使行人在候选耗尽前开始侧向调整。最终复测目录：

```text
/tmp/toilet_pipeline_runs/20260801_224056_regular_disabled
```

结果为事件流完成、69/69 local-motion 样本可行、无 stall、无机器人接触，最小预测动态净空
`0.290 m`，速度跟踪误差均值 `0.108 m/s`。视觉 gate 仍因门框/隔板等 18 个包络样本失败；
动态绕行已恢复，但静态扫掠安全仍是下一阶段最高优先级。

### 2.4 2--4 人固定矩阵与重叠恢复（2026-08-01）

`hunav_behavior_matrix` 已支持 `--motion-backend local_motion`、固定人数和两类目标布局：

- `spread`：优先拉开目标，顺序为 `urinal_1, urinal_5, urinal_3, urinal_2`；
- `adjacent`：相邻资源压力测试，顺序为 `urinal_1, urinal_2, urinal_3, urinal_4`。

固定 `seed=12345`、移动机器人 `crossing_conflict` 的结果：

| 人数/布局 | 任务 | 最小 pair 距离 | pair 重叠 | local 不可行 | 结论 |
| --- | --- | ---: | ---: | ---: | --- |
| 2/spread | 完成 | `1.076 m` | 0 | 0 | 动态侧通过，静态视觉 gate 失败 |
| 2/adjacent | 完成 | `0.767 m` | 0 | 0 | 相邻资源可完成，静态视觉 gate 失败 |
| 3/spread | 完成 | `0.694 m` | 0 | 0 | 多人动态侧通过，门体连续包络重叠 10 帧 |
| 4/spread（修复前） | 超时 | 约 `0.53 m` | 持续 | 持续 | 相邻资源同时退出后 sampled-RVO 永久拒绝全部候选 |
| 4/spread（重叠恢复后） | 完成 | `0.510 m` | 约 `29.3 s` | 25 | 永久死锁解除，但整体 gate 仍失败 |

相关运行目录：

```text
/tmp/toilet_multi_matrix_live
/tmp/toilet_multi_matrix_live_adjacent
/tmp/toilet_multi_matrix_live_34_gpu
/tmp/toilet_multi_matrix_live_4_fixed
```

修复后的 sampled-RVO 对当前已重叠 actor 不再把 `t=0` 碰撞解释为“永远无解”：只允许
立即增加分离距离的速度；原始路线能离开冲突的一方优先，否则使用稳定 agent ID 打破对称。
同时，开放区域软动态余量降为 `0.15 m`，并按候选的静态净空自动收缩；actor/静态硬包络
不缩小。速度采样由 4 档加密到 7 档，以减少拥挤区离散速度跳变。

4 人修复结果仍不能作为通过结论。新增分类诊断对同一日志得到：最小 peer 硬余量
`-0.029 m`、最小 robot 硬余量 `-0.498 m`、重叠恢复样本 9 个。前者主要来自
`urinal_1/urinal_2` 相邻站位及交错进入/退出；后者说明 scripted crossing 机器人进入了
行人硬包络，不能靠行人软净空解决。下一阶段必须：

1. 统一机器人控制端与行人 local-motion 的硬几何契约；
2. 为相邻 SmartObject interaction region 建立有朝向的人体占用/通行冲突关系；
3. 修复门口视觉包络连续命中，穿模 gate 通过前不得提升为默认 backend；
4. 将 `0.31 m/s` 保留为当前隔离基线，另做 `0.6/0.9/1.2 m/s` 直线与转弯标定。

## 3. 微场景矩阵

所有 local backend 必须在相同世界快照、目标和随机 seed 下运行：

| Case | 期望行为 | 主要失败 |
| --- | --- | --- |
| open_head_on | 稳定选边并通过 | 左右振荡、远距离冻结 |
| open_crossing | 平滑调速或绕行 | 穿透、急剧回头 |
| doorway_conflict | 明确等待或按 token 通过 | 双方挤门、死锁 |
| narrow_impossible | 等待，不持续行走动画 | 推挤、原地旋转 |
| narrow_passable | 选择可行侧并重接路线 | 选错窄侧、贴墙 |
| occupied_goal | interaction region 外等待 | 穿过占用者 |
| queue_release | 顺序晋升且无跳变 | 抢占、错误消失 |
| robot_departure | 障碍清除后及时恢复 | 恢复过慢、旧路线残留 |

## 4. 必选指标

- task completion、timeout、deadlock；
- 静态几何和 actor-actor 穿透帧数；
- 最小净空、TTC 和 personal-space intrusion；
- stop episode 次数和总冻结时间；
- heading 总变化、方向反转和角速度峰值；
- route progress regression；
- 选边切换次数和重规划次数；
- 机器人离开后的恢复延迟；
- root 速度与动画 locomotion 状态不一致时间；
- foot sliding、最终位置/yaw 和视觉扫掠包络重叠。

穿透是硬失败；个人空间侵入是社会指标，两者不得用同一个扩大半径替代。

## 5. 决策门槛

Interactive 默认 local backend 必须：

1. 三个固定 seed 下完成全部微场景；
2. 无静态或动态穿透；
3. 不可通行场景能稳定等待；
4. 可通行场景不被错误冻结；
5. 同输入重复运行轨迹在冻结容差内；
6. 行为诊断能解释每次 stop、replan 和 reaction transition。

若 HuNav 在狭窄区域持续失败，优先采用上下文策略：事件层通过 portal token/queue 处理
不可通行协商，HuNav 继续处理开放区域连续社会运动。禁止把 portal 几何特判塞回 HuNav
热路径。

## 6. 表现层 A/B

相同 motion trace 分别驱动 Isaac AnimGraph 和 SMPL-H，比较：

- root tracking RMSE；
- heading tracking error；
- walk/idle 切换延迟；
- foot sliding；
- 转弯自然度；
- interaction pose 精度；
- 骨骼视觉包络碰撞；
- 运行频率、显存和长期稳定性。

只有表现指标变化而 motion truth 不变，才是有效的 adapter A/B。若更换角色后路径或任务
结果变化，说明控制权边界仍然泄漏。
