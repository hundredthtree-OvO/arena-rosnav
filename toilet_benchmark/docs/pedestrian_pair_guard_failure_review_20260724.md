# 行人间硬安全层失败复盘

日期：2026-07-24

## 回退基线

- `arena-isaac`: `fbca3f6 fix robot-pedestrain collision`
- `arena-rosnav`: `33a468d4 add multi-pedestrain in data collection script`
- 上述提交保留机器人与行人的 hard guard、多人数据采集和现有事件编排。
- 本轮未提交的行人-行人 guard、暂停恢复和路径重建试验已全部回退。

## 失败样本

主要样本：

`/home/stardust/resources/arena_ws/data/toilet_manual/session_20260724_143644_seed42/episode_000001`

该 episode 配置了两个行人，目标分别为 `urinal_1` 和 `urinal_2`。元数据最终标记为
`succeeded`，终止原因是 `robot_reached_goal`，但这只表示机器人任务完成，不能证明行人交互
过程有效。事件记录中有 39 次机器人-行人 hard guard 介入，但没有记录行人-行人冻结、恢复、
路径重建或异常状态，因此仅凭 episode 状态无法识别 UI 中的穿越、闪现和回退。

关联失败样本：

- `session_20260724_115117_seed42`
- `session_20260724_123459_seed42`
- `session_20260724_124556_seed42`
- `session_20260724_142153_seed42/episode_000001`

## 已尝试方案

1. 在 Isaac People 更新循环中加入行人圆形代理的预测碰撞检测。
2. 从双向同时阻挡改为确定性单方让行，另一方继续沿原路径运动。
3. guard 命中时停止速度和 Walk 动画，解除后恢复原目标。
4. 保存剩余路径快照，解除后从当前位姿重建路径。
5. 增加静态 voxel 穿透逃逸判定，避免已经轻微重叠时完全锁死。
6. 增加短时释放保持，尝试抑制 guard 在相邻帧反复开关。

## 失败表现

- 狭窄区域内两个行人反复穿过彼此，随后闪现或回退。
- 行走动画、AnimGraph 根节点和 director 记录的实时位姿出现不同步。
- guard 在相邻帧改变让行方，造成双方交替冻结。
- 恢复时重建路径会与 Isaac People 正在执行的 `GoTo` 命令竞争。
- 近小便池、隔板和门口时，静态 voxel guard 与行人间 guard 叠加，容易形成无法释放的互锁。
- episode 的成功判定只依赖机器人到达目标，无法自动淘汰上述行人失败样本。

## 根因判断

当前失败不是简单的距离阈值问题。核心问题是多个模块同时拥有行人运动状态：

- director 负责状态机、目标和路径下发；
- Isaac People `CharacterBehavior` / `NavigationManager` 负责 `GoTo` 和动画；
- bridge 更新循环又尝试覆盖速度、路径和根节点位姿；
- voxel guard 同时对静态障碍进行逐帧约束。

在物理帧内强行冻结并重建 vendor AnimGraph 路径，会破坏单一运动所有权。视觉根节点、导航
候选位姿和 ROS live pose 可能来自不同阶段，导致碰撞判定抖动、路径回滚和闪现。继续增加
阈值、迟滞或优先级只能缓解个别样本，无法消除状态竞争。

## 后续建议

第一阶段不要再在 Isaac People 的逐帧更新中实现行人-行人硬碰撞。改由 director 在发出
`GoTo` 之前做事件级交通编排：

1. 为门口和小便池前狭窄通道定义逻辑区域，并采用单持有者 token。
2. 行人在区域外的明确等待点等待；获得 token 后一次性执行完整路径。
3. 前一行人离开区域后再放行后一行人，不在运动途中改写 AnimGraph 路径。
4. 将等待、获得区域、释放区域和超时写入 episode 事件，便于判定数据有效性。

该方案优先保证第一轮数据采集的稳定性、可复现性和可审计性。它不模拟自然的并排行走或
局部绕行，但能避免当前狭窄场景中的互锁和状态竞争。

第二阶段若需要多个行人同时自然运动，应把局部避障的轨迹所有权整体交给 HuNav、ORCA 或
其他独立局部规划器。Isaac People 只消费稳定轨迹并播放动画，director 只负责任务事件，
不要再由 bridge 同时修改其内部导航状态。

## 后续修改约束

- 不要在没有独立回归场景和事件日志的情况下恢复本轮行人间 guard。
- 不要通过提高代理半径解决狭窄区域互锁。
- 不要在 `Person.update()` 中同时暂停 AnimGraph、覆盖根节点并重建路径。
- 保留已提交的机器人-行人 hard guard；它与本轮已回退的行人-行人 guard 是两套功能。
