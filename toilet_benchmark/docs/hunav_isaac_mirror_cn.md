# 单行人 HuNav-Isaac Mirror

## 目的

该节点在现有 Isaac People 行人正常执行厕所流程时，并行运行一个 HuNav shadow：

- Isaac 仍由 `toilet_director_node -> /isaac/move_pedestrians -> AnimGraph` 驱动。
- 当前机器人主链路使用 `physx_diff_contact`；mirror 只读取其 `/odom`，不改变轮地接触控制。
- director 在 MovePed 返回 `ret=True` 后，于原有状态 topic 旁路发布 `motion_intent`。
- mirror 用同一实际起点、路径点、目标速度和最终 yaw 初始化 HuNav。
- mirror 只调用 HuNav 服务和写日志，不创建 Isaac move client，不会控制场景中的行人。

因此当前误差表示“相同高层意图下 HuNav 与现有 Isaac 执行器的轨迹分歧”，不是闭环控制误差。
`physx_diff_contact` 与 mirror 没有控制权冲突：前者负责机器人真实轮地运动，后者仅把机器人
状态作为 HuNav 社会运动计算的环境输入。

## 构建

```bash
cd /home/stardust/resources/arena_ws
colcon build --packages-select ros2isaacsim toilet_benchmark \
  --symlink-install --cmake-force-configure
source install/setup.bash
```

## 启动顺序

先按现有主链路启动 Isaac bridge：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

该 phase 已启用 People stack、character services、RTX 双雷达和 `/odom`。不需要额外启动
`rtx_scan` phase，也不要同时启动两个 bridge。

在启动 director 前先启动 mirror，避免漏掉第一条 motion intent：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark hunav_isaac_mirror \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/hunav_isaac_mirror.yaml
```

然后启动单行人流程，例如固定 `urinal_3`：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_director_node \
  --initial-agents 1 \
  --target-resource urinal_3
```

director 发布 `pedestrian_retiring` 后，mirror 默认自动结束。也可以随时 `Ctrl+C`，已记录样本仍会写入结果。

## 输出

每次运行写入：

```text
/tmp/toilet_hunav_mirror/<run_id>/
  manifest.json
  events.jsonl
  trace.jsonl
  result.json
  hunav.log
  ros_logs/
```

- `events.jsonl`：director 生命周期事件和完整 motion intent。
- `trace.jsonl`：逐帧 Isaac/HuNav 状态、实际传给 HuNav 的 robot 快照、目标和误差。
- `result.json`：样本频率、分段数、路径长度、位置、heading、路径偏差和交互摘要。

快速查看结果：

```bash
python3 - <<'PY'
import json
from pathlib import Path

run = max(Path("/tmp/toilet_hunav_mirror").iterdir())
print(json.dumps(json.loads((run / "result.json").read_text()), indent=2))
PY
```

## 关键约束

- `/isaac/pedestrian_states` 仍为 `people_msgs/People`，兼容原 director。
- velocity 只在 `accepted/executing` 状态下由连续有效 root 位姿估算；无效位姿、direct-pose
  和池化传送不会形成伪速度。
- yaw 通过 `yaw_rad/yaw_valid` tags 发布，不修改 `people_msgs` 接口定义。
- `yaw_error_rad`/`root_yaw_error_rad` 保留角色 root yaw 对比；角色本地前向轴偏置会影响该值。
- `motion_heading_error_rad` 只在双方速度高于 `motion_heading_min_speed_mps` 时比较速度方向，
  用于区分“角色 root 朝向偏置”和“真实运动方向偏差”。
- 每条 walking intent 都开启一个新 shadow segment，避免把 direct-pose 初始化或不同阶段混成一条轨迹。
- `physx_diff_contact` 发布的 `/odom` 是机器人实际 PhysX 位姿和速度；mirror 将其 twist
  从机器人局部系旋转到 `map` 后再传给 HuNav，不使用旧的理想运动积分替代该状态。
- `robot` 保存该次 ComputeAgents 请求使用的 odom 快照和观测年龄；`interaction` 保存
  Isaac/HuNav 行人与机器人的中心距离和扣除双方半径后的净间距。
- `actual_cross_track_error_m` 和 `hunav_cross_track_error_m` 表示相对 director nominal
  path 的横向偏差，可定位门后偏转。`*_near_target_jitter_m` 汇总目标半径附近的额外移动，
  可定位小便池附近的不稳定，但不能单独证明与 USD mesh 发生穿模。

## Live 验收建议

第一轮先固定一个角色、一个小便池并重复 5 次，检查：

- `intent_count` 与 `events.jsonl` 中 walking intent 数一致。
- `effective_sample_hz` 接近 bridge 实际的行人状态发布频率。
- `compute_failures` 为 0，且 `isaac_motion_commands_sent` 恒为 0。
- `actual.motion_state`、实际速度和 UI 中 walk/idle 一致。
- `robot_observation_sample_count` 接近 `sample_count`，且
  `max_robot_observation_age_sec` 没有持续超过 odom 发布周期。
- 机器人拦路轮次中对比 `guard_blocked_duration_sec`、
  `min_actual_robot_clearance_m` 和 `min_hunav_robot_clearance_m`。
- 门口与小便池异常轮次中检查 cross-track、target distance 和 near-target jitter，
  不再只看旧的 root yaw 汇总。
- 分段起点误差接近 0，之后误差连续增长或收敛，不出现时间错配造成的单帧跳变。

这一步通过后再进入单行人 shadow takeover；当前 mirror 结果不能直接证明 HuNav 已适合控制 AnimGraph。
