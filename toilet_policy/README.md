# Toilet Policy

该包提供厕所社交导航 benchmark 的小型行为克隆基线。训练输入为：

- 前后两个 `721` 维一维雷达数组；
- 目标在机器人坐标系内的 `x/y` 与目标朝向 `sin/cos`；
- odom 底盘线速度和角速度；
- 上一时刻操作者动作。

输出为 `/cmd_vel_gamepad_diff` 对应的 `linear.x` 和 `angular.z`。`/isaac/pedestrian_states` 不作为模型输入，只用于后续评估。

## 导出

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_policy toilet_policy_export \
  --index /home/stardust/resources/arena_ws/data/toilet_policy/v0_unreviewed/index.jsonl \
  --output /home/stardust/resources/arena_ws/data/toilet_policy/v0_unreviewed_aligned
```

## 训练

系统 ROS Python 不包含 PyTorch，使用 Isaac Sim 自带环境：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
PYTHONPATH=/home/stardust/resources/arena_ws/install/toilet_policy/local/lib/python3.10/dist-packages:$PYTHONPATH \
  /home/stardust/resources/isaac-sim-4.5.0/python.sh \
  -m toilet_policy.train \
  --dataset /home/stardust/resources/arena_ws/data/toilet_policy/v0_unreviewed_aligned \
  --output /home/stardust/resources/arena_ws/data/toilet_policy/models/lidar_gru_v0 \
  --epochs 20
```

该模型仅用于验证数据、训练和 Isaac 闭环推理链路，未经人工复核的数据不能作为正式 benchmark baseline。

训练过程中会同时生成：

- `best.pt`：验证损失最低的 epoch；
- `last.pt`：最后一个 epoch，可用于刻意检查过拟合模型的闭环表现。

## Isaac Sim 闭环测试

先启动 `bridge physx_diff_contact` 和 `spawn --phase physx_diff_contact`，不要同时启动 gamepad 节点。构建并 source 后，以预览模式启动：

```bash
ros2 run toilet_policy toilet_policy_inference --ros-args \
  --params-file /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_policy/config/inference.yaml
```

预览命令发布在 `/toilet_policy/cmd_vel_preview`，不会控制机器人。确认双雷达和 odom 均为 fresh 后，启用闭环：

```bash
ros2 service call /toilet_policy_inference/set_enabled std_srvs/srv/SetBool "{data: true}"
```

关闭闭环：

```bash
ros2 service call /toilet_policy_inference/set_enabled std_srvs/srv/SetBool "{data: false}"
```

也可在启动时添加 `-p closed_loop:=true`。节点在输入过期、目标到达和退出时发布零速度。第一版固定目标为 `[-3.8, -0.91, 0.0]`，可通过 `-p goal_pose:="[x, y, yaw]"` 覆盖。

## 完整 authored episode 评估

通用 `spawn --phase physx_diff_contact` 只负责导入场景资产和机器人，不读取
Route Editor episode。完整评估由下面的入口启动；它读取 episode 中的机器人
出生点和目标点、复位机器人、生成行人，并在全部行人激活后接通策略：

```bash
ros2 run toilet_policy toilet_policy_eval \
  --episode /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark_ui/examples/narrow_head_on_001.json \
  --checkpoint /home/stardust/resources/arena_ws/data/toilet_policy/models/lidar_gru_v0_gpu_150/best.pt \
  --planner-radius-m 0.3
```

运行前仍需启动 `bridge physx_diff_contact` 并执行一次
`spawn --phase physx_diff_contact`，但不要单独再启动 `toilet_authored_scenario`、
`toilet_policy_inference` 或 gamepad。eval 会关闭预测性人机 hard guard；真实人机
接触、场景碰撞、超时判失败，机器人到达 episode 的 `robot.goal_pose` 判成功。
每次 eval 都会自动为行人 ID 添加唯一 incarnation suffix，创建新的
People/AnimGraph 实例，避免跨 episode 复用路径游标、朝向或步态状态；角色模型和
authored route 仍保持 JSON 中的固定配置。
