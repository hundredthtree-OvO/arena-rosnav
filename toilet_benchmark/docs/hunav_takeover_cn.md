# HuNav Isaac Takeover

## 当前范围

该模式用于单行人回归和 shared-world 多行人可视化验证：

- director 继续负责编排入口、小便池、停留和出口事件。
- voxel route 暂时继续提供全局参考路径。
- HuNav 是行走阶段唯一的连续运动权威。
- Isaac `external_motion` adapter 只负责根位姿、碰撞 proxy 和 Walk/Idle 动画。
  它每帧以 Isaac 4.5 官方 `list[carb.Float3]` 格式同步 AnimGraph 所需的
  `Action`、`Walk`、`PathPoints`。HuNav 输出作为参考轨迹，正常 root 位移由
  MotionMatching 产生；只有跟踪误差超过 `0.65m` 时才会保护性重定位。
- 激活使用一次性 direct pose，不与 HuNav 行走同时执行。到达小便池后的停止会保留为
  shared HuNav world 中的零速度固定成员，直到真正退出场景才移除。

`--initial-agents 1` 用于稳定单人回归；`2` 及以上会让所有 active pedestrians
共享同一次 HuNav `reset_agents/compute_agents`。当前多人模式仍是实验入口，需重点观察
狭窄通道会车、一个人停止时另一人继续运动，以及退出后剩余 agent 是否连续。
后续行人的默认激活间隔为 `4.0s`，可通过
`config/toilet_benchmark.yaml` 的 `director.initial_spawn_interval_sec` 调整。

## 编译

本阶段修改了 `isaacsim_msgs/NavPed.msg`，必须重编译接口和两个使用方：

```bash
cd /home/stardust/resources/arena_ws
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select isaacsim_msgs ros2isaacsim toilet_benchmark \
  --symlink-install
source /home/stardust/resources/arena_ws/install/setup.bash
```

不要只重编译 `toilet_benchmark`，否则 bridge 与 director 会使用不同版本的
`NavPed` 类型。

## 启动

终端 1，保持当前主链路：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

终端 2，启动单行人回归：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --initial-agents 1
```

双行人 shared-world 验证只需把最后一项改为：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --initial-agents 2
```

director 默认自动启动独立 namespace 下的 HuNav manager。日志写入：

```text
/tmp/toilet_hunav_takeover/hunav_manager_<pid>.log
/tmp/toilet_hunav_takeover/director_hold_<pid>.jsonl
```

`director_hold_<pid>.jsonl` 会记录机器人接近反应的状态切换和每次停止
指令，包含锁存的 `hold_yaw`、Isaac 实时反馈的 `actual_yaw`、两者误差、
行人位姿、速度与当前 phase，用于区分规划目标抖动和 AnimGraph 根节点抖动。

## RViz 诊断

本阶段新增的可视化不依赖 bridge 的旧 voxel guard。使用专用配置启动：

```bash
rviz2 \
  -d /home/stardust/resources/arena_ws/install/toilet_benchmark/share/toilet_benchmark/config/hunav_takeover.rviz \
  --ros-args -p use_sim_time:=true
```

- 白色和红色分别显示 `/front_scan`、`/rear_scan`。
- 蓝色 Path 显示 director 给 HuNav 的当前语义 route。
- 绿色圆柱和箭头显示行人位置与 yaw，青色箭头表示本 tick HuNav 参考位移，橙色矩形
  表示 portal corridor。

白色和红色 RTX scan 仅用于检查机器人观测与数采结果，不参与行人 route、HuNav 局部运动
或 hard safety。terminal 同时记录 `generation`、`phase`、目标 yaw、速度和
`robot_contacts`。

如果已有 HuNav manager：

```bash
ros2 run toilet_benchmark toilet_director_node \
  --motion-backend hunav \
  --hunav-namespace /toilet_hunav_takeover \
  --no-start-hunav-manager \
  --initial-agents 1
```

## 回滚

省略 `--motion-backend hunav` 即回到兼容链路：

```bash
ros2 run toilet_benchmark toilet_director_node --initial-agents 1
```

## 首轮观察项

- 行人是否连续穿过 portal，而不是在 waypoint 瞬移吸附。
- 行走时 Walk 动画是否持续，速度归零或通信超时后是否立即 Idle。
- 机器人阻挡时 HuNav 是否停下或绕行，机器人移开后是否恢复。
- 到达小便池附近后是否稳定停止，最终 yaw 是否仍有角色资源偏置。
- 终端日志是否出现 `source=motion-backend settled handshake`；这表示 HuNav 已
  消费目标，director 不会把正常驻停误判成卡死并重置三次。
- bridge 是否出现 external-motion 类型不匹配或 AnimGraph root transform 警告。

当前由 walkable-map global router 负责静态全局路径，HuNav 负责局部社会运动。失败的离线
voxel tick 裁剪已删除；静态地图来自 USD/PhysX collision geometry，下一阶段再接入 Isaac
PhysX shape sweep 做短时域 hard safety。该链路与 RTX 雷达观测完全解耦。

## Walkable Map 隔离验收

保持 bridge 运行，先导出静态地图：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run ros2isaacsim export_walkable_map
```

导出文件位于：

```text
/home/stardust/resources/arena_ws/arena_assets/navigation/shenxinfu_841837.walkable.json
/home/stardust/resources/arena_ws/arena_assets/navigation/shenxinfu_841837.walkable.pgm
/home/stardust/resources/arena_ws/arena_assets/navigation/shenxinfu_841837.walkable.yaml
```

发布地图并检查入口、portal 和五个小便池 interaction anchor：

```bash
ros2 run toilet_benchmark walkable_map_publisher

rviz2 \
  -d /home/stardust/resources/arena_ws/install/toilet_benchmark/share/toilet_benchmark/config/walkable_map.rviz \
  --ros-args -p use_sim_time:=true
```

绿色 anchor 表示落在 free cell，红色表示 occupied、unknown 或地图范围外。
`walkable_map_publisher` 本身仅负责可视化，不控制或改变行人。

地图通过验收后，HuNav takeover 会自动使用该资产作为全局路线来源：

```text
path_planner.hunav_backend: walkable_map
```

默认 `isaac_people` 模式仍使用旧 voxel provider。HuNav 日志中的
`walkable-map route` 会包含场景指纹和完整稀疏 waypoint，便于确认运行时使用的地图版本。
