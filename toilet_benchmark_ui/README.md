# Toilet Benchmark 场景路线编辑器

独立 Qt 2D 编辑器，用于制作可直接交给 Isaac 执行的 `authored_route` episode。
RViz 继续只负责 LiDAR、TF 和 3D 诊断，不承担路线编辑。

## 编辑器负责什么

- 独立设置机器人出生位、朝向和任务终点；
- 添加、删除多个行人；
- 为每个行人设置出生位、朝向、速度和有序目标锚点；
- 在任意路线点设置阶段性停留时间；
- 在 walkable map 上检查出生点、路线和人体半径；
- 保存、加载标准 `toilet-social-nav-0.1` episode JSON。

编辑器不直接驱动 Isaac。旧的 `Generation`、`Phase`、`Subgoal TTL`、
`Shadow Route/Subgoal` 已移除，因为它们混淆了离线场景定义和在线调试，而且没有形成
可执行闭环。

## 构建

```bash
cd /home/stardust/resources/arena_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select toilet_benchmark toilet_benchmark_ui --symlink-install
source install/setup.bash
ros2 pkg executables toilet_benchmark_ui
```

应看到 `toilet_benchmark_ui route_editor_node`。

## 离线制作双行人窄通道场景

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark_ui route_editor_node
```

不需要启动 bridge。操作顺序：

1. 点击 `加载地图`，选择
   `/home/stardust/resources/arena_ws/arena_assets/navigation/shenxinfu_841837.walkable.json`；
2. 在参与者列表选择 `xms_mecanum`，点击 `设置出生点` 后在地图点击；
3. 保持选中机器人，点击 `设置机器人终点` 后在地图点击；终点必须在出生点容差圈外；
4. 点击 `添加行人`，选择该行人并设置出生点；
5. 设置行人的出生点行为和终点行为；需要运动时点击 `画路线`，依次点击必须到达的目标；
6. 切换 `选择`，点中某个 waypoint，再点击 `在选中点停留`；
7. 再添加一个行人，设置相反方向的出生点与路线；
8. 点击 `验证场景`，确认基础检查和规划检查均通过；
9. 点击 `保存场景`，得到可执行 episode JSON。

仓库内已提供一个示例：

```text
toilet_benchmark_ui/examples/narrow_head_on_001.json
```

它包含两个对向行人、独立出生位和路线，其中第一个行人在 waypoint 1 停留 1.5 秒。

## 鼠标和按钮

工具栏只保留高频操作：

| 操作 | 作用 |
| --- | --- |
| `选择` | 选择路线点，用于删除或添加停留 |
| `画路线` | 给当前选中的行人追加 waypoint |
| `编辑路线点` | 拖动当前 actor 的出生点、行人 waypoint；选中机器人时也可拖动终点 |
| `设置出生点` | 设置当前选中机器人或行人的出生位 |
| `设置机器人终点` | 设置独立于机器人出生位的任务终点 |
| `清空当前路线` | 只清空当前行人的路线和停留点 |

右键删除最近路线点，`Delete`/`Backspace` 删除选中点，滚轮缩放。中间分隔条可拖动。
加载已有场景后，`保存场景` 会先询问是否覆盖当前文件；选择“否”可另存为，另存成功后
新文件会成为当前场景文件。新建草稿则直接进入另存为。

机器人出生朝向使用角度手动编辑。每个行人可独立启用或关闭自动出生朝向；启用时由点 0
到首个有效路线点实时推导，关闭后可手动编辑。出生点可立即出发、停留指定时间或保持到本轮结束。
中间停留点可设置有限停留及独立朝向；终点单独选择到达后消失或保持到本轮结束，
保持时可设置终点朝向。出生点保持到本轮结束时，速度和路线约束不会生效，后续路线不可配置。
目标、出生点和停留点都绑定到参与者列表中的
具体 actor，不再共享一个全局 spawn/subgoal。

编辑器中的连线表示目标访问顺序，不再表示 Isaac 必须原样执行的直线段。在线 runner
会加载 `assets.walkable_map`，以 `0.30 m` 默认人体半径对相邻目标执行静态 Theta*/A*
规划，并将规划结果细分为不超过 `0.50 m` 的运动段。行人到达最后一个目标后会调用
bridge 行人池接口，移出可见场景而不销毁 AnimGraph Prim。
如果机器人先到达目标，数采会立即结束本轮录制并优雅回收尚未走完的行人，不再等待其完整路线。

## 在线状态预览

启动 bridge 后再打开编辑器，可以叠加显示 `/isaac/pedestrian_states` 和 `/odom`。
这些是只读预览，不会覆盖离线草稿。

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

另一个终端导入场景和机器人：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  spawn --phase physx_diff_contact
```

编辑器可直接 `加载地图`，不要求单独运行 `walkable_map_publisher`。只有希望地图或 anchor
由 ROS 在线推送时才需要该节点。

场景碰撞发生变化后，需要在 bridge 和资产已加载的情况下重新导出多高度 walkable map：

```bash
ros2 run ros2isaacsim export_walkable_map \
  --sample-heights 0.15,0.45,0.75,1.05
```

导出结果对四个高度的占据取并集，任一高度被静态碰撞体占据，该 XY cell 就不可通行。

## 在线执行场景

保存的 JSON 由 `toilet_benchmark` 的 authored scenario runner 执行。它负责机器人 reset、
稳定 ID 行人 spawn/reactivate、路线分段、停留、终点判定和退出；同一批 actor 不应由
其他运行器同时控制。

当前 runner 的 phase、停留和终态由 benchmark 自己持有。每个 phase 将 UI 路线展开后的
完整 `PathPoints` 一次性交给 People/MotionMatching，以保留 Isaac 原生连续步态；实际 pose
只用于出生和 waypoint 确认。runner 不读取 People 内部路径游标或终态，People 也不能改变
UI 保存的语义目标和事件顺序。

运行命令见 `toilet_benchmark` runner 的 `--help`，典型形式为：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_authored_scenario \
  --episode /absolute/path/to/narrow_head_on_001.json
```

## 接入默认人工数采

`toilet_benchmark/config/manual_collection.yaml` 默认已指向仓库示例
`examples/narrow_head_on_001.json`。完整数采时不需要另行启动
`toilet_authored_scenario`，`manual_collection_node` 会在 robot reset 和 rosbag ready 后自动
启动它，并从 JSON 派生机器人起终点和行人列表：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml
```

要换成 UI 新保存的场景，只需修改 `scenarios[*].episode_path`；不要同时手动启动
authored runner。数采节点仍只负责 reset、录包、机器人到达判定和碰撞失败，行人执行
由 authored runtime 负责。

这条第一版闭环用于验证“离线设计是否被 Isaac 原样执行”。动态社会避让是后续 runtime
层能力，不应重新塞回编辑器按钮中。

## 边界与风险

- 静态验证使用圆形半径采样，只用于快速拒绝明显穿墙路线，不替代骨骼扫掠检测；
- `constrain_to_path=true` 强调复现编辑路线，不保证两个对向行人自动协商；
- 同一 actor 只能由一个 authored runner 驱动；
- 在线验证前需先启动 bridge 并导入场景资产。

设计边界见 [`docs/route_editor_scenario_design_cn.md`](docs/route_editor_scenario_design_cn.md)。
