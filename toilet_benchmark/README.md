# Toilet Benchmark

`toilet_benchmark` 提供厕所社会导航 benchmark 的 authored 场景执行、人工数采、
Replay、数据审计和策略数据导出。

## 支持链路

1. Isaac bridge：`physx_diff_contact`
2. 场景和机器人导入：`spawn --phase physx_diff_contact`
3. 行人场景：`toilet_authored_scenario`
4. 人工数采：`manual_collection_node`
5. Replay：`toilet_replay*`
6. 数据处理：`toilet_dataset` 和 `toilet_policy_export`

路线编辑器位于 sibling 包 `toilet_benchmark_ui`。行人运行继续使用 Isaac
People/AnimGraph 和 `path_points_flat`，静态路径由 walkable map planner 生成。

## 构建

```bash
cd /home/stardust/resources/arena_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select toilet_benchmark toilet_benchmark_ui toilet_policy \
  --symlink-install
source install/setup.bash
```

## 启动 Isaac

终端 1：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  bridge physx_diff_contact
```

终端 2，导入场景和机器人：

```bash
cd /home/stardust/resources/arena_ws/src/arena/arena-isaac
source /home/stardust/resources/arena_ws/install/setup.bash
python3 scripts/arena_scene_profile.py \
  --profile scripts/profiles/shenxinfu_841837.yaml \
  spawn --phase physx_diff_contact
```

## Authored 场景

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_authored_scenario \
  --episode /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark_ui/examples/narrow_head_on_001.json \
  --planner-radius-m 0.3
```

默认采用 `detect_and_fail` 人机策略：不进行预测性人机 hard guard，实际接触由
事件判定失败。行人与静态环境的 walkable-map 路径约束保持启用。

## 人工数采

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark manual_collection_node \
  --config /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark/config/manual_collection.yaml
```

数采默认读取 authored episode，并保留机器人复位、手柄控制、rosbag、接触判负和
episode 归档。详细字段和验收见
[`docs/manual_collection_cn.md`](docs/manual_collection_cn.md)。

## 主要入口

| 命令 | 用途 |
| --- | --- |
| `toilet_authored_scenario` | 执行 UI authored episode |
| `manual_collection_node` | 人工采集和 episode reset |
| `toilet_replay` | 确定性 Replay |
| `toilet_replay_export` | 导出 Replay bundle |
| `toilet_replay_isaac` | 在 Isaac 中执行 Replay |
| `pedestrian_diagnostic_recorder` | 行人状态诊断 |
| `walkable_map_publisher` | 发布静态可行走地图 |
| `animgraph_phase_probe` | AnimGraph 生命周期隔离测试 |
| `toilet_dataset` | 数据审计和索引 |

当前代码边界见
[`docs/CURRENT_ARCHITECTURE_CN.md`](docs/CURRENT_ARCHITECTURE_CN.md)，完整文档索引见
[`docs/README.md`](docs/README.md)。
