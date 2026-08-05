# Isaac People AnimGraph 相位隔离探针

该探针用于判断连续 episode 的轨迹差异来自 authored 路线，还是来自池化
Isaac People 角色的 AnimGraph/Navigation 运行态。

## 设计边界

Isaac People 4.5 的 Python 源码公开了 `Action`、`Walk`、`PathPoints`、
`NavigationManager` 和命令级 `actual_walk_speed`，但没有公开 locomotion clip phase、
支撑脚状态或可靠的 graph reset API。因此探针不伪造“步态相位”，而是对比：

- `pooled`：每轮复用同一个 agent id；
- `fresh`：每轮使用新的 agent id，每个 AnimGraph 只执行一次。

若 fresh 稳定而 pooled 的轨迹误差随 generation 变化，即可把问题定位到复用边界。

## 使用

先正常启动 `bridge physx_diff_contact` 并导入场景，然后分别运行：

```bash
source /home/stardust/resources/arena_ws/install/setup.bash

ros2 run toilet_benchmark animgraph_phase_probe \
  --episode /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark_ui/examples/narrow_head_on_001.json \
  --agent-id toilet_agent_02 \
  --case turn \
  --lifecycle pooled \
  --repetitions 10

ros2 run toilet_benchmark animgraph_phase_probe \
  --episode /home/stardust/resources/arena_ws/src/arena/arena-rosnav/toilet_benchmark_ui/examples/narrow_head_on_001.json \
  --agent-id toilet_agent_02 \
  --case turn \
  --lifecycle fresh \
  --repetitions 10
```

`--case` 可选：

- `straight`：从出生点直接走向第二个 authored target；
- `turn`：执行完整路线但移除停顿；
- `stop_resume`：保留第一个停顿并继续后续路线。

结果默认写入 `/tmp/toilet_animgraph_phase_probe/<timestamp>/`：

- `episode_NNN.json`：该轮实际 EpisodeSpec；
- `episode_NNN.log`：authored runner 日志；
- `episode_NNN.jsonl`：逐帧 root pose、yaw、速度和 generation；
- `summary.json`：横向误差、路线误差、转向反复次数和相对第一轮 RMSE。

首要比较字段是 `first_2s_root_rmse_vs_run1_m`、
`first_2s_max_lateral_m` 和 `route_error_max_m`。
