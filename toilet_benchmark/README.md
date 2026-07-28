## Toilet Benchmark

HuNav 单行人 takeover 已提供实验入口。它保持
`bridge physx_diff_contact` 主链路不变，由 HuNav 独占行走位姿推进，并由 Isaac
external-motion adapter 驱动角色根位姿和 Walk/Idle 动画。

编译、启动、回滚及首轮观察项见
[`docs/hunav_takeover_cn.md`](docs/hunav_takeover_cn.md)。当前仅支持
`--initial-agents 1`，默认不带 `--motion-backend hunav` 时仍使用原
`IsaacPeopleBackend`。

This package is the upper-layer "event director" placeholder for toilet-scene
benchmarking on top of:

- `arena-isaac` as the simulation/robot/sensor/actor runtime
- `hunav` as the human local social-motion engine

Current contents:

- `config/toilet_semantics.yaml`
  Scene semantics and queue/resource anchor points.
- `config/toilet_benchmark.yaml`
  Benchmark parameters such as arrival rate and service times.
- `toilet_benchmark/toilet_director_node.py`
  Minimal node that loads both configs, waits for Isaac pedestrian services,
  spawns an initial batch of pedestrians, dispatches their first path, and
  runs the minimal state flow:
  `ENTERING -> WALK_TO_URINAL / QUEUEING -> USING_URINAL -> EXITING`.

Planned next steps:

1. Connect the director to Isaac pedestrian spawn/move services.
2. Add a Hunav adapter that publishes per-agent state updates.
3. Implement toilet event state machines and resource queues.

Quick check after build:

```bash
ros2 run toilet_benchmark toilet_director_node --help
ros2 run toilet_benchmark toilet_director_node --initial-agents 1
```

By default the node looks for:

- `/isaac/spawn_pedestrian`
- `/isaac/move_pedestrians`

and uses the packaged configs from `share/toilet_benchmark/config/`.

The default pedestrian assets are chosen from the packaged `character_pool`
using the same validated `original_*` Isaac People characters that the current
`arena-isaac` workflow already uses.

## Filling Toilet Semantics

The current `config/toilet_semantics.yaml` stores both:

- `scene_prim`: the scene object name you identified in Isaac Sim
- `pose` / `queue_slots`: the hand-authored benchmark anchor points actually used by pedestrians

Recommended workflow:

1. In Isaac Sim, select `Door_0000` and each `Edestalurinal_000*` prim.
2. Read their approximate world position/orientation from the property panel.
3. Do **not** use the mesh origin directly as the pedestrian target.
4. Instead, fill:
   - `entrances[*].pose` with a safe spawn point near the door
   - `urinals[*].pose` with the standing point in front of the urinal
   - `urinals[*].queue_slots` with one or two waiting points behind it

Validation:

- Run `ros2 run toilet_benchmark toilet_director_node --initial-agents 1`
- Confirm logs show the chosen entrance, urinal, and path points
- In Isaac, confirm the pedestrian:
  - spawns at the entrance
  - walks to the queue/stand point
  - pauses for the configured urinal service time
  - leaves through the exit point
