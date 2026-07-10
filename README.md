## Installation

The easiest way to install Arena-Rosnav is to use the [automatic installation](https://arena-rosnav.readthedocs.io/en/latest/tutorials/installation/).

## Isaac + HuNav Notes

For this workspace, `sim:=isaac` is wired to the `arena-isaac` bridge instead of
launching `ros2isaacsim` as a plain ROS node. This keeps Isaac Sim startup
aligned with the working `python.sh -m ros2isaacsim.run_isaacsim` flow used by
`arena-isaac`.

Practical implications:

- Set `isaac_path:=/abs/path/to/isaac-sim-4.5.0` when launching from
  `arena_bringup` if `ISAAC_PATH` is not already exported.
- `human:=hunav` with `sim:=isaac` now enables the Isaac People stack,
  character services, and `people_extension_mode=replicator_agent_core` by
  default so `/isaac/spawn_pedestrian` and `/isaac/move_pedestrians` can be
  used by upper-layer human simulators.
- If the installed `ros2isaacsim` package is missing its launch file, the
  bringup falls back to the source-tree launch file under
  `src/arena/arena-isaac/ros2isaacsim/launch/run_isaacsim.launch.py`.

## Toilet Benchmark Package

The workspace now also contains a small `toilet_benchmark/` package intended as
the upper-layer event director for toilet-scene benchmarking. Its current role
is to hold:

- semantic anchor configuration (`config/toilet_semantics.yaml`)
- benchmark parameters (`config/toilet_benchmark.yaml`)
- a minimal `toilet_director_node`

This package is intentionally separate from `arena-isaac`: Isaac remains the
runtime/actor/sensor layer, while the benchmark package is meant to own
scenario-specific event logic and later Hunav coordination.
