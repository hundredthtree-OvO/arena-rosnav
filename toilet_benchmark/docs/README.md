# Toilet Benchmark 文档索引

## 当前规范

1. `benchmark_design_cn.md`：benchmark 范围、Track、episode、接口和发布门槛；
2. `pedestrian_ecosystem_architecture_cn.md`：目的性行人生态、Smart Object、事件管线和冻结接口；
3. `code_structure_migration_cn.md`：从当前大文件向冻结边界迁移的执行顺序；
4. `motion_backend_evaluation_cn.md`：HuNav、ORCA/HRVO、Replay、Isaac AnimGraph 和 SMPL-H 的评估方法；
5. `replay_trajectory_schema_cn.md`：Replay 轨迹和时间基准。

发生冲突时，benchmark 对外语义以 `benchmark_design_cn.md` 为准，内部职责和端口以
`pedestrian_ecosystem_architecture_cn.md` 为准。

## 操作说明

- `hunav_takeover_cn.md`：HuNav 主链路；
- `hunav_phase0_smoke_cn.md`：隔离 smoke；
- `hunav_isaac_mirror_cn.md`：Isaac mirror；
- `manual_collection_cn.md`：人工数采。

## 实验与历史证据

- `hunav_behavior_characterization_20260730_cn.md`；
- `pedestrian_pair_guard_failure_review_20260724.md`；
- `hunav_pedestrian_pipeline_migration_plan_cn.md`。

这些文件记录实验结果和历史决策，不再单独定义当前接口。待代码迁移完成后再移动到
`docs/experiments/`，避免当前 dirty worktree 中发生大规模重命名。
