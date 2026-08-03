# Toilet Benchmark 文档索引

## 当前规范

1. `benchmark_design_cn.md`：benchmark 范围、Track、episode、接口和发布门槛；
2. `pedestrian_ecosystem_architecture_cn.md`：目的性行人生态、Smart Object、事件管线和冻结接口；
3. `code_structure_migration_cn.md`：从当前大文件向冻结边界迁移的执行顺序；
4. `motion_backend_evaluation_cn.md`：HuNav、ORCA/HRVO、Replay、Isaac AnimGraph 和 SMPL-H 的评估方法；
5. `replay_trajectory_schema_cn.md`：Replay 轨迹和时间基准。
6. `route_editor_runtime_cleanup_plan_cn.md`：阶段 0/1 基线冻结、运行链路清理和路线编辑器迁移计划。
7. `../toilet_benchmark_ui/README.md`：阶段 3 独立 Qt 2D 路线编辑器的启动和话题说明。

发生冲突时，benchmark 对外语义以 `benchmark_design_cn.md` 为准，内部职责和端口以
`pedestrian_ecosystem_architecture_cn.md` 为准。

## 操作说明

- `hunav_takeover_cn.md`：HuNav 主链路；
- `hunav_phase0_smoke_cn.md`：隔离 smoke；
- `hunav_isaac_mirror_cn.md`：Isaac mirror；
- `manual_collection_cn.md`：人工数采。
- `route_editor_runtime_cleanup_plan_cn.md`：路线编辑器与运行链路清理的当前执行状态。

阶段 3 编辑器不属于核心 benchmark 包，位于同一仓库的 sibling 包
`toilet_benchmark_ui`，因此这里仅保留操作入口，不复制 UI 代码或配置。RViz
继续用于 LiDAR、TF 和 3D 场景诊断，不再承担路线编辑。

## 实验与历史证据

- `hunav_behavior_characterization_20260730_cn.md`；
- `pedestrian_pair_guard_failure_review_20260724.md`；
- `hunav_pedestrian_pipeline_migration_plan_cn.md`。

这些文件记录实验结果和历史决策，不再单独定义当前接口。待代码迁移完成后再移动到
`docs/experiments/`，避免当前 dirty worktree 中发生大规模重命名。

## 文档清理规则

当前不直接删除“看起来旧”的文档：索引、启动说明、实验证据和迁移决策仍可能被
回归、Replay 或后续对照使用。清理时先执行引用扫描，并满足路线计划中“无 setup
入口、无测试引用、无操作面引用、完成 shadow 对照并保留回退 tag”的门槛；之后优先
移动到 `docs/experiments/`，确认无引用后再删除。这样不会因为清理文档改变现有
benchmark 的可复现入口。
