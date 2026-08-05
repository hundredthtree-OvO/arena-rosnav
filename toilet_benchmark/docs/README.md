# Toilet Benchmark 文档索引

## 当前文档

- `benchmark_design_cn.md`：当前 benchmark 的 Track、episode 和验收边界。
- `CURRENT_ARCHITECTURE_CN.md`：当前代码边界、运行所有权和主数据流。
- `manual_collection_cn.md`：人工数采配置、启动、reset 和有效性检查。
- `replay_trajectory_schema_cn.md`：Replay 轨迹字段和时间基准。
- `dataset_preparation_cn.md`：训练数据审计和导出流程。
- `animgraph_phase_probe_cn.md`：AnimGraph 生命周期隔离验证。
- `../toilet_benchmark_ui/README.md`：独立 2D Route Editor 操作说明。
- `../toilet_benchmark_ui/docs/route_editor_scenario_design_cn.md`：Route Editor 场景契约与运行边界。

## 当前边界

支持运行面是 `toilet_authored_scenario`、`manual_collection_node`、Replay、数据集和
策略工具。历史实验由 Git 保存，不在当前文档中继续维护。
