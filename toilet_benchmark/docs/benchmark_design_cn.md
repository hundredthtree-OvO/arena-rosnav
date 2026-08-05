# Toilet Benchmark 当前设计

## 1. 目标

本 benchmark 面向厕所内具有明确目的的社会导航任务。行人不是随机漫游，而是由
authored episode 指定出生点、阶段目标、停留和退出路线；机器人在同一 episode 中
完成导航、人工数采或策略评测。

当前优先保证：

- episode 可复现；
- 行人路径、生命周期和接触事件可观测；
- 数采、Replay 和评测共享同一 episode schema；
- 人机真实接触可判负，但不使用预测性 hard guard 替操作者避让；
- 运行链路可连续 reset，不依赖频繁重启 Isaac。

## 2. 支持 Track

### Authored Interactive Track

`toilet_authored_scenario` 读取 Route Editor 生成的 episode，使用 walkable map 规划
未显式 authored 的连接段，并通过 Isaac People/AnimGraph 执行路径。场景可包含多名
行人、停留点和机器人起点。

### Dataset Track

`manual_collection_node` 在 authored episode 上增加机器人 reset、手柄控制、rosbag、
接触判负和 session/episode 归档。默认输入由 `config/manual_collection.yaml` 指定。

### Replay Track

`toilet_replay*` 读取冻结轨迹，不依赖在线社会运动规划。Replay 用于确定性回放、
baseline 对照和数据检查。

## 3. 运行边界

```text
toilet_benchmark_ui
  authored episode / route / spawn / hold / robot start

toilet_benchmark
  schema / validation / authored runtime / collection / replay / dataset

arena-isaac
  PhysX robot / sensors / Isaac People / AnimGraph / contact events
```

旧 `toilet_director_node` 和 HuNav takeover 不属于当前运行面。Git 历史保留其研究
过程，但新代码不得再次依赖已删除入口。

## 4. Episode 契约

每个 episode 至少包含：

- 稳定的 `episode_id`、`scene_id` 和 seed；
- 机器人起点、目标和终止条件；
- 每名行人的角色、出生点、目标或 authored route；
- 可选 hold/subgoal 和时序约束；
- 接触、超时、完成和 reset 事件；
- 数据版本与生成配置。

UI 只负责编辑与静态验证；运行时负责生命周期、地图连接段、AnimGraph 执行和事件
输出。数采不得偷偷修改 authored 路径语义。

## 5. 失败条件

- 人机几何接触；
- 机器人与静态场景发生不允许的接触；
- 行人视觉包络持续穿透静态场景；
- 行人未在超时前完成 authored 生命周期；
- reset 后路径、朝向或 AnimGraph 游标跨 episode 漂移；
- 必需传感器、TF 或录制话题缺失。

## 6. 验证层级

1. schema/validator 单元测试；
2. planner、Replay、geometry 和 authored runtime 回归；
3. 同一 bridge 下多 episode reset 测试；
4. 单/多人固定 seed Isaac smoke；
5. 人工数采 session 审计；
6. 策略离线指标和 Isaac 闭环评测。

## 7. 当前非目标

- 继续维护旧 HuNav behavior/BT 实验；
- 继续扩展旧 Director 的厕所资源状态机；
- 用预测性人机 hard guard 代替碰撞失败标签；
- 同时维护多套互相争夺行人运动所有权的 backend。

后续新增局部运动模型或 SMPL-H embodiment 时，必须通过现有 episode、事件和 Replay
契约接入，不得改变 Dataset Track 的数据语义。
