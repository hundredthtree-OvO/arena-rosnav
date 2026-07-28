# HuNav Phase 0 隔离验证

日期：2026-07-26

## 目的

该入口只验证 HuNav agent manager 的运动输出，不启动 Isaac Sim，也不接入
`toilet_director_node`、voxel planner 或数采流程。它用于在迁移主链路前区分：

- HuNav 本体能否稳定、可重复地计算行人运动。
- 默认社会力能处理哪些普通交互。
- 哪些硬安全问题不能依赖 HuNav 的软排斥解决。

## 构建

```bash
cd /home/stardust/resources/arena_ws
source install/setup.bash
colcon build --packages-select toilet_benchmark --symlink-install --cmake-force-configure
source install/setup.bash
```

## 运行

查看场景：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke --list-scenarios
```

运行默认八个场景：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke --seed 42
```

连续运行 20 个固定 seed：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke \
  --seed 42 \
  --seed-count 20
```

只运行指定场景：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke \
  --scenarios single_pause_resume,head_on,crossing \
  --seed 42
```

只验证厕所门 corridor：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke \
  --scenarios toilet_portal_corridor \
  --seed 42 \
  --seed-count 20
```

默认启用独立几何硬安全层。运行 raw HuNav 对照：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke \
  --disable-hard-safety \
  --disable-coordination \
  --disable-terminal-alignment \
  --seed 42
```

runner 默认自行启动独立 namespace 下的 `hunav_agent_manager`，结束时正常回收。
如需连接手动启动的 manager：

```bash
ros2 run toilet_benchmark hunav_phase0_smoke \
  --no-start-manager \
  --namespace /my_hunav_test \
  --seed 42
```

## 输出

每次运行写入：

```text
/tmp/toilet_hunav_debug/<run_id>/
  manifest.json
  result.json
  runner.log
  hunav.log
  ros_logs/
  trace_<scenario>.jsonl
```

`result.json` 记录：

- 目标完成、超时和卡住状态。
- 行人间及行人-机器人最小距离和重叠帧数。
- 非有限状态、位姿跳变、最大速度。
- 最终目标距离、停稳速度和朝向误差。
- hard safety 的介入帧数、约束类型、修正次数和最大修正距离。
- bottleneck token 的等待帧，以及 terminal alignment 的执行次数。
- portal 是否进入和清空、最大横向偏差、门中停顿时间和反向进度。

退出码：

- `0`：全部场景通过。
- `2`：runner 正常完成，但至少一个场景未通过指标。
- `1`：服务、配置或进程级错误。
- `130`：用户中断。

## 当前隔离基线

raw HuNav 的 seed 42 验证表明：

- 单行人暂停恢复、普通迎面、直角交叉均能完成。
- 固定步长下，同一 seed 的轨迹文件可字节级复现。
- 狭窄通道中默认 SFM 会出现明显行人重叠。
- 静止和移动机器人场景中也会进入双方几何半径范围。

启用 hard safety 后，seed 42 到 61 的 120 个 case 表明：

- 行人-行人和行人-机器人重叠帧总数均为 0。
- 位姿跳变总数为 0，最大单步安全修正约 0.023 m。
- 单行人、普通迎面和直角交叉 60/60 通过。
- 狭窄通道 20/20 不再穿透，但转为接触互锁。
- 两类机器人场景均完成导航，失败项仅为最终 yaw 超过测试阈值。

因此 HuNav 可以作为局部社会运动候选，几何投影层可以作为硬碰撞层，但它不能负责
狭窄区域的通行优先级或终点朝向，不能继续增加硬安全层的规划职责。

## Phase 0.2 结论

在 hard safety 之外增加两个相互独立、可关闭的薄层：

- `coordination` 只为标注过的 capacity-one bottleneck 分配确定性通行 token；等待者冻结，
  当前持有者穿过并离开冲突区后才释放下一位。
- `terminal_alignment` 只在 HuNav 已消费最终目标后设置目标 yaw，不修改最终 x/y，不参与
  中途规划。

seed 42 的反事实验证结果：

- `narrow_gate` 启用协调时完成；关闭协调时双方互锁并以 `stalled` 结束。
- `static_robot` 和 `moving_robot` 启用终点对齐时完成；关闭后均只因 `final_yaw` 失败。
- 同一 seed 的 `narrow_gate` 单跑和批跑轨迹 SHA-256 完全一致。

seed 42 到 61 的 `narrow_gate`、`static_robot`、`moving_robot` 共 60 个定向 case：

- 60/60 完成，重叠帧、残余硬约束违规、位姿跳变和非有限状态均为 0。
- `narrow_gate` 每轮有 123 到 125 个协调等待帧，证明 token 实际参与了通行。
- 每个终点执行一次纯 yaw 对齐；最大单步硬安全修正为 0.0225 m 以内。
- seed 42 的默认七场景回归为 7/7。

## Phase 0.5 Portal Corridor 结论

`toilet_portal_corridor` 使用实际语义坐标：

- outside：`[-3.80, -0.90]`
- inside plane：`[-2.24, -0.90]`
- clear plane：`[-1.84, -0.90]`
- half width：`0.45 m`

第一次验证把 HuNav locomotion goal 直接放在 clear plane，5/5 都在该平面前停止。
原因是 HuNav 会在 `goal_radius` 内消费目标，因此事件平面不能和运动停止目标共用一个点。

修正后 clear plane 保持不变，locomotion goal 沿 portal 轴线放到其后 `0.35 m`。
seed 42 到 61 的结果为：

- 20/20 完成并越过 clear plane。
- 最大横向偏差约 `0.0177 m`。
- 门中低速停顿时间为 `0 s`。
- 反向进度为 `0 m`。
- 每轮均为 68 帧。

这证明 portal 应使用“区域/平面事件 + 平面后的连续运动目标”，而不是将门中心、
inside point 或 staging point 作为必须停靠的状态节点。

## 下一阶段

Phase 0 和 live shadow mirror 已证明观测与 corridor 几何边界可行。兼容 motion backend
的 service adapter 已完成，下一阶段继续收紧 backend 边界：

- `IsaacPeopleBackend` 已封装当前 `MovePed/NavPed`，锁住现有请求行为。
- 下一步将 route 生成和 pedestrian state stream 移出 director。
- `HuNavMotionBackend` 后续成为单一逐帧运动所有者。
- 单行人 takeover 通过前，不切换多人和正式数采链路。
