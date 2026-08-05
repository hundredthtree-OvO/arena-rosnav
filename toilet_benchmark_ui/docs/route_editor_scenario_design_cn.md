# Route Editor 场景闭环设计

## 边界

编辑器只维护 `ScenarioDraft` 并输出标准 `EpisodeSpec`。它不订阅运行器状态来修改草稿，
也不发布无法确认执行结果的 shadow command。

数据流固定为：

```text
walkable map -> ScenarioDraft -> EpisodeSpec JSON -> authored scenario runner -> Isaac services
```

## 场景对象

- `robot`：一个机器人出生位和朝向；
- `pedestrians[]`：每个行人拥有独立 ID、角色、出生位、朝向、速度和路线；
- `route_waypoints[]`：用户编辑的有序 world-frame 目标锚点；
- `holds[]`：绑定 waypoint index 的停留时长；
- `termination`：超时、终点容差和碰撞策略。

运行器先用静态 walkable map 在相邻目标间执行 Theta*/A*，再将 hold 索引映射到展开路径。
停留不是悬空 subgoal。运行器把规划路线切成以 hold waypoint 结尾的片段，到达后显式 stop，
定时结束再发送剩余片段。这样保存、加载和重跑都具有确定语义。

## 双行人窄通道验收

第一版验收只检查编排闭环，不把社会避让效果伪装成编辑器能力：

1. 两个行人从不同出生点激活，不共享 spawn；
2. 每个行人只消费自己的路线和速度；
3. waypoint hold 能暂停并恢复对应行人，不影响另一个 actor；
4. 机器人 reset 到 episode 定义的出生位；
5. 所有行人到终点或 timeout 后 runner 明确退出；
6. 重复固定 seed 时事件顺序、路线和停留时长一致。

之后如引入动态避让，应作为独立运动 backend 接入 authored runtime。编辑器可以增加
behavior/profile 字段，但不应包含某个规划器的 generation、phase、临时 TTL 或内部调试按钮。
