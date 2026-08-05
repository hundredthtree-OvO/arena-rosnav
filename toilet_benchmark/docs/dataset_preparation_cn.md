# Toilet Policy 数据准备

本工具只建立训练前的数据索引、质量标注和 session 级切分，不修改原始 rosbag，也不把未经复核的成功轨迹直接当作专家示范。

## 生成 v0 索引

```bash
source /home/stardust/resources/arena_ws/install/setup.bash
ros2 run toilet_benchmark toilet_dataset \
  --data-root /home/stardust/resources/arena_ws/data/toilet_manual \
  --session-glob 'session_20260805_*' \
  --output /home/stardust/resources/arena_ws/data/toilet_policy/v0 \
  --split-seed 42
```

输出包括：

- `index.jsonl`：每条 episode 的来源、hash、topic 数量、质量、split 和问题列表；
- `annotations.yaml`：人工质量标注；
- `splits.yaml`：按 session 分配的 train/validation/test，禁止同一 session 跨 split；
- `stats.json`：数据集统计。

## 人工复核

首次运行后，成功 episode 默认为 `pending_review`。查看对应录像或雷达可视化后，在 `annotations.yaml` 中修改：

```yaml
episodes:
  session_20260805_112142_451767_seed42/episode_000001:
    quality: clean_success
    notes: 无碰人、碰墙、穿模或异常停顿
  session_20260805_112142_451767_seed42/episode_000002:
    quality: recovered_scene_contact
    notes: 右侧机身接触隔板后恢复
```

允许的标签：

- `clean_success`：可进入第一版行为克隆；
- `recovered_scene_contact`：首版排除，保留用于恢复行为研究；
- `human_collision`：碰人失败，自动保护，不能人工改成 clean；
- `task_failure`：其他有效任务失败；
- `invalid_simulation`：漂移、穿模、传感器缺失等无效仿真；
- `pending_review`：尚未人工确认。

修改标注后重新运行同一命令。工具会保留 `annotations.yaml`，重建 index 和统计；只有无审计错误的 `clean_success` 会设置 `bc_eligible: true`。

只为验证训练和推理链路时，可以输出未经人工复核的 smoke-test 版本：

```bash
ros2 run toilet_benchmark toilet_dataset \
  --data-root /home/stardust/resources/arena_ws/data/toilet_manual \
  --session-glob 'session_20260805_*' \
  --output /home/stardust/resources/arena_ws/data/toilet_policy/v0_unreviewed \
  --split-seed 42 \
  --accept-unreviewed-success
```

该模式仍排除明确碰人、任务失败和无效 bag，并在 index 中写入 `training_admission: unreviewed_success_override`，不能作为正式 benchmark baseline。
