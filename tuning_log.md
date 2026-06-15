# GHand 机械手 PID / 轨迹约束调参记录

> 用途：记录真实硬件上调参过程，避免反复试参时丢失信息。
> 调参方法：先 P 后 D 再 I，一次只调一个关节，先空载后带载。

---

## 当前基准参数

- 控制器：`joint_trajectory_controller`
- 命令接口：`position`、`velocity`、`effort`
- 状态接口：`position`、`velocity`、`effort`
- 默认 publish_rate：`100 Hz`

---

## 调参目标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 稳态误差 | < 0.01 rad | 到位后与目标差距 |
| 超调量 | < 10% | 最大过冲相对目标的比例 |
| 调节时间 | < 0.2 s | 到达目标 ±0.01 rad 内的时间 |
| 无持续震荡 | — | 到位后不应有明显抖动 |

---

## 调参记录表

### 大拇指（thumb）

| 关节 | 日期 | P | I | D | goal (rad) | trajectory (rad) | 测试条件 | 现象 | 结论/下一步 |
|------|------|---|---|---|------------|------------------|----------|------|-------------|
| thumb_tmc_ps | | | | | | | | | |
| thumb_tmc_aa | | | | | | | | | |
| thumb_tmc_fe | | | | | | | | | |
| thumb_mcp | | | | | | | | | |
| thumb_dip | | | | | | | | | |

### 食指（index）

| 关节 | 日期 | P | I | D | goal (rad) | trajectory (rad) | 测试条件 | 现象 | 结论/下一步 |
|------|------|---|---|---|------------|------------------|----------|------|-------------|
| index_mcp_aa | | | | | | | | | |
| index_mcp | | | | | | | | | |
| index_pip | | | | | | | | | |
| index_dip | | | | | | | | | |

### 中指（middle）

| 关节 | 日期 | P | I | D | goal (rad) | trajectory (rad) | 测试条件 | 现象 | 结论/下一步 |
|------|------|---|---|---|------------|------------------|----------|------|-------------|
| middle_mcp | | | | | | | | | |
| middle_pip | | | | | | | | | |
| middle_dip | | | | | | | | | |

### 无名指（ring）

| 关节 | 日期 | P | I | D | goal (rad) | trajectory (rad) | 测试条件 | 现象 | 结论/下一步 |
|------|------|---|---|---|------------|------------------|----------|------|-------------|
| ring_mcp | | | | | | | | | |
| ring_pip | | | | | | | | | |
| ring_dip | | | | | | | | | |

### 小指（little）

| 关节 | 日期 | P | I | D | goal (rad) | trajectory (rad) | 测试条件 | 现象 | 结论/下一步 |
|------|------|---|---|---|------------|------------------|----------|------|-------------|
| little_mcp | | | | | | | | | |
| little_pip | | | | | | | | | |
| little_dip | | | | | | | | | |

---

## 最终确认参数（回填到 controllers.yaml）

复制到 `src/hand_bringup/config/controllers.yaml` 中 `joint_trajectory_controller.gains` 和 `constraints` 下。

```yaml
gains:
  thumb_tmc_ps:   { p: , i: , d:  }
  thumb_tmc_aa:   { p: , i: , d:  }
  thumb_tmc_fe:   { p: , i: , d:  }
  thumb_mcp:      { p: , i: , d:  }
  thumb_dip:      { p: , i: , d:  }
  index_mcp_aa:   { p: , i: , d:  }
  index_mcp:      { p: , i: , d:  }
  index_pip:      { p: , i: , d:  }
  index_dip:      { p: , i: , d:  }
  middle_mcp:     { p: , i: , d:  }
  middle_pip:     { p: , i: , d:  }
  middle_dip:     { p: , i: , d:  }
  ring_mcp:       { p: , i: , d:  }
  ring_pip:       { p: , i: , d:  }
  ring_dip:       { p: , i: , d:  }
  little_mcp:     { p: , i: , d:  }
  little_pip:     { p: , i: , d:  }
  little_dip:     { p: , i: , d:  }

constraints:
  goal_time: 2.0
  stopped_velocity_tolerance: 0.01
  thumb_tmc_ps:   { goal: , trajectory:  }
  thumb_tmc_aa:   { goal: , trajectory:  }
  thumb_tmc_fe:   { goal: , trajectory:  }
  thumb_mcp:      { goal: , trajectory:  }
  thumb_dip:      { goal: , trajectory:  }
  index_mcp_aa:   { goal: , trajectory:  }
  index_mcp:      { goal: , trajectory:  }
  index_pip:      { goal: , trajectory:  }
  index_dip:      { goal: , trajectory:  }
  middle_mcp:     { goal: , trajectory:  }
  middle_pip:     { goal: , trajectory:  }
  middle_dip:     { goal: , trajectory:  }
  ring_mcp:       { goal: , trajectory:  }
  ring_pip:       { goal: , trajectory:  }
  ring_dip:       { goal: , trajectory:  }
  little_mcp:     { goal: , trajectory:  }
  little_pip:     { goal: , trajectory:  }
  little_dip:     { goal: , trajectory:  }
```

---

## 调参过程备忘

1. **初始值建议**
   - P：从 10~50 开始，TMC/ MCP 负载大的关节可稍高，PIP/DIP 从低开始。
   - I：初始 `0.0`，只在有稳态误差时加入。
   - D：初始 `0.0`，出现超调/震荡时加入。
   - `goal` / `trajectory`：初始 `0.05~0.1 rad`，最后收紧到 `0.01~0.02 rad`。

2. **测试指令**
   - 小幅度阶跃：目标 ±0.1 rad，看响应速度和超调。
   - 小幅正弦：振幅 0.05 rad，频率 0.5~2 Hz，看跟踪延迟和震荡。

3. **观察工具**
   - `rqt_plot` / `PlotJuggler`：对比指令位置与实际位置。
   - `ros2 topic echo /joint_states`：查看实际反馈。

4. **安全注意**
   - 首次上电用低 P、小幅度指令。
   - 手指碰到限位或物体时立即急停。
   - 调一个关节时，其他关节最好保持当前位置或禁用。

---

## 历史记录

| 日期 | 修改内容 | 修改人 | 备注 |
|------|----------|--------|------|
| 2026-06-13 | 创建调参记录模板 | Claude | 待真实硬件填充 |
