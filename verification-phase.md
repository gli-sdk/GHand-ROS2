# GHand MoveIt + Mock 仿真第一阶段验证文档

## 验证目标

验证 `moveit.launch.py` 在 `use_sim:=true` 模式下能够正常启动 MoveIt + mock 硬件链路，并支持运动规划与执行。

## 适用范围

- Change: `moveit-mock-and-gazebo-simulation`
- Phase: 第一阶段（MoveIt + mock 跑通）
- 对应 Spec: `specs/moveit-mock-simulation/spec.md`

## 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 / 24.04 |
| ROS 2 版本 | Jazzy Jalisco |
| 构建工具 | colcon |
| 图形环境 | 支持 RViz 2 的桌面会话 |

## 前置步骤

```bash
cd ~/GHand-Ros2
colcon build --symlink-install
source install/setup.bash
```

## 测试用例

### TC-001: MoveIt mock 启动成功

**对应需求**: MoveIt launch supports mock-only mode

**步骤**:

```bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=true
```

**预期结果**:

- [ ] launch 完成，无报错
- [ ] 不提示 `joint_status_broadcaster` 或 `tactile_broadcaster` 激活失败
- [ ] RViz 窗口自动打开
- [ ] 终端无 `std::runtime_error` 或 abort

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-002: mock 模式下只加载兼容控制器

**对应需求**: Mock mode skips custom broadcasters

**步骤**:

```bash
# 新终端
source ~/GHand-Ros2/install/setup.bash
ros2 control list_controllers
```

**预期结果**:

- [ ] `joint_state_broadcaster` 状态为 `active`
- [ ] `joint_trajectory_controller` 状态为 `active`
- [ ] 不加载 `joint_status_broadcaster`
- [ ] 不加载 `tactile_broadcaster`

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-003: 硬件接口正确

**步骤**:

```bash
ros2 control list_hardware_interfaces
```

**预期结果**:

- [ ] 18 个关节具有 `position` 命令接口
- [ ] 18 个关节具有 `position` 状态接口
- [ ] 18 个关节具有 `velocity` 状态接口
- [ ] 不出现 `*/state`、`*/error`、`*resultant_force*` 等接口

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-004: /joint_states 正常发布

**步骤**:

```bash
ros2 topic echo /joint_states
```

**预期结果**:

- [ ] `name` 数组包含 18 个关节名
- [ ] `position` 数组长度与 `name` 一致
- [ ] 数据持续更新

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-005: MoveIt 能规划到预设状态

**对应需求**: MoveIt can plan for the full hand group

**步骤**:

1. 在 RViz 的 MotionPlanning 插件中，选择 **Planning Group** 为 `hand`
2. **Goal State** 选择 `fist`
3. 点击 **Plan**

**预期结果**:

- [ ] 规划成功，状态栏显示成功信息
- [ ] 规划的轨迹在 RViz 中预览显示

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-006: MoveIt 能执行轨迹到 /joint_states

**对应需求**: MoveIt can execute planned trajectories in mock mode

**步骤**:

1. 在 RViz 中完成 TC-005 的规划
2. 点击 **Execute**
3. 观察 RViz 模型和 `/joint_states`

**预期结果**:

- [ ] RViz 中手指模型按规划运动到 `fist` 状态
- [ ] `/joint_states` 中的位置值持续变化并最终稳定在目标值
- [ ] 再次选择 `open` 并执行，手指能回到张开状态

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-007: RViz 正确显示模型

**对应需求**: RViz visualizes the hand in mock mode

**步骤**:

1. 启动 `use_sim:=true`
2. 观察 RViz 窗口

**预期结果**:

- [ ] GHand 模型完整显示
- [ ] 无 `robot_description` 错误
- [ ] 无 TF 错误
- [ ] MotionPlanning 插件能加载 SRDF 规划组

**实际结果**: ____________________

**通过标准**: 全部勾选

---

## 回归验证

### TC-008: hand_bringup mock 模式未被破坏

**步骤**:

```bash
ros2 launch hand_bringup ghand.launch.py use_mock_hardware:=true
```

**预期结果**:

- [ ] 正常启动
- [ ] `/joint_states` 持续发布 18 个关节状态

**实际结果**: ____________________

**通过标准**: 全部勾选

---

### TC-009: hand_bringup 真实硬件模式未被破坏

**步骤**:

```bash
ros2 launch hand_bringup ghand.launch.py profile:=ethercat_default
```

**预期结果**:

- [ ] 加载 `GHand_hardware/GHandHardwareInterface`
- [ ] 不报错（在硬件已连接前提下）

**实际结果**: ____________________

**通过标准**: 全部勾选

---

## 验证结论

| 测试项 | 状态 |
|--------|------|
| TC-001 | 未执行 / 通过 / 失败 |
| TC-002 | 未执行 / 通过 / 失败 |
| TC-003 | 未执行 / 通过 / 失败 |
| TC-004 | 未执行 / 通过 / 失败 |
| TC-005 | 未执行 / 通过 / 失败 |
| TC-006 | 未执行 / 通过 / 失败 |
| TC-007 | 未执行 / 通过 / 失败 |
| TC-008 | 未执行 / 通过 / 失败 |
| TC-009 | 未执行 / 通过 / 失败 |

**总体结论**: ____________________

**验证人**: ____________________

**日期**: ____________________

## 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| launch 报 "Could not find resource" | 未 source 环境 | `source install/setup.bash` |
| RViz 模型不显示 | `robot_description` 未加载 | `ros2 topic echo /robot_description` |
| MotionPlanning 无法选择规划组 | SRDF 未加载 | 检查 RViz 参数 `robot_description_semantic` |
| Plan 成功但 Execute 无反应 | JTC 未 active | `ros2 control list_controllers` |
| 手指运动后回弹 | mock 镜像失败 | 检查 `GenericSystem` 接口配置 |
