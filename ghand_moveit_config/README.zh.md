# ghand_moveit_config

[English](README.md)

为 GHand5 和 Lite1 提供 MoveIt 2 规划。各产品的 SRDF、限位、运动学及控制器映射位于 `config/<product_name>/`。Lite1 使用六个主动关节规划，保留原厂关节联动；欠驱动手指组不使用通用 KDL 逆运动学求解器。

## 规划预览

```bash
ros2 launch ghand_bringup start.launch.py backend:=preview
ros2 launch ghand_bringup start.launch.py backend:=preview hand:=dual product_name:=ghandlite1
```

在 RViz 中使用 **Plan** 查看规划轨迹。预览由 `joint_state_publisher` 发布 `config/<product_name>/initial_positions.yaml` 中的固定初始关节状态，并启动 `robot_state_publisher`、`move_group` 和可选的 RViz。此模式不启动控制器管理器、硬件插件或轨迹执行后端，`allow_trajectory_execution` 为 `false`，不能使用 **Execute** 或 **Plan & Execute**。

## 规划与执行

```bash
ros2 launch ghand_bringup start.launch.py backend:=mock planning:=true
ros2 launch ghand_bringup start.launch.py backend:=gazebo planning:=true
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=true profile:=rs485_right device:=/dev/ttyUSB0
```

Mock 可验证轨迹控制，Gazebo 仿真运动和接触，实机后端在设备上执行。它们使用对应后端提供的关节反馈和轨迹 Action；预览只做运动学规划，不模拟动力学。

## 底层入口

```bash
ros2 launch ghand_moveit_config moveit.launch.py hand_side:=left
ros2 launch ghand_moveit_config dual_moveit.launch.py
ros2 launch ghand_moveit_config moveit.launch.py mode:=attached hand_side:=right
```

`product_name` 默认 `ghand5`；`hand_side` 可选 `left`、`right`、`dual`，默认 `right`，双手入口固定为 `dual`；`mode` 可选 `preview`（默认）或 `attached`；`use_rviz` 默认 `true`。附加模式只启动规划和可选 RViz，要求运行中的实机 bringup 提供匹配的型号、关节反馈和控制器。为避免重复启动 RViz，可关闭实机 bringup 的 RViz。仿真请用 Gazebo 场景入口，由它提供机器人描述和时钟。

双手 SRDF 提供 `left_hand`、`right_hand`、`dual_hands` 规划组及组合命名姿态 `open`、`fist`。旧参数 `use_sim` 已移除。

## 验证状态

描述、bringup、MoveIt 和 Gazebo 四个包已在 Humble 工作空间中构建。场景构建检查覆盖两种产品、单/双手预览、实机规划附加和启用或关闭规划的 Gazebo 场景。无界面预览已启动模型发布器、关节状态发布器和 `move_group`，但 MoveIt 2.5.10 在定时 SIGINT 退出时销毁轨迹执行管理器发生段错误，原因尚未查明。完整规划服务、Gazebo 运行及图形界面、实机运动尚未验证。
