# ghand_gazebo

[English](README.md)

在 ROS 2 Humble 上使用 Gazebo Fortress 仿真 GHand 灵巧手。所有启动文件均接受 `product_name:=ghand5`（默认）或 `product_name:=ghandlite1`。模型、控制器和 RViz 配置位于各自包的 `config/<product_name>/` 下，共用 Gazebo 宏位于 `config/common/`。模型生成工具读取 `mimic_joints.yaml`：GHand5 添加 DIP 联动，Lite1 保留原厂 URDF 的联动。

## 文件

- `launch/ghand_gazebo.launch.py`：单手仿真入口。
- `worlds/empty.world`、`worlds/grasp_world.world`：空场景与抓取任务场景。
- `scripts/generate_gazebo_robot_description.py`：为 `robot_state_publisher` 生成 Gazebo 专用机器人描述。

## 使用

```bash
ros2 launch ghand_bringup start.launch.py backend:=gazebo
ros2 launch ghand_bringup start.launch.py backend:=gazebo hand:=dual
ros2 launch ghand_bringup start.launch.py backend:=gazebo planning:=false
```

默认启用 MoveIt 及其 RViz 视图。设置 `planning:=false` 时可直接通过轨迹 Topic 或 Action 控制；`use_rviz:=false` 不启动 RViz，`headless:=true` 不启动 Gazebo 图形界面。控制器管理器由仿真插件启动，不要另起实机 bringup。时钟桥提供仿真时间；切换场景前按 Ctrl+C 退出当前启动。

底层入口：

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
ros2 launch ghand_gazebo ghand_gazebo.launch.py hand_side:=left
ros2 launch ghand_gazebo dual_hand_gazebo.launch.py
```

## 依赖

`gz_ros2_control`、`ros_gz_sim`、`ros_gz_bridge`、`ghand_description`、`ghand_moveit_config`，以及 `package.xml` 中声明的 ROS 2 启动与控制包。
