# ghand_controllers

[English](README.md)

GHand 控制栈的自定义 `ros2_controllers` 插件。

## 广播器

- `joint_status_broadcaster`：在 `/joint_status` 发布 `ghand_msgs/JointStatus`。
- `tactile_broadcaster`：在 `/tactile_data` 发布 `ghand_msgs/TactileData`。

可用 `topic` 参数修改输出话题。双手配置使用 `/left/...` 和 `/right/...` 区分两台设备的反馈；默认话题与单手配置兼容。

两个广播器均由控制器管理器加载，只读取状态接口。插件在 `ghand_controllers.xml` 中声明。

## 依赖

`controller_interface`、`ghand_msgs`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`、`realtime_tools`、`sensor_msgs`、`std_msgs`。
