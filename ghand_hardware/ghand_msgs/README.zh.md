# ghand_msgs

[English](README.md)

GHand 灵巧手的自定义 ROS 2 消息定义。

- `msg/JointStatus.msg`：各关节的状态和错误码。
- `msg/TactileData.msg`：触觉合力及分布力。

编译并加载工作空间环境后，可查看消息：

```bash
ros2 topic echo /joint_status ghand_msgs/JointStatus
ros2 topic echo /tactile_data ghand_msgs/TactileData
```

双手启动时，左右手分别在 `/left/joint_status`、`/right/joint_status`、`/left/tactile_data` 和 `/right/tactile_data` 发布相同类型的消息。

构建依赖为 `rosidl_default_generators`，运行依赖为 `rosidl_default_runtime`。
