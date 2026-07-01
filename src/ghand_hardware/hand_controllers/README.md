# hand_controllers

Custom `ros2_controllers` plugins for the GHand stack.

## Controllers

- `joint_status_broadcaster` — publishes `hand_msgs/JointStatus` on `/joint_status`.
- `tactile_broadcaster` — publishes `hand_msgs/TactileData` on `/tactile_data`.

Both are broadcasters (they only read state interfaces) and are loaded through
the controller manager.

## Plugin description

Controllers are declared in `hand_controllers.xml`.

## Dependencies

- `controller_interface`
- `hand_msgs`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`
- `realtime_tools`
- `sensor_msgs`
- `std_msgs`
