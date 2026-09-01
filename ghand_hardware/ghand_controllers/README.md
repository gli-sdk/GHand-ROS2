# ghand_controllers

Custom `ros2_controllers` plugins for the GHand stack.

## Controllers

- `joint_status_broadcaster` - publishes `ghand_msgs/JointStatus` on `/joint_status`.
- `tactile_broadcaster` - publishes `ghand_msgs/TactileData` on `/tactile_data`.

The optional `topic` parameter changes the output topic. The dual-hand profile
uses `/left/...` and `/right/...` topics so feedback from the two devices stays
separate. The defaults above preserve compatibility with single-hand setups.

Both are broadcasters (they only read state interfaces) and are loaded through
the controller manager.

## Plugin description

Controllers are declared in `ghand_controllers.xml`.

## Dependencies

- `controller_interface`
- `ghand_msgs`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`
- `realtime_tools`
- `sensor_msgs`
- `std_msgs`