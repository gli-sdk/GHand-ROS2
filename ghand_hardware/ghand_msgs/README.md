# ghand_msgs

Custom ROS 2 message definitions for the GHand dexterous hand.

## Messages

- `msg/JointStatus.msg` - per-joint status and error codes.
- `msg/TactileData.msg` - tactile resultant and distributed forces.

## Usage

After building the workspace, the messages are available in any ROS 2 node:

```bash
ros2 topic echo /joint_status ghand_msgs/JointStatus
ros2 topic echo /tactile_data ghand_msgs/TactileData
```

Dual-hand launches publish the same message types on `/left/joint_status`,
`/right/joint_status`, `/left/tactile_data`, and `/right/tactile_data`.

## Dependencies

- `rosidl_default_generators` (build time)
- `rosidl_default_runtime` (runtime)