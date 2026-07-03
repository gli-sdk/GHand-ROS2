# hand_msgs

Custom ROS 2 message definitions for the GHand dexterous hand.

## Messages

- `msg/JointStatus.msg` — per-joint status and error codes.
- `msg/TactileData.msg` — tactile resultant and distributed forces.

## Usage

After building the workspace, the messages are available in any ROS 2 node:

```bash
ros2 topic echo /joint_status hand_msgs/JointStatus
ros2 topic echo /tactile_data hand_msgs/TactileData
```

## Dependencies

- `rosidl_default_generators` (build time)
- `rosidl_default_runtime` (runtime)
