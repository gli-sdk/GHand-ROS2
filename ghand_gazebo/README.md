# ghand_gazebo

Gazebo Harmonic simulation for the GHand dexterous hand.

## Contents

- `launch/ghand_gazebo.launch.py` — main simulation launch file.
- `worlds/empty.world` — empty world.
- `worlds/grasp_world.world` — grasping task world.
- `scripts/generate_gazebo_robot_description.py` — helper to inject
  Gazebo-specific robot description into `robot_state_publisher`.

## Usage

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

Select the left-hand model at launch time with:

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py hand_side:=left
```

## Dependencies

- `gz_ros2_control`
- `ros_gz_sim`
- `ros_gz_bridge`
- `ghand_description`
- `ghand_moveit_config`
- plus standard ROS 2 launch/control packages (see `package.xml`)
