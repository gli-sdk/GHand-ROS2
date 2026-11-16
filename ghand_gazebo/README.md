# ghand_gazebo

All launch files accept `product_name:=ghand5` (the default) or
`product_name:=ghandlite1`. Product-specific descriptions, controllers and RViz
settings are under `config/<product_name>/` in their own packages. The shared
description helper loads `config/<product_name>/mimic_joints.yaml`: GHand5 adds
its DIP coupling, while Lite1 preserves the coupling already in the vendor URDF.
Shared Gazebo macros are under `config/common/`.

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

Run two hands, two trajectory controllers, MoveIt, and RViz together with:

```bash
ros2 launch ghand_gazebo dual_hand_gazebo.launch.py
```

## Dependencies

- `gz_ros2_control`
- `ros_gz_sim`
- `ros_gz_bridge`
- `ghand_description`
- `ghand_moveit_config`
- plus standard ROS 2 launch/control packages (see `package.xml`)
