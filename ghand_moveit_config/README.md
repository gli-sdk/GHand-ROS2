# ghand_moveit_config

MoveIt 2 configuration for the GHand dexterous hand.

One package serves all products. Product data is under `config/<product_name>/`;
planning pipeline settings at `config/` root are shared. Both launch files accept
`product_name:=ghand5` (the default) or `product_name:=ghandlite1`. Lite1 plans
in the joint space of six active joints; its underactuated finger groups do not
configure a generic KDL IK solver. The vendor mimic coupling is preserved.

## Contents

- `config/ghand5/ghand5_left.srdf`, `config/ghand5/ghand5_right.srdf` — semantic robot descriptions.
- `config/ghand5/kinematics.yaml` — kinematics solver configuration.
- `config/ompl_planning.yaml`, `config/chomp_planning.yaml` — planning pipeline
  settings.
- `config/ghand5/joint_limits.yaml` — velocity and acceleration limits.
- `config/ghand5/moveit_controllers.yaml` — controller manager mapping.
- `config/ghand5/moveit.rviz` — RViz configuration.
- `launch/moveit.launch.py` — top-level MoveIt launch file.

## Usage

```bash
ros2 launch ghand_moveit_config moveit.launch.py
```

The right hand is selected by default. Use `hand_side:=left` to launch the
left-hand model:

```bash
ros2 launch ghand_moveit_config moveit.launch.py hand_side:=left
```

Launch the combined dual-hand model and both controllers with:

```bash
# Mock hardware
ros2 launch ghand_moveit_config dual_moveit.launch.py use_sim:=true

# Real hardware using two serial devices
ros2 launch ghand_moveit_config dual_moveit.launch.py \
  use_sim:=false left_device:=/dev/ttyUSB0 right_device:=/dev/ttyUSB1
```

The dual SRDF provides `left_hand`, `right_hand`, and `dual_hands` planning
groups plus combined `open` and `fist` named states.

## Dependencies

- `moveit_ros_move_group`
- `moveit_kinematics`
- `moveit_planners`
- `moveit_simple_controller_manager`
- `moveit_configs_utils`
- `moveit_ros_visualization`
- `robot_state_publisher`
- `controller_manager`
- `joint_trajectory_controller`
- `joint_state_broadcaster`
- `rviz2`
- `xacro`
- `ghand_description`
