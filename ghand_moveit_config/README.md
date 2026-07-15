# ghand_moveit_config

MoveIt 2 configuration for the GHand dexterous hand.

## Contents

- `config/ghand5_right.srdf` — semantic robot description.
- `config/kinematics.yaml` — kinematics solver configuration.
- `config/ompl_planning.yaml`, `config/chomp_planning.yaml` — planning pipeline
  settings.
- `config/joint_limits.yaml` — velocity and acceleration limits.
- `config/moveit_controllers.yaml` — controller manager mapping.
- `config/moveit.rviz` — RViz configuration.
- `launch/moveit.launch.py` — top-level MoveIt launch file.

## Usage

```bash
ros2 launch ghand_moveit_config moveit.launch.py
```

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
