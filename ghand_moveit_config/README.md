# ghand_moveit_config

MoveIt 2 planning for GHand5 and Lite1. Product-specific SRDF, limits,
kinematics and controller mappings live in `config/<product_name>/`.
Lite1 plans over six active joints and preserves vendor mimic coupling;
its underactuated finger groups have no generic KDL IK solver.

## Planning preview

```bash
ros2 launch ghand_bringup start.launch.py backend:=preview
ros2 launch ghand_bringup start.launch.py backend:=preview hand:=dual product_name:=ghandlite1
```

RViz **Plan** displays a trajectory. Preview publishes fixed initial joint
states using `joint_state_publisher`, and starts `robot_state_publisher`,
`move_group` and optionally RViz. There is no controller manager, hardware
plugin or trajectory execution backend. `allow_trajectory_execution` is false.
Execute and Plan & Execute are unavailable for this scene; use Plan.
The initial pose comes from `config/<product_name>/initial_positions.yaml`.

## Planning and execution

```bash
ros2 launch ghand_bringup start.launch.py backend:=mock planning:=true
ros2 launch ghand_bringup start.launch.py backend:=gazebo planning:=true
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=true profile:=rs485_right device:=/dev/ttyUSB0
```

Gazebo simulates motion and contact. Hardware executes on the physical hand.
These scenes use actual joint feedback and the existing trajectory Action.
Preview is kinematic planning, not a dynamics simulation.

## Lower-level planning attachment

```bash
# Preview only
ros2 launch ghand_moveit_config moveit.launch.py hand_side:=left
ros2 launch ghand_moveit_config dual_moveit.launch.py

# Attach to an already running physical bringup, with its RViz disabled
ros2 launch ghand_moveit_config moveit.launch.py mode:=attached hand_side:=right
```

Arguments: `product_name` (default `ghand5`), `hand_side` (`left`, `right`,
`dual`; default `right`), `mode` (`preview`, `attached`; default `preview`),
`use_rviz` (default true).
The dual wrapper fixes `hand_side` to `dual`.
Attached mode starts only planning and optional RViz. It expects matching
product/hand joint feedback and controllers from physical bringup.
Use the Gazebo scene entry for simulation, which supplies its own description and clock.

The dual SRDF provides `left_hand`, `right_hand`, `dual_hands`, and combined
`open`/`fist` named states. The old `use_sim` argument is no longer supported.
