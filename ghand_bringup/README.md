# ghand_bringup

Launch and runtime configuration for single and dual GHands.

```bash
ros2 launch ghand_bringup ghand.launch.py use_mock_hardware:=true
ros2 launch ghand_bringup ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
ros2 launch ghand_bringup dual_hand.launch.py use_mock_hardware:=true
```

`launch/` starts the state publisher, controller manager, controllers and RViz.
`config/` contains controller YAML, communication profiles, RViz settings and
entry Xacros that combine `ghand_description` with the interface macro from
`ghand_hardware_interface`. Existing launch arguments and defaults are retained.

Launch files previously provided by `ghand_hardware_interface` are now launched
from `ghand_bringup`. Communication profiles are under `config/profiles/`.
Gazebo and MoveIt keep their own launch files and specialized configuration.
