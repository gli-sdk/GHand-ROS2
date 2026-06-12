# hand_bringup

Launch files and configuration for the GHand ros2_control stack.

## Usage

### Simulation (no hardware required)

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=true
```

This uses `mock_components/GenericSystem` as the hardware backend. The xacro
injection switches the `ros2_control` hardware plugin based on the `use_sim`
argument, so no EtherCAT/CANFD devices are needed.

### Real hardware

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
```

This loads `GHand_hardware/GHandHardwareInterface` and attempts to communicate
with the physical hand. Ensure EtherCAT/CANFD interfaces are available and
configured in `config/ros2_control.yaml`.

## Files

- `launch/ghand.launch.py` — main launch file.
- `config/ros2_control.yaml` — hardware component parameters.
- `config/controllers.yaml` — controller manager and controller parameters.
- `config/ghand.rviz` — RViz display configuration.

## Notes

- `use_sim:=true` is intended for development, CI, and visualization without the
  physical robot. It does not simulate physics or contacts (no Gazebo).
- The same controller configuration is used for both simulation and real
  hardware because both expose identical command/state interfaces.
