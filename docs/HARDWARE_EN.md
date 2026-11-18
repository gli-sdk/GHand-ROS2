# Hardware control

[Home](../README.md) · [Installation](INSTALLATION_EN.md)

### Connect Real Hardware

Check the device first:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ip -br link
sudo usermod -aG dialout "$USER"  # log out and back in afterwards
```

The default device ID is 49 for the left hand and 50 for the right hand.
The `ttyUSB` / `ttyACM` name depends on the adapter's USB implementation and
driver; it does not identify RS485 or CANFD. Select the communication profile
using the adapter model, wiring, and vendor documentation. RS485 uses the actual
serial path (for example, `/dev/ttyACM0`); the ZQWL CANFD configuration adds a
channel suffix (for example, `/dev/ttyACM0:0` for channel 0).
Keep the hand clear of obstacles during the first connection.

```bash
# RS485 left hand
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=left profile:=rs485_left device:=/dev/ttyUSB0

# RS485 right hand
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=right profile:=rs485_right device:=/dev/ttyUSB0

# CANFD left hand; :0 selects adapter channel 0
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=left profile:=canfd_left device:=/dev/ttyACM0:0

# EtherCAT; replace enp3s0 with the dedicated adapter
sudo setcap cap_net_raw,cap_net_admin=ep \
  /opt/ros/humble/lib/controller_manager/ros2_control_node
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=right profile:=ethercat_default ifname:=enp3s0
```

From another sourced terminal, verify the connection before commanding motion:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /joint_status --once
```

