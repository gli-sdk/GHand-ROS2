# 实机控制

[返回首页](../README_CN.md) · [安装与编译](INSTALLATION_CN.md)

### 连接前检查

先断开其他可能占用设备的程序，再确认系统识别到了通信设备：

```bash
# 查看 USB 串口设备；RS485 和 CANFD 适配器都可能使用这些名称
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# 查看设备标识，辅助确认适配器与端口的对应关系
ls -l /dev/serial/by-id/ 2>/dev/null

# EtherCAT 网卡名
ip -br link
```

`ttyUSB` / `ttyACM` 名称取决于适配器的 USB 实现与驱动，不能用于判断
RS485 或 CANFD 协议。请结合适配器型号、接线及厂商说明选择通信配置。
RS485 的 `device` 使用实际串口路径（如 `/dev/ttyACM0`）；本项目的 ZQWL
CANFD 配置使用串口路径加通道后缀（如 `/dev/ttyACM0:0`，表示通道 0）。

RS485/CANFD 用户需要串口权限。添加权限后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

设备重新插拔后，名称可能变化。启动前应再次确认 `/dev/ttyUSB*` 或
`/dev/ttyACM*`，不要默认它永远是 `0`。

### 首次连接真实手

先确认手型与设备 ID：默认左手 ID 为 `49`，右手 ID 为 `50`。首次测试时确保
手指周围没有障碍物，并保持急停或断电手段可用。

RS485：

```bash
# 左手，默认 /dev/ttyUSB0、1 Mbps、ID 49
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=left profile:=rs485_left device:=/dev/ttyUSB0

# 右手，默认 /dev/ttyUSB0、1 Mbps、ID 50
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=right profile:=rs485_right device:=/dev/ttyUSB0
```

CANFD（ZQWL CDC 串口适配器）：

```bash
# 左手；`:0` 表示适配器通道 0
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=left profile:=canfd_left device:=/dev/ttyACM0:0

# 右手
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=right profile:=canfd_right device:=/dev/ttyACM0:0
```

EtherCAT：

```bash
# SOEM 需要原始网络套接字权限；每次重新安装 ros2_control 后需重新设置
sudo setcap cap_net_raw,cap_net_admin=ep   /opt/ros/jazzy/lib/controller_manager/ros2_control_node

# 将 enp3s0 替换为直连灵巧手的网卡；该网卡不要配置普通网络业务
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false \
  hand:=right profile:=ethercat_default ifname:=enp3s0
```

启动成功后，在另一个已 source 的终端验证：

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /joint_status --once
```

应看到硬件状态为 `active`，且 `joint_state_broadcaster`、
`joint_trajectory_controller` 为 `active`。确认状态正常后，再执行后面的张开/握拳命令。

如果连接失败，优先检查：设备路径和权限、左右手 ID、波特率、EtherCAT 网卡名，
以及编译日志中对应后端是否显示为 enabled。

