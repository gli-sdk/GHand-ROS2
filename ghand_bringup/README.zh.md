# ghand_bringup

[English](README.md) · [实机连接指南](../docs/HARDWARE_CN.md)

统一启动 GHand 的规划预览、模拟硬件、Gazebo 仿真与实机控制。

## 公共入口

```bash
ros2 launch ghand_bringup start.launch.py backend:=preview
ros2 launch ghand_bringup start.launch.py backend:=mock use_rviz:=false
ros2 launch ghand_bringup start.launch.py backend:=gazebo
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false profile:=rs485_right device:=/dev/ttyUSB0
```

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `backend` | `gazebo` | `preview`、`mock`、`gazebo`、`hardware` |
| `product_name` | `ghand5` | `ghand5`、`ghandlite1` |
| `hand` | `right` | `left`、`right`、`dual` |
| `planning` | `true` | 启用 MoveIt；预览必须开启 |
| `use_rviz` | `true` | 是否显示 RViz |
| `profile` | 空 | 实机连接配置；单手默认按手侧选择 `rs485_<hand>` |
| `device`、`ifname` | 空 | 覆盖串口/CAN 设备或 EtherCAT 网口 |
| `left_profile`、`right_profile` | `rs485_left`、`rs485_dual_right` | 双手连接配置 |
| `left_device`、`right_device` | 空 | 覆盖双手设备路径 |
| `left_ifname`、`right_ifname` | 空 | 覆盖双手 EtherCAT 网口 |
| `headless` | `false` | 仅运行 Gazebo 服务端 |
| `log_level` | `info` | ROS 节点日志级别 |
| `sdk_log_level` | `info` | GHand SDK 控制台日志级别 |
| `world` | 空 | 覆盖 Gazebo 场景；为空时使用包默认场景 |

连接参数仅用于实机，`world` 仅用于 Gazebo。双手使用各自的连接参数，但必须选择同一型号。预览发布运动学状态并运行规划，不执行轨迹；mock 使用 ros2_control 的 `GenericSystem`，无需设备或 Gazebo 即可测试 `FollowJointTrajectory`。MoveIt 在后端、控制器和轨迹 Action 就绪后启动执行功能。

构建日志位于 `log/latest_build`。运行日志目录在启动时打印，可用 `ROS_LOG_DIR` 指定，默认位于 `~/.ros/log` 下。实机配置报错时会显示当前连接配置和参数。Gazebo 的控制器管理器由仿真插件提供，实机由 bringup 提供，MoveIt 不会另起管理器。只有 Gazebo 使用仿真时间。

## 底层入口与迁移

- `ghand.launch.py` 启动单手实机，使用 `hand_side:=left/right`；`dual_hand.launch.py` 启动双手实机。二者启动状态发布器、控制器和可选的 RViz；传入 `planning:=true` 可在控制器激活后启动 MoveIt。
- 单手底层入口默认 `ethercat_default`，与公共入口按手侧选择的默认配置不同。
- 旧参数 `use_mock_hardware` 和 `use_sim` 已移除，请显式选择场景。
- `moveit.launch.py` 和 `dual_moveit.launch.py` 默认仅做规划预览；实机规划与执行请用公共入口并设置 `planning:=true`。
- 切换场景前先停止当前启动进程。

## 产品与模型资产

各实机、MoveIt 和 Gazebo 入口均接受 `product_name`。本包根据已安装的 `config/<product_name>/controllers.yaml` 发现可选控制配置。连接配置与产品无关，触觉布局由产品 Xacro 提供。新增产品时保留共用硬件插件和各 ROS 包，补齐产品控制器及资源、URDF/网格、Gazebo 配置，以及 MoveIt SRDF、限位、运动学和控制器映射；启用实机前检查协议与能力差异，不能直接沿用 GHand5 的数值。

Lite1（`ghandlite1`）有 11 个反馈关节和 6 个受控关节。保留原厂关节联动比例，`THUMB_MCP` 与四个 `*_IP` 关节为被动关节。

```bash
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 profile:=rs485_right
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 profile:=ethercat_default ifname:=enx000000000000
ros2 launch ghand_moveit_config moveit.launch.py product_name:=ghandlite1
ros2 launch ghand_gazebo ghand_gazebo.launch.py product_name:=ghandlite1
ros2 launch ghand_moveit_config dual_moveit.launch.py product_name:=ghandlite1
```

Lite1 的协议基准为 GHand-Cpp-SDK 提交 `6651e46`：6 个各占 4 个寄存器的命令块；EtherCAT 命令包含 2 字节手部头和 6 个 8 字节命令（共 50 字节）；反馈包含 11 个寄存器的头和 11 条各 10 字节的记录。触觉点数为 38/31/31/31/31，与 SDK 配置一致。支持半闭环位置、速度、电流控制及 `CUSTOM` 选择，不支持闭环、MIT 或 kp/kd。位置反馈是电机反馈，不能据此判断是否有独立位置传感器。

资产须包含 `ghand5_system` 和 `ghand_lite1_system`。非标准路径可通过 colcon/CMake 参数 `-DGHAND_ASSETS_DIR=<ghand5_system>` 和 `-DGHAND_LITE1_ASSETS_DIR=<ghand_lite1_system>` 指定。Lite1 的主动关节限位以 `ghand_description/config/ghandlite1.yaml` 为准，该文件也用于生成资源及初始关节位置数据。描述安装路径改为 `ghand_description/urdf/<product_name>/` 和 `ghand_description/meshes/<product_name>/`；迁移时使用干净的安装目录，避免保留旧配置。

实机编译、Linux 后端、MoveIt/Gazebo 启动和运动及触觉反馈需在目标 ROS 环境中验证。建议依次检查规划预览、Gazebo 和实机，确认控制器已激活且反馈有效后再发送实机运动指令。可与指定 SDK 源码核对 Lite1 元数据：

```bash
python3 tests/check_lite1_sdk_config.py --sdk-dir /path/to/GHand-Cpp-SDK
```

## SDK 与 ROS 的职责

实机使用 `ghand_hardware_interface`、`ghand_msgs`、`ghand_controllers` 和 `ghand_sdk_vendor` 随附的 GHand C++ SDK 动态库。通过 `build.sh` 构建，无需本地 SDK 源码。仅使用规划预览或 Gazebo 时可不构建实机包。SDK 管理设备传输和协议，ROS 管理生命周期、单位换算与控制接口。启动使用 ROS 原生日志和进程退出事件；规划和仿真从 `ghand_description` 发现可用产品。
