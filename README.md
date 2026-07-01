# GHand-ROS2

GHand 五指灵巧手的 ROS 2 控制栈，基于 `ros2_control` 框架构建。项目将底层硬件抽象与上层应用逻辑分离，支持仿真优先开发，并可与 Gazebo、MoveIt 2、Isaac Sim 等工具链对接。

---

## 技术栈

| 项目 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 22.04 LTS 及以上 |
| ROS 2 | Humble Hawksbill / Jazzy |
| 开发语言 | C++17、Python 3.10+ |
| 构建工具 | colcon（C++ 使用 CMake） |
| 仿真器 | Gazebo Harmonic |
| 运动规划 | MoveIt 2 |
| EtherCAT 主站 | SOEM（`libsoem`） |
| CANFD 主站 | ZLG CAN（`libzlgcan`） |

---

## 架构

```text
┌─────────────────────────────────────┐
│  应用层：MoveIt 2 / RViz / 自定义节点  │
├─────────────────────────────────────┤
│  控制器层：joint_trajectory_controller │
│           joint_state_broadcaster    │
│           joint_status_broadcaster   │
│           tactile_broadcaster        │
├─────────────────────────────────────┤
│  ros2_control：GHand_hardware        │
│  (on_init / on_configure / read /   │
│   write)                            │
├─────────────────────────────────────┤
│  硬件驱动层：EtherCAT / CANFD / RS485  │
└─────────────────────────────────────┘
```

- **硬件层**：通过统一的 `communication.hpp` 抽象 EtherCAT、CANFD、RS485 三种通讯方式。
- **ros2_control 层**：`GHand_hardware` 实现 `hardware_interface::SystemInterface`，完成硬件生命周期管理与周期读写。
- **控制器层**：自定义 broadcaster 发布关节状态 `/joint_status` 与触觉数据 `/tactile_data`。
- **启动层**：`hand_bringup` 负责加载机器人描述、控制器管理器与各类控制器。

---

## 包结构

| 包名 | 说明 |
|------|------|
| `ghand_description` | 手部 URDF/Xacro 描述与 STL 模型 |
| `ghand_hardware_interface` | 硬件组件，实现 `hardware_interface::SystemInterface` |
| `hand_controllers` | 自定义 broadcaster：`joint_status_broadcaster`、`tactile_broadcaster` |
| `hand_msgs` | 自定义消息类型：`JointStatus`、`TactileData` |
| `hand_bringup` | 启动文件与控制器/机器人管理器配置 |
| `ghand_moveit_config` | MoveIt 2 运动规划配置 |
| `ghand_gazebo`        | Gazebo 物理仿真启动与配置 |

---

## 快速开始

### 1. 环境准备

- 安装 ROS 2（Humble 或 Jazzy）并配置好环境变量。
- 安装依赖（以 Jazzy 为例）：

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-moveit ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

- 如需连接真实硬件，确保已安装 `libsoem` 和/或 `libzlgcan`。

#### 可选：安装真实硬件依赖

以下依赖仅在连接真实硬件时需要；纯仿真开发可跳过。

**EtherCAT（SOEM）**

ROS 2 Jazzy 没有提供 `ros-jazzy-soem` apt 包，需要从源码编译：

```bash
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/pq/SOEM/build/install -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc)
make install
```

> 说明：
> - `CMAKE_POSITION_INDEPENDENT_CODE=ON` 必须加，因为 `libsoem.a` 是静态库，要被链接进本项目的动态库。
> - 本项目 `CMakeLists.txt` 会自动探测 `~/SOEM/build/install/cmake` 下的 `soemConfig.cmake`，无需手动设置环境变量。

**RS485（libmodbus）**

```bash
sudo apt update
sudo apt install -y libmodbus-dev
```

**CANFD（ZLG CAN）**

ZLG CAN 为周立功 CAN 接口私有 SDK，需从官方渠道获取：

1. 下载对应 Linux 版本的 ZLG CAN 驱动与 SDK（通常为 `libzlgcan.so` 及头文件 `zlgcan.h`）。
2. 将头文件放到 `/usr/local/include/`，库文件放到 `/usr/local/lib/`：

```bash
sudo cp zlgcan.h /usr/local/include/
sudo cp libzlgcan.so /usr/local/lib/
sudo ldconfig
```

3. 验证 CMake 能否找到：

```bash
cd ~/GHand-Ros2
colcon build --packages-select ghand_hardware_interface
```

预期出现：`Found ZLG CAN library; CANFD backend enabled`（或不再提示 disabled）。

### 2. 编译

```bash
cd ~/GHand-Ros2  # 或你的仓库目录
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动仿真（无需硬件）

```bash
ros2 launch hand_bringup ghand.launch.py use_mock_hardware:=true
```

仿真模式使用 `mock_components/GenericSystem` 作为后端，无需 EtherCAT/CANFD/RS485 设备，可用于开发、可视化与 CI。

### 4. 启动真实硬件

真实硬件模式加载 `GHand_hardware/GHandHardwareInterface`，并通过 **通讯配置文件** 选择后端与设备参数。启动前请确认：

- 已选择对应硬件的配置文件；
- EtherCAT/RS485/CANFD 接口已正确连接并配置；
- 当前用户有串口/CAN 设备读写权限（RS485 需 `dialout` 组）。

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

> 请将 `enx000000000000` 替换为你本机的 EtherCAT 网卡接口名。可通过 `ip link` 命令查看可用接口。

#### 4.1 通讯配置文件

所有后端参数统一放在 `src/hand_bringup/config/profiles/` 下，一条 `profile` 参数即可切换后端：

| 配置文件 | 后端 | 说明 |
|----------|------|------|
| `ethercat_default` | EtherCAT | 通过 `ifname` 参数指定网卡 |
| `rs485_left` | RS485 | 左手，从站 ID `49` (`0x31`)，设备 `/dev/ttyUSB0` |
| `rs485_right` | RS485 | 右手，从站 ID `50` (`0x32`)，设备 `/dev/ttyUSB0` |
| `canfd_usb` | CANFD (ZLG USB-CANFD) | 设备 `ZCAN_USBCANFD_100U:0:0`，ID `49` |
| `zqwl_canfd_left` | ZQWL USB-CANFD | 左手，串口 `/dev/ttyACM0`，CAN ID `49` |
| `zqwl_canfd_right` | ZQWL USB-CANFD | 右手，串口 `/dev/ttyACM0`，CAN ID `50` |

统一参数命名（配置文件与底层驱动均使用）：

| 参数 | 说明 |
|------|------|
| `communication_type` | 后端类型：`ethercat` / `rs485` / `canfd` / `zqwl_canfd` |
| `device` | 设备路径或标识 |
| `baud_rate` | RS485 串口波特率 / CANFD 仲裁段波特率 |
| `data_rate` | CANFD 数据段波特率 |
| `node_id` | 从站 ID / CAN 设备 ID（原 `slave_id` / `device_id`） |
| `ifname` | EtherCAT 网卡名 |

为兼容旧 launch 命令，以下旧参数仍可作为覆盖使用（优先级高于配置文件）：

| 旧参数 | 映射到统一参数 |
|--------|----------------|
| `communication_type` | `communication_type` |
| `device` | `device` |
| `baud_rate` | `baud_rate` |
| `dbit_rate` | `data_rate` |
| `slave_id` | `node_id` |
| `ifname` | `ifname` |

示例：使用配置文件但临时覆盖设备节点

```bash
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left device:=/dev/ttyUSB1
```

#### 4.2 编译检查

```bash
cd ~/GHand-Ros2
# 若系统未安装 libmodbus，可从源码本地构建（RS485 必需）
export PKG_CONFIG_PATH=/home/pq/GHand-Ros2/vendor/libmodbus-install/lib/pkgconfig:$PKG_CONFIG_PATH
colcon build --packages-select ghand_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

预期结果：
- `ghand_hardware_interface` 编译成功，无 error、无警告。
- 若 `libmodbus` 找到，CMake 不再提示 `RS485 backend will be disabled at runtime`。

#### 4.3 RS485 验证

##### 环境准备

1. 安装或构建 `libmodbus`：

```bash
# 方式一：系统包（推荐）
sudo apt update
sudo apt install -y libmodbus-dev

# 方式二：从源码本地构建
export PKG_CONFIG_PATH=/home/pq/GHand-Ros2/vendor/libmodbus-install/lib/pkgconfig:$PKG_CONFIG_PATH
```

2. 将当前用户加入 `dialout` 组并重新登录：

```bash
sudo usermod -aG dialout $USER
# 重新登录或执行 newgrp dialout
groups  # 确认包含 dialout
```

3. 确认设备节点：

```bash
ls -l /dev/ttyUSB*
```

##### 启动 RS485

```bash
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left
```

> `node_id` 需与机械手驱动板的 Modbus 从站 ID 一致；`rs485_left` 为 `49`（`0x31`），`rs485_right` 为 `50`（`0x32`）。

如需临时覆盖设备节点或从站 ID：

```bash
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left device:=/dev/ttyUSB1 slave_id:=50
```

##### 验证

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

预期结果：
- 日志出现 `Successfully configured!` / `Successfully activated!`。
- `/joint_states` 持续更新，位置/速度与真实手指一致。
- 发送位置指令后手指实际运动，无串口超时/CRC 错误。

##### 发送位置指令

```bash
# 方式一：使用项目示例脚本（基于 joint_trajectory_controller）
ros2 run hand_bringup send_hand_position.py

# 方式二：使用 position_controller 直接发布（需先切换控制器）
python3 /home/pq/send_hand_position_direct.py
```

##### 性能说明

早期实现中 `Rs485Driver::write()` 对每个关节单独发 Modbus 帧，导致 100 Hz 控制周期严重 overrun（write 耗时约 88 ms）。当前版本已改为**批量写连续保持寄存器**（0x0011~0x002A，共 26 个寄存器），write 时间降至约 10 ms 以内，满足实时控制需求。

#### 4.4 EtherCAT 验证

启动：

```bash
ros2 launch hand_bringup ghand.launch.py profile:=ethercat_default
```

如需指定其他 EtherCAT 网卡：

```bash
ros2 launch hand_bringup ghand.launch.py profile:=ethercat_default ifname:=eth0
```

验证：

```bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

预期结果：
- 日志出现 `Initialized ethercat communication backend.`
- PDO 数据正常刷新，手指运动后 `/joint_states` 同步更新。

#### 4.5 CANFD 验证

CANFD 后端使用 `vendor/usbcanfd_libusb_x64_1.0.12_251029/` 下的周立功 VCI 库：

```text
vendor/usbcanfd_libusb_x64_1.0.12_251029/libusbcanfd.so
vendor/usbcanfd_libusb_x64_1.0.12_251029/libzuds.so
vendor/dist/include/zlgcan/zcan.h
```

无需额外安装，编译时会自动链接 vendor 库。

##### 编译

```bash
cd ~/GHand-Ros2
colcon build --packages-select ghand_hardware_interface ghand_description hand_bringup --symlink-install
source install/setup.bash
```

预期：`ghand_hardware_interface` 编译成功，CMake 不再提示 `ZLG CAN library not found`。

##### 启动 CANFD

`device` 格式为 `设备类型:设备索引:通道索引`，例如 `ZCAN_USBCANFD_100U:0:0`。`baud_rate` 对应 CANFD 仲裁段波特率，`data_rate` 对应数据段波特率。

```bash
ros2 launch hand_bringup ghand.launch.py profile:=canfd_usb
```

如需覆盖设备或波特率：

```bash
ros2 launch hand_bringup ghand.launch.py \
  profile:=canfd_usb \
  device:=ZCAN_USBCANFD_100U:0:0 \
  baud_rate:=1000000 \
  dbit_rate:=5000000 \
  slave_id:=49
```

> 旧参数 `dbit_rate`、`slave_id` 仍可使用，分别映射到统一参数 `data_rate`、`node_id`。
> 若设备 CAN 时钟不是 60 MHz，可在配置文件中修改 `can_clk`，或使用旧参数 `can_clk:=<频率>` 覆盖。

##### 验证

CANFD 后端主要用于读取触觉数据：

```bash
ros2 topic echo /tactile_data
```

预期结果：
- 日志出现 `Successfully configured!` / `Successfully activated!`。
- 触碰传感器时，`resultant_force` 和 `distributed_forces` 数值明显变化。

#### 4.6 ZQWL USB-CANFD 验证

ZQWL 后端通过 USB 串口转 CANFD，使用 `zqwl_canfd_left` / `zqwl_canfd_right` 配置文件：

```bash
ros2 launch hand_bringup ghand.launch.py profile:=zqwl_canfd_left
```

如需覆盖串口设备或 CAN ID：

```bash
ros2 launch hand_bringup ghand.launch.py \
  profile:=zqwl_canfd_left \
  device:=/dev/ttyACM1 \
  slave_id:=50
```

> 旧参数 `slave_id` 映射到统一参数 `node_id`。

#### 4.7 无硬件时的错误提示

断开设备后启动：

```bash
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left
```

预期结果：
- 日志清晰打印 `Failed to initialize rs485 communication backend.`。
- ⚠️ 当前版本在配置失败时 `ros2_control_node` 会抛出 `std::runtime_error` 并 abort；建议后续在 `on_configure` 失败路径中优雅降级到 `on_error()` 状态，避免进程崩溃。

### 5. 启动 MoveIt 2

```bash
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch ghand_moveit_config moveit.launch.py
```

默认使用 mock 硬件。如需连接真实硬件，指定 `profile` 参数：

```bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=false profile:=rs485_left
```

### 6. 启动 Gazebo 仿真

确保已安装 Gazebo 依赖：

```bash
sudo apt update
sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz
```

启动 Gazebo 物理仿真（包含 MoveIt）：

```bash
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

启动后，Gazebo 会加载 `empty.world` 并生成 GHand 模型。RViz 中可通过 MotionPlanning 插件规划轨迹，轨迹会发送到 Gazebo 中的 `joint_trajectory_controller` 物理执行。

---

## 控制机械手

启动 `hand_bringup` 后，可以通过 `rqt_joint_trajectory_controller` 或示例脚本发送关节轨迹指令，快速验证控制器是否正常工作。

### 使用 `rqt_joint_trajectory_controller`

> 注意：GHand URDF 包含 18 个关节，但 EtherCAT 从站实际只驱动其中 13 个主动关节（不含 DIP 被动关节）。`rqt_joint_trajectory_controller` 会基于 URDF 生成 18 个滑块，直接向 `joint_trajectory_controller` 发送 18 关节轨迹会导致 JTC 因被动关节无法跟踪而失败，真实硬件下不会运动。
>
> 因此：
> - **mock 模式**（`use_mock_hardware:=true`）可以直接使用 rqt；
> - **真实硬件**需要先用 `rqt_jtc_bridge` 把 rqt 的 18 关节 topic 转换成 13 关节 action goal。

#### mock 模式 / 可视化调试

```bash
ros2 launch hand_bringup ghand.launch.py use_mock_hardware:=true
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

打开界面后：
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch hand_bringup ghand.launch.py use_mock_hardware:=false

1. 在左上角选择对应的控制器管理器命名空间（通常为空，即默认命名空间）。
2. 在控制器下拉框中选择 `joint_trajectory_controller`。
3. 拖动各关节滑块，点击 **Enable** 后即可向机械手发送位置指令。

#### 真实硬件

```bash
# 终端 1：启动真实硬件
ros2 launch hand_bringup ghand.launch.py use_mock_hardware:=false

# 终端 2：启动 rqt -> JTC bridge（过滤 18->13 关节）
ros2 run hand_bringup rqt_jtc_bridge.py

# 终端 3：启动 rqt
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

然后按 mock 模式同样的步骤操作。

> 注意：Jazzy 版本的 `rqt_joint_trajectory_controller` 有时会出现 **Enable 按钮状态没有正确同步** 的情况。如果拖动滑块后机械手没有反应，请尝试先点击 **Disable**，再点击 **Enable**，然后重新拖动滑块。

### 使用示例脚本（推荐真实硬件）

仓库提供了 `send_hand_position.py` 示例脚本，直接通过 action 向 JTC 发送 13 个主动关节的轨迹：

```bash
ros2 run hand_bringup send_hand_position.py
```

> 使用前请确保 `joint_trajectory_controller` 已成功加载并处于 `active` 状态，可通过以下命令检查：
>
> ```bash
> ros2 control list_controllers
> ```

---

## 常用话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节位置、速度、力矩 |
| `/joint_status` | `hand_msgs/JointStatus` | 关节状态码与错误码 |
| `/tactile_data` | `hand_msgs/TactileData` | 触觉合力与分布力 |
| `/controller_manager/robot_description` | `std_msgs/String` | 机器人 URDF 描述 |

---

## 配置说明

- `src/hand_bringup/config/profiles/`：通讯后端配置文件（EtherCAT/RS485/CANFD/ZQWL CANFD）。
- `src/hand_bringup/config/ros2_control.yaml`：硬件组件参数、通讯接口配置、关节列表。
- `src/hand_bringup/config/controllers.yaml`：控制器管理器参数、控制器类型与 PID 参数。
- `src/hand_bringup/config/ghand.rviz`：RViz 可视化配置。
- `src/ghand_moveit_config/config/`：MoveIt 运动学、规划器、关节限制等配置。

---

## 开发规范

- C++ 头文件使用 `.hpp`，源文件使用 `.cpp`。
- 插件描述文件使用 `.xml`（如 `hand_hardware_interface.xml`）。
- 消息定义放在 `msg/*.msg`。
- 启动文件使用 Python 格式（`.launch.py`）。
- 配置文件统一放在各包的 `config/` 目录下。

---

## 目录结构

```text
GHand-Ros2/                              # ROS 2 工作空间
├── src/
│   ├── ghand_description/               # URDF/Xacro 与模型文件
│   ├── ghand_hardware_interface/        # 硬件接口与通讯驱动
│   │   ├── include/GHand_hardware/      # 硬件接口头文件
│   │   ├── src/communication/           # EtherCAT / CANFD / RS485 驱动
│   │   └── hand_hardware_interface.xml  # 插件导出
│   ├── hand_controllers/                # 自定义 broadcaster
│   ├── hand_msgs/                       # 自定义消息
│   ├── hand_bringup/                    # 启动与配置
│   ├── ghand_moveit_config/             # MoveIt 2 配置
│   └── ghand_gazebo/                    # Gazebo 物理仿真
├── openspec/                            # 规格驱动开发流程文档
├── ros.md                               # 项目设计文档
├── ros2_control架构图.drawio.png        # 架构图
└── README.md                            # 本文件
```

---

## 许可证

MIT


colcon build --symlink-install
source install/setup.bash

# 验证各 profile
ros2 launch hand_bringup ghand.launch.py profile:=ethercat_default
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left
ros2 launch hand_bringup ghand.launch.py profile:=canfd_usb
ros2 launch hand_bringup ghand.launch.py profile:=zqwl_canfd_left

# 验证覆盖与错误提示
ros2 launch hand_bringup ghand.launch.py profile:=rs485_left device:=/dev/ttyUSB1
ros2 launch hand_bringup ghand.launch.py profile:=nonexistent

moveIt:
cd ~/GHand-Ros2
colcon build --symlink-install
source install/setup.bash

ros2 launch ghand_moveit_config moveit.launch.py use_sim:=true

gazebo:
# 1. 安装依赖（如未安装）
sudo apt update
sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz

# 2. 构建
cd ~/GHand-Ros2
colcon build --symlink-install
source install/setup.bash

# 3. 启动 Gazebo + MoveIt
ros2 launch ghand_gazebo ghand_gazebo.launch.py

