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
| 仿真器 | Gazebo Ignition 6 |
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
ros2 launch hand_bringup ghand.launch.py use_sim:=true
```

仿真模式使用 `mock_components/GenericSystem` 作为后端，无需 EtherCAT/CANFD 设备，可用于开发、可视化与 CI。

### 4. 启动真实硬件

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
```

真实硬件模式加载 `GHand_hardware/GHandHardwareInterface`，并通过 `config/ros2_control.yaml` 中的参数连接物理设备。启动前请确认：

- EtherCAT/RS485/CANFD 接口已正确连接并配置；
- `config/ros2_control.yaml` 中的 `communication_type` 与实际后端一致。

#### 4.1 切换通讯后端

修改 `src/hand_bringup/config/ros2_control.yaml`：

```yaml
communication_type: "ethercat"   # 或 "rs485" / "canfd"
read_timeout_ms: 500
```

并取消对应后端参数的注释：

- **EtherCAT**：`ifname`
- **RS485**：`device`、`baud_rate`、`slave_id` 等
- **CANFD**：`device`、`bitrate`、`dbit_rate`、`device_id`

#### 4.2 编译检查

```bash
cd ~/GHand-Ros2
colcon build --packages-select ghand_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

预期结果：
- `ghand_hardware_interface` 编译成功，无 error、无警告。

#### 4.3 RS485 验证

启动：

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
```

验证：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

预期结果：
- 日志出现 `Initialized rs485 communication backend.`
- `/joint_states` 持续更新，位置/速度与真实手指一致。
- 发送位置指令后手指实际运动，无串口超时/CRC 错误。

#### 4.4 EtherCAT 验证

启动：

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
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

CANFD 后端主要用于读取触觉数据。启动：

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
```

验证：

```bash
ros2 topic echo /tactile_data
```

预期结果：
- 日志出现 `Initialized canfd communication backend.`
- 触碰传感器时，`resultant_force` 和 `distributed_forces` 数值明显变化。

#### 4.6 无硬件时的错误提示

断开设备后启动：

```bash
ros2 launch hand_bringup ghand.launch.py use_sim:=false
```

预期结果：
- 日志清晰打印 `Failed to initialize <backend> communication backend.`，例如 `Failed to initialize ethercat communication backend.`。
- 系统进入 `on_error()` 冻结关节，不崩溃、不 segfault。

### 5. 启动 MoveIt 2

```bash
ros2 launch ghand_moveit_config moveit.launch.py
```

---

## 控制机械手

启动 `hand_bringup` 后，可以使用 `rqt_joint_trajectory_controller` 发送关节轨迹指令，快速验证控制器是否正常工作：

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

打开界面后：

1. 在左上角选择对应的控制器管理器命名空间（通常为空，即默认命名空间）。
2. 在控制器下拉框中选择 `joint_trajectory_controller`。
3. 拖动各关节滑块，点击 **Enable** 后即可向机械手发送位置指令。

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
│   └── ghand_moveit_config/             # MoveIt 2 配置
├── openspec/                            # 规格驱动开发流程文档
├── ros.md                               # 项目设计文档
├── ros2_control架构图.drawio.png        # 架构图
└── README.md                            # 本文件
```

---

## 许可证

MIT
