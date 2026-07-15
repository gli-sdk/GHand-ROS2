# GHand-ROS2

[![Version](https://img.shields.io/badge/version-v1.0.0-blue.svg)](CHANGELOG.md)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)

[English](README.md)

GHand-ROS2 是 GHand 五指灵巧手的 ROS 2 控制栈，基于 `ros2_control` 框架构建。支持 Gazebo 物理仿真、MoveIt 2 运动规划，并提供 EtherCAT、CANFD、RS485 多种真实硬件通信后端，方便用户在无硬件环境下进行算法开发与可视化验证。

## 🎬 演示

### MoveIt 运动规划
![MoveIt 仿真演示](docs/moveit_demo.gif)

### Gazebo 仿真

![Gazebo 仿真演示](docs/gazebo_demo.gif)


## ✨ 功能特性

- **多后端硬件驱动**
  - 支持 EtherCAT、CANFD、RS485 三种通信方式。
  - 通过 `profile` 参数一键切换后端与设备配置。
  - 统一硬件抽象层，真实硬件与 mock 硬件接口一致。

- **ros2_control 集成**
  - 实现 `hardware_interface::SystemInterface` 生命周期管理。
  - 周期读写关节指令与状态，支持位置、速度、力矩控制模式。

- **自定义广播器**
  - `joint_status_broadcaster` 发布关节状态码与错误码。
  - `tactile_broadcaster` 发布触觉合力与分布力。

- **仿真与运动规划**
  - 内置 Gazebo Harmonic 仿真启动文件与 ROS 2 Control 插件配置。
  - 内置 MoveIt 2 配置，支持 RViz 中轨迹规划与执行。

- **场景化启动**
  - `ghand_hardware_interface` 负责真实/mock 硬件启动。
  - `ghand_gazebo` 负责物理仿真启动，二者解耦不互相依赖。

## 📖 官方文档

详细技术规格、API 参考与使用教程请查看：[GHand 灵巧手文档](https://fcnzogxju7xr.feishu.cn/docx/AhZ6ds2iCoguaAxIzBxciYHinNo)。

## 📑 目录

- [功能特性](#-功能特性)
- [官方文档](#-官方文档)
- [系统要求](#-系统要求)
- [依赖说明](#-依赖说明)
- [安装](#-安装)
- [快速开始](#-快速开始)
- [演示](#-演示)
- [控制接口](#-控制接口)
- [目录结构](#-目录结构)
- [开源与生态资源](#-开源与生态资源)
- [更新日志](#-更新日志)
- [支持与反馈](#-支持与反馈)
- [许可证](#-许可证)

## 💻 系统要求

### 支持的平台

| 平台        | 要求                                      |
| ----------- | ----------------------------------------- |
| **操作系统** | Ubuntu 24.04 LTS                          |
| **ROS 2**   | Jazzy Jalisco                             |
| **编译器**  | GCC 11+（支持 C++17）                     |
| **Python**  | 3.10 或更高版本                            |
| **仿真器**  | Gazebo Harmonic（物理仿真需要）            |

## 🔧 依赖说明

### 必需依赖

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2
```

> 将 `${ROS_DISTRO}` 替换为 `jazzy`。

### 可选硬件依赖

仅连接真实硬件时需要；纯仿真开发可跳过。

| 后端       | 依赖包/库                              | 说明                              |
| ---------- | -------------------------------------- | --------------------------------- |
| EtherCAT   | `libsoem` / SOEM                       | 从源码编译， CMake 会自动探测      |
| RS485      | `libmodbus`                            | 系统包         |
| CANFD      | ZLG CAN SDK | 周立功 USB-CANFD 库 |

## 📦 安装

### 1. 环境准备

```bash
# 检查 ROS 2 环境
echo $ROS_DISTRO  # 应该输出: jazzy

# 创建工作空间
mkdir -p <workspace>/src
cd <workspace>
```

**注意**：请将 `<workspace>` 替换为您的工作空间路径，例如 `~/ghand-ros2`。

### 2. 克隆仓库

```bash
# 进入工作空间 src 目录
cd <workspace>/src

# 克隆主仓库
git clone https://gitee.com/glitech/ghand-ros2.git
```

### 3. 导入外部依赖

`ghand_description` 中的 URDF/Xacro 与 STL 模型由外部仓库提供，通过 `ghand.repos` 引入：

```bash
cd <workspace>/src
vcs import . < ghand-ros2/ghand.repos --recursive --skip-existing
```

**说明**：如果您的系统没有安装 `vcs` 工具，请先安装：
```bash
sudo apt-get install python3-vcstool
```

### 4. 编译

```bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
```

若只需 mock 硬件构建，可直接使用：

```bash
cd <workspace>
./src/ghand-ros2/build.sh
source install/setup.bash
```

## 🚀 快速开始

### 启动仿真（无需硬件）

```bash
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true
```

增加 `hand_side:=left` 可使用左手模型；默认值为 `right`。

仿真模式使用 `mock_components/GenericSystem`，无需任何硬件设备。

### 启动真实硬件

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

> 请将 `enx000000000000` 替换为你本机的 EtherCAT 网卡接口名。可通过 `ip link` 命令查看可用接口。

可用的通讯配置文件位于 `ghand_hardware/ghand_hardware_interface/config/profiles/`，支持 `ethercat_default`、`rs485_left`、`rs485_right`、`canfd_usb`、`zqwl_canfd_left`、`zqwl_canfd_right`。

### 启动 MoveIt 2

```bash
ros2 launch ghand_moveit_config moveit.launch.py
```

默认使用右手模型；启动左手模型时传入：

```bash
ros2 launch ghand_moveit_config moveit.launch.py hand_side:=left
```

### 启动 Gazebo 仿真

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

启动左手 Gazebo 仿真时传入：

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py hand_side:=left
```

启动后可在 RViz 中通过 MotionPlanning 插件规划轨迹，轨迹会发送到 Gazebo 中的 `joint_trajectory_controller` 物理执行。

## 🎮 控制接口

### Topic 接口

- **关节状态**：`/joint_states`（`sensor_msgs/JointState`）
- **关节状态码 / 错误码**：`/joint_status_broadcaster/joint_status`（`hand_msgs/JointStatus`）
- **触觉反馈**：`/tactile_broadcaster/tactile_data`（`hand_msgs/TactileData`）
- **轨迹命令**：`/joint_trajectory_controller/joint_trajectory`（`trajectory_msgs/JointTrajectory`）

### Action 接口

- **轨迹执行**：`/joint_trajectory_controller/follow_joint_trajectory`（`control_msgs/action/FollowJointTrajectory`）

### 控制示例

先启动硬件或仿真：

```bash
# mock 硬件
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true

# 或 Gazebo 仿真
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

#### 张开手掌

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: ["Thumb_MCP", "Thumb_TMC_FE", "Thumb_TMC_AA", "Thumb_TMC_PS",
                  "FF_PIP", "FF_MCP", "FF_MCP_AA",
                  "MF_PIP", "MF_MCP",
                  "RF_PIP", "RF_MCP",
                  "LF_PIP", "LF_MCP"],
    points: [{
      positions: [0.0, 0.0, 0.349, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

#### 握拳

```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: ["Thumb_MCP", "Thumb_TMC_FE", "Thumb_TMC_AA", "Thumb_TMC_PS",
                  "FF_PIP", "FF_MCP", "FF_MCP_AA",
                  "MF_PIP", "MF_MCP",
                  "RF_PIP", "RF_MCP",
                  "LF_PIP", "LF_MCP"],
    points: [{
      positions: [0.5, 0.0, 0.349, 0.5, 1.2, 1.2, 0.0, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2],
      time_from_start: {sec: 2}
    }]
  }'
```

### 监控与调试

```bash
# 列出所有运行的节点
ros2 node list

# 列出所有控制器
ros2 control list_controllers

# 列出所有话题
ros2 topic list

# 监听关节状态
ros2 topic echo /joint_states
```

## 📁 目录结构

```text
ghand-ros2/
├── ghand_hardware/
│   ├── ghand_hardware_interface/   # 硬件接口与通讯驱动
│   ├── hand_controllers/           # 自定义 broadcaster
│   └── hand_msgs/                  # 自定义消息类型
├── ghand_moveit_config/            # MoveIt 2 配置
├── ghand_gazebo/                   # Gazebo 物理仿真
├── ghand_description/              # 机器人描述模型
├── CHANGELOG.md                    # 版本历史
├── LICENSE                         # Apache License 2.0
├── README.md                       # English README
└── README_CN.md                    # 本文件
```

## 🌐 开源与生态资源

- **GLI 开源中心**：[GLI SDK GitHub 组织](https://github.com/gli-sdk)
- **官方文档**：[GHand 灵巧手文档](https://fcnzogxju7xr.feishu.cn/docx/AhZ6ds2iCoguaAxIzBxciYHinNo)
- **C++ SDK**：[GHand-Cpp-SDK](https://github.com/gli-sdk/GHand-Cpp-SDK)
- **Python SDK**：[GHand-Python-SDK](https://github.com/gli-sdk/GHand-Python-SDK)

## 📋 更新日志

详见 [CHANGELOG.md](CHANGELOG.md)。

## 📞 支持与反馈

- 📋 **技术支持：** 如有项目相关问题，请在本仓库提交 `Issue`。
- 📧 **一般咨询：** [support@glitech.com](mailto:support@glitech.com)

## 📄 许可证

本项目基于 [Apache License 2.0](LICENSE) 开源。

Copyright © 2026 GHand ROS2 Maintainers.
