## 安装与编译

真实硬件使用独立的 GHand C++ SDK（3.1 及兼容的 3.x 版本）。协议及传输依赖由 SDK 管理，ROS 不再单独编译 SOEM、libmodbus 或 CANFD 驱动。纯预览与 Gazebo 不需要 SDK。

### 1. 环境准备

```bash
# 本分支要求 Ubuntu 24.04 + ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 检查 ROS 2 环境
echo $ROS_DISTRO  # 应该输出: jazzy

# 创建工作空间
mkdir -p <workspace>/src
cd <workspace>
```

**注意**：请将 `<workspace>` 替换为您的工作空间路径，例如 `~/ghand-ros2`。

### 2. 克隆仓库与模型资产

模型资产由独立仓库提供。默认使用 Gitee，依次克隆源码的 Jazzy 分支和模型资产：

```bash
cd <workspace>/src

# 克隆 Jazzy 分支
git clone -b jazzy https://gitee.com/glitech/ghand-ros2.git
# 克隆模型资产
git clone --depth 1 https://gitee.com/glitech/ghand-assets.git ghand_assets

# GitHub 备用：替换上面的对应命令，不要重复克隆
# git clone -b jazzy https://github.com/gli-sdk/ghand-ros2.git ghand-ros2
# git clone --depth 1 https://github.com/gli-sdk/GHand-Assets.git ghand_assets
```

### 3. 安装 ROS 依赖

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
# 仅首次使用 rosdep 时需要执行；若提示已经初始化，可直接继续
sudo rosdep init
rosdep update
rosdep install --from-paths src/ghand-ros2 \
  --ignore-src -r -y
```

rosdep 会根据各包的 `package.xml` 安装 ros2_control、MoveIt、Gazebo 和
RS485 所需的 libmodbus 等依赖。

### 4. 构建

项目已携带 Linux x86_64 的 SDK 动态库、配套头文件和产品配置，位于
`ghand_hardware/ghand_sdk_vendor/sdk/`。无需克隆、编译或指定本地 SDK 源码目录。

在工作空间根目录执行：

```bash
./src/ghand-ros2/build.sh
source install/setup.bash

# 只构建 MoveIt 预览与 Gazebo，不构建硬件 SDK 包
# ./src/ghand-ros2/build.sh --simulation
```

ROS 硬件插件链接工作空间安装的 SDK 动态库；传输依赖由随附 SDK 决定，
无需 `--ethercat` / `--canfd` 构建开关。SDK 更新时整套替换库、头文件及 JSON，再重新构建。
其他架构需要对应架构的 SDK 产物。

每次打开新终端后加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source <workspace>/install/setup.bash
```

迁移建议使用新工作空间，避免旧安装残留。
