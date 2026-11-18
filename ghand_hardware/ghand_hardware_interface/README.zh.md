# ghand_hardware_interface

[English](README.md)

面向 GHand C++ SDK 3.1 及更高 3.x 版本的轻量 `ros2_control::SystemInterface` 适配器。设备通信、发现、协议编码、产品能力及设备命令由 SDK 管理；本包不实现 RS485、CANFD 或 EtherCAT 协议。

## 构建

加载 ROS 2 Humble 环境后，在工作空间根目录运行：

```bash
./src/ghand-ros2/build.sh
source install/setup.bash
```

适配器使用已安装的 `ghand_sdk_vendor`，链接 `GHand::ghand`（`libghand.so`），不编译 SDK 源码。关节与主动关节的校验表来自随附的产品 JSON，库和配置路径由 SDK 包提供。仅使用预览或 Gazebo 时可运行 `build.sh --simulation`，无需构建或安装 SDK。

随附产物为 Linux x86_64 的 SDK 3.1.0。升级时整体替换 `ghand_sdk_vendor/sdk/` 中配套的二进制、头文件和配置，并重建 SDK 包及适配器。哈希见 [SDK 清单](../ghand_sdk_vendor/SDK_MANIFEST.json)。

## 运行

```bash
ros2 launch ghand_bringup start.launch.py backend:=hardware \
  product_name:=ghand5 hand:=right planning:=false \
  profile:=rs485_right device:=/dev/ttyUSB0
ros2 run ghand_hardware_interface ghand_joint_controller_gui
```

插件名仍为 `ghand_hardware_interface/GHandHardwareInterface`，保留原有位置轨迹 Action、关节名和反馈话题。支持 `rs485`、`canfd` 和 `ethercat`，`zqwl_canfd` 是 `canfd` 的别名，旧 `zlg_canfd` 实现已移除。

连接参数包括 `product_name`、`device`（串口或 CAN 设备）、`ifname`（EtherCAT 网口）、`node_id`、`baud_rate`、`data_rate` 和 `control_mode`。同一 SDK 会话发现多只手时，用 `hand_side` 或 `slave_index` 选择。双手启动使用独立适配器与会话，不支持不同硬件插件共享一条总线。

RS485 使用 SDK 支持的波特率和 8N1；CANFD 位速率须匹配 SDK 的位时序配置。底层轮询和响应超时由 SDK 管理；`read_timeout_ms` 是 ROS 关节反馈的新鲜度期限，默认 500 ms。

## 接口与生命周期

- ROS 位置使用弧度，SDK 使用角度，按 SDK 关节 ID 映射。
- `velocity_percent` 和 `current_percent` 为设备百分比，不是 rad/s 或 Nm。
- 命令还可声明 `control_mode`、`loop_type`、`control_state`、`kp` 和 `kd`；可用性受设备能力及 Lite1 的 ROS 接口限制。
- 未写入的命令保持 NaN，加载控制器时不发送运动命令。关节与触觉状态来自 SDK 回调，关节反馈时效用单调时钟计算，无数据不视为新测量。

配置时连接 SDK；激活要求完整、有效且未过期的反馈，并清除旧命令；停用调用 `Stop()` 但保留会话；清理时断开连接；出错或关闭时停止并释放连接。SDK 错误码和消息会传递给 ROS。`read()` 复制回调缓存，`write()` 调用 `MoveJoints()`，写入延迟取决于 SDK 和所选传输方式。

已检查 SDK 链接及适配器构建。设备运动、控制器执行及仿真行为需在目标环境中验证；本包不维护设备协议测试。
