# GHand Gazebo 抓取工作台设计

**日期**: 2026-06-24  
**作者**: Claude Code (brainstorming with user)  
**状态**: 待实现  

## 1. 背景与目标

目前 `ghand_gazebo` 启动后只有空地面（`empty.world`），GHand 多指机器人手掌悬空，场景单调，无法验证抓取能力。

本设计目标：在 Gazebo Sim (gz / Harmonic) 中新增一个固定式抓取工作台场景，包含一张桌子和若干基础几何体可抓取物体。由于 GHand 手指关节均绕 z 轴旋转（在水平面内弯曲），需将手掌竖直放置（base_link 绕 y 轴旋转 -90°），使手指弯曲平面变为竖直方向，从而从上方向下包络桌面上的物体，进行包络抓取 / 捏取等演示与测试。

## 2. 需求总结

| 需求项 | 用户选择 | 说明 |
|---|---|---|
| 场景类型 | 交互式抓取工作台 | 有桌子、有物体 |
| 物体生成方式 | 固定 world 文件 | 启动即见，不可交互增减 |
| 物体类型 | 基础几何体 | 方块、圆柱、球 |
| 放置方式 | 直接摆在桌面上 | 手掌从上方接近 |
| 手掌布置 | 手掌竖直、桌面在手掌前方 | base_link 绕 y 轴旋转 -90°，手指向下弯曲 |
| 实现方案 | 方案 B：新增 `grasp_world.world` | 保留 `empty.world`，launch 默认切换 |

## 3. 总体架构

```
ghand_gazebo/
├── launch/
│   └── ghand_gazebo.launch.py      # 默认 world 改为 grasp_world.world
│                                   # spawn_entity z 改为 0.80，pitch 改为 -90°
└── worlds/
    ├── empty.world                 # 保持不变
    └── grasp_world.world           # 新增：桌子 + 3 个可抓取几何体
```

本改动**不修改** URDF、MoveIt 配置、控制器参数，保持现有硬件/仿真流程完整。

## 4. 场景布置

### 4.1 坐标系与尺寸

手掌竖直后，base_link 的局部 z 轴（原向上）转到世界 x 方向（水平向前），局部 x 轴（原向前）转到世界 -z 方向（向下），局部 y 轴保持侧向。手指沿局部 x 轴伸出，现在竖直向下，可伸向桌面上的物体。桌面上各物体位置以 base_link 为中心、在手掌前方（世界 x 正方向）排列。

| 元素 | 尺寸 | 位姿 | 备注 |
|---|---|---|---|
| 桌面 | 1.0 m × 0.6 m × 0.05 m | `(0.25, 0, 0.375)` | 桌面高 0.75 m，位于手掌前方 0.25 m |
| 方块 | 0.03 m 边长 | `(0.25, 0.00, 0.775)` | 质量 0.05 kg，位于桌面中心 |
| 圆柱 | 直径 0.04 m，高 0.08 m | `(0.25, 0.10, 0.79)` | 质量 0.08 kg |
| 球 | 直径 0.04 m | `(0.25, -0.10, 0.77)` | 质量 0.05 kg |
| GHand base_link | — | `(0, 0, 0.80)` | 手掌竖直，高度与桌面齐平 |

> 物体 z 坐标为“桌面高 + 半高”，确保底面与桌面接触。手掌竖直后，手指竖直向下伸出，正好覆盖桌面区域。

### 4.2 视觉与材质

- 桌面：木质颜色（`ambient/diffuse: 0.6 0.4 0.2 1`）。
- 方块：红色。
- 圆柱：蓝色。
- 球：绿色。

## 5. `grasp_world.world` 设计细节

基于现有 `empty.world` 扩展，保留以下系统插件与物理设置：

- `gz-sim-physics-system`
- `gz-sim-scene-broadcaster-system`
- `gz-sim-user-commands-system`
- `gz-sim-sensors-system`
- 重力 `0 0 -9.8`
- 时间步 `max_step_size=0.001`，`real_time_factor=1.0`

新增内容：

### 5.1 静态实验桌

```xml
<model name="grasp_table">
  <static>true</static>
  <pose>0.25 0.0 0.375 0 0 0</pose>  <!-- 桌面位于手掌前方 0.25 m，高 0.75 m -->
  <link name="table_top">
    <collision name="collision">
      <geometry>
        <box><size>1.0 0.6 0.05</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1.0 0.6 0.05</size></box>
      </geometry>
      <material>
        <ambient>0.6 0.4 0.2 1</ambient>
        <diffuse>0.6 0.4 0.2 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
      </material>
    </visual>
  </link>
</model>
```

### 5.2 可抓取方块

```xml
<model name="grasp_cube">
  <pose>0.25 0.00 0.775 0 0 0</pose>
  <link name="link">
    <inertial>
      <mass>0.05</mass>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <collision name="collision">
      <geometry>
        <box><size>0.03 0.03 0.03</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>0.03 0.03 0.03</size></box>
      </geometry>
      <material>
        <ambient>0.9 0.2 0.2 1</ambient>
        <diffuse>0.9 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 5.3 可抓取圆柱与球

结构与方块类似，仅几何形状与颜色不同：

- 圆柱：`<cylinder><radius>0.02</radius><length>0.08</length></cylinder>`，蓝色。
- 球：`<sphere><radius>0.02</radius></sphere>`，绿色。

## 6. 物理参数

### 6.1 物体参数

| 参数 | 值 | 说明 |
|---|---|---|
| 质量 | 0.05 ~ 0.08 kg | 轻质，便于手指夹起 |
| 摩擦 `mu1` / `mu2` | 0.8 | 较高摩擦，降低滑落 |
| 恢复系数 `restitution` | 0.1 | 减少接触弹跳 |
| 接触刚度 `kp` | 100000 | 默认刚性接触 |
| 接触阻尼 `kd` | 10 | 默认阻尼 |

### 6.2 手指参数

机器人手指已在 `ghand.gazebo.xacro` 中配置：

- `mu1 = mu2 = 0.5`
- `kp = 100000`, `kd = 10`
- `minDepth = 0.0001`
- `maxVel = 0.01`

先保持不动。若测试中出现打滑，再统一将手指与物体的 `mu` 提高到 0.8。

## 7. Launch 文件修改

### 7.1 默认 world 切换

在 `ghand_gazebo.launch.py` 中：

```python
DeclareLaunchArgument(
    "world",
    default_value=PathJoinSubstitution(
        [FindPackageShare("ghand_gazebo"), "worlds", "grasp_world.world"]
    ),
    description="Gazebo world file path",
)
```

### 7.2 机器人初始位姿调整

手掌竖直放置：base_link 绕 y 轴旋转 `-90°`（即 roll=0, pitch=-1.5708, yaw=0），使手指弯曲平面竖直，能够从前方向下包络桌面上的物体。

```python
spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-topic", "/robot_description",
        "-entity", "GHand",
        "-x", "0.0",
        "-y", "0.0",
        "-z", "0.80",           # 与桌面同高
        "-R", "0.0",            # roll
        "-P", "-1.57079632679", # pitch：绕 y 轴 -90°
        "-Y", "0.0",            # yaw
    ],
    output="screen",
)
```

### 7.3 兼容性

保留 `world` launch 参数，用户仍可显式运行空场景：

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py world:=empty.world
```

## 8. 测试与验证

### 8.1 场景可见性测试

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

验证：
- Gazebo 中显示桌面；
- 桌面中心附近显示红、蓝、绿三个物体；
- 机器人手掌竖直，位于桌面后方，手指弯曲平面竖直，指尖可伸向桌面上的物体。

### 8.2 MoveIt 规划可达性测试

```bash
# 在另一个终端
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=true
```

验证：
- RViz 中 MotionPlanning 插件加载成功；
- 可规划手指到物体附近；
- 无自碰撞、无与桌面/物体的异常碰撞报错。

### 8.3 抓取尝试测试

通过 MoveIt 或 `ros2 control` 发送关节轨迹，让手指闭合尝试夹起方块。观察：
- 手指是否能稳定接触物体；
- 物体是否被夹起并离开桌面；
- 若滑落，记录并调整摩擦参数。

## 9. 风险与应对

| 风险 | 现象 | 应对 |
|---|---|---|
| 手掌姿态与桌面不对齐 | 手指够不到桌面或从桌面下方穿过 | 调整 spawn 的 pitch 角；确认 base_link 高度与桌面齐平 |
| 手指穿透桌面或物体 | 视觉穿透、物理抖动 | 降低 `kp` 或增加 `minDepth` |
| 物体被弹飞 | 接触后高速飞离 | 降低 `restitution`，增加 `kd` |
| 抓不起来 | 物体从指间滑落 | 提高手指/物体 `mu` 到 0.8；减轻物体质量 |
| 机器人手掌与桌面碰撞 | 启动后手掌抖动或嵌入桌面 | 确认 spawn 高度与桌面齐平；手指不会穿过桌面 |

## 10. 后续扩展（非本设计范围）

- 将桌子、物体拆分为独立 SDF 模型文件；
- 增加 ROS 2 service 动态生成/删除物体；
- 引入 YCB-like 日常物体模型；
- 接入仿真触觉传感器数据，做闭环抓取。

## 11. 结论

本设计通过新增 `grasp_world.world` 并微调 launch 文件（包括将手掌竖直放置），为 GHand 在 Gazebo 中提供一个固定式抓取工作台。方案改动小、不影响现有功能、后续扩展方便。

## 12. 变更清单

- [ ] 新增 `src/ghand_gazebo/worlds/grasp_world.world`
- [ ] 修改 `src/ghand_gazebo/launch/ghand_gazebo.launch.py`
  - [ ] 默认 `world` → `grasp_world.world`
  - [ ] `spawn_entity` z → `0.80`
  - [ ] `spawn_entity` 增加 orientation：pitch = -90°（手掌竖直）
- [ ] 启动测试并截图
- [ ] （可选）根据抓取效果微调摩擦/质量参数
