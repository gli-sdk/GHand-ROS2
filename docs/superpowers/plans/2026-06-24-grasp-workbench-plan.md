# GHand Gazebo 抓取工作台实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Gazebo Sim 中为 GHand 添加一个固定式抓取工作台场景（`grasp_world.world`），包含桌子和基础几何体可抓取物体，并调整 launch 文件使手掌竖直、手指向下包络桌面物体。

**Architecture:** 新增 `ghand_gazebo/worlds/grasp_world.world`（基于现有 `empty.world` 扩展），在 `ghand_gazebo.launch.py` 中把默认 world 切换为该文件，并调整 `spawn_entity` 的高度（z=0.80）和姿态（pitch=-90°）。不改动 URDF、MoveIt、控制器配置。

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim (gz) Harmonic, Python launch, SDF 1.6.

## Global Constraints

- 不修改 `src/ghand_description/urdf/G5-A7-R.urdf`。
- 不修改 `src/ghand_moveit_config/` 下的 SRDF / YAML / launch。
- 不修改 `src/hand_bringup/config/controllers.yaml` 或 `src/hand_bringup/config/ros2_control.yaml`。
- 保留 `empty.world` 作为可选空场景。
- 所有新增文件使用 UTF-8 编码。
- 提交信息遵循 `feat(scope): description` 格式。

---

## File Structure

| 文件 | 操作 | 职责 |
|---|---|---|
| `src/ghand_gazebo/worlds/grasp_world.world` | 创建 | 包含静态桌面和三个可抓取几何体的 SDF world |
| `src/ghand_gazebo/launch/ghand_gazebo.launch.py` | 修改 | 默认 world 切换为 `grasp_world.world`；`spawn_entity` 高度改为 0.80，姿态 pitch=-90° |

---

## Task 1: 创建 `grasp_world.world`

**Files:**
- Create: `src/ghand_gazebo/worlds/grasp_world.world`
- Reference: `src/ghand_gazebo/worlds/empty.world`
- Test: 手动启动 `ros2 launch ghand_gazebo ghand_gazebo.launch.py world:=<path>/grasp_world.world use_rviz:=false` 并观察场景

**Interfaces:**
- Consumes: 现有 `empty.world` 的物理/光照/地面配置。
- Produces: 一个可被 `gz sim -r` 加载的 SDF world 文件，包含 `grasp_table`、`grasp_cube`、`grasp_cylinder`、`grasp_sphere` 四个模型。

- [ ] **Step 1: 复制 `empty.world` 作为基础**

```bash
cp src/ghand_gazebo/worlds/empty.world src/ghand_gazebo/worlds/grasp_world.world
```

- [ ] **Step 2: 在 `grasp_world.world` 的 `</world>` 前添加静态桌面**

在 `src/ghand_gazebo/worlds/grasp_world.world` 中，在 `</world>` 标签之前插入：

```xml
  <model name="grasp_table">
    <static>true</static>
    <pose>0.25 0.0 0.375 0 0 0</pose>
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

- [ ] **Step 3: 添加可抓取方块**

在 `grasp_table` 模型之后、`</world>` 之前插入：

```xml
  <model name="grasp_cube">
    <pose>0.25 0.0 0.775 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.03 0.03 0.03</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
          </bounce>
          <contact>
            <ode>
              <kp>100000</kp>
              <kd>10</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.03 0.03 0.03</size></box>
        </geometry>
        <material>
          <ambient>0.9 0.2 0.2 1</ambient>
          <diffuse>0.9 0.2 0.2 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
```

- [ ] **Step 4: 添加可抓取圆柱**

在 `grasp_cube` 模型之后、`</world>` 之前插入：

```xml
  <model name="grasp_cylinder">
    <pose>0.25 0.10 0.79 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.08</mass>
        <inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00001"/>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.02</radius><length>0.08</length></cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
          </bounce>
          <contact>
            <ode>
              <kp>100000</kp>
              <kd>10</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.02</radius><length>0.08</length></cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.9 1</ambient>
          <diffuse>0.2 0.2 0.9 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
```

- [ ] **Step 5: 添加可抓取球**

在 `grasp_cylinder` 模型之后、`</world>` 之前插入：

```xml
  <model name="grasp_sphere">
    <pose>0.25 -0.10 0.77 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>0.02</radius></sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
          </bounce>
          <contact>
            <ode>
              <kp>100000</kp>
              <kd>10</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>0.02</radius></sphere>
        </geometry>
        <material>
          <ambient>0.2 0.9 0.2 1</ambient>
          <diffuse>0.2 0.9 0.2 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
```

- [ ] **Step 6: 验证 world 文件语法**

```bash
cd /home/pq/GHand-Ros2
# 检查 SDF 是否能被 gz 解析（headless 模式，启动后按 Ctrl+C 退出）
gz sim -r -s src/ghand_gazebo/worlds/grasp_world.world
```

预期：命令启动 headless Gazebo，加载 world 后打印 `[Msg] Serving scene information on ...` 等日志，无 fatal XML 解析错误。按 `Ctrl+C` 退出。如果有错误，根据错误信息修正 XML。

- [ ] **Step 7: 提交**

```bash
git add src/ghand_gazebo/worlds/grasp_world.world
git commit -m "feat(gazebo): add grasp_world with table and primitives"
```

---

## Task 2: 修改 `ghand_gazebo.launch.py`

**Files:**
- Modify: `src/ghand_gazebo/launch/ghand_gazebo.launch.py`
- Test: 启动 launch 文件并观察机器人位姿

**Interfaces:**
- Consumes: `grasp_world.world`（Task 1 产出）。
- Produces: 默认加载 `grasp_world.world` 的 launch 文件；机器人以 `(0, 0, 0.80)` 位置、pitch=-90° 姿态生成。

- [ ] **Step 1: 修改默认 world 参数**

在 `src/ghand_gazebo/launch/ghand_gazebo.launch.py` 中，找到 `DeclareLaunchArgument("world", ...)` 并将其 `default_value` 改为：

```python
DeclareLaunchArgument(
    "world",
    default_value=PathJoinSubstitution(
        [FindPackageShare("ghand_gazebo"), "worlds", "grasp_world.world"]
    ),
    description="Gazebo world file path",
)
```

- [ ] **Step 2: 修改机器人生成位姿**

找到 `spawn_entity = Node(...)`，将其 `arguments` 改为：

```python
spawn_entity = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-topic", "/robot_description",
        "-entity", "GHand",
        "-x", "0.0",
        "-y", "0.0",
        "-z", "0.80",
        "-R", "0.0",
        "-P", "-1.57079632679",
        "-Y", "0.0",
    ],
    output="screen",
)
```

- [ ] **Step 3: 验证 launch 文件 Python 语法**

```bash
cd /home/pq/GHand-Ros2
python3 -m py_compile src/ghand_gazebo/launch/ghand_gazebo.launch.py
```

预期：无输出表示语法正确。

- [ ] **Step 4: 提交**

```bash
git add src/ghand_gazebo/launch/ghand_gazebo.launch.py
git commit -m "feat(gazebo): switch default world to grasp_world and orient hand vertically"
```

---

## Task 3: 集成启动测试

**Files:**
- Use: `src/ghand_gazebo/launch/ghand_gazebo.launch.py`
- Use: `src/ghand_moveit_config/launch/moveit.launch.py`
- Test: 手动运行并观察 Gazebo / RViz

**Interfaces:**
- Consumes: Task 1 和 Task 2 的结果。
- Produces: 可运行的抓取演示场景；验证清单记录。

- [ ] **Step 1: 构建工作区**

```bash
cd /home/pq/GHand-Ros2
colcon build --symlink-install
source install/setup.bash
```

- [ ] **Step 2: 启动 Gazebo 场景**

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

验证清单：
- [ ] Gazebo GUI 正常打开，无段错误。
- [ ] 场景中有一张木质颜色桌面。
- [ ] 桌面上有红色方块、蓝色圆柱、绿色球。
- [ ] 机器人手掌竖直，位于桌面后方（x≈0），高度与桌面齐平。
- [ ] 手指竖直向下，指尖能覆盖桌面区域。

- [ ] **Step 3: 启动 MoveIt（可选，验证规划可达性）**

在另一个终端：

```bash
cd /home/pq/GHand-Ros2
source install/setup.bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=true
```

验证清单：
- [ ] RViz 中 MotionPlanning 插件加载成功。
- [ ] 机器人模型与 Gazebo 中一致（竖直姿态）。
- [ ] 可规划手指到方块/圆柱/球附近。
- [ ] 无自碰撞、无与桌面的异常碰撞报错。

- [ ] **Step 4: 记录测试结果并截图**

如果一切正常，保存 Gazebo 和 RViz 截图到 `docs/superpowers/specs/assets/2026-06-24-grasp-workbench/`（目录不存在则创建）。

- [ ] **Step 5: 提交验证记录（可选）**

```bash
git add docs/superpowers/specs/assets/2026-06-24-grasp-workbench/
git commit -m "docs: add grasp workbench verification screenshots"
```

---

## Task 4: 物理参数微调（可选，根据测试结果）

**Files:**
- Modify: `src/ghand_gazebo/worlds/grasp_world.world`
- Modify（可能）: `src/ghand_description/urdf/ghand.gazebo.xacro`
- Test: 手动抓取尝试

**Interfaces:**
- Consumes: Task 3 的观察结果。
- Produces: 稳定的抓取接触参数。

- [ ] **Step 1: 如果物体打滑，提高摩擦**

将 `grasp_world.world` 中所有物体的 `<mu>0.8</mu>` 和 `<mu2>0.8</mu2>` 改为 `1.2` 或更高，或同时提高手指摩擦：

在 `src/ghand_description/urdf/ghand.gazebo.xacro` 中：

```xml
<xacro:macro name="ghand_gazebo_link_properties"
             params="link_name
                     mu1:=0.8
                     mu2:=0.8
                     ...">
```

- [ ] **Step 2: 如果物体弹飞，降低恢复系数或增加阻尼**

将 `restitution_coefficient` 从 `0.1` 改为 `0.0`，或将 `<kd>` 从 `10` 改为 `50`。

- [ ] **Step 3: 如果手指穿透物体，调整接触刚度/深度**

将 `<kp>` 从 `100000` 改为 `50000`，或将 `<minDepth>` 加入物体碰撞 surface（参考 `ghand.gazebo.xacro` 的 `<minDepth>${min_depth}</minDepth>`）。

- [ ] **Step 4: 重新测试并提交**

```bash
git add src/ghand_gazebo/worlds/grasp_world.world
git add src/ghand_description/urdf/ghand.gazebo.xacro  # 如果修改了
git commit -m "tune(gazebo): adjust contact/friction for stable grasping"
```

---

## Self-Review

### 1. Spec coverage

| Spec 要求 | 对应任务 |
|---|---|
| 新增 `grasp_world.world`，包含桌子 + 3 个几何体 | Task 1 |
| 默认 world 切换为 `grasp_world.world` | Task 2 Step 1 |
| `spawn_entity` z=0.80, pitch=-90° | Task 2 Step 2 |
| 保留 `empty.world` 兼容性 | Task 2 Step 1 中仍可通过 `world:=empty.world` 覆盖 |
| 物理参数（摩擦 0.8、恢复 0.1 等） | Task 1 Step 3-5 |
| 启动测试 | Task 3 |
| 微调摩擦/质量 | Task 4 |

无遗漏。

### 2. Placeholder scan

- 无 "TBD" / "TODO"。
- 无 "appropriate error handling" 等模糊描述。
- 所有代码块包含完整内容。
- 所有命令包含预期输出或验证清单。

### 3. Type consistency

- `grasp_world.world` 中模型名称与 launch 文件中无直接依赖，命名一致。
- launch 文件中的 `-P` 参数使用浮点数字符串，与 `ros_gz_sim create` 接口一致。

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-24-grasp-workbench-plan.md`. Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

Which approach?
