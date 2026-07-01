# EtherCAT 默认接口名去硬编码实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 移除 `ethercat_default.yaml` 中写死的 EtherCAT 网卡接口名，让用户通过已有的 `ifname` launch arg 显式指定，并更新相关文档。

**Architecture:** 保持现有 xacro / launch 覆盖逻辑不变：launch arg 非空时覆盖 YAML，否则回退 YAML。把 YAML 默认值清空，并补充启动示例文档。

**Tech Stack:** ROS 2 Jazzy, xacro, Python launch, YAML

## Global Constraints

- 不新增环境变量或其他配置来源，仅使用现有 `ifname` launch arg。
- 不改动 xacro 的覆盖逻辑。
- 保持其他 profile（rs485、canfd 等）不变。
- 仅修改 `ethercat_default.yaml` 与文档文件。

## File Structure

| 文件 | 说明 |
| --- | --- |
| `src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml` | 移除硬编码 `ifname` |
| `src/ghand_hardware/ghand_hardware_interface/README.md` | 新增 Launch 小节，含 EtherCAT 示例 |
| `README.md` | 更新 Quick Start 中真实硬件示例 |
| `README_CN.md` | 更新中文 Quick Start 中真实硬件示例 |

---

### Task 1: 清空 EtherCAT profile 中的默认接口名

**Files:**
- Modify: `src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml`

**Interfaces:**
- Consumes: 无
- Produces: `ifname` 字段变为空字符串，xacro 会跳过写入该参数

- [ ] **Step 1: 修改 YAML**

将文件内容改为：

```yaml
hardware:
  communication_type: ethercat
  ifname: ""
  read_timeout_ms: 500
  tactile_region_counts: "38,31,31,31,31"
```

- [ ] **Step 2: 验证修改**

运行：

```bash
grep -n "ifname" src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml
```

Expected: 显示 `ifname: ""`，不再包含 `enxe466e5820449`。

- [ ] **Step 3: Commit**

```bash
git add src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml
git commit -m "fix(ethercat_default): remove hardcoded EtherCAT interface name"
```

---

### Task 2: 更新包级 README 的启动示例

**Files:**
- Modify: `src/ghand_hardware/ghand_hardware_interface/README.md`

**Interfaces:**
- Consumes: 无
- Produces: 包级 README 新增 Launch 小节

- [ ] **Step 1: 在文件末尾追加 Launch 小节**

追加内容：

```markdown
## Launch

```bash
# Mock hardware (no physical hand required)
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true

# EtherCAT (replace enx... with your EtherCAT interface name, see `ip link`)
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

- [ ] **Step 2: 验证追加内容**

运行：

```bash
grep -n "ifname" src/ghand_hardware/ghand_hardware_interface/README.md
```

Expected: 至少出现一次 `ifname:=`。

- [ ] **Step 3: Commit**

```bash
git add src/ghand_hardware/ghand_hardware_interface/README.md
git commit -m "docs(ghand_hardware_interface): add launch examples with ifname override"
```

---

### Task 3: 更新根 README 的 EtherCAT 启动示例

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: 无
- Produces: 根 README 真实硬件启动命令包含 `ifname`

- [ ] **Step 1: 替换 Quick Start 中真实硬件命令**

找到 `### Run with Real Hardware` 小节，把：

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default
```

替换为：

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

并在该代码块后添加说明：

```markdown
> Replace `enx000000000000` with your EtherCAT network interface name. Run `ip link` to list available interfaces.
```

- [ ] **Step 2: 验证修改**

运行：

```bash
grep -n "ifname" README.md
```

Expected: 至少出现一次 `ifname:=`。

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs(readme): show ifname override for EtherCAT launch"
```

---

### Task 4: 更新中文根 README 的 EtherCAT 启动示例

**Files:**
- Modify: `README_CN.md`

**Interfaces:**
- Consumes: 无
- Produces: 中文 README 真实硬件启动命令包含 `ifname`

- [ ] **Step 1: 替换中文 Quick Start 中真实硬件命令**

找到 `### 启动真实硬件` 小节，把：

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default
```

替换为：

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

并在该代码块后添加说明：

```markdown
> 请将 `enx000000000000` 替换为你本机的 EtherCAT 网卡接口名。可通过 `ip link` 命令查看可用接口。
```

- [ ] **Step 2: 验证修改**

运行：

```bash
grep -n "ifname" README_CN.md
```

Expected: 至少出现一次 `ifname:=`。

- [ ] **Step 3: Commit**

```bash
git add README_CN.md
git commit -m "docs(readme_cn): show ifname override for EtherCAT launch"
```

---

### Task 5: 验证 xacro 覆盖行为

**Files:**
- 无新增或修改

**Interfaces:**
- Consumes: `ghand_hardware_description.urdf.xacro` 与 `ethercat_default.yaml`
- Produces: 确认 `ifname` launch arg 正确覆盖 YAML 空值

- [ ] **Step 1: 构建并 source 工作空间**

```bash
cd ~/GHand-Ros2
colcon build --symlink-install
source install/setup.bash
```

- [ ] **Step 2: 测试带 ifname 覆盖**

运行：

```bash
xacro src/ghand_hardware/ghand_hardware_interface/config/ghand_hardware_description.urdf.xacro \
  use_mock_hardware:=false \
  profile_file:=$(pwd)/src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml \
  ifname:=eth0 | grep 'name="ifname"'
```

Expected: 输出包含 `<param name="ifname">eth0</param>`。

- [ ] **Step 3: 测试不带 ifname 覆盖**

运行：

```bash
xacro src/ghand_hardware/ghand_hardware_interface/config/ghand_hardware_description.urdf.xacro \
  use_mock_hardware:=false \
  profile_file:=$(pwd)/src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml \
  ifname:= | grep 'name="ifname"'
```

Expected: 无输出（grep 返回空，说明 xacro 未写入 `ifname` 参数）。

- [ ] **Step 4: Commit**

无文件变更，无需 commit。若验证失败，回到 Task 1 检查 YAML 是否清空。

---

## Self-Review Checklist

- [x] Spec coverage: `ethercat_default.yaml` 清空、文档更新、xacro 验证均已覆盖。
- [x] Placeholder scan: 无 TBD/TODO/"implement later"；示例接口名 `enx000000000000` 是显式占位符并配了说明。
- [x] Type consistency: 仅涉及 YAML 字符串与文档文本，无接口/签名变化。
