# EtherCAT 默认接口名去硬编码设计

## 背景

`src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml` 中写死了一个特定机器生成的网卡接口名：

```yaml
ifname: enxe466e5820449
```

这类名称（基于 MAC 地址）在不同电脑上会变化，提交到仓库会导致其他用户启动 EtherCAT 时找不到接口。

## 目标

- 从仓库中移除机器相关的 EtherCAT 接口名。
- 让用户通过现有的 launch arg 显式指定接口名。
- 更新文档，给出可复制的启动示例。

## 方案选择

采用 **方案 A：launch arg 覆盖 + YAML 占位符 + 文档**。

`ghand.launch.py` 已经支持 `ifname` 启动参数，并通过 xacro 传入 `ghand_hardware_description.urdf.xacro`。xacro 中的逻辑是：若 launch arg 非空则使用 launch arg，否则回退到 profile YAML 中的 `ifname`。因此只需要把 YAML 中的默认值清空，并补充文档即可。

## 改动内容

### 1. `src/ghand_hardware/ghand_hardware_interface/config/profiles/ethercat_default.yaml`

```yaml
hardware:
  communication_type: ethercat
  ifname: ""
  read_timeout_ms: 500
  tactile_region_counts: "38,31,31,31,31"
```

空字符串会让 xacro 跳过写入 `ifname` 参数，从而强制用户通过 launch arg 提供。

### 2. 文档更新

在 `src/ghand_hardware/ghand_hardware_interface/README.md`、`README.md`、`README_CN.md` 的 EtherCAT 启动示例中，把命令改成：

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=eth0
```

并补充查看本机网卡名的提示：

```bash
ip link show
```

## 行为说明

- 用户未提供 `ifname` 时，xacro 不会生成 `ifname` 参数，硬件层会因缺少接口名而失败，提示用户配置。
- 用户通过 `ifname:=<接口名>` 启动时，该值会正确覆盖 YAML 中的空值。
- 其他通信 profile（rs485、canfd、zqwl_canfd）不涉及 `ifname`，不受影响。

## 验收标准

- [ ] `ethercat_default.yaml` 中不再出现 `enxe466e5820449`。
- [ ] README 中包含带 `ifname` 的 EtherCAT 启动示例。
- [ ] `ifname:=<接口名>` 启动时，生成的 URDF / ros2_control 参数中 `ifname` 为传入值。
- [ ] 不带 `ifname` 启动时，生成的参数中不包含 `ifname`。
