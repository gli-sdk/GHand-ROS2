# ghand_sdk_vendor

[English](README.md) · [SDK 中文文档](sdk/README.zh.md)

此包安装 SDK 生成的 Linux x86_64 发行产物，包括 `libghand.so`、公共头文件、CMake 导出、产品 JSON 和许可证声明。ROS 构建和运行均无需检出 SDK 源码。

硬件适配器只链接此包安装的 SDK 副本。`SDK_MANIFEST.json` 记录版本、源码修订和产物哈希。原始许可证和第三方声明位于 `sdk/`。

升级时，为目标架构生成完整的 SDK 安装产物，整体替换 `sdk/` 并保留库的符号链接；更新清单后重新构建此包和硬件适配器。不要混用不同 SDK 版本的头文件。
