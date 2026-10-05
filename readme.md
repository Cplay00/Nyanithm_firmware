# Nyanithm Firmware

Nyanithm 触摸控制器固件 -- hw_v1 通用版本

当前正式版：**1.6.6**（V1 系列通用固件，`round90y-v1-release`）。USB 产品与 CDC 接口名称为 `Nyanithm Controller V1`。升级说明、硬件配置提示和审修资料保存在仅本地的 `docs/`，不纳入版本控制。

构建与测试工具 `tools/`、维护指引 `AGENTS.md` 以及 `favicon.*` 图标也仅保留在本地，不纳入版本控制。

2026-10-05 起，唯一维护分支为 `hw_v1`，对应远端 `hw_v1_compatible`，主工作树为 `Nyanithm_firmware_hw_v1/`。原主树与通用版合并，保留已部署 beta16 的固件实现、主树最新控制面板及两侧资料。通用版临时分支在验证后移除。构建使用本目录 `tools/build_firmware.ps1 -Variant hw_v1`。合并记录及历史验证资料保留在本地 `docs/`。

## Contributors

| 角色 | 贡献者 | GitHub |
|------|--------|--------|
| 原开发者 | Catium2006 | [https://github.com/Catium2006](https://github.com/Catium2006) |
| 再维护者 | Cplay00 | [https://github.com/Cplay00](https://github.com/Cplay00) |

本固件由 [Catium2006](https://github.com/Catium2006) 原创开发，[Cplay00](https://github.com/Cplay00) 负责 hw_v1 硬件版本的兼容性维护。

完整贡献者列表见 [AUTHORS](AUTHORS)。

# License

本仓库大部分代码使用 `MPL 2.0` 许可发布

部分第三方文件（修改或未修改的）按原版权人要求保留许可内容

# Firmware

此固件使用 `Pico SDK` 直接开发，部分芯片的驱动修改自有关 `Arduino` 库。

若要编译此固件，建议在 `Visual Studio Code` 中安装 [【树莓派Pico插件】](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)，并导入这个项目。点击`Compile`即可编译。

编译出来的文件在 `build` 目录。

主控板子上有一个BOOT按钮，按住按钮再插数据线可以进入下载模式，电脑上会识别一个存储设备插入，把编译得到的`uf2`文件放进去即可自动下载。

# Library

[Adafruit_MPR121](https://github.com/adafruit/Adafruit_MPR121)

[vl53l0x-arduino](https://github.com/pololu/vl53l0x-arduino)

[CypressCY8CMBR3116](https://github.com/sebastianregelmann/CypressCY8CMBR3116)

## 更新日志

详见 [更新日志_固件.md](更新日志_固件.md)。
