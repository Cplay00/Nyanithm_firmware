# round90r：27 寸 MBR3116 换装后上电循环重启恢复

日期：2026-10-05。对象：HW1，USB 序列号 `50443405C0B5A81C`。用户确认上电立刻循环重启，并配合进入 BOOTSEL。

## 根因与实机证据

换装后的三片 MBR3116（0x40/0x41/0x42）均在线，但保存的控制器配置为 `hwVer=1, cfg0=0x01`，仍选择 MPR121。旧启动检查找不到 0x5A/0x5B/0x5C 后进入 `warn(true)` 无限灯光告警。该循环不喂已经开启的 2 秒看门狗，所以不断复位，无法进入正常扫描和配置入口。

刷前 C1 诊断在 uptime 1218/2058ms 时扫描计数均为 0；USB 约两秒重新枚举。最初观察到 beta1/round89a，随后设备被改为 1.6.5/round88e；恢复版以最后实际固件为基线。

## 固件修复

- 缺失触摸芯片或 ToF 时改为一次有限告警，保留主循环与配置入口；IR 模式跳过 ToF 检查。HW3/4 初始化与检查采用运行时相同的 MBR 选择规则。
- 使用 Pico SDK `watchdog_update()` 在有界启动阶段之间喂狗；运行时看门狗保留。
- ToF 检查 `forceInit()` 返回值，只轮询成功初始化的通道；每次复位后先恢复驱动地址，再按已有原生驱动流程初始化、改址及连续测量。
- 未修改触摸判定、确认、拉伸、去抖、映射和健康 ToF 滤波算法，未更改 128B 协议布局或 MBR NVRAM 表。

SDK 看门狗接口参考：[Raspberry Pi 官方文档](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_watchdog)。VL53L0X 初始化和测量流程参考：[ST UM2039](https://www.st.com/resource/en/user_manual/um2039-world-smallest-timeofflight-ranging-and-gesture-detection-sensor-application-programming-interface-stmicroelectronics.pdf)。

## 实际部署与配置修正

已部署 `dist/round90r_boot_recovery/Nyanithm_27_MBR_1.6.5_bootfix_round90r.uf2`：

- 基线提交 `415e0957f303ed69e11715bae25fd634abe0c229`，身份 `NYANFW1;1.6.5-bootfix;hw_v1;BID=round90r-1.6.5`。
- 175104B / 342 块，SHA256 `28b1a95443e4310cad95266287dce47779bb00c29c65b79259a498b532f52e0f`。
- UF2 范围 `[0x10000000,0x10015600)`，不包含配置所在的末扇区。用户确认 BOOTSEL 后写入唯一 RPI-RP2 盘；刷后按 CDC 序列号和完整固件身份核验目标。
- 保留 HW1、16 灯及其他配置；`cfg0` 从 0x01 改为 0x03，启用 MBR。依据既有 `sanitizeConfig()` 清除 MBR 不支持的逐键释放阈值，实际变更偏移仅 `[3,67,69,127]`（最后一字节为校验）。
- CFG_SET 使用命令与 128B 载荷两次独立写入，不使用超时轻推；CFG_SAVE 成功，保存后与重启后回读均逐字节等于目标配置。目标配置 SHA256 `2ad25a3bfdae106dfa6cd85a96888a4b6d3d523909a7f1252cc494bf1299b1e0`。
- 三片 MBR 表读两次相等且 CRC 有效，重启后与备份逐字节一致；未发 0xBA 或芯片 NVM 写命令。配置模式实际测得 4 路 ToF，读数 75/56/76/20mm。

主维护树 beta14/round90r 和 beta1 恢复产物均编译通过，**未部署**；本机实际继续使用 1.6.5 的触摸管线。

## 备份与局限

备份目录：`_dev_archive/round90r_bootloop_before_20261005_003058`。包含修改前维护树源码、既有 UF2、与最后实际设备身份/构建时间吻合的 1.6.5/round88e 回退 UF2，以及恢复后的原始控制器配置和三片 128B 表。原始配置 SHA256 `32e24386c876012e37ab8353a4b40b14662c89c36beebbcf41452fc42864d57b`。

本机 picotool 构建不含 USB save/load 支持，BOOTSEL 接口驱动也不具备读取能力，未取得完整物理 Flash 镜像。刷写未覆盖配置扇区；能够连接后先备份配置，再修正硬件选择。

## 验证与验收边界

- 启动故障集成测试执行生产 C++ 函数及 2 秒看门狗模型：旧源码 2/13，维护版和两个恢复版各 13/13，158956 个检查。
- 主维护树迁移测试修改前后均 164/164；快照读者 28/28，MPR 基线策略 6/6。beta1 迁移 138/138；1.6.5 采用归档 round89 的迁移脚本 117/117，不将其等同于当前版覆盖范围。
- 三份固件均经官方构建流程编译、UF2 完整性/身份/范围验证；独立只读审查未发现可操作缺陷。构建映射仅改为对应隔离源码目录。
- 刷后修配置前：30.537 秒/38 帧，uptime 61881→91615ms，扫描计数 61414→91664。
- 保存并重启修正配置后：30.572 秒/38 帧，uptime 1115→30883ms，扫描计数 824→28377。两轮只发送 BD/B1/C1，没有通过 BC 代喂看门狗；uptime 与扫描持续递增，未观察到再次复位，串口已关闭。
- 第 7 个逻辑触摸格出现间歇输入；用户确认监测时有触碰或物体，因此不能据此判定误触，也不能当作无人触碰基线。
- 本轮确认启动恢复、配置持久化及原生表保持；游戏 HID 手感、全触摸区和空气键仍待用户实际验收。

可复核数据位于 `dist/round90r_boot_recovery/`，源文件备份与原始配置/表位于上述备份目录。全部提交仅保留在本地，不推送。
