# hw_v1 通用版 beta16 审查与安装记录

2026-10-05；身份 `NYANFW1;1.6.6-beta16;hw_v1;BID=round90v-universal`，构建时间 `Oct 5 2026 06:19:29`，Pico SDK 2.2.0。

## 维护范围与交付

本轮从已部署的 beta15 通用版提交 `514a2b9c90dc78dd7e9accf09190fa2350273cab` 开始，将启动恢复和本轮审修统一在唯一维护分支 `hw_v1`。维护工作树已移至工作区根的 `Nyanithm_firmware_hw_v1_universal/`。依据用户最后指示，`hw_v1_compatible` 及其工作树保持存档，本轮不合入未经验收的 beta14 触摸实验，不修改旧树的固件或控制面板。所有更改仅本地提交，不推送。

最终 UF2：`dist/round90v_universal/Nyanithm_27_32_universal_beta16.uf2`，203264 字节、397 块；SHA-256 `8a350d99b8b6c0a58680e929ce333b11ecba4d2cfe864a371919a56f61309dd2`。验证 RP2040 family ID、所有块编号、载荷、身份及目标范围 `[0x10000000,0x10018d00)`，不触及末尾配置扇区 `0x101ff000`。

## 已修复的问题

| 范围 | 原问题与实际修复 |
|---|---|
| Flash 持久化 | 128B 编程不符合 SDK 的 256B 页要求，损坏页/页洞可能使选择停在旧页或覆盖已编程数据。改为 FF 填充的完整 256B 页，扫描全部 16 页，读取最后一个合法配置，写入最后一个占用页之后；满扇区才擦除。128B 协议、配置版本和旧合法页均兼容。 |
| 双核 USB | Core0 请求 USB 后没有确认 Core1 已暂停，启动泵和 Core1 可能重叠。使用原生 SDK semaphore 确认暂停，启动时 Core0 先持有 USB；等待总期限 2 秒，失败明确 watchdog 复位。Core1 的正常探测不再代喂 Core0 看门狗。 |
| CDC 帧边界 | B2 等待 96B RGB 载荷会堵住 HID；断流尾字节可能变成配置/刷机命令。改为非阻塞累计，500ms 超时后仍消耗剩余帧；DTR 下降清原生 RX 并隔离直到重开，实际拔线清状态，兼容未设置 DTR 的旧主机完整事务。为旧固定回复预留完整 TX FIFO。 |
| CDC 超时 | 配置/调试二进制输出存在无限等待，stdio 仅有空闲期限。二进制回复总期限 250ms，stdio 总期限 500ms，保持部分二进制回复不附带文本错误；B8 在连续输出组/身份读取之间喂狗，避免慢读累积超过 2 秒。 |
| I2C 故障准入 | MPR 短读和失败复用缓冲，持续失败可能创建新触摸；若干 16/32 位驱动读取会使用未初始化值。要求精确字节数，失败清缓冲/拒绝新 ON。MBR 保留原生 SYNC 读取及有限 wake NACK 重试。TCA9539 使用同一原生读写事务；VL 多字节写合成寄存器前缀加数据的一次事务，32 位移位显式无符号。 |
| ToF 空气键 | 8190/8191 或测量停止后继续使用旧高度可能保持空气键。仅有效测距参与滤波和区间判定；200ms 过期，无效时清滤波，脉冲也按时结束，恢复有效读数后可重新响应。 |
| LED | 释放数组方式、边界/区间溢出、未初始化像素和 16 灯尾部问题已修复。启动期间 LampArray 不能覆盖提示，退出启动后恢复原有所有权。PIO 时序不变。 |
| 无用代码 | 删除无调用者的旧无限告警/rainbow 模块、旧未检查 MBR 烧录助手、空初始化、未调用 Kalman 方法；删除 MPR Run 状态无效的手写基线修正及随后会被 CL=11 覆盖的基线种子采样/写入；删除被原生预算计算拒绝的 12ms ToF 请求。保留健康触摸判据及原生 MPR CONFIG1=0x10、CONFIG2=0x28，按已有配置设置去抖。 |

## 上电提示

仅上电检查一次。触摸型号/地址布局与配置不符，或配置所需的四/五路 ToF 数量、初始化结果不符时，全灯珠红色亮 2 秒，灭 0.5 秒，重复三次。两次间隔、总计约 7 秒，随后清尾灯并恢复配置灯数的灰色待机。提示期间持续喂狗并服务 USB；不更改配置、不写 MBR NVRAM。`lightLimit=0` 也能看到警告。

MPR 通过其已知地址识别（该芯片无专用 WHOAMI）；MBR 使用原厂 FAMILY=0x9A、DEVICE=0x0A05。原生驱动三次立即 NACK 重试之外，仅启动发现失败再进行两次间隔 10ms 的确认，每片最多三次发现；成功但身份不符立即停止。首次真机发现第三片 MBR 启动 mask 偶发为 03、随后三片身份均合法，据此修正，正确配置复刷后为 07、不再告警。B8 提供启动 touch/ToF mask 及当前原生身份以便区分发现失败和型号错误。

IR 为自动识别，并非持久化配置选项；WS2812 是单向链路，无法读取真实灯数。本轮不宣称能自动核对物理屏幕尺寸或灯珠数量。ToF 出现超范围测距本身不会当作硬件配置不符；B8 原有测距 warning 也不证明通信故障。

## 验证

| 验证 | 结果 |
|---|---|
| 迁移模型 `test_round50_migration.py` | 基线 153/153；最终 162/162，含 MPR 持续失败/恢复 |
| 生产触摸函数 `test_mbr_distance_pipeline.py` | 87/87，118629 检查 |
| 生产启动/提示/空气键 `test_boot_recovery.py` | 27/27，359056 检查；含 7 秒提示、wake NACK、持续缺失、8190/8191/200ms 过期 |
| 生产 Flash/CDC/驱动/LED `test_firmware_audit.py` | 17/17，2226 检查；含坏页/满扇区、载荷边界、DTR、USB 交接失败 |
| 生产 B8 慢读 `test_status_watchdog.py` | 8/8；每次输出模拟 500ms、每次原生 getter 120ms，最大喂狗间隔不超过 740ms |
| 原生寄存器回复 `test_native_register_dump.py` | 28/28；二进制长度、字节序、短读、USB 部分写/断连 |
| 快照读者 / MBR SYNC 驱动 | 分别 28/28、27/27 |
| Pico SDK Release 构建、UF2 检查 | 通过；最终 text=105492B、data=0、bss=10908B |
| 独立只读审查 | review-agent 工作流及多个只读子审查；新增真机发现再次复核，B8 累积喂狗缺陷已修复后复核通过 |

生产函数测试提取实际 C++ 并记录哈希，外设/时钟使用模拟；它们不能替代物理时序或游戏体验。面板 HTML 和共享协议结构未修改，本轮不需要面板 Mock 回归。测试、原始设备记录均留在工作区 `_dev_tools/round90v_*` 与 `_dev_archive/round90v_before_20261005/`。

## 真机安装与保存回归

用户授权两台保持 USB 连接、自动安装/重启/回读。本轮通过 VID/PID 和 USB serial 精确选板；一次只转换一台 BOOTSEL，确认目标 CDC 消失及唯一新增 RPI-RP2 后复制，不操作其他串口。最终两台写入同一个 SHA-256 的 UF2并返回相同构建时间。

| 设备 | 当前配置与回读 |
|---|---|
| 32 寸 `5303284739002C9C` | HW2，MBR 40/41/42，五路 ToF；最终启动 MPR=00/MBR=07、ToF physical/ready=1F，无配置告警。控制器 SHA-256 `a83b06fa09dcdde59f56f918b22ef165c30eb165a0f45f949d4b37a1665fbf39` 与本轮最新备份逐字节相等；三片 128B MBR 表逐字节保持。 |
| 27 寸 `50443405C0B5A81C` | HW1，MPR 5A/5B/5C，四路 ToF、16 灯；启动 MPR=07/MBR=00、ToF physical/ready=0F，无配置告警。控制器 SHA-256 `af26d4f789968d5f725599206c0caa3bbe82724b1903339ce4859f7b1dec5760` 相等；阈值、滤波、去抖、ECR 等固定控制相等。原生每次启动的自动配置会重新选择 CDC/CDT，少量电极充电电流变化已单列，并未强写回旧的自动校准结果。 |

32 寸临时清除 MBR 选择位，CFG_SET 采用命令/128B 两次独立写，响应只读、不插入 BC；保存、普通重启后确认持久化的错误选择、启动 mismatch=1、告警结束后能读取输入，再保存/重启恢复原 128B。测试覆盖页 15 写入及满扇区恢复到页 0；最终版本再次通过完整误配置/恢复测试，最终页 4。命令日志 CE 已保存。早期主机脚本把 B3 当成必须有文本 ACK、以及 Windows 枚举早于 COM 句柄就绪/7秒提示结束的问题已修正；失败尝试保留，临时错误配置均已恢复。

两台最终分别采集 75 组 BD/B1/C1，约 60.3 秒；uptime 与扫描次数递增，无复位/扫描停滞，未用 BC 代喂狗。另在两台实际发送超时后的 B2 迟到载荷（含 B3/BB/A5 字节）及 DTR 下降后的残留载荷，均未被当作命令，重开后 33B 输入/528B 遥测完整、扫描继续；随后普通重启成功。Windows 两台 HID 接口正常枚举。

短窗口性能比较：32 寸平均扫描 1395.72→1411.25μs（约 +1.1%），27 寸 1156.55→1134.00μs（约 -2.0%）；外界条件未固定，不能据此宣称性能提升或证明最坏延迟。与 beta15 构建相比 text 增加 760B、bss 相等；UF2 增加 1536B，新增可靠性机制没有降低二进制体积。生产源码净减少量见交付 manifest。

## 备份与边界

修改前源码和 beta15 回滚 UF2 位于 `_dev_archive/round90u_audit_before_20261005/`，最新设备配置/原生表和逐阶段报告位于 `_dev_archive/round90v_before_20261005/`。未取得完整物理 Flash 镜像；源码、可安装回滚 UF2及配置/原生表可用于恢复。

单配置扇区在整扇区擦除期间断电仍有固有风险，不能宣称完全断电安全。没有进行真实断电破坏测试。LED 电气颜色与实际闪烁节奏未经现场目视/示波器确认，已验证实际代码的三次 2 秒/0.5 秒状态及真机 mismatch 路径。HID 游戏手感、真实触摸/悬空与空气键响应仍需用户回台验收；60 秒 CDC 与 HID 枚举不等于游戏验收。MBR 悬空问题并未在本轮宣称根治。

## 原厂依据

- [Pico SDK Hardware APIs](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html)：Flash 256B 页对齐/长度，watchdog、I2C、PIO 原生 API；暂停确认使用 SDK semaphore。
- [NXP MPR121 Datasheet](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf)、[AN3891](https://www.nxp.com/docs/en/application-note/AN3891.pdf)：Stop/Run 寄存器限制、CL 初始装载、基线滤波语义。
- Infineon CY8CMBR3xxx Registers TRM（001-91082 Rev. E）§1.5.85/86，以及 Datasheet 001-85330 Rev. Q 第32页和 Design Guide 第69页；已读原厂本地资料 `_dev_tools/official_mbr_sources/`，用于 FAMILY/DEVICE、启动和 wake NACK。未生成或写入新的 128B 表。
- [ST VL53L0X Datasheet](https://www.st.com/resource/datasheet/vl53l0x.pdf)、[TI TCA9539 Datasheet](https://www.ti.com/lit/ds/symlink/tca9539.pdf)：原生测距与 I2C 事务。原 12ms 请求在本项目生产预算计算中被拒绝（实际所需 17579μs），删除不改变生效预算。
