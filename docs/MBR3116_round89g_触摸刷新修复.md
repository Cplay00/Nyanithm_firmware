# round89g：触摸刷新与旧帧滞留修复

## 问题与修复

用户报告round89e严重低刷新、拖判。本轮暂停距离标定，先修复上报链路。

刷前COM6内部扫描约205 Hz、每周期4.884 ms。RAW=0且cfg2=0时，B1仍返回历史非二值压力，C0输出全零。原`touchStateGen`在空气、压力和触摸I²C扫描全程为奇数；B1最多等待1 ms，主循环又立即开始下一次扫描，完整帧窗口过窄，导致反复复用历史上报。

Core0保留工作数组，扫描结束一次发布HID按键、32格判定、压力、空气、硬件位及验证位。generation仅覆盖81字节字段复制；HID、B1、C0全部读取完成帧，扫描期间也能读取上一完整帧。writer/reader使用[Pico SDK `__dmb()`](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_sync)保证复制顺序，ARM产物确认实际生成DMB。结构82字节含尾padding，仅内部使用，线上33/46/528字节及128字节配置不变。

当前严格门在原生OFF或读取失败时立即清空保持状态，本轮未调整sticky、dip、阈值、寄存器、扫描周期或重试。MPR共用完成帧发布路径，触摸处理保持。

## 验证与实机

- 迁移改前/后138/138；真实生产publisher/readers23/23，覆盖未发布工作数组、RAW 2→0、字段/DMB、撕裂、超时、回绕。
- 真实驱动与触摸管线34/34、8260断言；独立只读复核无阻断问题，独立reader及ARM发布顺序通过。
- Release ARM构建：1.6.6-beta4/BID round89g，UF2 186880B，SHA-256 `AA4A65DBBF98CCB809142BB765D8ACE786A489BAA742EDC8AA32DC25DAEFC941`。
- 已刷COM6，序列号`5303284739002C9C`；另一台COM1未操作。刷前/后控制器及0x40/41/42配置逐字节相等，179/127严格门、cfg3=0保留，RAW最终0、串口关闭。未写MBR NVM。

同主机、RAW=0、约20 Hz连续发送B1+C0，各120组：

| 指标 | round89e | round89g |
|---|---:|---:|
| B1+C0往返中位数 | 5.956 ms | 1.761 ms |
| 此窗口最大往返 | 8.557 ms | 3.944 ms |
| 内部扫描频率 | 203.44 Hz | 201.42 Hz |

三次RAW 2→1→0，每档200次B1，共1800帧，无读取超时，模式回显正确，模拟/关闭档均为二值。单B1约1320–1407回复/秒，**不代表电极测量刷新率**。窗口均为零信号、姿势未控制，实机模式切换不能替代有信号触摸验收；对应竞态另由生产代码抽取测试验证。往返耗时不是触摸到游戏判定的实测延迟。

原生扫描及I²C/SYNC失败仍为独立限制：刷后12秒239组遥测失败计数仍增加148，不能声称断触或隔空问题已解决。cfg1=0，未做实际HID/游戏手感验收。下一步由用户先复测连续滑动、快速轻点、抬手释放，恢复后再做RAW距离/面积对照。

## 回退与证据

- `_dev_archive/round89g_before_20261001`：源码、工具及旧round89e UF2，基线`27f7c90`；旧UF2哈希`48F84193BD8592823B8755BDC59052777C4B621EF82A13CB8E16EF1DC007F429`。
- `_dev_archive/round89g_device_before_20261001`：刷前设备配置。
- 工作区根`_dev_tools/round89g_latency_20261001`、`round89g_live_20261001`、`round89g_pipeline_host`及`round89g_build.log`/`round89g_arm_disassembly.txt`：原始时序、USB、刷写、测试、构建及反汇编证据。
- [实机证据JSON](MBR3116_round89g_刷新证据_20261001.json)。仅本地提交、不push。
