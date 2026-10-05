# round90y：V1 通用固件 1.6.6 正式版审修及安装

2026-10-06，以已合并 `6222c7a`／已部署 beta16 为基线，仅维护 `Nyanithm_firmware_hw_v1` 的 `hw_v1` 分支。兼容性存档、cplay、SDK 和 vendored 库未修改。[用户更新说明](1.6.5-1.6.6_更新说明.md)随本次发布物交付。

## 审查及修复

独立只读复核覆盖 USB/CDC、Flash、双核交接、触摸、ToF 与启动告警，最终未发现剩余的具体发布阻断。此结论不是所有硬件组合均无缺陷的证明。

- 配置载荷超时后迟到字节曾进入命令循环：统一 TinyUSB 读取及总截止时间，保留余字节边界，通过 DTR 隔离与 mount/umount 会话 epoch 防止跨会话拼接；半块配置不生效。
- 配置模式逐字节回复及连续命令积压可能超过看门狗：整帧共用 250ms 发送上限，每轮泵 USB/喂狗，发送末尾重新核对会话；新会话不接旧回复尾部。
- C5 阻塞 HID 并在超时后丢失一字节边界：复用正常模式增量载荷状态，RAW 100ms、LED 500ms，迟到字节仅丢弃。
- cfg3 旧算法只限制响应间隔，首次按下未附加延迟：按完整输出格式记录首次观察时间，最多 15ms／32 条有界队列，每次服务一个成熟变化；RAW 格式切换清队列，空气遥测跟随实际输出。队列满时不提前上报，但不能保证保留超出容量的所有变化。
- MPR ECR 读取失败曾按 0 恢复，校准恢复失败仍 ready：检查备份、Stop、目标、恢复及校准 readback，最多三次恢复；失败暴露 not-ready，上电可告警。有效触摸时跳过启动校准保持原行为。
- MPR 状态读取失败曾走普通长释放保留：结果携带有效性，不确认新 ON，仅保留已确认 ON 至故障开始后 50ms，再释放并清凭据。
- 非零逐键阈值低于全局值曾被覆盖：按设置覆盖，0／未映射电极继承全局。独立复核发现中间优化遗漏继承，已在刷机前修正，并用真实 helper／初始化写入 spy 补测。
- ToF 清中断失败曾反复刷新旧锁存：失败立即失效，恢复先丢旧锁存；mux 失败跳过，有效期采用全部 I/O 完成后的时刻。HW3/4 压力读取家族选择与初始化对齐。

复用 Pico SDK I2C/watchdog/multicore、TinyUSB CDC 和原有驱动；未引入第二套硬件抽象。移除重复载荷读取和 RAW 替换。原生语义对照 [NXP MPR121 Rev.4](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf)、[ST UM2039](https://www.st.com/resource/en/user_manual/dm00279088-world-smallest-time-of-flight-ranging-and-gesture-detection-sensor-application-programming-interface-stmicroelectronics.pdf)及本机 SDK 2.2.0/TinyUSB 源码。

## 自动化及构建

| 测试 | 通过场景 |
|---|---:|
| test_round50_migration，修改前 162／修改后新增故障场景 | 190/190 |
| test_mbr_distance_pipeline，生产管线及 MPR 状态故障 | 92/92 |
| test_boot_recovery，告警、初始化失败、真实阈值写入 | 30/30 |
| test_touch_snapshot_readers，HID/CDC 快照及延迟 | 36/36 |
| test_release_faults，当前十个生产函数的故障注入 | 36/36 |
| test_firmware_audit，Flash/CDC/LED/双核 | 21/21 |
| test_native_register_dump | 28/28 |
| test_mpr_read_contract | 24/24 |
| test_mbr3116_difference_counts | 27/27 |
| test_status_watchdog，慢速 B8 输出 | 8/8 |
| test_mpr_baseline_policy | 6/6 |

共 11 组、498 场景通过，当前证据位于 `_dev_tools/round90y_*`；旧默认目录历史报告不作本轮证据。主机故障注入不等于真实硬件故障测量。

SDK Release 构建通过：UF2 206848B／404 块，目标范围 `0x10000000..0x10019400`，不含末配置扇区。身份 `NYANFW1;1.6.6;hw_v1;BID=round90y-v1-release`，构建时间 2026-10-06 02:31:30。UF2 SHA256 `d395857e50917eb8c19ea2e39bb7dfc01ae4c5385ef485940ef8b4c6be216dc0`。

ELF text 107372B／BSS 12124B，相比上次合并基线构建分别增加 1880B／1216B。延迟队列容量固定，cfg3=0 不等待队列。

## 安装及实机回归

| 设备 | 实际硬件／序列号 | 结果 |
|---|---|---|
| 32 寸 HW2／COM6 | 三片 MBR3116，5303284739002C9C | 正式版身份正确，配置相等，三片 128B 原生表全部相等；五路 ToF ready=1F。 |
| 27 寸 HW1／COM1 | 三片 MPR121，50443405C0B5A81C | 正式版身份正确，配置和原生控制字段相等；四路 ToF ready=0F。原生自动调谐 CDC/CDT 字段允许复位后变化。 |

两台实际迟到 RAW/LED/CFG_SET 隔离、DTR 恢复、相同配置 CFG_SET→CFG_SAVE→重启→回读及 CE 日志均通过。32 寸受控配置错误时记录 mismatch=1，提示后正常进入配置，原配置随后恢复。三次全红与 2s/0.5s 时序由生产函数 spy 验证，未有人眼光学观测。

各 90 秒／112 帧 BD/B1/C1 监测中，运行时间、扫描计数持续增加，无复位或停滞，未用 BC 喂狗。平均扫描：32 寸 1.461ms、27 寸 1.180ms；升级前各 15 秒观测约 1.409／1.137ms。距离、触摸和环境未受控，不把约 4% 差异归因于代码或作为受控跑分结论。

配置 SHA256 保持：32 寸 `a83b06fa09dcdde59f56f918b22ef165c30eb165a0f45f949d4b37a1665fbf39`；27 寸 `af26d4f789968d5f725599206c0caa3bbe82724b1903339ce4859f7b1dec5760`。无 MBR NVRAM 写入。未取得完整物理 Flash 镜像，备份含源码、UF2、配置及原生芯片读回。

Windows 两台 BusReportedDeviceDesc 已确认固件 USB 产品／CDC 接口名称 `Nyanithm Controller V1`。系统缓存 FriendlyName 需管理员权限；SetupAPI 脚本已完成只读计划及独立复核，但 RunAs 返回“操作已取消”，未提权，所以系统仍显示 `USB 串行设备 (COM6/COM1)`。驱动及 COM 号不变。管理员 PowerShell 执行 `tools/update_v1_com_name.ps1 -Apply` 可完成此步；要求来源为 [微软 SetupDiSetDeviceRegistryProperty 文档](https://learn.microsoft.com/en-us/windows/win32/api/setupapi/nf-setupapi-setupdisetdeviceregistrypropertyw)。

## 备份、缓存及边界

修改前完整 ZIP `_dev_archive/round90y_release_before_20261006_015804/main_complete.zip` 含 12222 文件，CRC、逐文件 SHA256 和全 refs bundle 均已核验。ZIP SHA256 `e0a4d4cc317077e531bcbdc0811800ce92408a94e5ee6845a3b8e8509dc7da5c`。设备证据 `_dev_archive/round90y_devices_20261006`；发布物 `_dev_archive/round90y_release_1.6.6_20261006`。

删除前再次核对 11049 个旧缓存文件与备份 SHA256 相等。仅清理主树 20 个旧 build 目录，共 1233180775 文件字节（约 1.15GiB），保留 build_release、发布物、资料及历史恢复树，未清理 SDK 或其他变体。

游戏内轻触、滑动、保持、空气键手感仍待用户验收；单 Flash 扇区擦除时断电注入未覆盖。通信故障处理不能保证消除原生信号饱和造成的隔空触发。仅本机交付和提交，不 push。
