# round89e 原生读取重试

按[原厂I2C Communication Guidelines](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf?fileId=8ac78c8c7d0d8da4017d0ebe3508318e)处理唤醒时NACK：寄存器读取复用现有Pico SDK总线事务，最多尝试3次，仅`PICO_ERROR_GENERIC`重试；TIMEOUT、INVALID_ARG和短读立即失败。SDK中GENERIC也包括其他abort，不能把所有该类错误都叫作NACK。没有增加等待或保留旧数据。

每次仍在同一总线锁内执行指针STOP写和读取，重试重新定位寄存器；不改MPR、配置写入、SYNC两次上限、5 ms轮询、15 ms主机读取年龄或持续门控拒绝策略。次数有界，各次SDK截止时间独立；极端错误可延长扫描，成功但超龄的样本仍被拒绝，且不会增加“最终读取失败”计数。

验证：改前/后迁移138/138，完整实际驱动14/14，抽取实际寄存器方法+DIFF方法+触摸管线34/34场景、8260断言，实际reader18/18，独立只读复核通过。新增fake设备动态地址的初次host入口初始化问题已改为显式初始化，最终全过。Pico SDK 2.2.0 / GCC 14.2.1 Release构建通过；`1.6.6-beta3 / BID=round89e`，UF2 186880B，SHA-256 `48F84193BD8592823B8755BDC59052777C4B621EF82A13CB8E16EF1DC007F429`。

在线刷入与版本回读通过；控制器及三片配置与刷前逐字节相同，ATH ON、179/127 profile保留，RAW=0，未执行0xBA。12.027秒239组C1/B1/C0，读取失败累计16→155（增加139），普通滑条输出均零。姿势未控制，不能验收悬空或接触；此前同类12秒窗口增加155，不能由这两次窗口推算改善比例。**有界重试没有消除读取失败，尚未定位传输与SYNC的占比，也未经连续轻触或游戏验证。** 下一步应细分读取失败来源和缓存超龄，避免盲目增加去抖或拉伸。

源与前版UF2备份`_dev_archive/round89e_before_20261001/`（基线0c9032a），设备备份`_dev_archive/round89e_device_before_20261001/`。原始事务在`_dev_tools/round89e_live_20261001/`，汇总见[实机JSON](MBR3116_round89e_刷写与诊断_20261001.json)。隔空问题仍按[ATH对照路线](MBR3116_round89d_ATH对照与后续路线.md)继续，不宣称本版已经解决。
