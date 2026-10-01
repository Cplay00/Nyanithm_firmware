# round89t EZ-Click与NVRAM配置路线

2026-10-02。针对“彻底关闭SmartSense、低灵敏度、高FT、带Shield，经0xBA刷入NVRAM”的方案继续核查。用户已确认SH没有连接屏蔽铜网，且只接受固件/已有寄存器调整。

可以用EZ-Click导出正式配置，并通过现有0xBA写入3116的非易失配置；但这不替换芯片内部算法。上述组合不能在当前设备原样成立：公开接口支持手动阈值，没有找到完全关闭SmartSense测量调谐的开关；低灵敏度/高FT已到合法边界；Shield缺少物理电极。现有证据不支持仅重新导出这组参数就能解决悬空。

## 配置工具与寄存器边界

本机已安装EZ-Click 2.0 SP2，注册表版本2.0.0.2305，程序位于`C:\Program Files (x86)\Cypress\EZ-Click\2.0\EZ-Click.exe`。随附用户手册001-90407 Rev.*D，第11页说明导出`.h`、`.hex`、`.iic`，第15/18页说明Automatic threshold与Enable shield。此轮只读说明文件，没有运行GUI或生成配置。

MBR3的`.h`包含128字节数组；Intel HEX和BCP指令文件不能当128字节平面二进制直接发0xBA。[AN90071 §5.2.1.2/5.2.3](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)描述该导出与Host API路径。日后导出必须选CY8CMBR3116，逐片保留0x40/41/42地址及实际引脚配置；不把默认0x37配置复制给三片。

通用工具手册出现50–245等阈值说明，不能直接套在3116上；3116专属TRM和AN90071规定FT31–200。工具导出、CRC正确与设备接受证明格式和执行成功，不证明悬空与接触可分。

|项目|官方边界与实机记录|裁决|
|---|---|---|
|关闭SmartSense|ATH_EN关闭自动阈值，使FINGER_THRESHOLD接管；KBA90796区分调谐与手动阈值|只能称手动阈值模式，不能承诺关闭全部调谐或固定前端增益|
|低灵敏度|100/200/300/400 fF；400 fF对应50 counts/0.4 pF。四组恢复表所有已使能按钮均该档|已经最低，没有500/800 fF档|
|高FT|合法31–200；当前配置200，ATH ON时未接管，此前OFF对照已使它生效|不能继续提高至201–255|
|高迟滞|HYS0–31绝对计数，override才生效；同时提高ON、降低OFF边界|最大FT+H=231仍低于255悬空；过大可能增加抬手残留|
|Shield|必须有SH连接的实际屏蔽电极；用户确认未接|保持OFF，不用空脚或地铜代替|
|EMC|3116须禁用CS10–15；当前三片CS10/11使能，EMC位置1也无效|禁用会损失0x40第9/11格、0x41第1/3格，不直接全板开启|
|基线/滤波/去抖/Auto-reset|处理噪声、漂移和状态，并非距离输入|须有对应证据再单独调，不用长等待或吸收长按冒充贴面识别|

依据：[3116 TRM §1.5.5、FT系列、1.5.26及1.5.60](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf)、[KBA90796](https://community.infineon.com/t5/Knowledge-Base-Articles/Tuning-the-MBR3xxx-CapSense-Controllers/ta-p/248038)。231由AN90071 §6.2.2的ON=FT+H推导，不是新寄存器常量。当前HYS override为OFF，存储字段不等于实际自动迟滞。

0x42的CS10/11没有映射游戏格，单片理论上可禁用这些输入满足EMC条件；但不能解决0x40/41的限制，也没有当前问题由EMI造成的证据。AN90071指出EMC增加响应时间，不为本次距离目标安排该NVM实验。

Shield主要用于液体耐受及特定电场/金属耦合布局。[AN90071 §2.6.1、4.2及5.1.1.11](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)还描述接近感应应用中Shield可增加距离；不能从同相驱动推导它必把20–30 mm压缩为1–2 mm。当前无连接，条件已不满足，无需额外用户动作验证。

## 已有NVRAM实测和烧录链路

[round89d](MBR3116_round89d_ATH对照与后续路线.md)的单颗ATH OFF不是RP2040软件模拟。`_dev_tools/round89c_ath_ab.py`通过0xBA给0x40写128字节，仅改变0x4F bit3及原厂CRC；经过保存、芯片复位、普通重启和读回核对，报告包含`after_reboot_equal`。

该组保持400 fF、FT200，其他字段不变；用户确认手掌约5 mm全过程无接触。改动芯片的第17格仍有176个DIFF255样本，原生BUTTON_STAT也ON。软件profile虽保留，原生状态仍说明问题并非只来自RP2040映射。高度为手动估计，不据此量化其他高度误触率，也不宣称所有组合都试过。改由EZ-Click导出相同字段不会获得额外测量模式。

这不等于所有NVRAM调参都无价值：提高有效FT/H可能拒绝低于新ON边界的未饱和弱误触，但也可能漏轻触或增加释放残留。该收益必须实测；它不覆盖已达到255的悬空，不能交付为严格接触或统一毫米距离。

`src/app_link.cpp`将0xBA后的地址及128字节交给`program_cy8cmbr3116_custom()`；`src/production_mode.cpp`执行CRC检查、当前表备份、RAM写入/比较、SAVE_CHECK_CRC(0x02)、有界等待及状态检查、软件复位(0xFF)、按配置地址逐字节读回。期间处理USB任务和看门狗；不使用旧的固定20 ms保存辅助函数。

`initCY8CMBR3116()`的启动烧表调用已注释，正常启动不覆盖NVRAM。保存的阈值可以参与芯片内部判断；游戏上报仍由RP2040完成。128字节配置不是替换3116固件程序。

未来有可验证新组合时：新读设备身份/三片表/控制器配置并备份源和UF2；用正确型号官方配置，对比全部字段并保持地址/映射/供电及无关位；单片保存、复位、读回；先核对原生BUTTON_STAT，再验证HID、轻触/滑动/多押和延迟；不合格则恢复并普通重启读回。无分离依据的表不留作生产NVRAM。

## 本轮裁决与验证

没有当前硬件可实施的“SmartSense全关闭+Shield”组合，也没有新的低灵敏度档位。继续此路线需要新官方接口依据或新的可分测量证据；重复烧写已有ATH/FT/Sensitivity组合没有明确收益。严格贴面与毫米标定目标仍未解决。

离线脚本核对最近四组恢复表一致、CRC/地址有效、每片12个已使能按钮均400 fF/FT200，并从实际32格映射计算EMC冲突；本机工具和手册存在性/哈希通过。见[核对JSON](MBR3116_round89t_NVRAM路线核对.json)、[复现包](MBR3116_round89t_NVRAM核对复现包.zip)。这是记录核对，不冒称本轮新读设备。

本轮无串口/NVRAM操作、新配置输出、固件C++或成品面板修改，免构建/迁移/Mock；仅本地Git提交，不push。设备最后核对状态仍round89g及原三片表。
