# MBR3116 round90f：原生信号边界、离线复核与下一次实验

2026-10-03；主维护树 `Nyanithm_firmware_hw_v1`，分支 `hw_v1_compatible`，研究基线 HEAD `101f51d`。

**已完成无需用户配合的原厂语义核查、历史 RAW 复核和离线工具。RAW 保留了 DIFF=255 后的变化，但单格 RAW/BASE/DIFF/CP 仍有相反动作标签的完全重合记录，不能直接换成原始差值门限。** 当前没有通过验证的新抗悬空判据。

本轮不访问设备。上轮最后部署/验收记录是 beta10 / round90e，用户反馈“手感正常，隔空仍存在”；本轮没有新增刷写、NVRAM 或控制器配置操作，未改变 C++、SDK、vendored 库、协议和面板。研究编号 round90f 不代表存在 beta11 固件。

## 1. MPR 对照必须修正的原厂语义

下表的源码位置是研究基线，寄存器写入意图不是本次 MPR 模块的实机读回。用户已确认恢复三片 MBR，其他未变；尚无那次无隔空 MPR 的实际逐电极快照。

| 核查项 | 官方含义与源码事实 | 对后续的影响 |
|---|---|---|
| CONFIG2=0x28 | CDT=0.5µs、SFI=6、ESI=1ms；`src/mpr121.cpp` 的“SFI=2”注释有误 | 名义输出更新为6ms；不能把每次1ms轮询当新硬件样本。扫描耗时超过ESI时周期还会变长 |
| 最终充电参数 | 个别 CDCx/CDTx 非零时覆盖全局；`initMPR121()` 先启用 AutoConfig，等待200ms后关闭 | 必须读0x5F–0x72。全局16µA/0.5µs不能代替实际逐电极增益 |
| ECR/CL | 00保留当前基线寄存器初值；01禁用跟踪；10/11从首次电极数据装载高5位/全部10位并跟踪 | `calibrateBaseline()` 用CL=11却注释为装载手写基线；`writeRegister()` 将11改10却称“只跟踪”，均需按原厂装载来源重新审查 |
| BVA掩码 | BVA是0x7B的bits3:2；bits5:4是RETRY | 校准中 `& ~0x30` 清的是重试位。当前启动先将0x7B清零，该步骤通常无实际影响，不能据此解释MBR隔空 |
| 基线写入时序 | §5.1规定写寄存器须Stop，ECR及GPIO/LED是例外 | `writeBaselineRun()` 在Run直接写0x1E+e，应列入接口修正任务；改动前必须保留已接受MPR版本并完成真实MPR回归 |

依据：[NXP MPR121 Rev.4 §5.1、5.8–5.11](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf)、[AN3890 Filtering and Timing](https://www.nxp.com/docs/en/application-note/AN3890.pdf)。另已确认FDLT=255是减慢滤波，不能凭此宣称禁用跟踪。[AN3891 Baseline System](https://www.nxp.com/docs/en/application-note/AN3891.pdf)

这些是接口解释及合规问题，不是已经证实的MBR悬空根因。本轮只登记问题，避免在当前实装MBR时顺带修改已接受的MPR行为。后续先读回，再将“必要接口修正”和“抗悬空规则”分开提交。

MPR以充电后电压间接测电容；I、T、VDD和电极电容共同决定计数灵敏度。没有实际逐电极参数和电压，不能用MBR测得的CP替代MPR电容，也不能把counts转换成手指距离。[NXP AN3889](https://www.nxp.com/docs/en/application-note/AN3889.pdf)

## 2. 现有 MBR 寄存器有没有新的接触信息

| 输入 | 可用边界 | 本轮决定 |
|---|---|---|
| 全格按钮DIFF | 普通按钮0–255；原生读组内部同步；与另读BUTTON_STAT不保证同扫描 | 保留round90d/e质量处理；不能恢复截断信息 |
| 单格RAW/BASE | 0x82选择电极，等待实际刷新，读取0xDB–0xE7同步组并核对DEBUG_SENSOR_ID | 用作机理诊断；没有全板同时RAW接口 |
| SYNC_COUNTER | 有效字段仅4位，0–15，用于读组一致性 | 不当独立扫描序号；同值或跨模跳变不证明读取相同/不同物理扫描 |
| DEBUG_CP | 整数pF；TRM正文和位字段对“每次刷新/改变SENSOR_ID更新”的描述不同 | 不假定亚pF分辨率或高速动态接触信号；本批保持窗CP均为30pF |
| DEBUG_AVG_RAW | 仅启用ALP的接近传感器有定义；普通按钮或ALP关闭时未定义 | 排除当前普通按钮及旧ALP OFF参考记录的这一字段 |
| REFRESH_CTRL | 控制Look-for-Touch/Prox；Active典型20ms或更长的扫描处理时间 | 不通过改此字节宣称Active扫描变为1ms，不将主机帧率当独立刷新率 |

依据：[MBR Registers TRM §1.5.63、1.5.103、1.5.121–128](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf)、[MBR Datasheet，Power States](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf)。诊断选择后等待、13B读组与原厂Host API一致。[原厂 Host APIs](https://www.infineon.com/dgdl/Infineon-CY8CMBR3xxx_Host_APIs-ApplicationNotes-v09_00-EN.zip?fileId=8ac78c8c7cdc391c017d0723702846a2)

未发现可直接替代32格DIFF、且已有规范保证的低成本接触专用寄存器。这不证明所有未来硬件/算法路径都无解，但否决了把上述字段改名成“距离”或“新扫描计数”的方案。

## 3. 四次历史动作的 RAW 复核

输入是round89q的四次已完成、用户确认动作，beta4/round89g；中心0x40:CS7普通按钮，参考0x42:CS0临时16位Proximity、ALP/自动复位关闭。两者经C7顺序读取，不同时。每次动作结束后的控制器和三片备份/重启回读逐字节相等，四次原始备份也一致；**进行中的临时模式依据当时应用/恢复日志，未在这组文件中保存逐片实时NVRAM快照**。

输入44个文件逐个SHA-256核验；722条CSV，21条错误排除，另39条无错误但phase_valid=0排除，不补零。保持窗沿用旧round89s的6–11.5s，175个合格观察记录；没有为新结论重选窗口。原始双SYNC值未保存在CSV中，脚本不能独立重验当时的读组一致性。

| 既有动作标签 | 保持窗观察 | DIFF=255观察 | RAW−BASE范围 | CP |
|---|---:|---:|---:|---:|
| 单指接触 | 43 | 40 | 111–203 | 30pF |
| 单指约3mm悬空 | 43 | 0 | 53–92 | 30pF |
| 手掌约5mm悬空 | 45 | 2 | 85–146 | 30pF |
| 手掌接触 | 44 | 44 | 226–541 | 30pF |

手掌接触44个观察全部DIFF=255，RAW仍由3055变到3370，RAW−BASE由226变到541。这说明这批记录的255输出限制并未让所有诊断值失去变化；它不等于已经验证整个模拟前端没有非线性或削顶。所有有效阶段中，115个DIFF未达255且原始差值>0的记录，`DIFF/(RAW−BASE)`为1.8611–1.8824，中位1.8774；这只是本电极、本配置下的经验比例，不是官方通用常数，不用于反推饱和DIFF、单位换算或生成配置。

更强的约束来自完全重合：三个不同中心元组同时见于“单指接触”和“手掌悬空”的固定保持窗。例子：

| 中心值 | 单指接触记录 | 手掌悬空记录 |
|---|---|---|
| DIFF=208、RAW=2941、BASE=2830、CP=30 | `finger_contact.csv`第52行，6.11394s | `palm_hover5.csv`第93行，11.19640s |
| DIFF=255、RAW=2968、BASE=2830、CP=30 | `finger_contact.csv`第66行，7.83760s | `palm_hover5.csv`第53行，6.27825s |

行号含CSV表头；完整文件ID、全部三个元组及参考通道值见[RAW复核结果](MBR3116_round90f_RAW复核.json)。这否决仅凭**当前单格**RAW/BASE/DIFF/CP严格区分这组动作标签；不否决取得新信息的时序/多通道方法。整次动作确认和固定保持窗仍不是独立物理接触时刻证明，175个观察也不是175次独立接触。

旧16位参考通道在这组动作中有不同响应，但[round89s](MBR3116_round89s_同配置结论与现有硬件边界.md)已发现拟合不泛化；本轮没有重新拟合，不把临时单参考变成全板总门。当前生产profile OFF，历史模式不是当前实机配置。

## 4. 读取结构与实时成本

当前C7配置模式路径每次写SENSOR_ID后等待40ms，即使再次选同一格；轮换32格仅等待就至少1.28s。该诊断不用于证明游戏响应延迟，也不能塞进每轮输入扫描。

按源码BR_I2C=400kHz、每字节含ACK/NACK的9个时钟计算，下表只含一次成功的地址/指针/数据时钟，不含START/STOP、软件、时钟拉伸、锁等待和重试，**不是耗时上界**。

| 交易 | 单片理论时钟成本 | 三片顺序成本 |
|---|---:|---:|
| MBR35B差值组：`(35+3)*9/400000` | 0.855ms | 2.565ms |
| MBR13B固定选择诊断组：`(13+3)*9/400000` | 0.360ms | 1.080ms |
| MPR0x00–0x2A原生43B一致组 | 1.035ms | 3.105ms |

未来若准备固定选择诊断流，应在进入时选择/等待、后续只读相同选择，并验证刷新及失败语义；先作为可关闭的研究诊断，不能当全板RAW算法。默认关闭、不阻塞正常游戏、诊断取消/断连后恢复原模式。需要修改CDC/跨核时另做只读审查及实际代码测试。

## 5. 下一步收敛到一个主实验

**主实验是MPR实际测量参数与原生一致信号对照**，用来区分“测量增益/输出范围”和“基线/处理策略”假设。需要用户回来并允许换回MPR时才开始，本轮已准备离线字段解码器和记录要求。

1. 保留beta10及四块当前配置；核对模块版本、接线和用户确认的其他未变条件。读MPR0x5B–0x72、0x7B–0x7F、阈值、0x2B–0x40以及状态/OOR，保留原始字节、地址、成功标记、读取起止和硬件身份。单纯读取不切Stop、不校准、不写基线。
2. MPR信号用原厂支持的0x00–0x2A单次多字节读取，分别标记每片的读取时间。当前压力路径分开读滤波值和基线，不能直接当这个一致组。解码基线高8位时保留低2位未知范围，不假造内部精确10位基线。
3. 先一个代表格，记录从靠近到轻触、约4秒慢触、短长按到释放；再用手掌接近作面积反例。保持真实动作标签，未有独立接触时刻时只做整次动作比较。参数完整后比较接近/接触相对实际阈值的裕量，不跨芯片直接比counts。
4. MBR补采只在能区分新假设时实施：普通按钮原配置下固定同一格RAW/BASE/DIFF，读组严格有效；不再重复旧四组静态保持来拟合单幅值阈值，不先烧新表。记录两种芯片采样周期不同的限制。
5. 若MPR的实测电流/时间、计数范围或基线策略提供可测解释，优先寻找MBR官方等价能力；没有官方等价接口时明确记录边界，再判断模块测量条件或额外信息需求。没有区分裕量则停止同类门限重试。

驱动工作按“实际读取契约→最少必要结构→质量/时限→芯片专属判据→完成快照”推进。round90d/e已经完成的同步读取与故障准入不重写；MPR上述原生语义修正另立小提交并做真实MPR回归。未通过慢触、多押、滑动及响应成本的规则不进入HID，不用ToF充当接触真值。

## 6. 离线工具、证据与验证

- `tools/decode_mpr121_snapshot.py`：仅解码保存的字节；强制声明hardware-readback/source-intent/synthetic-test来源。缺失逐电极字节保留未知，不把全局值冒充实际值；输出名义周期、CL/BVA、逐电极充电参数和可读基线精度。输入自述hardware-readback也不等于验证身份或读取成功。
- `tools/analyze_mbr_raw_evidence.py`：校验输入哈希、完整动作、固件/设备分层、四块恢复、时间顺序、原始差值算术和范围；保留错误/phase排除，输出饱和统计及精确重合样本。没有串口/网络、NVRAM编码器、阈值拟合或在线判据。
- [输入清单](MBR3116_round90f_输入清单.json)、[源码写入意图](MBR3116_round90f_MPR源码写入意图.json)、[字段解码](MBR3116_round90f_MPR字段解码.json)与[证据包](MBR3116_round90f_离线证据.zip)保留可复核输入。源码意图不冒充现场数据。

工作区中复跑（主变体目录）：

~~~powershell
py -3 tools/analyze_mbr_raw_evidence.py docs/MBR3116_round90f_输入清单.json --data-root ../_dev_tools/round89q_no_autoreset_20261002 --out docs/MBR3116_round90f_RAW复核.json
py -3 tools/decode_mpr121_snapshot.py docs/MBR3116_round90f_MPR源码写入意图.json --out docs/MBR3116_round90f_MPR字段解码.json
py -3 tools/test_touch_offline_research.py
~~~

本机本轮实际使用Python3.14绝对路径，因为当前shell的`py -3`入口不可用。24项测试通过，包含原厂编码、缺失参数不填零、来源标记、精度，以及损坏字节/算术/恢复配置/未确认动作/非有限时间/改窗拒绝。证据包解压后可用同一工具复跑，包内说明给出命令；字节哈希和输出一致性另有核对。

本轮只有Python离线工具和文档，没有固件或触摸管线行为修改，豁免固件编译、迁移、Mock和新增真机游戏验收；工具测试不等于传感器或抗悬空验证。本地提交，不push；MBR隔空仍未解决，后续换模块、动作采样和游戏验收留待用户在场。
