# round89f 电容原理与方案修订

2026-10-01。按用户新增观察重新研究原理，仅更新文档；固件、面板、设备配置均未改动。范围仍为固件和现有寄存器。最近确认的设备版本为 `1.6.6-beta3 / BID=round89e`。

## 1. 观察与物理含义

用户确认图中高度从磨砂亚克力顶面计算，覆盖层约2–3 mm。以下是目测区间，不是量块标定结果：

|物体|面板显示约0时的高度|显示255时仍悬空的高度|
|---|---:|---:|
|单指|5–7 mm|约3 mm|
|手掌|8–10 mm|5–6 mm|

两者实际接触顶面都是 `h=0`。读数封顶发生在真正接触之前；掌面比单指更早封顶。这个观察支持面积和间隙共同影响感应，不能单独证明内部增益、几何面积或毫米换算系数。

[AN90071 §2.1–2.2](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)描述的是电容耦合：人体改变电极对环境的电容，CSD PLUS将其转成RAW计数。手无需导通PCB；亚克力是介质。理想平板模型中的重叠面积增大、间隙减小都会增大耦合。对空气间隙 `h` 与覆盖层厚度 `t`，可用 `ΔC ≈ ε0 A / (h + t/εr)` 理解趋势。这是根据串联介质推导的示意模型，不是此PCB的距离公式；有限电极、边缘场和人体回路会改变实际关系。

同一信号可能对应较小物体更近、较大物体更远。面积还受单格电极大小限制，不能直接使用整只手掌的几何面积。面板的0受基线、噪声及报送模式影响，也不能解释为物理耦合严格为零。

## 2. 两种芯片的真正差异

|项目|MPR121|CY8CMBR3116按钮|
|---|---|---|
|测量|恒流充电、固定时间后测电压；电容增大时电压读数降低|CSD PLUS转换电容；RAW随电容增加|
|固件用于模拟压力的量|`baseline - filteredData`，报送层可再乘2|芯片已归一化的DIFF，按钮范围0–255|
|原生调谐自由度|可设置CDC电流、CDT时间等|SmartSense自动调谐，按钮提供四档灵敏度及阈值设置|
|是否只能接触后响应|否|否|

MPR测量及调谐依据[NXP AN3889 §Capacitance Measurement / Configuration](https://www.nxp.com/docs/en/application-note/AN3889.pdf)。[AN3893 Overview](https://www.nxp.com/docs/en/application-note/AN3893.pdf)明确说明普通输入也可以检测未接触的靠近，面积和配置决定表现。用户此前MPR设备的接触体验可以作为需求，但不是MPR必须接触才改变电阻的原理证据。两种固件的数值也不能按同一压力/距离刻度比较。

本仓库 `src/hw_devices.cpp` 的 `mprReadPressureSnap()`、`mbrDistanceReadSample()`、`mbrReadPressureSnap()`及 `src/chuni_io.cpp` 的GET_INPUT报送符合上述数据来源。**C5 level=2的“原始结果”在MBR上仍是归一化DIFF，并不是DEBUG_RAW_COUNT。** 因此观察B1的0–255无法检验模拟前端RAW是否饱和。

## 3. 255在哪里产生

原厂[2018年技术答复：Regarding EZ_Click/MBR3 SNR](https://community.infineon.com/t5/PSOC-4/Regarding-EZ-Click-MBR3-SNR/td-p/106939)说明 `DIFF = (RAW - BASELINE) × K`，K由内部算法决定，用户不能直接设置。结合[AN90071 §6.2.3](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)的按钮截断规则，可用以下链路解释：

```mermaid
flowchart LR
    A[面积、间隙、覆盖层和人体耦合] --> B[电容变化]
    B --> C[CSD PLUS / SmartSense]
    C --> D[RAW 与 BASELINE]
    D --> E[内部 K 归一化]
    E --> F[按钮 DIFF 截断到255]
    F --> G[固件门控 / 面板0–255]
```

255证明按钮DIFF已封顶，**不证明RAW也封顶，不证明接触或满压力**。两种真实幅度都被压成255后，除以2、更换显示刻度或加一个软件“低灵敏度”系数仍无法重新区分。应取得截断之前的信号。

[原厂KBA90796](https://community.infineon.com/t5/Knowledge-Base-Articles/Tuning-the-MBR3xxx-CapSense-Controllers/ta-p/248038)将灵敏度设置和手动阈值控制分开说明。关闭ATH接管判定阈值，不等于关闭SmartSense或取得IDAC/归一化K控制权。原厂公开寄存器中未找到可将全部按钮任意配置为更低增益或取消255截断的接口；不使用未公开寄存器推测。

## 4. 当前设备的可调边界

[TRM Rev. E](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf)的相关定义：SENSITIVITY四档为50 counts/0.1、0.2、0.3、0.4 pF；按钮FT合法31–200；CS0/CS1可改为Proximity，DIFF范围0–65535，PROX_TOUCH_TH控制其BUTTON_STAT。RAW、BASELINE调试字段是16位存储，CP调试字段为整pF；字段宽度不保证实际有效分辨率或不饱和。禁用传感器的读数未定义。

本次设备证据仍是三片CS0–11全部400 fF、ATH ON、FT=200，0x40的ATH OFF单变量实验已恢复。该实验在用户确认未接触的手掌约5 mm悬空时仍出现DIFF=255及输出。见[ATH实测](MBR3116_round89d_ATH对照与后续路线.md)。这足以否定“只让FT=200生效就解决”的方案，不足以否定RAW等所有固件路线。

按钮auto-reset现为20秒。等待足够久后归零不能当作距离截止：长时间真实接触也可能被复位。NT基线追踪同样没有独立接触信息，快速靠近和停留历史必须一起验证。HYS、去抖、滤波用于过渡与噪声，不会恢复已截断的信息。

## 5. RAW方案先验证哪些问题

**第一关：信号是否可分。** 保持原表，固定0x40:CS7（第17格），同时记录RAW、BASELINE、DIFF、有效/失败标签和动作时间。采集单指/手掌接触、悬空、缓慢靠近、抬起，并在区域内复测；后续覆盖其余区域。比较 `s=RAW-BASELINE`，另评估 `q=(RAW-BASELINE)/BASELINE`，BASELINE=0时无效。q只是可能改善尺度漂移的实验量，不能假定它必然消除SmartSense变化。

先用当前C7完成静态分离验证；它写易失SENSOR_ID，不编程NVM。现有强饱和子集中，接触的RAW差值157–261、手掌悬空137–149，但接触过程中还出现过更小差值。不能取150作为全局截止，更不能把强信号子集的间隔当成整组接触裕量。判断必须包含弱接触和全部有效帧，失败帧单列，不能补零。

只有禁止类别的保守上界Z低于有效目标类别下界，并且留有噪声/漂移裕量，才允许建立Z/F及ON/OFF标定。按每次启动和不同姿势复验尺度；这仍是用户标注条件下的信号标定，不是独立测得的毫米距离。若重叠，停止固定标量门控的部署，保留明确的失效类别。

**第二关：多指是否及时可读。** [原厂Host API ReadSensorDebugData](https://www.infineon.com/dgdl/Infineon-CY8CMBR3xxx_Host_APIs-ApplicationNotes-v09_00-EN.zip?fileId=8ac78c8c7cdc391c017d0723702846a2)要求选择SENSOR_ID后等待一个refresh interval，再读取并校验调试窗口。每片一次只选一个电极。现有C7固定等40 ms，只适合诊断，不搬进游戏扫描。

[Datasheet Rev. Q，Power Consumption and Operational States / Response Time](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf?fileId=8ac78c8c7d0d8da4017d0ebe3508318e)给出Active约20 ms典型刷新；扫描处理时间更长时刷新也更长。REFRESH_CTRL用于Look-for-Touch/Proximity，不能承诺按钮1 ms刷新。主机5 ms读取年龄与芯片测量年龄不同。

按原厂一个刷新间隔串行切换同片12个电极，典型轮询约240 ms，尚未计总线/重试；三片并行选择也不能消除同片的12个选择周期。这是方案预算估算，不是实机测量。候选按需调度只能作为实验：必须验证同片多指、快速换键和释放，不以单指成功替代全板验收。SYNC相等只证明一致性，不证明采到了新扫描。

读取稳定性诊断仍有必要，但它是保证证据有效的支线。round89e有界重试后仍存在失败，待细分传输/SYNC/超龄；它本身不会改变悬空电容。

## 6. 其他现有寄存器路线

|路线|能验证什么|推进条件/限制|
|---|---|---|
|Proximity的较宽信号与触摸门限|局部电极是否能获得更多幅度空间|每片仅CS0/CS1；三片映射第2、4、18、20、29、31格。保留BUTTON_STAT触摸判定的可能性，需改固件尺度/门控配套，不直接启用当前按钮管线；仍受面积/距离混淆|
|相邻电极轮廓或参考信号|能否辅助识别某些悬空姿势|先做离线分类及多指/手掌接触反例，不能只按全板平均幅度或触发格数拒绝|
|NT/HYS/滤波单变量|是否改善噪声、过渡和特定姿势误触|静态饱和无法据此可靠辨认接触；量化轻触、慢接近、长按和延迟代价|

Proximity需要实验，不等于六个电极能代表任意位置的手高度；目前未实施该实验。全板方案仍优先验证RAW可分性。没有新增电极或独立接触传感器时，不能承诺所有面积与姿势都严格零悬空。

## 7. 与旧计划比对后的修订

保留旧计划的全表备份、官方语义、单变量、读回、重启和HID验收要求，修改以下判断：

- ATH假设已裁决且A/B不满足最终目标，优先级不再停留于“FT未生效”。
- 16位DEBUG_DIFFERENCE字段不自动打破按钮归一化饱和；真正应比较DEBUG_RAW_COUNT与BASELINE。
- NT“吸收悬空”只能改变基线历史，不能当作独立接触鉴别或先验根治。
- 将Proximity一概描述为没有触摸状态过于绝对；原厂有PROX_TOUCH_TH/BUTTON_STAT，但仅两个指定输入，现有固件也不支持其量程。
- 旧计划的未接线CS15“健康哨兵”不采用：禁用读数未定义。
- 不用固定线性DIFF→mm换算，不把C7等待、SYNC一致或主机轮询频率叫作已证明的新芯片刷新。

本阶段仅研究及文档提交。未执行串口命令、刷写、寄存器写入或新固件编译；UF2仍为round89e，源代码未变。下一次需要用户配合时，先做同一电极的RAW接触/悬空分离采样，提前提醒并先启动记录，避免准备期间已被20秒auto-reset吸收。
