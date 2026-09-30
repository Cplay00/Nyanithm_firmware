# round89d ATH对照与后续路线

用户仅接受固件和现有寄存器调整。本阶段在round89c诊断固件上完成单颗ATH对照，最终恢复全部原配置；没有新增固件代码。

## 结果

只对0x40修改`DEVICE_CFG2`的ATH_EN（0x4F bit3，0x68→0x60）及原厂CRC；FT CS0–11保持200，灵敏度、HYS、滤波、引脚、另外两片芯片和179/127软件profile保持原值。修改和恢复均通过128字节读回与重启读回。依据[原厂TRM](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf)，手动FT在ATH关闭后接管。

|动作|ATH|第17格DIFF峰值|第17格最终ON帧|主要结论|
|---|---|---:|---:|---|
|连续轻触后移开，用户确认接触连续|ON|255|413 / 1156|有效接触有短暂输出中断|
|手掌约5 mm悬空与远离，提示重复3次，用户确认全程未接触|OFF|255|265 / 1444|手动FT=200仍不能拒绝无接触悬空|
|连续轻触后移开，用户确认接触连续|OFF|255|298 / 1156|能触发，但仍有短暂输出中断|

帧数包含动作前后远离时间。高度和动作时刻由用户手动控制，不能把两组ON帧数差异当成灵敏度/漏触率的定量A/B结果。悬空阶段第15–17格持续出现输出，第19格有少量输出；其中第15、第17、第19格都属于改动的0x40，不是只被未改芯片干扰。第17格有176个DIFF=255样本。原生BUTTON_STAT也在悬空时ON，因此现有硬件候选位与DIFF标量组合仍不能可靠区分接触。

原设置连续轻触中有16帧呈现B1 DIFF=255、C0硬件ON、最终OFF；其中14帧所在C1时间窗有读取失败计数增加。相同高信号且最终ON的410帧中，有84帧所在时间窗计数增加。它支持读取失败与断帧有关，但命令不是同一时刻、计数覆盖全部芯片，尚不能排除BUTTON_STAT失败、超龄和其他时序因素。

ATH OFF轻触末尾另有一帧最终输出，不能将该组结束前一秒标为全零；动作时间没有独立标记。最终单独回读时普通输入33字节均零，RAW=0。详见[实测JSON](MBR3116_round89d_ATH对照证据_20261001.json)。

## 路线调整

1. **先修正或验证原生读取链路。** 官方Datasheet的I2C Communication Guidelines指出低功耗状态可以NACK，主机应重试直到ACK；Host API低层已有重试。当前自研寄存器读取遇到错误即返回，连续门控也会立即撤销输出。下一步只对可重试的传输错误做有界、无额外等待的原生重试实验，区分TIMEOUT与一般错误，保留SYNC一致性、原生OFF和新低信号立即拒绝。不得用长时间保留旧样本掩盖断帧。随后比较失败计数、扫描耗时和连续接触表现。
2. **归并DIFF门限路线的边界。** ATH关闭与最大合法手动FT未消除已确认的悬空；已有400 fF档也没有更低的合法按钮灵敏度可选。可以评估原生HYS的释放行为，但不承诺由此辨别已饱和的悬空与接触，不再把DIFF>255或非法FT当作方案。
3. **转向RAW按需确认的可行性验证。** 先固定一个电极，用原生SENSOR_ID及调试快照验证短时锁存延迟、RAW/基线分离裕量；不把现有C7的40 ms阻塞搬进扫描。只有跨姿势、轻触、悬空及重启后仍可分离，才设计多指调度和面板标定。此前强饱和子集的RAW差距不能当全局阈值。
4. **保留可调信号边界，拒绝虚假毫米精度。** Z/F/ON/OFF可定义经实测的信号区间；当前硬件没有独立接触或距离读数，无法保证同一数值对应固定毫米。若RAW也重叠或多指刷新过慢，应明确记录可达范围与漏触代价，交付“改善及可调”而非声称所有姿势零悬空触发。

NACK规则及Host API参考：[Datasheet](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf)、[Design Guide §5.2.3.2.2](https://www.infineon.com/dgdl/Infineon-AN90071_CY8CMBR3xxx_CapSense%28R%29_Design_Guide-ApplicationNotes-v09_00-EN.pdf?fileId=8ac78c8c7cdc391c017d07236b6e4698)、[原厂Host API](https://www.infineon.com/dgdl/Infineon-CY8CMBR3xxx_Host_APIs-ApplicationNotes-v09_00-EN.zip?fileId=8ac78c8c7cdc391c017d0723702846a2)。

## 恢复与工具记录

第一次ATH写后，主机解析`done\r\n`时在`\r`处提前结束，剩余`\n`使C6二进制读回偏移一字节。脚本已尝试回退；重新独立读回证实三片及控制器都是原值。修正为消费完整换行，并用逐字节假串口验证后，再次施加ATH OFF成功。一次悬空阶段因Windows阶段文件锁中断，未采到动作，已排除；改用原子文件替换和瞬态读错容忍后重新采集成功。

最后恢复0x40原表并逐片核对，控制器及三片配置与`_dev_archive/round89c_device_before_20261001/`逐字节相同，ATH重新ON。原始CSV、完整编程/回读日志、最终快照在工作区`_dev_tools/round89c_live_20261001/`。本阶段只改文档和证据，免固件重建；在线固件仍是已验证的round89c，游戏内手感未验收。仅本地提交，不push。
