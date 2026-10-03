# MBR3116 round90d：原理核查、空闲电容与一致读取修复

2026-10-03；主维护树 `Nyanithm_firmware_hw_v1`，分支 `hw_v1_compatible`，修改前 HEAD `20b4af4`。

**首批官方接口核查、历史反例回放、当前设备备份与空闲测图完成，最小读取修复已刷入。用户游戏反馈“手感正常，隔空仍存在”；抗悬空尚未根治。** 版本 `1.6.6-beta9` / `BID=round90d`。详细总计划见 [重构计划](MBR3116_算法与驱动重构计划_20261003.md)。

## 1. 当前设备与对照边界

用户确认“已换回 MBR3116，三片都是 MBR3116”“其他未变”；空闲采集前确认“已离开”，双手及物体移离至少20cm。替换结果支持进一步检查两种测量链及模块内部电路，不能据此确定某个软件函数是悬空根因。

- 目标 VID/PID `CAFE/4009`，序列号 `5303284739002C9C`，当时 `COM6`。
- 检测帧 `bc000e050502`：没有MPR，MBR地址 `0x40/0x41/0x42`，5个ToF，hwVer=2；走三芯片、32格路径。
- 刷前固件beta8/round90b/SDK2.2.0，RAW=0，内部事件记录OFF、pending=0。
- 当前距离profile **OFF**，八字节 `214,1,0,0,255,70,50,251`。与10月2日末次快照相比，控制器byte7/8为12/10（原10/8），byte93为0（原1），相关校验随之变化；三片MBR配置完全相同。这些变化在本轮操作前已存在，原因未证实，不能把旧ON实验与当前OFF当同配置对照。
- 三片当前SENSOR_EN=0x0FFF、CS0..11启用，启用通道灵敏度配置400fF、FT配置值200、ATH=ON、Proximity=OFF。ATH=ON时FT配置值不等同所有时刻的实际自动阈值。

刷前源码 `git archive HEAD`、匹配本地UF2、控制器128B及三片各128B备份到 `_dev_archive/round90d_before_20261003/`。旧UF2 SHA-256为 `2c59f8be0289a2bacbca40a7fadc59d83e32d48f76a278c9e7a0667dd2335d18`；这是核对版本的本地构建产物，不是从芯片读出的Flash镜像。

## 2. 官方原理与生产调用链

| 环节 | MBR3116 | MPR121及本树实现 | 对下一步的意义 |
|---|---|---|---|
| 电容测量 | CSD PLUS与SmartSense测量/调谐，内部固化程序 | 恒流充电、定时采样、10位ADC；电流/充电时间可配置 | counts不属同一量纲，不照搬门限 |
| 基线和信号 | DIFF经过归一化，不能直接当RAW−BASE | 可读滤波数据和基线，本树使用基线减滤波值 | 先解释调谐/归一化差异，再讨论软件基线 |
| 按钮信号范围 | 普通按钮DIFF为0..255，超过截断；16位容器不扩大范围 | 滤波数据10位，基线寄存器给出高8位 | 容器/软件门限不能恢复饱和丢失的信息 |
| 实时全格差值 | 原厂ReadDiffCounts是0xB9..0xDB共35B，首尾SYNC验证差值组 | 连续读取遵循自身寄存器语义 | 本次复用已有严格MBR读组 |
| 单格诊断 | SENSOR_ID选择RAW/BASE/CP，切换后等刷新；CP整数pF | 逐通道数据可读 | 轮换MBR32格诊断不是同时测量，不直接放入游戏循环 |
| RP2040能力 | 可改读法、质量管理与状态机，不能重写MBR内部扫描程序 | 本树有独立MPR初始化/基线策略 | 重写是否根治取决于可分输入 |

依据：[MBR Datasheet](https://www.infineon.com/assets/row/public/documents/30/49/infineon-cy8cmbr3002-cy8cmbr3102-cy8cmbr3106s-cy8cmbr3108-cy8cmbr3110-cy8cmbr3116-datasheet-en.pdf)、[MBR TRM Rev. E](https://www.infineon.com/assets/row/public/documents/30/57/infineon-cy8cmbr3xxx-capsense-express-controllers-registers-trm-additionaltechnicalinformation-en.pdf)、[AN90071 §6.2.3](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)、[原厂Host APIs](https://www.infineon.com/dgdl/Infineon-CY8CMBR3xxx_Host_APIs-ApplicationNotes-v09_00-EN.zip?fileId=8ac78c8c7cdc391c017d0723702846a2)、[MPR121 Datasheet](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf)、[NXP AN3889](https://www.nxp.com/docs/en/application-note/AN3889.pdf)。

Host API的SetDebugDataSensorId要求等刷新后读诊断；ReadDiffCounts检查首尾同步。本树继续用Pico SDK I2C及现有总线锁完成指针写、STOP、读取，符合MBR数据指针设置语义。SYNC只证明差值组一致，**不保证另读BUTTON_STAT与它来自同一扫描时刻**。

当前三芯片调用链：updateInputState → updateTouch_v1 → 原生按钮位 → MBR差值验证 → 确认/保持/拉伸/slide → 完成快照 → HID/CDC。hwVer3/4走双芯片updateTouch_v2。profile ON走mbrDistanceReadSample/RetrySample和严格读组；当前OFF首次验证与压力快照调用旧get_DIFFERENCE_COUNT_SENSOR入口。MPR使用自身滤波/基线与专属验证尺度。

源码MPR分支使用CONFIG1=0x10、CONFIG2=0x28，关闭AutoConfig，设置触摸期基线参数及启动空闲基线处理。这是**源码设置**，不是此次MPR模块的现场寄存器快照；尚未取得本次无悬空MPR的现场数据。round90e补充核查：FDLT=255的“冻结”注释不能直接作为硬件禁用跟踪的证明，AN3891定义为减慢滤波；详见[基线研究](MBR3116_round90e_故障准入修复与基线研究.md)。

## 3. 用户确认离开后的空闲测图

沿生产V1_LANE_M/E映射读取全部36个启用传感器，正序、反序各一遍。C7只改变易失SENSOR_ID，复用40ms刷新等待；没有CFG_SET、CFG_SAVE或MBR NVRAM烧写。采后四块配置逐字节一致，RAW=0、记录OFF。

- 72次观察、71次有效。0x42/CS5（第28格）第一遍明确返回 `3116 debug fail`，第二遍有效；错误保留，未填零。
- 32个实际通道63/64次有效，每格至少一次。CP **11–33pF**，两遍有效的格数值一致；有效DIFF全部0，最大|RAW−BASE|=2。
- 0x42/CS8..11启用但未映射32格，测得4pF，单列保留。仅凭4pF不能确认悬空引脚或认定隔空根因，也未据此改表。
- 这是顺序诊断；整数pF、主机事务耗时和SYNC不能当全板瞬时电容、精确距离或独立扫描频率。

| 格号 | 奇数格CP/pF | 偶数格CP/pF |
|---|---|---|
| 1/2 | 11 | 24 |
| 3/4 | 14 | 24 |
| 5/6 | 15 | 24 |
| 7/8 | 18 | 23 |
| 9/10 | 32 | 23 |
| 11/12 | 33 | 24 |
| 13/14 | 32 | 25 |
| 15/16 | 31 | 25 |
| 17/18 | 30 | 24 |
| 19/20 | 30 | 25 |
| 21/22 | 29 | 25 |
| 23/24 | 27 | 24 |
| 25/26 | 28 | 22 |
| 27/28 | 27 | 22 |
| 29/30 | 26 | 21 |
| 31/32 | 24 | 20 |

映射内通道均在AN90071按钮CP推荐范围5–45pF内；本次不支持“实际按钮静态CP全部超量程”，仍不能排除模块供电、CMOD、耦合、噪声和调谐问题。[AN90071按钮设计参数表](https://assets.infineon.com/is/content/infineon/infineon/row/public/documents/30/42/infineon-an90071-cy8cmbr3xxx-capsenser-design-guide-applicationnotes-en.pdf)

## 4. 历史反例回放

只回放旧确认动作，输出新目录，原始哈希不变；没有新采同类姿势或改参数。

| 材料 | 本轮结果 | 约束 |
|---|---|---|
| round89x瞬时向量 | 3个确认动作；触摸frame154223、食指悬空frame163457同为仅第17格255 | 仅该帧32格DIFF不能严格区分 |
| round89z空间规则 | 5组、7227合格帧，四指反例保留 | 平顶/宽度/曲率直接拒绝误伤合法多押 |
| round90b冻结时间规则 | 两次慢触首次ON速度0.9375/0.8823529均被1.5规则拒绝；两次悬空无final ON不可评估；两次中止排除保留 | 不换门限重算通过，不把无ON窗口算过滤成功 |

相同瞬时向量不证明所有历史联合判据无解，下一候选必须明确新增可分信息。帧数不是独立动作数，本轮不推算识别准确率。

## 5. 最小读取修复及剩余故障语义

旧入口直接读0xBA起32B，不检查SYNC，传输失败后仍解码未完整接收的局部缓冲。游戏调用方虽判断返回码，仍缺少读组一致性保障；压力快照也走同入口。这是实际调用缺陷，不能称其解释了已有合格原生悬空ON。

现复用 `readDifferenceCounts(resultBuffer, nullptr, true)`：完整传输且SYNC一致才更新缓冲，保留0成功/非0失败，失败不发布新差值。singleAttempt=true避免叠加读组重试：游戏调用方最多两组，每组最多三次wake NACK尝试，压力快照只一组。判据、配置结构、NVRAM、SDK、vendored库未改。

正常多3B，400kHz下额外线传输约67.5µs/被读取芯片（源码推算，**未测最坏端到端延迟**）。底层读deadline从8.2增至8.5ms，单次写/读deadline合计约13.6ms，不含跨核锁等待，不能当整帧上限。

**残留**：profile OFF旧验证中两次差值读取都失败时仍保留原生触摸位，后续确认计数推进，连续失败可能形成HID ON。本次SYNC失败也进入这条已有路径。压力快照失败保留旧值；其历史“32B”注释尚未更新。下一次状态机改动需分别定义“禁止故障形成新ON”和“已ON短故障保持/释放”，不能简单清零造成长按/连打断触。

## 6. 验证、部署与下一批计划

| 检查 | 结果 |
|---|---|
| 迁移测试，改前/改后 | 各138/138 |
| 生产驱动，原测试 | 改前21/21 |
| 六个新增回归在旧驱动 | 编译成功，运行在寄存器/长度断言失败；日志保留 |
| 修复后生产驱动 | 27/27；含旧入口成功、SYNC不一致、短读、超时、三次NACK、空指针 |
| 生产管线集成 | 42/42场景、72794检查；假设备/时钟，不是真机 |
| 完成快照读取方 | 28/28场景，主机编译生产路径 |
| 只读交叉审查 | 指定diff无可操作缺陷，提示故障放行/时限边界；静态审查未独立执行上述测试 |
| SDK2.2.0构建 | 成功，UF2 196096B |
| 原目标刷机/回读 | beta9/BID round90d；控制器与0x40/41/42配置4/4逐字节保持 |
| 真机短探测 | 12.032s、239CDC帧slider全零；姿势未控制，不能当抗悬空验收 |
| 用户游戏回归 | 回复“手感正常，隔空仍存在”；定性反馈，无量化动作数/接触到HID时间 |

新UF2 SHA-256 `d39a458f35a5532f5d64d5fb42345b271411aeeca7746ed540e2ecddbdb55530`。核对序列号、RP2040 UF2地址范围/签名后刷写，镜像不覆盖配置扇区。结束RAW=0、事件记录OFF、串口关闭。短探测距离门失败计数0，但profile OFF时不覆盖旧验证读取，不能据此宣布零读取错误。

命令采用本机可用 `C:/Users/HP/AppData/Local/Python/pythoncore-3.14-64/python.exe`（当前shell的py -3不可用）；构建 `pwsh -File tools/build_firmware.ps1 -Variant hw_v1 -BuildDir build_round90d`。完整输出、原包、反例、配置、脚本、审查和两版UF2见 [核对摘要](MBR3116_round90d_证据核对.json) / [证据包](MBR3116_round90d_证据包.zip)。

下一批按此顺序执行，每项有独立进入条件：

1. **故障状态契约**：新ON必须有有效对应证据；已ON短故障保持和超时释放单独设计。先生产路径故障注入、历史逐帧比较，再修改/复核/刷机/游戏验收，保持有界重试。
2. **测量链差异**：取得MPR现场配置/代表格信号；MBR仅为新的归一化/基线假设安排固定单格RAW/BASE/DIFF/原生状态对照，优先17格及CP两端。无新假设不重复全套姿势。
3. **电气对照**：核对模块原理图、CMOD、去耦、地参考、装配；照片不能确定值。证据支持时再测，不直接启用Shield或生成NVRAM表。
4. **新算法**：实时可取得的新信息通过慢触、多指平顶、快悬空、滑动和同255反例后，才进入准入/保持/释放算法。当前不部署新距离判据，不重写芯片固化扫描，不以长等待掩盖失败。

阶段A完成首批接口核查与三类回放，基线深挖待做；B完成身份/备份/空闲测图，现场MPR对照未完成；C完成一个生产读取入口修复，故障契约待做；本修复有用户游戏反馈，D/E抗悬空候选及全面验收仍未完成。仅主维护变体本地提交，不push。
