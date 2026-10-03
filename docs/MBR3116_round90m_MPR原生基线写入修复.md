# round90m：撤除 MPR121 非原生周期基线写入

2026-10-04，阶段B第一项，beta12/BID round90m。仅构建，未刷机，未占用设备。

NXP [MPR121手册 §5.1](https://www.nxp.com/docs/en/data-sheet/MPR121.pdf)规定，除ECR/GPIO控制外，写寄存器须处于Stop模式。原扫描尾部每500ms轮转电极，空闲条件满足时调用`writeBaselineRun()`在Run模式写0x1E–0x29。撤掉周期任务和该方法；不在游戏扫描中改成Stop→Run，因为这会暂停整片并触发CL重新装载。

[AN3891](https://www.nxp.com/docs/en/application-note/AN3891.pdf)的FDL255减慢基线跟踪，不是关闭跟踪；手册给出SFI=6、CL10/11首次电极数据装载、BVA在3:2。修正注释，保留启动校准实际指令、参数与旧CL/BVA策略，后续另行处理。13/12、去抖11、增益/滤波、正常确认/保持代码及协议结构保持原样；没有改MBR路径、SDK/vendor或配置。

此项修复源码原生遵循性，不能断言过去写入被硬件接受或造成round90l九段ON，更不代表MBR隔空已解决。停止纠偏后的长期空闲噪声、长按表现需要实机验收。

## 验证与回退

- 修改前备份：`_dev_archive/round90m_before_20261004_042009/`有clean HEAD f0fa0cb完整Git源码包、已接受beta11/round90g UF2和SHA清单。UF2为匹配本地产物、非物理Flash镜像；SHA256 `4412c8af403cba32125cc7135628b6c193d278cbfeeac944022ccfcad35f34d1`。
- 配置引用SHA `d2b2cbe5ecb3f6dc625ed1c17e0e1672ebf688a2e5d5be7db2d08209c1f05596`来自round90i，本轮未访问设备，未称实时回读。
- `tools/test_mpr_baseline_policy.py`直接编译完整生产`updateInputState()`，注入符合旧纠偏条件的模拟空闲读数：旧4/6通过，两MPR布局确实写入而失败；新6/6通过，3600轮共39624检查。IR/ToF调度、看门狗、扫描及发布顺序保持，MBR布局旁证。模拟时钟跨度不是实机长期噪声验收。
- 迁移修改前后153/153，生产触摸84/84共118538检查通过；两套生产触摸函数与前版逐字节相同，启动校准等可执行语句未变。
- SDK2.2.0独立`build_round90m`成功，UF2 199680字节，SHA256 `d9e760c66dd45645a32df2133d7dffbcd8b067b0f2f9e2ac365c57ed3b369e9c`。详情及源码/测试哈希见[核查JSON](MBR3116_round90m_验证核查.json)。

## 后续

读取有效性和故障准入另做独立提交/候选，随后准备默认关闭、有界的正常原生记录。启动校准CL/BVA策略再单独处理。固定0x5A/ELE7、相邻ELE8对照，分别验收各项，不能将合并后的差异归因单项。

全板移开、明确维护窗口后才刷/重启；关闭CDC采集后再验收游戏轻触、慢触、长按、多指、滑动和悬空。当前仍为上一轮接受版本，本轮未经新真机验证。本地提交，不push。
