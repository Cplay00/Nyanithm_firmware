# round90b 记录边沿修复

2026-10-02；基底e6da844。修复诊断记录器，保留round89y游戏判定，未改I2C、NVRAM、128B配置或协议长度。

## 三组动作的核查

用户完成慢轻触、快速真实点按、快速食指悬空三组，每组三次；三个文件均completed/user_confirmed/raw_restored=true、mock=false。九个窗口共864帧，248B原包的session、frameId、DIFF、slider、native/verified均与解码一致，各96帧连续，443–498ms。

但是九次reason都为MANUAL、triggerFrame/triggerMs为0，没有自动冻结。动作开始后约3s才手动冻结，只留下末尾，不能据此计算首次ON斜率、先后顺序或悬空成功抑制率。第一组第一次尾窗还有68帧输出ON；其余尾窗全OFF不表示动作期间没有触发。动作确认保留，排除的是窗口用途，不作废用户动作。

实际producer `mbrDistanceReadAttempt()`对count<=255置rangeMask位；故FFFF表示16个电极都在合法范围，256会清相应位。失败读取保留上次数据/掩码并清VALID。864帧三片mask全部FFFF，按错误条件没有任何可触发帧。

round90a误用`rangeMask!=0`作异常，加上单测fixture缺省mask=0，没有贯穿实际producer语义；只读复核也未发现此项。原始文件及标签不改写。审计见`_dev_tools/round90a_events_20261002/capture_audit.json`，采后控制器/三片逐字节保持、RAW0。

## 修复

诊断在预热后记录native/verified/final任一变化，完整保留失败、缓存和成功帧。质量审核在离线执行，记录成功不能当作接触分界或合法动作证据。删除记录器对rangeMask/逐片有效性的过滤，保留180ms预热、160ms后史、96帧容量、10s退出及SDK跨核/断连机制。新增简注明确1=该电极count<=255。

主机严格门禁更新beta8/BID round90b。每次采后检查自动EDGE、triggerFrame在窗口内、对应时间一致、前史>=180ms及后史>=160ms；缺边沿或短窗口立即保存证据并停止本组，不继续后两次，不显示成功。新增单次快速点按入口，先验证采集链再安排三组补采。

## 验证与设备

- 新单测提取真实`mbrDistanceReadAttempt()`，核对FFFF、256→FF7F、恢复255→FFFF；其生成的质量进入实际记录器。覆盖缓存变化和无效边沿保留，SDK真实队列/实际CDC命令块及200跨核会话通过，本次1579断言（并发调度会改变计数）。
- 用e6da844原记录器运行扩展测试，明确在`state==POST`断言失败；相同扩展测试在修复版通过。旧失败和新通过日志分存，未把预期失败算成功用例。
- 迁移前后138/138、driver21/21、readers28/28、生产管线42/42及72794断言通过。独立只读复核沿producer→C8→记录器完成，无新阻塞缺陷。
- SDK2.2.0构建beta8/BID round90b；UF2 **196096B**、SHA256 `2c59f8be0289a2bacbca40a7fadc59d83e32d48f76a278c9e7a0667dd2335d18`，text102060B、bss10364B。队列仍运行时23,280B。
- 已刷USB序列号5303284739002C9C，控制器及0x40/41/42逐字节保持，PROFILE仍`214,1,1,0,255,70,50,250`，RAW0。刷前源码/UF2及配置备份在`_dev_archive/round90b_before_20261002/`，另有刷前完整读回。
- 真机预检：默认OFF，连续96帧手动读回，窗口512ms；断连确认FROZEN。6s记录关闭平均5085.21us、开启5226.89us，整窗ARMED；局部hook平均21.72us、最大85us。姿势未控制、短窗口，约142us差不能当纯记录器开销或零延迟保证，未据此宣称隔空改善。
- 主机缺边沿、窗口不足均验证只执行第一次后停止；确认保留原始包。单次提示页通过Edge/bundled Playwright桌面1280×900和手机390×844，非空、无溢出/控制台错误，开始→确认及中止通过。Browser插件不可用。

自动边沿预检和三组九次补采已完成、动作已确认：预检96帧，前298ms/后160ms；补采864帧，前304–333ms/后161–164ms，全部自动EDGE且窗口完整。九次目标格全帧合格，847帧全32格合格。补采重做因果特征分析，详见`MBR3116_round90b_时间判据验证.md`。尚未部署抗悬空规则，HID游戏手感未重新验收。

## 证据

`_dev_tools/round90b_bug_reproduction.txt`、`round90b_history_tests.txt`、`round90b_sampler_tests.txt`、`round90b_build.txt`、`round90b_pipeline_tests/`、`round90b_flash_20261002/`、`round90b_preflight_20261002/`、`round90b_ui_check.json`；一次实机检查入口8774，数据目录`round90b_sanity_20261002/`。

本地提交，不push。
