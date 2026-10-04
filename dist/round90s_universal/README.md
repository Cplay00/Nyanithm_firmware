# 27 / 32 寸通用启动恢复版

两台实际已安装同一文件 `Nyanithm_27_32_universal_bootfix_round90s.uf2`。

版本 `1.6.6-beta15-bootfix` / BID `round90s-universal`，201728B；SHA256 `a87037d8f68aea9212395ea1d00c6a5ad5ba0c10a3170bf7fb568eba530df8fd`。

同一UF2支持HW1/HW2的4/5路ToF及MPR/MBR原生驱动；保留各自设备配置。32寸MBR选择已纠正，27寸MPR配置原样保留。故障告警有限，配置入口可达；运行时2秒看门狗保留。

发布源码位于本地 `round90s_universal_bootfix` 分支及 `_dev_tools/round90s_universal_source`。恢复产物来自beta11正常管线加启动补丁；主树beta14产物未部署。记录见 `../../docs/round90s_27与32寸通用启动恢复.md`。两台约60秒监测无重启；32寸近距目标测试五路测距有效、空气键有响应，早先无效状态原因未确定，游戏动作仍待验收。
