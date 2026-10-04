# 27 寸 MBR3116 启动恢复

本机已刷入 `Nyanithm_27_MBR_1.6.5_bootfix_round90r.uf2`，版本 `1.6.5-bootfix` / `round90r-1.6.5`，大小 175104B。

SHA256：`28b1a95443e4310cad95266287dce47779bb00c29c65b79259a498b532f52e0f`。

`Nyanithm_beta1_bootfix_round90r.uf2` 和 `Nyanithm_beta14_round90r_not_deployed.uf2` 为其他基线的编译产物，未部署。本机触摸管线保持最后实际使用的 1.6.5 基线；硬件选择已修正为 MBR，配置保存与重启回读一致。

详见 `../../docs/round90r_27寸MBR换装启动恢复.md`。本目录 JSON 保存刷前、刷后、配置回读、原生表保持与离线测试证据；UF2 不包含末配置扇区。完整 Flash 镜像未能读取，已有回退 UF2、控制器配置及三片表备份。
