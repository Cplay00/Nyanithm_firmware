/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */


#include <cy8cmbr3116.h>
#include <cy8cmbr3116_cfg.h>

#include <hardware/watchdog.h>
#include <i2c_port.h>
#include <pico/stdlib.h>
#include <production_mode.h>
#include <stdio.h>
#include <tusb.h>

// round88b: 配置区单寄存器读取统一走 i2c_write_read(原子 write+restart+read
// 组合事务, 末尾产生 STOP -- 与 verify_cy8cmbr3116_burn/0xC6 读回/驱动数据路径
// 同形, round75/80 真机验证过的形状)。1.6.5 首版烧录轮询手写的
// write(nostop=true)+read(nostop=true) 组合在整个会话中从不产生 STOP,
// 真机实测 CTRL_CMD 轮询永远读不到有效值 -> 误判 timeout -> 不发软复位,
// 新配置停在 RAM、触摸行为错乱。注意: i2c_read 的末参是 SDK 的 nostop,
// 不是"阻塞"语义。
static bool mbrReadReg(uint8_t addr, uint8_t reg, uint8_t* out) {
    return i2c_write_read(0, addr, &reg, 1, out, 1) == 1;
}

bool detect3116(uint8_t addr) {
    // round88b: 读形状统一为组合事务(原 write(nostop=true)+read(nostop=true)
    // 从不产生 STOP, 与新烧录序列同一误用源)。
    uint8_t a = 0;
    if (!mbrReadReg(addr, I2C_ADDR_ADDRESS, &a) || a != addr) {
        return false;
    }
    return true;
}

void program_cy8cmbr3116_custom(uint8_t addr, uint8_t* cfg) {
    // round88 烧录序列修正(逐条对标 TRM):
    // - TRM §1.5.80 CTRL_CMD: 设备在启动和命令完成时把该寄存器清零, 主机只能
    //   在其值为 0 时写入; 非零时写入行为未定义。旧代码连写两次 0x02 属违例,
    //   改为单次写入 + 轮询 0x86 归零判完成(官方语义, 取代盲等)。
    // - TRM §1.5.81/§1.5.82: 完成后读 0x88 bit0 判 ERR(0=成功); 出错再读
    //   0x89 取错误码(0x00 成功 / 0xFD Flash 写失败 / 0xFE 配置 CRC 不符 /
    //   0xFF 命令无效)。出错绝不发 0xFF 软复位, 避免芯片带着坏配置复位。
    // - Datasheet 时序参数: NVM 写入命令执行最长 ~220ms, 轮询上限 500ms 覆盖;
    //   期间泵 tud_task + watchdog(本函数在配置模式 Core0 调用, 看门狗无人喂)。
    // - 软复位(0xFF)后芯片需重新启动(读 Flash/CRC/校准基线), I2C 短暂 NAK;
    //   轮询探测恢复后再返回, 后续 verify 读回不再撞首击 NAK。
    for (uint8_t i = 0; i < 128; i++) {
        uint8_t buf[2] = { i, cfg[i] };
        i2c_write(0, addr, buf, 2, true);
    }
    // round88b: 前置检查 CTRL_CMD==0(TRM §1.5.80: 非零时写入行为未定义)。
    if (!mbrReadReg(addr, CTRL_CMD_ADDRESS, &cur)) {
        printf("CTRL_CMD readback failed, burn aborted.\n");
        return;
    }
    if (cur != 0) {
        // 前一命令未完成, 此刻写 0x86 行为未定义(TRM §1.5.80) -- 放弃本次烧录
        printf("CTRL_CMD busy (0x%02X), burn aborted.\n", cur);
        return;
    }
    uint8_t buf[2] = { CTRL_CMD_ADDRESS, 0x02 };  // 校验 CRC 并保存配置到 NVM
    i2c_write(0, addr, buf, 2, true);
    bool done = false;
    for (int i = 0; i < 50 && !done; i++) {  // 50 x 10ms = 500ms 上限
        tud_task();
        watchdog_update();
        sleep_ms(10);
        // round88b: TRM §1.5.80 -- 命令完成时设备把 0x86 清零; 轮询读回为 0
        // 即完成。读形状必须是 write(1B, 带 STOP)+read 组合事务(mbrReadReg),
        // 读不到寄存器值与读到非零值同等视为"未完成"。
        if (mbrReadReg(addr, CTRL_CMD_ADDRESS, &cur) && cur == 0) {
            done = true;
        }
    }
    if (!done) {
        printf("CTRL_CMD timeout, burn aborted.\n");
        return;
    }
    uint8_t status = 0;
    if (!mbrReadReg(addr, 0x88, &status)) {
        printf("CTRL_CMD_STATUS readback failed, burn aborted.\n");
        return;
    }
    if (status & 0x01) {  // TRM §1.5.81: bit0 ERR
        uint8_t err = 0;
        mbrReadReg(addr, 0x89, &err);  // TRM §1.5.82: 0xFD flash / 0xFE CRC / 0xFF invalid
        printf("burn error: status=0x%02X err=0x%02X%s%s\n", status, err,
               err == 0xFD ? " (flash write failed)" : "",
               err == 0xFE ? " (config CRC mismatch)" : "");
        return;  // 出错不发软复位
    }
    buf[1] = 0xFF;  // 软复位, 新配置生效
    i2c_write(0, addr, buf, 2, true);
    // 复位后启动延时: 轮询探测 I2C 恢复(最长 200ms), 期间泵 USB/看门狗
    for (int i = 0; i < 20; i++) {
        tud_task();
        watchdog_update();
        sleep_ms(10);
        uint8_t a = 0;
        if (mbrReadReg(addr, I2C_ADDR_ADDRESS, &a) && a == addr) {
            break;
        }
    }
}

// round75: post-burn read-back for the panel 0xBA path. After the soft
// reset the chip needs a moment before I2C answers again; poll briefly,
// then compare the THRESHOLD register (0x0C) with what was written.
// Diagnostic only -- the caller has already printed "done", so a skipped
// verify must not look like a failed burn.
void verify_cy8cmbr3116_burn(uint8_t addr, uint8_t* cfg) {
    for (int attempt = 0; attempt < 30; attempt++) {
        tud_task();  // keep the USB stack serviced while we wait
        watchdog_update();
        uint8_t reg = 0x0C;
        uint8_t val = 0;
        if (i2c_write(0, addr, &reg, 1, true) == 1 &&
            i2c_read(0, addr, &val, 1, false) == 1) {
            if (val == cfg[0x0C]) {
                printf("verify ok\n");
            } else {
                printf("verify mismatch: reg 0x0C wrote 0x%02X read 0x%02X\n", cfg[0x0C], val);
            }
            return;
        }
        sleep_ms(10);
    }
    printf("verify skipped (chip busy)\n");
}

// round80: read back the chip's current 128-byte config block (0x00-0x7F)
// for the panel 0xC6 "read from chip" path. Chunked 4x32B reads with an
// explicit register-pointer write per chunk -- the write-with-stop then
// 32B-read transaction shape is the one already proven on-device by
// verify_cy8cmbr3116_burn (round75), and a single 128B burst is untested
// on this chip family. USB + watchdog are pumped between chunks because
// this runs on Core0 inside config-mode handleCommand (updateInputState
// is not feeding the dog there).
// Returns false on any I2C failure; the caller answers with a text line.
bool read_cy8cmbr3116_config(uint8_t addr, uint8_t* cfg) {
    for (uint8_t chunk = 0; chunk < 4; chunk++) {
        tud_task();
        watchdog_update();
        uint8_t reg = chunk * 32;
        if (i2c_write(0, addr, &reg, 1, true) != 1) return false;
        if (i2c_read(0, addr, cfg + chunk * 32, 32, false) != 32) return false;
    }
    return true;
}

void program3116() {
    while (1) {
        tud_task();  // round51: Core0 owns the USB stack in production mode
        printf("\n");
        if (detect3116(0x43) && detect3116(0x44)) {
            printf("+-----------------------------+\n");
            printf("|both 0x43 and 0x44 are ready.|\n");
            printf("+-----------------------------+\n");
        } else if (detect3116(0x43)) {
            if (detect3116(0x37)) {
                printf("programing 0x44.\n");
                program_cy8cmbr3116_custom(0x37, cy8cmbr3116_cfg_0x44);
            } else {
                printf("waiting for the SECOND chip.\n");
            }
        } else if (detect3116(0x37)) {
            printf("programing 0x43.\n");
            program_cy8cmbr3116_custom(0x37, cy8cmbr3116_cfg_0x43);
        } else {
                printf("waiting for the FIRST chip.\n");
        }
        sleep_ms(500);
    }
}

void productionMode() {
    program3116();
}
