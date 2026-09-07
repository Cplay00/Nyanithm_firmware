/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */


#include <cy8cmbr3116.h>
#include <cy8cmbr3116_cfg.h>

#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <production_mode.h>
#include <stdio.h>
#include <tusb.h>

bool detect3116(uint8_t addr) {
    // 尝试读取地址寄存器值
    uint8_t buf[1] = { I2C_ADDR_ADDRESS };
    i2c_write(0, addr, buf, 1, true);
    buf[0] = 0;
    i2c_read(0, addr, buf, 1, true);
    if (buf[0] == addr) {
        return true;
    }
    return false;
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
    uint8_t reg = CTRL_CMD_ADDRESS;
    uint8_t cur = 0;
    if (i2c_write(0, addr, &reg, 1, true) == 1) {
        i2c_read(0, addr, &cur, 1, true);
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
        reg = CTRL_CMD_ADDRESS;
        if (i2c_write(0, addr, &reg, 1, true) == 1 &&
            i2c_read(0, addr, &cur, 1, true) == 1 && cur == 0) {
            done = true;
        }
    }
    if (!done) {
        printf("CTRL_CMD timeout, burn aborted.\n");
        return;
    }
    uint8_t status = 0;
    reg = 0x88;  // CTRL_CMD_STATUS: bit0 ERR
    if (i2c_write(0, addr, &reg, 1, true) == 1 &&
        i2c_read(0, addr, &status, 1, true) == 1 && (status & 0x01)) {
        uint8_t err = 0;
        reg = 0x89;  // CTRL_CMD_ERR
        i2c_write(0, addr, &reg, 1, true);
        i2c_read(0, addr, &err, 1, true);
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
        reg = I2C_ADDR_ADDRESS;
        uint8_t a = 0;
        if (i2c_write(0, addr, &reg, 1, true) == 1 &&
            i2c_read(0, addr, &a, 1, true) == 1 && a == addr) {
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
