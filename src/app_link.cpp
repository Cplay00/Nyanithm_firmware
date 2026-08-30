/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <app_link.h>
#include <button.h>
#include <chuni_io.h>
#include <hardware/watchdog.h>
#include <production_mode.h>
#include <boot_mode.h>
#include <controller_config.h>
#include <gpio_def.h>
#include <hw_devices.h>
#include <i2c_port.h>
#include <nyanithm_shared.h>
#include <pca954x.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <tusb.h>

bool hid_working = true;

extern bool useMuxScan;  // from hw_devices.cpp

void getStatus() {
    // round46e: config-mode I/O restored to stdio (round45- era mechanism).
    // Core1 keeps running tud_task() in config mode, which is exactly how the
    // pre-round45 firmware worked (maindev_loop on Core0 + stdio_cdc).
    printf("Nyanithm build " __DATE__ " " __TIME__ "\n");
    printf("hwVer: %d\n", ControllerConfig.hwVer);
    if (ControllerConfig.hwVer == 1 || ControllerConfig.hwVer == 3) {
        printf("original ToF distance (or error code):\n    tof0 = %d tof1 = %d tof2 = %d  tof3 = %d\n", heightDataOriginal[0], heightDataOriginal[1], heightDataOriginal[2], heightDataOriginal[3]);
        printf("using config in page %d\n", getConfigPage());
        for (int i = 0; i < 4; i++) {
            if (heightDataOriginal[i] > 3000 && heightDataOriginal[i] != 8190) {
                printf("*********************\n");
                printf("ToF sensor #%d warning!\n", i);
                printf("*********************\n");
            }
        }
    }
    if (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) {
        printf("original ToF distance (or error code):\n    tof0 = %d tof1 = %d tof2 = %d  tof3 = %d tof4 = %d\n", heightDataOriginal[0], heightDataOriginal[1], heightDataOriginal[2],
               heightDataOriginal[3], heightDataOriginal[4]);
        printf("using config in page %d\n", getConfigPage());
        for (int i = 0; i < 5; i++) {
            if (heightDataOriginal[i] > 3000 && heightDataOriginal[i] != 8190) {
                printf("*********************\n");
                printf("ToF sensor #%d warning!\n", i);
                printf("*********************\n");
            }
        }
    }
}


// round75: bounded CDC payload read for config-mode commands. The old
// 0xBA/0xBB handlers used unbounded getchar(), which never pumps
// tud_task(): once the RX FIFO drained mid-payload (host stall, or the
// [64][64][x] packet split NAKing against the single-buffer OUT endpoint)
// Core0 wedged, updateInputState() stopped feeding the 2s hardware
// watchdog and the board rebooted under the host. Same mechanism as
// setConfig(): tud_cdc_read() plus an explicit tud_task() pump and
// watchdog_update() every iteration, bounded by a total deadline.
static bool readCdcPayload(uint8_t* buf, int len, uint32_t timeout_ms) {
    uint32_t totalStart = to_ms_since_boot(get_absolute_time());
    int count = 0;
    while (count < len) {
        tud_task();
        watchdog_update();
        if (tud_cdc_available()) {
            int n = tud_cdc_read(&buf[count], (uint32_t)(len - count));
            if (n > 0) count += n;
        } else if (to_ms_since_boot(get_absolute_time()) - totalStart > timeout_ms) {
            printf("read timeout at byte %d of %d. payload aborted.\n", count, len);
            return false;
        }
    }
    return true;
}

void handleCommand() {
    // round46f: config-mode idle auto-exit. If no CDC command arrives for
    // 60s (e.g. ConfigApp closed/disconnected without sending CMD_EXIT),
    // reboot back to normal mode so the game input never stays locked out.
    uint32_t lastCmdMs = to_ms_since_boot(get_absolute_time());
    // round46g: command log for ConfigApp disconnect forensics.
    // Records every byte received while in config mode; host reads it back
    // with the diagnostic command 0xCE (ConfigApp never sends 0xCE).
    static uint8_t  cfgCmdLog[256];
    static uint16_t cfgCmdLogWr = 0;
    static uint16_t cfgCmdLogTotal = 0;
    while (true) {
        while (tud_cdc_available() == 0) {
            // round51: Core0 owns tud_task() in config mode -- pump the USB
            // stack while waiting so CDC RX is serviced (fixes round46c-style
            // starvation without re-introducing the cross-core tud_task race).
            tud_task();
            sleep_ms(1);
            updateInputState();
            if (to_ms_since_boot(get_absolute_time()) - lastCmdMs > 60000) {
                reboot();
            }
        }
        uint8_t cmd = getchar();
        lastCmdMs = to_ms_since_boot(get_absolute_time());
        cfgCmdLog[cfgCmdLogWr & 0xFF] = cmd;
        cfgCmdLogWr++;
        cfgCmdLogTotal++;
        // round78b: stateful flashing confirm, shared with cdc_respond
        // (chuni_io.cpp). The panel runs in Chrome, whose serial stack holds
        // the trailing 0xA5 write for seconds (no flush semantics; real-device
        // repro: the confirm never landed within the old readCdcPayload(5s)
        // window). 0xBB only arms a 10s window; the next 0xA5 -- whenever it
        // lands -- boots, a wrong byte or expiry denies once. round51's
        // stray-0xBB protection is unchanged in strength.
        if (flashingArmed && to_ms_since_boot(get_absolute_time()) - flashingArmedAt > 10000) {
            flashingArmed = false;
            printf("flashing denied\n");
        }
        if (flashingArmed) {
            if (cmd == 0xA5) {
                flashingArmed = false;
                boot_flashing();  // returns only on safe-erase failure (stays alive)
                continue;
            }
            flashingArmed = false;
            if (cmd == CMD_FLASHING) {
                flashingArmed = true;  // host retry: re-arm, consume
                flashingArmedAt = to_ms_since_boot(get_absolute_time());
                continue;
            }
            printf("flashing denied\n");  // byte consumed, matching the old confirm read
            continue;
        }
        if (cmd == CMD_FLASHING) {
            // round51: confirmation byte required -- round78b arms the stateful
            // window above instead of blocking in readCdcPayload.
            flashingArmed = true;
            flashingArmedAt = to_ms_since_boot(get_absolute_time());
            continue;
        } else if (cmd == CMD_DEV_DETECT) {
            putchar(CMD_DEV_DETECT);
        } else if (cmd == CMD_CFG_READ) {
            readConfig();
            uint8_t* ptr = (uint8_t*)&ControllerConfig;
            for (int i = 0; i < sizeof(controller_config); i++) {
                putchar(*ptr);
                ptr++;
            }
            printf("read...\n");
            printf("using config in page %d\n", getConfigPage());
        } else if (cmd == CMD_CFG_ERASE) {
            printf("erase...\n");
            eraseConfigSector();
        } else if (cmd == CMD_CFG_SET) {
            setConfig();
        } else if (cmd == CMD_CFG_SAVE) {
            printf("save...\n");
            saveConfig();
        } else if (cmd == CMD_GET_STATUS) {
            printf("get status...\n");
            getStatus();
        } else if (cmd == CMD_EXIT) {
            reboot();
        } else if (cmd == CMD_LOAD3116CONFIG) {
            // round75: bounded payload read + address whitelist + post-burn
            // read-back. The old unbounded getchar() loop rebooted the board
            // whenever the payload stalled, and a misaligned command stream
            // could reach the programmer with an arbitrary I2C address.
            uint8_t cfg[129];
            if (readCdcPayload(cfg, (int)sizeof(cfg), 1000)) {
                uint8_t address = cfg[0];
                if (address == 0x37 || (address >= 0x40 && address <= 0x44)) {
                    printf("programing 3116 chip\n");
                    program_cy8cmbr3116_custom(address, &cfg[1]);
                    printf("done\n");
                    verify_cy8cmbr3116_burn(address, &cfg[1]);
                } else {
                    printf("load3116: address 0x%02X not allowed. burn aborted.\n", address);
                }
            }
        } else if (cmd == CMD_DETECT) {
            // round52: read-only hardware identity probe so the host control
            // panel can gate on real hardware instead of config-file values
            // (hwVer / CFG0_BIT_MBR3116 can be written wrongly; this cannot).
            // 6-byte binary response:
            //   [0]=0xBC sync  [1]=mprMask  [2]=mbrMask  [3]=tofCount  [4]=flags  [5]=config hwVer
            // mprMask: bit0..2 = MPR121 present at 0x5A..0x5C (i2c0)
            // mbrMask: bit0=0x37, bit1..3=0x40..0x42, bit4/5=0x43/0x44 (i2c0)
            // flags:   bit0 = PCA9545 mux present (i2c1 0x70), bit1 = useMuxScan
            // Absent addresses cost one 10ms findI2CDevice timeout each, so the
            // full probe may take ~150ms on sparsely populated hardware.
            // round76c: worst case is ~1.7s (10 retry iterations x ~11ms per
            // absent probe, x 10 probes, + mux channel scan) -- right against
            // the 2s watchdog, and updateInputState() is NOT running while
            // handleCommand blocks here. A slow bus crossed the line and
            // watchdog-reset the device mid-probe (real-device repro). Pump
            // the dog (and USB) between probes.
            uint8_t mprMask = 0;
            for (int i = 0; i < 3; i++) {
                tud_task();
                watchdog_update();
                if (findI2CDevice(0, 0x5A + i)) mprMask |= (1 << i);
            }
            static const uint8_t mbrAddrs[6] = { 0x37, 0x40, 0x41, 0x42, 0x43, 0x44 };
            uint8_t mbrMask = 0;
            for (int i = 0; i < 6; i++) {
                tud_task();
                watchdog_update();
                if (findI2CDevice(0, mbrAddrs[i])) mbrMask |= (1 << i);
            }
            watchdog_update();
            bool muxPresent = findI2CDevice(1, 0x70);
            uint8_t flags = (muxPresent ? 0b00000001 : 0) | (useMuxScan ? 0b00000010 : 0);
            uint8_t tofCount = 0;
            if (muxPresent) {
                PCA954X mux(1, 0x70, GPIO_PCA9545_RESET);
                if (!useMuxScan) {
                    // Independent-address mode: enable all 5 mux channels so a
                    // 5th ToF is reachable even when the config says 27", then
                    // restore the mask initToF established (see checkToF).
                    mux.setReg(0x1F);
                    for (int i = 0; i < 5; i++) {
                        tud_task();
                        watchdog_update();
                        if (findI2CDevice(1, 0x30 + i)) tofCount++;
                    }
                    mux.setReg((ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 0x1F : 0x0F);
                } else {
                    // Mux-scan fallback: every sensor keeps 0x29 behind its own
                    // channel. updateAir switches channels per cycle, so leaving
                    // the mux on the last probed channel is fine (see checkToF).
                    for (int ch = 0; ch < 5; ch++) {
                        tud_task();
                        watchdog_update();
                        mux.setChannel(ch);
                        if (findI2CDevice(1, 0x29)) tofCount++;
                    }
                }
            }
            watchdog_update();
            putchar(CMD_DETECT);
            putchar(mprMask);
            putchar(mbrMask);
            putchar(tofCount);
            putchar(flags);
            putchar(ControllerConfig.hwVer);
            // round63: 探测帧是纯二进制、无换行结尾。stdio_cdc 按驱动逐次
            // flush,但为稳妥,显式刷出,确保单发 0xBC(面板保存前存活性探测用
            // 0xB8 不依赖此,但任何独立发送 0xBC 的宿主都能立即收到完整帧)。
            stdio_flush();
        } else if (cmd == CMD_CFG_KEEPALIVE) {
            // round57: 配置模式心跳。面板每 30s 发送一次,仅重置 60s 自动退出
            // 计时器(lastCmdMs 已在 getchar() 后更新),不回显任何字节,避免
            // 污染面板的文本/字节响应流。旧固件收到 0xC3 会回 "unknown
            // command",面板侧需容忍(见面板心跳注释)。
        } else if (cmd == 0xCE) {
            // round46g: dump config-mode command log: [1B len][len bytes oldest->newest]
            uint16_t n = (cfgCmdLogTotal < 256) ? cfgCmdLogTotal : 256;
            if (n > 255) n = 255;
            putchar((uint8_t)n);
            uint16_t start = cfgCmdLogWr - n;
            for (uint16_t i = 0; i < n; i++) {
                putchar(cfgCmdLog[(start + i) & 0xFF]);
            }
        } else {
            printf("unknown command...\n");
        }
    }
}
