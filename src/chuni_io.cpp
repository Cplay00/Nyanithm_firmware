/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <app_link.h>
#include <button.h>

#include <boot_mode.h>
#include <chuni_io.h>
#include <controller_config.h>
#include <gpio_def.h>
#include <hw_devices.h>
#include <nyanithm_shared.h>
#include <tusb.h>
#include <usb_device.h>
#include <stdio.h>
#include <string.h>

// round53: build variant, injected via CMake (hw_v1 / cplay). Used by the
// control panel to match firmware files against the connected device.
#ifndef NYANITHM_VARIANT
#define NYANITHM_VARIANT "unknown"
#endif

// called by usb_device.cpp to run in core #1, while main codes run in core #0
void hid_task_chuni_input() {
    if (!hid_working) {
        return;
    }

    // Poll every 1ms (phase3 step1: 2ms->1ms, reduce stale data in fast swipes)
    const uint32_t interval_ms = 1;
    static uint32_t start_ms = 0;

    if (to_ms_since_boot(get_absolute_time()) - start_ms < interval_ms)
        return;  // not enough time
    start_ms += interval_ms;

    // Remote wakeup
    if (tud_suspended()) {
        tud_remote_wakeup();
    }

    /*------------- Keyboard -------------*/
    if (tud_hid_n_ready(0)) {
        /**
         * report_buf
         * [0]
         * [1]
         * [2] A - H
         * [3] I - P
         * [4] Q - X
         * [5] Y, Z, 1 - 6
         * [6] 7 - 0, ENTER, ESCAPE, DELETE, TAB
         * [7] SPACE, -_, =+, [{, ]}, \|, [?], [?]
         */
        uint8_t report_buf[15];
        memset(report_buf, 0x00, sizeof(report_buf));

        // touch
        if (ControllerConfig.cfg1 & CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD) {
            // round46: seqlock snapshot read (eliminates torn touchData reads)
            // round46t: bounded spin (never block Core1 >5ms)
            uint32_t g;
            uint32_t spinStart = to_ms_since_boot(get_absolute_time());
            while (true) {
                do { g = touchStateGen; } while ((g & 1) && (to_ms_since_boot(get_absolute_time()) - spinStart) < 5);
                report_buf[2] = touchData[0];
                report_buf[3] = touchData[1];
                report_buf[4] = touchData[2];
                report_buf[5] = touchData[3];
                if (g == touchStateGen) break;
                // round47b-patch: on timeout, skip this HID report entirely (keep last frame)
                // instead of sending torn data that causes phantom touch / micro-dropout.
                if (to_ms_since_boot(get_absolute_time()) - spinStart >= 5) return;
            }
        }

        // air
        if (ControllerConfig.cfg1 & CFG1_BIT_ENABLE_AIR_INPUT_AS_KEYBOARD) {
            // round46: seqlock snapshot read for airKeys
            // round46t: bounded spin
            uint32_t g;
            uint32_t spinStart = to_ms_since_boot(get_absolute_time());
            while (true) {
                do { g = touchStateGen; } while ((g & 1) && (to_ms_since_boot(get_absolute_time()) - spinStart) < 5);
                report_buf[9] = 0;
                for (int i = 0; i < 6; i++) {
                    if (airKeys[i]) {
                        report_buf[9] |= (1 << i);
                    }
                }
                if (g == touchStateGen) break;
                if (to_ms_since_boot(get_absolute_time()) - spinStart >= 5) return;  // round47b-patch: skip, keep last frame
            }
        }

        if (getButtonState(BUTTON_UP)) {
            report_buf[6] |= 0b00010000;  // ENTER
        } else if (getButtonState(BUTTON_DOWN)) {
            report_buf[8] |= 0b10000000;  // F1 (bit71 -> usage 0x3B; bitmap starts at report bit 16)
        } else if (getButtonState(BUTTON_PUSH)) {
            report_buf[6] |= 0b00100000;  // ESCAPE
        }

        tud_hid_n_report(0, 0, report_buf, sizeof(report_buf));
    }
}

#define CLAMP(val, lo, hi) (val < lo ? lo : (val > hi ? hi : val))

bool game_connected;
uint32_t connected_time;

struct NyanithmInput {
    uint8_t slider[32];
    uint8_t air;
};

NyanithmInput inputState;

// round46: game-session telemetry (CMD_DEBUG_TELEMETRY = 0xC1).
// While the game runs, the serial port is exclusive to chuniio_nyanithm.dll,
// so the host cannot poll CMD_DEBUG_CHAIN. Core1 instead records every edge of
// the responses it SERVES to the DLL (plus counters). After the game closes the
// host reads the report to reconstruct what the DLL actually received.
struct NyanithmTelemetry {
    uint32_t servedCount;      // CMD_GET_INPUT served
    uint32_t edgeCount;        // edge events recorded (total)
    uint32_t cdcRxBytes;
    uint32_t cdcTxBytes;
    uint32_t edgeIdx;          // next ring slot (0..31)
    uint16_t riseCnt[32];      // per-cell rising edges seen by the DLL
    uint16_t fallCnt[32];      // per-cell falling edges seen by the DLL
    uint32_t edgeMs[32];       // firmware uptime ms at edge
    uint32_t edgeMask[32];     // changed slider cells (bit 0..31)
    uint8_t  edgeAir[32];      // air byte at edge
    uint8_t  edgeDir[32];      // 1=rise 2=fall 3=mixed
} g_tele;

static NyanithmInput prevInputState;
static uint8_t prevAirState = 0;
static uint32_t latencyFrameMs = 0;  // round55: arrival timestamp of the frame currently held back (cfg3 latency)

// round47b-patch: maindev_loop() removed (dead code, replaced by cdc_respond on Core1).
// It read touchData32 without seqlock and was never called after the round46
// Core0/Core1 split. Kept inputState/NyanithmInput definitions below for cdc_respond.

volatile bool pending_config_mode = false;
volatile bool pending_flashing = false;
// round78b: stateful 0xBB confirm window, shared by cdc_respond (normal mode,
// Core1) and handleCommand (config mode, Core0) -- the two never run at once.
volatile bool flashingArmed = false;
volatile uint32_t flashingArmedAt = 0;
volatile uint8_t flashDiagCode = 0;
volatile uint8_t flashDiagRc = 0xFF;
volatile uint32_t flashDiagGapMs = 0;
volatile bool in_config_mode = false;

// round68: game-raw baseline from cfg2 bit1 (persisted, default off). The
// active pressure report mode is rawModeActive():
//   cfg2 bit1=1  -> game-raw mode (GET_INPUT: pressed=max(1,pressure), released=0)
//   0xC5 0x01    -> panel mode (GET_INPUT: all lanes report simulated pressure)
//   0xC5 0x02    -> panel mode, raw flavor (GET_INPUT: all lanes report the
//                   unscaled physical reading; MBR lanes are identical to 0x01)
//   0xC5 0x00    -> back to the cfg2 bit1 baseline
// Binary mode (both off) keeps GET_INPUT byte-identical to 1.5.x.
// gameRawEnabled itself is defined in hw_devices.cpp (round66 0xC5-session pattern).
// round73: 基线改为每次命令逐值重算,不再一次性锁存——Core1 在 main()->initUSBDevice()
// 即启动并开始跑 cdc_respond(),早于 boot_mode 的 readConfig();原实现的首次锁存读到的
// 是全零 ControllerConfig,cfg2 bit1 在所有冷启动下永不生效(P1)。逐次求值顺带让
// 保存后的 cfg2 改动无需重启即可反映到基线;0xC5 会话(rawReportLevel)仍独立于该基线。
void updateGameRawBaseline() {
    gameRawEnabled = !!(ControllerConfig.cfg2 & CFG2_BIT_GAME_RAW_SLIDER);
}

bool rawModeActive() {
    return rawReportLevel != 0 || gameRawEnabled;
}

// CDC command responder running on Core1. Replaces the CDC portion of maindev_loop
// (which ran on Core0 and was blocked by updateInputState). Bulk tud_cdc I/O avoids
// per-byte stdio_cdc mutex overhead; responses land in ~2-3ms instead of 6ms+jitter.
void cdc_respond() {
    // In config mode, Core0's handleCommand() owns the CDC channel (getchar/putchar via
    // stdio_cdc). Core1 must NOT touch tud_cdc_read/write here (would steal CMD_GET_STATUS),
    // AND must not run the standby timeout: game_connected must stay true so LampArray
    // (Windows dynamic lighting) stays suppressed and the green config-mode LED is not
    // overwritten by the rainbow/ambient effect. Original firmware achieved this because
    // maindev_loop's timeout lived after handleCommand() which never returns.
    if (in_config_mode) {
        return;
    }
    // round68: lazily sync the game-raw baseline from cfg2 on first command.
    updateGameRawBaseline();
    // Standby timeout (normalMode only): 5s without CDC activity -> return to idle
    // (game_connected=false), re-enabling LampArray/Windows dynamic lighting.
    // round76: a host still holding the port open (DTR high; the control panel
    // asserts it explicitly on open) is not idle -- keep LampArray suppressed
    // across gaps between panel transactions, else the strip snaps back to the
    // Windows accent color (usually blue) 5s after the last 0xB2 frame. A crashed
    // host that never closes the port keeps DTR high until replug.
    if (game_connected && to_ms_since_boot(get_absolute_time()) - connected_time > 5000) {
        if (tud_cdc_connected()) {
            connected_time = to_ms_since_boot(get_absolute_time());
        } else {
            game_connected = false;
        }
    }
    if (!tud_cdc_available()) {
        return;
    }

    game_connected = true;
    connected_time = to_ms_since_boot(get_absolute_time());

    uint8_t cmd = 0;
    if (tud_cdc_read(&cmd, 1) == 0) {
        return;
    }
    g_tele.cdcRxBytes++;  // round46: telemetry - count command byte

    // round78b: stateful flashing confirm (see the CMD_FLASHING handler).
    // The panel runs in Chrome, whose serial stack holds the trailing 0xA5
    // write for seconds (no flush semantics), so the confirm must be honored
    // whenever it lands, not within a blocking window. Expiry or a wrong
    // byte denies once; a host retry re-arms. round78c: the window is 30s --
    // Chrome's hold is unbounded-ish (observed >10s) and the panel's
    // closePort flush releases the backlog, so the late confirm must still
    // land inside the window.
    if (flashingArmed) {
        if (to_ms_since_boot(get_absolute_time()) - flashingArmedAt > 30000) {
            flashingArmed = false;
            flashDiagCode = 2;  // deny: window expired
            flashDiagGapMs = to_ms_since_boot(get_absolute_time()) - flashingArmedAt;
            flashDiagRc = 0xFF;
            const char* denyMsg = "flashing denied\n";
            tud_cdc_write((const uint8_t*)denyMsg, strlen(denyMsg));
            tud_cdc_write_flush();
            g_tele.cdcTxBytes += strlen(denyMsg);
        } else if (cmd == 0xA5) {
            flashingArmed = false;
            flashDiagCode = 1;  // boot attempted (rc recorded in boot_flashing)
            flashDiagGapMs = to_ms_since_boot(get_absolute_time()) - flashingArmedAt;
            flashDiagRc = 0xFF;
            pending_flashing = true;  // Core0 normal loop: boot_flashing()
            return;
        } else if (cmd == CMD_FLASHING) {
            flashingArmedAt = to_ms_since_boot(get_absolute_time());  // host retry: re-arm
            flashDiagCode = 0;
            return;
        } else {
            flashingArmed = false;
            flashDiagCode = 3;  // deny: interloping byte
            flashDiagGapMs = to_ms_since_boot(get_absolute_time()) - flashingArmedAt;
            flashDiagRc = 0xFF;
            const char* denyMsg = "flashing denied\n";
            tud_cdc_write((const uint8_t*)denyMsg, strlen(denyMsg));
            tud_cdc_write_flush();
            g_tele.cdcTxBytes += strlen(denyMsg);
            // fall through: the interloping byte is dispatched normally
        }
    }

    if (cmd == CMD_GET_API_LEVEL) {
        uint8_t v = NYANITHM_API_LEVEL;
        tud_cdc_write(&v, 1);
        tud_cdc_write_flush();
    }
    if (cmd == CMD_DEV_DETECT) {
        uint8_t v = (uint8_t)CMD_DEV_DETECT;
        tud_cdc_write(&v, 1);
        tud_cdc_write_flush();
    }
    if (cmd == CMD_GET_VERSION) {
        // Length-prefixed ASCII info string so the host can display firmware
        // version, API level, hardware version, compile date/time and Pico SDK
        // version in one shot. Format: [1B len][len ASCII bytes].
        // round53: appended VAR + NYANFW1 signature. The signature is resolved
        // at compile time (string-literal concatenation), so the exact bytes
        // "NYANFW1;<version>;<variant>" are baked into the UF2 image inside
        // this format string AND returned live -- the control panel scans UF2
        // files for it to validate firmware version/variant before flashing.
        char info[160];
        int n = snprintf(info, sizeof(info),
            "FW:%s API:0x%02X HW:v%d Built:%s %s SDK:%d.%d.%d VAR:%s NYANFW1;" NYANITHM_FW_VERSION ";" NYANITHM_VARIANT,
            NYANITHM_FW_VERSION, NYANITHM_API_LEVEL,
            (int)ControllerConfig.hwVer,
            __DATE__, __TIME__,
            PICO_SDK_VERSION_MAJOR, PICO_SDK_VERSION_MINOR, PICO_SDK_VERSION_REVISION,
            NYANITHM_VARIANT);
        if (n < 0) n = 0;
        if (n > 255) n = 255;
        uint8_t len = (uint8_t)n;
        tud_cdc_write(&len, 1);
        tud_cdc_write((const uint8_t*)info, len);
        tud_cdc_write_flush();
    }
    if (cmd == CMD_GET_INPUT) {
        // round46: seqlock snapshot read - the DLL must never see a torn
        // mix of old/new cells (root cause of in-game phantom/dropped keys).
        // round46t: bounded spin - if Core0 wedges (WDT resets in ~2s) the
        // DLL is never blocked more than 5ms; it gets the last state instead.
        uint32_t g;
        uint8_t air = 0;
        // round66: pressure snapshot + touch bits copied in the SAME seqlock
        // scope; declared at function scope so the raw-mode substitution below
        // (after the loop) can read them.
        uint8_t tmpSlider[32];
        uint8_t tmpPressure[32];
        // round73: 仅当本轮真正提交了新帧(seqlock 成功且未被 cfg3 持帧)才做
        // raw 压力替换。超时路径若照旧替换,tmpPressure 可能是奇偶栅栏切换中途
        // 的新旧混合快照,违背"超时保留 last-good"约定;持帧路径重替换也会让
        // 已服务状态被同轮新采样覆盖。两种情况都应原样保持上一服务帧。
        bool frameFresh = false;
        uint32_t spinStart = to_ms_since_boot(get_absolute_time());
        while (true) {
            do { g = touchStateGen; } while ((g & 1) && (to_ms_since_boot(get_absolute_time()) - spinStart) < 1);
            // round47b-patch: read into temp buffer; only commit to inputState on
            // seqlock success. On timeout, keep last good inputState (no torn data).
            for (int i = 0; i < 32; i++) tmpSlider[i] = touchData32[i];
            for (int i = 0; i < 32; i++) tmpPressure[i] = pressureSnap[i];
            air = 0;
            for (int i = 0; i < 6; i++) {
                if (airKeys[i]) air |= 1 << i;
            }
            if (g == touchStateGen) {
                // round55: additive input latency (cfg3, 0-15ms). Hold each new
                // frame until nowMs >= frameFirstSeenMs + latencyMs, so state
                // younger than the configured delay is never served. Poll rate
                // is unaffected (busy callers spin at their own cadence).
                uint32_t nowMs = to_ms_since_boot(get_absolute_time());
                uint32_t latencyMs = ControllerConfig.cfg3 > INPUT_LATENCY_MAX_MS
                                         ? INPUT_LATENCY_MAX_MS
                                         : ControllerConfig.cfg3;
                bool changed = memcmp(inputState.slider, tmpSlider, 32) != 0 || inputState.air != air;
                if (changed) {
                    if (nowMs - latencyFrameMs < latencyMs) break;  // too fresh - keep serving previous frame
                    latencyFrameMs = nowMs;
                }
                memcpy(inputState.slider, tmpSlider, 32);
                inputState.air = air;
                frameFresh = true;
                break;
            }
            if (to_ms_since_boot(get_absolute_time()) - spinStart >= 1) { air = inputState.air; break; }  // round47b-patch: timeout - sync air to last good value for telemetry consistency
        }
        // inputState already committed on success; on timeout it retains last good value

        // round66/68/69: raw pressure substitution - tmpPressure was copied in
        // the same seqlock scope as touchData32. Panel mode (rawReportLevel):
        // all lanes report pressure. Game mode (cfg2 bit1 baseline):
        // pressed lanes report linear-normalized pressure (0-255 scale), released
        // lanes 0. Both off keeps the 0/128 frame byte-identical to 1.5.x.
        // round73: 追加 frameFresh 门控(见上)。
        // round80: MPR121 pressure is contact-area proportional (user-measured:
        // finger 30-50, palm ~130) and never reaches 255 natively -- report x2
        // (clamp 255) in BOTH panel real-pressure and game raw modes so the
        // reported scale stays linear up to the game cap. The old game-mode x4
        // saturated even medium touches and destroyed the area linearity.
        // MBR3116 DIFFERENCE_COUNT is natively 0-255 -- untouched. Scaling
        // happens only at this reporting layer; pressureSnap / 0xC2 debug
        // still carry raw values.
        // round84: 0xC5 grew a level 2 = raw flavor: pressureSnap is reported
        // unscaled (no MPR x2) so the panel can show the physical sensor
        // reading; level 1 keeps the round80 simulated-report semantics.
        if (rawModeActive() && frameFresh) {
            bool useMbr = (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) ||
                          ControllerConfig.hwVer >= 3;  // same rule as sanitizeConfig
            if (rawReportLevel) {
                // panel mode: level 1 = simulated (x2 clamp, round80), level 2
                // = raw (unscaled). Untouched keys report idle values either way.
                bool simScaled = (rawReportLevel == 1);
                for (int i = 0; i < 32; i++) {
                    uint16_t v = tmpPressure[i];
                    if (!useMbr && simScaled) {
                        v <<= 1;
                        if (v > 255) v = 255;
                    }
                    inputState.slider[i] = (uint8_t)v;
                }
            } else {
                // game mode: linear-normalized 0-255 pressure. Round69:
                // binary-compatible non-zero = pressed, with continuous
                // magnitude for the DLL.
                for (int i = 0; i < 32; i++) {
                    uint8_t pressed = tmpSlider[i] ? 1 : 0;
                    uint16_t v = tmpPressure[i];
                    if (!useMbr) {
                        v <<= 1;
                        if (v > 255) v = 255;
                    }
                    uint8_t p = (uint8_t)v;
                    inputState.slider[i] = pressed ? (p > 0 ? p : 1) : 0;
                }
            }
        }

        // round46: telemetry - record response edges + counters
        if (g_tele.servedCount > 0) {
            uint32_t mask = 0;
            uint8_t dir = 0;
            for (int i = 0; i < 32; i++) {
                uint8_t cur = inputState.slider[i] ? 1 : 0;
                uint8_t prev = prevInputState.slider[i] ? 1 : 0;
                if (cur != prev) {
                    mask |= (1u << i);
                    if (cur) { g_tele.riseCnt[i]++; dir |= 1; }
                    else     { g_tele.fallCnt[i]++; dir |= 2; }
                }
            }
            if (mask || (air != prevAirState)) {
                uint8_t idx = (uint8_t)(g_tele.edgeIdx & 31);
                g_tele.edgeMs[idx] = to_ms_since_boot(get_absolute_time());
                g_tele.edgeMask[idx] = mask;
                g_tele.edgeAir[idx] = air;
                g_tele.edgeDir[idx] = dir;
                g_tele.edgeIdx++;
                g_tele.edgeCount++;
            }
        }
        memcpy(&prevInputState, &inputState, sizeof(NyanithmInput));
        prevAirState = air;
        g_tele.servedCount++;
        g_tele.cdcTxBytes += sizeof(NyanithmInput);
        tud_cdc_write(&inputState, sizeof(NyanithmInput));
        tud_cdc_write_flush();
    }
    if (cmd == CMD_DEBUG_RAW) {
        // Diagnostic: read raw MPR121 data for troubleshooting dead electrodes.
        // Format: [6B touch status] [24B MPR2 filtered data] [12B MPR2 baseline]
        //         [1B MPR2 ELE0 touch_th] [1B release_th] [1B charge_curr] [1B ECR] [1B AUTOCONFIG0]
        uint16_t t0 = mpr0.touched();
        uint16_t t1 = mpr1.touched();
        uint16_t t2 = mpr2.touched();
        tud_cdc_write((uint8_t*)&t0, 2);
        tud_cdc_write((uint8_t*)&t1, 2);
        tud_cdc_write((uint8_t*)&t2, 2);
        for (uint8_t e = 0; e < 12; e++) {
            uint16_t fd = mpr2.filteredData(e);
            tud_cdc_write((uint8_t*)&fd, 2);
        }
        for (uint8_t e = 0; e < 12; e++) {
            uint8_t bl = mpr2.readRegister8(MPR121_BASELINE_0 + e);
            tud_cdc_write(&bl, 1);
        }
        uint8_t th_touch = mpr2.readRegister8(MPR121_TOUCHTH_0);
        uint8_t th_rel = mpr2.readRegister8(MPR121_RELEASETH_0);
        uint8_t cdc0 = mpr2.readRegister8(MPR121_CHARGECURR_0);
        uint8_t ecr = mpr2.readRegister8(MPR121_ECR);
        uint8_t ac0 = mpr2.readRegister8(MPR121_AUTOCONFIG0);
        tud_cdc_write(&th_touch, 1);
        tud_cdc_write(&th_rel, 1);
        tud_cdc_write(&cdc0, 1);
        tud_cdc_write(&ecr, 1);
        tud_cdc_write(&ac0, 1);
        tud_cdc_write_flush();
    }
    if (cmd == CMD_DEBUG_ALL) {
        // Full diagnostic: all 3 MPR121s. 129 bytes total.
        // Format: [6B t0,t1,t2] x3[24B filt + 12B base + 5B cfg] = 6+3*41 = 129
        MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
        uint16_t t0 = mpr0.touched();
        uint16_t t1 = mpr1.touched();
        uint16_t t2 = mpr2.touched();
        tud_cdc_write((uint8_t*)&t0, 2);
        tud_cdc_write((uint8_t*)&t1, 2);
        tud_cdc_write((uint8_t*)&t2, 2);
        for (uint8_t m = 0; m < 3; m++) {
            for (uint8_t e = 0; e < 12; e++) {
                uint16_t fd = mprs[m]->filteredData(e);
                tud_cdc_write((uint8_t*)&fd, 2);
            }
            for (uint8_t e = 0; e < 12; e++) {
                uint8_t bl = mprs[m]->readRegister8(MPR121_BASELINE_0 + e);
                tud_cdc_write(&bl, 1);
            }
            uint8_t th = mprs[m]->readRegister8(MPR121_TOUCHTH_0);
            uint8_t tr = mprs[m]->readRegister8(MPR121_RELEASETH_0);
            uint8_t cdc = mprs[m]->readRegister8(MPR121_CHARGECURR_0);
            uint8_t ecr = mprs[m]->readRegister8(MPR121_ECR);
            uint8_t ac = mprs[m]->readRegister8(MPR121_AUTOCONFIG0);
            tud_cdc_write(&th, 1);
            tud_cdc_write(&tr, 1);
            tud_cdc_write(&cdc, 1);
            tud_cdc_write(&ecr, 1);
            tud_cdc_write(&ac, 1);
            tud_cdc_write_flush();  // Flush per-MPR to avoid buffer overflow
        }
        // Final flush already done per-MPR above
    }
    if (cmd == CMD_DEBUG_CHAIN) {
        // Full chain diagnostic: raw MPR121 -> verified -> game output
        // round46b: 46 bytes: [AA][55] sync + 44B payload
        //   payload: [6B raw t0/t1/t2] [6B verified rawTouch] [32B touchData32]
        // round45p: use hwTouch snapshot instead of live I2C reads (zero overhead, no main-loop blocking)
        // round46: seqlock snapshot read so raw/verified/game are one coherent cycle
        // round46t: bounded spin (same rationale as CMD_GET_INPUT)
        uint32_t g;
        uint16_t hw[3], rw[3];
        uint8_t td[32];
        uint32_t spinStart = to_ms_since_boot(get_absolute_time());
        while (true) {
            do { g = touchStateGen; } while ((g & 1) && (to_ms_since_boot(get_absolute_time()) - spinStart) < 5);
            hw[0] = hwTouch[0]; hw[1] = hwTouch[1]; hw[2] = hwTouch[2];
            rw[0] = rawTouch[0]; rw[1] = rawTouch[1]; rw[2] = rawTouch[2];
            for (int i = 0; i < 32; i++) td[i] = touchData32[i];
            if (g == touchStateGen) break;
            if (to_ms_since_boot(get_absolute_time()) - spinStart >= 5) break;
        }
        uint8_t sync[2] = {0xAA, 0x55};
        tud_cdc_write(sync, 2);
        tud_cdc_write((uint8_t*)&hw[0], 2);
        tud_cdc_write((uint8_t*)&hw[1], 2);
        tud_cdc_write((uint8_t*)&hw[2], 2);
        tud_cdc_write((uint8_t*)&rw[0], 2);
        tud_cdc_write((uint8_t*)&rw[1], 2);
        tud_cdc_write((uint8_t*)&rw[2], 2);
        tud_cdc_write(td, 32);
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += 46;
    }
    if (cmd == CMD_DEBUG_TELEMETRY) {
        // round46: game-session telemetry report.
        // Layout (host: monitor_chain2.ps1 -Telemetry):
        //  Fixed 528-byte payload (no length prefix - too large for 1 byte):
        //  [0..3]    reportUptimeMs u32
        //  [4..7]    servedCount u32
        //  [8..11]   edgeCount u32
        //  [12..15]  cdcRxBytes u32
        //  [16..19]  cdcTxBytes u32
        //  [20..23]  loopMinUs u32
        //  [24..27]  loopMaxUs u32
        //  [28..31]  loopSumUs u32
        //  [32..35]  loopCount u32
        //  [36..39]  i2cErrCount u32 (reserved)
        //  [40..103]  riseCnt[32] u16
        //  [104..167] fallCnt[32] u16
        //  [168..203] verifyFail[36] u8
        //  [204..207] edgeIdx u32
        //  [208..527] edges[32] x {u32 ms, u32 mask, u8 air, u8 dir}
        uint8_t buf[528];
        uint32_t off = 0;
        uint32_t up = to_ms_since_boot(get_absolute_time());
        memcpy(&buf[off], &up, 4); off += 4;
        memcpy(&buf[off], &g_tele.servedCount, 4); off += 4;
        memcpy(&buf[off], &g_tele.edgeCount, 4); off += 4;
        memcpy(&buf[off], &g_tele.cdcRxBytes, 4); off += 4;
        memcpy(&buf[off], &g_tele.cdcTxBytes, 4); off += 4;
        memcpy(&buf[off], &g_loopMinUs, 4); off += 4;
        memcpy(&buf[off], &g_loopMaxUs, 4); off += 4;
        memcpy(&buf[off], &g_loopSumUs, 4); off += 4;
        memcpy(&buf[off], &g_loopCount, 4); off += 4;
        uint32_t i2cReserved = 0;
        memcpy(&buf[off], &i2cReserved, 4); off += 4;
        for (int i = 0; i < 32; i++) { memcpy(&buf[off], &g_tele.riseCnt[i], 2); off += 2; }
        for (int i = 0; i < 32; i++) { memcpy(&buf[off], &g_tele.fallCnt[i], 2); off += 2; }
        for (int i = 0; i < 36; i++) buf[off++] = g_verifyFail[i];
        memcpy(&buf[off], &g_tele.edgeIdx, 4); off += 4;
        for (int i = 0; i < 32; i++) {
            memcpy(&buf[off], &g_tele.edgeMs[i], 4); off += 4;
            memcpy(&buf[off], &g_tele.edgeMask[i], 4); off += 4;
            buf[off++] = g_tele.edgeAir[i];
            buf[off++] = g_tele.edgeDir[i];
        }
        // CDC TX buffer is small (64B) - loop-write with tud_task() pumping
        uint32_t sent = 0;
        uint32_t pumpStart = to_ms_since_boot(get_absolute_time());
        while (sent < off && (to_ms_since_boot(get_absolute_time()) - pumpStart) < 500) {
            uint32_t n = tud_cdc_write(buf + sent, off - sent);
            sent += n;
            if (n == 0) {
                tud_task();
                sleep_us(50);
            }
        }
        // round46c: 500ms bound - if the host stops reading, drop the rest
        // instead of wedging Core1 (and the whole CDC) forever.
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += off;
    }
    if (cmd == CMD_DEBUG_DIFF) {
        // round46b: compact calibration diagnostic - bulk I2C reads.
        // round46b: 44 bytes: [AA][55] sync + 42B payload
        //   payload: [6B t0,t1,t2] [36B diff clamped 0..255] (base - filt)
        // round46: 75 single-byte I2C reads (~7ms/frame); round46b: 9 bulk
        // transactions (~1ms/frame) so 5ms host-side sampling is possible.
        uint8_t buf[44];
        MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
        buf[0] = 0xAA;
        buf[1] = 0x55;
        for (uint8_t m = 0; m < 3; m++) {
            uint16_t t = mprs[m]->touched();
            buf[2 + m * 2] = (uint8_t)(t & 0xFF);
            buf[3 + m * 2] = (uint8_t)(t >> 8);
        }
        uint8_t o = 8;
        for (uint8_t m = 0; m < 3; m++) {
            uint8_t filt[24];
            uint8_t base[12];
            bool ok_filt = mprs[m]->readRegisters(MPR121_FILTDATA_0L, filt, 24);
            bool ok_base = mprs[m]->readRegisters(MPR121_BASELINE_0, base, 12);
            if (!ok_filt || !ok_base) {
                // round46i: failed bulk read - emit clean zeros for this MPR
                // (a partial read would otherwise produce false 255 spikes)
                for (uint8_t e = 0; e < 12; e++) buf[o++] = 0;
            } else {
                for (uint8_t e = 0; e < 12; e++) {
                    uint16_t f = filt[e * 2] | ((uint16_t)filt[e * 2 + 1] << 8);
                    uint16_t b = (uint16_t)base[e] << 2;
                    int16_t diff = (int16_t)b - (int16_t)f;
                    buf[o++] = (diff < 0) ? 0 : ((diff > 255) ? 255 : (uint8_t)diff);
                }
            }
        }
        tud_cdc_write(buf, sizeof(buf));
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += sizeof(buf);
    }
    if (cmd == CMD_GET_RAW_STATUS) {
        // round66: query the raw pressure report switch. Text line (style of 0xB8)
        // so the panel's waitForLine can read it. round68: reports the ACTIVE mode
        // (panel session OR cfg2 bit1 game baseline), not just the session level.
        // round84: the session level itself when one is active (RAW=1/2), else
        // the baseline (RAW=0). Level 1/2 both count as active.
        char resp[8];
        uint8_t lvl = rawReportLevel ? rawReportLevel : (gameRawEnabled ? 1 : 0);
        resp[0] = 'R'; resp[1] = 'A'; resp[2] = 'W'; resp[3] = '=';
        resp[4] = (char)('0' + lvl); resp[5] = '\n'; resp[6] = 0;
        tud_cdc_write((const uint8_t*)resp, 6);
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += 6;
    }
    if (cmd == CMD_SET_RAW_REPORT) {
        // round66: set the raw pressure report switch. Host sends 1 byte.
        // round68: 0x00 reverts to the cfg2 bit1 game-raw baseline (not a hard
        // binary mode) so a panel session on a game-raw-enabled device does not
        // permanently flip the device back to binary.
        // round84: payload is a level: 0=off, 1=simulated report (x2), 2=raw
        // report (unscaled). Unknown values keep the current state (a legacy
        // host that only ever sends 0/1 is unaffected).
        uint8_t v = 0;
        // round78: subtraction compare (wrap-safe); was a 49.7-day rollover hazard.
        uint32_t rawStart = to_ms_since_boot(get_absolute_time());
        while (tud_cdc_available() == 0 && to_ms_since_boot(get_absolute_time()) - rawStart < 100) {
            tud_task();
            sleep_us(100);
        }
        if (tud_cdc_read(&v, 1) == 1) {
            if (v == 0) rawReportLevel = 0;
            else if (v == 1 || v == 2) rawReportLevel = v;
        }
        char resp[8];
        uint8_t lvl = rawReportLevel ? rawReportLevel : (gameRawEnabled ? 1 : 0);
        resp[0] = 'R'; resp[1] = 'A'; resp[2] = 'W'; resp[3] = '=';
        resp[4] = (char)('0' + lvl); resp[5] = '\n'; resp[6] = 0;
        tud_cdc_write((const uint8_t*)resp, 6);
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += 6;
    }
    if (cmd == CMD_SET_LED) {
        uint8_t leds[96];
        uint32_t got = 0;
        // 96 bytes may arrive in multiple USB packets; loop until complete,
        // pumping tud_task() so the USB stack keeps receiving.
        // round51: bounded wait (500ms). A partial transfer (host close / unplug)
        // used to wedge Core1 here forever while Core0 kept feeding the watchdog,
        // freezing HID input until power cycle.
        // round78: subtraction compare (wrap-safe); was a 49.7-day rollover hazard.
        uint32_t ledStart = to_ms_since_boot(get_absolute_time());
        while (got < 96) {
            uint32_t r = tud_cdc_read(leds + got, 96 - got);
            got += r;
            if (got < 96) {
                tud_task();
                if (!tud_cdc_available()) {
                    sleep_us(50);
                }
                if (to_ms_since_boot(get_absolute_time()) - ledStart >= 500) {
                    return;  // stale payload bytes are dropped as unknown commands
                }
            }
        }
        if (g_lampCount < 31) {
            RGB_LED.fill(0, 0, 0, g_lampCount, 31 - g_lampCount);
        }
        // 16 灯模式: DLL 仍按 31 颗发送(含 15 颗间隙灯)，偶数索引(0,2,...,30)
        //           是 16 颗判定灯，stride=2 跳过间隙灯
        // 31 灯模式: stride=1 线性读取
        const uint8_t dll_stride = (g_lampCount == 16) ? 6 : 3;
        uint8_t r, g, b;
        for (int i = 0; i < g_lampCount; i++) {
            b = leds[i * dll_stride + 0];
            r = leds[i * dll_stride + 1];
            g = leds[i * dll_stride + 2];
            uint32_t R = (r * ControllerConfig.lightLimit) / 255;
            uint32_t G = (g * ControllerConfig.lightLimit) / 255;
            uint32_t B = (b * ControllerConfig.lightLimit) / 255;
            r = R;
            g = G;
            b = B;
            if (g_lampCount == 31 && (i & 1) && (ControllerConfig.cfg0 & CFG0_BIT_DARKER_GAP)) {
                r >>= 2;
                g >>= 2;
                b >>= 2;
            }
            RGB_LED.setColor((g_lampCount - 1) - i, r, g, b);
        }
        RGB_LED.flush();
    }
    if (cmd == CMD_CONFIG_MODE) {
        RGB_LED.fill(0x00, 0x0f, 0x00);
        RGB_LED.flush();
        // round66: pressure reporting is a session/panel feature - turn it off
        // before handing the CDC channel to Core0's config-mode command handler
        // (GET_INPUT is paused there; stale raw state must not leak back).
        // round84: session state is now the rawReportLevel (0/1/2).
        rawReportLevel = 0;
        hid_working = false;
        in_config_mode = true;  // Core1 stops reading CDC; Core0 handleCommand takes over
        // round51: Core0 becomes the sole tud_task() driver in config mode
        // (handleCommand's idle loop pumps it). Prevents the cross-core
        // tud_task() race with Core1's loop via the stdio_cdc printf/getchar path.
        core0_owns_usb = true;
        // Delegate to Core0: handleCommand() runs the config command loop with
        // watchdog feeding there (the CDC console in config mode lives on
        // Core0 by design; normal-mode 0xBB separately arms pending_flashing).
        pending_config_mode = true;
    }
    if (cmd == CMD_FLASHING) {
        // round78b: 0xBB only ARMS a 10s confirm window and returns -- no
        // blocking wait. The panel runs in Chrome, whose serial stack may
        // hold the trailing 0xA5 write for seconds (no flush semantics;
        // real-device repro: confirm never landed within the old blocking
        // windows). Whenever the 0xA5 finally lands, the armed check at the
        // top of this function honors it. round51's stray-0xBB protection
        // stays: only a 0xA5 inside the window boots, anything else denies.
        flashingArmed = true;
        flashingArmedAt = to_ms_since_boot(get_absolute_time());
        flashDiagCode = 0;
        return;
    }
    if (cmd == CMD_FLASH_DIAG) {
        // round78c: post-mortem dump for silent flashing outcomes. Binary:
        // [0xCD][code][rc][gap u32 LE]. Works in normal mode; config mode has
        // the same handler (app_link.cpp).
        uint32_t g = flashDiagGapMs;
        uint8_t frame[7] = { CMD_FLASH_DIAG, flashDiagCode, flashDiagRc,
                             (uint8_t)(g & 0xFF), (uint8_t)((g >> 8) & 0xFF),
                             (uint8_t)((g >> 16) & 0xFF), (uint8_t)((g >> 24) & 0xFF) };
        tud_cdc_write(frame, sizeof(frame));
        tud_cdc_write_flush();
        g_tele.cdcTxBytes += sizeof(frame);
    }
}
