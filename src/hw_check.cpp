/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <controller_config.h>
#include <error_state.h>
#include <gpio_def.h>
#include <hardware/watchdog.h>
#include <hw_devices.h>
#include <hw_check.h>
#include <i2c_port.h>
#include <pca954x.h>

extern bool useMuxScan;  // from hw_devices.cpp

void checkToF() {
    if (usingIR) return;
    watchdog_update();
    if (!findI2CDevice(1, 0x70)) {
        error(false);
        watchdog_update();
        return;
    }
    PCA954X mux(1, 0x70, GPIO_PCA9545_RESET);

    int sensorCount = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 5 : 4;
    bool missing = false;
    if (!useMuxScan) {
        // Independent address mode: sensors at 0x30+, all mux channels enabled
        for (int i = 0; i < sensorCount; i++) {
            watchdog_update();
            if (!(g_tofReadyMask & (1u << i)) || !findI2CDevice(1, 0x30 + i)) missing = true;
        }
        // Restore mux state: all channels enabled (initToF sets this)
        uint8_t muxMask = (sensorCount == 5) ? 0x1F : 0x0F;
        mux.setReg(muxMask);
    } else {
        // Mux scan fallback: check each channel at default 0x29
        for (int i = 0; i < sensorCount; i++) {
            watchdog_update();
            if (mux.setChannel(i) != 1 || !(g_tofReadyMask & (1u << i)) ||
                !findI2CDevice(1, 0x29)) missing = true;
        }
        // Leave mux in last channel state; updateAir switches per cycle
    }
    watchdog_update();
    if (missing) warn(false);
    watchdog_update();
}

void check3116() {
    bool v2 = ControllerConfig.hwVer >= 3;
    bool missing = false;
    for (uint8_t i = 0; i < (v2 ? 2 : 3); ++i) {
        watchdog_update();
        if (!findI2CDevice(0, (v2 ? 0x43 : 0x40) + i)) missing = true;
    }
    watchdog_update();
    if (missing) warn(false);
    watchdog_update();
}

void checkMPR121() {
    bool missing = false;
    for (uint8_t i = 0; i < 3; ++i) {
        watchdog_update();
        if (!findI2CDevice(0, 0x5a + i)) missing = true;
    }
    watchdog_update();
    if (missing) warn(false);
    watchdog_update();
}

void checkHardwareState() {
    // A hardware mismatch must leave the main loop/configuration reachable.
    // The old forever-alert loops never fed the 2s watchdog and rebooted here.
    if ((ControllerConfig.cfg0 & CFG0_BIT_MBR3116) || ControllerConfig.hwVer >= 3) {
        check3116();
    } else {
        checkMPR121();
    }
    checkToF();
}
