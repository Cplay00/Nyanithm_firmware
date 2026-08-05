/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <controller_config.h>
#include <error_state.h>
#include <gpio_def.h>
#include <hw_check.h>
#include <i2c_port.h>
#include <pca954x.h>

extern bool useMuxScan;  // from hw_devices.cpp

void checkToF() {
    if (!findI2CDevice(1, 0x70)) {
        error(true);
    }
    PCA954X mux(1, 0x70, GPIO_PCA9545_RESET);

    int sensorCount = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 5 : 4;
    if (!useMuxScan) {
        // Independent address mode: sensors at 0x30+, all mux channels enabled
        for (int i = 0; i < sensorCount; i++) {
            if (!findI2CDevice(1, 0x30 + i)) {
                warn(true);
            }
        }
        // Restore mux state: all channels enabled (initToF sets this)
        uint8_t muxMask = (sensorCount == 5) ? 0x1F : 0x0F;
        mux.setReg(muxMask);
    } else {
        // Mux scan fallback: check each channel at default 0x29
        for (int i = 0; i < sensorCount; i++) {
            mux.setChannel(i);
            if (!findI2CDevice(1, 0x29)) {
                warn(true);
            }
        }
        // Leave mux in last channel state; updateAir switches per cycle
    }
}

void check3116() {
    // if (!findI2CDevice(0, 0x40)) {
    //     warn(true);
    // }a
    // if (!findI2CDevice(0, 0x41)) {
    //     warn(true);
    // }
    // if (!findI2CDevice(0, 0x42)) {
    //     warn(true);
    // }
}

void checkMPR121() {
    if (!findI2CDevice(0, 0x5a)) {
        warn(true);
    }
    if (!findI2CDevice(0, 0x5b)) {
        warn(true);
    }
    if (!findI2CDevice(0, 0x5c)) {
        warn(true);
    }
}

void checkHardwareState() {
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        check3116();
    } else {
        checkMPR121();
    }
    checkToF();
}
