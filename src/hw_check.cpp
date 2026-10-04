/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 * Copyright (c) 2026 Catium2006
 */
#include <controller_config.h>
#include <hardware/watchdog.h>
#include <hw_devices.h>
#include <hw_check.h>
#include <i2c_port.h>
#include <tusb.h>

bool hardwareMismatchAtBoot = false;
uint8_t detectedMprMask = 0, detectedMbrMask = 0;

static void startupWait(uint32_t durationMs) {
    const uint32_t start = to_ms_since_boot(get_absolute_time());
    do {
        watchdog_update();
        // Core1 is parked; Core0 owns USB until startup completes.
        tud_task();
        sleep_ms(10);
    } while (to_ms_since_boot(get_absolute_time()) - start < durationMs);
}

bool hardwareConfigMismatch() {
    uint8_t mprMask = 0, mbrMask = 0;
    CY8CMBR3116* chips[5] = {&MBR3116A, &MBR3116B, &MBR3116C, &MBR3116D, &MBR3116E};
    for (uint8_t i = 0; i < 3; ++i) {
        watchdog_update();
        if (findI2CDevice(0, 0x5a + i, 1)) mprMask |= 1u << i;
    }
    for (uint8_t i = 0; i < 5; ++i) {
        watchdog_update();
        uint8_t id[3] = {};
        // Infineon TRM: FAMILY_ID=0x9A; CY8CMBR3116 DEVICE_ID=0x0A05.
        // Startup only: a chip can still be waking after the driver's immediate
        // NACK retries. Confirm failed discovery after two bounded 10ms waits.
        for (uint8_t attempt = 0; attempt < 3; ++attempt) {
            if (chips[i]->requestDataFromAddress(0x8f, sizeof(id), id) == 0) {
                if (id[0] == 0x9a && id[1] == 0x05 && id[2] == 0x0a)
                    mbrMask |= 1u << i;
                break;
            }
            if (attempt < 2) startupWait(10);
        }
    }
    const bool v2 = ControllerConfig.hwVer >= 3;
    detectedMprMask = mprMask;
    detectedMbrMask = mbrMask;
    const bool useMbr = v2 || (ControllerConfig.cfg0 & CFG0_BIT_MBR3116);
    const bool touchMismatch = useMbr
        ? (mprMask != 0 || mbrMask != (v2 ? 0x18 : 0x07))
        : (mprMask != 0x07 || mbrMask != 0);
    const uint8_t expectedTof = (ControllerConfig.hwVer == 2 ||
                                 ControllerConfig.hwVer == 4) ? 0x1f : 0x0f;
    // IR is auto-selected, not a saved configuration bit. WS2812 has no readback.
    return touchMismatch || (!usingIR &&
        (g_tofPhysicalMask != expectedTof || g_tofReadyMask != expectedTof));
}

void checkHardwareState() {
    hardwareMismatchAtBoot = hardwareConfigMismatch();
    if (hardwareMismatchAtBoot) {
        for (uint8_t flash = 0; flash < 3; ++flash) {
            // All physical chain positions; visible even if lightLimit is zero.
            RGB_LED.fill(255, 0, 0);
            RGB_LED.flush();
            startupWait(2000);
            if (flash < 2) {
                RGB_LED.fill(0, 0, 0);
                RGB_LED.flush();
                startupWait(500);
            }
        }
    }
    RGB_LED.fill(0, 0, 0);
    RGB_LED.fill(15, 15, 15, 0, g_lampCount);
    RGB_LED.flush();
    // FIFO submission is not completion; allow the final frame/latch to finish.
    startupWait(2);
}
