/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <button.h>
#include <error_state.h>
#include <pico/stdlib.h>
#include <hw_devices.h>

void warn(bool forever = true) {
    do {
        for (int i = 0; i < 2; i++) {
            RGB_LED.fill(250, 150, 0);
            RGB_LED.flush();
            sleep_ms(200);
            RGB_LED.fill(0, 0, 0);
            RGB_LED.flush();
            sleep_ms(200);
        }
        if (getButtonState(BUTTON_PUSH)) {
            sleep_ms(100);
            if (getButtonState(BUTTON_PUSH)) {
                forever = false;
            }
        }
    } while (forever);
    RGB_LED.fill(0x0f, 0x0f, 0x0f);
}

void error(bool forever = true) {
    do {
        for (int i = 0; i < 2; i++) {
            RGB_LED.fill(250, 250, 0);
            RGB_LED.flush();
            sleep_ms(200);
            RGB_LED.fill(0, 0, 0);
            RGB_LED.flush();
            sleep_ms(200);
        }
        if (getButtonState(BUTTON_PUSH)) {
            sleep_ms(100);
            if (getButtonState(BUTTON_PUSH)) {
                forever = false;
            }
        }
    } while (forever);
    RGB_LED.fill(0x0f, 0x0f, 0x0f);
}