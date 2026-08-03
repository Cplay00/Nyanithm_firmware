/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <app_link.h>
#include <button.h>
#include <controller_config.h>
#include <hw_check.h>
#include <hw_devices.h>
#include <tusb.h>
#include <usb_device.h>

void start_4kMode() {

    while (true) {

        updateInputState();
        updateTouchData4k();
        if (g_lampCount == 16) {
            RGB_LED.fill(0x08, 0x08, 0x08);
            RGB_LED.setColor(3, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);
            RGB_LED.setColor(7, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);
            RGB_LED.setColor(11, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);

            if (touchData4k[0]) {
                for (int j = 0; j < 4; j++)
                    RGB_LED.setColor(15 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            if (touchData4k[1]) {
                for (int j = 0; j < 4; j++)
                    RGB_LED.setColor(11 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            if (touchData4k[2]) {
                for (int j = 0; j < 4; j++)
                    RGB_LED.setColor(7 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            if (touchData4k[3]) {
                for (int j = 0; j < 4; j++)
                    RGB_LED.setColor(3 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
        } else {
            RGB_LED.fill(0x08, 0x08, 0x08);
            RGB_LED.setColor(7, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);
            RGB_LED.setColor(15, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);
            RGB_LED.setColor(23, ControllerConfig.lightLimit, 0, ControllerConfig.lightLimit);

            // K
            if (touchData4k[0]) {
                for (int j = 0; j < 7; j++)
                    RGB_LED.setColor(30 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            // J
            if (touchData4k[1]) {
                for (int j = 0; j < 7; j++)
                    RGB_LED.setColor(22 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            // F
            if (touchData4k[2]) {
                for (int j = 0; j < 7; j++)
                    RGB_LED.setColor(14 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
            // D
            if (touchData4k[3]) {
                for (int j = 0; j < 7; j++)
                    RGB_LED.setColor(6 - j, ControllerConfig.lightLimit, ControllerConfig.lightLimit, 0);
            }
        }

        RGB_LED.flush();

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

            // K
            if (touchData4k[0]) {
                report_buf[3] |= 0b00000100;
            }
            // J
            if (touchData4k[1]) {
                report_buf[3] |= 0b00000010;
            }
            // F
            if (touchData4k[2]) {
                report_buf[2] |= 0b00100000;
            }
            // D
            if (touchData4k[3]) {
                report_buf[2] |= 0b00001000;
            }

            if (getButtonState(BUTTON_UP)) {
                report_buf[6] |= 0b00010000;  // ENTER
            } else if (getButtonState(BUTTON_DOWN)) {
                report_buf[8] |= 0b10000000;  // F2
            } else if (getButtonState(BUTTON_PUSH)) {
                report_buf[6] |= 0b00100000;  // ESCAPE
            }
            tud_hid_n_report(0, 0, report_buf, sizeof(report_buf));
        }
    }
}

void start_6kMode() {
    while (true) {
        updateInputState();
        updateTouchData6k();
        RGB_LED.fill(0, 0, 0);

        if (g_lampCount == 16) {
            RGB_LED.setColor(0, 25, 0, 0);
            RGB_LED.setColor(1, 25, 0, 0);
            RGB_LED.setColor(2, 25, 0, 0);

            RGB_LED.setColor(3, 0, 25, 0);
            RGB_LED.setColor(4, 0, 25, 0);
            RGB_LED.setColor(5, 0, 25, 0);

            RGB_LED.setColor(6, 0, 0, 25);
            RGB_LED.setColor(7, 0, 0, 25);
            RGB_LED.setColor(8, 0, 0, 25);

            RGB_LED.setColor(9, 25, 0, 0);
            RGB_LED.setColor(10, 25, 0, 0);
            RGB_LED.setColor(11, 25, 0, 0);

            RGB_LED.setColor(12, 0, 25, 0);
            RGB_LED.setColor(13, 0, 25, 0);

            RGB_LED.setColor(14, 0, 0, 25);
            RGB_LED.setColor(15, 0, 0, 25);

            if (touchData6k[0]) {
                RGB_LED.setColor(0, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(1, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(2, ControllerConfig.lightLimit, 0, 0);
            }
            if (touchData6k[1]) {
                RGB_LED.setColor(3, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(4, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(5, 0, ControllerConfig.lightLimit, 0);
            }
            if (touchData6k[2]) {
                RGB_LED.setColor(6, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(7, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(8, 0, 0, ControllerConfig.lightLimit);
            }
            if (touchData6k[3]) {
                RGB_LED.setColor(9, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(10, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(11, ControllerConfig.lightLimit, 0, 0);
            }
            if (touchData6k[4]) {
                RGB_LED.setColor(12, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(13, 0, ControllerConfig.lightLimit, 0);
            }
            if (touchData6k[5]) {
                RGB_LED.setColor(14, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(15, 0, 0, ControllerConfig.lightLimit);
            }
        } else {
            RGB_LED.setColor(2, 25, 0, 0);
            RGB_LED.setColor(3, 25, 0, 0);
            RGB_LED.setColor(4, 25, 0, 0);

            RGB_LED.setColor(6, 0, 25, 0);
            RGB_LED.setColor(7, 0, 25, 0);
            RGB_LED.setColor(8, 0, 25, 0);

            RGB_LED.setColor(10, 0, 0, 25);
            RGB_LED.setColor(11, 0, 0, 25);
            RGB_LED.setColor(12, 0, 0, 25);

            RGB_LED.setColor(18, 25, 0, 0);
            RGB_LED.setColor(19, 25, 0, 0);
            RGB_LED.setColor(20, 25, 0, 0);

            RGB_LED.setColor(22, 0, 25, 0);
            RGB_LED.setColor(23, 0, 25, 0);
            RGB_LED.setColor(24, 0, 25, 0);

            RGB_LED.setColor(26, 0, 0, 25);
            RGB_LED.setColor(27, 0, 0, 25);
            RGB_LED.setColor(28, 0, 0, 25);

            if (touchData6k[0]) {
                RGB_LED.setColor(2, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(3, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(4, ControllerConfig.lightLimit, 0, 0);
            }
            if (touchData6k[1]) {
                RGB_LED.setColor(6, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(7, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(8, 0, ControllerConfig.lightLimit, 0);
            }
            if (touchData6k[2]) {
                RGB_LED.setColor(10, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(11, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(12, 0, 0, ControllerConfig.lightLimit);
            }
            if (touchData6k[3]) {
                RGB_LED.setColor(18, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(19, ControllerConfig.lightLimit, 0, 0);
                RGB_LED.setColor(20, ControllerConfig.lightLimit, 0, 0);
            }
            if (touchData6k[4]) {
                RGB_LED.setColor(22, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(23, 0, ControllerConfig.lightLimit, 0);
                RGB_LED.setColor(24, 0, ControllerConfig.lightLimit, 0);
            }
            if (touchData6k[5]) {
                RGB_LED.setColor(26, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(27, 0, 0, ControllerConfig.lightLimit);
                RGB_LED.setColor(28, 0, 0, ControllerConfig.lightLimit);
            }
        }
        RGB_LED.flush();

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

            // L
            if (touchData6k[5]) {
                report_buf[3] |= 0b00001000;
            }
            // K
            if (touchData6k[4]) {
                report_buf[3] |= 0b00000100;
            }
            // J
            if (touchData6k[3]) {
                report_buf[3] |= 0b00000010;
            }
            // D
            if (touchData6k[2]) {
                report_buf[2] |= 0b00001000;
            }
            // S
            if (touchData6k[1]) {
                report_buf[4] |= 0b00000100;
            }
            // A
            if (touchData6k[0]) {
                report_buf[2] |= 0b00000001;
            }

            if (getButtonState(BUTTON_UP)) {
                report_buf[6] |= 0b00010000;  // ENTER
            } else if (getButtonState(BUTTON_DOWN)) {
                report_buf[8] |= 0b10000000;  // F2
            } else if (getButtonState(BUTTON_PUSH)) {
                report_buf[6] |= 0b00100000;  // ESCAPE
            }

            tud_hid_n_report(0, 0, report_buf, sizeof(report_buf));
        }
    }
}