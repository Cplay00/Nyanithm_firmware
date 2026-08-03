/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <hw_devices.h>
#include <controller_config.h>

typedef struct {
    uint8_t r, g, b;
} RGB;
static RGB hsv_to_rgb(uint8_t h) {
    uint8_t region = h / 43;
    uint8_t remainder = (h - (region * 43)) * 6;

    uint8_t p = 0;
    uint8_t q = (255 * (255 - remainder)) / 255;
    uint8_t t = (255 * remainder) / 255;

    uint8_t r, g, b;

    switch (region) {
    case 0:
        r = 255;
        g = t;
        b = 0;
        break;
    case 1:
        r = q;
        g = 255;
        b = 0;
        break;
    case 2:
        r = 0;
        g = 255;
        b = t;
        break;
    case 3:
        r = 0;
        g = q;
        b = 255;
        break;
    case 4:
        r = t;
        g = 0;
        b = 255;
        break;
    default:
        r = 255;
        g = 0;
        b = q;
        break;
    }
    return (RGB){ r, g, b };
}

// 每次调用更新一帧彩虹流水
void update_rainbow_frame() {

    static uint8_t count = 0;
    if (count == 4) {
        count = 0;

        static uint8_t offset = 0;  // 每次调用偏移色相

        if (g_lampCount < 31) {
            RGB_LED.fill(0, g_lampCount, 31 - g_lampCount);
        }
        for (int i = 0; i < g_lampCount; i++) {
            uint8_t hue = (i * 256 / g_lampCount + offset) & 0xFF;  // 0-255 循环
            RGB rgb = hsv_to_rgb(hue);
            RGB_LED.setPixelColor((g_lampCount - 1) - i, WS2812::RGB((rgb.r * ControllerConfig.lightLimit) / 255, (rgb.g * ControllerConfig.lightLimit) / 255, (rgb.b * ControllerConfig.lightLimit) / 255));
        }
        RGB_LED.show();

        offset++;  // 改变偏移量，形成流动
    } else {
        count++;
    }
}