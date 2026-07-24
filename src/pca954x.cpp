/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include "pico/stdlib.h"
#include <PCA954X.h>

#include <debug.h>
#include <i2c_port.h>

#define HIGH true
#define LOW false

PCA954X::PCA954X(uint8_t _port, uint8_t i2c_addr, uint _gpio_reset) {
    port = _port;
    addr = i2c_addr;
    gpio_reset = _gpio_reset;
}

void PCA954X::init() {
    gpio_init(gpio_reset);
    gpio_set_dir(gpio_reset, true);
    gpio_pull_up(gpio_reset);
    gpio_put(gpio_reset, HIGH);
    sleep_ms(1);
    reset();
}

void PCA954X::reset() {
    gpio_put(gpio_reset, LOW);
    sleep_ms(1);
    gpio_put(gpio_reset, HIGH);
}

int PCA954X::setChannel(uint8_t channel) {
    if (channel > 7) {
        return false;
    }
    uint8_t mask = 1 << channel;
    int result = i2c_write(port, addr, &mask, 1, false);
    return result;
}

uint8_t PCA954X::getReg() {
    uint8_t reg = 0;
    int result = i2c_read(port, addr, &reg, 1, false);
    return reg;
}

int PCA954X::setReg(uint8_t mask) {
    int result = i2c_write(port, addr, &mask, 1, false);
    return result != PICO_ERROR_GENERIC;
}

#undef HIGH
#undef LOW