/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <i2c_port.h>
#include <tca9539.h>

uint8_t TCA9539::readReg(uint8_t reg) {
    uint8_t value = 0;
    if (i2c_write_read(_i2c_bus, _i2c_address, &reg, 1, &value, 1) != 1) return 0;
    return value;
}
void TCA9539::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    i2c_write(_i2c_bus, _i2c_address, buf, 2, false);
}

TCA9539::TCA9539(uint8_t i2c_bus, uint8_t i2c_address) {
    _i2c_bus = i2c_bus;
    _i2c_address = i2c_address;
}
uint8_t TCA9539::getInputP0() {
    return readReg(0x00);
}
uint8_t TCA9539::getInputP1() {
    return readReg(0x01);
}
void TCA9539::setOutputP0(uint8_t mask) {
    writeReg(0x02, mask);
}
void TCA9539::setOutputP1(uint8_t mask) {
    writeReg(0x03, mask);
}
void TCA9539::invertP0(uint8_t mask) {
    writeReg(0x04, mask);
}
void TCA9539::invertP1(uint8_t mask) {
    writeReg(0x05, mask);
}
void TCA9539::setConfP0(uint8_t mask) {
    writeReg(0x06, mask);
}
void TCA9539::setConfP1(uint8_t mask) {
    writeReg(0x07, mask);
}
bool TCA9539::isConnected() {
    uint8_t reg = 0, value = 0;
    return i2c_write_read(_i2c_bus, _i2c_address, &reg, 1, &value, 1) == 1;
}
