/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */


#include <cy8cmbr3116.h>
#include <cy8cmbr3116_cfg.h>

#include <hardware/watchdog.h>
#include <i2c_port.h>
#include <pico/stdlib.h>
#include <production_mode.h>
#include <stdio.h>
#include <string.h>
#include <tusb.h>

namespace {
constexpr uint8_t MBR_CONFIG_SIZE = 128;
constexpr uint8_t MBR_CRC_OFFSET = 0x7E;
constexpr uint32_t MBR_IDLE_TIMEOUT_MS = 200;
constexpr uint32_t MBR_SAVE_TIMEOUT_MS = 1200;
constexpr uint32_t MBR_RESET_TIMEOUT_MS = 300;

void serviceMbrWait(uint32_t ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < ms) {
        tud_task();
        watchdog_update();
        sleep_ms(1);
    }
}

bool mbrReadReg(uint8_t addr, uint8_t reg, uint8_t* out) {
    return i2c_write_stop_read(0, addr, reg, out, 1) == 1;
}

bool mbrWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2] = { reg, value };
    return i2c_write(0, addr, data, sizeof(data), false) == (int)sizeof(data);
}

uint16_t mbrConfigCrc(const uint8_t* cfg) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < MBR_CRC_OFFSET; i++) {
        crc ^= (uint16_t)cfg[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

bool mbrWaitCommandIdle(uint8_t addr, uint32_t timeoutMs, bool tolerateNak) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool sawAck = false;
    while (to_ms_since_boot(get_absolute_time()) - start < timeoutMs) {
        tud_task();
        watchdog_update();
        uint8_t command = 0xFF;
        if (mbrReadReg(addr, CTRL_CMD_ADDRESS, &command)) {
            sawAck = true;
            if (command == 0) return true;
        } else if (!tolerateNak && sawAck) {
            return false;
        }
        sleep_ms(5);
    }
    return false;
}

bool mbrWaitAfterReset(uint8_t addr) {
    // SW_RESET may execute as late as 50 ms after ACK; TI2CBOOT is up to 15 ms.
    serviceMbrWait(65);
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start < MBR_RESET_TIMEOUT_MS) {
        uint8_t chipAddress = 0;
        if (mbrReadReg(addr, I2C_ADDR_ADDRESS, &chipAddress) && chipAddress == addr) return true;
        serviceMbrWait(5);
    }
    return false;
}

bool mbrResetAndWait(uint8_t addr) {
    return mbrWriteReg(addr, CTRL_CMD_ADDRESS, 0xFF) && mbrWaitAfterReset(addr);
}

bool mbrWriteConfig(uint8_t addr, const uint8_t* cfg) {
    uint8_t packet[MBR_CONFIG_SIZE + 1];
    packet[0] = 0;
    memcpy(packet + 1, cfg, MBR_CONFIG_SIZE);
    return i2c_write(0, addr, packet, sizeof(packet), false) == (int)sizeof(packet);
}

bool mbrCompareConfig(const uint8_t* lhs, const uint8_t* rhs, uint8_t* mismatch) {
    for (uint8_t i = 0; i < MBR_CONFIG_SIZE; i++) {
        if (lhs[i] != rhs[i]) {
            if (mismatch) *mismatch = i;
            return false;
        }
    }
    return true;
}

// Roll-back for a failed burn: nothing was persisted to NVM yet at this
// point, so resetting the part is enough to return it to its previous config.
// Returns false only when the reset or the post-reset read-back itself failed,
// which the caller reports so the host can see the part is in an unverified
// state (a follow-up 0xC6 read is the recovery action).
bool mbrRecoverPrevious(uint8_t addr, const uint8_t* previous) {
    if (!mbrResetAndWait(addr)) return false;
    uint8_t current[MBR_CONFIG_SIZE];
    return read_cy8cmbr3116_config(addr, current) && mbrCompareConfig(current, previous, nullptr);
}
}

bool detect3116(uint8_t addr) {
    uint8_t value = 0;
    return mbrReadReg(addr, I2C_ADDR_ADDRESS, &value) && value == addr;
}

MbrProgramResult program_cy8cmbr3116_custom(uint8_t addr, const uint8_t* cfg) {
    MbrProgramResult result = { MBR_PROGRAM_INVALID_CONFIG, 0, 0, 0 };
    if (!cfg) return result;
    const uint8_t verifyAddr = cfg[I2C_ADDR_ADDRESS];
    if (verifyAddr < 0x08 || verifyAddr > 0x77) return result;
    const uint16_t expectedCrc = mbrConfigCrc(cfg);
    const uint16_t suppliedCrc = (uint16_t)cfg[0x7E] | ((uint16_t)cfg[0x7F] << 8);
    if (expectedCrc != suppliedCrc) {
        result.offset = MBR_CRC_OFFSET;
        return result;
    }

    uint8_t previous[MBR_CONFIG_SIZE];
    if (!read_cy8cmbr3116_config(addr, previous)) {
        result.code = MBR_PROGRAM_BACKUP_FAILED;
        return result;
    }
    if (!mbrWaitCommandIdle(addr, MBR_IDLE_TIMEOUT_MS, false)) {
        result.code = MBR_PROGRAM_BUSY;
        return result;
    }

    if (!mbrWriteConfig(addr, cfg)) {
        result.code = MBR_PROGRAM_RAM_WRITE_FAILED;
        // 0x80 sentinel: the post-failure reset/verify-back also failed, so the
        // part's live config is unverified (distinct from chip CTRL_CMD_ERR
        // codes 0xFD/0xFE which only appear for MBR_PROGRAM_DEVICE_ERROR).
        if (!mbrRecoverPrevious(addr, previous)) result.error = 0x80;
        return result;
    }
    serviceMbrWait(1);

    uint8_t ram[MBR_CONFIG_SIZE];
    uint8_t mismatch = 0;
    if (!read_cy8cmbr3116_config(addr, ram) || !mbrCompareConfig(ram, cfg, &mismatch)) {
        result.code = MBR_PROGRAM_RAM_VERIFY_FAILED;
        result.offset = mismatch;
        if (!mbrRecoverPrevious(addr, previous)) result.error = 0x80;
        return result;
    }

    if (!mbrWaitCommandIdle(addr, MBR_IDLE_TIMEOUT_MS, false) ||
        !mbrWriteReg(addr, CTRL_CMD_ADDRESS, 0x02)) {
        result.code = MBR_PROGRAM_SAVE_START_FAILED;
        if (!mbrRecoverPrevious(addr, previous)) result.error = 0x80;
        return result;
    }
    if (!mbrWaitCommandIdle(addr, MBR_SAVE_TIMEOUT_MS, true)) {
        result.code = MBR_PROGRAM_SAVE_TIMEOUT;
        // Command state is unknown: never write another CTRL_CMD while it may be nonzero.
        return result;
    }

    uint8_t status = 0;
    if (!mbrReadReg(addr, CTRL_CMD_STATUS_ADDRESS, &status)) {
        result.code = MBR_PROGRAM_STATUS_READ_FAILED;
        return result;
    }
    result.status = status;
    if (status & 0x01) {
        result.code = MBR_PROGRAM_DEVICE_ERROR;
        if (!mbrReadReg(addr, CTRL_CMD_ERR_ADDRESS, &result.error)) result.error = 0xFF;
        return result;
    }

    if (!mbrWriteReg(addr, CTRL_CMD_ADDRESS, 0xFF) || !mbrWaitAfterReset(verifyAddr)) {
        result.code = MBR_PROGRAM_RESET_FAILED;
        return result;
    }

    uint8_t persisted[MBR_CONFIG_SIZE];
    if (!read_cy8cmbr3116_config(verifyAddr, persisted) || !mbrCompareConfig(persisted, cfg, &mismatch)) {
        result.code = MBR_PROGRAM_NVM_VERIFY_FAILED;
        result.offset = mismatch;
        return result;
    }

    result.code = MBR_PROGRAM_OK;
    return result;
}

bool read_cy8cmbr3116_config(uint8_t addr, uint8_t* cfg) {
    if (!cfg) return false;
    for (uint8_t chunk = 0; chunk < 4; chunk++) {
        tud_task();
        watchdog_update();
        uint8_t reg = chunk * 32;
        if (i2c_write_stop_read(0, addr, reg, cfg + chunk * 32, 32) != 32) return false;
    }
    return true;
}

void program3116() {
    while (1) {
        tud_task();
        printf("\n");
        if (detect3116(0x43) && detect3116(0x44)) {
            printf("+-----------------------------+\n");
            printf("|both 0x43 and 0x44 are ready.|\n");
            printf("+-----------------------------+\n");
        } else if (detect3116(0x43)) {
            if (detect3116(0x37)) {
                printf("programing 0x44.\n");
                program_cy8cmbr3116_custom(0x37, cy8cmbr3116_cfg_0x44);
            } else {
                printf("waiting for the SECOND chip.\n");
            }
        } else if (detect3116(0x37)) {
            printf("programing 0x43.\n");
            program_cy8cmbr3116_custom(0x37, cy8cmbr3116_cfg_0x43);
        } else {
            printf("waiting for the FIRST chip.\n");
        }
        sleep_ms(500);
    }
}

void productionMode() {
    program3116();
}
