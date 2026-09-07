/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __PRODUCTION_MODE_H__
#define __PRODUCTION_MODE_H__

#include <boot_mode.h>

void productionMode();

enum MbrProgramCode : uint8_t {
    MBR_PROGRAM_OK = 0,
    MBR_PROGRAM_INVALID_CONFIG,
    MBR_PROGRAM_BACKUP_FAILED,
    MBR_PROGRAM_BUSY,
    MBR_PROGRAM_RAM_WRITE_FAILED,
    MBR_PROGRAM_RAM_VERIFY_FAILED,
    MBR_PROGRAM_SAVE_START_FAILED,
    MBR_PROGRAM_SAVE_TIMEOUT,
    MBR_PROGRAM_STATUS_READ_FAILED,
    MBR_PROGRAM_DEVICE_ERROR,
    MBR_PROGRAM_RESET_FAILED,
    MBR_PROGRAM_NVM_VERIFY_FAILED,
};

struct MbrProgramResult {
    MbrProgramCode code;
    uint8_t offset;
    uint8_t status;
    uint8_t error;
};

MbrProgramResult program_cy8cmbr3116_custom(uint8_t addr, const uint8_t* cfg);

bool read_cy8cmbr3116_config(uint8_t addr, uint8_t* cfg);

bool detect3116(uint8_t addr);

#endif