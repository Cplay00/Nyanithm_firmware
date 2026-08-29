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

void program_cy8cmbr3116_custom(uint8_t addr, uint8_t* cfg);

void verify_cy8cmbr3116_burn(uint8_t addr, uint8_t* cfg);

bool detect3116(uint8_t addr);

#endif