/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __APP_LINK_H__
#define __APP_LINK_H__

#include <tusb.h>

void handleCommand();
bool readCdcPayload(uint8_t* buffer, int length, uint32_t timeoutMs);
void configCdcSessionStateChanged(bool dtr);
void resetConfigCdcSession();
extern bool hid_working ;


#endif
