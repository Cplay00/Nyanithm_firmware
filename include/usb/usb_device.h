/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __USB_DEVICE_H__
#define __USB_DEVICE_H__

#include <tusb.h>


void initUSBDevice();

// round51: true while Core0 is the sole tud_task() driver (config mode,
// 4k/6k other-modes, production mode). Core1's loop steps aside when set.
extern volatile bool core0_owns_usb;




#endif
