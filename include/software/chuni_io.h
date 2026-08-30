/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __CHUNI_IO_H__
#define __CHUNI_IO_H__

#include <stdint-gcc.h>
#include <nyanithm_shared.h>



// round47b-patch: maindev_loop() removed (dead code, replaced by cdc_respond).

void hid_task_chuni_input();


extern bool game_connected;
extern volatile bool pending_config_mode;
extern volatile bool pending_flashing;  // round78: normal-mode 0xBB -> Core0 executes
extern volatile bool flashingArmed;     // round78b: stateful 0xBB confirm window
extern volatile uint32_t flashingArmedAt;
extern volatile bool in_config_mode;
// round78c: flashing post-mortem, readable via CMD_FLASH_DIAG (0xCD) after a
// silent failure (deny/boot-fail text went to an already-closed host port).
// code: 0=none 1=boot attempted 2=deny window expired 3=deny wrong byte.
// rc: flash_safe_execute return (0xFF = never reached); gapMs: device-side
// 0xBB->confirm/timeout distance.
extern volatile uint8_t flashDiagCode;
extern volatile uint8_t flashDiagRc;
extern volatile uint32_t flashDiagGapMs;

// CDC command responder, called from Core1 (usb_device.cpp multicore_entry).
// Uses bulk tud_cdc read/write instead of stdio getchar/putchar (old maindev_loop,
// so CDC responses are no longer blocked by Core0's updateInputState() scan.
void cdc_respond();



#endif
