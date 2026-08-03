#ifndef __CHUNI_IO_H__
#define __CHUNI_IO_H__

#include <stdint-gcc.h>
#include <nyanithm_shared.h>



// round47b-patch: maindev_loop() removed (dead code, replaced by cdc_respond).

void hid_task_chuni_input();


extern bool game_connected;
extern volatile bool pending_config_mode;
extern volatile bool in_config_mode;

// CDC command responder, called from Core1 (usb_device.cpp multicore_entry).
// Uses bulk tud_cdc read/write instead of stdio getchar/putchar (old maindev_loop,
// so CDC responses are no longer blocked by Core0's updateInputState() scan.
void cdc_respond();



#endif
