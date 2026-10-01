#ifndef __TOUCH_SNAPSHOT_H__
#define __TOUCH_SNAPSHOT_H__

#include <stdint.h>

struct TouchInputSnapshot {
    uint8_t keys[4];
    uint8_t slider[32];
    uint8_t pressure[32];
    uint16_t hardware[3];
    uint16_t verified[3];
    uint8_t air;
};

#endif
