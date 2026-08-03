/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __HW_DEVICES_H__
#define __HW_DEVICES_H__
#include <ws2812.h>
#include <cy8cmbr3116.h>
#include <cy8cmbr3116_cfg.h>
#include <gpio_def.h>
#include <mpr121.h>
#include <pca954x.h>
#include <vl53l0x.h>

extern VL53L0X tof0;
extern VL53L0X tof1;
extern VL53L0X tof2;
extern VL53L0X tof3;
extern VL53L0X tof4;

extern MPR121 mpr0;
extern MPR121 mpr1;
extern MPR121 mpr2;

extern CY8CMBR3116 MBR3116A;
extern CY8CMBR3116 MBR3116B;
extern CY8CMBR3116 MBR3116C;

extern PCA954X mux0;

extern WS2812 RGB_LED;
extern uint8_t g_lampCount;

void initHwDevices();
void updateInputState();


void updateTouchData4k();
void updateTouchData6k();

extern int16_t heightData[5];
extern uint16_t heightDataOriginal[5];
extern bool airKeys[6];

extern uint8_t touchData[4];
extern bool touchData4k[4];
extern bool touchData6k[6];

// round46: cross-core seqlock generation. Core0 bumps this BEFORE and AFTER
// updating the shared touch state; Core1 readers copy shared bytes only when
// the generation is even and unchanged across the copy (torn-read elimination).
extern volatile uint32_t touchStateGen;

// round46: diagnostics shared from Core0 to Core1 (telemetry command 0xC1)
extern uint16_t hwTouch[3];    // pre-verification MPR121 touch snapshot
extern uint16_t rawTouch[3];   // verified touch bits (after software verify)
extern uint8_t  g_verifyFail[36];  // per-electrode I2C verification rejections (saturating)
extern uint32_t g_loopMinUs, g_loopMaxUs, g_loopSumUs, g_loopCount;  // Core0 cycle timing

#endif
