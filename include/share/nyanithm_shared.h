/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __NYANITHM_SHARED_H__
#define __NYANITHM_SHARED_H__

#ifdef PICO_SDK_VERSION_MAJOR
#include <stdint-gcc.h>
#else
#include <stdint.h>
#endif

#define CONTROLLER_CONFIG_MAGIC 0x88
#define CONTROLLER_CONFIG_VERSION 0x02
#define NYANITHM_API_LEVEL 0x10
#define NYANITHM_FW_VERSION "1.5.3"


const uint8_t CFG0_BIT_FORCE16LEDS = 0b00000001;
const uint8_t CFG0_BIT_MBR3116 = 0b00000010;
const uint8_t CFG0_BIT_DARKER_GAP = 0b00000100;
const uint8_t CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD = 0b00000001;
const uint8_t CFG1_BIT_ENABLE_AIR_INPUT_AS_KEYBOARD = 0b00000010;
// round55: cfg2/cfg3 previously reserved (always 0).
const uint8_t CFG2_BIT_DISABLE_LAMP_ARRAY = 0b00000001;  // set = OFF. Clear = Windows Dynamic Lighting (LampArray USB HID) enabled - legacy configs (cfg2==0) keep it on.
// cfg3: additive input latency, 0-15 ms (MBR3116-style tuning knob, 0 = off).
const uint8_t INPUT_LATENCY_MAX_MS = 15;

struct controller_config {
    uint8_t magic;            // 此值必须为 CONTROLLER_CONFIG_MAGIC
    uint8_t cfgVer;           // 配置文件版本
    uint8_t hwVer;            // 硬件版本
    uint8_t cfg0;             // b0: 强制使用16灯模式; b1: 使用cy8cmbr3116; b2: 降低非判定区域灯光亮度
    uint8_t cfg1;             // b0: 启用触摸板键盘输入; b1: 启用Air键盘输入
    uint8_t cfg2;             // b0: 关闭 LampArray / Windows 动态照明 (round55, 置1=禁用)
    uint8_t cfg3;             // 附加输入延迟 ms, 0-15, 0=关闭 (round55)
    uint8_t th_touch;         //
    uint8_t th_release;       //
    uint8_t debounce;         // 低4位dt, 高4位dr, 有效值3位
    uint16_t airMax;          // air判定上限
    uint16_t airMin;          // air判定下限
    int16_t heightOffset[5];  // 高度偏移值
    uint8_t lightLimit;
    uint8_t heightRangeCfg;   // Air key segment overlap (mm), 0=default 10
    uint8_t reserved[101];  //
    uint8_t xorSum;         // 前127字节异或和, 用于校验
};

extern controller_config defaultConfig;

#define THRE_TOUCH_DEF 4    // default touch threshold value
#define THRE_RELEASE_DEF 5  // default release threshold value

typedef enum {
    CMD_GET_API_LEVEL = 0xAF,
    CMD_DEV_DETECT = 0xB0,
    CMD_GET_INPUT,
    CMD_SET_LED,
    CMD_CONFIG_MODE,
    CMD_CFG_READ,
    CMD_CFG_SAVE,
    CMD_CFG_ERASE,
    CMD_CFG_SET,
    CMD_GET_STATUS,
    CMD_EXIT,
    CMD_LOAD3116CONFIG,
    CMD_FLASHING,
    CMD_DETECT,
    CMD_GET_VERSION = 0xBD,
    CMD_DEBUG_RAW = 0xBE,
    CMD_DEBUG_ALL = 0xBF,
    CMD_DEBUG_CHAIN = 0xC0,
    CMD_DEBUG_TELEMETRY = 0xC1,
    CMD_DEBUG_DIFF = 0xC2,
    CMD_CFG_KEEPALIVE = 0xC3,  // round57: 配置模式心跳,静默重置 60s 自动退出计时器
} NyanithmCmd;

#endif
