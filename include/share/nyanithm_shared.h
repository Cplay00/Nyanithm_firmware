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
#define NYANITHM_FW_VERSION "1.6.3-beta1"


const uint8_t CFG0_BIT_FORCE16LEDS = 0b00000001;
const uint8_t CFG0_BIT_MBR3116 = 0b00000010;
const uint8_t CFG0_BIT_DARKER_GAP = 0b00000100;
const uint8_t CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD = 0b00000001;
const uint8_t CFG1_BIT_ENABLE_AIR_INPUT_AS_KEYBOARD = 0b00000010;
// round55: cfg2/cfg3 previously reserved (always 0).
const uint8_t CFG2_BIT_DISABLE_LAMP_ARRAY = 0b00000001;  // set = OFF. Clear = Windows Dynamic Lighting (LampArray USB HID) enabled - legacy configs (cfg2==0) keep it on.
// round68/69: 压力报送&压力映射 单开关(实验)。开启后 GET_INPUT 对游戏侧
// (chuniio_nyanithm.dll)报线性归一化 0-255 映射值: 按下=映射压力(MPR ×2 clamp 255 /
// MBR 原生 0-255), 释放=0; 面板会话(0xC5 0x01) round80 起对 MPR 同步 ×2 尺度,
// MBR 仍报原始 0-255。
// 默认 OFF (legacy configs keep binary 0/128)。Requires round66 pressure snapshot
// pipeline. NOTE: API stays 0x10 for gen-1 hardware - 0xC4/0xC5 are additive
// commands gated by firmware version on the host side, not by API level.
// round80: MPR 映射从 ×4 改为 ×2 -- 用户实测 MPR diff 与接触面积成比例
// (手指 30-50, 手掌 ~130), 原生到不了 255; ×2 报送保持线性直到游戏内上限
// (clamp 255), ×4 会把中等触摸也打满、丢失面积比例信息。面板实时报送
// (0xC5 level=1, round84 起会话为三级 0/1/2) 同步 ×2 尺度; MBR 原生 0-255 不变。
const uint8_t CFG2_BIT_GAME_RAW_SLIDER = 0b00000010;
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
    // round64: v1.6 per-key thresholds, indexed by slider lane 0-31 (game view).
    // 0 = inherit global th_touch/th_release. MPR121 range 1-63 (same scale as
    // th_touch). MBR3116: values 1-255 stored; the software verify layer applies
    // max(80, value), so the effective gate only tightens from the round50
    // baseline (hardware gate stays 128) -- values <=128 never relax it.
    // thReleaseKey only applies to MPR121 (MBR has no per-sensor release reg).
    uint8_t thTouchKey[32];    //
    uint8_t thReleaseKey[32];  //
    // round80: MBR3116 悬空抑制门 (hover-rejection gate)。0 = 关闭 (完全旧行为);
    // >0 时 MBR 新触摸 ON 验证门抬高为 max(verifyBaseK, mbrTouchGate), 仅作用于
    // 触摸判定 (release/sticky 不变)。背景: 芯片 NVM 阈值 0x0C=128 使 BUTTON_STAT
    // 在 PCB+亚克力上方 ~4mm 悬空即置位, 而固件验证基线 (80) 比芯片还松, 悬空直通;
    // 用户实测 4mm 开始触发 / 0.5mm 已饱和 255, 物理上悬空近触与实触不可区分,
    // 本门只能把触发点从 4mm 压近贴面, 建议真机从 200-220 起调。
    // v1 (hw1/hw2) 与 v2 (hw3/hw4) 双主控布局同样生效。
    // 注意: slide 过渡信号同样必须先过此门 (reject 在邻格快路径之前);
    // gate>=200 时强信号快路径的有效阈值也被抬到 gate 值, 属预期行为。
    // round84 修订: (a) 游戏车道 k±1 上一周期有已确认触摸时, 本格 gate 预检
    // 下放为 verifyBaseK+40 (slide 连续性豁免; 悬空带首格永无已确认邻居,
    // 无法自我锚定, 防护不被穿透); (b) 悬空根治主闸门应上芯片侧
    // FINGER_THRESHOLD (面板「悬空截止校准」联动写入, 合法区间 31-200),
    // 本门降级为噪声兜底 (建议 ~130), max 语义下单提其一无效。
    uint8_t mbrTouchGate;      //
    uint8_t reserved[36];      //
    uint8_t xorSum;            // 前127字节异或和, 用于校验
};

// round64: the config blob must stay 128 bytes (flash page layout + CFG_SET
// protocol depend on it).
#if defined(__cplusplus)
static_assert(sizeof(struct controller_config) == 128, "controller_config must stay 128 bytes");
#else
_Static_assert(sizeof(struct controller_config) == 128, "controller_config must stay 128 bytes");
#endif

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
    CMD_GET_RAW_STATUS = 0xC4,  // round66: 压力上报开关查询,回文本行 RAW=0|1\n
    CMD_SET_RAW_REPORT = 0xC5,  // round66: 压力上报开关设置,跟 1B 0/1,回 RAW=0|1\n
    CMD_READ3116CONFIG = 0xC6,  // round80: 读回 3116 芯片当前 128B 配置块。铁律:
                                // 主机必须分两次 write -- 先 [0xC6], 再 [addr]
                                // (与 CFG_SET 双写铁律同源, 单次合并写会被
                                // stdio/TinyUSB 路径吞读载荷)。白名单同 0xBA
                                // (0x37/0x40-0x44)。成功回 128B 二进制(0x00-0x7F
                                // 原值, 0x7E/0x7F CRC 不重算);失败回文本行
                                // "3116 read fail\n"。仅配置模式受理, 需固件
                                // >= 1.6.2-beta1。
    CMD_FLASH_DIAG = 0xCD,      // round78c: 刷写结局诊断,回 [0xCD][code][rc][gap u32 LE];仅事后查询
} NyanithmCmd;

#endif
