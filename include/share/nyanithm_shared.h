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
#include <stddef.h>

#define CONTROLLER_CONFIG_MAGIC 0x88
#define CONTROLLER_CONFIG_VERSION 0x02
#define NYANITHM_API_LEVEL 0x10
#define NYANITHM_FW_VERSION "1.6.6-beta9"
#define NYANITHM_BUILD_ID "round90d"

const uint8_t MBR_TRACE_VERSION = 1;
const uint8_t MBR_TRACE_BUSY = 0;
const uint8_t MBR_TRACE_PROFILE = 0x01;
const uint8_t MBR_TRACE_COPY_FAILED = 0x80;
const uint8_t MBR_TRACE_ATTEMPTED = 0x01;
const uint8_t MBR_TRACE_VALID = 0x02;
const uint8_t MBR_TRACE_HAS_GOOD = 0x04;
const uint8_t MBR_TRACE_COHERENT = 0x08;
const uint8_t MBR_TRACE_BUTTON_VALID = 0x10;
const uint8_t MBR_TRACE_READ_THIS_FRAME = 0x20;

// RP2040 LE wire layout; times/IDs describe host reads, not chip scans.
// VALID: last attempt; COHERENT: retained counts. Counters reset with profile/layout.
struct MbrTraceChip {
    uint32_t readCalls;
    uint32_t ioFailures;
    uint32_t syncMismatches;
    uint32_t attemptStartMs;
    uint32_t attemptEndMs;
    uint32_t goodStartMs;
    uint32_t goodEndMs;
    uint32_t buttonStartMs;
    uint32_t buttonEndMs;
    uint16_t rangeMask; // 1 = count <=255 per electrode.
    uint8_t flags;
    uint8_t sync;
};
struct MbrTouchTrace {
    uint8_t tag;
    uint8_t version;
    uint8_t chipCount;
    uint8_t flags;
    uint32_t frameId;
    uint32_t publishedMs;
    uint16_t counts[32];
    uint8_t slider[32];
    uint16_t hardware[3];
    uint16_t verified[3];
    MbrTraceChip chips[3];
};
static_assert(sizeof(MbrTraceChip) == 40, "trace chip layout");
static_assert(offsetof(MbrTouchTrace, counts) == 12, "trace counts offset");
static_assert(offsetof(MbrTouchTrace, chips) == 120, "trace quality offset");
static_assert(sizeof(MbrTouchTrace) == 240, "trace wire length");

const uint8_t MBR_HISTORY_VERSION = 1;
const uint16_t MBR_HISTORY_CAPACITY = 96;
const uint16_t MBR_HISTORY_PRE_MS = 180;
const uint16_t MBR_HISTORY_POST_MS = 160;
enum MbrHistoryState : uint8_t {
    MBR_HISTORY_OFF, MBR_HISTORY_ARMED, MBR_HISTORY_POST,
    MBR_HISTORY_FROZEN, MBR_HISTORY_FAILED, MBR_HISTORY_UNSUPPORTED
};
const uint8_t MBR_HISTORY_EDGE = 1;
const uint8_t MBR_HISTORY_MANUAL = 2;
const uint8_t MBR_HISTORY_TIMEOUT = 4;
const uint8_t MBR_HISTORY_PROFILE_CHANGED = 8;
struct MbrHistoryStatus {
    uint8_t tag, version, state, pending;
    uint32_t session, triggerFrame, triggerMs, startedMs, endedMs;
    uint32_t stored, overwritten, hookCount, hookSumUs, hookMaxUs;
    uint16_t capacity, queued, preMs, postMs;
    uint8_t reason, reserved[3];
};
struct MbrHistoryFrame {
    uint8_t tag, version, reserved[2];
    uint32_t session;
    MbrTouchTrace trace;
};
static_assert(sizeof(MbrHistoryStatus) == 56, "history status wire length");
static_assert(sizeof(MbrHistoryFrame) == 248, "history frame wire length");


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

// Versioned, opt-in global MBR difference-count gate. These are signal counts,
// not a physical distance measurement. Old zero-filled reserved bytes stay off.
const uint8_t MBR_DISTANCE_PROFILE_MAGIC = 0xD6;
const uint8_t MBR_DISTANCE_PROFILE_VERSION = 1;
const uint8_t MBR_DISTANCE_FLAG_ENABLED = 0x01;
const uint8_t MBR_DISTANCE_PROFILE_CHECK_SEED = 0xA7;

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
    // max(80, value), so the software gate only tightens from round50.
    // Native BUTTON_STAT remains a separate candidate gate; ATH/configuration
    // determines its actual threshold, not a universal fixed value of 128.
    // thReleaseKey only applies to MPR121 (MBR has no per-sensor release reg).
    uint8_t thTouchKey[32];    //
    uint8_t thReleaseKey[32];  //
    // round80: MBR3116 悬空抑制门 (hover-rejection gate)。0 = 关闭 (完全旧行为);
    // >0 时 MBR 新触摸 ON 验证门抬高为 max(verifyBaseK, mbrTouchGate), 仅作用于
    // 触摸判定 (release/sticky 不变)。本字段为历史新触摸验证门；信号值不等于
    // 毫米距离，近距悬空与接触可能重叠或同时饱和，不能保证贴面才触发。
    // v1 (hw1/hw2) 与 v2 (hw3/hw4) 双主控布局同样生效。
    // 注意: slide 过渡信号同样必须先过此门 (reject 在邻格快路径之前);
    // gate>=200 时强信号快路径的有效阈值也被抬到 gate 值, 属预期行为。
    // round84 修订: (a) 游戏车道 k±1 上一周期有已确认触摸时, 本格 gate 预检
    // 下放为 verifyBaseK+40 (slide 连续性豁免)。已接触邻格也可能让悬空格受益，
    // 因此不能作持续硬截止；round89a 的独立 profile 在豁免前持续执行。
    // 原生手动 FINGER_THRESHOLD 31-200 仅在 ATH_EN=0 时接管，仍需实测可分性。
    uint8_t mbrTouchGate;      //
    // Global zero/full signal points and ON/OFF percentages of that range.
    // All eight bytes must validate before flags can enable the strict gate.
    // Profile check = 0xA7 XOR bytes 91..97; distinct from the blob xorSum.
    uint8_t mbrDistanceMagic;       // byte 91: MBR_DISTANCE_PROFILE_MAGIC
    uint8_t mbrDistanceVersion;     // byte 92: independent profile version
    uint8_t mbrDistanceFlags;       // byte 93: bit 0 enables; other bits invalid
    uint8_t mbrDistanceZero;        // byte 94: 0..255, zero output at/below Z
    uint8_t mbrDistanceFull;        // byte 95: 0..255, must be greater than Z
    uint8_t mbrDistanceOnPercent;   // byte 96: 1..100
    uint8_t mbrDistanceOffPercent;  // byte 97: 0..99, must be below ON
    uint8_t mbrDistanceCheck;       // byte 98: profile check byte
    uint8_t reserved[28];      // bytes 99..126
    uint8_t xorSum;            // 前127字节异或和, 用于校验
};

// round64: the config blob must stay 128 bytes (flash page layout + CFG_SET
// protocol depend on it).
#if defined(__cplusplus)
static_assert(sizeof(struct controller_config) == 128, "controller_config must stay 128 bytes");
static_assert(offsetof(controller_config, mbrTouchGate) == 90, "legacy MBR gate offset must stay 90");
static_assert(offsetof(controller_config, mbrDistanceMagic) == 91, "MBR profile offset must stay 91");
static_assert(offsetof(controller_config, mbrDistanceCheck) == 98, "MBR profile check offset must stay 98");
static_assert(offsetof(controller_config, reserved) == 99, "reserved offset must stay 99");
static_assert(offsetof(controller_config, xorSum) == 127, "config checksum offset must stay 127");
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
    CMD_DEBUG_TELEMETRY = 0xC1,  // 528B; +36 u32 gate read failures on round89c+.
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
    // round88: MBR3116 单传感器调试数据读取(仅配置模式受理)。需固件 >= 1.6.5。
    // 主机必须分两次 write -- 先 [0xC7], 再 2B 载荷 [addr][sensor](与 CFG_SET /
    // 0xC6 双写铁律同源, 单次合并写会被 stdio/TinyUSB 路径吞读载荷)。地址白名单
    // 0x40-0x44(0x37 工装地址不受理, 固件驱动对象无此构造地址), sensor 合法
    // 区间 0-15(TRM SENSOR_ID 0x82)。
    // 固件写 SENSOR_ID -> 有界泵等待40ms -> 突发读调试区。等待不代表
    // 识别了一个新芯片扫描，刷新周期仍须按原厂条件和实机状态核实。
    // 0xDB..0xE7 共 13 字节 -> SYNC1(0xDB)==SYNC2(0xE7) 且 DEBUG_SENSOR_ID
    // (0xDC)==sensor 校验通过后回 11 字节二进制(全小端):
    //   [0]=0xC7 回显 [1]=addr [2]=sensor [3]=SYNC_COUNTER
    //   [4]=DEBUG_CP(pF 原值) [5..6]=DIFFERENCE_COUNT [7..8]=BASELINE
    //   [9..10]=RAW_COUNT (0xE4 AVG_RAW_COUNT 不回传, 面板如需可后续扩展)
    // 校验失败或 I2C 错误回文本行 "3116 debug fail\n"(按首字节 0xC7 与长度区分)。
    // 用途: 指定电极的信号观察与研究；按钮 DIFF 为0..255，RAW/BASELINE
    // 单位不同，不能把16位容器或未标定信号换算为毫米距离。
    CMD_MBR3116_DEBUG = 0xC7,
    CMD_MBR_TOUCH_TRACE = 0xC8, // round89x: 240B frame, BUSY=0, or timeout if TX full.
    CMD_MBR_HISTORY_ARM = 0xC9,
    CMD_MBR_HISTORY_FREEZE = 0xCA,
    CMD_MBR_HISTORY_READ = 0xCB,
    CMD_MBR_HISTORY_STATUS = 0xCC,
} NyanithmCmd;

#endif
