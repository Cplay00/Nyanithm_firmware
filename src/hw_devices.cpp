/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <controller_config.h>
#include <hw_devices.h>
#include <mbr_history.h>
#include <mbr_distance_gate.h>
#include <tca9539.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <cstring>
#include <i2c_port.h>

VL53L0X tof0(1, 0x29);
VL53L0X tof1(1, 0x29);
VL53L0X tof2(1, 0x29);
VL53L0X tof3(1, 0x29);
VL53L0X tof4(1, 0x29);

CY8CMBR3116 MBR3116A(0, 0x40);
CY8CMBR3116 MBR3116B(0, 0x41);
CY8CMBR3116 MBR3116C(0, 0x42);

CY8CMBR3116 MBR3116D(0, 0x43);
CY8CMBR3116 MBR3116E(0, 0x44);

MPR121 mpr0(0, 0x5A);
MPR121 mpr1(0, 0x5B);
MPR121 mpr2(0, 0x5C);

PCA954X mux0(1, 0x70, GPIO_PCA9545_RESET);

WS2812 RGB_LED(pio0, 0, GPIO_RGB, 31);

uint8_t g_lampCount = 31;

TCA9539 iox0(1, 0x74);

bool usingIR = false;
bool useMuxScan = false;
uint8_t g_tofPhysicalMask = 0;
uint8_t g_tofReadyMask = 0;  // Only successfully initialized channels may be polled.
uint8_t heightRange = 10;  // Air key segment overlap (mm), updated from config in initHwDevices

// ===== round49: Kalman 1D filter for ToF sensor smoothing =====
struct Kalman1D {
    float x;  // position estimate (mm, ToF distance)
    float v;  // velocity (mm/cycle, negative = hand rising)
    float p;  // estimation uncertainty
    float lastMeas;  // last raw measurement (for differential velocity)
};
static Kalman1D kalman[5] = {
    {0, 0, 200.0f, -1.0f}, {0, 0, 200.0f, -1.0f}, {0, 0, 200.0f, -1.0f}, {0, 0, 200.0f, -1.0f}, {0, 0, 200.0f, -1.0f}
};
static const float KALMAN_Q = 25.0f;
static const float KALMAN_R = 100.0f;
static const float KALMAN_KV = 0.3f;
static const float LOOKAHEAD_K = 0.5f;
static const float MAX_LOOKAHEAD = 2.0f;
static const float CONF_THRESHOLD = 80.0f;
static const float V_MIN = 1.0f;
static const int16_t EXIT_HYSTERESIS = 5;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
static inline void kalmanUpdate(Kalman1D& k, float meas) {
    float x_pred = k.x + k.v;
    float p_pred = k.p + KALMAN_Q;
    float K = p_pred / (p_pred + KALMAN_R);
    float innov = meas - x_pred;
    k.x = x_pred + K * innov;
    // Differential velocity: immune to predict-step position drift
    if (k.lastMeas > 0.0f) {
        float dv = meas - k.lastMeas;
        if (dv > 50.0f) dv = 50.0f;       // clamp outliers
        if (dv < -50.0f) dv = -50.0f;
        k.v = k.v * (1.0f - KALMAN_KV) + KALMAN_KV * dv;
    }
    k.p = (1.0f - K) * p_pred;
    k.lastMeas = meas;
}


// ===== round49: Air key lock state machine =====
enum AirKeyState : uint8_t { AKS_IDLE, AKS_ACTIVE, AKS_COOLDOWN };
struct AirKeyTracker {
    AirKeyState state;
    uint32_t lastActiveMs;
    uint8_t cooldown;
};
static AirKeyTracker airKeyTracker[6] = {};
static bool tofValid[5] = {};
static uint32_t tofSampleMs[5] = {};
static constexpr uint32_t TOF_FRESH_MS = 200;


void initToFReset() {
    gpio_init(GPIO_TOF_RESET);
    gpio_set_dir(GPIO_TOF_RESET, true);
    gpio_pull_up(GPIO_TOF_RESET);
    gpio_put(GPIO_TOF_RESET, HIGH);
}

void resetToF() {
    gpio_put(GPIO_TOF_RESET, LOW);
    sleep_ms(1);
    gpio_put(GPIO_TOF_RESET, HIGH);
}

void initToF() {
    g_tofReadyMask = 0;
    initToFReset();
    resetToF();
    sleep_ms(2);
    // Probe all possible channels before assigning addresses. A fifth sensor
    // remains at 0x29 when the saved layout expects four channels.
    g_tofPhysicalMask = 0;
    for (uint8_t i = 0; i < 5; ++i) {
        watchdog_update();
        uint8_t reg = 0xc0, model = 0;
        if (mux0.setChannel(i) == 1 &&
            i2c_write_read(1, 0x29, &reg, 1, &model, 1) == 1 && model == 0xee) {
            g_tofPhysicalMask |= 1u << i;
        }
    }
    VL53L0X* tofs[5] = { &tof0, &tof1, &tof2, &tof3, &tof4 };
    int sensorCount = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 5 : 4;
    // Phase 1: init each sensor through mux, assign unique I2C address
    for (int i = 0; i < sensorCount; i++) {
        watchdog_update();
        if (mux0.setChannel(i) != 1) continue;
        tofs[i]->setI2CAddressOnly(0x29);
        tofs[i]->setTimeout(200);
        if (!tofs[i]->forceInit()) continue;
        // Keep the native default budget; the former 12ms request was rejected.
        tofs[i]->setAddress(0x30 + i);
        tofs[i]->startContinuous(0);
        g_tofReadyMask |= 1u << i;
        sleep_ms(5);  // stagger: distribute measurement completion phases
    }
    // Phase 2: enable all mux channels simultaneously (all sensors on bus)
    uint8_t muxMask = (sensorCount == 5) ? 0x1F : 0x0F;
    mux0.setReg(muxMask);
    sleep_ms(2);
    // Phase 3: verify all sensors respond at their new addresses
    useMuxScan = false;
    for (int i = 0; i < sensorCount; i++) {
        watchdog_update();
        if (!(g_tofReadyMask & (1u << i)) || !findI2CDevice(1, 0x30 + i, 10)) {
            useMuxScan = true;
            break;
        }
    }
    if (useMuxScan) {
        // Fallback: reset all sensors, re-init with mux-per-channel scanning
        resetToF();
        sleep_ms(2);
        g_tofReadyMask = 0;
        for (int i = 0; i < sensorCount; i++) {
            watchdog_update();
            if (mux0.setChannel(i) != 1) continue;
            tofs[i]->setI2CAddressOnly(0x29);
            tofs[i]->setTimeout(200);
            if (!tofs[i]->forceInit()) continue;
            // Keep the native default budget; the former 12ms request was rejected.
            tofs[i]->startContinuous(0);
            g_tofReadyMask |= 1u << i;
            sleep_ms(5);
        }
    }
    watchdog_update();
}

// ===== round64: lane (game view, 0-31) <-> electrode (chip index m, bit e) =====
// Forward layout tables mirror the touchData32[] assignment in updateTouch_v1
// (v1 layout: chips A/B/C for MBR3116 or mpr0/1/2, lanes 0-7 -> chip1, 8-15 ->
// chip0/chip1 interleaved, 16-23 -> chip0/chip2, 24-31 -> chip0/chip2) and
// updateTouch_v2 (v2 layout: chips D/E, lanes 0-15 -> chip1, 16-31 -> chip0).
// laneTable[m][e] holds the lane for each physical electrode in the active
// layout, -1 when the electrode is not part of the 32-lane slider (e.g. v2
// electrodes with no lane), so threshold lookups are O(1) in the hot loop.
static const uint8_t V1_LANE_M[32] = {
    1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2
};
static const uint8_t V1_LANE_E[32] = {
    11, 0, 10, 1, 9, 2, 8, 3, 11, 4, 10, 5, 9, 6, 8, 7,
    7, 0, 6, 1, 5, 2, 4, 3, 3, 4, 2, 5, 1, 6, 0, 7
};
static const uint8_t V2_LANE_M[32] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
static const uint8_t V2_LANE_E[32] = {
    4, 0, 5, 1, 6, 2, 7, 3, 8, 15, 9, 14, 10, 13, 11, 12,
    12, 11, 13, 10, 14, 9, 15, 8, 3, 7, 2, 6, 1, 5, 0, 4
};
static int16_t laneTable[3][16];  // -1 = electrode not mapped to a lane

static void buildLaneTable() {
    for (int m = 0; m < 3; m++)
        for (int e = 0; e < 16; e++) laneTable[m][e] = -1;
    bool v2 = (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4);
    const uint8_t* lm = v2 ? V2_LANE_M : V1_LANE_M;
    const uint8_t* le = v2 ? V2_LANE_E : V1_LANE_E;
    for (uint8_t lane = 0; lane < 32; lane++) {
        uint8_t m = lm[lane], e = le[lane];
        if (m < 3 && e < 16) laneTable[m][e] = lane;
    }
}

static uint8_t electrodeBaseTouchTh(uint8_t m, uint8_t e) {
    // round64: config-driven per-lane touch threshold. laneTable maps the
    // physical electrode back to its slider lane (game view); 0 = inherit the
    // global th_touch. This replaces the round46 unified-threshold skeleton:
    // the old per-electrode override table (round45b) is superseded by the
    // v1.6 per-key config fields.
    int16_t lane = laneTable[m][e];
    if (lane >= 0 && ControllerConfig.thTouchKey[lane] != 0) {
        return ControllerConfig.thTouchKey[lane];
    }
    return 0;  // inherit global ControllerConfig.th_touch
}

static uint8_t electrodeBaseReleaseTh(uint8_t m, uint8_t e) {
    // round64: release twin of electrodeBaseTouchTh. MPR121 only (MBR has no
    // per-sensor release register). 0 = inherit global th_release.
    int16_t lane = laneTable[m][e];
    if (lane >= 0 && ControllerConfig.thReleaseKey[lane] != 0) {
        return ControllerConfig.thReleaseKey[lane];
    }
    return 0;  // inherit global ControllerConfig.th_release
}

void initMPR121() {
    watchdog_update();
    mpr0.init(6, 3, true);
    watchdog_update();
    mpr1.init(6, 3, true);
    watchdog_update();
    mpr2.init(6, 3, true);
    watchdog_update();

    // Let auto-config run for 200ms to set optimal per-electrode charge current (CDC).
    // Without this delay, CDC may be 0 for some electrodes -> no touch detection.
    sleep_ms(200);

    // Freeze charge current by disabling auto-config BEFORE baseline calibration.
    // This prevents auto-config noise from triggering the "touched" check in
    // calibrateBaseline() stage 2, which would skip baseline setup entirely.
    mpr0.writeRegister(MPR121_AUTOCONFIG0, 0x00);
    mpr1.writeRegister(MPR121_AUTOCONFIG0, 0x00);
    mpr2.writeRegister(MPR121_AUTOCONFIG0, 0x00);

    // round46j: use ConfigApp values as-is - no forced minimums, no
    // per-electrode special-casing. ALL electrodes get the unified
    // th_touch/th_release that ConfigApp applied.
    uint8_t th_t = ControllerConfig.th_touch;
    uint8_t th_r = ControllerConfig.th_release;
    // round64: per-electrode thresholds driven by config (lane table lookup,
    // 0 = inherit global). Touch + release both written per electrode.
    for (uint8_t m = 0; m < 3; m++) {
        watchdog_update();
        for (uint8_t e = 0; e < 12; e++) {
            uint8_t base = electrodeBaseTouchTh(m, e);
            uint8_t baseR = electrodeBaseReleaseTh(m, e);
            uint8_t eth = (base > th_t) ? base : th_t;
            uint8_t erh = (baseR > th_r) ? baseR : th_r;
            if (m == 0) mpr0.setThresholdsForElectrode(e, eth, erh);
            else if (m == 1) mpr1.setThresholdsForElectrode(e, eth, erh);
            else mpr2.setThresholdsForElectrode(e, eth, erh);
        }
    }

    uint8_t dt, dr;
    dt = ControllerConfig.debounce & 0b1111;
    dr = (ControllerConfig.debounce >> 4) & 0b1111;
    mpr0.setDebounce(dt, dr);
    mpr1.setDebounce(dt, dr);
    mpr2.setDebounce(dt, dr);

    // Power-on baseline calibration: force baseline = current idle filtered value
    // so touch delta starts at 0. Skipped if any electrode is touched at boot.
    mpr0.calibrateBaseline();
    watchdog_update();
    mpr1.calibrateBaseline();
    watchdog_update();
    mpr2.calibrateBaseline();
    watchdog_update();
}

void initI2C() {
    initI2CBus(0, GPIO_I2C_0_SDA, GPIO_I2C_0_SCL, BR_I2C);
    initI2CBus(1, GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, BR_I2C);
}

void detectIR() {
    if (iox0.isConnected()) {
        usingIR = true;
    } else {
        usingIR = false;
    }
}

void initIR() {
    // 使用红外对射组件
    iox0.setConfP0(0x00);
    iox0.setConfP1(0xFF);
    // iox0.setOutputP0(0xFF);
}

void updateIR() {
    static int i = 0;
    // for (int i = 0; i < 6; i++) {
    uint8_t output_mask = 1 << i;
    iox0.setOutputP0(output_mask);
    sleep_us(100);
    uint8_t ir_state = iox0.getInputP1();
    airKeys[i] = ir_state & output_mask;
    // }
    i++;
    if (i == 6) {
        i = 0;
    }
}

void initHwDevices() {
    // round46b: hardware watchdog - if Core0 ever wedges (e.g. stuck I2C bus),
    // the chip auto-resets after 2s instead of freezing the game input forever.
    // Bounded startup stages also feed it; updateInputState() owns runtime feeds.
    watchdog_enable(2000, true);
    // round64: build the lane<->electrode inverse table BEFORE any threshold
    // consumer runs (initMPR121 / software verify layers).
    buildLaneTable();
    g_lampCount = (ControllerConfig.cfg0 & CFG0_BIT_FORCE16LEDS) ? 16 : 31;
    heightRange = (ControllerConfig.heightRangeCfg == 0) ? 10 : ControllerConfig.heightRangeCfg;
    for (int i = 0; i < 5; i++) { kalman[i].x = 0; kalman[i].v = 0; kalman[i].p = 200.0f; kalman[i].lastMeas = -1.0f; }
    for (int j = 0; j < 6; j++) { airKeyTracker[j].state = AKS_IDLE; airKeyTracker[j].cooldown = 0; }
    RGB_LED.fill(0, 0, 0);
    initI2C();
    detectIR();
    if (usingIR) {
        initIR();
    } else {
        mux0.init();
        initToF();
    }
    watchdog_update();
    if ((ControllerConfig.cfg0 & CFG0_BIT_MBR3116) || ControllerConfig.hwVer >= 3) {
        // MBR3116 runs its existing native NVRAM configuration.
    } else {
        initMPR121();
    }
    watchdog_update();
}

uint8_t touchData[4];
uint8_t touchData32[32];
uint16_t rawTouch[3] = {0, 0, 0};  // non-static for CMD_DEBUG_CHAIN access  // raw 16-bit touch data for stuck-key detection
uint16_t hwTouch[3] = {0, 0, 0};   // round45p: pre-verification hardware touch snapshot for CMD_DEBUG_CHAIN (zero I2C overhead)
uint32_t lastTouchedMs[3][12] = {0};  // round45m: last touch timestamp per electrode (baseline correction skips recently-touched)

TouchInputSnapshot publishedTouchState{};
MbrTouchTrace publishedMbrTrace{};
volatile uint32_t touchStateGen = 0;

// round46: telemetry shared from Core0 to Core1 (CMD_DEBUG_TELEMETRY = 0xC1)
uint8_t  g_verifyFail[36] = {0};          // per-electrode I2C verification rejections
uint32_t g_loopMinUs = 0xFFFFFFFF, g_loopMaxUs = 0, g_loopSumUs = 0, g_loopCount = 0;

// round66: real-time pressure snapshot (0-255 clamped diff per slider lane).
// Core0 fills this every 5ms while a pressure report session is on; Core1
// substitutes the published copy into the GET_INPUT 33B frame.
// Zero I2C cost while off.
// round84: rawReportMode (bool) became rawReportLevel (0xC5 session level):
// 0=off, 1=simulated (round80 report semantics), 2=raw unscaled.
uint8_t pressureSnap[32] = {0};
volatile uint8_t rawReportLevel = 0;  // set by Core1 (0xC5), auto-cleared on CDC disconnect
volatile bool gameRawEnabled = false;  // round68: cfg2 bit1 baseline (experimental)
static uint32_t pressureSnapLastMs = 0;
static const uint32_t PRESSURE_SNAP_INTERVAL_MS = 5;

// round89a: opt-in signal-domain gate. A recent coherent host read is not a
// new chip scan: SYNC is used only for consistency, never as a refresh ID.
// Disabled profiles perform no additional I2C reads. Core0 owns this state.
static const uint32_t MBR_DISTANCE_POLL_MS = 5;
static const uint32_t MBR_DISTANCE_MAX_READ_AGE_MS = 15;
volatile uint32_t g_mbrDistanceReadFailures = 0;
struct MbrDistanceSample {
    bool retryNeeded = false;
    uint16_t counts[16] = {0};
    uint32_t readStartedMs = 0;
    bool attempted = false;
    bool valid = false;
    MbrTraceChip quality{};
};
static MbrDistanceSample mbrDistanceSamples[3];
static MbrDistanceGateState mbrDistanceStates[3][16];
static bool mbrDistanceEnabledForFrame = false;
static bool mbrDistanceResetForFrame = false;
static bool mbrButtonValid[3] = {};
static uint32_t mbrButtonStartMs[3] = {}, mbrButtonEndMs[3] = {};
// Host policy for profile-off transport faults, not a contact-distance rule.
// Keep an already published ON through a short fault, without refreshing its
// confirmation/stretch timers. Expire from the first failing read's start.
static const uint32_t MBR_LEGACY_FAULT_HOLD_MS = 50;
struct MbrLegacyFaultState {
    bool active = false;
    uint32_t startedMs = 0;
};

static void prepareMbrDistanceGate() {
    static uint8_t previousProfile[8] = {0};
    static uint8_t previousLayout = 0;
    bool useMbr = (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) ||
                  ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4;
    bool enabled = useMbr && mbrDistanceGateEnabled(ControllerConfig);
    mbrDistanceResetForFrame = false;
    if (enabled != mbrDistanceEnabledForFrame ||
        (enabled && (previousLayout != ControllerConfig.hwVer ||
         std::memcmp(previousProfile, &ControllerConfig.mbrDistanceMagic,
                     sizeof(previousProfile)) != 0))) {
        mbrDistanceResetForFrame = true;
        for (uint8_t m = 0; m < 3; m++) {
            mbrDistanceSamples[m] = MbrDistanceSample{};
            for (uint8_t e = 0; e < 16; e++) mbrDistanceStates[m][e] = MbrDistanceGateState{};
        }
        std::memcpy(previousProfile, &ControllerConfig.mbrDistanceMagic, sizeof(previousProfile));
        previousLayout = ControllerConfig.hwVer;
    }
    mbrDistanceEnabledForFrame = enabled;
    for (uint8_t m = 0; m < 3; ++m) {
        mbrDistanceSamples[m].quality.flags &= ~MBR_TRACE_READ_THIS_FRAME;
        mbrButtonValid[m] = false;
        mbrButtonStartMs[m] = mbrButtonEndMs[m] = 0;
    }
}

static bool mbrDistanceReadAttempt(uint8_t m, CY8CMBR3116* chip) {
    MbrDistanceSample& sample = mbrDistanceSamples[m];
    uint32_t now = to_ms_since_boot(get_absolute_time());
    CY8CMBR3116::DifferenceReadInfo info{};
    sample.valid = chip->readDifferenceCounts(sample.counts, &info, true);
    MbrTraceChip& quality = sample.quality;
    quality.ioFailures += info.ioFailures;
    quality.syncMismatches += info.syncMismatches;
    quality.attemptEndMs = to_ms_since_boot(get_absolute_time());
    quality.flags |= MBR_TRACE_ATTEMPTED | MBR_TRACE_READ_THIS_FRAME;
    quality.flags &= ~MBR_TRACE_VALID;
    if (sample.valid) {
        quality.flags |= MBR_TRACE_VALID | MBR_TRACE_HAS_GOOD | MBR_TRACE_COHERENT;
        quality.goodStartMs = now;
        quality.goodEndMs = quality.attemptEndMs;
        quality.sync = info.sync;
        quality.rangeMask = 0;
        for (uint8_t e = 0; e < 16; ++e) {
            if (sample.counts[e] <= 255) quality.rangeMask |= 1u << e;
        }
        for (uint8_t e = 0; e < 16; e++) {
            int16_t lane = laneTable[m][e];
            if (lane >= 0) pressureSnap[lane] = sample.counts[e] > 255 ? 255 : sample.counts[e];
        }
    }
    return !sample.valid && info.ioFailures == 0 && info.syncMismatches == 1;
}

static void mbrDistanceReadSample(uint8_t m, CY8CMBR3116* chip) {
    MbrDistanceSample& sample = mbrDistanceSamples[m];
    sample.retryNeeded = false;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (!sample.attempted || now - sample.readStartedMs >= MBR_DISTANCE_POLL_MS) {
        sample.readStartedMs = now;
        sample.attempted = true;
        ++sample.quality.readCalls;
        sample.quality.attemptStartMs = now;
        sample.retryNeeded = mbrDistanceReadAttempt(m, chip);
        if (!sample.valid && !sample.retryNeeded) ++g_mbrDistanceReadFailures;
    }
}

// Retry a torn group after the other chips, within the same two-burst budget.
static void mbrDistanceRetrySample(uint8_t m, CY8CMBR3116* chip) {
    MbrDistanceSample& sample = mbrDistanceSamples[m];
    if (!sample.retryNeeded) return;
    sample.retryNeeded = false;
    mbrDistanceReadAttempt(m, chip);
    if (!sample.valid) ++g_mbrDistanceReadFailures;
}

static uint16_t readMbrButtons(uint8_t m, CY8CMBR3116* chip) {
    uint16_t value = 0;
    mbrButtonStartMs[m] = to_ms_since_boot(get_absolute_time());
    mbrButtonValid[m] = chip->get_BUTTON_STAT((uint8_t*)&value) == 0;
    mbrButtonEndMs[m] = to_ms_since_boot(get_absolute_time());
    return mbrButtonValid[m] ? value : 0;
}

static bool mbrLegacyFaultHold(MbrLegacyFaultState& state, bool fault, bool wasOn,
                               uint32_t readStartedMs, uint32_t now) {
    if (!fault) {
        state.active = false;
        return false;
    }
    if (!state.active) {
        state.active = true;
        state.startedMs = readStartedMs;
    }
    return wasOn && (now - state.startedMs) < MBR_LEGACY_FAULT_HOLD_MS;
}

// All chip reads finish before masks are formed at one common host time. A
// slow later chip must not leave an earlier, now over-age sample authorized.
static uint16_t mbrDistanceAllowedMask(uint8_t m, uint16_t hardwareBits, uint32_t now) {
    const MbrDistanceSample& sample = mbrDistanceSamples[m];
    bool usable = sample.valid && now - sample.readStartedMs <= MBR_DISTANCE_MAX_READ_AGE_MS;
    uint16_t allowed = 0;
    for (uint8_t e = 0; e < 16; e++) {
        bool mapped = laneTable[m][e] >= 0;
        if (mbrDistanceGateUpdate(ControllerConfig, sample.counts[e], usable && mapped,
                                  (hardwareBits & (1u << e)) != 0, mbrDistanceStates[m][e])) {
            allowed |= 1u << e;
        }
    }
    return allowed;
}

// round66: MPR121 bulk diff read (mirrors CMD_DEBUG_DIFF's 0xC2 path: 3 chips,
// filtered+baseline in 6 bulk transactions ~1ms). Writes pressureSnap clamped
// 0-255. Returns false on I2C error (snapshot kept stale).
static bool mprReadPressureSnap() {
    MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
    bool allOk = true;
    for (uint8_t m = 0; m < 3; m++) {
        uint8_t filt[24];
        uint8_t base[12];
        bool ok_filt = mprs[m]->readRegisters(MPR121_FILTDATA_0L, filt, 24);
        bool ok_base = mprs[m]->readRegisters(MPR121_BASELINE_0, base, 12);
        if (!ok_filt || !ok_base) {
            allOk = false;
            continue;
        }
        for (uint8_t e = 0; e < 12; e++) {
            uint16_t f = filt[e * 2] | ((uint16_t)filt[e * 2 + 1] << 8);
            uint16_t b = (uint16_t)base[e] << 2;
            int16_t diff = (int16_t)b - (int16_t)f;
            uint8_t v = (diff < 0) ? 0 : ((diff > 255) ? 255 : (uint8_t)diff);
            int16_t lane = laneTable[m][e];
            if (lane >= 0) pressureSnap[lane] = v;
        }
    }
    return allOk;
}

// MBR3116 native coherent diff group (35B including SYNC, 16x16-bit diff).
// v1 layout 3 chips / v2 layout 2 chips.
static bool mbrReadPressureSnap() {
    bool allOk = true;
    uint8_t numChips = (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4) ? 2 : 3;
    for (uint8_t m = 0; m < numChips; m++) {
        uint16_t diffs[16];
        CY8CMBR3116* chip = (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4)
                                ? (m == 0 ? &MBR3116D : &MBR3116E)
                                : (m == 0 ? &MBR3116A : (m == 1 ? &MBR3116B : &MBR3116C));
        if (chip->get_DIFFERENCE_COUNT_SENSOR(diffs) != 0) {
            allOk = false;
            continue;
        }
        for (uint8_t e = 0; e < 16; e++) {
            int16_t lane = laneTable[m][e];
            if (lane >= 0) {
                uint16_t d = diffs[e];
                pressureSnap[lane] = (d > 255) ? 255 : (uint8_t)d;
            }
        }
    }
    return allOk;
}

// round66: called from updateInputState() before publishing the completed scan.
// Throttled to every PRESSURE_SNAP_INTERVAL_MS; no-op while no pressure
// report session is active. round68: also on while cfg2 bit1 game-raw
// baseline is set.
static void updatePressureSnap() {
    // The gate publishes the same coherent count snapshot during touch scan,
    // independent of a panel session, and avoids a duplicate pressure read.
    if (mbrDistanceEnabledForFrame) return;
    if (rawReportLevel == 0 && !gameRawEnabled) return;
    uint32_t nowMs = to_ms_since_boot(get_absolute_time());
    if (nowMs - pressureSnapLastMs < PRESSURE_SNAP_INTERVAL_MS) return;
    pressureSnapLastMs = nowMs;
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        mbrReadPressureSnap();
    } else {
        mprReadPressureSnap();
    }
}

#define GET_BIT(UNUM, BIT) (UNUM & (1 << BIT))

bool touchData4k[4];
bool touchData6k[6];

void updateTouch_v2() {
    // round50: full optimization pipeline migrated from updateTouch_v1/MPR121
    uint16_t t0 = 0, t1 = 0;
    t0 = readMbrButtons(0, &MBR3116D);
    t1 = readMbrButtons(1, &MBR3116E);

    hwTouch[0] = t0; hwTouch[1] = t1; hwTouch[2] = 0;  // round50: pre-verification snapshot

    uint16_t distanceAllowed[2] = {0xFFFF, 0xFFFF};
    if (mbrDistanceEnabledForFrame) {
        mbrDistanceReadSample(0, &MBR3116D);
        mbrDistanceReadSample(1, &MBR3116E);
        mbrDistanceRetrySample(0, &MBR3116D);
        mbrDistanceRetrySample(1, &MBR3116E);
        uint32_t sampledAt = to_ms_since_boot(get_absolute_time());
        distanceAllowed[0] = mbrDistanceAllowedMask(0, t0, sampledAt);
        distanceAllowed[1] = mbrDistanceAllowedMask(1, t1, sampledAt);
        t0 &= distanceAllowed[0]; t1 &= distanceAllowed[1];
    }

    // round84: per-GAME-LANE stretched snapshot of the previous cycle. The
    // gate-anchoring check (laneNeighborConfirmed) is lane-space, so the
    // stretched bits from both chips are projected through laneTable once
    // per cycle -- cross-chip boundaries (e.g. lane 15/16) anchor naturally.
    static bool prevLaneStretched[32] = {0};

    static uint16_t prevStretched[2] = {0, 0};  // round50: slide-aware spatial filtering
    static const uint8_t CONFIRM_CYCLES_V2 = 2;
    static uint8_t confirmReq[2][16] = {0};
    static uint8_t verifiedCount[2][16] = {0};
    static uint32_t lastTouchedMsV2[2][16] = {0};
    static bool fastOkV2[2][16] = {0};  // round80: strong flick first-cycle output flag
    if (mbrDistanceResetForFrame) {
        std::memset(prevLaneStretched, 0, sizeof(prevLaneStretched));
        std::memset(prevStretched, 0, sizeof(prevStretched));
        std::memset(confirmReq, 0, sizeof(confirmReq));
        std::memset(verifiedCount, 0, sizeof(verifiedCount));
        std::memset(lastTouchedMsV2, 0, sizeof(lastTouchedMsV2));
        std::memset(fastOkV2, 0, sizeof(fastOkV2));
    }
    uint32_t nowVer = to_ms_since_boot(get_absolute_time());
    uint16_t legacyFaultMask[2] = {0, 0};
    uint32_t legacyReadStartedMs[2] = {mbrButtonStartMs[0], mbrButtonStartMs[1]};
    // round64: per-lane MBR thresholds. base_k defaults to 80 (=round50
    // MBR3116_VERIFY_TH); amplitude tiers are relative offsets off it, so
    // per-key values only tighten; native BUTTON_STAT remains a separate gate.
        static const uint16_t MBR_VERIFY_BASE = 80;
        static const uint16_t MBR_TIER_OFFSET_MEDIUM = 40;
        static const uint16_t MBR_TIER_OFFSET_STRONG = 120;
        static const uint16_t MBR_TIER_OFFSET_SKIP = 220;
        // round80: preserve the legacy strong-signal fast path. Native button
        // DIFF is 0..255, and 200 is a count threshold, not evidence of contact
        // or a fixed height. The legacy +220 skip tier is retained for default-
        // off compatibility; round89a rejects invalid values above 255.
        static const uint16_t MBR_FLICK_FAST_TH = 200;
        // round84: gate value granted to a slide-edge lane whose GAME-LANE
        // neighbor confirmed last cycle (see laneNeighborConfirmed below).
        // = MBR_TIER_OFFSET_MEDIUM, so the relaxed gate coincides with the
        // existing medium-amplitude tier boundary.
        static const uint16_t MBR_NEIGHBOR_GATE_MARGIN = 40;
        uint16_t verifyBaseK[2][16];
        for (uint8_t m = 0; m < 2; m++) {
            for (uint8_t e = 0; e < 16; e++) {
                int16_t lane = laneTable[m][e];
                uint16_t base = (lane >= 0 && ControllerConfig.thTouchKey[lane] != 0)
                                    ? ControllerConfig.thTouchKey[lane]
                                    : MBR_VERIFY_BASE;
                // round64: MBR can only tighten. The chip's hardware gate is
                // 128; a config value <=128 can never be stricter than that, so
                // keep the round50 verify baseline (80) for those to avoid
                // silently relaxing the software gate.
                verifyBaseK[m][e] = (base > MBR_VERIFY_BASE) ? base : MBR_VERIFY_BASE;
            }
        }
    {   // round50: software verification
        uint16_t raw[2] = {t0, t1};
        CY8CMBR3116* chips[2] = {&MBR3116D, &MBR3116E};
        uint16_t diffCounts[2][16];
        bool diffRead[2] = {false, false};
        bool diffOk[2] = {false, false};
        bool diffRetried[2] = {false, false};
        for (uint8_t m = 0; m < 2; m++) {
            for (uint8_t e = 0; e < 16; e++) {
                if (mbrDistanceEnabledForFrame && !(distanceAllowed[m] & (1u << e))) {
                    verifiedCount[m][e] = 0;
                    confirmReq[m][e] = CONFIRM_CYCLES_V2;
                    fastOkV2[m][e] = false;
                    lastTouchedMsV2[m][e] = 0;
                    continue;
                }
                if (raw[m] & (1 << e)) {
                    if (verifiedCount[m][e] < 2 ||
                        (!mbrDistanceEnabledForFrame && !(prevStretched[m] & (1u << e)))) {
                        if (!diffRead[m]) {
                            if (mbrDistanceEnabledForFrame) {
                                diffOk[m] = mbrDistanceSamples[m].valid;
                                std::memcpy(diffCounts[m], mbrDistanceSamples[m].counts, sizeof(diffCounts[m]));
                            } else {
                                legacyReadStartedMs[m] = to_ms_since_boot(get_absolute_time());
                                diffOk[m] = (chips[m]->get_DIFFERENCE_COUNT_SENSOR(diffCounts[m]) == 0);
                                if (!diffOk[m]) {
                                    diffRetried[m] = true;
                                    diffOk[m] = (chips[m]->get_DIFFERENCE_COUNT_SENSOR(diffCounts[m]) == 0);
                                }
                            }
                            diffRead[m] = true;
                        }
                        if (!diffOk[m]) {
                            raw[m] &= ~(1u << e);
                            legacyFaultMask[m] |= 1u << e;
                            confirmReq[m][e] = CONFIRM_CYCLES_V2;
                        } else {
                            uint16_t diff = diffCounts[m][e];
                            if (!mbrDistanceEnabledForFrame && diff > 255) {
                                raw[m] &= ~(1u << e);
                                legacyFaultMask[m] |= 1u << e;
                                continue;
                            }
                            uint16_t baseK = verifyBaseK[m][e];
                            // round80: hover-rejection gate (mbrTouchGate, 0=off).
                            // Legacy new-touch verification only: no physical
                            // distance guarantee, and held/release/sticky state
                            // is untouched. The opt-in round89a gate has already
                            // applied a continuous per-electrode hard rejection.
                            // round84: confirmed-neighbor anchoring. A slide's
                            // trailing/leading edge sweeps weak signal into a
                            // lane next to one that JUST confirmed -- spatial
                            // continuity for legacy slide behavior. A hovering
                            // finger next to real contact can share this anchor.
                            // Anchor is the GAME LANE (k +/- 1 via laneTable),
                            // never "any electrode on the same chip": 3116
                            // electrodes are physically shuffled (v2 puts tc1/2
                            // and tc15/16 on one chip), a same-chip rule would
                            // let a hover on one side slip through while the
                            // other side is held. This relaxation cannot bypass
                            // the opt-in continuous gate applied before this loop.
                            uint16_t rejectTh = baseK;
                            if (ControllerConfig.mbrTouchGate > rejectTh) rejectTh = ControllerConfig.mbrTouchGate;
                            int16_t lane = laneTable[m][e];
                            bool laneNeighborConfirmed = false;
                            if (lane >= 0) {
                                laneNeighborConfirmed =
                                    (lane > 0 && prevLaneStretched[lane - 1]) ||
                                    (lane < 31 && prevLaneStretched[lane + 1]);
                            }
                            if (laneNeighborConfirmed && rejectTh > baseK + MBR_NEIGHBOR_GATE_MARGIN) {
                                rejectTh = baseK + MBR_NEIGHBOR_GATE_MARGIN;
                            }
                            bool neighborWasActive = (e > 0 && (prevStretched[m] & (1 << (e - 1)))) ||
                                                     (e < 15 && (prevStretched[m] & (1 << (e + 1))));
                            if (diff < rejectTh) {
                                raw[m] &= ~(1 << e);
                                verifiedCount[m][e] = 0;
                                confirmReq[m][e] = CONFIRM_CYCLES_V2;
                                fastOkV2[m][e] = false;
                                if (g_verifyFail[m * 16 + e] < 255) g_verifyFail[m * 16 + e]++;
                            } else if (verifiedCount[m][e] >= 2) {
                                // A recent confirmation can keep its cadence only
                                // after a fresh valid read when output was OFF.
                            } else if (verifiedCount[m][e] == 0 && !diffRetried[m] &&
                                       diff >= MBR_FLICK_FAST_TH) {
                                // round80: strong flick fast path -- first-cycle
                                // output restores the old firmware's zero-latency
                                // behavior for isolated flick taps. The isolated
                                // +1 penalty is waived via fastOkV2 in the confirm
                                // loop, still guarded by the 40ms release-bounce
                                // window there. Gate interplay: with mbrTouchGate
                                // >= 200 the reject check above lifts the effective
                                // fast threshold to the gate value (expected).
                                verifiedCount[m][e] = 2;
                                confirmReq[m][e] = 1;
                                fastOkV2[m][e] = true;
                            } else if (verifiedCount[m][e] == 0 && !diffRetried[m] &&
                                       (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_SKIP) || neighborWasActive)) {
                                verifiedCount[m][e] = 2;
                                confirmReq[m][e] = 1;
                            } else {
                                verifiedCount[m][e]++;
                                if (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_STRONG))      confirmReq[m][e] = 1;
                                else if (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_MEDIUM)) confirmReq[m][e] = 2;
                                else                                                         confirmReq[m][e] = 3;
                            }
                        }
                    }
                } else {
                    if (verifiedCount[m][e] != 0) {
                        if (lastTouchedMsV2[m][e] == 0 || (nowVer - lastTouchedMsV2[m][e]) > 50) {
                            verifiedCount[m][e] = 0;
                            confirmReq[m][e] = CONFIRM_CYCLES_V2;
                            fastOkV2[m][e] = false;
                        }
                    }
                }
            }
        }
        if (!mbrDistanceEnabledForFrame) {
            uint32_t checkedAt = to_ms_since_boot(get_absolute_time());
            for (uint8_t m = 0; m < 2; ++m) {
                if (!mbrButtonValid[m] ||
                    checkedAt - mbrButtonStartMs[m] > MBR_DISTANCE_MAX_READ_AGE_MS) {
                    legacyFaultMask[m] = 0xFFFF;
                    legacyReadStartedMs[m] = mbrButtonStartMs[m];
                    raw[m] = 0;
                }
            }
        }
        t0 = raw[0]; t1 = raw[1];
    }
    rawTouch[0] = t0; rawTouch[1] = t1; rawTouch[2] = 0;
    {   // round50: pulse stretching + sticky touch + dip tolerance + spatial filtering
        static const uint32_t STRETCH_MS_V2 = 5;
        static const uint8_t DIP_TOLERANCE_CYCLES_V2 = 3;
        static const uint8_t STICKY_THRESHOLD_V2 = 15;
        static const uint8_t STICKY_GRACE_MAX_V2 = 15;
        static const uint8_t STICKY_GRACE_SHORT_V2 = 12;
        static uint8_t touchCount[2][16] = {0};
        static uint32_t lastConfirmed[2][16] = {0};
        static uint8_t dipGrace[2][16] = {0};
        static uint8_t stickyGrace[2][16] = {0};
        static MbrLegacyFaultState legacyFaults[2][16];
        if (mbrDistanceResetForFrame) {
            std::memset(touchCount, 0, sizeof(touchCount));
            std::memset(lastConfirmed, 0, sizeof(lastConfirmed));
            std::memset(dipGrace, 0, sizeof(dipGrace));
            std::memset(stickyGrace, 0, sizeof(stickyGrace));
            for (uint8_t m = 0; m < 2; ++m)
                for (uint8_t e = 0; e < 16; ++e) legacyFaults[m][e] = MbrLegacyFaultState{};
        }
        uint16_t raw[2] = {t0, t1};
        uint16_t stretched[2] = {0, 0};
        uint32_t now = to_ms_since_boot(get_absolute_time());
        for (uint8_t m = 0; m < 2; m++) {
            for (uint8_t e = 0; e < 16; e++) {
                bool fault = !mbrDistanceEnabledForFrame && (legacyFaultMask[m] & (1u << e));
                bool hold = mbrLegacyFaultHold(legacyFaults[m][e], fault,
                    (prevStretched[m] & (1u << e)) != 0, legacyReadStartedMs[m], now);
                if (fault) {
                    if (hold) {
                        stretched[m] |= 1u << e;
                    } else {
                        touchCount[m][e] = dipGrace[m][e] = stickyGrace[m][e] = 0;
                        lastConfirmed[m][e] = lastTouchedMsV2[m][e] = 0;
                        verifiedCount[m][e] = 0;
                        confirmReq[m][e] = CONFIRM_CYCLES_V2;
                        fastOkV2[m][e] = false;
                    }
                    continue;
                }
                if (mbrDistanceEnabledForFrame && !(distanceAllowed[m] & (1u << e))) {
                    touchCount[m][e] = 0;
                    lastConfirmed[m][e] = 0;
                    dipGrace[m][e] = 0;
                    stickyGrace[m][e] = 0;
                    continue;
                }
                bool isTouched = (raw[m] >> e) & 1;
                if (isTouched) {
                    lastTouchedMsV2[m][e] = now;
                    dipGrace[m][e] = DIP_TOLERANCE_CYCLES_V2;
                    if (touchCount[m][e] < 255) touchCount[m][e]++;
                    // round50fix: match v1 stickyGrace logic (duration-based, no decrement during touch)
                    if (touchCount[m][e] >= STICKY_THRESHOLD_V2) {
                        stickyGrace[m][e] = (touchCount[m][e] >= 50) ? STICKY_GRACE_MAX_V2
                                                                      : STICKY_GRACE_SHORT_V2;
                    }
                    uint8_t reqCycles = confirmReq[m][e] ? confirmReq[m][e] : CONFIRM_CYCLES_V2;
                    // round50fix: spatial penalty - isolated electrode needs +1 confirm cycle
                    bool hasNeighbor = ((e > 0 && (prevStretched[m] & (1 << (e - 1)))) ||
                                        (e < 15 && (prevStretched[m] & (1 << (e + 1)))));
                    // round80: waive the isolated +1 for strong flick signals
                    // (fastOkV2, set only when diff >= MBR_FLICK_FAST_TH on a
                    // first-sighting verify) unless we are within 40ms of the
                    // last confirmed touch -- a residual-signal re-enhancement
                    // right after release must not double-fire as a new tap.
                    bool fastExempt = fastOkV2[m][e] &&
                        (lastConfirmed[m][e] == 0 || (now - lastConfirmed[m][e]) >= 40);
                    if (!hasNeighbor && !fastExempt) reqCycles += 1;
                    if (touchCount[m][e] >= reqCycles) {
                        lastConfirmed[m][e] = now;
                        stretched[m] |= (1 << e);
                    } else if (touchCount[m][e] >= STICKY_THRESHOLD_V2 && stickyGrace[m][e] > 0) {
                        stickyGrace[m][e]--;
                        stretched[m] |= (1 << e);
                    } else if (lastConfirmed[m][e] != 0 && touchCount[m][e] >= 3 && dipGrace[m][e] > 0) {
                        dipGrace[m][e]--;
                        stretched[m] |= (1 << e);
                    }
                } else {
                    if (touchCount[m][e] >= STICKY_THRESHOLD_V2 && stickyGrace[m][e] > 0) {
                        stickyGrace[m][e]--;
                        stretched[m] |= (1 << e);
                    } else if (lastConfirmed[m][e] != 0 && touchCount[m][e] >= 3 && dipGrace[m][e] > 0) {
                        dipGrace[m][e]--;
                        stretched[m] |= (1 << e);
                    } else {
                        touchCount[m][e] = 0;
                        stickyGrace[m][e] = 0;
                        if (lastConfirmed[m][e] != 0 && (now - lastConfirmed[m][e]) < STRETCH_MS_V2) {
                            stretched[m] |= (1 << e);
                        }
                    }
                }
            }
        }
        t0 = stretched[0]; t1 = stretched[1];
    }
    prevStretched[0] = t0; prevStretched[1] = t1;

    // round84: project this cycle's stretched bits into lane space for next
    // cycle's laneNeighborConfirmed gate anchoring.
    for (uint8_t lane = 0; lane < 32; lane++) {
        uint8_t lm = V2_LANE_M[lane], le = V2_LANE_E[lane];
        uint16_t bits = (lm == 0) ? t0 : t1;
        prevLaneStretched[lane] = (bits >> le) & 1;
    }

    touchData32[0] = GET_BIT(t1, 4) ? 128 : 0;
    touchData32[1] = GET_BIT(t1, 0) ? 128 : 0;

    touchData32[2] = GET_BIT(t1, 5) ? 128 : 0;
    touchData32[3] = GET_BIT(t1, 1) ? 128 : 0;

    touchData32[4] = GET_BIT(t1, 6) ? 128 : 0;
    touchData32[5] = GET_BIT(t1, 2) ? 128 : 0;

    touchData32[6] = GET_BIT(t1, 7) ? 128 : 0;
    touchData32[7] = GET_BIT(t1, 3) ? 128 : 0;

    //

    touchData32[8] = GET_BIT(t1, 8) ? 128 : 0;
    touchData32[9] = GET_BIT(t1, 15) ? 128 : 0;

    touchData32[10] = GET_BIT(t1, 9) ? 128 : 0;
    touchData32[11] = GET_BIT(t1, 14) ? 128 : 0;

    touchData32[12] = GET_BIT(t1, 10) ? 128 : 0;
    touchData32[13] = GET_BIT(t1, 13) ? 128 : 0;

    touchData32[14] = GET_BIT(t1, 11) ? 128 : 0;
    touchData32[15] = GET_BIT(t1, 12) ? 128 : 0;

    //

    touchData32[16] = GET_BIT(t0, 12) ? 128 : 0;
    touchData32[17] = GET_BIT(t0, 11) ? 128 : 0;

    touchData32[18] = GET_BIT(t0, 13) ? 128 : 0;
    touchData32[19] = GET_BIT(t0, 10) ? 128 : 0;

    touchData32[20] = GET_BIT(t0, 14) ? 128 : 0;
    touchData32[21] = GET_BIT(t0, 9) ? 128 : 0;

    touchData32[22] = GET_BIT(t0, 15) ? 128 : 0;
    touchData32[23] = GET_BIT(t0, 8) ? 128 : 0;

    //

    touchData32[24] = GET_BIT(t0, 3) ? 128 : 0;
    touchData32[25] = GET_BIT(t0, 7) ? 128 : 0;

    touchData32[26] = GET_BIT(t0, 2) ? 128 : 0;
    touchData32[27] = GET_BIT(t0, 6) ? 128 : 0;

    touchData32[28] = GET_BIT(t0, 1) ? 128 : 0;
    touchData32[29] = GET_BIT(t0, 5) ? 128 : 0;

    touchData32[30] = GET_BIT(t0, 0) ? 128 : 0;
    touchData32[31] = GET_BIT(t0, 4) ? 128 : 0;

    // round50: set touchData[0..3] AFTER touchData32[] (same fix as round45t in v1)
    touchData[0] = *(0 + (uint8_t*)(&t0));
    touchData[1] = *(1 + (uint8_t*)(&t0));
    touchData[2] = *(0 + (uint8_t*)(&t1));
    touchData[3] = *(1 + (uint8_t*)(&t1));
}

void updateTouch_v1() {

    uint16_t t0 = 0, t1 = 0, t2 = 0;  // round50: init to 0 for I2C error safety
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        t0 = readMbrButtons(0, &MBR3116A);
        t1 = readMbrButtons(1, &MBR3116B);
        t2 = readMbrButtons(2, &MBR3116C);
    } else {
        t0 = mpr0.touched();
        t1 = mpr1.touched();
        t2 = mpr2.touched();
    }

    // round45p: save pre-verification hardware touch snapshot for CMD_DEBUG_CHAIN
    hwTouch[0] = t0; hwTouch[1] = t1; hwTouch[2] = t2;

    uint16_t distanceAllowed[3] = {0xFFFF, 0xFFFF, 0xFFFF};
    if (mbrDistanceEnabledForFrame) {
        mbrDistanceReadSample(0, &MBR3116A);
        mbrDistanceReadSample(1, &MBR3116B);
        mbrDistanceReadSample(2, &MBR3116C);
        mbrDistanceRetrySample(0, &MBR3116A);
        mbrDistanceRetrySample(1, &MBR3116B);
        mbrDistanceRetrySample(2, &MBR3116C);
        uint32_t sampledAt = to_ms_since_boot(get_absolute_time());
        distanceAllowed[0] = mbrDistanceAllowedMask(0, t0, sampledAt);
        distanceAllowed[1] = mbrDistanceAllowedMask(1, t1, sampledAt);
        distanceAllowed[2] = mbrDistanceAllowedMask(2, t2, sampledAt);
        t0 &= distanceAllowed[0]; t1 &= distanceAllowed[1]; t2 &= distanceAllowed[2];
    }

    // round84: per-GAME-LANE stretched snapshot of the previous cycle (MBR
    // branch only -- the MPR branch keeps its behavior untouched this round).
    // Projected after the shared stretch stage from the STRETCHED bits
    // (incl. sticky-held lanes); cross-chip lane borders (e.g. 15/16) anchor
    // naturally through lane space.
    static bool prevLaneStretched[32] = {0};

    // round45u: previous cycle's stretched output for slide-aware spatial filtering.
    // Used to detect slide transitions: if neighbor was active (including sticky-maintained),
    // current cell is part of a slide, not an isolated noise spike.
    static uint16_t prevStretched[3] = {0, 0, 0};

    // Save raw 16-bit touch data for stuck-key detection (before stretching)
    // Verify touches against actual sensor data. Only verify the first 2 cycles
    // of a new touch (sustained touches skip I2C reads to minimize latency).
   // Filters false touches from MPR121 timing mismatch, baseline drift, noise.
   static const uint8_t  CONFIRM_CYCLES = 2;  // round36: 3->2, software verification handles noise filtering
    static uint8_t confirmReq[3][12] = {0};     // round45: amplitude-tier confirmation cycles (1=fast, 2=normal, 3=strict)
   static uint8_t verifiedCount[3][12] = {0};     // round50: shared MPR121/MBR3116 verification
   static bool fastOk[3][12] = {0};  // round80: strong flick first-cycle output flag (shared MPR/MBR)
    if (mbrDistanceResetForFrame) {
        std::memset(prevLaneStretched, 0, sizeof(prevLaneStretched));
        std::memset(prevStretched, 0, sizeof(prevStretched));
        std::memset(confirmReq, 0, sizeof(confirmReq));
        std::memset(verifiedCount, 0, sizeof(verifiedCount));
        std::memset(lastTouchedMs, 0, sizeof(lastTouchedMs));
        std::memset(fastOk, 0, sizeof(fastOk));
    }
   uint32_t nowVer = to_ms_since_boot(get_absolute_time());  // round45r: sticky dip verification preservation
   const bool legacyMbr = (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) && !mbrDistanceEnabledForFrame;
   uint16_t legacyFaultMask[3] = {0, 0, 0};
   uint32_t legacyReadStartedMs[3] = {mbrButtonStartMs[0], mbrButtonStartMs[1], mbrButtonStartMs[2]};
   if (!(ControllerConfig.cfg0 & CFG0_BIT_MBR3116)) {
        uint16_t raw[3] = {t0, t1, t2};
        MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
        // round64: per-lane software thresholds. sw_th_k = per-key value if
        // configured (non-zero), else the global th_touch; floor of 4 kept so
        // verify thresholds never collapse to 0.
        uint8_t sw_th_k[3][12];
        uint8_t verify_th_k[3][12];
        // round80: strong flick fast path base. User empirical data: MPR diff
        // is contact-area proportional (finger 30-50, palm ~130) and never
        // reaches the 255 cap natively. 50 raw (=100 on the x2 report scale)
        // covers firm finger/palm taps while keeping >=3.5x margin over the
        // worst observed idle-noise pulses (diff>=14, round47b). max() with
        // sw+4 (the confirmReq=1 tier boundary) preserves per-key tightened
        // thresholds -- the fast path never bypasses a user-raised gate.
        static const uint8_t MPR_FLICK_FAST_BASE = 50;
        for (uint8_t m = 0; m < 3; m++) {
            for (uint8_t e = 0; e < 12; e++) {
                uint8_t base = electrodeBaseTouchTh(m, e);
                uint8_t sw = (base != 0) ? base : ControllerConfig.th_touch;
                if (sw < 4) sw = 4;
                sw_th_k[m][e] = sw;
                verify_th_k[m][e] = (sw > 1) ? (sw - 1) : 1;  // round41: -1 LSB tolerance
            }
        }
      for (uint8_t m = 0; m < 3; m++) {
            for (uint8_t e = 0; e < 12; e++) {
                if (raw[m] & (1 << e)) {
                    if (verifiedCount[m][e] < 2) {
                        // New touch - verify with sensor data
                        uint16_t filt = mprs[m]->filteredData(e);
                       if (filt == 0) {
                            // round47b-patch: I2C read error - re-read once before trusting.
                            // Old code directly trusted hardware (verifiedCount++), which lets
                            // a hardware false touch through if I2C fails 2 cycles in a row.
                            // Now: re-read; if still 0, skip this electrode this cycle (don't
                            // increment verifiedCount, don't clear raw - let next cycle retry).
                            uint16_t filt2 = mprs[m]->filteredData(e);
                            if (filt2 == 0) {
                                // Failed evidence must not advance a new ON confirmation.
                                raw[m] &= ~(1 << e);
                                verifiedCount[m][e] = 0;
                                fastOk[m][e] = false;
                                confirmReq[m][e] = CONFIRM_CYCLES;
                            } else {
                                // First read was glitch, second OK - use filt2
                                filt = filt2;
                                uint16_t base = mprs[m]->baselineData(e);
                                int16_t diff = (int16_t)base - (int16_t)filt;
                            if (diff < static_cast<int16_t>(verify_th_k[m][e])) {
                                    raw[m] &= ~(1 << e);
                                    verifiedCount[m][e] = 0;
                                    confirmReq[m][e] = CONFIRM_CYCLES;
                                    fastOk[m][e] = false;
                                    if (g_verifyFail[m * 12 + e] < 255) g_verifyFail[m * 12 + e]++;
                                } else {
                                    // round47b-patch: intentionally no strong-signal fast-path
                                    // (diff>=sw_th+6 skip). First read was 0 (I2C unstable),
                                    // so require full 2-cycle verification.
                                    verifiedCount[m][e]++;
                                    if (diff >= (int16_t)(sw_th_k[m][e] + 4))      confirmReq[m][e] = 1;
                                    else if (diff >= (int16_t)(sw_th_k[m][e] + 2)) confirmReq[m][e] = 2;
                                    else                                        confirmReq[m][e] = 3;
                                }
                            }
                       } else {
                            uint16_t base = mprs[m]->baselineData(e);
                            int16_t diff = (int16_t)base - (int16_t)filt;
                               if (diff < static_cast<int16_t>(verify_th_k[m][e])) {
                               raw[m] &= ~(1 << e);
                               verifiedCount[m][e] = 0;
                               confirmReq[m][e] = CONFIRM_CYCLES;
                               fastOk[m][e] = false;
                               if (g_verifyFail[m * 12 + e] < 255) g_verifyFail[m * 12 + e]++;  // round46: telemetry
                           } else {
                                // round45o: skip second verification for very strong signals (saves 1 cycle ~2ms for flick notes)
                                // round45u: slide transition acceleration - if neighbor was active
                                // in previous cycle, skip 2nd verification (slide, not noise)
                                bool neighborWasActive = (e > 0 && (prevStretched[m] & (1 << (e-1)))) ||
                                                          (e < 11 && (prevStretched[m] & (1 << (e+1))));
                                // round80: strong flick fast path -- first-cycle output
                                // for isolated firm taps (see MPR_FLICK_FAST_BASE). The
                                // isolated +1 penalty is waived via fastOk in the confirm
                                // loop, still guarded by the 40ms release-bounce window.
                                // The sw+6/neighbor slide path below stays NON-exempt:
                                // it fires on weak slide signals where the penalty is
                                // the only noise filter.
                                uint8_t fastTh = MPR_FLICK_FAST_BASE;
                                if (sw_th_k[m][e] + 4 > fastTh) fastTh = sw_th_k[m][e] + 4;
                                if (diff >= (int16_t)fastTh) {
                                    verifiedCount[m][e] = 2;
                                    confirmReq[m][e] = 1;
                                    fastOk[m][e] = true;
                                } else if (diff >= (int16_t)(sw_th_k[m][e] + 6) || neighborWasActive) {
                                    verifiedCount[m][e] = 2;  // skip second I2C verification cycle
                                    confirmReq[m][e] = 1;     // fastest confirmation
                                } else {
                                    verifiedCount[m][e]++;
                                    if (diff >= (int16_t)(sw_th_k[m][e] + 4))      confirmReq[m][e] = 1;  // strong signal: fastest
                                    else if (diff >= (int16_t)(sw_th_k[m][e] + 2)) confirmReq[m][e] = 2;  // medium
                                    else                                         confirmReq[m][e] = 3;  // weak/edge: strict, anti-false-touch
                                }
                           }
                        }
                    }
                   // else: sustained touch, already verified, skip I2C
               } else {
                   // round45r: preserve verification if recently touched (sticky dip recovery).
                   // When touch returns from sticky dip, verifiedCount is still 2 -> skip re-verification.
                   // This prevents miss when re-verification would fail due to weak signal or I2C timing mismatch.
                     // 50ms window covers max sticky dip (18 cycles ~47ms at
                     // 2.7ms/cycle), while fast re-taps (release <50ms ago)
                     // skip re-verification for zero-latency re-trigger.
                     if (lastTouchedMs[m][e] == 0 || (nowVer - lastTouchedMs[m][e]) > 50) {
                       verifiedCount[m][e] = 0;
                       confirmReq[m][e] = CONFIRM_CYCLES;
                       fastOk[m][e] = false;
                   }
                   // else: keep verifiedCount, instant re-trigger when touch returns
               }
            }
        }
        t0 = raw[0]; t1 = raw[1]; t2 = raw[2];
    } else {
        // round50: MBR3116 software verification (migrated from MPR121 optimization).
        // Uses get_DIFFERENCE_COUNT_SENSOR() to verify chip's touch decision against
        // actual signal strength. Catches I2C errors, false positives from timing
        // mismatch between BUTTON_STAT and DIFFERENCE_COUNT reads.
        // MBR3116 hardware threshold is 128 (config); verify threshold 80 gives
        // ~37% margin for I2C read timing skew (equivalent to MPR121's sw_th-1).
        uint16_t raw[3] = {t0, t1, t2};
        CY8CMBR3116* chips[3] = {&MBR3116A, &MBR3116B, &MBR3116C};
        uint8_t maxElec[3] = {12, 12, 8};  // electrodes used per chip in v1 layout
        // round64: per-lane MBR thresholds. base_k defaults to 80 (=round50
        // MBR3116_VERIFY_TH); amplitude tiers are relative offsets off it, so
        // per-key values only tighten; native BUTTON_STAT remains a separate gate.
        static const uint16_t MBR_VERIFY_BASE = 80;
        static const uint16_t MBR_TIER_OFFSET_MEDIUM = 40;
        static const uint16_t MBR_TIER_OFFSET_STRONG = 120;
        static const uint16_t MBR_TIER_OFFSET_SKIP = 220;
        // round80: preserve the legacy strong-signal fast path. Native button
        // DIFF is 0..255, and 200 is a count threshold, not evidence of contact
        // or a fixed height. The legacy +220 skip tier is retained for default-
        // off compatibility; round89a rejects invalid values above 255.
        static const uint16_t MBR_FLICK_FAST_TH = 200;
        // round84: gate value granted to a slide-edge lane whose GAME-LANE
        // neighbor confirmed last cycle (mirror of the v2 layout change).
        static const uint16_t MBR_NEIGHBOR_GATE_MARGIN = 40;
        uint16_t verifyBaseK[3][16];
        for (uint8_t m = 0; m < 3; m++) {
            for (uint8_t e = 0; e < 16; e++) {
                int16_t lane = laneTable[m][e];
                uint16_t base = (lane >= 0 && ControllerConfig.thTouchKey[lane] != 0)
                                    ? ControllerConfig.thTouchKey[lane]
                                    : MBR_VERIFY_BASE;
                // round64: MBR can only tighten. The chip's hardware gate is
                // 128; a config value <=128 can never be stricter than that, so
                // keep the round50 verify baseline (80) for those to avoid
                // silently relaxing the software gate.
                verifyBaseK[m][e] = (base > MBR_VERIFY_BASE) ? base : MBR_VERIFY_BASE;
            }
        }
        uint16_t diffCounts[3][16];
        bool diffRead[3] = {false, false, false};
        bool diffOk[3] = {false, false, false};
        bool diffRetried[3] = {false, false, false};
        for (uint8_t m = 0; m < 3; m++) {
            for (uint8_t e = 0; e < maxElec[m]; e++) {
                if (mbrDistanceEnabledForFrame && !(distanceAllowed[m] & (1u << e))) {
                    verifiedCount[m][e] = 0;
                    confirmReq[m][e] = CONFIRM_CYCLES;
                    fastOk[m][e] = false;
                    lastTouchedMs[m][e] = 0;
                    continue;
                }
                if (raw[m] & (1 << e)) {
                    if (verifiedCount[m][e] < 2 ||
                        (!mbrDistanceEnabledForFrame && !(prevStretched[m] & (1u << e)))) {
                        // New touch - verify with sensor difference count
                        if (!diffRead[m]) {
                            if (mbrDistanceEnabledForFrame) {
                                diffOk[m] = mbrDistanceSamples[m].valid;
                                std::memcpy(diffCounts[m], mbrDistanceSamples[m].counts, sizeof(diffCounts[m]));
                            } else {
                                legacyReadStartedMs[m] = to_ms_since_boot(get_absolute_time());
                                diffOk[m] = (chips[m]->get_DIFFERENCE_COUNT_SENSOR(diffCounts[m]) == 0);
                                if (!diffOk[m]) {
                                    // round47b-patch: I2C error - re-read once before trusting
                                    diffRetried[m] = true;
                                    diffOk[m] = (chips[m]->get_DIFFERENCE_COUNT_SENSOR(diffCounts[m]) == 0);
                                }
                            }
                            diffRead[m] = true;
                        }
                        if (!diffOk[m]) {
                            // A failed group cannot count toward a new ON.
                            raw[m] &= ~(1u << e);
                            legacyFaultMask[m] |= 1u << e;
                            confirmReq[m][e] = CONFIRM_CYCLES;
                        } else {
                            uint16_t diff = diffCounts[m][e];
                            if (!mbrDistanceEnabledForFrame && diff > 255) {
                                raw[m] &= ~(1u << e);
                                legacyFaultMask[m] |= 1u << e;
                                continue;
                            }
                            uint16_t baseK = verifyBaseK[m][e];
                            // round80: hover-rejection gate (mbrTouchGate, 0=off).
                            // Legacy new-touch verification only: no physical
                            // distance guarantee, and held/release/sticky state
                            // is untouched. The opt-in round89a gate has already
                            // applied a continuous per-electrode hard rejection.
                            // round84: confirmed-neighbor anchoring (v2 mirror --
                            // see the full rationale there). Anchor is the GAME
                            // LANE (k +/- 1 via laneTable), never "any electrode
                            // on the same chip"; v1 mixes electrode orders across
                            // chips A/B/C, so the same shuffle hazard applies.
                            uint16_t rejectTh = baseK;
                            if (ControllerConfig.mbrTouchGate > rejectTh) rejectTh = ControllerConfig.mbrTouchGate;
                            int16_t lane = laneTable[m][e];
                            bool laneNeighborConfirmed = false;
                            if (lane >= 0) {
                                laneNeighborConfirmed =
                                    (lane > 0 && prevLaneStretched[lane - 1]) ||
                                    (lane < 31 && prevLaneStretched[lane + 1]);
                            }
                            if (laneNeighborConfirmed && rejectTh > baseK + MBR_NEIGHBOR_GATE_MARGIN) {
                                rejectTh = baseK + MBR_NEIGHBOR_GATE_MARGIN;
                            }
                            // round45u: slide transition acceleration
                            bool neighborWasActive = (e > 0 && (prevStretched[m] & (1 << (e - 1)))) ||
                                                     (e + 1 < maxElec[m] && (prevStretched[m] & (1 << (e + 1))));
                            if (diff < rejectTh) {
                                // False touch - signal insufficient
                                raw[m] &= ~(1 << e);
                                verifiedCount[m][e] = 0;
                                confirmReq[m][e] = CONFIRM_CYCLES;
                                fastOk[m][e] = false;
                                if (g_verifyFail[m * 12 + e] < 255) g_verifyFail[m * 12 + e]++;
                            } else if (verifiedCount[m][e] >= 2) {
                                // Preserve rapid recontact cadence after fresh
                                // evidence, never on retained verification alone.
                            } else if (verifiedCount[m][e] == 0 && !diffRetried[m] &&
                                       diff >= MBR_FLICK_FAST_TH) {
                                // round80: strong flick fast path -- first-cycle
                                // output restores the old firmware's zero-latency
                                // behavior for isolated flick taps. The isolated
                                // +1 penalty is waived via fastOk in the confirm
                                // loop, still guarded by the 40ms release-bounce
                                // window there. Gate interplay: with mbrTouchGate
                                // >= 200 the reject check above lifts the effective
                                // fast threshold to the gate value (expected).
                                verifiedCount[m][e] = 2;
                                confirmReq[m][e] = 1;
                                fastOk[m][e] = true;
                            } else if (verifiedCount[m][e] == 0 && !diffRetried[m] &&
                                       (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_SKIP) || neighborWasActive)) {
                                // Very strong signal or slide - skip 2nd verification
                                verifiedCount[m][e] = 2;
                                confirmReq[m][e] = 1;
                            } else {
                                verifiedCount[m][e]++;
                                if (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_STRONG))      confirmReq[m][e] = 1;
                                else if (diff >= (uint16_t)(baseK + MBR_TIER_OFFSET_MEDIUM)) confirmReq[m][e] = 2;
                                else                                                         confirmReq[m][e] = 3;
                            }
                        }
                    }
                    // Healthy sustained ON retains the no-extra-read path.
                } else {
                    // round45r: preserve verification if recently touched (sticky dip recovery)
                    if (verifiedCount[m][e] != 0) {
                        if (lastTouchedMs[m][e] == 0 || (nowVer - lastTouchedMs[m][e]) > 50) {
                            verifiedCount[m][e] = 0;
                            confirmReq[m][e] = CONFIRM_CYCLES;
                            fastOk[m][e] = false;
                        }
                    }
                }
            }
        }
        if (legacyMbr) {
            uint32_t checkedAt = to_ms_since_boot(get_absolute_time());
            for (uint8_t m = 0; m < 3; ++m) {
                if (!mbrButtonValid[m] ||
                    checkedAt - mbrButtonStartMs[m] > MBR_DISTANCE_MAX_READ_AGE_MS) {
                    legacyFaultMask[m] = 0xFFFF;
                    legacyReadStartedMs[m] = mbrButtonStartMs[m];
                    raw[m] = 0;
                }
            }
        }
        t0 = raw[0]; t1 = raw[1]; t2 = raw[2];
    }
    rawTouch[0] = t0;
    rawTouch[1] = t1;
    rawTouch[2] = t2;

    // Touch pulse stretching: extend brief touches so the game's 300-500Hz
    // polling reliably samples fast-swipe notes (big-slide / zigzag / X-pattern).
    // Touch is output ONLY after CONFIRM_CYCLES consecutive cycles of sustained
    // contact, filtering 1-cycle random noise (autoconfig disabled, no periodic noise).
    // After release, the touch is held for STRETCH_MS so game polling catches it.
    static const uint32_t STRETCH_MS = 5;              // round47: 3->5ms. After a confirmed touch releases, the output
    // is held for 5ms so the game's 300-500Hz (2-3.3ms) polling samples at least one
    // "pressed" frame during the release transition. Does NOT cover main-loop stalls
    // (those freeze all output, not just stretch); it only smooths normal-speed releases.
    static const uint8_t  DIP_TOLERANCE_CYCLES = 3;     // round45n: 2->3, more pre-sticky dip tolerance
    static const uint8_t  STICKY_THRESHOLD = 15;        // round45n: touchCount to enter sticky mode (~15ms sustained touch)
   static const uint8_t  STICKY_GRACE_MAX = 15;        // round45n: sticky dip tolerance cycles (~15ms, >>dipGrace for sustained holds)
    static const uint8_t  STICKY_GRACE_SHORT = 12;      // round47: 8->12 (lock-hand tolerance ~32ms; round45p comment said faster release, but miss risk > release latency)
   static uint8_t  touchCount[3][12] = {0};
   static uint32_t lastConfirmed[3][12] = {0};
   static uint8_t  dipGrace[3][12] = {0};
   static uint8_t  stickyGrace[3][12] = {0};           // round45n: sticky-mode dip grace counter
   static MbrLegacyFaultState legacyFaults[3][12];
    if (mbrDistanceResetForFrame) {
        std::memset(touchCount, 0, sizeof(touchCount));
        std::memset(lastConfirmed, 0, sizeof(lastConfirmed));
        std::memset(dipGrace, 0, sizeof(dipGrace));
        std::memset(stickyGrace, 0, sizeof(stickyGrace));
        for (uint8_t m = 0; m < 3; ++m)
            for (uint8_t e = 0; e < 12; ++e) legacyFaults[m][e] = MbrLegacyFaultState{};
    }
   uint16_t raw[3] = {t0, t1, t2};
   uint16_t stretched[3] = {0, 0, 0};
   uint32_t now = to_ms_since_boot(get_absolute_time());
   for (uint8_t m = 0; m < 3; m++) {
       for (uint8_t e = 0; e < 12; e++) {
           bool fault = legacyMbr && (legacyFaultMask[m] & (1u << e));
           bool hold = mbrLegacyFaultHold(legacyFaults[m][e], fault,
               (prevStretched[m] & (1u << e)) != 0, legacyReadStartedMs[m], now);
           if (fault) {
               if (hold) {
                   stretched[m] |= 1u << e;
               } else {
                   touchCount[m][e] = dipGrace[m][e] = stickyGrace[m][e] = 0;
                   lastConfirmed[m][e] = lastTouchedMs[m][e] = 0;
                   verifiedCount[m][e] = 0;
                   confirmReq[m][e] = CONFIRM_CYCLES;
                   fastOk[m][e] = false;
               }
               continue;
           }
           if (mbrDistanceEnabledForFrame && !(distanceAllowed[m] & (1u << e))) {
               touchCount[m][e] = 0;
               lastConfirmed[m][e] = 0;
               dipGrace[m][e] = 0;
               stickyGrace[m][e] = 0;
               continue;
           }
           bool isTouched = (raw[m] >> e) & 1;
           if (isTouched) {
               lastTouchedMs[m][e] = now;
               dipGrace[m][e] = DIP_TOLERANCE_CYCLES;
               if (touchCount[m][e] < 255) touchCount[m][e]++;
                // round45n: replenish sticky grace for sustained touches
                if (touchCount[m][e] >= STICKY_THRESHOLD) {
                     // round45p: adaptive grace - short holds get less (faster release),
                     // long holds (touchCount>=50, ~130ms) get maximum tolerance.
                     stickyGrace[m][e] = (touchCount[m][e] >= 50) ? STICKY_GRACE_MAX
                                       : STICKY_GRACE_SHORT;
                }
                uint8_t reqCycles = confirmReq[m][e] ? confirmReq[m][e] : CONFIRM_CYCLES;
                // round45j: spatial consistency - isolated electrode (no same-MPR neighbor touched)
                // needs more confirm cycles to filter isolated noise (real touches span 1-3 adjacent cells)
                // round45u: use prevStretched (includes sticky-maintained touches) for neighbor check.
                // During slide transition, cell A's hardware bit=0 but sticky keeps it active.
                // Using raw[m] would treat cell B as isolated -> extra confirm delay -> slide miss.
                bool hasNeighbor = ((e > 0 && (prevStretched[m] & (1 << (e-1)))) || (e < 11 && (prevStretched[m] & (1 << (e+1)))));
                // round47b-patch: spatial penalty applies to ALL reqCycles (including 1).
                // Old code skipped strong signals (reqCycles==1) -> 1-cycle noise pulse on
                // high-idle-noise electrodes (M2E0 cell17: diff>=14 -> confirmReq=1 -> instant
                // output) passed through. Now isolated electrodes always need >=2 cycles.
                // Cost: isolated strong flick tap +1 cycle (~2.7ms) - acceptable.
                // round80: waive for strong flick signals (fastOk, set only when
                // diff >= MBR_FLICK_FAST_TH / MPR fast threshold on a first-sighting
                // verify -- orders of magnitude above the round47b noise spectrum)
                // unless within 40ms of the last confirmed touch: a residual-signal
                // re-enhancement right after release must not double-fire as a new tap.
                bool fastExempt = fastOk[m][e] &&
                    (lastConfirmed[m][e] == 0 || (now - lastConfirmed[m][e]) >= 40);
                if (!hasNeighbor && !fastExempt) reqCycles += 1;
                if (touchCount[m][e] >= reqCycles) {
                    lastConfirmed[m][e] = now;
                    stretched[m] |= (1 << e);
                }
            } else {
                // round45n: sticky touch - once sustained (touchCount>=STICKY_THRESHOLD),
                // tolerate much longer dips (STICKY_GRACE_MAX cycles) WITHOUT clearing touchCount.
                // Critical: touchCount preserved during sticky dips => instant re-trigger
                // when hardware touch returns (no re-verify/re-confirm delay = no miss).
                if (touchCount[m][e] >= STICKY_THRESHOLD && stickyGrace[m][e] > 0) {
                    stickyGrace[m][e]--;
                    stretched[m] |= (1 << e);
                } else if (lastConfirmed[m][e] != 0 && touchCount[m][e] >= 3 && dipGrace[m][e] > 0) {
                    // round45n: pre-sticky dipGrace (touchCount>=3, was >=4) for building-up touches
                    dipGrace[m][e]--;
                    stretched[m] |= (1 << e);
                } else {
                    touchCount[m][e] = 0;
                    stickyGrace[m][e] = 0;
                    if (lastConfirmed[m][e] != 0 &&
                        (now - lastConfirmed[m][e]) < STRETCH_MS) {
                        stretched[m] |= (1 << e);
                    }
                }
            }
        }
    }
    t0 = stretched[0];
    t1 = stretched[1];
    t2 = stretched[2];

    // round45u: save stretched output for next cycle's slide-aware spatial filtering
    prevStretched[0] = t0;
    prevStretched[1] = t1;
    prevStretched[2] = t2;

    // round84: project this cycle's stretched bits into lane space for next
    // cycle's laneNeighborConfirmed gate anchoring (MBR only -- the MPR branch
    // is untouched this round). Uses the STRETCHED bits (incl. sticky-held
    // lanes), matching the v2 projection exactly: anchor semantics must stay
    // identical across layouts.
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        for (uint8_t lane = 0; lane < 32; lane++) {
            uint8_t lm = V1_LANE_M[lane], le = V1_LANE_E[lane];
            uint16_t bits = (lm == 0) ? t0 : ((lm == 1) ? t1 : t2);
            prevLaneStretched[lane] = (bits >> le) & 1;
        }
    }

    touchData32[0] = GET_BIT(t1, 11) ? 128 : 0;
    touchData32[1] = GET_BIT(t1, 0) ? 128 : 0;

    touchData32[2] = GET_BIT(t1, 10) ? 128 : 0;
    touchData32[3] = GET_BIT(t1, 1) ? 128 : 0;

    touchData32[4] = GET_BIT(t1, 9) ? 128 : 0;
    touchData32[5] = GET_BIT(t1, 2) ? 128 : 0;

    touchData32[6] = GET_BIT(t1, 8) ? 128 : 0;
    touchData32[7] = GET_BIT(t1, 3) ? 128 : 0;

    touchData32[8] = GET_BIT(t0, 11) ? 128 : 0;
    touchData32[9] = GET_BIT(t1, 4) ? 128 : 0;

    touchData32[10] = GET_BIT(t0, 10) ? 128 : 0;
    touchData32[11] = GET_BIT(t1, 5) ? 128 : 0;

    touchData32[12] = GET_BIT(t0, 9) ? 128 : 0;
    touchData32[13] = GET_BIT(t1, 6) ? 128 : 0;

    touchData32[14] = GET_BIT(t0, 8) ? 128 : 0;
    touchData32[15] = GET_BIT(t1, 7) ? 128 : 0;

    touchData32[16] = GET_BIT(t0, 7) ? 128 : 0;
    touchData32[17] = GET_BIT(t2, 0) ? 128 : 0;

    touchData32[18] = GET_BIT(t0, 6) ? 128 : 0;
    touchData32[19] = GET_BIT(t2, 1) ? 128 : 0;

    touchData32[20] = GET_BIT(t0, 5) ? 128 : 0;
    touchData32[21] = GET_BIT(t2, 2) ? 128 : 0;

    touchData32[22] = GET_BIT(t0, 4) ? 128 : 0;
    touchData32[23] = GET_BIT(t2, 3) ? 128 : 0;

    touchData32[24] = GET_BIT(t0, 3) ? 128 : 0;
    touchData32[25] = GET_BIT(t2, 4) ? 128 : 0;

    touchData32[26] = GET_BIT(t0, 2) ? 128 : 0;
    touchData32[27] = GET_BIT(t2, 5) ? 128 : 0;

    touchData32[28] = GET_BIT(t0, 1) ? 128 : 0;
    touchData32[29] = GET_BIT(t2, 6) ? 128 : 0;

    touchData32[30] = GET_BIT(t0, 0) ? 128 : 0;
    touchData32[31] = GET_BIT(t2, 7) ? 128 : 0;

    // round45t: set touchData[0..3] AFTER touchData32[] to fix cross-core race condition.
    // Previously touchData[0..2] were set BEFORE touchData32[], so Core1 HID could read
    // a mix of old/new bytes (phantom touch / micro-dropout). Now all 4 bytes are set
    // in a quick burst after touchData32[] is complete, minimizing the race window.
    // HID slider 24-31: derive from touchData32[24-31] to match CDC physical mapping.
    // Previous ((t0 & 0x0f00) >> 8) | ((t1 & 0x0f00) >> 4) used mpr0/mpr1 ELE8-11
    // which are physically at slider 0-15, causing in-game sticky keys on 24-31.
    touchData[0] = t0;
    touchData[1] = t1;
    touchData[2] = t2;
    touchData[3] = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (touchData32[24 + i]) touchData[3] |= (1 << i);
    }
}

void updateTouchData4k() {
    for (uint8_t i = 0; i < 4; i++) {
        touchData4k[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 8; j++) {
            touchData4k[i] |= touchData32[i * 8 + j];
        }
    }
}

void updateTouchData6k() {
    for (uint8_t i = 0; i < 6; i++)
        touchData6k[i] = 0;

    for (uint8_t i = 29; i > 25; i--) {
        touchData6k[0] |= touchData32[i];
    }
    for (uint8_t i = 25; i > 21; i--) {
        touchData6k[1] |= touchData32[i];
    }
    for (uint8_t i = 21; i > 17; i--) {
        touchData6k[2] |= touchData32[i];
    }
    for (uint8_t i = 13; i > 9; i--) {
        touchData6k[3] |= touchData32[i];
    }
    for (uint8_t i = 9; i > 5; i--) {
        touchData6k[4] |= touchData32[i];
    }
    for (uint8_t i = 5; i > 1; i--) {
        touchData6k[5] |= touchData32[i];
    }
}

uint16_t heightDataOriginal[5] = { 4095, 4095, 4095, 4095, 4095 };
int16_t heightData[5] = { 4094, 4094, 4094, 4094, 4094 };
bool airKeys[6];


void updateAir() {
    VL53L0X* tofs[5] = { &tof0, &tof1, &tof2, &tof3, &tof4 };
    int sensorCount = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 5 : 4;

    // Phase 1: Read ToF + Kalman update
    bool anyUpdated = false;
    const uint32_t sampleNow = to_ms_since_boot(get_absolute_time());
    bool gotNewData[5] = {};
    for (int i = 0; i < sensorCount; i++) {
        if (!(g_tofReadyMask & (1u << i))) continue;
        if (useMuxScan) mux0.setChannel(i);
        if (tofs[i]->readRangeContinuousMillimetersAsync(heightDataOriginal + i)) {
            if (heightDataOriginal[i] >= 8190) {
                tofValid[i] = false;
                kalman[i] = {0, 0, 200.0f, -1.0f};
            } else {
                float meas = (float)((int16_t)heightDataOriginal[i] + ControllerConfig.heightOffset[i]);
                if (kalman[i].p >= 199.0f) {
                    // First valid measurement: hard-set position, zero velocity
                    kalman[i].x = meas;
                    kalman[i].v = 0;
                    kalman[i].p = KALMAN_R;
                    kalman[i].lastMeas = meas;
                } else {
                    kalmanUpdate(kalman[i], meas);
                }
                gotNewData[i] = true;
                tofValid[i] = true;
                tofSampleMs[i] = sampleNow;
            }
            anyUpdated = true;
        }
    }
    // Phase 2: Kalman predict (sensors without new data only) + update heightData for debug
    for (int i = 0; i < sensorCount; i++) {
        if (!gotNewData[i] && heightDataOriginal[i] < 8190) { kalman[i].x += kalman[i].v; kalman[i].v *= 0.85f; }
        if (tofValid[i] && sampleNow - tofSampleMs[i] >= TOF_FRESH_MS) {
            tofValid[i] = false;
            kalman[i] = {0, 0, 200.0f, -1.0f};
            anyUpdated = true;
        }
        heightData[i] = tofValid[i] ? (int16_t)kalman[i].x : 4095;
    }
    if (!anyUpdated) {
        // A fault may be the last completion; expire the release pulse even
        // if every peripheral then stops producing measurements.
        for (int j = 0; j < 6; ++j) {
            if (airKeyTracker[j].state != AKS_ACTIVE &&
                sampleNow - airKeyTracker[j].lastActiveMs >= 10) airKeys[j] = false;
        }
        return;
    }

    // Phase 3: Slider gating (from previous cycle's touchData)
    bool handOnSlider = false;
    for (int i = 0; i < 32; i++) {
        if (touchData32[i]) { handOnSlider = true; break; }
    }

    // Phase 4: Air key detection with Kalman + lookahead + lock
    int16_t dH = ((ControllerConfig.airMax - ControllerConfig.airMin) * 21) >> 7;
    uint32_t nowMs = to_ms_since_boot(get_absolute_time());

    for (int j = 0; j < 6; j++) {
        int16_t rangeLow = ControllerConfig.airMin + (int16_t)dH * j - (int16_t)heightRange;
        int16_t rangeHigh = ControllerConfig.airMin + (int16_t)dH * (j + 1) + (int16_t)heightRange;
        AirKeyTracker& tk = airKeyTracker[j];

        // Check detection: any sensor in range (with lookahead for rising hand)
        bool inRange = false;
            for (int i = 0; i < sensorCount; i++) {
            if (!tofValid[i] || kalman[i].x <= 0 || kalman[i].x >= 4000) continue;
            float v = kalman[i].v;
            // Actual position in range (maintains ACTIVE, also triggers)
            if (kalman[i].x >= rangeLow && kalman[i].x <= rangeHigh) {
                inRange = true;
            }
            // Lookahead position (early trigger when approaching from below)
            if (!handOnSlider && v < -V_MIN) {
                float la = clampf(-v * LOOKAHEAD_K, 0.0f, MAX_LOOKAHEAD);
                float xAhead = kalman[i].x + v * la;
                if (xAhead >= rangeLow && xAhead <= rangeHigh) {
                    inRange = true;
                }
            }
        }

        switch (tk.state) {
        case AKS_IDLE:
            if (inRange) {
                tk.state = AKS_ACTIVE;
                tk.lastActiveMs = nowMs;
            }
            break;
        case AKS_ACTIVE:
            if (inRange) {
                tk.lastActiveMs = nowMs;
            } else {
                // Hysteresis: check wider range with actual position
                bool inHyst = false;
                for (int i = 0; i < sensorCount; i++) {
                    if (!tofValid[i] || kalman[i].x <= 0 || kalman[i].x >= 4000) continue;
                    if (kalman[i].x >= rangeLow - EXIT_HYSTERESIS &&
                        kalman[i].x <= rangeHigh + EXIT_HYSTERESIS) {
                        inHyst = true; break;
                    }
                }
                if (!inHyst) {
                    tk.state = AKS_COOLDOWN;
                    tk.cooldown = 1;
                    tk.lastActiveMs = nowMs;
                }
            }
            break;
        case AKS_COOLDOWN:
            if (tk.cooldown > 0) tk.cooldown--;
            else tk.state = AKS_IDLE;
            break;
        }

        airKeys[j] = (tk.state == AKS_ACTIVE) ||
                     ((int32_t)(nowMs - tk.lastActiveMs) < 10);
    }
}

static MbrTouchTrace buildMbrTouchTrace() {
    MbrTouchTrace trace{};
    trace.tag = CMD_MBR_TOUCH_TRACE;
    trace.version = MBR_TRACE_VERSION;
    trace.frameId = (touchStateGen + 2u) / 2u;
    trace.publishedMs = to_ms_since_boot(get_absolute_time());
    std::memcpy(trace.slider, touchData32, sizeof(touchData32));
    std::memcpy(trace.hardware, hwTouch, sizeof(hwTouch));
    std::memcpy(trace.verified, rawTouch, sizeof(rawTouch));
    bool useMbr = (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) ||
                  ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4;
    if (useMbr) {
        trace.chipCount = (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4) ? 2 : 3;
        if (mbrDistanceEnabledForFrame) trace.flags |= MBR_TRACE_PROFILE;
        for (uint8_t m = 0; m < trace.chipCount; ++m) {
            if (mbrDistanceEnabledForFrame) {
                trace.chips[m] = mbrDistanceSamples[m].quality;
                for (uint8_t e = 0; e < 16; ++e) {
                    int16_t lane = laneTable[m][e];
                    if (lane >= 0) trace.counts[lane] = mbrDistanceSamples[m].counts[e];
                }
            }
            trace.chips[m].buttonStartMs = mbrButtonStartMs[m];
            trace.chips[m].buttonEndMs = mbrButtonEndMs[m];
            if (mbrButtonValid[m]) trace.chips[m].flags |= MBR_TRACE_BUTTON_VALID;
        }
    }
    return trace;
}

static void publishTouchState() {
    const MbrTouchTrace trace = buildMbrTouchTrace();
    ++touchStateGen;
    __dmb();
    std::memcpy(publishedTouchState.keys, touchData, sizeof(touchData));
    std::memcpy(publishedTouchState.slider, touchData32, sizeof(touchData32));
    std::memcpy(publishedTouchState.pressure, pressureSnap, sizeof(pressureSnap));
    std::memcpy(publishedTouchState.hardware, hwTouch, sizeof(hwTouch));
    std::memcpy(publishedTouchState.verified, rawTouch, sizeof(rawTouch));
    std::memcpy(&publishedMbrTrace, &trace, sizeof(trace));
    publishedTouchState.air = 0;
    for (uint8_t i = 0; i < 6; ++i) {
        if (airKeys[i]) publishedTouchState.air |= 1u << i;
    }
    __dmb();
    ++touchStateGen;
    recordMbrHistory(trace);
}

void updateInputState() {
    watchdog_update();
    uint32_t loopStartUs = time_us_32();
    // Air update first: lower latency for judgment-critical path
    if (usingIR) {
        updateIR();
    } else {
        updateAir();
    }
    // Sensor I/O leaves the previous completed frame available to Core1.
    prepareMbrDistanceGate();
    updatePressureSnap();
    if (ControllerConfig.hwVer == 1 || ControllerConfig.hwVer == 2) {
        updateTouch_v1();
    }
    if (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4) {
        updateTouch_v2();
    }
    publishTouchState();

    // MPR baselines are writable only in Stop mode. Native tracking owns
    // runtime baselines; remove the ineffective Run-mode correction I2C load.

    // round46: Core0 loop cycle timing (diagnostic telemetry, Core1 reads via 0xC1)
    {
        uint32_t loopUs = time_us_32() - loopStartUs;
        if (loopUs < g_loopMinUs) g_loopMinUs = loopUs;
        if (loopUs > g_loopMaxUs) g_loopMaxUs = loopUs;
        g_loopSumUs += loopUs;
        g_loopCount++;
    }
}
