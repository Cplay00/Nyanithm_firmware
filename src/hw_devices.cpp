/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <controller_config.h>
#include <hw_devices.h>
#include <tca9539.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>

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
static inline void kalmanPredict(Kalman1D& k) {
    k.v *= 0.85f;  // velocity decay: uncertainty grows without measurement
    k.x += k.v;
    k.p += KALMAN_Q;
}

// ===== round49: Air key lock state machine =====
enum AirKeyState : uint8_t { AKS_IDLE, AKS_ACTIVE, AKS_COOLDOWN };
struct AirKeyTracker {
    AirKeyState state;
    uint32_t lastActiveMs;
    uint8_t cooldown;
};
static AirKeyTracker airKeyTracker[6] = {};


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
    initToFReset();
    resetToF();
    sleep_ms(2);
    VL53L0X* tofs[5] = { &tof0, &tof1, &tof2, &tof3, &tof4 };
    int sensorCount = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 5 : 4;
    // Phase 1: init each sensor through mux, assign unique I2C address
    for (int i = 0; i < sensorCount; i++) {
        mux0.setChannel(i);
        tofs[i]->setTimeout(200);
        tofs[i]->forceInit();
        tofs[i]->setMeasurementTimingBudget(12000);
        tofs[i]->setAddress(0x30 + i);
        tofs[i]->startContinuous(0);
        sleep_ms(5);  // stagger: distribute measurement completion phases
    }
    // Phase 2: enable all mux channels simultaneously (all sensors on bus)
    uint8_t muxMask = (sensorCount == 5) ? 0x1F : 0x0F;
    mux0.setReg(muxMask);
    sleep_ms(2);
    // Phase 3: verify all sensors respond at their new addresses
    useMuxScan = false;
    for (int i = 0; i < sensorCount; i++) {
        if (!findI2CDevice(1, 0x30 + i, 10)) {
            useMuxScan = true;
            break;
        }
    }
    if (useMuxScan) {
        // Fallback: reset all sensors, re-init with mux-per-channel scanning
        resetToF();
        sleep_ms(2);
        for (int i = 0; i < sensorCount; i++) {
            mux0.setChannel(i);
            tofs[i]->setI2CAddressOnly(0x29);
            tofs[i]->setTimeout(200);
            tofs[i]->forceInit();
            tofs[i]->setMeasurementTimingBudget(12000);
            tofs[i]->startContinuous(0);
            sleep_ms(5);
        }
    }
}

static uint8_t electrodeBaseTouchTh(uint8_t m, uint8_t e) {
    // round46: unified-threshold build variant.
    //   1 = every electrode uses the SAME global th_touch/th_release
    //       (majority-of-keys calibration; values set via ConfigApp after
    //       running calibrate_thresholds.ps1). High-idle-noise electrodes
    //       are handled by the software verification chain instead of
    //       per-electrode hardware thresholds.
    //   0 = restore the round45b per-electrode override table below.
#define UNIFIED_THRESHOLD_V1 1
#if UNIFIED_THRESHOLD_V1
    (void)m;
    (void)e;
    return 0;  // unified: fall back to global ControllerConfig.th_touch
#else
    // round45b: per-electrode elevated touch threshold for high-idle-noise electrodes.
    // Idle diff analysis (detail_log, 423 idle samples) showed these electrodes spike
    // above global th_touch=6 (M2E0 max=13). Elevate base threshold above observed idle
    // max to suppress false touches. Real touches (diff 20-50) unaffected. Returns 0
    // to mean "use global th_touch".
    if (m == 2 && e == 0) return 15;  // M2E0 cell17 idle max=13, +2 margin (round45g: was 16, lowered to reduce miss)
    if (m == 2 && e == 3) return 10;  // M2E3 cell23 idle max=8, +2 (was 12)
    if (m == 2 && e == 1) return 9;   // M2E1 cell19 idle max=7, +2 (was 11)
    if (m == 0 && e == 5) return 9;   // M0E5 cell20 idle max=7, +2 (was 11)
    if (m == 0 && e == 6) return 9;   // M0E6 cell18 idle max=7, +2 (was 11)
    if (m == 1 && e == 7) return 8;   // M1E7 cell15 idle max=6, +2 (was 10)
    if (m == 2 && e == 2) return 9;   // M2E2 cell21 idle max=6, +3 (round45h: was 8, raised to curb idle trigger)
    if (m == 2 && e == 4) return 10;  // round45r: M2E4 cell25, was 7 but still 2x/10min idle trigger, raise to 10
    return 0;
#endif
}

void initMPR121() {
    mpr0.init(6, 3, true);
    mpr1.init(6, 3, true);
    mpr2.init(6, 3, true);

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
    // round45b: per-electrode elevated thresholds for high-idle-noise electrodes
    for (uint8_t m = 0; m < 3; m++) {
        for (uint8_t e = 0; e < 12; e++) {
            uint8_t base = electrodeBaseTouchTh(m, e);
            uint8_t eth = (base > th_t) ? base : th_t;
            if (m == 0) mpr0.setThresholdsForElectrode(e, eth, th_r);
            else if (m == 1) mpr1.setThresholdsForElectrode(e, eth, th_r);
            else mpr2.setThresholdsForElectrode(e, eth, th_r);
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
    mpr1.calibrateBaseline();
    mpr2.calibrateBaseline();
}

void initI2C() {
    initI2CBus(0, GPIO_I2C_0_SDA, GPIO_I2C_0_SCL, BR_I2C);
    initI2CBus(1, GPIO_I2C_1_SDA, GPIO_I2C_1_SCL, BR_I2C);
}

void program_cyc8mbr3116_with_address(uint8_t address, uint8_t* cfg) {
    for (uint8_t i = 0; i < 128; i++) {
        uint8_t buf[2] = { i, cfg[i] };
        i2c_write(0, address, buf, 2, false);
    }
    uint8_t buf[2] = { 0x86, 0x02 };  // 给CTRL_CMD发送命令，检查CRC并保存，地址0x86，写入2
    i2c_write(0, address, buf, 2, false);
    sleep_ms(20);

    buf[1] = 0xff;  // 软复位
    i2c_write(0, address, buf, 2, false);
    sleep_ms(20);
}

void initCY8CMBR3116() {
    // program_cyc8mbr3116_with_address(0x40, cy8cmbr3116_cfg_0x40);
    // program_cyc8mbr3116_with_address(0x41, cy8cmbr3116_cfg_0x41);
    // program_cyc8mbr3116_with_address(0x42, cy8cmbr3116_cfg_0x42);

    // gpio_init(GPIO_3116RST0);
    // gpio_init(GPIO_3116RST1);

    // gpio_set_dir(GPIO_3116RST0, true);
    // gpio_set_dir(GPIO_3116RST1, true);

    // gpio_pull_down(GPIO_3116RST0);
    // gpio_pull_down(GPIO_3116RST1);

    // gpio_set_drive_strength(GPIO_3116RST0,GPIO_DRIVE_STRENGTH_8MA);
    // gpio_set_drive_strength(GPIO_3116RST1,GPIO_DRIVE_STRENGTH_8MA);

    // gpio_put(GPIO_3116RST0, false);
    // sleep_ms(10);
    // gpio_put(GPIO_3116RST0, true);
    // gpio_put(GPIO_3116RST1, false);
    // sleep_ms(10);
    // gpio_put(GPIO_3116RST1, true);
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
    // updateInputState() calls watchdog_update() every cycle.
    watchdog_enable(2000, true);
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
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        initCY8CMBR3116();
    } else {
        initMPR121();
    }
}

uint8_t touchData[4];
uint8_t touchData32[32];
uint16_t rawTouch[3] = {0, 0, 0};  // non-static for CMD_DEBUG_CHAIN access  // raw 16-bit touch data for stuck-key detection
uint16_t hwTouch[3] = {0, 0, 0};   // round45p: pre-verification hardware touch snapshot for CMD_DEBUG_CHAIN (zero I2C overhead)
uint32_t lastTouchedMs[3][12] = {0};  // round45m: last touch timestamp per electrode (baseline correction skips recently-touched)

// round46: cross-core seqlock generation for the shared touch state.
// Core0 bumps to odd BEFORE writing (updateTouch/updateAir), bumps to even AFTER.
// Core1 (HID + CDC) readers copy the shared bytes only between an even pair of
// generation reads, eliminating torn reads that produced phantom touches /
// dropped cells in the game (the CDC path was never covered by round45t).
volatile uint32_t touchStateGen = 0;

// round46: telemetry shared from Core0 to Core1 (CMD_DEBUG_TELEMETRY = 0xC1)
uint8_t  g_verifyFail[36] = {0};          // per-electrode I2C verification rejections
uint32_t g_loopMinUs = 0xFFFFFFFF, g_loopMaxUs = 0, g_loopSumUs = 0, g_loopCount = 0;

#define GET_BIT(UNUM, BIT) (UNUM & (1 << BIT))

bool touchData4k[4];
bool touchData6k[6];

void updateTouch_v2() {
    uint16_t t0, t1;
    MBR3116D.get_BUTTON_STAT((uint8_t*)&t0);
    MBR3116E.get_BUTTON_STAT((uint8_t*)&t1);

    touchData[0] = *(0 + (uint8_t*)(&t0));
    touchData[1] = *(1 + (uint8_t*)(&t0));
    touchData[2] = *(0 + (uint8_t*)(&t1));
    touchData[3] = *(1 + (uint8_t*)(&t1));

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
}

void updateTouch_v1() {

    uint16_t t0, t1, t2;
    if (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) {
        MBR3116A.get_BUTTON_STAT((uint8_t*)&t0);
        MBR3116B.get_BUTTON_STAT((uint8_t*)&t1);
        MBR3116C.get_BUTTON_STAT((uint8_t*)&t2);
    } else {
        t0 = mpr0.touched();
        t1 = mpr1.touched();
        t2 = mpr2.touched();
    }

    // round45p: save pre-verification hardware touch snapshot for CMD_DEBUG_CHAIN
    hwTouch[0] = t0; hwTouch[1] = t1; hwTouch[2] = t2;

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
   if (!(ControllerConfig.cfg0 & CFG0_BIT_MBR3116)) {
        uint16_t raw[3] = {t0, t1, t2};
        MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
        uint8_t sw_th = (ControllerConfig.th_touch < 4) ? 4 : ControllerConfig.th_touch;
        // Verification threshold is sw_th - 2 to tolerate 2 LSB timing mismatch
        // between touched() read and filteredData/baselineData read.
        // Without this margin, light touches (diff=4-5) are falsely rejected
        // because filtered data fluctuates ?2-3 LSB between I2C reads.
        uint8_t verify_th = (sw_th > 1) ? (sw_th - 1) : 1;  // round41: -2->-1 LSB tolerance
      static uint8_t verifiedCount[3][12] = {0};
       uint32_t nowVer = to_ms_since_boot(get_absolute_time());  // round45r: for sticky dip verification preservation
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
                                // Genuine I2C failure - don't trust, don't reject, just skip
                                confirmReq[m][e] = CONFIRM_CYCLES;
                            } else {
                                // First read was glitch, second OK - use filt2
                                filt = filt2;
                                uint16_t base = mprs[m]->baselineData(e);
                                int16_t diff = (int16_t)base - (int16_t)filt;
                                if (diff < verify_th) {
                                    raw[m] &= ~(1 << e);
                                    verifiedCount[m][e] = 0;
                                    confirmReq[m][e] = CONFIRM_CYCLES;
                                    if (g_verifyFail[m * 12 + e] < 255) g_verifyFail[m * 12 + e]++;
                                } else {
                                    // round47b-patch: intentionally no strong-signal fast-path
                                    // (diff>=sw_th+6 skip). First read was 0 (I2C unstable),
                                    // so require full 2-cycle verification.
                                    verifiedCount[m][e]++;
                                    if (diff >= (int16_t)(sw_th + 4))      confirmReq[m][e] = 1;
                                    else if (diff >= (int16_t)(sw_th + 2)) confirmReq[m][e] = 2;
                                    else                                   confirmReq[m][e] = 3;
                                }
                            }
                       } else {
                            uint16_t base = mprs[m]->baselineData(e);
                            int16_t diff = (int16_t)base - (int16_t)filt;
                               if (diff < verify_th) {
                               raw[m] &= ~(1 << e);
                               verifiedCount[m][e] = 0;
                               confirmReq[m][e] = CONFIRM_CYCLES;
                               if (g_verifyFail[m * 12 + e] < 255) g_verifyFail[m * 12 + e]++;  // round46: telemetry
                           } else {
                                // round45o: skip second verification for very strong signals (saves 1 cycle ~2ms for flick notes)
                                // round45u: slide transition acceleration - if neighbor was active
                                // in previous cycle, skip 2nd verification (slide, not noise)
                                bool neighborWasActive = (e > 0 && (prevStretched[m] & (1 << (e-1)))) ||
                                                          (e < 11 && (prevStretched[m] & (1 << (e+1))));
                                 if (diff >= (int16_t)(sw_th + 6) || neighborWasActive) {
                                    verifiedCount[m][e] = 2;  // skip second I2C verification cycle
                                    confirmReq[m][e] = 1;     // fastest confirmation
                                } else {
                                    verifiedCount[m][e]++;
                                    if (diff >= (int16_t)(sw_th + 4))      confirmReq[m][e] = 1;  // strong signal: fastest
                                    else if (diff >= (int16_t)(sw_th + 2)) confirmReq[m][e] = 2;  // medium
                                    else                                   confirmReq[m][e] = 3;  // weak/edge: strict, anti-false-touch
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
                   }
                   // else: keep verifiedCount, instant re-trigger when touch returns
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
   uint16_t raw[3] = {t0, t1, t2};
   uint16_t stretched[3] = {0, 0, 0};
   uint32_t now = to_ms_since_boot(get_absolute_time());
   for (uint8_t m = 0; m < 3; m++) {
       for (uint8_t e = 0; e < 12; e++) {
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
                if (!hasNeighbor) reqCycles += 1;
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
    bool gotNewData[5] = {};
    for (int i = 0; i < sensorCount; i++) {
        if (useMuxScan) mux0.setChannel(i);
        if (tofs[i]->readRangeContinuousMillimetersAsync(heightDataOriginal + i)) {
            if (heightDataOriginal[i] >= 8190) {
                heightData[i] = 4095;  // I2C error sentinel
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
            }
            anyUpdated = true;
        }
    }
    // Phase 2: Kalman predict (sensors without new data only) + update heightData for debug
    for (int i = 0; i < sensorCount; i++) {
        if (!gotNewData[i] && heightDataOriginal[i] < 8190) { kalman[i].x += kalman[i].v; kalman[i].v *= 0.85f; }
        heightData[i] = (int16_t)kalman[i].x;
    }
    if (!anyUpdated) return;

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
        bool risingConfident = false;
        for (int i = 0; i < sensorCount; i++) {
            if (kalman[i].x <= 0 || kalman[i].x >= 4000) continue;
            float v = kalman[i].v;
            // Actual position in range (maintains ACTIVE, also triggers)
            if (kalman[i].x >= rangeLow && kalman[i].x <= rangeHigh) {
                inRange = true;
                if (v < -V_MIN) risingConfident = true;
            }
            // Lookahead position (early trigger when approaching from below)
            if (!handOnSlider && v < -V_MIN) {
                float la = clampf(-v * LOOKAHEAD_K, 0.0f, MAX_LOOKAHEAD);
                float xAhead = kalman[i].x + v * la;
                if (xAhead >= rangeLow && xAhead <= rangeHigh) {
                    inRange = true;
                    risingConfident = true;
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
                    if (kalman[i].x <= 0 || kalman[i].x >= 4000) continue;
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

void updateInputState() {
    watchdog_update();
    uint32_t loopStartUs = time_us_32();
    touchStateGen++;
    // Air update first: lower latency for judgment-critical path
    if (usingIR) {
        updateIR();
    } else {
        updateAir();
    }
    if (ControllerConfig.hwVer == 1 || ControllerConfig.hwVer == 2) {
        updateTouch_v1();
    }
    if (ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4) {
        updateTouch_v2();
    }
    touchStateGen++;

    // Software baseline auto-correction: gradually adjust baseline to match
    // idle filtered data. Fixes baseline stuck too high (e.g., M2E0 baseline=212
    // vs idle filt=207, diff=5 causes false touches). Runs every 2s when idle.
    if (!(ControllerConfig.cfg0 & CFG0_BIT_MBR3116) &&
        (ControllerConfig.hwVer == 1 || ControllerConfig.hwVer == 2)) {
        // round47: idle baseline auto-correction (single software baseline writer).
        // MPR121 hardware now freezes baseline during touch (FDLT=0xFF), so no
        // anti-collapse / release-recovery writes are needed. This correction only
        // nudges idle baseline down when it drifts high (prevents false touches).
        // Uses writeBaselineRun() - direct I2C write without Stop->Run, so it never
        // interrupts touch measurement. Per-electrode 2s cooldown avoids re-writes.
        static uint32_t lastBaselineAdjust = 0;
        static uint32_t blWriteMs[3][12] = {0};
        uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        if (nowMs - lastBaselineAdjust > 500 && lastBaselineAdjust > 0) {
            lastBaselineAdjust = nowMs;
            MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
            static uint8_t correctionIndex = 0;
            // 8 electrodes per cycle, 500ms interval (2.25s rotation for all 36: 36/8*0.5s)
            for (uint8_t i = 0; i < 8; i++) {
                uint8_t idx = (correctionIndex + i) % 36;
                uint8_t m = idx / 12;
                uint8_t e = idx % 12;
                if ((rawTouch[m] & (1 << e)) || (nowMs - lastTouchedMs[m][e] < 2000)) continue;  // not idle
                if (nowMs - blWriteMs[m][e] < 2000) continue;  // round47: 2s cooldown per electrode
                uint16_t filt = mprs[m]->filteredData(e);
                uint16_t base = mprs[m]->baselineData(e);
                int16_t diff = (int16_t)base - (int16_t)filt;
                if (diff > 3 && filt > 0 && diff < 50) {
                    uint8_t bl_reg = mprs[m]->readRegister8(MPR121_BASELINE_0 + e);
                    uint8_t adj = (diff > 6) ? 2 : 1;
                    if (bl_reg > adj) bl_reg -= adj;
                    else bl_reg = 0;
                    if (bl_reg == 0) continue;  // round47-review: read failure guard (baseline 0 -> 2-3s fake touch)
                    // round47b: sanity check - bl_reg must be near filt>>2 (same electrode).
                    // Corrupted I2C reads (observed in snapshot bursts: base=8/44/64 vs ~712)
                    // would otherwise write a nonsense baseline. Allow +/-15 LSB tolerance.
                    {
                        uint16_t expected = (filt > 3) ? (uint16_t)(filt >> 2) : 0;
                        int16_t blDiff = (int16_t)bl_reg - (int16_t)expected;
                        if (blDiff > 15 || blDiff < -15) continue;  // corrupted read - skip write
                    }
                    mprs[m]->writeBaselineRun(e, bl_reg);  // round47: no Stop->Run
                    blWriteMs[m][e] = nowMs;
                }
            }
            correctionIndex = (correctionIndex + 8) % 36;
        } else if (lastBaselineAdjust == 0) {
            lastBaselineAdjust = nowMs;
        }
    }

    // round46: Core0 loop cycle timing (diagnostic telemetry, Core1 reads via 0xC1)
    {
        uint32_t loopUs = time_us_32() - loopStartUs;
        if (loopUs < g_loopMinUs) g_loopMinUs = loopUs;
        if (loopUs > g_loopMaxUs) g_loopMaxUs = loopUs;
        g_loopSumUs += loopUs;
        g_loopCount++;
    }
}
