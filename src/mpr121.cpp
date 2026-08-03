/* Modified from https://github.com/adafruit/Adafruit_MPR121 */
/*!
 * @file Adafruit_MPR121.cpp
 *
 *  @mainpage Adafruit MPR121 arduino driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the MPR121 I2C 12-chan Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 sensor from Adafruit
 *  ----> https://www.adafruit.com/products/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to
 *  interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *  @section license License
 *
 *  BSD license, all text here must be included in any redistribution.
 */

#include <mpr121.h>

#include <stdio.h>
#include <string.h>

#include <hardware/i2c.h>

#include <debug.h>
#include <i2c_port.h>

/*!
 *  @brief    Begin an MPR121 object on a given I2C bus. This function resets
 *            the device and writes the default settings.
 *  @param    port
 *            I2C port which is going to be used
 *  @param    i2c_addr
 *            the i2c address the device can be found on. Defaults to 0x5A.
 *  @param    touchThreshold
 *            touch detection threshold value
 *  @param    releaseThreshold
 *            release detection threshold value
 *  @param    autoconfig
 *            enable autoconfig option
 *  @returns  true on success, false otherwise
 */
MPR121::MPR121(uint8_t _port, uint8_t i2c_addr) {
    port = _port;
    addr = i2c_addr;
}

void MPR121::init(uint8_t touchThreshold, uint8_t releaseThreshold, bool autoconfig) {
    writeRegister(MPR121_SOFTRESET, 0x63);

    sleep_ms(1);

    // for (uint8_t i = 0; i < 0x7F; i++) {
    //     //  Serial.print("$"); Serial.print(i, HEX);
    //     //  Serial.print(": 0x"); Serial.println(readRegister8(i), HEX);
    // }

    writeRegister(MPR121_ECR, 0x0);

    uint8_t c = readRegister8(MPR121_CONFIG2);
#ifdef ENABLE_DEBUG
    // printf("read: MPR121_CONFIG2 = 0x%2x\n", c);
#endif
    if (c != 0x24) {
        DEBUG("mpr121 init failed!");
        good = false;
        return;
    }

    // Serial.println("write Configuration to sensor ...");
    setThresholds(touchThreshold, releaseThreshold);
    writeRegister(MPR121_MHDR, 0x01);  // round4: revert to round1 (MHDR too large dragged baseline down during touch => stickier)
    writeRegister(MPR121_NHDR, 0x01);
    writeRegister(MPR121_NCLR, 0x1F);  // round47b: 14->31, 恢复 round34 已验证值. NCLR=14 使 idle 噪声尖峰
    // 14ms 即可推高 baseline, NCLF=127 下降极慢构成棘轮 -> M2 芯片静置突发触发
    // (round47 实测 20min 5 次 RAW: M2E0 x3/M2E3/M2E4). FDLT=0xFF 冻结已保证触摸期
    // baseline 不塌陷, "释放后回升快"需求不存在, NCLR=31 抑制噪声推高且无副作用.
    writeRegister(MPR121_FDLR, 0x00);

    writeRegister(MPR121_MHDF, 0x01);
    // falling baseline tracking slowed: fix slow-swipe / light-press missed trigger
    // NCLF 1->63 (need 63 consecutive samples before baseline follows touch),
    // NHDF 2->1 (smaller noise-step), FDLF 0->4 (small-delta delay)
    writeRegister(MPR121_NHDF, 0x01);
    writeRegister(MPR121_NCLF, 0x7F);  // round45k: 63->127 slower downward. round32 lowered to 63 to fix baseline-too-high (idle false touch), but 63 too fast: during touch filt drops, baseline tracks it down -> diff shrinks -> sudden release + cannot retrigger for seconds (baseline too low). 127 needs 127ms sustained low filt before baseline drops 1, so short/medium touches dont collapse baseline. Idle baseline drift now handled by software baseline correction (round45i) instead of aggressive NCLF.
    writeRegister(MPR121_FDLF, 0x04);

    // round47: 触摸期(touched filter)baseline 冻结 —— 修复长按塌陷
    // 此前 NHDT/NCLT/FDLT 全为 0x00,触摸期间 baseline 继续跟踪(NCLF 塌陷);
    // AN3891/esp-idf 驱动确认 FDLT=0xFF 可在触摸期间禁用 baseline 跟踪。
    writeRegister(MPR121_NHDT, 0x01);  // round47: 0->1, 触摸期噪声半增量阈值
    writeRegister(MPR121_NCLT, 0x10);  // round47: 0->16, 触摸期噪声计数限制
    writeRegister(MPR121_FDLT, 0xFF);  // round47: 0->255, 触摸期禁用 baseline 跟踪(冻结)

    writeRegister(MPR121_DEBOUNCE, 0);
    // CONFIG1: original 0x10 (FFI=6, CDC=16uA) - unchanged
    writeRegister(MPR121_CONFIG1, 0x10);
    // phase2: SFI 4->2 (0x20->0x28). ESI kept at 1ms (0x29 caused delay/missed keys).
    writeRegister(MPR121_CONFIG2, 0x28);  // CDT=0.5us, SFI=2, ESI=1ms

    setAutoconfig(autoconfig);

    // enable X electrodes = start MPR121
    // CL Calibration Lock: B10 = 5 bits for baseline tracking
    // ELEPROX_EN  proximity: disabled
    // ELE_EN Electrode Enable:  amount of electrodes running (12)
    uint8_t ECR_SETTING = 0b10000000 + 12;
    writeRegister(MPR121_ECR, ECR_SETTING);  // start with above ECR setting
    good = true;
}

/*!
 *  @brief      returns true if this mpr121 is ready.
 */
bool MPR121::ready() {
    return good;
}

/*!
 *  @brief      Enable autoconfig option.
 *  @param      autoconfig
 *              enable / disabled autoconfig
 *              when enabled, the MPR121 automatically searches and sets the
 *              charging parameters for every enabled pad.
 *              this happens on each time the MPR121 transitions
 *              from Stop Mode to Run Mode.
 */
void MPR121::setAutoconfig(bool autoconfig) {
    // register map at
    // https://www.nxp.com/docs/en/data-sheet/MPR121.pdf#page=17&zoom=auto,-416,747
    if (autoconfig) {
        // recommend settings found at
        // https://www.nxp.com/docs/en/application-note/AN3889.pdf#page=9&zoom=310,-42,373
        // FFI (First Filter Iterations) same as FFI in CONFIG1
        // FFI           → 00 Sets samples taken to 6 (Default)
        // RETRY
        // RETRY
        // BVA same as CL in ECR
        // BVA same as CL in ECR
        // ARE Auto-Reconfiguration Enable
        // ACE Auto-Configuration Enable
        // 0x0B == 0b00001011
        writeRegister(MPR121_AUTOCONFIG0, 0b00001011);

        // details on values
        // https://www.nxp.com/docs/en/application-note/AN3889.pdf#page=7&zoom=310,-42,792
        // correct values for Vdd = 3.3V
        writeRegister(MPR121_UPLIMIT, 200);      // ((Vdd - 0.7)/Vdd) * 256
        writeRegister(MPR121_TARGETLIMIT, 180);  // UPLIMIT * 0.9
        writeRegister(MPR121_LOWLIMIT, 130);     // UPLIMIT * 0.65
    } else {
        // really only disable ACE.
        writeRegister(MPR121_AUTOCONFIG0, 0b00001010);
    }
}

/*!
 *  @brief      Set the touch and release thresholds for all 13 channels on the
 *              device to the passed values. The threshold is defined as a
 *              deviation value from the baseline value, so it remains constant
 * even baseline value changes. Typically the touch threshold is a little bigger
 * than the release threshold to touch debounce and hysteresis. For typical
 * touch application, the value can be in range 0x05~0x30 for example. The
 * setting of the threshold is depended on the actual application. For the
 * operation details and how to set the threshold refer to application note
 * AN3892 and MPR121 design guidelines.
 *  @param      touch
 *              the touch threshold value from 0 to 255.
 *  @param      release
 *              the release threshold from 0 to 255.
 */
void MPR121::setThresholds(uint8_t touch, uint8_t release) {
    // set all thresholds (the same)
    for (uint8_t i = 0; i < 12; i++) {
        writeRegister(MPR121_TOUCHTH_0 + 2 * i, touch);
        writeRegister(MPR121_RELEASETH_0 + 2 * i, release);
    }
}

void MPR121::setThresholdsForElectrode(uint8_t electrode, uint8_t touch, uint8_t release) {
    if (electrode > 11) return;
    writeRegister(MPR121_TOUCHTH_0 + 2 * electrode, touch);
    writeRegister(MPR121_RELEASETH_0 + 2 * electrode, release);
}

// /*!
//  *  @brief      Set the touch and release thresholds for each channels on the
//  *              device to the passed values.
//  *  @param      touch
//  *              the touch threshold collection.
//  *  @param      release
//  *              the release threshold collection.
//  */
// void MPR121::setThresholds(mpr121Thresholds* Thresholds) {
//     for (int i = 0; i < 12; i++) {
//         writeRegister(MPR121_TOUCHTH_0 + 2 * i, Thresholds->touch[i]);
//         writeRegister(MPR121_RELEASETH_0 + 2 * i, Thresholds->release[i]);
//     }
// }

/*!
 *  @brief      Set the touch and release debounce value for every channel.
 *  @param      dt
 *              touch.
 *  @param      dr
 *              release.
 */
void MPR121::setDebounce(uint8_t dt, uint8_t dr) {
    dt &= 0b111;
    dr &= 0b111;
    uint8_t debounce = dt | (dr << 4);
    writeRegister(MPR121_DEBOUNCE, debounce);
}

void MPR121::calibrateBaseline(bool force) {
    // Stage 1: let filtered data settle (third filter time constant)
    sleep_ms(20);

    // Stage 2: wait for untouched window (1 retry, max 100ms). Skipped if force=true
    // (sticky-key recovery path: touched()!=1 is stuck, must recalibrate anyway).
    if (!force) {
        if (touched() != 0) {
            sleep_ms(100);
            if (touched() != 0) return;
        }
    }

    // Stage 3: 8-sample burst, 2ms interval. Trimmed mean (drop min/max) for
    // outlier resistance. Total ~16ms, well within <1s budget.
    const uint8_t N = 8;
    uint16_t samples[12][N];
    for (uint8_t s = 0; s < N; s++) {
        for (uint8_t i = 0; i < 12; i++) {
            samples[i][s] = filteredData(i);
        }
        if (s < N - 1) sleep_ms(2);
    }

    // Stage 4: compute trimmed mean per electrode. Every electrode gets a
    // best-effort baseline (trimmed mean drops min/max). Previously channels
    // with jitter > 3 LSB were skipped (valid=false), leaving the power-on
    // default baseline which permanently disabled touch on weak/noisy
    // electrodes such as MPR2 ELE0 (slider cell 17). Now all electrodes seeded.
    uint8_t bl[12];
    bool valid[12] = {false};
    for (uint8_t i = 0; i < 12; i++) {
        uint16_t mn = 0xFFFF, mx = 0;
        for (uint8_t s = 0; s < N; s++) {
            if (samples[i][s] < mn) mn = samples[i][s];
            if (samples[i][s] > mx) mx = samples[i][s];
        }
        uint32_t sum = 0;
        for (uint8_t s = 0; s < N; s++) sum += samples[i][s];
        sum -= mn; sum -= mx;  // trimmed: drop min and max
        bl[i] = (sum / (N - 2)) >> 2;
        // Subtract 1 LSB margin so baseline is slightly BELOW idle filtered data.
        // This prevents false touches from baseline calibration noise (trimmed mean
        // can be 1-2 LSB above true idle due to sample timing). With margin,
        // idle diff = -1 (below threshold) instead of 0-2 (at threshold edge).
        if (bl[i] > 1) bl[i] -= 1;  // round36: -2->-1, software verification handles false touches
        valid[i] = true;
    }

    // Stage 5: CL-seeded baseline (method 2)
    // 5.1 temporarily disable autoconfig BVA so Stop->Run won't override baseline
    uint8_t autoconfig0_backup = readRegister8(MPR121_AUTOCONFIG0);
    writeRegister(MPR121_AUTOCONFIG0, autoconfig0_backup & ~0x30);

    // 5.2 enter Stop Mode
    uint8_t ecr_backup = readRegister8(MPR121_ECR);
    uint8_t data[2];
    data[0] = MPR121_ECR;
    data[1] = 0x00;
    i2c_write(port, addr, data, 2, false);

    // 5.3 write baseline registers (Stop Mode, direct I2C)
    for (uint8_t i = 0; i < 12; i++) {
        if (!valid[i]) continue;
        data[0] = MPR121_BASELINE_0 + i;
        data[1] = bl[i];
        i2c_write(port, addr, data, 2, false);
    }

    // 5.4 disable autoconfig entirely (ACE=0) - already in Stop mode, direct write.
    // Periodic charge recalibration causes periodic false triggers. Frozen.
    data[0] = MPR121_AUTOCONFIG0;
    data[1] = 0x00;
    i2c_write(port, addr, data, 2, false);

    // 5.5 Set ECR with CL=11 (load baseline from registers AND enable tracking).
    // Single Stop->Run transition: loads written baseline values immediately
    // and enables baseline tracking in one atomic operation.
    // This replaces the fragile two-step approach (CL=01 then CL=10) where
    // the second write was silently lost because the MPR121 was still
    // processing the first Stop->Run transition. CL=11 does both in one step.
    // ECR = 0xCC: CL=11, ELEPROX=00, ELE_EN=12
    data[0] = MPR121_ECR;
    data[1] = (ecr_backup & 0x0F) | 0xC0;  // CL=11, ELEPROX=00, ELE_EN preserved
    i2c_write(port, addr, data, 2, false);

    // Verify ECR was accepted (readback with retry).
    sleep_ms(1);
    for (uint8_t retry = 0; retry < 3; retry++) {
        uint8_t ecr_now = readRegister8(MPR121_ECR);
        if (ecr_now == data[1]) break;  // verified
        sleep_ms(1);
        i2c_write(port, addr, data, 2, false);
    }
}

/*!
 *  @brief      Read the filtered data from channel t. The ADC raw data outputs
 *              run through 3 levels of digital filtering to filter out the high
 * frequency and low frequency noise encountered. For detailed information on
 * this filtering see page 6 of the device datasheet.
 *  @param      t
 *              the channel to read
 *  @returns    the filtered reading as a 10 bit unsigned value
 */
uint16_t MPR121::filteredData(uint8_t t) {
    if (t > 12)
        return 0;
    return readRegister16(MPR121_FILTDATA_0L + t * 2);
}

/*!
 *  @brief      Read the baseline value for the channel. The 3rd level filtered
 *              result is internally 10bit but only high 8 bits are readable
 * from registers 0x1E~0x2A as the baseline value output for each channel.
 *  @param      t
 *              the channel to read.
 *  @returns    the baseline data that was read
 */
uint16_t MPR121::baselineData(uint8_t t) {
    if (t > 12)
        return 0;
    uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
    return (bl << 2);
}

// round46b: bulk register read (single I2C transaction) - used by the fast
// CMD_DEBUG_DIFF diagnostic (9 transactions instead of 75 single-byte reads).
// round46i: returns false on failed transaction and zeroes dst - a failed
// bulk read previously left uninitialized stack garbage in dst, which the
// diff clamp turned into false diff=255 spikes (th_touch=257 calibration).
bool MPR121::readRegisters(uint8_t reg, uint8_t* dst, uint8_t n) {
    uint8_t data[1] = { reg };
    int ret = i2c_write_read(port, addr, data, 1, dst, n);
    if (ret < 0) {
        memset(dst, 0, n);
        return false;
    }
    return true;
}

/**
 *  @brief      Read the touch status of all 13 channels as bit values in a 12
 * bit integer.
 *  @returns    a 12 bit integer with each bit corresponding to the touch status
 *              of a sensor. For example, if bit 0 is set then channel 0 of the
 * device is currently deemed to be touched.
 */
uint16_t MPR121::touched(void) {
    uint8_t reg = MPR121_TOUCHSTATUS_L;
    uint8_t buffer[2] = {0, 0};
    int ret = i2c_write_read(port, addr, &reg, 1, buffer, 2);
    if (ret < 0) {
        sleep_us(50);
        ret = i2c_write_read(port, addr, &reg, 1, buffer, 2);
        if (ret < 0) return 0;  // I2C error: safe default = no touch
    }
    uint16_t t = buffer[1];
    t <<= 8;
    t |= buffer[0];
    return t & 0x0FFF;
}

/*!
 *  @brief      Read the contents of an 8 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 8 bit value that was read.
 */
uint8_t MPR121::readRegister8(uint8_t reg) {
    uint8_t buffer[1] = {0};
    int ret = i2c_write_read(port, addr, &reg, 1, buffer, 1);
    if (ret < 0) {
        sleep_us(50);
        i2c_write_read(port, addr, &reg, 1, buffer, 1);
    }
    return buffer[0];
}

/*!
 *  @brief      Read the contents of a 16 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 16 bit value that was read.
 */
uint16_t MPR121::readRegister16(uint8_t reg) {
    uint8_t buffer[2] = {0, 0};
    int ret = i2c_write_read(port, addr, &reg, 1, buffer, 2);
    if (ret < 0) {
        sleep_us(50);
        i2c_write_read(port, addr, &reg, 1, buffer, 2);
    }
    uint16_t val = buffer[1];
    val <<= 8;
    val |= buffer[0];
    return val;
}

/*!
    @brief  Writes 8-bits to the specified destination register
    @param  reg the register address to write to
    @param  value the value to write
*/
void MPR121::writeRegister(uint8_t reg, uint8_t value) {
    // MPR121 must be put in Stop Mode to write to most registers
    bool stop_required = true;

    // first get the current set value of the MPR121_ECR register
    // Adafruit_BusIO_Register ecr_reg = Adafruit_BusIO_Register(i2c_dev, MPR121_ECR, 1);

    // uint8_t ecr_backup = ecr_reg.read();

    uint8_t ecr_backup = readRegister8(MPR121_ECR);

    if ((reg == MPR121_ECR) || ((0x73 <= reg) && (reg <= 0x7A))) {
        stop_required = false;
    }

    uint8_t data[2] = { 0x00, 0x00 };

    if (stop_required) {
        // clear this register to set stop mode
        // ecr_reg.write(0x00);
        data[0] = MPR121_ECR;
        data[1] = 0x00;
        i2c_write(port, addr, data, 2, false);
    }

    // Adafruit_BusIO_Register the_reg = Adafruit_BusIO_Register(i2c_dev, reg, 1);
    // the_reg.write(value);
    data[0] = reg;
    data[1] = value;
    i2c_write(port, addr, data, 2, false);

    if (stop_required) {
        // write back the previous set ECR settings
        // ecr_reg.write(ecr_backup);
        data[0] = MPR121_ECR;
        data[1] = ecr_backup;
        // If CL=11 (load+track from calibrateBaseline), switch to CL=10 (track only).
        // This prevents baseline reload on every subsequent Stop->Run cycle.
        if ((ecr_backup & 0xC0) == 0xC0) {
            data[1] = (ecr_backup & 0x0F) | 0x80;  // CL=10
        }
        i2c_write(port, addr, data, 2, false);
    }
}

/*!
    @brief  Run 模式下直写 baseline 寄存器(0x1E+e),不做 Stop->Run 切换。
    @param  e    电极索引(0-11),寄存器地址 = MPR121_BASELINE_0 + e
    @param  val  8 位 baseline 值
    @note   round47: 与 writeRegister() 的区别 —— writeRegister 会先 Stop
            再 Run 整个芯片(全局中断、ECR 恢复);writeBaselineRun 只写
            0x1E+e 一个寄存器,可在触摸检测运行中单独更新某个电极的
            baseline,不打扰其他电极的跟踪。
*/
void MPR121::writeBaselineRun(uint8_t e, uint8_t val) {
    if (e > 11) return;  // round47-review: guard against out-of-range electrode (0x2B+ would hit filter regs)
    uint8_t data[2] = { (uint8_t)(MPR121_BASELINE_0 + e), val };
    i2c_write(port, addr, data, 2, false);
}
