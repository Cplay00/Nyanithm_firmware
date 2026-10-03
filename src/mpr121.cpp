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
    // Preserve the accepted rising-filter setting. NCL is a sample count,
    // not milliseconds; FDLT below slows tracking and does not freeze it.
    writeRegister(MPR121_NCLR, 0x1F);
    writeRegister(MPR121_FDLR, 0x00);

    writeRegister(MPR121_MHDF, 0x01);
    // falling baseline tracking slowed: fix slow-swipe / light-press missed trigger
    // Preserve the accepted falling-filter settings. See NXP AN3891 for
    // sample-count and filter-delay semantics; neither is a wall-clock time.
    writeRegister(MPR121_NHDF, 0x01);
    writeRegister(MPR121_NCLF, 0x7F);
    writeRegister(MPR121_FDLF, 0x04);

    // Touched-state baseline filter: FDLT=255 slows tracking (AN3891),
    // it does not disable tracking. Values stay unchanged in round90m.
    writeRegister(MPR121_NHDT, 0x01);
    writeRegister(MPR121_NCLT, 0x10);
    writeRegister(MPR121_FDLT, 0xFF);

    writeRegister(MPR121_DEBOUNCE, 0);
    // CONFIG1: original 0x10 (FFI=6, CDC=16uA) - unchanged
    writeRegister(MPR121_CONFIG1, 0x10);
    // SFI encoding 01 means 6 samples; ESI=1ms, nominal output period=6ms.
    writeRegister(MPR121_CONFIG2, 0x28);  // CDT=0.5us, SFI=6, ESI=1ms

    setAutoconfig(autoconfig);

    // enable X electrodes = start MPR121
    // CL=10: enable tracking and seed from the first electrode data's high 5 bits.
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

    // Stage 5: legacy startup policy retained for a separate controlled change.
    // NOTE: 0x30 addresses RETRY, not BVA (bits 3:2). The CL=11 resume below
    // loads the first electrode data, so these manual writes do not establish
    // a persistent manual seed. Do not infer the effective baseline from bl[].
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

    // 5.5 CL=11 enables tracking and seeds all 10 bits from the first
    // electrode data (datasheet 5.11), not from the baseline registers.
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
    uint16_t value = 0;
    readFilteredData(t, value);
    return value;
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
    uint16_t value = 0;
    readBaselineData(t, value);
    return value;
}

// round46b: bulk register read (single I2C transaction) - used by the fast
// CMD_DEBUG_DIFF diagnostic (9 transactions instead of 75 single-byte reads).
// round46i: returns false on failed transaction and zeroes dst - a failed
// bulk read previously left uninitialized stack garbage in dst, which the
// diff clamp turned into false diff=255 spikes (th_touch=257 calibration).
bool MPR121::readRegisters(uint8_t reg, uint8_t* dst, uint8_t n) {
    if (dst == nullptr || n == 0) return false;
    uint8_t data[1] = { reg };
    int ret = i2c_write_read(port, addr, data, 1, dst, n);
    if (ret != n) {
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
    uint16_t status = 0;
    readTouchStatus(status);
    return status;
}

bool MPR121::readRegisterChecked(uint8_t reg, uint8_t* dst, uint8_t n, bool* retried) {
    if (retried) *retried = false;
    if (readRegisters(reg, dst, n)) return true;
    sleep_us(50);
    if (retried) *retried = true;
    return readRegisters(reg, dst, n);
}

bool MPR121::readTouchStatus(uint16_t& status) {
    status = 0;
    uint8_t buffer[2] = {};
    if (!readRegisterChecked(MPR121_TOUCHSTATUS_L, buffer, 2)) return false;
    status = ((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8)) & 0x0FFF;
    return true;
}

bool MPR121::readFilteredData(uint8_t t, uint16_t& value, bool* retried) {
    value = 0;
    if (retried) *retried = false;
    if (t > 12) return false;
    uint8_t buffer[2] = {};
    if (!readRegisterChecked(MPR121_FILTDATA_0L + t * 2, buffer, 2, retried)) return false;
    uint16_t reading = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    if (reading > 0x03FF) return false;  // native 10-bit output; reserved bits are not data
    value = reading;
    return true;
}

bool MPR121::readBaselineData(uint8_t t, uint16_t& value, bool* retried) {
    value = 0;
    if (retried) *retried = false;
    if (t > 12) return false;
    uint8_t buffer[1] = {};
    if (!readRegisterChecked(MPR121_BASELINE_0 + t, buffer, 1, retried)) return false;
    value = (uint16_t)buffer[0] << 2;  // only high 8 bits of the native baseline are readable
    return true;
}

/*!
 *  @brief      Read the contents of an 8 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 8 bit value that was read.
 */
uint8_t MPR121::readRegister8(uint8_t reg) {
    uint8_t buffer[1] = {0};
    readRegisterChecked(reg, buffer, 1);
    return buffer[0];
}

/*!
 *  @brief      Read the contents of a 16 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 16 bit value that was read.
 */
uint16_t MPR121::readRegister16(uint8_t reg) {
    uint8_t buffer[2] = {0, 0};
    readRegisterChecked(reg, buffer, 2);
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
        // Legacy CL=11 -> CL=10 policy retained. Both modes reload from the
        // first electrode data on Stop/Run (all 10 bits vs the high 5 bits).
        if ((ecr_backup & 0xC0) == 0xC0) {
            data[1] = (ecr_backup & 0x0F) | 0x80;  // CL=10
        }
        i2c_write(port, addr, data, 2, false);
    }
}
