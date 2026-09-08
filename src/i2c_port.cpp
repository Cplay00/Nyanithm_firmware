/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <debug.h>
#include <hardware/i2c.h>
#include <hardware/sync.h>
#include <i2c_config.h>
#include <i2c_port.h>
#include <pico/stdlib.h>

// Per-bus spinlock: serializes I2C access across cores (Core0 touch polling vs
// Core1 CDC/debug reads). Without this, concurrent i2c_blocking calls on the
// same bus corrupt transactions, producing garbage register reads.
static spin_lock_t *i2c_locks[2] = { nullptr, nullptr };

uint I2C_SDA[2] = { 0, 4 };
uint I2C_SCL[2] = { 1, 5 };

// round88e: the previous fixed 5 ms absolute deadline was a WHOLE-transaction
// budget, silently capping the maximum transfer length: the 129-byte MBR3116
// config burst needs ~2.9 ms at BR400K but ~5.8 ms at BR200K (boot production
// mode), so every NVM config write returned PICO_ERROR_TIMEOUT before the
// chip even ACKed the address. Scale the deadline with length -- 100 us/byte
// is ~2.2x the 200 kHz byte time -- keeping the 5 ms base for controller
// setup, clock stretch and STOP. Healthy transactions are unaffected; only
// failure detection on a wedged bus slows (worst 17.9 ms for the config
// burst), which every caller tolerates.
static absolute_time_t i2cDeadline(size_t len) {
    return make_timeout_time_us(5000 + (uint64_t)len * 100);
}

int i2c_write(uint8_t port, uint8_t addr, uint8_t* src, size_t len, bool nostop) {
#if HWI2C
    if (port >= 2) return PICO_ERROR_INVALID_ARG;
    spin_lock_t *lk = i2c_locks[port];
    uint32_t save = 0;
    if (lk) save = spin_lock_blocking(lk);
    i2c_inst_t *i2c = (port == 0) ? i2c0 : i2c1;
    int ret = i2c_write_blocking_until(i2c, addr, src, len, nostop, i2cDeadline(len));
    if (lk) spin_unlock(lk, save);
    return ret;
#else
    return PICO_ERROR_GENERIC;
#endif
}
int i2c_read(uint8_t port, uint8_t addr, uint8_t* dst, size_t len, bool nostop) {
#if HWI2C
    if (port >= 2) return PICO_ERROR_INVALID_ARG;
    spin_lock_t *lk = i2c_locks[port];
    uint32_t save = 0;
    if (lk) save = spin_lock_blocking(lk);
    i2c_inst_t *i2c = (port == 0) ? i2c0 : i2c1;
    int ret = i2c_read_blocking_until(i2c, addr, dst, len, nostop, i2cDeadline(len));
    if (lk) spin_unlock(lk, save);
    return ret;
#else
    return PICO_ERROR_GENERIC;
#endif
}

// Atomic write-read: holds the spinlock for the ENTIRE write+read sequence.
// Prevents the other core from changing the MPR121 register pointer between
// write (register address) and read (data), which caused corrupted reads
// and false touch detections.
int i2c_write_read(uint8_t port, uint8_t addr, uint8_t* wr, size_t wr_len, uint8_t* rd, size_t rd_len) {
#if HWI2C
    if (port >= 2) return PICO_ERROR_INVALID_ARG;
    spin_lock_t *lk = i2c_locks[port];
    uint32_t save = 0;
    if (lk) save = spin_lock_blocking(lk);
    i2c_inst_t *i2c = (port == 0) ? i2c0 : i2c1;
    int ret = i2c_write_blocking_until(i2c, addr, wr, wr_len, true, i2cDeadline(wr_len));
    if (ret == (int)wr_len) {
        ret = i2c_read_blocking_until(i2c, addr, rd, rd_len, false, i2cDeadline(rd_len));
    } else if (ret >= 0) {
        ret = PICO_ERROR_GENERIC;
    }
    if (lk) spin_unlock(lk, save);
    return ret;
#else
    return PICO_ERROR_GENERIC;
#endif
}

// CY8CMBR3xxx requires the data-pointer write to end with STOP before the
// following read. Hold the software lock across both native SDK calls so no
// other core can replace the pointer between those two bus transactions.
int i2c_write_stop_read(uint8_t port, uint8_t addr, uint8_t reg, uint8_t* rd, size_t rd_len) {
#if HWI2C
    if (port >= 2) return PICO_ERROR_INVALID_ARG;
    spin_lock_t *lk = i2c_locks[port];
    uint32_t save = 0;
    if (lk) save = spin_lock_blocking(lk);
    i2c_inst_t *i2c = (port == 0) ? i2c0 : i2c1;
    int ret = i2c_write_blocking_until(i2c, addr, &reg, 1, false, i2cDeadline(1));
    if (ret == 1) {
        ret = i2c_read_blocking_until(i2c, addr, rd, rd_len, false, i2cDeadline(rd_len));
    } else if (ret >= 0) {
        ret = PICO_ERROR_GENERIC;
    }
    if (lk) spin_unlock(lk, save);
    return ret;
#else
    return PICO_ERROR_GENERIC;
#endif
}

void initI2CBus(uint8_t port, uint gpio_sda, uint gpio_scl, uint baudrate) {
    I2C_SDA[port] = gpio_sda;
    I2C_SCL[port] = gpio_scl;
    // Claim a spinlock for this bus on first init (multicore serialization).
    if (i2c_locks[port] == nullptr) {
        int lock_num = spin_lock_claim_unused(true);
        i2c_locks[port] = spin_lock_init(lock_num);
    }
#if HWI2C
    int br;
    // I2C Initialisation
    if (port == 0)
        br = i2c_init(i2c0, baudrate);
    if (port == 1)
        br = i2c_init(i2c1, baudrate);
#if ENABLE_DEBUG
    // printf("baudrate:%d\n", br);
#endif
    gpio_set_function(gpio_sda, GPIO_FUNC_I2C);
    gpio_set_function(gpio_scl, GPIO_FUNC_I2C);
    gpio_pull_up(gpio_sda);
    gpio_pull_up(gpio_scl);
#else
#warning Using Software I2C Interface
    gpio_init(gpio_sda);
    gpio_init(gpio_scl);
    gpio_pull_up(gpio_sda);
    gpio_pull_up(gpio_scl);
#endif
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void scanI2CBus(uint8_t port) {
    // printf("I2C Bus Scan\n");
    uint8_t total_devices = 0;
    for (int addr = 0; addr < (1 << 7); addr++) {

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read(port, addr, &rxdata, 1, false);

        if (ret >= 0) {
            printf("device at 0x%02x\n", addr);
            total_devices++;
        }
    }
    printf("%d devices found\n", total_devices);
    printf("Done.\n");
}

bool findI2CDevice(uint8_t port, uint8_t address,uint32_t timeout_ms) {
    uint8_t buf[1];
    for(int i  = 0 ; i < timeout_ms ; i++){
        if(i2c_read(port, address, buf, 1, false) >= 0){
            return true;
        }
        sleep_ms(1);
    }
    return false;
}
