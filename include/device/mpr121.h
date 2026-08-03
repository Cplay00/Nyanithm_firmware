/*!
 *  @file Adafruit_MPR121.h
 *
 *  This is a library for the MPR121 12-Channel Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */
#ifndef __MPR121_H__
#define __MPR121_H__
#include <i2c_port.h>

#include <nyanithm_shared.h>

// The default I2C address
#define MPR121_I2CADDR_DEFAULT 0x5A  ///< default I2C address

/*!
 *  Device register map
 */
enum {
    MPR121_TOUCHSTATUS_L = 0x00,
    MPR121_TOUCHSTATUS_H = 0x01,
    MPR121_FILTDATA_0L = 0x04,
    MPR121_FILTDATA_0H = 0x05,
    MPR121_BASELINE_0 = 0x1E,
    MPR121_MHDR = 0x2B,
    MPR121_NHDR = 0x2C,
    MPR121_NCLR = 0x2D,
    MPR121_FDLR = 0x2E,
    MPR121_MHDF = 0x2F,
    MPR121_NHDF = 0x30,
    MPR121_NCLF = 0x31,
    MPR121_FDLF = 0x32,
    MPR121_NHDT = 0x33,
    MPR121_NCLT = 0x34,
    MPR121_FDLT = 0x35,

    MPR121_TOUCHTH_0 = 0x41,
    MPR121_RELEASETH_0 = 0x42,
    MPR121_DEBOUNCE = 0x5B,
    MPR121_CONFIG1 = 0x5C,
    MPR121_CONFIG2 = 0x5D,
    MPR121_CHARGECURR_0 = 0x5F,
    MPR121_CHARGETIME_1 = 0x6C,
    MPR121_ECR = 0x5E,
    MPR121_AUTOCONFIG0 = 0x7B,
    MPR121_AUTOCONFIG1 = 0x7C,
    MPR121_UPLIMIT = 0x7D,
    MPR121_LOWLIMIT = 0x7E,
    MPR121_TARGETLIMIT = 0x7F,

    MPR121_GPIODIR = 0x76,
    MPR121_GPIOEN = 0x77,
    MPR121_GPIOSET = 0x78,
    MPR121_GPIOCLR = 0x79,
    MPR121_GPIOTOGGLE = 0x7A,

    MPR121_SOFTRESET = 0x80,
};

class MPR121 {

    uint8_t port;
    uint8_t addr;
    bool good = false;

public:
    MPR121(uint8_t _port, uint8_t i2c_addr);

    bool ready();

    void init(uint8_t touchThreshold, uint8_t releaseThreshold, bool autoconfig);

    uint16_t filteredData(uint8_t t);
    uint16_t baselineData(uint8_t t);
    bool readRegisters(uint8_t reg, uint8_t* dst, uint8_t n);  // round46b: bulk read; round46i: returns success

    void setAutoconfig(bool autoconfig);

    uint8_t readRegister8(uint8_t reg);
    uint16_t readRegister16(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void writeBaselineRun(uint8_t e, uint8_t val);  // round47: Run 模式直写 baseline(0x1E+e),不做 Stop->Run
    uint16_t touched(void);

    void setThresholds(uint8_t touch, uint8_t release);

    void setThresholdsForElectrode(uint8_t electrode, uint8_t touch, uint8_t release);

    void setDebounce(uint8_t dt,uint8_t dr);

    void calibrateBaseline(bool force = false);
};

#endif
