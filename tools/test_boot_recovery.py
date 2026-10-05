#!/usr/bin/env python3
"""Run production startup/check/air functions with failed peripherals and a 2s WDT.

Offline C++ integration, no device access. One process per case; extracts the
current source rather than duplicating startup logic. Uses the existing LLVM
host-test runner and real shared configuration header.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import test_mbr_distance_pipeline as integration

HOST = r'''
extern "C" { int _fltused = 0; }
static controller_config ControllerConfig{};
static bool usingIR, useMuxScan;
static bool hardwareMismatchAtBoot;
static uint8_t detectedMprMask, detectedMbrMask;
static uint8_t g_tofReadyMask, g_tofPhysicalMask;
static uint8_t g_lampCount = 31, heightRange = 10;
static uint16_t heightDataOriginal[5] = {4095,4095,4095,4095,4095};
static int16_t heightData[5] = {4094,4094,4094,4094,4094};
static uint8_t touchData32[32];
static bool airKeys[6];
static uint32_t lastFeed, maxFeedGap, wdtDelay;
static unsigned feeds, warningCycles, errorCycles, muxChannel, muxMask, resets;
static unsigned polls[5], forceCalls[5], startCalls[5];
static unsigned mprInitCalls, mprCalibrationCalls, mbrInitCalls;
static uint8_t physicalAddress[5] = {0x29,0x29,0x29,0x29,0x29};
static uint8_t presentMask = 15, initFailureMask, probeFailureMask;
static uint8_t mprPresent = 0, mbrPresent = 7;
static unsigned idReads[5], wakeNacks;
static bool idTimeout;
static bool muxPresent = true, irPresent, addressFailure;
static uint32_t forceDuration = 20, calibrationDuration = 30;
static bool rangeCompletes = true;
static bool mprInitializationGood = true;
static bool checkThresholdWrites;
static unsigned thresholdWrites;
static uint16_t rangeValues[5] = {100,100,100,100,100};
static const unsigned GPIO_TOF_RESET = 5, GPIO_PCA9545_RESET = 4;
static const unsigned MPR121_AUTOCONFIG0 = 0x7b, BUTTON_PUSH = 2;
static const bool HIGH = true, LOW = false;
static void advance(uint32_t ms) {
    fakeNow += ms;
    if (wdtDelay) {
        uint32_t gap = fakeNow - lastFeed;
        if (gap > maxFeedGap) maxFeedGap = gap;
        CHECK(gap < wdtDelay);
    }
}
static void sleep_ms(uint32_t ms) { advance(ms); }
static void watchdog_update() { lastFeed = fakeNow; ++feeds; }
static void watchdog_enable(uint32_t ms, bool pause) {
    CHECK(ms == 2000 && pause); wdtDelay = ms; watchdog_update();
}
static void gpio_init(unsigned) {}
static void gpio_set_dir(unsigned, bool) {}
static void gpio_pull_up(unsigned) {}
static void gpio_put(unsigned pin, bool level) {
    if (pin == GPIO_TOF_RESET && !level) {
        ++resets;
        for (unsigned i = 0; i < 5; ++i) physicalAddress[i] = 0x29;
    }
}
static unsigned redCycles, blackCycles;
static uint32_t redAt[3], blackAt[2];
struct FakeLed {
    void fill(unsigned r, unsigned g, unsigned b) {
        if (r == 255 && g == 0 && b == 0) {
            CHECK(redCycles < 3); redAt[redCycles++] = fakeNow;
        }
        if (r == 0 && g == 0 && b == 0 && redCycles > 0 && blackCycles < 2)
            blackAt[blackCycles++] = fakeNow;
    }
    void fill(unsigned r, unsigned g, unsigned b, unsigned first, unsigned count) {
        CHECK(r == 15 && g == 15 && b == 15 && first == 0 && count == g_lampCount);
    }
    void flush() {}
};
static FakeLed RGB_LED;
static void tud_task() {}
static bool getButtonState(unsigned) { return false; }
class PCA954X {
public:
    constexpr PCA954X(unsigned, unsigned, unsigned) {}
    void init() { advance(2); }
    int setChannel(uint8_t channel) {
        muxChannel = channel; muxMask = 1u << channel;
        advance(1); return muxPresent ? 1 : -1;
    }
    int setReg(uint8_t mask) { muxMask = mask; advance(1); return muxPresent ? 1 : 0; }
};
static PCA954X mux0(1,0x70,GPIO_PCA9545_RESET);
class VL53L0X {
public:
    unsigned id; uint8_t address = 0x29; uint16_t timeout = 0;
    constexpr explicit VL53L0X(unsigned n): id(n) {}
    void setI2CAddressOnly(uint8_t value) { address = value; }
    void setTimeout(uint16_t value) { CHECK(value == 200); timeout = value; }
    bool forceInit() {
        ++forceCalls[id]; advance(forceDuration);
        return muxPresent && muxChannel == id && address == physicalAddress[id] &&
               (presentMask & (1u << id)) && !(initFailureMask & (1u << id));
    }
    bool setMeasurementTimingBudget(unsigned budget) { CHECK(budget == 12000); return false; }
    void setAddress(uint8_t value) {
        if (!addressFailure) physicalAddress[id] = value;
        address = value;
    }
    void startContinuous(unsigned period) {
        CHECK(period == 0); ++startCalls[id];
        CHECK(!(initFailureMask & (1u << id)));
        CHECK(presentMask & (1u << id));
    }
    bool readRangeContinuousMillimetersAsync(uint16_t* range) {
        ++polls[id]; CHECK(presentMask & (1u << id));
        CHECK(!(initFailureMask & (1u << id)));
        CHECK(address == physicalAddress[id]);
        if (useMuxScan) CHECK(muxChannel == id);
        if (!rangeCompletes) return false;
        *range = rangeValues[id]; return true;
    }
};
static VL53L0X tof0(0),tof1(1),tof2(2),tof3(3),tof4(4);
class MPR121 {
public:
    bool ready() { return mprInitializationGood; }
    bool init(unsigned,unsigned,bool) { ++mprInitCalls; advance(10); return true; }
    void writeRegister(unsigned,unsigned) { advance(1); }
    void setThresholdsForElectrode(unsigned e,unsigned touch,unsigned release) {
        if (checkThresholdWrites) {
            CHECK(e < 12);
            const unsigned m = thresholdWrites / 12;
            const bool overridden = ControllerConfig.thTouchKey[0] && m == 1 && e == 11;
            CHECK(touch == (overridden ? 5u : 6u));
            CHECK(release == (overridden ? 2u : 4u));
        }
        ++thresholdWrites; advance(1);
    }
    void setDebounce(unsigned,unsigned) { advance(1); }
    void calibrateBaseline() { ++mprCalibrationCalls; advance(calibrationDuration); }
};
static MPR121 mpr0,mpr1,mpr2;
class CY8CMBR3116 {
public:
    uint8_t i2c_port = 0, DEVICE_I2C_ADDRESS;
    constexpr explicit CY8CMBR3116(uint8_t addr): DEVICE_I2C_ADDRESS(addr) {}
    uint8_t requestDataFromAddress(uint8_t,uint8_t,uint8_t*);
};
static CY8CMBR3116 MBR3116A(0x40),MBR3116B(0x41),MBR3116C(0x42),MBR3116D(0x43),MBR3116E(0x44);
static void initI2C() {}
static void detectIR() { usingIR = irPresent; }
static void initIR() { advance(2); }
static void initCY8CMBR3116() { ++mbrInitCalls; }
static bool findI2CDevice(uint8_t port,uint8_t address,uint32_t = 10) {
    advance(60); // failed timed transfers, not just instantaneous NACKs
    if (port == 0) {
        if (address >= 0x5a && address <= 0x5c) return mprPresent & (1u << (address - 0x5a));
        if (address >= 0x40 && address <= 0x42) return mbrPresent & (1u << (address - 0x40));
        if (address >= 0x43 && address <= 0x44) return mbrPresent & (1u << (address - 0x43));
        CHECK(false); return false;
    }
    if (address == 0x70) return muxPresent;
    if (!muxPresent) return false;
    for (unsigned i = 0; i < 5; ++i)
        if ((presentMask & (1u << i)) && !(probeFailureMask & (1u << i)) &&
            (muxMask & (1u << i)) && physicalAddress[i] == address) return true;
    return false;
}
static int i2c_write_read(uint8_t port, uint8_t addr, uint8_t* reg, unsigned,
                          uint8_t* dst, unsigned len) {
    CHECK(port == 1 && addr == 0x29 && *reg == 0xc0 && len == 1);
    advance(5);
    if (!muxPresent || !(presentMask & (1u << muxChannel))) return -1;
    *dst = 0xee; return 1;
}
static int i2c_write_stop_read(uint8_t port, uint8_t addr, uint8_t reg, uint8_t* dst, unsigned len) {
    CHECK(port == 0 && addr >= 0x40 && addr <= 0x44 && reg == 0x8f && len == 3);
    advance(60);
    const unsigned chip = addr - 0x40;
    ++idReads[chip];
    if (idTimeout && chip == 0) return PICO_ERROR_TIMEOUT;
    if (chip == 0 && idReads[chip] <= wakeNacks) return PICO_ERROR_GENERIC;
    if (!(mbrPresent & (1u << (addr - 0x40)))) return -1;
    dst[0] = 0x9a; dst[1] = 5; dst[2] = 0x0a; return 3;
}
'''

CASES = r'''
extern "C" void bootRecoveryEntry() {
    const char* command = GetCommandLineA(); unsigned number = 99;
    for (unsigned i = 0; command[i]; ++i) if (command[i] == '=') {
        number = 0;
        while (command[++i] >= '0' && command[i] <= '9') number = number * 10 + command[i] - '0';
        break;
    }
    CHECK(number < 30);
    ControllerConfig.hwVer = 1; ControllerConfig.cfg0 = CFG0_BIT_MBR3116;
    ControllerConfig.th_touch = 6; ControllerConfig.th_release = 4;
    ControllerConfig.airMin = 200; ControllerConfig.airMax = 500;
    if (number == 1) { ControllerConfig.hwVer = 2; presentMask = 31; }
    if (number == 2) { ControllerConfig.hwVer = 2; } // 32-inch config on four sensors
    if (number == 3) { muxPresent = false; }
    if (number == 4) { initFailureMask = 2; }
    if (number == 5) { forceDuration = 650; addressFailure = true; }
    if (number == 6) { ControllerConfig.cfg0 = 0; mprPresent = 0; }
    if (number == 7) { irPresent = true; muxPresent = false; }
    if (number == 8) { ControllerConfig.hwVer = 3; ControllerConfig.cfg0 = 0; mbrPresent = 0x18; }
    if (number == 9) { ControllerConfig.hwVer = 4; ControllerConfig.cfg0 = 0; mbrPresent = 0x18; presentMask = 31; }
    if (number == 10) { mbrPresent = 0; }
    if (number == 11) { ControllerConfig.cfg0 = 0; calibrationDuration = 750; mprPresent = 7; mbrPresent = 0; }
    if (number == 12) { presentMask = 0; }
    if (number == 13) { ControllerConfig.hwVer = 2; presentMask = 31; mprPresent = 0; } // 32-inch MBR
    if (number == 14) { ControllerConfig.cfg0 = 0; mbrPresent = 0; mprPresent = 7; } // 27-inch MPR
    if (number == 15) { ControllerConfig.hwVer = 2; ControllerConfig.cfg0 = 0; presentMask = 31; mprPresent = 0; } // wrong MPR selection on 32-inch MBR
    if (number == 16) { mbrPresent = 0; mprPresent = 7; }
    if (number == 17) { presentMask = 31; } // extra fifth sensor still at 0x29
    if (number == 18) { ControllerConfig.hwVer = 3; } // wrong MBR address layout
    if (number == 19) { ControllerConfig.cfg0 |= CFG0_BIT_FORCE16LEDS; mbrPresent = 0; ControllerConfig.lightLimit = 0; } // wrong MBR selection on 27-inch MPR
    if (number == 23) wakeNacks = 2;
    if (number == 24) wakeNacks = 3;
    if (number == 25) idTimeout = true;
    if (number == 26) wakeNacks = 9; // all three discovery probes fail
    if (number == 27) { ControllerConfig.cfg0 = 0; mprPresent = 7; mbrPresent = 0; mprInitializationGood = false; }
    if (number >= 28) {
        ControllerConfig.cfg0 = 0; mprPresent = 7; mbrPresent = 0; checkThresholdWrites = true;
        if (number == 29) { ControllerConfig.thTouchKey[0] = 5; ControllerConfig.thReleaseKey[0] = 2; }
    }
    initHwDevices();
    checkHardwareState();
    CHECK(wdtDelay == 2000 && maxFeedGap < 2000);
    const bool useMbr = (ControllerConfig.cfg0 & CFG0_BIT_MBR3116) || ControllerConfig.hwVer >= 3;
    CHECK(mprInitCalls == (useMbr ? 0u : 3u));
    if (number >= 28) CHECK(thresholdWrites == 36);
#ifndef LEGACY_STARTUP
    if (number == 2) CHECK(useMuxScan && g_tofReadyMask == 15);
    if (number == 3 || number == 7 || number == 12) CHECK(g_tofReadyMask == 0);
    if (number == 4) CHECK(useMuxScan && g_tofReadyMask == 13 && startCalls[1] == 0);
    if (number == 5) CHECK(useMuxScan && g_tofReadyMask == 15 && resets == 2);
    if (number == 0 || number == 8) CHECK(!useMuxScan && g_tofReadyMask == 15);
    if (number == 1 || number == 9) CHECK(!useMuxScan && g_tofReadyMask == 31);
    if (number == 13 || number == 15) CHECK(!useMuxScan && g_tofReadyMask == 31);
    if (number == 14 || number == 16) CHECK(!useMuxScan && g_tofReadyMask == 15);
#endif
    const bool expectedWarning = number == 2 || number == 3 || number == 4 ||
        number == 6 || number == 10 || number == 12 || number == 15 || number == 16 ||
        number == 17 || number == 18 || number == 19 || number == 25 || number == 26 || number == 27;
    CHECK(redCycles == (expectedWarning ? 3u : 0u));
    if (expectedWarning) {
        CHECK(blackAt[0] - redAt[0] == 2000 && blackAt[1] - redAt[1] == 2000);
        CHECK(redAt[1] - blackAt[0] == 500 && redAt[2] - blackAt[1] == 500);
        CHECK(fakeNow - redAt[0] >= 7000 && fakeNow - redAt[0] <= 7020);
    }
    if (number == 23) CHECK(idReads[0] == 3);
    if (number == 24) CHECK(idReads[0] == 4); // wake completes between startup probes
    if (number == 25) CHECK(idReads[0] == 3); // one timeout per independent discovery probe
    if (number == 26) CHECK(idReads[0] == 9);
    if (number >= 20 && number <= 22) {
        rangeValues[0] = 300;
        updateAir();
        bool on = false; for (unsigned j = 0; j < 6; ++j) on |= airKeys[j];
        CHECK(on);
        if (number == 20 || number == 21) rangeValues[0] = number == 20 ? 8190 : 8191;
        if (number == 22) { rangeCompletes = false; advance(200); }
        updateAir(); advance(11); updateAir();
        for (unsigned j = 0; j < 6; ++j) CHECK(!airKeys[j]);
        CHECK(heightData[0] == 4095);
        rangeCompletes = true; rangeValues[0] = 300;
        for (unsigned frame = 0; frame < 3; ++frame) updateAir();
        on = false; for (unsigned j = 0; j < 6; ++j) on |= airKeys[j];
        CHECK(on); // recovery can activate again after cooldown
    }
    // Main-loop reachability and uninitialized-channel exclusion, beyond multiple WDT periods.
    for (unsigned frame = 0; frame < 1000; ++frame) {
        watchdog_update();
        if (!usingIR) updateAir();
        advance(5);
    }
#ifdef LEGACY_STARTUP
    g_tofReadyMask = (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) ? 31 : 15;
#endif
    for (unsigned i = 0; i < 5; ++i)
        CHECK(polls[i] == ((g_tofReadyMask & (1u << i)) && !usingIR ? (number >= 20 && number <= 22 ? 1006u : 1000u) : 0u));
    writeText("PASS checks="); writeNumber(checks); writeText("\n"); ExitProcess(0);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=integration.VARIANT)
    parser.add_argument('--output', type=Path, default=integration.WORKSPACE / '_dev_tools/round90r_boot_host')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sources = {name: (args.source_root / 'src' / name).read_text(encoding='utf-8')
               for name in ('hw_devices.cpp', 'hw_check.cpp', 'cy8cmbr3116.cpp')}
    pieces = []
    lane_helpers, _ = integration.extract_block(sources['hw_devices.cpp'], 'static const uint8_t V1_LANE_M', 'void initMPR121()')
    pieces.append(lane_helpers)
    hashes = {}
    for name, signatures in (
        ('cy8cmbr3116.cpp', ['uint8_t CY8CMBR3116::requestDataFromAddress(']),
        ('hw_devices.cpp', ['void initToFReset()', 'void resetToF()', 'void initToF()',
                            'void initMPR121()', 'void initHwDevices()', 'void updateAir()']),
        ('hw_check.cpp', ['static void startupWait(', 'bool hardwareConfigMismatch()', 'void checkHardwareState()']),
    ):
        for signature in signatures:
            piece, line = integration.extract_function(sources[name], signature)
            pieces.append(piece)
            hashes[signature] = {'file': name, 'line': line,
                                 'sha256': hashlib.sha256(piece.encode()).hexdigest()}
    kalman, _ = integration.extract_block(sources['hw_devices.cpp'], 'struct Kalman1D {', 'void initToFReset()')
    prelude = integration.PRELUDE[:integration.PRELUDE.index('int i2c_write_stop_read')]
    if 'uint8_t g_tofReadyMask' not in sources['hw_devices.cpp']:
        prelude += '\n#define LEGACY_STARTUP\n'
    generated = args.output / 'boot_recovery.cpp'
    generated.write_text(prelude + HOST + kalman + '\n'.join(pieces) + CASES, encoding='utf-8')
    executable = args.output / 'boot_recovery.exe'
    command = [str(integration.CLANG), '-std=c++11', '-Wall', '-Wextra', '-Werror',
               '-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-but-set-variable', '-Wno-missing-braces',
               '-ffreestanding', '-fno-builtin', '-fno-stack-protector', '-fuse-ld=lld', '-nostdlib',
               f'-I{integration.VARIANT / "include/share"}', f'-I{integration.VARIANT / "include/software"}',
               '-ID:/pico-sdk/src/common/pico_base_headers/include', str(generated),
               '-Wl,/entry:bootRecoveryEntry', '-Wl,/subsystem:console', str(integration.KERNEL32),
               '-o', str(executable)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if compiled.returncode:
        raise SystemExit(compiled.stdout + compiled.stderr)
    results = []
    for case in range(30):
        result = subprocess.run([str(executable), f'--case={case}'], capture_output=True, text=True, timeout=10)
        count = re.search(r'PASS checks=(\d+)', result.stdout)
        passed = result.returncode == 0 and count is not None
        results.append({'case': case, 'passed': passed, 'checks': int(count[1]) if count else 0,
                        'output': result.stdout + result.stderr})
        print(f'{"PASS" if passed else "FAIL"} {case}: {result.stdout.strip()}')
    report = {'kind': 'exact startup/check/air C++ with fake peripherals and watchdog',
              'source_root': str(args.source_root), 'functions': hashes,
              'results': results, 'compile_command': command}
    (args.output / 'results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    total = sum(r['passed'] for r in results)
    print(f'Boot recovery: {total}/30 scenarios, {sum(r["checks"] for r in results)} checks')
    return 0 if total == 30 else 1


if __name__ == '__main__':
    raise SystemExit(main())
