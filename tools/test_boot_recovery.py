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
static uint8_t g_tofReadyMask;
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
static uint8_t mprPresent = 7, mbrPresent = 7;
static bool muxPresent = true, irPresent, addressFailure;
static uint32_t forceDuration = 20, calibrationDuration = 30;
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
struct FakeLed {
    void fill(unsigned r, unsigned g, unsigned) {
        if (r == 250 && g == 150) ++warningCycles;
        if (r == 250 && g == 250) ++errorCycles;
    }
    void flush() {}
};
static FakeLed RGB_LED;
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
        *range = 100; return true;
    }
};
static VL53L0X tof0(0),tof1(1),tof2(2),tof3(3),tof4(4);
class MPR121 {
public:
    bool init(unsigned,unsigned,bool) { ++mprInitCalls; advance(10); return true; }
    void writeRegister(unsigned,unsigned) { advance(1); }
    void setThresholdsForElectrode(unsigned,unsigned,unsigned) { advance(1); }
    void setDebounce(unsigned,unsigned) { advance(1); }
    void calibrateBaseline() { ++mprCalibrationCalls; advance(calibrationDuration); }
};
static MPR121 mpr0,mpr1,mpr2;
static uint8_t electrodeBaseTouchTh(unsigned,unsigned) { return 0; }
static uint8_t electrodeBaseReleaseTh(unsigned,unsigned) { return 0; }
static void buildLaneTable() {}
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
'''

CASES = r'''
extern "C" void bootRecoveryEntry() {
    const char* command = GetCommandLineA(); unsigned number = 99;
    for (unsigned i = 0; command[i]; ++i) if (command[i] == '=') {
        number = 0;
        while (command[++i] >= '0' && command[i] <= '9') number = number * 10 + command[i] - '0';
        break;
    }
    CHECK(number < 17);
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
    if (number == 8) { ControllerConfig.hwVer = 3; ControllerConfig.cfg0 = 0; mbrPresent = 3; }
    if (number == 9) { ControllerConfig.hwVer = 4; ControllerConfig.cfg0 = 0; mbrPresent = 3; presentMask = 31; }
    if (number == 10) { mbrPresent = 0; }
    if (number == 11) { ControllerConfig.cfg0 = 0; calibrationDuration = 750; }
    if (number == 12) { presentMask = 0; }
    if (number == 13) { ControllerConfig.hwVer = 2; presentMask = 31; mprPresent = 0; } // 32-inch MBR
    if (number == 14) { ControllerConfig.cfg0 = 0; mbrPresent = 0; } // 27-inch MPR
    if (number == 15) { ControllerConfig.hwVer = 2; ControllerConfig.cfg0 = 0; presentMask = 31; mprPresent = 0; } // wrong MPR selection on 32-inch MBR
    if (number == 16) { mbrPresent = 0; } // wrong MBR selection on 27-inch MPR
    initHwDevices();
    checkHardwareState();
    CHECK(wdtDelay == 2000 && maxFeedGap < 2000);
    CHECK(mbrInitCalls == ((ControllerConfig.cfg0 & CFG0_BIT_MBR3116) || ControllerConfig.hwVer >= 3 ? 1u : 0u));
    CHECK(mprInitCalls == (mbrInitCalls ? 0u : 3u));
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
    if (number == 3) CHECK(errorCycles == 2);
    if (number == 2 || number == 4 || number == 6 || number == 10 || number == 12 || number == 15 || number == 16) CHECK(warningCycles == 2);
    if (number == 0 || number == 1 || number == 5 || number == 7 || number == 8 || number == 9 || number == 11 || number == 13 || number == 14)
        CHECK(warningCycles == 0 && errorCycles == 0);
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
        CHECK(polls[i] == ((g_tofReadyMask & (1u << i)) && !usingIR ? 1000u : 0u));
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
               for name in ('hw_devices.cpp', 'hw_check.cpp', 'error_state.cpp')}
    pieces = []
    hashes = {}
    for name, signatures in (
        ('error_state.cpp', ['void warn(', 'void error(']),
        ('hw_devices.cpp', ['void initToFReset()', 'void resetToF()', 'void initToF()',
                            'void initMPR121()', 'void initHwDevices()', 'void updateAir()']),
        ('hw_check.cpp', ['void checkToF()', 'void check3116()', 'void checkMPR121()',
                          'void checkHardwareState()']),
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
    for case in range(17):
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
    print(f'Boot recovery: {total}/17 scenarios, {sum(r["checks"] for r in results)} checks')
    return 0 if total == 17 else 1


if __name__ == '__main__':
    raise SystemExit(main())
