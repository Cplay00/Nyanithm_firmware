#!/usr/bin/env python3
"""Compile exact production touch functions against fake devices and a host clock.

This is a host integration test, not a hardware measurement or a second Python
implementation of the pipeline. Every run extracts the current production C++
functions verbatim, includes the real configuration/gate headers, and launches
one fresh process per scenario so production function-local static state starts
normally. Generated source, an executable and source hashes live in _dev_tools.

Windows runner uses LLVM and kernel32 only; it does not require MSVC CRT headers.
No firmware source, device configuration, serial port or existing test is changed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


VARIANT = Path(__file__).resolve().parents[1]
WORKSPACE = VARIANT.parent
CLANG = Path(r"C:\Program Files\LLVM\bin\clang++.exe")
KERNEL32 = Path(r"C:\Program Files (x86)\Windows Kits\10\Lib\10.0.26100.0\um\x64\kernel32.lib")


def extract_function(source: str, signature: str) -> tuple[str, int]:
    """Balance C++ braces while ignoring comments and ordinary string literals."""
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    state = "code"
    i = opening
    while i < len(source):
        char = source[i]
        following = source[i:i + 2]
        if state == "line":
            if char == "\n":
                state = "code"
        elif state == "block":
            if following == "*/":
                state = "code"
                i += 1
        elif state in ('"', "'"):
            if char == "\\":
                i += 1
            elif char == state:
                state = "code"
        elif following == "//":
            state = "line"
            i += 1
        elif following == "/*":
            state = "block"
            i += 1
        elif char in ('"', "'"):
            state = char
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[start:i + 1], source.count("\n", 0, start) + 1
        i += 1
    raise ValueError(f"Unbalanced production function: {signature}")


def extract_block(source: str, first: str, after: str) -> tuple[str, int]:
    start = source.index(first)
    end = source.index(after, start)
    return source[start:end], source.count("\n", 0, start) + 1


PRELUDE = r'''
#include <stdint.h>
#include <stddef.h>
#include <mbr_distance_gate.h>
#include <pico/error.h>

extern "C" __declspec(dllimport) void* __stdcall GetStdHandle(unsigned long);
extern "C" __declspec(dllimport) int __stdcall WriteFile(void*, const void*, unsigned long, unsigned long*, void*);
extern "C" __declspec(dllimport) char* __stdcall GetCommandLineA();
extern "C" __declspec(dllimport) void __stdcall ExitProcess(unsigned int);
extern "C" void* memset(void* destination, int value, size_t count) {
    volatile unsigned char* bytes = static_cast<volatile unsigned char*>(destination);
    for (size_t i = 0; i < count; ++i) bytes[i] = static_cast<unsigned char>(value);
    return destination;
}
extern "C" void* memcpy(void* destination, const void* source, size_t count) {
    volatile unsigned char* out = static_cast<volatile unsigned char*>(destination);
    const volatile unsigned char* in = static_cast<const volatile unsigned char*>(source);
    for (size_t i = 0; i < count; ++i) out[i] = in[i];
    return destination;
}
extern "C" int memcmp(const void* left, const void* right, size_t count) {
    const unsigned char* a = static_cast<const unsigned char*>(left);
    const unsigned char* b = static_cast<const unsigned char*>(right);
    for (size_t i = 0; i < count; ++i) if (a[i] != b[i]) return a[i] < b[i] ? -1 : 1;
    return 0;
}
namespace std { using ::memset; using ::memcpy; using ::memcmp; }
static unsigned checks = 0;
static void writeText(const char* text) {
    unsigned long size = 0, written = 0;
    while (text[size]) ++size;
    WriteFile(GetStdHandle(static_cast<unsigned long>(-11)), text, size, &written, nullptr);
}
static void writeNumber(unsigned value) {
    char digits[11]; unsigned count = 0;
    do { digits[count++] = static_cast<char>('0' + value % 10); value /= 10; } while (value);
    for (unsigned i = 0; i < count / 2; ++i) {
        char tmp = digits[i]; digits[i] = digits[count - i - 1]; digits[count - i - 1] = tmp;
    }
    unsigned long written = 0;
    WriteFile(GetStdHandle(static_cast<unsigned long>(-11)), digits, count, &written, nullptr);
}
static void check(bool condition, const char* expression, unsigned line) {
    ++checks;
    if (!condition) {
        writeText("FAIL line "); writeNumber(line); writeText(": "); writeText(expression); writeText("\n");
        ExitProcess(1);
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)
static uint32_t fakeNow = 100;
static uint32_t get_absolute_time() { return fakeNow; }
static uint32_t to_ms_since_boot(uint32_t time) { return time; }

int i2c_write_stop_read(uint8_t, uint8_t, uint8_t, uint8_t*, size_t);
class CY8CMBR3116 {
public:
    uint8_t i2c_port = 1, DEVICE_I2C_ADDRESS = 0;
    uint16_t hardwareBits = 0;
    uint16_t counts[16] = {0};
    unsigned coherentReads = 0, legacyReads = 0, statusReads = 0;
    uint32_t delayMs = 0;
    bool readFails = false, statusFails = false;
    unsigned tornReadsRemaining = 0;
    unsigned genericFailuresRemaining = 0;
    uint8_t get_BUTTON_STAT(uint8_t* result) {
        ++statusReads;
        if (statusFails) return 1;
        result[0] = hardwareBits & 0xFF; result[1] = hardwareBits >> 8;
        return 0;
    }
    uint8_t get_DIFFERENCE_COUNT_SENSOR(uint16_t* result) {
        ++legacyReads;
        if (readFails) return 1;
        for (unsigned i = 0; i < 16; ++i) result[i] = counts[i];
        return 0;
    }
    int fakeReadRegisters(uint8_t address, uint8_t count, uint8_t* result) {
        CHECK(address == SYNC_COUNTER0_ADDRESS);
        CHECK(count == SYNC_COUNTER1_ADDRESS - SYNC_COUNTER0_ADDRESS + 1);
        ++coherentReads;
        fakeNow += delayMs;
        if (readFails) return PICO_ERROR_TIMEOUT;
        if (genericFailuresRemaining) { --genericFailuresRemaining; return PICO_ERROR_GENERIC; }
        result[0] = 7;
        for (unsigned i = 0; i < 16; ++i) {
            result[1 + 2 * i] = counts[i] & 0xFF;
            result[2 + 2 * i] = counts[i] >> 8;
        }
        result[count - 2] = 0;  // GPO_DATA is not a difference count.
        result[count - 1] = tornReadsRemaining ? 8 : 7;
        if (tornReadsRemaining) --tornReadsRemaining;
        return count;
    }
    uint8_t requestDataFromAddress(uint8_t address, uint8_t count, uint8_t* result);
    bool readDifferenceCounts(uint16_t result[16]);
};
class MPR121 {
public:
    uint16_t hardwareBits = 0;
    uint16_t differences[12] = {0};
    unsigned dataReads = 0;
    uint16_t touched() { return hardwareBits; }
    uint16_t filteredData(uint8_t) { ++dataReads; return 1000; }
    uint16_t baselineData(uint8_t electrode) { ++dataReads; return 1000 + differences[electrode]; }
};
controller_config ControllerConfig = {};
CY8CMBR3116 MBR3116A, MBR3116B, MBR3116C, MBR3116D, MBR3116E;
int i2c_write_stop_read(uint8_t port, uint8_t address, uint8_t reg, uint8_t* result, size_t count) {
    CHECK(port == 1);
    CY8CMBR3116* chips[] = {&MBR3116A, &MBR3116B, &MBR3116C, &MBR3116D, &MBR3116E};
    for (CY8CMBR3116* chip : chips)
        if (chip->DEVICE_I2C_ADDRESS == address) return chip->fakeReadRegisters(reg, static_cast<uint8_t>(count), result);
    CHECK(false);
    return PICO_ERROR_INVALID_ARG;
}
MPR121 mpr0, mpr1, mpr2;
#define GET_BIT(UNUM, BIT) (UNUM & (1 << BIT))
'''


HARNESS = r'''
static bool isV2() { return ControllerConfig.hwVer == 3 || ControllerConfig.hwVer == 4; }
static unsigned chipCount() { return isV2() ? 2 : 3; }
static CY8CMBR3116& chip(unsigned index) {
    if (isV2()) return index == 0 ? MBR3116D : MBR3116E;
    return index == 0 ? MBR3116A : (index == 1 ? MBR3116B : MBR3116C);
}
static unsigned laneChip(unsigned lane) { return (isV2() ? V2_LANE_M : V1_LANE_M)[lane]; }
static unsigned laneElectrode(unsigned lane) { return (isV2() ? V2_LANE_E : V1_LANE_E)[lane]; }
static void setLane(unsigned lane, uint16_t difference, bool hardware = true) {
    CY8CMBR3116& device = chip(laneChip(lane));
    unsigned electrode = laneElectrode(lane);
    device.counts[electrode] = difference;
    if (hardware) device.hardwareBits |= 1u << electrode;
    else device.hardwareBits &= ~(1u << electrode);
}
static void clearDevices() {
    for (unsigned m = 0; m < chipCount(); ++m) {
        chip(m).hardwareBits = 0;
        for (unsigned e = 0; e < 16; ++e) chip(m).counts[e] = 0;
    }
}
static void profile(bool enabled = true) {
    ControllerConfig.mbrDistanceMagic = MBR_DISTANCE_PROFILE_MAGIC;
    ControllerConfig.mbrDistanceVersion = MBR_DISTANCE_PROFILE_VERSION;
    ControllerConfig.mbrDistanceFlags = enabled ? MBR_DISTANCE_FLAG_ENABLED : 0;
    ControllerConfig.mbrDistanceZero = 100;
    ControllerConfig.mbrDistanceFull = 200;
    ControllerConfig.mbrDistanceOnPercent = 80;
    ControllerConfig.mbrDistanceOffPercent = 50;
    ControllerConfig.mbrDistanceCheck = mbrDistanceProfileCheck(ControllerConfig);
}
static void initialize(bool v2, bool enabled = true) {
    CY8CMBR3116* devices[] = {&MBR3116A, &MBR3116B, &MBR3116C, &MBR3116D, &MBR3116E};
    for (unsigned i = 0; i < 5; ++i) {
        devices[i]->i2c_port = 1;
        devices[i]->DEVICE_I2C_ADDRESS = 0x40 + i;
    }
    ControllerConfig.hwVer = v2 ? 3 : 1;
    ControllerConfig.cfg0 = CFG0_BIT_MBR3116;
    ControllerConfig.th_touch = 6;
    ControllerConfig.th_release = 3;
    ControllerConfig.mbrTouchGate = 130;
    if (enabled) profile();
    buildLaneTable();
}
static void frame(uint32_t elapsed = 5) {
    fakeNow += elapsed;
    prepareMbrDistanceGate();
    if (isV2()) updateTouch_v2(); else updateTouch_v1();
}
static void output(unsigned lane, bool active) { CHECK((touchData32[lane] != 0) == active); }
static unsigned coherentReads() {
    unsigned count = 0;
    for (unsigned m = 0; m < chipCount(); ++m) count += chip(m).coherentReads;
    return count;
}
static unsigned legacyReads() {
    unsigned count = 0;
    for (unsigned m = 0; m < chipCount(); ++m) count += chip(m).legacyReads;
    return count;
}
static void held(unsigned lane = 0) {
    setLane(lane, 250);
    for (unsigned i = 0; i < 20; ++i) { frame(); output(lane, true); }
}
static void reConfirm(unsigned lane = 0) {
    setLane(lane, 190);
    frame(); output(lane, false);
    frame(); output(lane, false);
    frame(); output(lane, true);
}
static void legacySequence() {
    ControllerConfig.mbrTouchGate = 220;
    held();
    setLane(0, 120); frame(); output(0, true);  // Historic held-touch bypass remains.
    setLane(1, 150);
    for (unsigned i = 0; i < 4; ++i) frame();
    output(1, true);  // Historic confirmed game-lane neighbour relaxation remains.
    CHECK(coherentReads() == 0);
    CHECK(legacyReads() > 0);
    CHECK(g_mbrDistanceReadFailures == 0);
}
static void runScenario(unsigned scenario, bool v2) {
    initialize(v2, scenario != 9);
    switch (scenario) {
    case 0:  // Sustained strict OFF defeats sticky, dip, stretch and verification retention.
        held();
        setLane(0, 150); frame(); output(0, false);
        setLane(0, 170); frame(); output(0, false);
        reConfirm();
        CHECK(legacyReads() == 0);
        break;
    case 1:  // A confirmed neighbour cannot relax the strict global ON boundary.
        ControllerConfig.mbrTouchGate = 220;
        held(); setLane(1, 170);
        for (unsigned i = 0; i < 4; ++i) { frame(); output(0, true); output(1, false); }
        setLane(0, 150); frame(); output(0, false); output(1, false);
        setLane(0, 170); frame(); output(0, false); output(1, false);
        break;
    case 2:  // Native OFF clears immediately even while a high count is cached.
        held(); setLane(0, 250, false); frame(1); output(0, false);
        reConfirm();
        break;
    case 3: {  // Failed gate reads count once; cache/profile changes preserve the total.
        held(); chip(laneChip(0)).readFails = true;
        frame(); output(0, false); CHECK(!mbrDistanceSamples[laneChip(0)].valid);
        CHECK(g_mbrDistanceReadFailures == 1);
        frame(1); output(0, false); CHECK(g_mbrDistanceReadFailures == 1);
        frame(4); output(0, false); CHECK(g_mbrDistanceReadFailures == 2);
        profile(false); frame(1); CHECK(g_mbrDistanceReadFailures == 2);
        unsigned legacyBefore = legacyReads();
        profile(); frame(1); CHECK(g_mbrDistanceReadFailures == 3);
        chip(laneChip(0)).readFails = false;
        reConfirm(); CHECK(legacyReads() == legacyBefore);
        CHECK(g_mbrDistanceReadFailures == 3);
        chip(laneChip(0)).genericFailuresRemaining = 2;
        frame(); output(0, true); CHECK(g_mbrDistanceReadFailures == 3);
        chip(laneChip(0)).genericFailuresRemaining = 3;
        frame(); output(0, false); CHECK(g_mbrDistanceReadFailures == 4);
        g_mbrDistanceReadFailures = 0xFFFFFFFFu;
        chip(laneChip(0)).readFails = true;
        frame(); CHECK(g_mbrDistanceReadFailures == 0);
        break;
    }
    case 4: {  // The 5ms cache is used; a due read applies the low count immediately.
        setLane(0, 250); frame(0); output(0, true);
        unsigned readCount = coherentReads();
        setLane(0, 150); frame(4); output(0, true); CHECK(coherentReads() == readCount);
        frame(1); output(0, false); CHECK(coherentReads() == readCount + chipCount());
        CHECK(legacyReads() == 0);
        break;
    }
    case 5: {  // Host-read age is inclusive at 15ms, rejected at 16ms.
        setLane(0, 250); prepareMbrDistanceGate();
        unsigned m = laneChip(0), e = laneElectrode(0);
        mbrDistanceReadSample(m, &chip(m));
        uint32_t start = mbrDistanceSamples[m].readStartedMs;
        CHECK(mbrDistanceAllowedMask(m, 1u << e, start + 15) == (1u << e));
        CHECK(mbrDistanceAllowedMask(m, 1u << e, start + 16) == 0);
        CHECK(!mbrDistanceStates[m][e].active);
        CHECK(mbrDistanceAllowedMask(m, 1u << e, start + 15) == (1u << e));
        break;
    }
    case 6: {  // Later chip blocks for 17ms: an earlier sample must not be authorized.
        unsigned lane = v2 ? 16 : 30;
        CHECK(laneChip(lane) == 0);
        setLane(lane, 250); chip(1).delayMs = 17;
        frame(); output(lane, false);
        CHECK(fakeNow - mbrDistanceSamples[0].readStartedMs == 17);
        CHECK(hwTouch[0] != 0); CHECK(rawTouch[0] == 0);
        chip(1).delayMs = 0;
        frame(); output(lane, true);
        break;
    }
    case 7: {  // Changed profile cannot inherit an active state in its new hysteresis band.
        held(); unsigned readsBefore = coherentReads();
        ControllerConfig.mbrDistanceOnPercent = 95;  // New OFF=150, ON=195.
        ControllerConfig.mbrDistanceCheck = mbrDistanceProfileCheck(ControllerConfig);
        setLane(0, 190); frame(1); output(0, false);
        CHECK(mbrDistanceResetForFrame);
        CHECK(coherentReads() == readsBefore + chipCount());
        frame(); output(0, false);
        setLane(0, 250); frame(); output(0, true);
        break;
    }
    case 8:  // Layout version changes clear cached samples and all downstream held states.
        held();
        ControllerConfig.hwVer = v2 ? 4 : 2;
        buildLaneTable(); setLane(0, 190);
        frame(1); CHECK(mbrDistanceResetForFrame); output(0, false);
        frame(); output(0, false); frame(); output(0, true);
        // Then change physical layout too, as normal hardware re-initialization does.
        clearDevices(); ControllerConfig.hwVer = v2 ? 1 : 3; buildLaneTable();
        clearDevices(); setLane(0, 190);
        frame(); CHECK(mbrDistanceResetForFrame); output(0, false);
        frame(); output(0, false); frame(); output(0, true);
        break;
    case 9:  // Legacy zero-reserved configuration, including a stray enable bit.
        CHECK(!mbrDistanceGateEnabled(ControllerConfig));
        ControllerConfig.mbrDistanceFlags = MBR_DISTANCE_FLAG_ENABLED;
        CHECK(!mbrDistanceGateEnabled(ControllerConfig));
        legacySequence();
        break;
    case 10:  // Signed but deliberately disabled profile follows the legacy pipeline.
        profile(false); CHECK(mbrDistanceProfileValid(ControllerConfig));
        legacySequence();
        break;
    case 11:  // Real forward and inverse tables agree for all 32 output lanes.
        for (unsigned lane = 0; lane < 32; ++lane) {
            clearDevices(); frame();
            setLane(lane, 250); frame();
            for (unsigned other = 0; other < 32; ++other) output(other, other == lane);
            CHECK(pressureSnap[lane] == 250);
            CHECK(laneTable[laneChip(lane)][laneElectrode(lane)] == static_cast<int>(lane));
            setLane(lane, 0); frame(); output(lane, false);
        }
        if (!v2) {
            clearDevices(); chip(2).hardwareBits = 0xFF00;
            for (unsigned e = 8; e < 16; ++e) chip(2).counts[e] = 250;
            frame();
            for (unsigned lane = 0; lane < 32; ++lane) output(lane, false);
            CHECK(mbrDistanceAllowedMask(2, 0xFF00, fakeNow) == 0);
        }
        CHECK(legacyReads() == 0);
        break;
    case 12: {  // Exercise the actual driver SYNC check and its bounded retry.
        held(); CY8CMBR3116& device = chip(laneChip(0));
        unsigned previousReads = device.coherentReads;
        device.tornReadsRemaining = 2;
        frame(); output(0, false); CHECK(device.coherentReads == previousReads + 2);
        CHECK(g_mbrDistanceReadFailures == 1);
        previousReads = device.coherentReads; device.tornReadsRemaining = 1;
        frame(); output(0, true); CHECK(device.coherentReads == previousReads + 2);
        CHECK(g_mbrDistanceReadFailures == 1);
        CHECK(device.tornReadsRemaining == 0);
        CHECK(!device.readDifferenceCounts(nullptr));
        break;
    }
    case 13:  // A non-button-scale count above 255 is rejected, although display clamps.
        held(); setLane(0, 256); frame(); output(0, false);
        CHECK(pressureSnap[0] == 255);
        reConfirm();
        break;
    case 14:  // BUTTON_STAT transport failure is native OFF despite a valid count sample.
        held(); chip(laneChip(0)).statusFails = true;
        frame(1); output(0, false); CHECK(hwTouch[laneChip(0)] == 0);
        chip(laneChip(0)).statusFails = false;
        reConfirm();
        break;
    case 15:  // Disable/re-enable does not inherit verification, sticky or neighbour state.
        held(); ControllerConfig.mbrTouchGate = 220;
        profile(false); setLane(0, 120); frame(1);
        CHECK(mbrDistanceResetForFrame); output(0, false);
        ControllerConfig.mbrTouchGate = 130; profile();
        reConfirm();
        break;
    default: CHECK(false);
    }
    if (scenario != 3 && scenario != 12) CHECK(g_mbrDistanceReadFailures == 0);
}
static void runMpr(bool withProfile) {
    initialize(false, withProfile); ControllerConfig.cfg0 = 0;
    MPR121* mprs[3] = {&mpr0, &mpr1, &mpr2};
    unsigned m = V1_LANE_M[0], e = V1_LANE_E[0];
    mprs[m]->differences[e] = 60; mprs[m]->hardwareBits = 1u << e;
    for (unsigned i = 0; i < 20; ++i) { frame(); output(0, true); }
    CHECK(!mbrDistanceEnabledForFrame);
    CHECK(coherentReads() == 0); CHECK(legacyReads() == 0);
    CHECK(g_mbrDistanceReadFailures == 0);
    mprs[m]->differences[e] = 1; frame(); output(0, true);  // Existing sustained verification.
    mprs[m]->hardwareBits = 0; frame(1); output(0, true);  // Existing sticky release.
    CHECK(mprs[m]->dataReads == 2);
}
extern "C" void mbrPipelineTestEntry() {
    const char* command = GetCommandLineA();
    unsigned number = 999;
    for (unsigned i = 0; command[i]; ++i) {
        if (command[i] == '=' && i >= 6 && command[i - 1] == 'e' && command[i - 2] == 's') {
            number = 0;
            while (command[++i] >= '0' && command[i] <= '9') number = number * 10 + command[i] - '0';
            break;
        }
    }
    if (number < 32) runScenario(number / 2, (number & 1) != 0);
    else if (number < 34) runMpr(number == 33);
    else CHECK(false);
    writeText("PASS checks="); writeNumber(checks); writeText("\n");
    ExitProcess(0);
}
'''

SCENARIOS = [
    "sustained-OFF-clears-all-retention", "neighbour-cannot-relax-strict-ON",
    "native-OFF-with-cached-high-count", "count-I2C-failure-and-reconfirmation",
    "5ms-cache-and-due-refresh", "15ms-inclusive-host-age", "later-chip-17ms-stall",
    "profile-change-in-new-hysteresis-band", "layout-change-clears-held-state",
    "legacy-zero-profile-and-stray-enable", "valid-disabled-profile-legacy",
    "all-32-lane-mappings-and-unmapped-bits", "actual-driver-SYNC-bounded-retry",
    "above-255-count-fails-closed", "BUTTON_STAT-failure", "disable-enable-clears-retention",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang", type=Path, default=CLANG)
    parser.add_argument("--kernel32", type=Path, default=KERNEL32)
    parser.add_argument("--sdk", type=Path, default=Path(r"D:\pico-sdk"))
    parser.add_argument("--output", type=Path, default=WORKSPACE / "_dev_tools" / "mbr_distance_pipeline_host")
    parser.add_argument("--case", type=int, help="Run just one numbered scenario (0..33)")
    args = parser.parse_args()
    if sys.platform != "win32":
        parser.error("This runner currently targets the installed Windows LLVM/SDK toolchain.")
    if args.case is not None and not 0 <= args.case < 34:
        parser.error("--case must be 0..33")
    for dependency in (args.clang, args.kernel32):
        if not dependency.is_file():
            parser.error(f"Missing compiler dependency: {dependency}")
    source_path = VARIANT / "src" / "hw_devices.cpp"
    driver_path = VARIANT / "src" / "cy8cmbr3116.cpp"
    sdk_headers = args.sdk / "src/common/pico_base_headers/include"
    if not (sdk_headers / "pico/error.h").is_file():
        parser.error("Official Pico SDK error header is required")
    source = source_path.read_text(encoding="utf-8")
    driver = driver_path.read_text(encoding="utf-8")
    device_header = (VARIANT / "include" / "device" / "cy8cmbr3116.h").read_text(encoding="utf-8")
    definitions = "\n".join(re.search(rf"^#define {name}\s+[^\n]+", device_header, re.M).group(0)
                            for name in ("SYNC_COUNTER0_ADDRESS", "SYNC_COUNTER1_ADDRESS"))
    parts: list[tuple[Path, str, tuple[str, int]]] = [
        (driver_path, "requestDataFromAddress", extract_function(driver, "uint8_t CY8CMBR3116::requestDataFromAddress(")),
        (driver_path, "readDifferenceCounts", extract_function(driver, "bool CY8CMBR3116::readDifferenceCounts(")),
        (source_path, "lane-tables-and-helpers", extract_block(source, "static const uint8_t V1_LANE_M", "void initMPR121()")),
        (source_path, "touch-globals", extract_block(source, "uint8_t touchData[4];", "// round66: real-time pressure")),
        (source_path, "distance-globals", extract_block(source, "static const uint32_t MBR_DISTANCE_POLL_MS", "static void prepareMbrDistanceGate()")),
    ]
    parts.extend((source_path, name, extract_function(source, signature)) for name, signature in [
        ("prepareMbrDistanceGate", "static void prepareMbrDistanceGate()"),
        ("mbrDistanceReadSample", "static void mbrDistanceReadSample("),
        ("mbrDistanceAllowedMask", "static uint16_t mbrDistanceAllowedMask("),
        ("updateTouch_v2", "void updateTouch_v2()"),
        ("updateTouch_v1", "void updateTouch_v1()"),
    ])
    chunks = ['#include <touch_snapshot.h>', definitions, PRELUDE]
    manifest = {"kind": "production-source host integration; fake devices, not real hardware", "fragments": []}
    for path, name, (code, line) in parts:
        chunks.append(f'\n#line {line} "{path.as_posix()}"\n{code}\n')
        manifest["fragments"].append({"name": name, "path": str(path), "line": line,
                                      "sha256": hashlib.sha256(code.encode()).hexdigest()})
        if name == "touch-globals":
            chunks.append("\nuint8_t pressureSnap[32] = {0};\n")
    chunks.extend(['\n#line 1 "host_pipeline_scenarios.cpp"\n', HARNESS])
    args.output.mkdir(parents=True, exist_ok=True)
    generated = args.output / "test_mbr_distance_pipeline_generated.cpp"
    executable = args.output / "test_mbr_distance_pipeline.exe"
    generated.write_text("\n".join(chunks), encoding="utf-8", newline="\n")
    command = [str(args.clang), "-std=c++11", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
               "-Wno-missing-braces",  # Preserve production's legacy array initializers verbatim.
               "-ffreestanding", "-fno-builtin", "-fno-stack-protector", "-fuse-ld=lld", "-nostdlib",
               f"-I{VARIANT / 'include' / 'share'}", f"-I{VARIANT / 'include' / 'software'}",
               f"-I{sdk_headers}",
               str(generated), "-Wl,/entry:mbrPipelineTestEntry", "-Wl,/subsystem:console",
               str(args.kernel32), "-o", str(executable)]
    manifest["compile_command"] = command
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if compiled.returncode:
        print(compiled.stdout + compiled.stderr, end="")
        return compiled.returncode
    results = []
    numbers = [args.case] if args.case is not None else range(34)
    for number in numbers:
        name = (("v2" if number & 1 else "v1") + ":" + SCENARIOS[number // 2]
                if number < 32 else "MPR:" + ("signed-enabled-profile-ignored" if number == 33 else "legacy-zero-profile"))
        result = subprocess.run([str(executable), f"--case={number}"], capture_output=True, text=True, timeout=15)
        match = re.search(r"PASS checks=(\d+)", result.stdout)
        passed = result.returncode == 0 and match is not None
        results.append({"number": number, "name": name, "passed": passed,
                        "checks": int(match.group(1)) if match else 0, "output": result.stdout + result.stderr})
        print(f"{'PASS' if passed else 'FAIL'} {number:02d} {name}: {result.stdout.strip()}")
    manifest["results"] = results
    manifest["source_sha256"] = hashlib.sha256(source.encode()).hexdigest()
    manifest["driver_sha256"] = hashlib.sha256(driver.encode()).hexdigest()
    manifest["header_sha256"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in (VARIANT / "include" / "share" / "nyanithm_shared.h",
                                               VARIANT / "include" / "software" / "mbr_distance_gate.h")}
    manifest_path = args.output / "results.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    passed_count = sum(result["passed"] for result in results)
    print(f"Production integration: {passed_count}/{len(results)} scenarios, "
          f"{sum(result['checks'] for result in results)} checks. Fake clock/devices; no real hardware.")
    print(f"Evidence: {manifest_path}")
    return 0 if passed_count == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
