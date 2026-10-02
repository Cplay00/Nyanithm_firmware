#!/usr/bin/env python3
"""Compile the production touch snapshot readers with deterministic host fakes.

The complete HID function and GET_INPUT / DEBUG_CHAIN / TELEMETRY bodies are
extracted from src/chuni_io.cpp, including their retry, timeout, pressure,
telemetry, and USB write paths. No reader implementation is copied here.
The fake clock and generation reads inject writer interleavings; no device,
serial port, Pico SDK source, or firmware configuration is changed.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def extract_body(source, marker):
    """Return a marked C++ block while ignoring braces in comments/literals."""
    if source.count(marker) != 1:
        raise ValueError(f"Expected one production marker: {marker}")
    opening = source.index("{", source.index(marker))
    depth = 0
    state = "code"
    index = opening
    while index < len(source):
        char = source[index]
        following = source[index + 1:index + 2]
        if state == "line_comment":
            if char == "\n":
                state = "code"
        elif state == "block_comment":
            if char == "*" and following == "/":
                state = "code"
                index += 1
        elif state in ("string", "character"):
            if char == "\\":
                index += 1
            elif char == ('"' if state == "string" else "'"):
                state = "code"
        elif char == "/" and following == "/":
            state = "line_comment"
            index += 1
        elif char == "/" and following == "*":
            state = "block_comment"
            index += 1
        elif char == '"':
            state = "string"
        elif char == "'":
            state = "character"
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
        index += 1
    raise ValueError(f"Unterminated production block: {marker}")


HOST = r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <vector>
#include <nyanithm_shared.h>
#include <touch_snapshot.h>

controller_config ControllerConfig{};
bool hid_working = true;
uint8_t touchData[4]{};
uint8_t touchData32[32]{};
uint8_t pressureSnap[32]{};
bool airKeys[6]{};
uint16_t hwTouch[3]{};
uint16_t rawTouch[3]{};
TouchInputSnapshot publishedTouchState{};
MbrTouchTrace publishedMbrTrace{};
void recordMbrHistory(const MbrTouchTrace&) {}
MbrTouchTrace buildMbrTouchTrace() {
    MbrTouchTrace trace{};
    trace.tag = CMD_MBR_TOUCH_TRACE;
    trace.version = MBR_TRACE_VERSION;
    trace.frameId = 1234;
    for (unsigned i = 0; i < 32; ++i) trace.counts[i] = 0x1234 + i;
    std::memcpy(trace.slider, touchData32, 32);
    std::memcpy(trace.hardware, hwTouch, 6);
    std::memcpy(trace.verified, rawTouch, 6);
    return trace;
}
uint8_t rawReportLevel = 0;
bool gameRawEnabled = false;
volatile uint32_t g_mbrDistanceReadFailures = 0;
uint32_t g_loopMinUs = 0, g_loopMaxUs = 0, g_loopSumUs = 0, g_loopCount = 0;
uint8_t g_verifyFail[36]{};

// Reads occur at the actual reader's sequence checks. A hook can complete a
// writer between the opening sequence read and the closing sequence read.
struct FakeGeneration {
    uint32_t value = 2;
    unsigned int reads = 0;
    std::function<void(unsigned int)> onRead;
    FakeGeneration& operator++() { ++value; return *this; }
    operator uint32_t() {
        assert(++reads < 10000);
        if (onRead) onRead(reads);
        return value;
    }
} touchStateGen;

std::function<void()> onBarrier;
void __dmb() { if (onBarrier) onBarrier(); }

uint64_t clockUs = 100000;
unsigned int clockCalls = 0;
std::function<void()> onClock;
uint64_t get_absolute_time() {
    assert(++clockCalls < 200);
    clockUs += 250;
    if (onClock) onClock();
    return clockUs;
}
uint32_t to_ms_since_boot(uint64_t time) {
    return static_cast<uint32_t>(time / 1000);
}

std::vector<uint8_t> cdcBytes;
std::vector<uint8_t> hidBytes;
unsigned int hidReports = 0;
uint32_t cdcWriteLimit = 0xFFFFFFFFu;
uint32_t cdcAvailable = 256;
#define CFG_TUD_CDC_TX_BUFSIZE 256
uint32_t tud_cdc_write_available() { return cdcAvailable; }
std::function<void()> onCdcWrite;
uint32_t tud_cdc_write(const void* data, uint32_t length) {
    if (onCdcWrite) onCdcWrite();
    length = std::min(length, cdcWriteLimit);
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    cdcBytes.insert(cdcBytes.end(), bytes, bytes + length);
    return length;
}
uint32_t tud_cdc_write_flush() { return 0; }
void tud_task() {}
void sleep_us(uint32_t duration) { clockUs += duration; }
bool tud_suspended() { return false; }
bool tud_remote_wakeup() { return true; }
bool tud_hid_n_ready(uint8_t) { return true; }
bool tud_hid_n_report(uint8_t, uint8_t, const void* data, uint16_t length) {
    ++hidReports;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    hidBytes.assign(bytes, bytes + length);
    return true;
}
enum { BUTTON_UP, BUTTON_DOWN, BUTTON_PUSH };
bool getButtonState(int) { return false; }
"""


CASES = r"""
unsigned int passed = 0;
void pass(const char* name) {
    ++passed;
    std::cout << "PASS " << name << '\n';
}

void paintShared(uint8_t seed) {
    for (unsigned int lane = 0; lane < 32; ++lane) {
        touchData32[lane] = (lane + seed) % 3 == 0 ? 128 : 0;
        pressureSnap[lane] = static_cast<uint8_t>(20 + seed + lane);
    }
    for (unsigned int byte = 0; byte < 4; ++byte)
        touchData[byte] = static_cast<uint8_t>(0x21 + seed * 4 + byte);
    for (unsigned int key = 0; key < 6; ++key)
        airKeys[key] = (key + seed) % 2 == 0;
    for (unsigned int chip = 0; chip < 3; ++chip) {
        hwTouch[chip] = static_cast<uint16_t>(0x1000 + seed * 0x100 + chip);
        rawTouch[chip] = static_cast<uint16_t>(0x2000 + seed * 0x100 + chip);
    }
}

uint8_t expectedAir() {
    return publishedTouchState.air;
}

void beginCase() {
    static uint64_t nextStart = 100000;
    nextStart += 100000;
    clockUs = nextStart;
    clockCalls = 0;
    onClock = {};
    onBarrier = {};
    touchStateGen.value = 2;
    touchStateGen.reads = 0;
    touchStateGen.onRead = {};
    cdcBytes.clear();
    hidBytes.clear();
    hidReports = 0;
    ControllerConfig = {};
    ControllerConfig.hwVer = 1;
    ControllerConfig.cfg0 = CFG0_BIT_MBR3116;
    ControllerConfig.cfg1 = CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD |
                            CFG1_BIT_ENABLE_AIR_INPUT_AS_KEYBOARD;
    rawReportLevel = 0;
    gameRawEnabled = false;
    inputState = {};
    prevInputState = {};
    prevAirState = 0;
    latencyFrameMs = 0;
    g_tele = {};
    g_mbrDistanceReadFailures = 0;
    g_loopMinUs = g_loopMaxUs = g_loopSumUs = g_loopCount = 0;
    std::memset(g_verifyFail, 0, sizeof(g_verifyFail));
    cdcWriteLimit = 0xFFFFFFFFu;
    cdcAvailable = 256;
    onCdcWrite = {};
    paintShared(1);
    publishTouchState();
}

std::vector<uint8_t> currentInputBytes() {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&inputState);
    return {bytes, bytes + sizeof(inputState)};
}

void assertInput(bool pressure = false) {
    assert(cdcBytes.size() == 33);
    for (unsigned int lane = 0; lane < 32; ++lane) {
        uint8_t expected = pressure && publishedTouchState.slider[lane]
                               ? publishedTouchState.pressure[lane] : publishedTouchState.slider[lane];
        assert(cdcBytes[lane] == expected);
    }
    assert(cdcBytes[32] == expectedAir());
    assert(g_tele.servedCount == 1);
    assert(g_tele.cdcTxBytes == 33);
}

void assertHid() {
    assert(hidReports == 1 && hidBytes.size() == 15);
    for (unsigned int byte = 0; byte < 4; ++byte)
        assert(hidBytes[byte + 2] == publishedTouchState.keys[byte]);
    assert(hidBytes[9] == expectedAir());
}

std::vector<uint8_t> expectedChain() {
    std::vector<uint8_t> bytes{0xAA, 0x55};
    for (uint16_t word : publishedTouchState.hardware) {
        bytes.push_back(static_cast<uint8_t>(word));
        bytes.push_back(static_cast<uint8_t>(word >> 8));
    }
    for (uint16_t word : publishedTouchState.verified) {
        bytes.push_back(static_cast<uint8_t>(word));
        bytes.push_back(static_cast<uint8_t>(word >> 8));
    }
    bytes.insert(bytes.end(), publishedTouchState.slider, publishedTouchState.slider + 32);
    return bytes;
}

void injectNewGeneration() {
    touchStateGen.onRead = [](unsigned int reads) {
        if (reads == 2) {
            paintShared(7);
            publishTouchState();
        }
    };
}

int main() {
    static_assert(sizeof(NyanithmInput) == 33, "GET_INPUT wire size changed");

    // This must precede every valid DEBUG_CHAIN read: no last-good frame yet.
    beginCase();
    touchStateGen.value = 3;
    uint64_t started = clockUs;
    readDebugChain();
    assert(cdcBytes.size() == 46 && cdcBytes[0] == 0xAA && cdcBytes[1] == 0x55);
    assert(std::all_of(cdcBytes.begin() + 2, cdcBytes.end(), [](uint8_t v) { return v == 0; }));
    assert(clockUs - started >= 5000 && clockUs - started < 7000);
    assert(g_tele.cdcTxBytes == 46);
    pass("DEBUG_CHAIN initial odd timeout sends zero payload, fixed 46 bytes");

    beginCase();
    readDebugChain();
    assert(cdcBytes == expectedChain());
    std::vector<uint8_t> lastChain = cdcBytes;
    pass("DEBUG_CHAIN even coherent frame preserves native wire layout");

    beginCase();
    touchStateGen.value = 3;
    paintShared(9);
    readDebugChain();
    assert(cdcBytes == lastChain && cdcBytes.size() == 46);
    pass("DEBUG_CHAIN odd timeout reuses whole last-good frame");

    beginCase();
    injectNewGeneration();
    readDebugChain();
    assert(touchStateGen.reads >= 4 && cdcBytes == expectedChain());
    pass("DEBUG_CHAIN changed generation discards first copy and retries");

    beginCase();
    readGetInput();
    assertInput();
    pass("GET_INPUT even frame commits touch and air, native 33 bytes");

    beginCase();
    std::memset(inputState.slider, 17, sizeof(inputState.slider));
    inputState.air = 0x15;
    std::vector<uint8_t> lastInput = currentInputBytes();
    touchStateGen.value = 3;
    started = clockUs;
    readGetInput();
    assert(cdcBytes == lastInput);
    assert(clockUs - started >= 1000 && clockUs - started < 3000);
    assert(prevAirState == 0x15);
    pass("GET_INPUT stable odd timeout retains all last-good input bytes");

    beginCase();
    std::memset(inputState.slider, 43, sizeof(inputState.slider));
    inputState.air = 0x03;
    lastInput = currentInputBytes();
    rawReportLevel = 2;
    gameRawEnabled = true;
    touchStateGen.value = 3;
    readGetInput();
    assert(cdcBytes == lastInput);
    pass("GET_INPUT odd timeout cannot replace last-good with raw pressure");

    beginCase();
    injectNewGeneration();
    readGetInput();
    assert(touchStateGen.reads >= 4);
    assertInput();
    pass("GET_INPUT changed generation retries all touch, pressure and air");

    beginCase();
    gameRawEnabled = true;
    injectNewGeneration();
    readGetInput();
    assert(touchStateGen.reads >= 4);
    assertInput(true);
    pass("GET_INPUT retry pressure stays paired with the accepted touch frame");

    beginCase();
    touchStateGen.value = 3;
    uint64_t completesAt = clockUs + 500;
    onClock = [completesAt]() {
        if (clockUs >= completesAt) {
            paintShared(7);
            touchStateGen.value = 2;
            publishTouchState();
            onClock = {};
        }
    };
    readGetInput();
    assertInput();
    pass("GET_INPUT writer finishes within deadline, then accepts even frame");

    beginCase();
    touchStateGen.value = 0xFFFFFFFEu;
    injectNewGeneration();
    readGetInput();
    assert(touchStateGen.value == 0 && touchStateGen.reads >= 4);
    assertInput();
    pass("GET_INPUT generation rollover still rejects changed first copy");

    beginCase();
    clockUs = static_cast<uint64_t>(0xFFFFFFFFu) * 1000 + 250;
    touchStateGen.value = 3;
    std::memset(inputState.slider, 11, sizeof(inputState.slider));
    inputState.air = 0x12;
    lastInput = currentInputBytes();
    readGetInput();
    assert(cdcBytes == lastInput && clockCalls < 20);
    pass("GET_INPUT timeout remains bounded across millisecond rollover");

    beginCase();
    hid_task_chuni_input();
    assertHid();
    pass("HID even generation publishes coherent 15-byte keyboard frame");

    beginCase();
    ControllerConfig.cfg1 = CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD;
    touchStateGen.value = 3;
    started = clockUs;
    hid_task_chuni_input();
    assert(hidReports == 0 && hidBytes.empty());
    assert(clockUs - started >= 5000 && clockUs - started < 8000);
    pass("HID touch stable odd timeout skips report entirely");

    beginCase();
    ControllerConfig.cfg1 = CFG1_BIT_ENABLE_AIR_INPUT_AS_KEYBOARD;
    touchStateGen.value = 3;
    hid_task_chuni_input();
    assert(hidReports == 0 && hidBytes.empty());
    pass("HID air stable odd timeout skips report entirely");

    beginCase();
    injectNewGeneration();
    hid_task_chuni_input();
    assert(touchStateGen.reads >= 6);
    assertHid();
    pass("HID changed generation retries before USB report");

    beginCase();
    readTelemetry();
    assert(cdcBytes.size() == 528);
    assert(std::all_of(cdcBytes.begin() + 36, cdcBytes.begin() + 40, [](uint8_t v) { return v == 0; }));
    assert(g_tele.cdcTxBytes == 528);
    pass("TELEMETRY empty gate counter keeps fixed 528-byte layout");

    beginCase();
    g_mbrDistanceReadFailures = 0x12345678u;
    g_loopCount = 0xAABBCCDDu;
    g_tele.riseCnt[0] = 0xE1F2;
    g_tele.fallCnt[31] = 0x3456;
    g_verifyFail[35] = 0xBC;
    g_tele.edgeIdx = 0x778899AAu;
    g_tele.edgeDir[31] = 0xEF;
    cdcWriteLimit = 64;
    onCdcWrite = []() { g_mbrDistanceReadFailures = 0x87654321u; };
    readTelemetry();
    assert(cdcBytes.size() == 528);
    const uint8_t fields[] = {0xDD, 0xCC, 0xBB, 0xAA, 0x78, 0x56, 0x34, 0x12, 0xF2, 0xE1};
    assert(std::equal(fields, fields + sizeof(fields), cdcBytes.begin() + 32));
    assert(cdcBytes[166] == 0x56 && cdcBytes[167] == 0x34);
    assert(cdcBytes[203] == 0xBC && cdcBytes[204] == 0xAA && cdcBytes[207] == 0x77);
    assert(cdcBytes[527] == 0xEF && g_tele.cdcTxBytes == 528);
    assert(g_mbrDistanceReadFailures == 0x87654321u);
    pass("TELEMETRY gate counter snapshots once, split TX preserves adjacent fields");

    beginCase();
    paintShared(9);
    started = clockUs;
    readGetInput();
    assertInput();
    assert(clockUs - started < 1000 && touchStateGen.reads == 2);
    assert(std::memcmp(publishedTouchState.slider, touchData32, 32) != 0);
    pass("GET_INPUT during sensor I/O immediately serves prior completed frame");

    beginCase();
    paintShared(9);
    started = clockUs;
    hid_task_chuni_input();
    assertHid();
    assert(clockUs - started < 2000);
    pass("HID during sensor I/O uses completed keys and air, not working arrays");

    beginCase();
    paintShared(9);
    started = clockUs;
    readDebugChain();
    assert(cdcBytes == expectedChain() && clockUs - started < 1000);
    pass("DEBUG_CHAIN during sensor I/O keeps hardware, verified and slider paired");

    beginCase();
    rawReportLevel = 2;
    readGetInput();
    for (unsigned int lane = 0; lane < 32; ++lane)
        assert(cdcBytes[lane] == publishedTouchState.pressure[lane]);
    cdcBytes.clear();
    g_tele.servedCount = g_tele.cdcTxBytes = 0;
    rawReportLevel = 0;
    paintShared(9);
    readGetInput();
    assertInput();
    pass("RAW 2 to 0 during long scan immediately restores completed binary frame");

    beginCase();
    paintShared(9);
    unsigned int barriers = 0;
    onBarrier = [&barriers]() {
        assert(touchStateGen.value & 1u);
        if (++barriers == 2) {
            assert(std::memcmp(publishedTouchState.slider, touchData32, 32) == 0);
            assert(std::memcmp(publishedTouchState.pressure, pressureSnap, 32) == 0);
            assert(std::memcmp(publishedTouchState.keys, touchData, 4) == 0);
            assert(std::memcmp(publishedTouchState.hardware, hwTouch, 6) == 0);
            assert(std::memcmp(publishedTouchState.verified, rawTouch, 6) == 0);
            uint8_t air = 0;
            for (unsigned int key = 0; key < 6; ++key) if (airKeys[key]) air |= 1u << key;
            assert(publishedTouchState.air == air);
        }
    };
    publishTouchState();
    onBarrier = {};
    assert(barriers == 2 && !(touchStateGen.value & 1u));
    readGetInput();
    assertInput();
    pass("Production publisher brackets every field with two barriers then exposes new frame");

    beginCase();
    paintShared(9);
    readMbrTrace();
    assert(cdcBytes.size() == 240 && cdcBytes[0] == CMD_MBR_TOUCH_TRACE && cdcBytes[1] == 1);
    assert(cdcBytes[12] == 0x34 && cdcBytes[13] == 0x12);
    assert(std::memcmp(cdcBytes.data() + 76, publishedTouchState.slider, 32) == 0);
    assert(g_tele.cdcTxBytes == 240);
    pass("C8 reads the completed full-width frame and native LE counts");

    beginCase();
    touchStateGen.value |= 1u;
    readMbrTrace();
    assert(cdcBytes.size() == 240 && cdcBytes[3] == MBR_TRACE_COPY_FAILED);
    assert(touchStateGen.reads == 3 && clockCalls < 10);
    for (unsigned i = 4; i < 240; ++i) assert(cdcBytes[i] == 0);
    pass("C8 odd writer produces explicit invalid frame in three bounded attempts");

    beginCase();
    touchStateGen.onRead = [](unsigned int read) {
        if (read == 2) { paintShared(9); publishTouchState(); }
    };
    readMbrTrace();
    assert(cdcBytes.size() == 240 && cdcBytes[3] == 0);
    assert(std::memcmp(cdcBytes.data() + 76, publishedTouchState.slider, 32) == 0);
    pass("C8 retries a concurrent publication without mixing fields");

    beginCase();
    cdcAvailable = 239;
    readMbrTrace();
    assert(cdcBytes.size() == 1 && cdcBytes[0] == MBR_TRACE_BUSY);
    assert(g_tele.cdcTxBytes == 1 && clockCalls < 10);
    pass("C8 insufficient TX space sends only BUSY without waiting");

    beginCase();
    cdcAvailable = 0;
    readMbrTrace();
    assert(cdcBytes.empty() && g_tele.cdcTxBytes == 0 && clockCalls < 10);
    pass("C8 full TX FIFO returns without blocking HID");

    std::cout << passed << '/' << passed << " production-reader scenarios passed\n";
}
"""


def find_compiler(explicit):
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise SystemExit(f"Compiler does not exist: {path}")
        return [str(path), "c++"] if path.stem == "zig" else [str(path)]
    zig = Path(r"D:\tools\zig-x86_64-windows-0.16.0\zig.exe")
    if zig.is_file():
        return [str(zig), "c++"]
    for name in ("clang++", "g++", "c++"):
        path = shutil.which(name)
        if path:
            return [path]
    raise SystemExit("No native C++ compiler found; specify --compiler.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="Path to a native C++ compiler or Zig executable")
    args = parser.parse_args()
    variant = Path(__file__).resolve().parents[1]
    production = variant / "src" / "chuni_io.cpp"
    source = production.read_text(encoding="utf-8")
    hardware = (variant / "src" / "hw_devices.cpp").read_text(encoding="utf-8")
    globals_start = source.index("struct NyanithmInput {")
    globals_end = source.index("volatile bool pending_config_mode", globals_start)
    globals_code = source[globals_start:globals_end]
    pieces = [
        HOST,
        "void publishTouchState() {" + extract_body(hardware, "static void publishTouchState()") + "}\n",
        globals_code,
        "bool rawModeActive() {" + extract_body(source, "bool rawModeActive()") + "}\n",
        "void hid_task_chuni_input() {" + extract_body(source, "void hid_task_chuni_input()") + "}\n",
        "void readGetInput() {" + extract_body(source, "if (cmd == CMD_GET_INPUT)") + "}\n",
        "void readDebugChain() {" + extract_body(source, "if (cmd == CMD_DEBUG_CHAIN)") + "}\n",
        "void readMbrTrace() {" + extract_body(source, "if (cmd == CMD_MBR_TOUCH_TRACE)") + "}\n",
        "void readTelemetry() {" + extract_body(source, "if (cmd == CMD_DEBUG_TELEMETRY)") + "}\n",
        CASES,
    ]
    with tempfile.TemporaryDirectory(prefix="nyanithm_touch_readers_") as directory:
        temporary = Path(directory)
        (temporary / "stdint-gcc.h").write_text("#pragma once\n#include <cstdint>\n", encoding="utf-8")
        harness = temporary / "test.cpp"
        harness.write_text("\n".join(pieces), encoding="utf-8")
        binary = temporary / ("test.exe" if os.name == "nt" else "test")
        command = find_compiler(args.compiler) + [
            "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
            "-I", str(temporary), "-I", str(variant / "include" / "share"),
            "-I", str(variant / "include" / "software"),
            str(harness), "-o", str(binary),
        ]
        result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
        if result.returncode:
            raise SystemExit((result.stdout + result.stderr)[-12000:])
        print(f"Compiled production readers extracted from: {production}", flush=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
