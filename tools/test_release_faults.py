#!/usr/bin/env python3
"""Fault-inject exact release CDC/native-driver functions; no hardware access."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

from test_mbr_distance_pipeline import extract_function
from test_touch_snapshot_readers import find_compiler

ROOT = Path(__file__).resolve().parents[1]
HOST = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <string>
#include <vector>
uint32_t nowMs, pumps, feeds, flushed, txChunk = 256;
bool mounted = true, flashingArmed = false;
std::deque<uint8_t> rx;
std::vector<uint8_t> tx;
std::function<void()> pumpHook;
uint32_t get_absolute_time() { return nowMs++; }
uint32_t to_ms_since_boot(uint32_t n) { return n; }
bool tud_mounted() { return mounted; }
void tud_task() { ++pumps; if (pumpHook) pumpHook(); }
void watchdog_update() { ++feeds; }
void sleep_ms(uint32_t n) { nowMs += n; }
void sleep_us(uint32_t) {}
uint32_t tud_cdc_available() { return rx.size(); }
uint32_t tud_cdc_read(void* dest, uint32_t n) {
    n = std::min(n, uint32_t(rx.size()));
    for (uint32_t i = 0; i < n; ++i) { ((uint8_t*)dest)[i] = rx.front(); rx.pop_front(); }
    return n;
}
void tud_cdc_read_flush() { rx.clear(); ++flushed; }
uint32_t tud_cdc_write(const uint8_t* p, uint32_t n) {
    n = std::min(n, txChunk); tx.insert(tx.end(), p, p + n); return n;
}
void tud_cdc_write_flush() {}
std::deque<int> readResults, writeResults;
uint8_t ecrValue = 0x8c;
std::vector<std::vector<uint8_t>> busWrites;
unsigned busReads;
int i2c_write_read(uint8_t, uint8_t, uint8_t* reg, size_t, uint8_t* dest, size_t n) {
    ++busReads;
    int result = int(n);
    if (!readResults.empty()) { result = readResults.front(); readResults.pop_front(); }
    if (result > 0) {
        std::memset(dest, 0xff, n);
        if (*reg == 0x5e) dest[0] = ecrValue;
    }
    return result;
}
int i2c_write(uint8_t, uint8_t, uint8_t* data, size_t n, bool noStop) {
    assert(n == 2 && !noStop); busWrites.push_back({data, data+n});
    int result = int(n);
    if (!writeResults.empty()) { result = writeResults.front(); writeResults.pop_front(); }
    if (result == 2 && data[0] == 0x5e) ecrValue = data[1];
    return result;
}
constexpr uint8_t MPR121_ECR = 0x5e, MPR121_TOUCHSTATUS_L = 0, MPR121_AUTOCONFIG0 = 0x7b;
class MPR121 {
public:
    uint8_t port = 0, addr = 0x5a; bool good = true;
    bool readRegisters(uint8_t, uint8_t*, uint8_t);
    void writeRegister(uint8_t, uint8_t);
    bool readTouchStatus(uint16_t*);
    void calibrateBaseline(bool force);
};
constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0b, RESULT_INTERRUPT_STATUS = 0x13;
constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
class VL53L0X {
public:
    uint8_t port = 1, address = 0x29;
    bool interrupt_clear_pending = false, did_timeout = false, expired = false;
    unsigned nativeReads = 0, timeoutStarts = 0;
    uint8_t status = 7; uint16_t sample = 321;
    uint8_t readReg(uint8_t reg) { assert(reg == RESULT_INTERRUPT_STATUS); ++nativeReads; return status; }
    uint16_t readReg16Bit(uint8_t reg) { assert(reg == RESULT_RANGE_STATUS + 10); ++nativeReads; return sample; }
    void startTimeout() { ++timeoutStarts; }
    bool checkTimeoutExpired() { return expired; }
    bool readRangeContinuousMillimetersAsync(uint16_t*);
};
'''

CASES = r'''
unsigned passed = 0;
void pass(const char* name) { ++passed; std::cout << "PASS " << name << '\n'; }
void begin() {
    rx.clear(); tx.clear(); pumpHook = {}; mounted = true; flashingArmed = true;
    nowMs = pumps = feeds = flushed = 0; txChunk = 256;
    resetConfigCdcSession();
    configPayloadReading = false;
    readResults.clear(); writeResults.clear(); busWrites.clear(); busReads = 0; ecrValue = 0x8c;
}
int main() {
    uint8_t buf[128]{};
    begin(); for (unsigned n=0; n<128; ++n) rx.push_back(n);
    assert(readCdcPayload(buf, 128, 1000));
    for (unsigned n=0; n<128; ++n) assert(buf[n] == n);
    assert(!configPayloadRemaining && feeds > 0); pass("complete CFG_SET remains byte exact");

    for (unsigned length : {1u, 128u}) {
        begin(); if (length == 128) for (unsigned n=0; n<64; ++n) rx.push_back(n);
        assert(!readCdcPayload(buf, length, 100));
        assert(configPayloadRemaining == (length == 128 ? 64 : 1) && !flashingArmed);
        for (unsigned n=0; n<configPayloadRemaining; ++n) rx.push_back(n%3 == 0 ? 0xb6 : n%3 == 1 ? 0xbb : 0xa5);
        while (configPayloadRemaining) assert(discardConfigPayload());
        assert(rx.empty() && !discardConfigPayload()); pass("late destructive bytes stay payload and are consumed");
    }
    begin(); pumpHook = []() { nowMs += 10; rx.push_back(0x88); };
    assert(!readCdcPayload(buf, 128, 100));
    assert(nowMs < 140 && configPayloadRemaining > 0 && feeds > 0);
    pass("continuous slow trickle cannot bypass total payload deadline");

    begin(); configCdcSessionStateChanged(true);
    pumpHook = []() { if (pumps == 1) { rx.push_back(0xb6); configCdcSessionStateChanged(false); } };
    assert(!readCdcPayload(buf, 128, 100));
    assert(configPayloadQuarantined && rx.empty());
    pumpHook = {}; rx.push_back(0xb9); assert(discardConfigPayload() && rx.empty());
    rx.push_back(0xbb); configCdcSessionStateChanged(true);
    assert(!configPayloadQuarantined && !discardConfigPayload() && rx.empty());
    pass("DTR drop quarantines late bytes until clean rising edge");

    begin(); pumpHook = []() { if (pumps == 1) { rx.push_back(0xb6); configCdcSessionStateChanged(true); } };
    assert(!readCdcPayload(buf, 128, 100) && rx.empty());
    pass("low-DTR payload followed by rising edge cannot join sessions");

    begin(); pumpHook = []() { if (pumps == 1) { rx.push_back(0xb6); resetConfigCdcSession(); } };
    assert(!readCdcPayload(buf, 128, 100) && rx.empty());
    pass("reset and remount in one USB pump aborts old read through epoch");

    begin(); nowMs = UINT32_MAX - 50;
    assert(!readCdcPayload(buf, 128, 100) && nowMs < 80 && feeds > 0);
    pass("payload deadline survives unsigned uptime wrap");

    begin(); txChunk = 0;
    assert(!writeCdcPayload(buf, 128) && nowMs >= 250 && nowMs < 270 && feeds > 0);
    pass("blocked TX is bounded and watchdog is pumped");
    begin(); txChunk = 7; assert(writeCdcPayload(buf, 128) && tx.size() == 128);
    pass("binary reply keeps offsets under short USB writes");
    begin(); pumpHook = []() { if (pumps == 1) resetConfigCdcSession(); };
    assert(!writeCdcPayload(buf, 128) && tx.empty()); pass("session reset stops old reply before write");
    begin(); pumpHook = []() { if (pumps == 2) resetConfigCdcSession(); };
    assert(!writeCdcPayload(buf, 128) && tx.size() == 128);
    pass("reset after last packet prevents trailer in the new session");

    for (int fault : {-2,-1,0}) {
        begin(); MPR121 m; readResults = {fault,fault}; m.writeRegister(0x41, 7);
        assert(!m.good && busWrites.empty() && busReads == 2);
        pass("failed ECR backup never stops a running MPR");
    }
    for (int fault : {-1,0,1}) {
        begin(); MPR121 m; writeResults = {fault}; m.writeRegister(0x41, 7);
        assert(!m.good && busWrites.size() == 1 && busWrites[0][0] == 0x5e);
        pass("failed Stop write prevents unsafe target write");
    }
    begin(); MPR121 m; writeResults = {2,-1,2}; m.writeRegister(0x41, 7);
    assert(!m.good && busWrites.size() == 3 && busWrites.back()[1] == 0x8c);
    pass("failed target write still restores electrode enable state");
    begin(); m = {}; writeResults = {2,2,-1,0,2}; m.writeRegister(0x41, 7);
    assert(m.good && busWrites.size() == 5 && busWrites.back()[1] == 0x8c);
    pass("bounded restore retries recover temporary write faults");
    begin(); m = {}; writeResults = {2,2,-1,0,1}; m.writeRegister(0x41, 7);
    assert(!m.good && busWrites.size() == 5); pass("persistent restore failure exposes not-ready status");
    begin(); m = {}; m.writeRegister(0x5e, 0x8c);
    assert(m.good && busReads == 0 && busWrites.size() == 1);
    pass("direct ECR write needs no artificial Stop cycle");
    begin(); uint16_t bits = 0; readResults = {-1,2};
    assert(m.readTouchStatus(&bits) && bits == 0x0fff && busReads == 2);
    pass("native status retry masks out non-electrode bits");
    begin(); readResults = {1,-1}; bits = 0xffff;
    assert(!m.readTouchStatus(&bits) && bits == 0 && busReads == 2);
    pass("native status failure is distinct from valid all-OFF");

    begin(); m = {}; m.calibrateBaseline(true);
    assert(m.good && ecrValue == 0xcc && busWrites.size() == 3);
    pass("healthy native baseline calibration restores CL=11 Run Mode");
    begin(); m = {}; writeResults = {2,2,-1,0,1}; m.calibrateBaseline(true);
    assert(!m.good && ecrValue == 0 && busWrites.size() == 5);
    pass("baseline restore failure is reported not-ready instead of silent Stop Mode");
    begin(); m = {}; writeResults = {2,2,-1,2}; m.calibrateBaseline(true);
    assert(m.good && ecrValue == 0xcc && busWrites.size() == 4);
    pass("baseline restore recovers temporary write failure");
    begin(); m = {}; readResults = {1,-1,-1,1}; m.calibrateBaseline(true);
    assert(m.good && ecrValue == 0xcc && busReads == 4);
    pass("baseline restore requires checked readback and retries transient read failure");
    begin(); m = {}; readResults = {-1}; m.calibrateBaseline(true);
    assert(!m.good && busWrites.empty()); pass("failed calibration ECR backup never enters Stop Mode");
    begin(); m = {}; readResults = {-1,-1}; m.calibrateBaseline(false);
    assert(!m.good && busWrites.empty()); pass("failed status does not authorize baseline recalibration");
    begin(); m = {}; m.calibrateBaseline(false);
    assert(m.good && busWrites.empty()); pass("held touch at startup safely skips calibration without warning");
    begin(); m = {}; writeResults = {2,-1,2}; m.calibrateBaseline(true);
    assert(!m.good && ecrValue == 0xcc); pass("autoconfig write fault is visible while electrodes are restored");

    for (int fault : {-1,0,1}) {
        begin(); VL53L0X tof; uint16_t range = 0; writeResults = {fault,fault,2};
        assert(tof.readRangeContinuousMillimetersAsync(&range) && range == 8190);
        assert(tof.interrupt_clear_pending && tof.nativeReads == 2);
        assert(tof.readRangeContinuousMillimetersAsync(&range) && range == 8190 && tof.nativeReads == 2);
        assert(tof.readRangeContinuousMillimetersAsync(&range) && range == 8190 && !tof.interrupt_clear_pending);
        assert(tof.nativeReads == 2); tof.sample = 450;
        assert(tof.readRangeContinuousMillimetersAsync(&range) && range == 450 && tof.nativeReads == 4);
        pass("unacknowledged ToF latch never refreshes an old measurement");
    }
    begin(); VL53L0X tof; uint16_t range = 0; tof.status = 0;
    assert(!tof.readRangeContinuousMillimetersAsync(&range));
    tof.expired = true;
    assert(tof.readRangeContinuousMillimetersAsync(&range) && range == 8190 && tof.did_timeout);
    assert(tof.timeoutStarts == 1 && busWrites.empty()); pass("ToF no-result timeout stays invalid and bounded");
    std::cout << passed << '/' << passed << " release-fault scenarios passed\n";
}
'''


def main():
    app = (ROOT / 'src/app_link.cpp').read_text(encoding='utf-8')
    start = app.index('static uint32_t configPayloadRemaining')
    pieces = [HOST, app[start:app.index('void resetConfigCdcSession()', start)]]
    evidence = []
    for file, signatures in (
        ('app_link.cpp', ['void resetConfigCdcSession(', 'void configCdcSessionStateChanged(',
                          'static bool discardConfigPayload(', 'bool readCdcPayload(', 'static bool writeCdcPayload(']),
        ('mpr121.cpp', ['bool MPR121::readRegisters(', 'void MPR121::writeRegister(', 'bool MPR121::readTouchStatus(', 'void MPR121::calibrateBaseline(']),
        ('vl53l0x.cpp', ['bool VL53L0X::readRangeContinuousMillimetersAsync(']),
    ):
        source = (ROOT / 'src' / file).read_text(encoding='utf-8')
        for signature in signatures:
            code, line = extract_function(source, signature)
            pieces.append(code)
            evidence.append({'file': file, 'line': line, 'sha256': hashlib.sha256(code.encode()).hexdigest()})
    pieces.append(CASES)
    with tempfile.TemporaryDirectory(prefix='nyanithm_release_faults_') as directory:
        temp = Path(directory); harness = temp / 'test.cpp'; binary = temp / 'test.exe'
        harness.write_text('\n'.join(pieces), encoding='utf-8')
        command = find_compiler(None) + ['-std=c++17','-Wall','-Wextra','-Werror','-O2',str(harness),'-o',str(binary)]
        compiled = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=60)
        if compiled.returncode: raise SystemExit(compiled.stdout + compiled.stderr)
        result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
        print(result.stdout, end=''); print(result.stderr, end='')
        output = ROOT.parent / '_dev_tools/round90y_release_faults.json'
        output.write_text(json.dumps({'functions':evidence,'returncode':result.returncode,'output':result.stdout+result.stderr},indent=2)+'\n',encoding='utf-8')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
