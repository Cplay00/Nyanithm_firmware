#!/usr/bin/env python3
"""Compile the production config-mode C6 branch with native-I2C/USB fakes.

Exercises transfer failures, forbidden addresses, legacy MBR compatibility,
partial CDC replies, watchdog pumping, timeout wraparound, and disconnect.
No device access and no copied C6 implementation.
"""
import os
from pathlib import Path
import subprocess
import tempfile

from test_touch_snapshot_readers import extract_body, find_compiler
from test_mbr_distance_pipeline import extract_function

HOST = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
struct Read { uint8_t address, offset; size_t size; };
std::vector<Read> reads;
std::vector<uint8_t> transmitted;
std::string errors;
uint32_t clockMs, watchdogPumps, usbPumps, writeCalls;
uint32_t configSessionEpoch = 0;
uint8_t payload;
bool payloadOk, connected, mbrOk;
int failCall, failResult, mbrReads;
uint32_t txChunk, stopAfter, disconnectAfter;
bool readCdcPayload(uint8_t* dst, int len, uint32_t timeout) {
    assert(len == 1 && timeout == 500);
    if (payloadOk) dst[0] = payload;
    return payloadOk;
}
int i2c_write_read(uint8_t port, uint8_t address, uint8_t* wr, size_t wrLen,
                   uint8_t* dst, size_t rdLen) {
    assert(port == 0 && wrLen == 1); // pointer write only, no register values
    reads.push_back({address, wr[0], rdLen});
    for (size_t i = 0; i < rdLen; ++i) dst[i] = uint8_t((wr[0] + i) ^ address);
    return int(reads.size()) == failCall ? failResult : int(rdLen);
}
bool read_cy8cmbr3116_config(uint8_t address, uint8_t* dst) {
    ++mbrReads;
    for (unsigned i = 0; i < 128; ++i) dst[i] = uint8_t(i ^ address);
    return mbrOk;
}
uint32_t get_absolute_time() { return clockMs++; }
uint32_t to_ms_since_boot(uint32_t t) { return t; }
void watchdog_update() { ++watchdogPumps; }
void tud_task() { ++usbPumps; }
void sleep_ms(uint32_t t) { clockMs += t; }
void tud_cdc_write_flush() {}
bool tud_cdc_connected() { return connected && transmitted.size() < disconnectAfter; }
bool tud_mounted() { return connected && transmitted.size() < disconnectAfter; }
uint32_t tud_cdc_write(const uint8_t* src, uint32_t size) {
    ++writeCalls;
    uint32_t n = std::min(size, txChunk);
    n = std::min(n, stopAfter - uint32_t(transmitted.size()));
    transmitted.insert(transmitted.end(), src, src + n);
    return n;
}
int capturePrintf(const char* text) {
    errors += text;
    return int(std::strlen(text));
}
template<class... Args> int capturePrintf(const char* format, Args... args) {
    char buf[128];
    int size = std::snprintf(buf, sizeof(buf), format, args...);
    errors += buf;
    return size;
}
#define printf capturePrintf
'''

CASES = r'''
#undef printf
unsigned passed = 0;
void reset(uint8_t address = 0x5A) {
    reads.clear(); transmitted.clear(); errors.clear();
    clockMs = 1; watchdogPumps = usbPumps = writeCalls = 0;
    payload = address; payloadOk = connected = mbrOk = true;
    failCall = -1; failResult = -1; mbrReads = 0;
    txChunk = 128; stopAfter = disconnectAfter = UINT32_MAX;
}
void pass(const char* label) { ++passed; std::cout << "PASS " << label << '\n'; }
void checkImage(uint8_t address) {
    assert(transmitted.size() == 128 && errors.empty());
    for (unsigned i = 0; i < 128; ++i) assert(transmitted[i] == uint8_t(i ^ address));
}
int main() {
    for (uint8_t address = 0x5A; address <= 0x5C; ++address) {
        reset(address); readC6(); checkImage(address);
        assert(reads.size() == 2 && mbrReads == 0);
        assert(reads[0].address == address && reads[0].offset == 0 && reads[0].size == 43);
        assert(reads[1].address == address && reads[1].offset == 0x2B && reads[1].size == 85);
        assert(watchdogPumps >= 2);
        pass("MPR coherent signal bank + independent controls, unchanged byte order");
    }
    for (int call = 1; call <= 2; ++call) {
        for (int result : {-2, -1, 0, call == 1 ? 42 : 84}) {
            reset(); failCall = call; failResult = result; readC6();
            assert(reads.size() == unsigned(call) && transmitted.empty());
            assert(errors == "mpr read fail\n" && mbrReads == 0);
            pass("negative, zero and short native reads discard entire image");
        }
    }
    for (uint8_t address : {0x37, 0x40, 0x44}) {
        reset(address); readC6(); checkImage(address);
        assert(reads.empty() && mbrReads == 1);
        pass("legacy MBR address routes through unchanged driver");
    }
    reset(0x40); mbrOk = false; readC6();
    assert(transmitted.empty() && errors == "3116 read fail\n");
    pass("legacy MBR read failure unchanged");
    for (uint8_t address : {0x00, 0x39, 0x45, 0x59, 0x5D, 0x7F}) {
        reset(address); readC6();
        assert(reads.empty() && mbrReads == 0 && transmitted.empty() && !errors.empty());
        pass("unlisted address never reaches I2C");
    }
    reset(); payloadOk = false; readC6();
    assert(reads.empty() && mbrReads == 0 && transmitted.empty() && errors.empty());
    pass("missing CDC payload cannot reuse stale address");
    reset(); txChunk = 7; readC6(); checkImage(0x5A);
    assert(writeCalls == 19 && watchdogPumps >= writeCalls);
    pass("partial successful USB writes advance exact payload offset");
    reset(); txChunk = 0; readC6();
    assert(transmitted.empty() && errors.empty() && watchdogPumps > 0);
    assert(clockMs >= 250 && clockMs < 270);
    pass("stalled TX is bounded to 250ms and pumps watchdog");
    reset(); stopAfter = 64; readC6();
    assert(transmitted.size() == 64 && errors.empty() && clockMs < 270);
    pass("partial binary timeout appends no misleading error bytes");
    reset(); connected = false; readC6();
    assert(reads.size() == 2 && transmitted.empty() && writeCalls == 0);
    pass("disconnected host sends no binary reply");
    reset(); txChunk = 64; disconnectAfter = 64; readC6();
    assert(transmitted.size() == 64 && writeCalls == 1 && errors.empty());
    pass("disconnect after first packet terminates without a suffix");
    reset(); clockMs = UINT32_MAX - 100; txChunk = 0;
    uint32_t started = clockMs; readC6();
    assert(uint32_t(clockMs - started) >= 250 && uint32_t(clockMs - started) < 270);
    pass("unsigned timeout remains bounded across uptime wrap");
    std::cout << passed << '/' << passed << " native-register scenarios passed\n";
}
'''


def main():
    variant = Path(__file__).resolve().parents[1]
    production = variant / 'src/app_link.cpp'
    source = production.read_text(encoding='utf-8')
    body = extract_body(source, 'else if (cmd == CMD_READ3116CONFIG)')
    sender, _ = extract_function(source, 'static bool writeCdcPayload(')
    with tempfile.TemporaryDirectory(prefix='nyanithm_native_dump_') as directory:
        work = Path(directory)
        harness = work / 'dump.cpp'
        binary = work / ('dump.exe' if os.name == 'nt' else 'dump')
        harness.write_text(HOST + sender + '\nvoid readC6() {\n' + body + '\n}\n' + CASES, encoding='utf-8')
        command = find_compiler(None) + ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-O2',
                                        str(harness), '-o', str(binary)]
        result = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace')
        if result.returncode:
            raise SystemExit((result.stdout + result.stderr)[-12000:])
        print(f'Compiled production C6 branch from: {production}', flush=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
