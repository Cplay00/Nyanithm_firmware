#!/usr/bin/env python3
"""Exercise production recorder/CDC code with the official SDK queue implementation.

Only clocks and hardware locks are replaced for the host; no device is accessed.
"""
import os
import argparse
from pathlib import Path
import subprocess
import tempfile

from test_touch_snapshot_readers import extract_body, find_compiler

STUBS = {
    'pico.h': '#pragma once\n#include <cstdint>\n#include <cassert>\nusing uint = unsigned int;\n',
    'hardware/sync.h': r'''#pragma once
#include <pico.h>
#include <mutex>
struct spin_lock_t { std::mutex mutex; };
inline spin_lock_t hostLocks[32];
inline uint next_striped_spin_lock_num() { return 16; }
inline uint32_t spin_lock_blocking(spin_lock_t* lock) { lock->mutex.lock(); return 0; }
inline void spin_unlock(spin_lock_t* lock, uint32_t) { lock->mutex.unlock(); }
''',
    'pico/lock_core.h': r'''#pragma once
typedef struct { spin_lock_t* spin_lock; } lock_core_t;
inline void (*hostNotifyHook)() = nullptr;
inline void lock_init(lock_core_t* core, uint id) { core->spin_lock = &hostLocks[id]; }
inline void lock_internal_spin_unlock_with_notify(lock_core_t* core, uint32_t save) {
    spin_unlock(core->spin_lock, save);
    if (hostNotifyHook) { auto hook = hostNotifyHook; hostNotifyHook = nullptr; hook(); }
}
inline void lock_internal_spin_unlock_with_wait(lock_core_t* core, uint32_t save) { spin_unlock(core->spin_lock, save); }
''',
    'pico/critical_section.h': r'''#pragma once
#include <hardware/sync.h>
struct critical_section_t { spin_lock_t* lock; };
inline void critical_section_init(critical_section_t* guard) { guard->lock = &hostLocks[17]; }
inline void critical_section_enter_blocking(critical_section_t* guard) { spin_lock_blocking(guard->lock); }
inline void critical_section_exit(critical_section_t* guard) { spin_unlock(guard->lock, 0); }
''',
    'pico/stdlib.h': '#pragma once\n#include <atomic>\ninline std::atomic<uint32_t> hostUs{0};\ninline uint32_t time_us_32() { return hostUs.fetch_add(1); }\n'
}

HOST = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <mbr_history.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
const unsigned CFG_TUD_CDC_TX_BUFSIZE = 256;
unsigned available = 256;
std::vector<uint8_t> tx;
struct { uint32_t cdcTxBytes = 0; } g_tele;
unsigned tud_cdc_write_available() { return available; }
unsigned tud_cdc_write(const void* source, unsigned count) {
    assert(count <= available);
    const auto* p = static_cast<const uint8_t*>(source);
    tx.insert(tx.end(), p, p + count); available -= count; return count;
}
void tud_cdc_write_flush() {}
'''

PRODUCER = r'''
struct MbrDistanceSample { uint16_t counts[16]{}; bool valid{}; MbrTraceChip quality{}; };
MbrDistanceSample mbrDistanceSamples[3];
int16_t laneTable[3][16];
uint8_t pressureSnap[32];
uint32_t producerMs = 100;
uint32_t get_absolute_time() { return producerMs; }
uint32_t to_ms_since_boot(uint32_t value) { return value; }
struct CY8CMBR3116 {
    struct DifferenceReadInfo { uint32_t ioFailures{}, syncMismatches{}; uint8_t sync{}; };
    uint16_t native[16]{};
    bool readDifferenceCounts(uint16_t* out, DifferenceReadInfo* info, bool) {
        std::memcpy(out, native, sizeof(native)); info->sync = 1; return true;
    }
};
'''

CASES = r'''
unsigned checks = 0;
void checkAt(bool condition, const char* expression, unsigned line) {
    ++checks;
    if (!condition) { std::cerr << "FAIL " << line << ": " << expression << '\n'; std::abort(); }
}
#define check(expression) checkAt(bool(expression), #expression, __LINE__)
MbrTouchTrace frame(uint32_t ms, uint32_t id, uint16_t mask = 0) {
    MbrTouchTrace t{};
    t.tag = CMD_MBR_TOUCH_TRACE; t.version = MBR_TRACE_VERSION;
    t.flags = MBR_TRACE_PROFILE; t.chipCount = 3;
    t.publishedMs = ms; t.frameId = id; t.hardware[0] = t.verified[0] = mask;
    t.slider[0] = mask ? 128 : 0;
    for (unsigned i = 0; i < 3; ++i) {
        t.chips[i].flags = MBR_TRACE_VALID | MBR_TRACE_HAS_GOOD | MBR_TRACE_COHERENT | MBR_TRACE_BUTTON_VALID | MBR_TRACE_READ_THIS_FRAME;
        t.chips[i].rangeMask = 0xffff;
    }
    for (unsigned i = 0; i < 32; ++i) t.counts[i] = uint16_t(id + i);
    return t;
}
void arm(uint32_t ms, uint32_t id = 1) {
    check(requestMbrHistory(CMD_MBR_HISTORY_ARM));
    check(!requestMbrHistory(CMD_MBR_HISTORY_ARM));
    MbrHistoryFrame f{};
    check(!readMbrHistory(f));
    recordMbrHistory(frame(ms, id));
    check(getMbrHistoryStatus().state == MBR_HISTORY_ARMED);
}
void command(uint8_t cmd, unsigned space = 256) { available = space; tx.clear(); runCommand(cmd); }
int main() {
    for (auto& chip : laneTable) for (auto& lane : chip) lane = -1;
    laneTable[0][7] = 16;
    CY8CMBR3116 chip{};
    mbrDistanceReadAttempt(0, &chip);
    check(mbrDistanceSamples[0].quality.rangeMask == 0xffff);
    chip.native[7] = 256;
    mbrDistanceReadAttempt(0, &chip);
    check(mbrDistanceSamples[0].quality.rangeMask == 0xff7f);
    check(mbrDistanceSamples[0].counts[7] == 256);
    chip.native[7] = 255;
    mbrDistanceReadAttempt(0, &chip);
    check(mbrDistanceSamples[0].quality.rangeMask == 0xffff);
    initMbrHistory();
    recordMbrHistory(frame(0, 0, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_OFF);
    check(getMbrHistoryStatus().stored == 0);
    check(!requestMbrHistory(0));
    command(CMD_MBR_HISTORY_ARM, 1);
    check(tx.size() == 1 && tx[0] == 0 && !getMbrHistoryStatus().pending);
    arm(100);
    const auto epoch = getMbrHistoryStatus().session;
    for (unsigned i = 2; i <= 45; ++i) recordMbrHistory(frame(100 + (i - 1) * 5, i));
    recordMbrHistory(frame(325, 46, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_POST);
    check(getMbrHistoryStatus().triggerFrame == 46);
    for (unsigned i = 47; i <= 78; ++i) recordMbrHistory(frame(325 + (i - 46) * 5, i, 1));
    auto s = getMbrHistoryStatus();
    check(s.state == MBR_HISTORY_FROZEN && s.queued == 78 && s.reason == MBR_HISTORY_EDGE);
    check(s.endedMs - s.triggerMs == MBR_HISTORY_POST_MS);
    command(CMD_MBR_HISTORY_READ, 247);
    check(tx.size() == 1 && getMbrHistoryStatus().queued == 78);
    command(CMD_MBR_HISTORY_READ, 0);
    check(tx.empty() && getMbrHistoryStatus().queued == 78);
    for (unsigned i = 1; i <= 78; ++i) {
        command(CMD_MBR_HISTORY_READ);
        check(tx.size() == sizeof(MbrHistoryFrame));
        MbrHistoryFrame f{}; std::memcpy(&f, tx.data(), sizeof(f));
        check(f.session == epoch && f.trace.frameId == i && f.trace.counts[31] == i + 31);
    }
    command(CMD_MBR_HISTORY_READ); check(tx.size() == 1 && tx[0] == 0);
    arm(1000);
    for (unsigned i = 2; i <= 120; ++i) recordMbrHistory(frame(1000 + i * 5, i));
    check(getMbrHistoryStatus().queued == 96 && getMbrHistoryStatus().overwritten == 24);
    command(CMD_MBR_HISTORY_FREEZE); check(tx.size() == 2 && tx[1] == 1);
    recordMbrHistory(frame(1700, 121));
    check(getMbrHistoryStatus().reason == MBR_HISTORY_MANUAL);
    MbrHistoryFrame f{};
    check(readMbrHistory(f) && f.trace.frameId == 25);
    command(CMD_MBR_HISTORY_STATUS); check(tx.size() == sizeof(MbrHistoryStatus));
    check(requestMbrHistory(CMD_MBR_HISTORY_ARM));
    check(!readMbrHistory(f));
    recordMbrHistory(frame(2000, 1));
    check(getMbrHistoryStatus().queued == 1 && getMbrHistoryStatus().session == epoch + 2);
    disconnectMbrHistory();
    recordMbrHistory(frame(2010, 2));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN);
    arm(3000);
    auto bad = frame(3005, 2); bad.flags = 0;
    recordMbrHistory(bad);
    check(getMbrHistoryStatus().reason == MBR_HISTORY_PROFILE_CHANGED);
    check(requestMbrHistory(CMD_MBR_HISTORY_ARM)); recordMbrHistory(bad);
    check(getMbrHistoryStatus().state == MBR_HISTORY_UNSUPPORTED);
    arm(0xFFFFFF00u);
    recordMbrHistory(frame(0xFFFFFFC0u, 2, 1));
    recordMbrHistory(frame(0x60u, 3, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN);
    arm(1000);
    recordMbrHistory(frame(11000, 2));
    check(getMbrHistoryStatus().reason == MBR_HISTORY_TIMEOUT);
    arm(100);
    recordMbrHistory(frame(150, 2, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_ARMED);
    recordMbrHistory(frame(300, 3, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_ARMED);
    recordMbrHistory(frame(310, 4, 0));
    check(getMbrHistoryStatus().triggerFrame == 4);
    requestMbrHistory(CMD_MBR_HISTORY_FREEZE); recordMbrHistory(frame(320, 5));
    check(requestMbrHistory(CMD_MBR_HISTORY_ARM));
    disconnectMbrHistory(); recordMbrHistory(frame(400, 6));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN && !getMbrHistoryStatus().pending);
    check(requestMbrHistory(CMD_MBR_HISTORY_ARM));
    hostNotifyHook = disconnectMbrHistory;
    recordMbrHistory(frame(500, 7));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN && getMbrHistoryStatus().queued == 0);
    arm(1000);
    auto invalid = frame(1200, 2, 1); invalid.chips[2].flags = 0;
    recordMbrHistory(invalid);
    check(getMbrHistoryStatus().state == MBR_HISTORY_POST && getMbrHistoryStatus().triggerFrame == 2);
    recordMbrHistory(frame(1210, 3, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_POST);
    recordMbrHistory(frame(1220, 4, 0));
    check(getMbrHistoryStatus().triggerFrame == 2);
    requestMbrHistory(CMD_MBR_HISTORY_FREEZE); recordMbrHistory(frame(1230, 5));
    check(readMbrHistory(f)); check(readMbrHistory(f) && f.trace.chips[2].flags == 0);
    arm(2000);
    auto produced = frame(2200, 2, 1);
    produced.chips[0] = mbrDistanceSamples[0].quality;
    produced.chips[0].flags |= MBR_TRACE_BUTTON_VALID;
    recordMbrHistory(produced);
    check(getMbrHistoryStatus().state == MBR_HISTORY_POST);
    recordMbrHistory(frame(2360, 3, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN);
    check(readMbrHistory(f));
    check(readMbrHistory(f) && f.trace.chips[0].rangeMask == 0xffff);
    arm(3000);
    auto cached = frame(3200, 2, 1);
    for (auto& q : cached.chips) q.flags &= ~MBR_TRACE_READ_THIS_FRAME;
    recordMbrHistory(cached);
    check(getMbrHistoryStatus().state == MBR_HISTORY_POST);
    recordMbrHistory(frame(3360, 3, 1));
    check(getMbrHistoryStatus().state == MBR_HISTORY_FROZEN);
    std::atomic<bool> stop{false};
    std::thread producer([&] {
        for (uint32_t id = 1; !stop; ++id) recordMbrHistory(frame(id * 5, id));
    });
    for (unsigned round = 0; round < 200; ++round) {
        while (!requestMbrHistory(CMD_MBR_HISTORY_ARM)) std::this_thread::yield();
        while (getMbrHistoryStatus().pending) std::this_thread::yield();
        while (!requestMbrHistory(CMD_MBR_HISTORY_FREEZE)) std::this_thread::yield();
        while (getMbrHistoryStatus().pending) std::this_thread::yield();
        auto frozen = getMbrHistoryStatus();
        check(frozen.state == MBR_HISTORY_FROZEN && frozen.queued <= 96);
        uint32_t previous = 0;
        while (readMbrHistory(f)) {
            check(f.session == frozen.session && f.trace.frameId > previous);
            check(f.trace.counts[31] == uint16_t(f.trace.frameId + 31));
            previous = f.trace.frameId;
        }
    }
    stop = true; producer.join();
    std::cout << "PASS: production history, SDK queue, CDC backpressure, 200 cross-core sessions; " << checks << " assertions\n";
}
'''

def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--recorder-source', type=Path, default=repo / 'src/mbr_history.cpp')
    args = parser.parse_args()
    sdk = Path('D:/pico-sdk')
    body = extract_body((repo / 'src/chuni_io.cpp').read_text(encoding='utf-8'), 'if (cmd == CMD_MBR_HISTORY_ARM ||')
    producer = extract_body((repo / 'src/hw_devices.cpp').read_text(encoding='utf-8'), 'static bool mbrDistanceReadAttempt(')
    with tempfile.TemporaryDirectory(prefix='mbr_history_') as directory:
        tmp = Path(directory)
        for name, data in STUBS.items():
            p = tmp / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(data, encoding='utf-8')
        harness = tmp / 'host.cpp'
        harness.write_text(HOST + PRODUCER + '\nbool mbrDistanceReadAttempt(uint8_t m, CY8CMBR3116* chip) {' + producer + '}\nvoid runCommand(uint8_t cmd) {' + body + '}\n' + CASES, encoding='utf-8')
        binary = tmp / ('test.exe' if os.name == 'nt' else 'test')
        cmd = find_compiler(None) + ['-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-x', 'c++',
            '-I', str(tmp), '-I', str(repo / 'include/software'), '-I', str(repo / 'include/share'),
            '-I', str(sdk / 'src/common/pico_util/include'), str(args.recorder_source),
            str(sdk / 'src/common/pico_util/queue.c'), str(harness), '-o', str(binary)]
        built = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
        if built.returncode:
            raise SystemExit((built.stdout + built.stderr)[-12000:])
        subprocess.run([str(binary)], check=True, timeout=30)

if __name__ == '__main__':
    main()
