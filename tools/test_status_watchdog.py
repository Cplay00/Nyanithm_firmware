#!/usr/bin/env python3
"""Run exact production B8 output with worst-case bounded stdio/I2C delays."""
import os
from pathlib import Path
import subprocess
import tempfile
from test_mbr_distance_pipeline import extract_function
from test_touch_snapshot_readers import find_compiler

HOST = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <string>
struct { uint8_t hwVer; } ControllerConfig;
static bool hardwareMismatchAtBoot;
static uint8_t g_tofPhysicalMask, g_tofReadyMask, detectedMprMask, detectedMbrMask;
static uint16_t heightDataOriginal[5];
static uint32_t nowMs, lastFeed, maxGap, feeds;
static std::string text;
static bool failReads;
static void advance(uint32_t ms) {
    nowMs += ms;
    if (nowMs-lastFeed > maxGap) maxGap = nowMs-lastFeed;
    assert(nowMs-lastFeed < 2000);
}
static void watchdog_update() { ++feeds; lastFeed = nowMs; }
static int fakePrintf(const char* fmt, ...) {
    advance(500); // production stdio's whole-call deadline, slow host
    char out[512]; va_list args; va_start(args, fmt);
    int count = vsnprintf(out, sizeof(out), fmt, args); va_end(args);
    assert(count >= 0 && count < int(sizeof(out))); text += out; return count;
}
class CY8CMBR3116 {
public:
    uint8_t get_FAMILY_ID(uint8_t* id) { advance(120); *id=0x9a; return failReads; }
    uint8_t get_DEVICE_ID(uint16_t* id) { advance(120); *id=0x0a05; return failReads; }
};
static CY8CMBR3116 MBR3116A, MBR3116B, MBR3116C, MBR3116D, MBR3116E;
static unsigned getConfigPage() { return 15; }
#define printf fakePrintf
'''

CASES = r'''
int main() {
    for (uint8_t hw = 1; hw <= 4; ++hw) {
        for (bool failures : {false, true}) {
            ControllerConfig.hwVer=hw; failReads=failures;
            nowMs=lastFeed=maxGap=feeds=0; text.clear();
            const unsigned count = (hw==2 || hw==4) ? 5 : 4;
            g_tofPhysicalMask=g_tofReadyMask=(1u<<count)-1;
            for (unsigned i=0;i<5;++i) heightDataOriginal[i] = failures ? 8191 : 300;
            getStatus();
            assert(maxGap <= 740 && feeds >= 10);
            assert(text.find("startup config mismatch: 0;") != std::string::npos);
            assert(text.find("using config in page 15") != std::string::npos);
            unsigned warnings=0; size_t offset=0;
            while ((offset=text.find("warning!",offset)) != std::string::npos) { ++warnings; ++offset; }
            assert(warnings == (failures ? count : 0));
            if (!failures) assert(text.find("FAMILY=0x9A DEVICE=0x0A05") != std::string::npos);
            assert(nowMs >= 3500); // total B8 may be slow, each feed interval is bounded
        }
    }
    puts("8/8 production B8 slow-TX/watchdog scenarios passed");
}
'''


def main():
    root = Path(__file__).resolve().parents[1]
    function, _ = extract_function((root/'src/app_link.cpp').read_text(encoding='utf-8'), 'void getStatus()')
    with tempfile.TemporaryDirectory(prefix='nyanithm_status_') as folder:
        cpp = Path(folder)/'status.cpp'
        binary = Path(folder)/('status.exe' if os.name == 'nt' else 'status')
        cpp.write_text(HOST+function+CASES, encoding='utf-8')
        command = find_compiler(None)+['-std=c++17','-Wall','-Wextra','-Werror','-Wno-date-time','-O2',str(cpp),'-o',str(binary)]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise SystemExit(result.stdout+result.stderr)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
