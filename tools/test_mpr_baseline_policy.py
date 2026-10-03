#!/usr/bin/env python3
"""Exercise the exact production scan loop against an I2C-write spy.

No device access. This checks the Run-mode baseline policy, scan ordering,
watchdog and publication in both layouts and air modes, not chip physics.
The old idle correction is deliberately eligible in the MPR cases.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import test_mbr_distance_pipeline as integration

HOST = r'''
static unsigned order[12], orderCount, baselineReads, baselineWrites;
static unsigned watchdogPumps, publications;
static bool usingIR;
static uint16_t rawTouch[3] = {};
static uint32_t lastTouchedMs[3][12] = {};
static uint32_t g_loopMinUs = 0xffffffffu, g_loopMaxUs, g_loopCount;
static uint64_t g_loopSumUs;
static void mark(unsigned value) { CHECK(orderCount < 12); order[orderCount++] = value; }
static uint32_t time_us_32() { return fakeNow * 1000u; }
static void watchdog_update() { ++watchdogPumps; mark(0); }
static void updateIR() { mark(1); }
static void updateAir() { mark(2); }
static void prepareMbrDistanceGate() { mark(3); }
static void updatePressureSnap() { mark(4); }
static void updateTouch_v1() { mark(5); }
static void updateTouch_v2() { mark(6); }
static void publishTouchState() { ++publications; mark(7); }
class MPR121 {
public:
    uint16_t filteredData(uint8_t) { ++baselineReads; return 800; }
    uint16_t baselineData(uint8_t) { ++baselineReads; return 808; }
    uint8_t readRegister8(uint8_t) { ++baselineReads; return 202; }
    void writeBaselineRun(uint8_t, uint8_t) { ++baselineWrites; }
};
static MPR121 mpr0, mpr1, mpr2;
static controller_config ControllerConfig = {};
#define MPR121_BASELINE_0 0x1e
'''

CASES = r'''
extern "C" void baselinePolicyEntry() {
    const char* command = GetCommandLineA();
    unsigned scenario = 99;
    for (unsigned i = 0; command[i]; ++i) if (command[i] == '=') {
        scenario = static_cast<unsigned>(command[i + 1] - '0'); break;
    }
    CHECK(scenario < 6);
    ControllerConfig.hwVer = scenario < 2 ? scenario + 1 : scenario - 1;
    ControllerConfig.cfg0 = scenario >= 2 ? CFG0_BIT_MBR3116 : 0;
    usingIR = (scenario & 1) != 0;
    // Five minutes covers the rotation and cooldown. The injected idle diff
    // is eligible for old correction; this is not a physical idle-noise model.
    for (unsigned n = 0; n < 600; ++n) {
        fakeNow += 501;
        orderCount = 0;
        updateInputState();
        CHECK(orderCount == 6);
        CHECK(order[0] == 0 && order[1] == (usingIR ? 1u : 2u));
        CHECK(order[2] == 3 && order[3] == 4);
        CHECK(order[4] == (ControllerConfig.hwVer < 3 ? 5u : 6u));
        CHECK(order[5] == 7);
    }
    CHECK(watchdogPumps == 600 && publications == 600 && g_loopCount == 600);
    CHECK(baselineWrites == 0);
    CHECK(baselineReads == 0);
    writeText("PASS checks="); writeNumber(checks); writeText("\n");
    ExitProcess(0);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=integration.VARIANT / 'src/hw_devices.cpp')
    parser.add_argument('--output', type=Path, default=integration.WORKSPACE / '_dev_tools/round90m_baseline_host')
    args = parser.parse_args()
    source = args.source.read_text(encoding='utf-8')
    loop, line = integration.extract_function(source, 'void updateInputState()')
    common = integration.PRELUDE.split('int i2c_write_stop_read(')[0]
    args.output.mkdir(parents=True, exist_ok=True)
    generated = args.output / 'baseline_policy.cpp'
    generated.write_text(common + HOST + f'\n#line {line} "{args.source.as_posix()}"\n' + loop + CASES,
                         encoding='utf-8', newline='\n')
    executable = args.output / 'baseline_policy.exe'
    command = [str(integration.CLANG), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-Wno-unused-variable', '-Wno-missing-braces', '-ffreestanding', '-fno-builtin', '-fno-stack-protector',
               '-fuse-ld=lld', '-nostdlib', f'-I{integration.VARIANT / "include/share"}',
               f'-I{integration.VARIANT / "include/software"}',
               '-ID:/pico-sdk/src/common/pico_base_headers/include', str(generated),
               '-Wl,/entry:baselinePolicyEntry', '-Wl,/subsystem:console', str(integration.KERNEL32),
               '-o', str(executable)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if compiled.returncode:
        raise SystemExit(compiled.stdout + compiled.stderr)
    results = []
    for case in range(6):
        result = subprocess.run([str(executable), f'--case={case}'], capture_output=True, text=True, timeout=10)
        count = re.search(r'PASS checks=(\d+)', result.stdout)
        passed = result.returncode == 0 and count is not None
        results.append({'case': case, 'passed': passed, 'checks': int(count[1]) if count else 0,
                        'output': result.stdout + result.stderr})
        print(f'{"PASS" if passed else "FAIL"} {case}: {result.stdout.strip()}')
    report = {'kind': 'exact production scan loop; fake devices, no hardware',
              'source': str(args.source), 'function_line': line,
              'function_sha256': hashlib.sha256(loop.encode()).hexdigest(),
              'compile_command': command, 'results': results}
    (args.output / 'results.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    total = sum(r['passed'] for r in results)
    print(f'Baseline policy: {total}/6 scenarios; evidence: {args.output / "results.json"}')
    return 0 if total == 6 else 1


if __name__ == '__main__':
    raise SystemExit(main())
