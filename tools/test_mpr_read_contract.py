#!/usr/bin/env python3
"""Compile native MPR production reads with exact-length/NACK/dirty-buffer fakes.

Uses the actual class/register header and method bodies. No device access or
copied read implementation. Failed I/O intentionally dirties the whole buffer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import test_mbr_distance_pipeline as integration

HOST = r'''
struct Reply { int count; uint16_t value; };
static Reply replies[4]; static unsigned queued, consumed, sleeps;
static void queue(int count, uint16_t value = 0xabcd) { CHECK(queued < 4); replies[queued++] = Reply{count, value}; }
static void sleep_us(uint32_t delay) { CHECK(delay == 50); ++sleeps; }
static int i2c_write_read(uint8_t port, uint8_t addr, uint8_t* wr, size_t wrLen, uint8_t* dst, size_t n) {
    CHECK(port == 0 && addr == 0x5a && wrLen == 1 && wr[0] <= 0x80);
    CHECK(consumed < queued);
    Reply reply = replies[consumed++];
    for (size_t i = 0; i < n; ++i) dst[i] = i == 0 ? reply.value & 255 : (i == 1 ? reply.value >> 8 : 0xa5);
    return reply.count;
}
'''

CASES = r'''
extern "C" void mprReadEntry() {
    const char* command = GetCommandLineA(); unsigned number = 99;
    for (unsigned i = 0; command[i]; ++i) if (command[i] == '=') {
        number = 0; while (command[++i] >= '0' && command[i] <= '9') number = number * 10 + command[i] - '0'; break;
    }
    CHECK(number < 24); MPR121 device(0, 0x5a);
    if (number < 8) {
        const uint8_t lengths[] = {1, 2, 12, 24}; uint8_t n = lengths[number / 2];
        uint8_t output[24]; memset(output, 0xcc, sizeof(output));
        queue(number & 1 ? n - 1 : -1);
        CHECK(!device.readRegisters(4, output, n));
        for (unsigned i = 0; i < n; ++i) CHECK(output[i] == 0);
        CHECK(consumed == 1 && sleeps == 0);
        queue(n, 0x1234); CHECK(device.readRegisters(4, output, n)); CHECK(output[0] == 0x34);
    } else if (number < 12) {
        const int failures[] = {-1, 0, 1, 3}; queue(failures[number - 8]); queue(failures[number - 8]);
        CHECK(device.touched() == 0); CHECK(consumed == 2 && sleeps == 1);
    } else if (number < 14) {
        queue(number == 12 ? -1 : 0); queue(number == 12 ? -1 : 0);
        CHECK(device.readRegister8(0x1e) == 0); CHECK(consumed == 2 && sleeps == 1);
    } else if (number < 16) {
        queue(number == 14 ? -1 : 1); queue(number == 14 ? -1 : 1);
        CHECK(device.readRegister16(4) == 0); CHECK(consumed == 2 && sleeps == 1);
    } else if (number < 18) {
        queue(number == 16 ? -1 : 1); queue(2, 0x1234);
        CHECK(device.readRegister16(4) == 0x1234); CHECK(consumed == 2 && sleeps == 1);
    } else if (number == 18) {
        queue(2, 0);
#ifdef CHECKED_MPR_READS
        uint16_t value = 99; bool retry = true;
        CHECK(device.readFilteredData(7, value, &retry) && value == 0 && !retry);
#else
        CHECK(device.filteredData(7) == 0);
#endif
        CHECK(consumed == 1 && sleeps == 0);
    } else if (number == 19) {
        queue(2, 0);
#ifdef CHECKED_MPR_READS
        uint16_t value = 99; CHECK(device.readTouchStatus(value) && value == 0);
#else
        CHECK(device.touched() == 0);
#endif
    } else if (number == 20) {
        CHECK(device.filteredData(13) == 0 && device.baselineData(13) == 0);
        CHECK(consumed == 0);
    } else if (number == 21) {
        queue(-1); queue(2, 0x1fff); CHECK(device.touched() == 0xfff);
        CHECK(consumed == 2 && sleeps == 1);
    } else if (number == 22) {
        queue(2, 0xffff);
#ifdef CHECKED_MPR_READS
        CHECK(device.filteredData(7) == 0);
#else
        // The deployed legacy primitive returns the complete native word;
        // reserved-bit rejection belongs to the optional checked API. This
        // transport test must not silently substitute the retired API contract.
        CHECK(device.filteredData(7) == 0xffff);
#endif
    } else {
        queue(2, 0x0255); CHECK(device.filteredData(12) == 0x255);
        queue(1, 0x0034); CHECK(device.baselineData(12) == 0xd0);
    }
    CHECK(consumed == queued);
    writeText("PASS checks="); writeNumber(checks); writeText("\n"); ExitProcess(0);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=integration.VARIANT / 'src/mpr121.cpp')
    parser.add_argument('--header', type=Path, default=integration.VARIANT / 'include/device/mpr121.h')
    parser.add_argument('--output', type=Path, default=integration.WORKSPACE / '_dev_tools/round90n_mpr_read_host')
    args = parser.parse_args()
    source_path = args.source.resolve()
    header_path = args.header.resolve()
    source = source_path.read_text('utf-8'); header = header_path.read_text('utf-8')
    signatures = ['MPR121::MPR121(', 'uint16_t MPR121::filteredData(', 'uint16_t MPR121::baselineData(',
                  'bool MPR121::readRegisters(', 'uint16_t MPR121::touched(',
                  'uint8_t MPR121::readRegister8(', 'uint16_t MPR121::readRegister16(']
    checked = 'readRegisterChecked' in header
    if not checked and 'readTouchStatus' in header:
        signatures += ['bool MPR121::readTouchStatus(']
    if checked:
        signatures += ['bool MPR121::readRegisterChecked(', 'bool MPR121::readTouchStatus(',
                       'bool MPR121::readFilteredData(', 'bool MPR121::readBaselineData(']
    header = re.sub(r'^#include.*$', '', header, flags=re.M)
    common = integration.PRELUDE.split('int i2c_write_stop_read(')[0]
    chunks = [common, HOST, header, '#define CHECKED_MPR_READS' if checked else '']
    manifest = {'kind': 'exact native MPR class/reads; fake I2C, no hardware', 'fragments': [],
                'checked_api_available': checked, 'source_sha256': hashlib.sha256(source_path.read_bytes()).hexdigest(),
                'header_sha256': hashlib.sha256(header_path.read_bytes()).hexdigest()}
    for signature in signatures:
        code, line = integration.extract_function(source, signature)
        chunks.append(f'\n#line {line} "{source_path.as_posix()}"\n' + code)
        manifest['fragments'].append({'signature': signature, 'line': line, 'sha256': hashlib.sha256(code.encode()).hexdigest()})
    chunks.append(CASES); args.output.mkdir(parents=True, exist_ok=True)
    generated = args.output / 'mpr_read.cpp'; executable = args.output / 'mpr_read.exe'
    generated.write_text('\n'.join(chunks), encoding='utf-8', newline='\n')
    command = [str(integration.CLANG), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-Wno-unused-variable', '-Wno-unused-private-field', '-ffreestanding', '-fno-builtin',
               '-fno-stack-protector', '-fuse-ld=lld', '-nostdlib',
               f'-I{integration.VARIANT / "include/share"}', f'-I{integration.VARIANT / "include/software"}',
               '-ID:/pico-sdk/src/common/pico_base_headers/include', str(generated),
               '-Wl,/entry:mprReadEntry', '-Wl,/subsystem:console', str(integration.KERNEL32), '-o', str(executable)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if compiled.returncode: raise SystemExit(compiled.stdout + compiled.stderr)
    manifest['compile_command'] = command; results = []
    for case in range(24):
        result = subprocess.run([str(executable), f'--case={case}'], capture_output=True, text=True, timeout=10)
        count = re.search(r'PASS checks=(\d+)', result.stdout)
        passed = result.returncode == 0 and count is not None
        results.append({'case': case, 'passed': passed, 'checks': int(count[1]) if count else 0,
                        'output': result.stdout + result.stderr})
        print(f'{"PASS" if passed else "FAIL"} {case}: {result.stdout.strip()}')
    manifest['results'] = results
    (args.output / 'results.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    total = sum(r['passed'] for r in results); print(f'MPR native reads: {total}/24; evidence: {args.output / "results.json"}')
    return 0 if total == 24 else 1


if __name__ == '__main__':
    raise SystemExit(main())
