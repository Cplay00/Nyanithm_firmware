#!/usr/bin/env python3
"""Compile the production MBR3116 driver with a fake I2C transport and test it.

No serial device, Pico board, register configuration, or copied driver method is
used. The fake checks the exact STOP-read window and injects transfer failures
and torn snapshots into src/cy8cmbr3116.cpp itself.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


HARNESS = r"""
#ifdef NDEBUG
#undef NDEBUG // Zig release mode sets this; the harness must execute asserts.
#endif
#include <cy8cmbr3116.h>
#include <pico/error.h>
#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

struct Transfer {
    std::array<uint8_t, 35> bytes;
    int returned;
};
static std::vector<Transfer> transfers;
static size_t reads = 0;
static uint8_t expectedRegister = 0xB9;
static size_t expectedSize = 35;

int i2c_write(uint8_t, uint8_t, uint8_t*, size_t, bool) {
    assert(false && "difference reads must not write configuration");
    return -1;
}
int i2c_read(uint8_t, uint8_t, uint8_t*, size_t, bool) {
    assert(false && "difference reads must use the atomic STOP-read transport");
    return -1;
}
int i2c_write_stop_read(uint8_t port, uint8_t address, uint8_t reg,
                        uint8_t* data, size_t count) {
    assert(port == 1 && address == 0x41);
    assert(reg == expectedRegister && count == expectedSize);
    assert(reads < transfers.size() && "unbounded/unexpected retry");
    const Transfer& transfer = transfers[reads++];
    // A timeout can leave an arbitrary prefix in the receive buffer.
    size_t copied = transfer.returned < 0 ? 7 : (size_t)transfer.returned;
    if (copied > count) copied = count;
    std::memcpy(data, transfer.bytes.data(), copied);
    return transfer.returned;
}

static const std::array<uint16_t, 16> first = {
    0x1234, 0xFEDC, 0, 255, 1, 2, 16, 32,
    64, 128, 200, 201, 202, 203, 204, 205
};
static const std::array<uint16_t, 16> second = {
    0x4321, 0xCDEF, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18
};

static Transfer frame(const std::array<uint16_t, 16>& values,
                      uint8_t sync0, uint8_t sync1, int returned = 35) {
    Transfer transfer{};
    transfer.bytes[0] = sync0;
    for (size_t i = 0; i < values.size(); ++i) {
        transfer.bytes[1 + i * 2] = (uint8_t)values[i];
        transfer.bytes[2 + i * 2] = (uint8_t)(values[i] >> 8);
    }
    transfer.bytes[33] = 0xA5; // GPO_DATA is not a count or the ending sync.
    transfer.bytes[34] = sync1;
    transfer.returned = returned;
    return transfer;
}

static void prepare(std::initializer_list<Transfer> script) {
    transfers = script;
    reads = 0;
    expectedRegister = 0xB9;
    expectedSize = 35;
}
static void expectValues(const std::array<uint16_t, 18>& output,
                         const std::array<uint16_t, 16>& expected) {
    assert(output.front() == 0xBEEF && output.back() == 0xBEEF);
    for (size_t i = 0; i < expected.size(); ++i) assert(output[i + 1] == expected[i]);
}

int main() {
    CY8CMBR3116 chip(1, 0x41);
    std::array<uint16_t, 18> output;
    output.fill(0xBEEF);
    const auto unchanged = output;

    prepare({frame(first, 3, 3)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 1);
    expectValues(output, first);
    std::puts("PASS coherent burst, all channel offsets, little-endian and canaries");

    output = unchanged;
    prepare({frame(first, 15, 0), frame(second, 0, 0)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 2);
    expectValues(output, second);
    std::puts("PASS torn burst then valid retry, including counter wrap");

    output = unchanged;
    prepare({frame(first, 1, 2), frame(second, 2, 3)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 2 && output == unchanged);
    std::puts("PASS persistent torn bursts bounded to two attempts, no publication");

    output = unchanged;
    prepare({frame(first, 3, 3, 12)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 1 && output == unchanged);
    std::puts("PASS short transfer leaves whole output unchanged");

    output = unchanged;
    prepare({frame(first, 3, 3, PICO_ERROR_GENERIC), frame(second, 4, 4, PICO_ERROR_GENERIC),
             frame(first, 5, 5, PICO_ERROR_GENERIC)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 3 && output == unchanged);
    std::puts("PASS persistent generic errors bounded to three, partial buffer unpublished");

    prepare({frame(first, 3, 3, PICO_ERROR_GENERIC), frame(second, 4, 4)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 2);
    expectValues(output, second);
    std::puts("PASS first wake NACK then complete native burst");

    output = unchanged;
    prepare({frame(first, 3, 3, PICO_ERROR_GENERIC), frame(first, 3, 3, PICO_ERROR_GENERIC),
             frame(second, 4, 4)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 3);
    expectValues(output, second);
    std::puts("PASS two wake NACKs then complete native burst");

    output = unchanged;
    prepare({frame(first, 3, 3, PICO_ERROR_TIMEOUT)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 1 && output == unchanged);
    std::puts("PASS SDK timeout is not repeated or published");

    prepare({frame(first, 3, 3, PICO_ERROR_INVALID_ARG)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 1 && output == unchanged);
    std::puts("PASS invalid argument is not retried");

    prepare({frame(first, 3, 4), frame(first, 3, 3, PICO_ERROR_GENERIC), frame(second, 4, 4)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 3);
    expectValues(output, second);
    std::puts("PASS SYNC and transport retries preserve coherent publication");

    output = unchanged;
    prepare({frame(first, 3, 4), frame(second, 4, 4, 34)});
    assert(!chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 2 && output == unchanged);
    std::puts("PASS retry short transfer never publishes either prefix");

    prepare({});
    assert(!chip.readDifferenceCounts(nullptr));
    assert(reads == 0);
    std::puts("PASS null output rejected without bus access");

    const std::array<uint16_t, 16> zeros{};
    output = unchanged;
    prepare({frame(zeros, 0, 0), frame(zeros, 0, 0)});
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(chip.readDifferenceCounts(output.data() + 1));
    assert(reads == 2);
    expectValues(output, zeros);
    std::puts("PASS zero-valued and repeated coherent snapshots remain valid reads");
    Transfer status = frame(first, 0, 0, 2);
    status.bytes[0] = 0x81; status.bytes[1] = 0x04;
    prepare({frame(first, 0, 0, PICO_ERROR_GENERIC), status});
    expectedRegister = 0xAA; expectedSize = 2;
    std::array<uint8_t, 4> bits{0xBE, 0, 0, 0xEF};
    assert(chip.get_BUTTON_STAT(bits.data() + 1) == 0);
    assert(reads == 2 && bits[0] == 0xBE && bits[1] == 0x81 && bits[2] == 0x04 && bits[3] == 0xEF);
    std::puts("PASS native BUTTON_STAT uses bounded wake retry");
    std::puts("14/14 production driver tests passed");
}
"""

I2C_STUB = """#pragma once
#include <cstddef>
#include <cstdint>
int i2c_write(uint8_t, uint8_t, uint8_t*, size_t, bool);
int i2c_read(uint8_t, uint8_t, uint8_t*, size_t, bool);
int i2c_write_stop_read(uint8_t, uint8_t, uint8_t, uint8_t*, size_t);
"""


def find_compiler(explicit):
    if explicit:
        compiler = Path(explicit)
        if not compiler.is_file():
            raise SystemExit(f"Compiler does not exist: {compiler}")
        return [str(compiler), "c++"] if compiler.stem == "zig" else [str(compiler)]
    # This workspace has Zig with its own libc++/Windows SDK dependencies.
    zig = Path(r"D:\tools\zig-x86_64-windows-0.16.0\zig.exe")
    if zig.is_file():
        return [str(zig), "c++"]
    for name in ("clang++", "g++", "c++"):
        compiler = shutil.which(name)
        if compiler:
            return [compiler]
    raise SystemExit("No native C++ compiler found; specify --compiler.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="Path to a native g++, clang++, or Zig executable")
    parser.add_argument("--sdk", type=Path, default=Path(r"D:\pico-sdk"))
    args = parser.parse_args()
    variant = Path(__file__).resolve().parents[1]
    sdk_headers = args.sdk / "src/common/pico_base_headers/include"
    if not (sdk_headers / "pico/error.h").is_file():
        parser.error("Official Pico SDK error header is required")
    compiler = find_compiler(args.compiler)
    with tempfile.TemporaryDirectory(prefix="nyanithm_mbr_diff_") as directory:
        temporary = Path(directory)
        (temporary / "i2c_port.h").write_text(I2C_STUB, encoding="utf-8")
        (temporary / "stdint-gcc.h").write_text("#pragma once\n#include <cstdint>\n", encoding="utf-8")
        harness = temporary / "test.cpp"
        harness.write_text(HARNESS, encoding="utf-8")
        binary = temporary / ("test.exe" if os.name == "nt" else "test")
        command = compiler + [
            "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
            "-I", str(temporary), "-I", str(variant / "include" / "device"),
            "-I", str(sdk_headers),
            str(variant / "src" / "cy8cmbr3116.cpp"), str(harness), "-o", str(binary),
        ]
        build = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
        if build.returncode:
            # Zig may emit large diagnostics while building its bundled libc++.
            # Preserve the useful tail if compilation fails; tests never pass on
            # a compiler error or with assertions disabled in release mode.
            raise SystemExit((build.stdout + build.stderr)[-12000:])
        print(f"Compiled production driver: {variant / 'src' / 'cy8cmbr3116.cpp'}", flush=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
