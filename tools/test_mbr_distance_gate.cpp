/* Host tests against the real firmware header; no Pico SDK or device needed.
 * Portable build: c++ -std=c++11 -Iinclude/share -Iinclude/software
 *   tools/test_mbr_distance_gate.cpp -o test_mbr_distance_gate
 * On Windows without a C runtime development kit, define
 * MBR_GATE_TEST_FREESTANDING_WINDOWS and link kernel32 with entry
 * mbrGateTestEntry. This only changes the host test output/exit wrapper.
 */

#include <mbr_distance_gate.h>

#if defined(MBR_GATE_TEST_FREESTANDING_WINDOWS)
extern "C" __declspec(dllimport) void* __stdcall GetStdHandle(unsigned long);
extern "C" __declspec(dllimport) int __stdcall WriteFile(
    void*, const void*, unsigned long, unsigned long*, void*);
extern "C" __declspec(dllimport) void __stdcall ExitProcess(unsigned int);

// Clang lowers aggregate initialization/copy to these runtime operations even
// in freestanding mode. Keep this small host wrapper independent of an MSVC
// runtime development kit; volatile prevents lowering the loops recursively.
extern "C" void* memset(void* destination, int value, size_t count) {
    volatile unsigned char* out = static_cast<unsigned char*>(destination);
    for (size_t i = 0; i < count; i++) out[i] = static_cast<unsigned char>(value);
    return destination;
}
extern "C" void* memcpy(void* destination, const void* source, size_t count) {
    volatile unsigned char* out = static_cast<unsigned char*>(destination);
    const volatile unsigned char* in = static_cast<const unsigned char*>(source);
    for (size_t i = 0; i < count; i++) out[i] = in[i];
    return destination;
}

static void printLine(const char* text) {
    unsigned long length = 0;
    while (text[length]) length++;
    unsigned long written = 0;
    WriteFile(GetStdHandle(static_cast<unsigned long>(-11)), text, length, &written, nullptr);
    WriteFile(GetStdHandle(static_cast<unsigned long>(-11)), "\n", 1, &written, nullptr);
}
#else
#include <stdio.h>
static void printLine(const char* text) { puts(text); }
#endif

static unsigned int failures = 0;
static unsigned int checks = 0;

static void check(bool condition, const char* label) {
    checks++;
    if (!condition) {
        failures++;
        if (failures <= 10) printLine(label);
    }
}

static controller_config profile(uint8_t zero = 100, uint8_t full = 200,
                                 uint8_t on = 80, uint8_t off = 50) {
    controller_config config = {};
    config.mbrDistanceMagic = MBR_DISTANCE_PROFILE_MAGIC;
    config.mbrDistanceVersion = MBR_DISTANCE_PROFILE_VERSION;
    config.mbrDistanceFlags = MBR_DISTANCE_FLAG_ENABLED;
    config.mbrDistanceZero = zero;
    config.mbrDistanceFull = full;
    config.mbrDistanceOnPercent = on;
    config.mbrDistanceOffPercent = off;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    return config;
}

static void rejected(controller_config config, const char* label) {
    check(!mbrDistanceProfileValid(config), label);
    check(!mbrDistanceGateEnabled(config), label);
    MbrDistanceGateState state;
    state.active = true;
    check(!mbrDistanceGateUpdate(config, 255, true, true, state) && !state.active, label);
    mbrDistanceSanitize(config, true);
    check(config.mbrDistanceFlags == 0 && !mbrDistanceGateEnabled(config), label);
}

static int runTests() {
    check(sizeof(controller_config) == 128, "FAIL: config length changed");
    check(offsetof(controller_config, mbrTouchGate) == 90, "FAIL: legacy gate moved");
    check(offsetof(controller_config, mbrDistanceMagic) == 91, "FAIL: profile moved");
    check(offsetof(controller_config, mbrDistanceCheck) == 98, "FAIL: profile check moved");
    check(offsetof(controller_config, xorSum) == 127, "FAIL: blob checksum moved");

    controller_config legacy = {};
    rejected(legacy, "FAIL: old zero-filled config enabled");
    legacy.mbrDistanceFlags = 1;
    rejected(legacy, "FAIL: flags alone enabled a legacy profile");
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&legacy);
    for (unsigned int i = 91; i < 127; i++) bytes[i] = 255;
    rejected(legacy, "FAIL: erased/random reserved bytes enabled");

    controller_config config = profile();
    check(mbrDistanceProfileValid(config) && mbrDistanceGateEnabled(config), "FAIL: valid opt-in rejected");
    config.mbrDistanceMagic ^= 1;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    rejected(config, "FAIL: unknown signature accepted");
    config = profile();
    config.mbrDistanceVersion++;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    rejected(config, "FAIL: unknown version accepted");
    config = profile();
    config.mbrDistanceFlags |= 2;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    rejected(config, "FAIL: unknown flags accepted");
    config = profile();
    config.mbrDistanceCheck ^= 1;
    rejected(config, "FAIL: bad profile check accepted");
    rejected(profile(100, 100), "FAIL: equal zero/full accepted");
    rejected(profile(200, 100), "FAIL: reversed zero/full accepted");
    rejected(profile(100, 200, 0, 0), "FAIL: zero ON accepted");
    rejected(profile(100, 200, 101, 50), "FAIL: ON above 100 accepted");
    rejected(profile(100, 200, 80, 80), "FAIL: equal ON/OFF accepted");
    rejected(profile(100, 200, 80, 81), "FAIL: reversed ON/OFF accepted");
    rejected(profile(100, 200, 80, 255), "FAIL: OFF above 100 accepted");

    config = profile();
    mbrDistanceSanitize(config, true);
    check(mbrDistanceGateEnabled(config), "FAIL: sanitize changed a valid MBR opt-in");
    mbrDistanceSanitize(config, false);
    check(mbrDistanceProfileValid(config) && !mbrDistanceGateEnabled(config), "FAIL: MPR opt-in survived sanitize");
    config = profile();
    config.mbrDistanceFlags = 0;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    check(mbrDistanceProfileValid(config) && !mbrDistanceGateEnabled(config), "FAIL: disabled valid profile invalid");

    config = profile();
    check(mbrDistanceNormalize(config, 0) == 0, "FAIL: below zero mapping");
    check(mbrDistanceNormalize(config, 100) == 0, "FAIL: zero mapping");
    check(mbrDistanceNormalize(config, 150) == 127, "FAIL: midpoint floor mapping");
    check(mbrDistanceNormalize(config, 200) == 255, "FAIL: full mapping");
    check(mbrDistanceNormalize(config, 65535) == 255, "FAIL: input above 255 wrapped");
    check(mbrDistanceOnCount(config) == 180, "FAIL: ON count");
    check(mbrDistanceOffCount(config) == 150, "FAIL: OFF count");
    MbrDistanceGateState state;
    check(!mbrDistanceGateUpdate(config, 179, true, true, state), "FAIL: entered below ON");
    check(mbrDistanceGateUpdate(config, 180, true, true, state), "FAIL: exact ON rejected");
    check(mbrDistanceGateUpdate(config, 151, true, true, state), "FAIL: hysteresis hold rejected");
    check(!mbrDistanceGateUpdate(config, 150, true, true, state), "FAIL: exact OFF held");
    check(!mbrDistanceGateUpdate(config, 170, true, true, state), "FAIL: state re-entered below ON");
    check(mbrDistanceGateUpdate(config, 255, true, true, state), "FAIL: firm signal rejected");
    check(!mbrDistanceGateUpdate(config, 255, false, true, state), "FAIL: invalid sample held active");
    check(!state.active, "FAIL: invalid sample did not clear state");
    check(mbrDistanceGateUpdate(config, 255, true, true, state), "FAIL: recovery after invalid sample");
    check(!mbrDistanceGateUpdate(config, 256, true, true, state) && !state.active,
          "FAIL: out-of-range button sample accepted");
    check(!mbrDistanceGateUpdate(config, 65535, true, true, state),
          "FAIL: corrupt/proximity scale treated as full");
    check(mbrDistanceGateUpdate(config, 255, true, true, state), "FAIL: recovery after corrupt sample");
    check(!mbrDistanceGateUpdate(config, 255, true, false, state), "FAIL: native OFF held active");
    check(!state.active, "FAIL: native OFF did not clear state");
    state.active = true;
    config.mbrDistanceFlags = 0;
    config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    check(!mbrDistanceGateUpdate(config, 255, true, true, state) && !state.active, "FAIL: disabled state not reset");

    // Exhaust every Z/F pair and percentage, including one-count spans and
    // extreme ON/OFF values. Quantization must never make OFF >= ON or assert
    // at/below the configured zero point.
    for (unsigned int zero = 0; zero < 255; zero++) {
        for (unsigned int full = zero + 1; full <= 255; full++) {
            for (unsigned int on = 1; on <= 100; on++) {
                config = profile(static_cast<uint8_t>(zero), static_cast<uint8_t>(full),
                                 static_cast<uint8_t>(on), static_cast<uint8_t>(on - 1));
                unsigned int onCount = mbrDistanceOnCount(config);
                unsigned int offCount = mbrDistanceOffCount(config);
                check(onCount > zero && onCount <= full, "FAIL: exhaustive ON boundary");
                check(offCount >= zero && offCount < onCount, "FAIL: exhaustive hysteresis collapsed");
                state.active = false;
                check(!mbrDistanceGateUpdate(config, static_cast<uint16_t>(zero), true, true, state),
                      "FAIL: exhaustive zero point triggered");
                check(mbrDistanceGateUpdate(config, static_cast<uint16_t>(onCount), true, true, state),
                      "FAIL: exhaustive exact ON rejected");
                check(!mbrDistanceGateUpdate(config, static_cast<uint16_t>(offCount), true, true, state),
                      "FAIL: exhaustive exact OFF held");
            }
        }
    }

    printLine("Real mbr_distance_gate.h tests: layout, legacy/invalid rejection, sanitize,");
    printLine("mapping, held-touch release, I2C-invalid/native-OFF reset, all Z/F/ON boundaries.");
    printLine(failures == 0 ? "PASS (16,320,000 exhaustive checks plus focused cases)" : "FAIL");
    return failures == 0 ? 0 : 1;
}

#if defined(MBR_GATE_TEST_FREESTANDING_WINDOWS)
extern "C" void mbrGateTestEntry() { ExitProcess(static_cast<unsigned int>(runTests())); }
#else
int main() { return runTests(); }
#endif
