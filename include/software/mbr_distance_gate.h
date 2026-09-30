/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __MBR_DISTANCE_GATE_H__
#define __MBR_DISTANCE_GATE_H__

#include <nyanithm_shared.h>

// Pure signal-domain helpers: no I2C, USB, clocks, or persistence. The gate
// intersects the chip's native BUTTON_STAT and cannot assert an absent touch.
inline uint8_t mbrDistanceProfileCheck(const controller_config& config) {
    return MBR_DISTANCE_PROFILE_CHECK_SEED ^ config.mbrDistanceMagic ^
           config.mbrDistanceVersion ^ config.mbrDistanceFlags ^
           config.mbrDistanceZero ^ config.mbrDistanceFull ^
           config.mbrDistanceOnPercent ^ config.mbrDistanceOffPercent;
}

inline bool mbrDistanceProfileValid(const controller_config& config) {
    return config.mbrDistanceMagic == MBR_DISTANCE_PROFILE_MAGIC &&
           config.mbrDistanceVersion == MBR_DISTANCE_PROFILE_VERSION &&
           (config.mbrDistanceFlags & ~MBR_DISTANCE_FLAG_ENABLED) == 0 &&
           config.mbrDistanceCheck == mbrDistanceProfileCheck(config) &&
           config.mbrDistanceFull > config.mbrDistanceZero &&
           config.mbrDistanceOnPercent >= 1 &&
           config.mbrDistanceOnPercent <= 100 &&
           config.mbrDistanceOffPercent < config.mbrDistanceOnPercent;
}

inline bool mbrDistanceGateEnabled(const controller_config& config) {
    return mbrDistanceProfileValid(config) &&
           (config.mbrDistanceFlags & MBR_DISTANCE_FLAG_ENABLED) != 0;
}

// Never clamp malformed points into an enabled profile. A separate profile
// signature, version, and check prevent a lone legacy reserved flags byte
// from enabling the gate. Non-MBR configurations always clear the opt-in.
inline void mbrDistanceSanitize(controller_config& config, bool useMbr) {
    if (!mbrDistanceProfileValid(config)) {
        config.mbrDistanceFlags = 0;
        config.mbrDistanceCheck = 0;
    } else if (!useMbr) {
        config.mbrDistanceFlags = 0;
        config.mbrDistanceCheck = mbrDistanceProfileCheck(config);
    }
}

inline uint8_t mbrDistanceNormalize(const controller_config& config, uint16_t diff) {
    if (!mbrDistanceProfileValid(config) || diff <= config.mbrDistanceZero) return 0;
    if (diff >= config.mbrDistanceFull) return 255;
    uint16_t span = config.mbrDistanceFull - config.mbrDistanceZero;
    return static_cast<uint8_t>((diff - config.mbrDistanceZero) * 255u / span);
}

// Count-space boundaries avoid a second rounding through the 0..255 display
// scale. ON rounds up; OFF rounds down, so even a one-count span has hysteresis.
inline uint16_t mbrDistanceOnCount(const controller_config& config) {
    if (!mbrDistanceProfileValid(config)) return 256;
    uint16_t span = config.mbrDistanceFull - config.mbrDistanceZero;
    return config.mbrDistanceZero + (span * config.mbrDistanceOnPercent + 99u) / 100u;
}

inline uint16_t mbrDistanceOffCount(const controller_config& config) {
    if (!mbrDistanceProfileValid(config)) return 255;
    uint16_t span = config.mbrDistanceFull - config.mbrDistanceZero;
    return config.mbrDistanceZero + span * config.mbrDistanceOffPercent / 100u;
}

struct MbrDistanceGateState {
    bool active = false;
};

// The caller uses its legacy pipeline when the profile is disabled. While
// enabled, every valid sensor sample must update this state, including held
// touches. Invalid samples and native OFF clear immediately; callers must also
// clear any downstream confirmation/stretch/sticky state on a strict rejection.
inline bool mbrDistanceGateUpdate(const controller_config& config, uint16_t diff,
                                  bool sampleValid, bool hardwareTouched,
                                  MbrDistanceGateState& state) {
    // Button difference counts are normalized to 0..255 by the chip. A larger
    // value is not a valid full-scale button sample (it could be another mode
    // or corrupt transport data); the display mapper may clamp, this gate may not.
    if (!mbrDistanceGateEnabled(config) || !sampleValid || !hardwareTouched || diff > 255) {
        state.active = false;
    } else if (state.active) {
        if (diff <= mbrDistanceOffCount(config)) state.active = false;
    } else if (diff >= mbrDistanceOnCount(config)) {
        state.active = true;
    }
    return state.active;
}

#endif
