/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#include <controller_config.h>
#include <app_link.h>
#include <mbr_distance_gate.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/stdio.h>
#include <tusb.h>

#define FLASH_STORAGE_START (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

controller_config defaultConfig{
    .magic = CONTROLLER_CONFIG_MAGIC,     //
    .cfgVer = CONTROLLER_CONFIG_VERSION,  //
    .hwVer = 1,                           //
    .cfg0 = 0x00,                         //
    .cfg1 = 0x00,                         //
    .cfg2 = 0x00,                         //
    .cfg3 = 0x00,                         //
    .th_touch = 6,                        // round32: was 4, caused MPR0 ELE3 permanent false touch (diff=4=threshold)
    .th_release = 4,                      // round32: was 3, increase hysteresis to prevent rapid touch/release cycling
    .debounce = 0b00010001,               // dt=1/dr=1, 1ms debounce at ESI=1ms (adjustable via ConfigApp)
    .airMax = 500,                        //
    .airMin = 200,                        //
    .heightOffset = { 0, 0, 0, 0, 0 },   //
    .lightLimit = 255,                    //
    .heightRangeCfg = 0,                  // 0 = runtime default (10mm)
    .thTouchKey = { 0 },                  // round64: per-key touch, 0 = inherit th_touch
    .thReleaseKey = { 0 },                // round64: per-key release, 0 = inherit th_release
    .mbrDistanceFlags = 0,                // strict MBR gate is explicitly opt-in
    .xorSum = 0,                          // 前127字节异或和
};

controller_config ControllerConfig;

uint8_t config_buf[sizeof(controller_config)];

uint8_t currentPage = 0;

// round51: silent integrity check for the flash read path. Must not printf:
// readConfig can run on Core0 while Core1 owns the USB stack (boot time).
static bool validateConfigQuiet(controller_config* config) {
    if (config->magic != CONTROLLER_CONFIG_MAGIC) return false;
    if (config->cfgVer != CONTROLLER_CONFIG_VERSION) return false;
    uint8_t* ptr = (uint8_t*)config;
    uint8_t sum = 0;
    for (int i = 0; i < (int)sizeof(controller_config) - 1; i++) {
        sum ^= *ptr++;
    }
    return sum == config->xorSum;
}

// round51: clamp loaded / host-supplied configs into safe ranges instead of
// accepting arbitrary values (the old hwVer gate was commented out entirely,
// so a corrupted hwVer/air range silently took effect).
static void sanitizeConfig(controller_config* config) {
    if (config->hwVer < 1 || config->hwVer > 4) config->hwVer = defaultConfig.hwVer;
    if (config->th_touch < 1) config->th_touch = 1;
    if (config->th_touch > 63) config->th_touch = 63;
    if (config->th_release < 1) config->th_release = 1;
    if (config->th_release > 63) config->th_release = 63;
    if (config->airMin >= config->airMax) {
        config->airMin = defaultConfig.airMin;
        config->airMax = defaultConfig.airMax;
    }
    // round55: new cfg2/cfg3 fields. cfg2: bit0 (LampArray) + bit1 (round68
    // experimental game-raw-slider) defined, mask the rest.
    // cfg3: clamp latency to INPUT_LATENCY_MAX_MS (15ms).
    config->cfg2 &= (CFG2_BIT_DISABLE_LAMP_ARRAY | CFG2_BIT_GAME_RAW_SLIDER);
    if (config->cfg3 > INPUT_LATENCY_MAX_MS) config->cfg3 = INPUT_LATENCY_MAX_MS;
    // round64: per-key threshold clamping. 0 = inherit global (preserved).
    // MPR121: same 1-63 scale as th_touch. MBR3116: byte range 1-255 (values
    // only tighten the legacy software verification; BUTTON_STAT is separate).
    bool useMbr = (config->cfg0 & CFG0_BIT_MBR3116) || config->hwVer >= 3;
    mbrDistanceSanitize(*config, useMbr);
    uint8_t maxKey = useMbr ? 255 : 63;
    for (int i = 0; i < 32; i++) {
        if (config->thTouchKey[i] > maxKey) config->thTouchKey[i] = maxKey;
        if (useMbr) {
            // release threshold is meaningless on MBR (no per-sensor register);
            // treat any non-zero value as invalid -> inherit global.
            config->thReleaseKey[i] = 0;
        } else if (config->thReleaseKey[i] > maxKey) {
            config->thReleaseKey[i] = maxKey;
        }
    }
}

static void recomputeXorSum(controller_config* config) {
    uint8_t* ptr = (uint8_t*)config;
    uint8_t sum = 0;
    for (int i = 0; i < (int)sizeof(controller_config) - 1; i++) {
        sum ^= *ptr++;
    }
    config->xorSum = sum;
}

void saveConfigSafe(void* param) {
    // Append after every occupied page, including interrupted/invalid writes.
    // NOR programming cannot restore 0 bits to 1 without a sector erase.
    const uint8_t* sector = (const uint8_t*)(XIP_BASE + FLASH_STORAGE_START);
    unsigned next = 0;
    for (unsigned page = 0; page < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; ++page) {
        for (unsigned byte = 0; byte < FLASH_PAGE_SIZE; ++byte) {
            if (sector[page * FLASH_PAGE_SIZE + byte] != 0xff) {
                next = page + 1;
                break;
            }
        }
    }
    if (next == FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE) {
        flash_range_erase(FLASH_STORAGE_START, FLASH_SECTOR_SIZE);
        next = 0;
    }
    // The wire config stays 128B; the SDK requires whole 256B flash pages.
    uint8_t pageData[FLASH_PAGE_SIZE];
    memset(pageData, 0xff, sizeof(pageData));
    memcpy(pageData, &ControllerConfig, sizeof(ControllerConfig));
    flash_range_program(FLASH_STORAGE_START + (next * FLASH_PAGE_SIZE),
                        pageData, sizeof(pageData));
    currentPage = next;
}

void saveConfig() {
    int result = flash_safe_execute(saveConfigSafe, nullptr, 10);
    printf("saveConfig result: %d, currentPage = %d\n", result, currentPage);
}

void eraseConfigSectorSafe(void* param) {
    flash_range_erase(FLASH_STORAGE_START, FLASH_SECTOR_SIZE);
    currentPage = 0xff;  // round51: next save starts at page 0 (was page 1)
}

void eraseConfigSector() {
    int result = flash_safe_execute(eraseConfigSectorSafe, nullptr, 10);
    printf("eraseConfig result: %d\n", result);
}

void readConfigSafe(void* param) {
    uint8_t* addr = (uint8_t*)(XIP_BASE + FLASH_STORAGE_START);
    bool found = false;
    controller_config* target = nullptr;
    for (int page = 0; page < 16; page++) {
        controller_config* config = (controller_config*)addr;
        // printf("magic = %d\n", config->magic);
        // 检查是否为存储的配置
        // round51: full integrity check (magic + cfgVer + xorSum) before
        // accepting a page; a torn write (power loss mid-program) falls back
        // to the previous valid page instead of loading bit-rotten values.
        if (validateConfigQuiet(config)) {
            found = true;
            target = config;
            currentPage = page;
        }
        addr += FLASH_PAGE_SIZE;
    }
    if (found) {
        // printf("found config in page %d\n", currentPage);
        memcpy(&ControllerConfig, target, sizeof(controller_config));
        // round51: clamp fields to safe ranges and refresh the checksum so
        // the next save persists a fully valid image.
        sanitizeConfig(&ControllerConfig);
        recomputeXorSum(&ControllerConfig);
    } else {
        // printf("config not found\n");
        currentPage = 0xff;
        recomputeXorSum(&defaultConfig);
        memcpy(&ControllerConfig, &defaultConfig, sizeof(controller_config));
        // printf("using default config\n");
        saveConfigSafe(nullptr);
    }
}

void readConfig() {
    flash_safe_execute(readConfigSafe, nullptr, 10);
}

uint8_t getConfigPage() {
    return currentPage;
}

void setConfig() {
    // Read a complete payload through TinyUSB with a shared deadline and
    // watchdog/session handling. Partial transfers never change configuration.
    if (!readCdcPayload(config_buf, sizeof(controller_config), 1000)) return;
    const int count = sizeof(controller_config);
    if (checkConfigBuf((controller_config*)config_buf)) {
        printf("read %d byte. check ok. config changed.\n", count);
        memcpy(&ControllerConfig, config_buf, sizeof(controller_config));
        // round51: sanitize host-supplied config too (clamped values are
        // saved with a refreshed xorSum on the next CFG_SAVE).
        sanitizeConfig(&ControllerConfig);
        recomputeXorSum(&ControllerConfig);
    } else {
        printf("read %d byte. illegal config.\nconfig will not be changed\n", count);
    }
}

/**
 * 检查配置兼容性，校验数据完整性
 */
bool checkConfigBuf(controller_config* config) {
    if (config->magic != CONTROLLER_CONFIG_MAGIC) {
        printf("magic number wrong\n");
        return false;
    }
    // if (config->hwVer != CONTROLLER_HARDWARE_VERSION) {
    //     printf("hardware version wrong\n");
    //     printf("should be %d t read %d", CONTROLLER_HARDWARE_VERSION, config->hwVer);
    //     return false;
    // }
    if (config->cfgVer != CONTROLLER_CONFIG_VERSION) {
        printf("config version wrong\n");
        printf("should be %d but read %d\n", CONTROLLER_CONFIG_VERSION, config->cfgVer);
        return false;
    }
    uint8_t* ptr = (uint8_t*)config;
    uint8_t sum = 0x00;
    for (int i = 0; i < sizeof(controller_config) - 1; i++) {
        sum ^= *ptr;
        ptr++;
    }
    if (sum != config->xorSum) {
        printf("xorSum wrong\n");
        return false;
    }
    return true;
}
