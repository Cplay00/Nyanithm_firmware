#include <app_link.h>
#include <button.h>
#include <production_mode.h>
#include <boot_mode.h>
#include <controller_config.h>
#include <hw_devices.h>
#include <nyanithm_shared.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <tusb.h>

bool hid_working = true;

void getStatus() {
    // round46e: config-mode I/O restored to stdio (round45- era mechanism).
    // Core1 keeps running tud_task() in config mode, which is exactly how the
    // pre-round45 firmware worked (maindev_loop on Core0 + stdio_cdc).
    printf("Nyanithm build " __DATE__ " " __TIME__ "\n");
    printf("hwVer: %d\n", ControllerConfig.hwVer);
    if (ControllerConfig.hwVer == 1 || ControllerConfig.hwVer == 3) {
        printf("original ToF distance (or error code):\n    tof0 = %d tof1 = %d tof2 = %d  tof3 = %d\n", heightDataOriginal[0], heightDataOriginal[1], heightDataOriginal[2], heightDataOriginal[3]);
        printf("using config in page %d\n", getConfigPage());
        for (int i = 0; i < 4; i++) {
            if (heightDataOriginal[i] > 3000 && heightDataOriginal[i] != 8190) {
                printf("*********************\n");
                printf("ToF sensor #%d warning!\n", i);
                printf("*********************\n");
            }
        }
    }
    if (ControllerConfig.hwVer == 2 || ControllerConfig.hwVer == 4) {
        printf("original ToF distance (or error code):\n    tof0 = %d tof1 = %d tof2 = %d  tof3 = %d tof4 = %d\n", heightDataOriginal[0], heightDataOriginal[1], heightDataOriginal[2],
               heightDataOriginal[3], heightDataOriginal[4]);
        printf("using config in page %d\n", getConfigPage());
        for (int i = 0; i < 5; i++) {
            if (heightDataOriginal[i] > 3000 && heightDataOriginal[i] != 8190) {
                printf("*********************\n");
                printf("ToF sensor #%d warning!\n", i);
                printf("*********************\n");
            }
        }
    }
}


void handleCommand() {
    // round46f: config-mode idle auto-exit. If no CDC command arrives for
    // 60s (e.g. ConfigApp closed/disconnected without sending CMD_EXIT),
    // reboot back to normal mode so the game input never stays locked out.
    uint32_t lastCmdMs = to_ms_since_boot(get_absolute_time());
    // round46g: command log for ConfigApp disconnect forensics.
    // Records every byte received while in config mode; host reads it back
    // with the diagnostic command 0xCE (ConfigApp never sends 0xCE).
    static uint8_t  cfgCmdLog[256];
    static uint16_t cfgCmdLogWr = 0;
    static uint16_t cfgCmdLogTotal = 0;
    while (true) {
        while (tud_cdc_available() == 0) {
            sleep_ms(1);
            updateInputState();
            if (to_ms_since_boot(get_absolute_time()) - lastCmdMs > 60000) {
                reboot();
            }
        }
        uint8_t cmd = getchar();
        lastCmdMs = to_ms_since_boot(get_absolute_time());
        cfgCmdLog[cfgCmdLogWr & 0xFF] = cmd;
        cfgCmdLogWr++;
        cfgCmdLogTotal++;
        if (cmd == CMD_DEV_DETECT) {
            putchar(CMD_DEV_DETECT);
        } else if (cmd == CMD_CFG_READ) {
            readConfig();
            uint8_t* ptr = (uint8_t*)&ControllerConfig;
            for (int i = 0; i < sizeof(controller_config); i++) {
                putchar(*ptr);
                ptr++;
            }
            printf("read...\n");
            printf("using config in page %d\n", getConfigPage());
        } else if (cmd == CMD_CFG_ERASE) {
            printf("erase...\n");
            eraseConfigSector();
        } else if (cmd == CMD_CFG_SET) {
            setConfig();
        } else if (cmd == CMD_CFG_SAVE) {
            printf("save...\n");
            saveConfig();
        } else if (cmd == CMD_GET_STATUS) {
            printf("get status...\n");
            getStatus();
        } else if (cmd == CMD_EXIT) {
            reboot();
        } else if (cmd == CMD_LOAD3116CONFIG) {
            uint8_t address = getchar();
            uint8_t cfg[128];
            for (int i = 0; i < 128; i++) {
                cfg[i] = getchar();
            }
            printf("programing 3116 chip\n");
            program_cy8cmbr3116_custom(address, cfg);
            printf("done\n");

        } else if (cmd == CMD_FLASHING) {
            boot_flashing();
        } else if (cmd == 0xCE) {
            // round46g: dump config-mode command log: [1B len][len bytes oldest->newest]
            uint16_t n = (cfgCmdLogTotal < 256) ? cfgCmdLogTotal : 256;
            if (n > 255) n = 255;
            putchar((uint8_t)n);
            uint16_t start = cfgCmdLogWr - n;
            for (uint16_t i = 0; i < n; i++) {
                putchar(cfgCmdLog[(start + i) & 0xFF]);
            }
        } else {
            printf("unknown command...\n");
        }
    }
}
