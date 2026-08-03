
#include <app_link.h>
#include <boot_mode.h>
#include <button.h>
#include <chuni_io.h>
#include <controller_config.h>
#include <gpio_def.h>
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include <hw_check.h>
#include <hw_devices.h>
#include <i2c_port.h>
#include <pico/flash.h>
#include <production_mode.h>
#include <rainbow.h>

void reboot() {
    // round46h: delay_ms=0 -> _watchdog_enable(0) sets WATCHDOG_CTRL_TRIGGER,
    // an IMMEDIATE reset. The old watchdog_reboot(0,0,10) armed a 10ms window
    // that was continuously re-armed by watchdog_update() inside
    // updateInputState() (added round46b), so CMD_EXIT / config-mode idle
    // auto-exit never actually reset the device.
    watchdog_reboot(0, 0, 0);
}

void boot_switch() {
    initButtons();
    sleep_ms(10);

    if (getButtonState(BUTTON_PUSH)) {
        boot_appLinkMode();
    } else if (getButtonState(BUTTON_UP)) {
        boot_productionMode();
        // boot_flashing();
    } else if (getButtonState(BUTTON_DOWN)) {
        boot_otherModes();
    } else {
        boot_normalMode();
    }
}

void boot_productionMode() {
    hid_working = false;
    game_connected = true;
    initI2CBus(0, GPIO_I2C_0_SDA, GPIO_I2C_0_SCL, BR200K);
    RGB_LED.fill(WS2812::RGB(0xff, 0xff, 0xff));
    RGB_LED.show();

    productionMode();

}

void boot_appLinkMode() {
    RGB_LED.fill(WS2812::RGB(0x00, 0x0f, 0x00));
    RGB_LED.show();

    hid_working = false;

    sleep_ms(10);
    readConfig();
    initHwDevices();
    sleep_ms(10);
    handleCommand();

    reboot();
}

void boot_normalMode() {
    RGB_LED.fill(WS2812::RGB(0x0f, 0x0f, 0x0f));
    RGB_LED.show();

    sleep_ms(10);
    readConfig();

    sleep_ms(10);
    initHwDevices();

    sleep_ms(10);
    chekcHardwareState();

    sleep_ms(10);
    while (true) {
        updateInputState();
        // CDC command response moved to Core1 (cdc_respond); Core0 focuses on
        // sensor scanning to eliminate input jitter at 120fps.
        if (pending_config_mode) {
            pending_config_mode = false;
            handleCommand();  // flash_safe_execute context lives on Core0
        }
        // NOTE 1: do NOT call update_rainbow_frame() here. When game_connected=false
        // the LampArray driver (lamp_array_apply on Core1) owns the WS2812 strip; a
        // concurrent Core0 show() corrupts the WS2812 bitstream (flicker).
        // NOTE 2: do NOT sleep here when !game_connected. The game reads slider input
        // via the HID keyboard path (touchData[4], enabled by
        // CFG1_BIT_ENABLE_SLIDER_INPUT_AS_KEYBOARD) -- there is NO CDC activity in
        // that mode, so game_connected stays false. Sleeping here throttles
        // updateInputState() to ~100Hz, so touchData[4] lags the real touch state by
        // up to 10ms; during fast slider swipes the 2ms HID report keeps re-sending
        // stale "pressed" bits after the finger leaves -> perceptible sticky keys,
        // even though the CDC panel (which reads touchData32 with game_connected=true,
        // no sleep) looks fine. Keep this loop at full speed so touchData[4] stays as
        // fresh as touchData32.
    }
}

void start_4kMode();
void start_6kMode();

void boot_otherModes() {
    readConfig();

    sleep_ms(10);
    initHwDevices();

    sleep_ms(10);
    chekcHardwareState();

    sleep_ms(10);
    hid_working = false;
    RGB_LED.fill(WS2812::RGB(0, 0, 0));
    if (g_lampCount == 16) {
        for (int j = 0; j < 4; j++)
            RGB_LED.setPixelColor(15 - j, WS2812::RGB(0, 0, ControllerConfig.lightLimit));
        for (int j = 0; j < 4; j++)
            RGB_LED.setPixelColor(11 - j, WS2812::RGB(0, ControllerConfig.lightLimit, 0));
    } else {
        for (int j = 0; j < 7; j++)
            RGB_LED.setPixelColor(30 - j, WS2812::RGB(0, 0, ControllerConfig.lightLimit));
        for (int j = 0; j < 7; j++)
            RGB_LED.setPixelColor(22 - j, WS2812::RGB(0, ControllerConfig.lightLimit, 0));
    }
    // // F
    //     for (int j = 0; j < 7; j++)
    //         RGB_LED.setPixelColor(14 - j, WS2812::RGB(255, 255, 0));
    // // D
    //     for (int j = 0; j < 7; j++)
    //         RGB_LED.setPixelColor(6 - j, WS2812::RGB(255, 255, 0));

    RGB_LED.show();
    while (true) {
        updateInputState();
        updateTouchData4k();
        if (touchData4k[0]) {
            start_4kMode();
        }
        if (touchData4k[1]) {
            start_6kMode();
        }
    }
}

void boot_flashing() {
    flash_safe_execute([](void* p) { flash_range_erase(0, 4096); }, nullptr, 100);
    reboot();
}
