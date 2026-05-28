#pragma once
// ELECROW CrowPanel 1.28" HMI ESP32-S3 Rotary Display — pin map.
// Source: Elecrow-RD/CrowPanel-1.28inch-HMI-ESP32-Rotary-Display GitHub examples
// (RotaryScreen_1_28_new.ino + Simple example/Encoder_code/Encoder_code.ino).

namespace HW {

// Rotary encoder (Bourns-style 2-bit quad + push)
constexpr int PIN_ENC_A   = 45;   // CLK
constexpr int PIN_ENC_B   = 42;   // DT
constexpr int PIN_ENC_BTN = 41;   // active-low, pull-up

// Display: GC9A01 round IPS, SPI (no MISO)
constexpr int PIN_DISP_SCLK = 10;
constexpr int PIN_DISP_MOSI = 11;
constexpr int PIN_DISP_DC   = 3;
constexpr int PIN_DISP_CS   = 9;
constexpr int PIN_DISP_RST  = 14;
constexpr int PIN_DISP_BL   = 46;   // backlight, PWM via ledc

// Touch: CST816D, dedicated I2C bus
constexpr int PIN_TOUCH_SDA = 6;
constexpr int PIN_TOUCH_SCL = 7;
constexpr int PIN_TOUCH_INT = 5;
constexpr int PIN_TOUCH_RST = 13;

// External expansion I2C (the JST connector — for future daughter-boards)
constexpr int PIN_EXT_I2C_SDA = 38;
constexpr int PIN_EXT_I2C_SCL = 39;

// Onboard indicators
constexpr int PIN_POWER_LED = 40;
constexpr int PIN_RGB_DATA  = 48;   // 5x WS2812 ring (named to avoid Arduino-ESP32 v3 PIN_RGB_LED macro collision)
constexpr int RGB_LED_COUNT = 5;

// Power-rail enable pins (Elecrow example asserts these HIGH in setup; they
// gate the touch / RGB subsystems on early board revs).
constexpr int PIN_PWR_EN_1 = 1;
constexpr int PIN_PWR_EN_2 = 2;

} // namespace HW
