// Hardware pin map and tuning constants.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Wi-Fi credentials live in include/secrets.h (git-ignored).
// Copy include/secrets.example.h to include/secrets.h and edit it.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""  // empty -> skip station mode and start the access point
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

namespace cfg {

// ---- Pins (ESP32 DevKit) ---------------------------------------------------
constexpr uint8_t PIN_TRIG = 5;      // HC-SR04 TRIG
constexpr uint8_t PIN_ECHO = 18;     // HC-SR04 ECHO (through 5V -> 3.3V divider!)
constexpr uint8_t PIN_SERVO = 13;    // SG90 / MG90S signal
constexpr uint8_t PIN_ONEWIRE = 4;   // DS18B20 data (4.7k pull-up to 3.3V)
constexpr uint8_t PIN_SDA = 21;      // SSD1306 OLED
constexpr uint8_t PIN_SCL = 22;
constexpr uint8_t OLED_ADDR = 0x3C;

// ---- Sweep -------------------------------------------------------------------
constexpr int SWEEP_MIN_DEG = 0;
constexpr int SWEEP_MAX_DEG = 180;
constexpr int SWEEP_STEP_DEG = 2;
constexpr uint32_t SCAN_PERIOD_MS = 50;  // 20 Hz: one filtered reading per step

// ---- Servo (LEDC PWM) ----------------------------------------------------------
constexpr uint8_t SERVO_LEDC_CHANNEL = 0;
constexpr uint16_t SERVO_MIN_US = 500;    // pulse width at 0 deg
constexpr uint16_t SERVO_MAX_US = 2400;   // pulse width at 180 deg
constexpr uint32_t SERVO_SETTLE_MIN_MS = 6;     // minimum settle after a 2 deg step
constexpr float SERVO_SETTLE_MS_PER_DEG = 2.5f; // ~0.1 s / 60 deg plus ringing

// ---- Ranging --------------------------------------------------------------------
constexpr size_t PINGS_PER_READING = 5;   // 5-sample median filter
constexpr size_t MIN_VALID_PINGS = 3;     // majority must see an echo
constexpr float MIN_RANGE_CM = 2.0f;
constexpr float MAX_RANGE_CM = 150.0f;    // specified accuracy window: 5..150 cm
constexpr float TIMEOUT_RANGE_FACTOR = 1.1f;  // listen a little past max range
constexpr uint32_t PING_GAP_US = 600;     // let the transducer stop ringing

// Linear calibration applied after the median filter:
//   reported = median * CAL_SCALE + CAL_OFFSET_CM
// Fit these with the dashboard's calibration panel and a tape measure.
constexpr float CAL_SCALE = 1.0f;
constexpr float CAL_OFFSET_CM = 0.0f;

// ---- Temperature ------------------------------------------------------------------
constexpr float DEFAULT_TEMP_C = 20.0f;   // used when no DS18B20 is detected
constexpr uint8_t DS18B20_RESOLUTION = 11;  // 0.125 C, 375 ms conversion
constexpr uint32_t TEMP_PERIOD_MS = 1000;
constexpr uint32_t TEMP_STALE_MS = 5000;

// ---- Networking ---------------------------------------------------------------------
constexpr const char* HOSTNAME = "radar";          // http://radar.local
constexpr const char* AP_SSID = "UltrasonicRadar";
constexpr const char* AP_PASS = "radar1234";       // >= 8 chars
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint16_t HTTP_PORT = 80;
constexpr uint16_t WS_PORT = 81;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr uint32_t OLED_PERIOD_MS = 100;           // 10 fps

// ---- FreeRTOS -------------------------------------------------------------------------
constexpr int SENSOR_CORE = 1;       // APP CPU: deterministic sensing
constexpr int NET_CORE = 0;          // PRO CPU: shares the core with the Wi-Fi stack
constexpr uint32_t SENSOR_STACK = 4096;
constexpr uint32_t NET_STACK = 8192;
constexpr uint32_t SENSOR_PRIORITY = 5;
constexpr uint32_t NET_PRIORITY = 2;
constexpr size_t SAMPLE_QUEUE_LEN = 64;

constexpr const char* FW_VERSION = "1.0.0";

}  // namespace cfg
