// Ultrasonic Scanning Radar with Live Web Display
//
//   core 1  sensorTask  servo sweep + HC-SR04 pings + median filter  (20 Hz)
//   core 0  netTask     Wi-Fi, HTTP dashboard, WebSockets, OLED, DS18B20
//
// The two tasks only share a FreeRTOS queue (samples) and a few atomics
// (mode, hold angle, temperature, counters) - see app_state.h.

#include <Arduino.h>

#include "app_state.h"
#include "config.h"
#include "net_task.h"
#include "sensor_task.h"

SharedState g_state;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[radar] Ultrasonic Scanning Radar v%s\n", cfg::FW_VERSION);

  g_state.samples = xQueueCreate(cfg::SAMPLE_QUEUE_LEN, sizeof(RadarSample));
  configASSERT(g_state.samples);

  xTaskCreatePinnedToCore(sensorTask, "sensor", cfg::SENSOR_STACK, nullptr, cfg::SENSOR_PRIORITY,
                          nullptr, cfg::SENSOR_CORE);
  xTaskCreatePinnedToCore(netTask, "net", cfg::NET_STACK, nullptr, cfg::NET_PRIORITY, nullptr,
                          cfg::NET_CORE);
}

void loop() {
  // Everything runs in the two pinned tasks; free the Arduino loop task.
  vTaskDelete(nullptr);
}
