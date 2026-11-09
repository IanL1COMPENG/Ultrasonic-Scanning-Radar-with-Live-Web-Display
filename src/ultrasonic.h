// Interrupt-driven HC-SR04 driver.
//
// Instead of busy-waiting in pulseIn(), the echo pin raises a GPIO interrupt
// on both edges, timestamps them with the 1 us esp_timer and wakes the
// calling task with a direct-to-task notification. The sensing task sleeps
// while the sound is in flight.
#pragma once

#include <Arduino.h>

class Ultrasonic {
 public:
  void begin(uint8_t trigPin, uint8_t echoPin);

  // Fire one ping. Returns the echo pulse width in microseconds (round trip),
  // or 0 if no echo arrived within `timeoutUs` or the sensor was still busy.
  uint32_t ping(uint32_t timeoutUs);

 private:
  static void IRAM_ATTR onEchoEdge(void* arg);

  uint8_t trig_ = 0;
  uint8_t echo_ = 0;
  volatile bool armed_ = false;
  volatile int64_t riseUs_ = 0;
  volatile uint32_t widthUs_ = 0;
  volatile TaskHandle_t waiter_ = nullptr;
};
