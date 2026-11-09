// Minimal hobby-servo driver on the ESP32 LEDC peripheral (hardware PWM, so
// the pulse train keeps running while the CPU sleeps).
#pragma once

#include <Arduino.h>

class ServoDriver {
 public:
  void begin(uint8_t pin, uint8_t channel, uint16_t minUs, uint16_t maxUs) {
    channel_ = channel;
    minUs_ = minUs;
    maxUs_ = maxUs;
    ledcSetup(channel_, kFreqHz, kResolutionBits);
    ledcAttachPin(pin, channel_);
  }

  void write(int deg) {
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    const uint32_t us = minUs_ + (static_cast<uint32_t>(maxUs_ - minUs_) * deg) / 180u;
    const uint32_t duty = (us * ((1u << kResolutionBits) - 1u)) / kPeriodUs;
    ledcWrite(channel_, duty);
    angle_ = deg;
  }

  int angle() const { return angle_; }

 private:
  static constexpr uint32_t kFreqHz = 50;
  static constexpr uint32_t kPeriodUs = 1000000u / kFreqHz;
  static constexpr uint8_t kResolutionBits = 16;
  uint8_t channel_ = 0;
  uint16_t minUs_ = 500;
  uint16_t maxUs_ = 2400;
  int angle_ = -1;
};
