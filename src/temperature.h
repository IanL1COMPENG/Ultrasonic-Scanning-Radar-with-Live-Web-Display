// Non-blocking DS18B20 reader. Starts a conversion, comes back when it is
// done, and smooths the result. Falls back to a default temperature when no
// sensor is present and keeps trying to detect one.
#pragma once

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

class TemperatureSensor {
 public:
  void begin(uint8_t pin);
  void service(uint32_t nowMs);  // call often; returns quickly

  float celsius() const { return tempC_; }
  bool valid() const { return valid_; }
  bool present() const { return present_; }

 private:
  bool detect();

  OneWire bus_;
  DallasTemperature dallas_;
  DeviceAddress addr_{};
  bool present_ = false;
  bool pending_ = false;
  bool valid_ = false;
  float tempC_ = 20.0f;
  uint32_t requestedAtMs_ = 0;
  uint32_t lastRequestMs_ = 0;
  uint32_t lastGoodMs_ = 0;
  uint32_t lastDetectMs_ = 0;
  uint16_t conversionMs_ = 375;
};
