#include "temperature.h"

#include <radar_core.h>

#include "config.h"

void TemperatureSensor::begin(uint8_t pin) {
  tempC_ = cfg::DEFAULT_TEMP_C;
  bus_.begin(pin);
  dallas_.setOneWire(&bus_);
  dallas_.begin();
  detect();
}

bool TemperatureSensor::detect() {
  lastDetectMs_ = millis();
  dallas_.begin();
  present_ = dallas_.getAddress(addr_, 0);
  if (present_) {
    dallas_.setResolution(addr_, cfg::DS18B20_RESOLUTION);
    dallas_.setWaitForConversion(false);  // never block the caller
    conversionMs_ = dallas_.millisToWaitForConversion(cfg::DS18B20_RESOLUTION);
    pending_ = false;
    Serial.println(F("[temp] DS18B20 detected"));
  }
  return present_;
}

void TemperatureSensor::service(uint32_t nowMs) {
  if (!present_) {
    valid_ = false;
    if (nowMs - lastDetectMs_ > 5000) detect();
    return;
  }

  if (!pending_) {
    if (nowMs - lastRequestMs_ >= cfg::TEMP_PERIOD_MS) {
      dallas_.requestTemperaturesByAddress(addr_);
      pending_ = true;
      requestedAtMs_ = lastRequestMs_ = nowMs;
    }
  } else if (nowMs - requestedAtMs_ >= conversionMs_ + 10u) {
    pending_ = false;
    const float t = dallas_.getTempC(addr_);
    if (radar::isPlausibleTemp(t)) {
      // Light exponential smoothing: the air temperature changes slowly,
      // the sensor's LSB jitter should not move the range readings.
      tempC_ = valid_ ? tempC_ + 0.3f * (t - tempC_) : t;
      valid_ = true;
      lastGoodMs_ = nowMs;
    } else if (t == DEVICE_DISCONNECTED_C) {
      present_ = false;
      Serial.println(F("[temp] DS18B20 lost"));
    }
  }

  if (valid_ && nowMs - lastGoodMs_ > cfg::TEMP_STALE_MS) valid_ = false;
}
