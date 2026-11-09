// Radar-style rendering on a 128x64 SSD1306 I2C OLED.
#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class OledView {
 public:
  bool begin(uint8_t sda, uint8_t scl, uint8_t addr);
  bool ok() const { return ok_; }

  void message(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);

  // dist: 181 entries indexed by angle, -1 = no echo.
  void render(const float* dist, int sweepAngle, float distCm, float tempC, bool tempValid,
              const char* footer);

 private:
  Adafruit_SSD1306 d_{128, 64, &Wire, -1, 400000UL, 400000UL};
  bool ok_ = false;
};
