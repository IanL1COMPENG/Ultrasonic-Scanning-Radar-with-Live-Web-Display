// RadarCore: hardware-independent math for the ultrasonic radar.
// Header-only so the exact same code runs on the ESP32 and in host unit tests.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace radar {

// ---------------------------------------------------------------------------
// Speed of sound compensation
// ---------------------------------------------------------------------------

// Speed of sound in dry air (m/s) as a function of temperature (deg C):
// ideal-gas law, c = 331.3 * sqrt(1 + T / 273.15).  343.2 m/s at 20 C.
// (The popular 331.3 + 0.606*T line drifts 0.2 % high by 40 C; the ESP32 has
// a hardware FPU so the exact form costs nothing.)
inline float speedOfSound(float tempC) { return 331.3f * sqrtf(1.0f + tempC / 273.15f); }

// Convert an HC-SR04 echo pulse width (round-trip time, microseconds) to a
// one-way distance in centimetres.  m/s * us = 1e-6 m = 1e-4 cm.
inline float echoToCm(uint32_t echoUs, float tempC) {
  return static_cast<float>(echoUs) * speedOfSound(tempC) * 1e-4f * 0.5f;
}

// Inverse of echoToCm: the round-trip time an echo from `cm` would take.
// Used to size the echo timeout for the configured maximum range.
inline uint32_t cmToEchoUs(float cm, float tempC) {
  return static_cast<uint32_t>(cm * 2.0f / (speedOfSound(tempC) * 1e-4f) + 0.5f);
}

// DS18B20 returns 85.0 on power-on reset and -127 when disconnected.
constexpr bool isPlausibleTemp(float tempC) {
  return tempC > -55.0f && tempC < 125.0f && tempC != 85.0f;
}

// ---------------------------------------------------------------------------
// Median filter
// ---------------------------------------------------------------------------

constexpr size_t kMaxMedianSamples = 15;

// Median of the valid (> 0) samples.  Samples <= 0 or NaN mark a missed echo.
// Returns -1 when fewer than `minValid` samples are valid, so a reading is
// only reported when a majority of pings agree that something is there.
inline float median(const float* samples, size_t count, size_t minValid) {
  float buf[kMaxMedianSamples];
  size_t n = 0;
  for (size_t i = 0; i < count && n < kMaxMedianSamples; ++i) {
    if (samples[i] > 0.0f) buf[n++] = samples[i];  // also rejects NaN
  }
  if (n == 0 || n < minValid) return -1.0f;

  // Insertion sort: N <= 15, branch-light, no heap.
  for (size_t i = 1; i < n; ++i) {
    const float key = buf[i];
    size_t j = i;
    while (j > 0 && buf[j - 1] > key) {
      buf[j] = buf[j - 1];
      --j;
    }
    buf[j] = key;
  }
  return (n & 1u) ? buf[n / 2] : 0.5f * (buf[n / 2 - 1] + buf[n / 2]);
}

// ---------------------------------------------------------------------------
// Sweep controller: bounces between minDeg and maxDeg in `step` increments.
//   0, 2, 4 ... 178, 180, 178 ... 2, 0, 2 ...
// ---------------------------------------------------------------------------
class Sweep {
 public:
  Sweep(int minDeg, int maxDeg, int stepDeg)
      : min_(minDeg), max_(maxDeg), step_(stepDeg > 0 ? stepDeg : 1), angle_(minDeg) {}

  int angle() const { return angle_; }
  int direction() const { return dir_; }

  void advance() {
    int next = angle_ + dir_ * step_;
    if (next > max_) {
      dir_ = -1;
      next = angle_ - step_;
      if (angle_ < max_) next = max_;  // land exactly on the end stop
    } else if (next < min_) {
      dir_ = 1;
      next = angle_ + step_;
      if (angle_ > min_) next = min_;
    }
    angle_ = clamp(next);
    if (angle_ == max_) dir_ = -1;
    if (angle_ == min_) dir_ = 1;
  }

  // Jump to an angle (e.g. when leaving hold mode) without changing direction.
  void moveTo(int deg) { angle_ = clamp(deg); }

  int clamp(int deg) const { return deg < min_ ? min_ : (deg > max_ ? max_ : deg); }

 private:
  int min_, max_, step_;
  int angle_;
  int dir_ = 1;
};

}  // namespace radar
