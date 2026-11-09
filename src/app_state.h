// State shared between the sensing task (core 1) and the network task (core 0).
#pragma once

#include <Arduino.h>
#include <atomic>

// One filtered reading at one servo angle.
struct RadarSample {
  uint32_t seq;
  uint32_t tMs;
  int16_t angle;        // degrees, 0 = right, 180 = left
  int8_t dir;           // sweep direction (+1 / -1, 0 when holding)
  uint8_t validPings;   // how many of the pings returned an echo
  float distanceCm;     // median-filtered, temperature-compensated; -1 = no echo
  float tempC;          // temperature used for the speed of sound
  uint32_t cycleUs;     // time spent measuring this step
};

enum class ScanMode : uint8_t { Sweep = 0, Paused = 1, Hold = 2 };

struct SharedState {
  QueueHandle_t samples = nullptr;            // sensor -> net, RadarSample

  // net -> sensor
  std::atomic<uint8_t> mode{static_cast<uint8_t>(ScanMode::Sweep)};
  std::atomic<int32_t> holdAngle{90};

  // Temperature is read on core 0 (OneWire bit-banging must never delay the
  // echo ISR on core 1) and consumed by the sensor task.
  std::atomic<int32_t> tempCentiC{2000};
  std::atomic<bool> tempValid{false};

  // sensor -> net diagnostics
  std::atomic<uint32_t> samplesProduced{0};
  std::atomic<uint32_t> overruns{0};
  std::atomic<uint32_t> droppedSamples{0};
};

extern SharedState g_state;

inline float sharedTempC() { return g_state.tempCentiC.load() / 100.0f; }
