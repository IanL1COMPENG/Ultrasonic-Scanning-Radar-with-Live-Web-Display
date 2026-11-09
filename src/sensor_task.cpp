#include "sensor_task.h"

#include <esp_timer.h>
#include <radar_core.h>

#include <cmath>
#include <cstdlib>

#include "app_state.h"
#include "config.h"
#include "servo_driver.h"
#include "ultrasonic.h"

namespace {

void publish(const RadarSample& s) {
  if (xQueueSend(g_state.samples, &s, 0) != pdTRUE) {
    // Consumer fell behind (e.g. while Wi-Fi connects): drop the oldest so
    // the dashboard always sees the most recent sweep.
    RadarSample stale;
    xQueueReceive(g_state.samples, &stale, 0);
    xQueueSend(g_state.samples, &s, 0);
    g_state.droppedSamples.fetch_add(1);
  }
  g_state.samplesProduced.fetch_add(1);
}

uint32_t settleMsFor(int fromDeg, int toDeg) {
  const float ms = std::abs(toDeg - fromDeg) * cfg::SERVO_SETTLE_MS_PER_DEG;
  return ms < cfg::SERVO_SETTLE_MIN_MS ? cfg::SERVO_SETTLE_MIN_MS : static_cast<uint32_t>(ms);
}

}  // namespace

void sensorTask(void*) {
  Ultrasonic sonar;
  sonar.begin(cfg::PIN_TRIG, cfg::PIN_ECHO);  // ISR lands on this core (1)

  ServoDriver servo;
  servo.begin(cfg::PIN_SERVO, cfg::SERVO_LEDC_CHANNEL, cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);

  radar::Sweep sweep(cfg::SWEEP_MIN_DEG, cfg::SWEEP_MAX_DEG, cfg::SWEEP_STEP_DEG);
  servo.write(sweep.angle());
  vTaskDelay(pdMS_TO_TICKS(600));  // let the servo reach its start position

  uint32_t seq = 0;
  uint32_t settleUntilMs = millis();
  float pings[cfg::PINGS_PER_READING];

  const TickType_t period = pdMS_TO_TICKS(cfg::SCAN_PERIOD_MS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const ScanMode mode = static_cast<ScanMode>(g_state.mode.load());
    if (mode == ScanMode::Paused) {
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    // Wait for the servo to finish the move commanded at the end of the
    // previous cycle (usually already elapsed while we slept).
    const int32_t waitMs = static_cast<int32_t>(settleUntilMs - millis());
    if (waitMs > 0) vTaskDelay(pdMS_TO_TICKS(waitMs));
    // A long settle (big jump into/out of hold mode) is expected to blow the
    // budget; re-phase afterwards instead of counting it as an overrun.
    const bool longSettle = waitMs > static_cast<int32_t>(cfg::SCAN_PERIOD_MS / 2);

    const int64_t t0 = esp_timer_get_time();
    const float tempC = sharedTempC();
    const uint32_t timeoutUs =
        radar::cmToEchoUs(cfg::MAX_RANGE_CM * cfg::TIMEOUT_RANGE_FACTOR, tempC);

    // ---- 5 pings -> median ----------------------------------------------
    size_t valid = 0, misses = 0;
    for (size_t i = 0; i < cfg::PINGS_PER_READING; ++i) {
      // Once a majority has missed, the median can't be valid: skip the
      // rest and keep the 20 Hz budget.
      if (misses > cfg::PINGS_PER_READING - cfg::MIN_VALID_PINGS) {
        pings[i] = -1.0f;
        continue;
      }
      const uint32_t echoUs = sonar.ping(timeoutUs);
      float cm = echoUs ? radar::echoToCm(echoUs, tempC) : -1.0f;
      if (cm < cfg::MIN_RANGE_CM || cm > cfg::MAX_RANGE_CM) cm = -1.0f;
      pings[i] = cm;
      if (cm > 0) ++valid; else ++misses;
      if (i + 1 < cfg::PINGS_PER_READING) delayMicroseconds(cfg::PING_GAP_US);
    }
    float distance = radar::median(pings, cfg::PINGS_PER_READING, cfg::MIN_VALID_PINGS);
    if (distance > 0) distance = distance * cfg::CAL_SCALE + cfg::CAL_OFFSET_CM;

    const uint32_t cycleUs = static_cast<uint32_t>(esp_timer_get_time() - t0);

    RadarSample s{};
    s.seq = seq++;
    s.tMs = millis();
    s.angle = static_cast<int16_t>(servo.angle());
    s.dir = mode == ScanMode::Hold ? 0 : static_cast<int8_t>(sweep.direction());
    s.validPings = static_cast<uint8_t>(valid);
    s.distanceCm = distance;
    s.tempC = tempC;
    s.cycleUs = cycleUs;
    publish(s);

    // ---- move to the next angle while we sleep ---------------------------
    int next;
    if (mode == ScanMode::Hold) {
      next = sweep.clamp(g_state.holdAngle.load());
      sweep.moveTo(next);  // resume sweeping from here later
    } else {
      sweep.advance();
      next = sweep.angle();
    }
    const int from = servo.angle();
    if (next != from) {
      servo.write(next);
      settleUntilMs = millis() + settleMsFor(from, next);
    }

    const TickType_t now = xTaskGetTickCount();
    if (now - lastWake > period) {
      if (longSettle) {
        lastWake = now;  // re-phase instead of bursting to catch up
      } else {
        g_state.overruns.fetch_add(1);
      }
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
