#include "ultrasonic.h"

#include <esp_timer.h>

void Ultrasonic::begin(uint8_t trigPin, uint8_t echoPin) {
  trig_ = trigPin;
  echo_ = echoPin;
  pinMode(trig_, OUTPUT);
  digitalWrite(trig_, LOW);
  pinMode(echo_, INPUT);
  // The interrupt is allocated on the core that calls attachInterrupt, so
  // calling begin() from the sensor task keeps the ISR on core 1.
  attachInterruptArg(digitalPinToInterrupt(echo_), &Ultrasonic::onEchoEdge, this, CHANGE);
}

void IRAM_ATTR Ultrasonic::onEchoEdge(void* arg) {
  auto* self = static_cast<Ultrasonic*>(arg);
  const int64_t now = esp_timer_get_time();
  if (!self->armed_) return;

  if (digitalRead(self->echo_)) {
    self->riseUs_ = now;
  } else if (self->riseUs_ != 0) {
    self->widthUs_ = static_cast<uint32_t>(now - self->riseUs_);
    self->armed_ = false;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(self->waiter_, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
}

uint32_t Ultrasonic::ping(uint32_t timeoutUs) {
  // Many HC-SR04 clones hold ECHO high for up to ~40 ms when nothing comes
  // back. Firing again now would be ignored, so report a miss immediately.
  if (digitalRead(echo_)) return 0;

  waiter_ = xTaskGetCurrentTaskHandle();
  ulTaskNotifyTake(pdTRUE, 0);  // clear any stale notification
  riseUs_ = 0;
  widthUs_ = 0;
  armed_ = true;

  digitalWrite(trig_, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_, LOW);

  // +2 ticks covers the ~0.5 ms burst latency before ECHO rises and tick
  // rounding; the pulse width itself is checked against timeoutUs below.
  const TickType_t waitTicks = pdMS_TO_TICKS(timeoutUs / 1000) + 2;
  if (ulTaskNotifyTake(pdTRUE, waitTicks) == 0) {
    armed_ = false;
    return 0;
  }
  const uint32_t width = widthUs_;
  return width > timeoutUs ? 0 : width;
}
