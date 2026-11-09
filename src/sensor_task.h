#pragma once

// FreeRTOS task pinned to core 1: drives the servo, fires the ultrasonic
// pings, filters them and publishes one RadarSample every SCAN_PERIOD_MS.
void sensorTask(void* arg);
