#pragma once

// FreeRTOS task pinned to core 0: Wi-Fi, HTTP (dashboard), WebSocket stream,
// OLED refresh and the DS18B20 temperature reads.
void netTask(void* arg);
