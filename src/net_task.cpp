#include "net_task.h"

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <radar_core.h>

#include "app_state.h"
#include "config.h"
#include "oled_view.h"
#include "temperature.h"
#include "web_page_gen.h"  // generated from web/index.html by tools/embed_web.py

namespace {

WebServer http(cfg::HTTP_PORT);
WebSocketsServer ws(cfg::WS_PORT);
OledView oled;
TemperatureSensor thermo;

constexpr int kAngles = 181;
float scan[kAngles];          // latest distance per angle, -1 = none
RadarSample last{};           // most recent sample
String ipString;
bool apMode = false;
uint32_t ipBannerUntilMs = 0;

// Scan-rate estimate over a 1 s window.
uint32_t rateWindowStartMs = 0;
uint32_t rateWindowStartCount = 0;
float scanRateHz = 0;
uint32_t maxCycleUs = 0;

char txBuf[2048];

const char* modeName(ScanMode m) {
  switch (m) {
    case ScanMode::Paused: return "paused";
    case ScanMode::Hold: return "hold";
    default: return "sweep";
  }
}

// ---------------------------------------------------------------------------
// JSON messages (hand-formatted: they're small, frequent and fixed-shape)
// ---------------------------------------------------------------------------

size_t formatHello() {
  return snprintf(txBuf, sizeof txBuf,
                  "{\"type\":\"hello\",\"fw\":\"%s\",\"minRange\":%.0f,\"maxRange\":%.0f,"
                  "\"step\":%d,\"periodMs\":%u,\"pings\":%u}",
                  cfg::FW_VERSION, cfg::MIN_RANGE_CM, cfg::MAX_RANGE_CM, cfg::SWEEP_STEP_DEG,
                  (unsigned)cfg::SCAN_PERIOD_MS, (unsigned)cfg::PINGS_PER_READING);
}

size_t formatSnapshot() {
  size_t n = snprintf(txBuf, sizeof txBuf, "{\"type\":\"snapshot\",\"d\":[");
  for (int a = 0; a < kAngles && n < sizeof txBuf - 16; ++a) {
    n += snprintf(txBuf + n, sizeof txBuf - n, a ? ",%.1f" : "%.1f", scan[a] > 0 ? scan[a] : -1.0f);
  }
  n += snprintf(txBuf + n, sizeof txBuf - n, "]}");
  return n;
}

size_t formatStatus() {
  const float t = sharedTempC();
  const ScanMode m = static_cast<ScanMode>(g_state.mode.load());
  return snprintf(
      txBuf, sizeof txBuf,
      "{\"type\":\"status\",\"temp\":%.2f,\"tempValid\":%s,\"sos\":%.2f,\"rate\":%.2f,"
      "\"overruns\":%u,\"dropped\":%u,\"maxCycleMs\":%.2f,\"mode\":\"%s\",\"hold\":%d,"
      "\"clients\":%u,\"rssi\":%d,\"ap\":%s,\"ip\":\"%s\",\"uptime\":%lu,\"heap\":%u}",
      t, g_state.tempValid.load() ? "true" : "false", radar::speedOfSound(t), scanRateHz,
      (unsigned)g_state.overruns.load(), (unsigned)g_state.droppedSamples.load(),
      maxCycleUs / 1000.0f, modeName(m), (int)g_state.holdAngle.load(),
      (unsigned)ws.connectedClients(), apMode ? 0 : (int)WiFi.RSSI(), apMode ? "true" : "false",
      ipString.c_str(), (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap());
}

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------

void handleCommand(const uint8_t* payload, size_t len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* cmd = doc["cmd"] | "";

  if (!strcmp(cmd, "pause")) {
    g_state.mode.store(static_cast<uint8_t>(ScanMode::Paused));
  } else if (!strcmp(cmd, "sweep") || !strcmp(cmd, "resume")) {
    g_state.mode.store(static_cast<uint8_t>(ScanMode::Sweep));
  } else if (!strcmp(cmd, "hold")) {
    int angle = doc["angle"] | 90;
    angle = constrain(angle, cfg::SWEEP_MIN_DEG, cfg::SWEEP_MAX_DEG);
    g_state.holdAngle.store(angle);
    g_state.mode.store(static_cast<uint8_t>(ScanMode::Hold));
  } else if (!strcmp(cmd, "clear")) {
    for (float& v : scan) v = -1.0f;
  } else {
    return;
  }
  const size_t n = formatStatus();
  ws.broadcastTXT(txBuf, n);
}

void onWsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      Serial.printf("[ws] client %u connected\n", client);
      size_t n = formatHello();
      ws.sendTXT(client, txBuf, n);
      n = formatSnapshot();
      ws.sendTXT(client, txBuf, n);
      n = formatStatus();
      ws.sendTXT(client, txBuf, n);
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[ws] client %u disconnected\n", client);
      break;
    case WStype_TEXT:
      handleCommand(payload, len);
      break;
    default:
      break;
  }
}

// Drain everything the sensor task produced and forward it as one batch.
void pumpSamples() {
  RadarSample s;
  size_t n = 0;
  int count = 0;
  while (count < 32 && xQueueReceive(g_state.samples, &s, 0) == pdTRUE) {
    if (s.angle >= 0 && s.angle < kAngles) scan[s.angle] = s.distanceCm;
    if (s.cycleUs > maxCycleUs) maxCycleUs = s.cycleUs;
    last = s;
    if (count == 0) n = snprintf(txBuf, sizeof txBuf, "{\"type\":\"s\",\"s\":[");
    n += snprintf(txBuf + n, sizeof txBuf - n, "%s[%d,%.1f,%d,%u,%u]", count ? "," : "", s.angle,
                  s.distanceCm, s.dir, (unsigned)s.validPings, (unsigned)s.seq);
    ++count;
  }
  if (count == 0) return;
  n += snprintf(txBuf + n, sizeof txBuf - n, "]}");
  if (ws.connectedClients() > 0) ws.broadcastTXT(txBuf, n);
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

void setupHttp() {
  http.on("/", HTTP_GET, [] {
    http.sendHeader("Content-Encoding", "gzip");
    http.sendHeader("Cache-Control", "no-cache");
    http.send_P(200, "text/html", reinterpret_cast<const char*>(WEB_PAGE_GZ), WEB_PAGE_GZ_LEN);
  });
  http.on("/api/status", HTTP_GET, [] {
    formatStatus();
    http.sendHeader("Access-Control-Allow-Origin", "*");
    http.send(200, "application/json", txBuf);
  });
  http.on("/api/scan", HTTP_GET, [] {
    formatSnapshot();
    http.sendHeader("Access-Control-Allow-Origin", "*");
    http.send(200, "application/json", txBuf);
  });
  http.onNotFound([] { http.send(404, "text/plain", "Not found"); });
  http.begin();
}

// ---------------------------------------------------------------------------
// Wi-Fi: station mode if credentials are set, otherwise / on failure an AP
// ---------------------------------------------------------------------------

void startWifi() {
  WiFi.persistent(false);
  WiFi.setHostname(cfg::HOSTNAME);

  if (strlen(WIFI_SSID) > 0) {
    oled.message("Connecting Wi-Fi", WIFI_SSID);
    Serial.printf("[wifi] connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < cfg::WIFI_CONNECT_TIMEOUT_MS) {
      vTaskDelay(pdMS_TO_TICKS(250));
      Serial.print('.');
    }
    Serial.println();
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);  // modem sleep adds 100+ ms of WebSocket jitter
    WiFi.setAutoReconnect(true);
    ipString = WiFi.localIP().toString();
    apMode = false;
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(cfg::AP_SSID, cfg::AP_PASS);
    ipString = WiFi.softAPIP().toString();
    apMode = true;
    Serial.printf("[wifi] access point \"%s\" (password %s)\n", cfg::AP_SSID, cfg::AP_PASS);
  }

  if (MDNS.begin(cfg::HOSTNAME)) {
    MDNS.addService("http", "tcp", cfg::HTTP_PORT);
    MDNS.addService("ws", "tcp", cfg::WS_PORT);
  }
  Serial.printf("[wifi] dashboard: http://%s/  (http://%s.local/)\n", ipString.c_str(),
                cfg::HOSTNAME);
}

void updateRate(uint32_t nowMs) {
  if (nowMs - rateWindowStartMs < 1000) return;
  const uint32_t count = g_state.samplesProduced.load();
  scanRateHz = (count - rateWindowStartCount) * 1000.0f / (nowMs - rateWindowStartMs);
  rateWindowStartCount = count;
  rateWindowStartMs = nowMs;
}

}  // namespace

void netTask(void*) {
  for (float& v : scan) v = -1.0f;

  oled.begin(cfg::PIN_SDA, cfg::PIN_SCL, cfg::OLED_ADDR);
  oled.message("Ultrasonic Radar", "starting...");

  thermo.begin(cfg::PIN_ONEWIRE);

  startWifi();
  setupHttp();
  ws.begin();
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(15000, 3000, 2);  // drop dead browser tabs

  ipBannerUntilMs = millis() + 8000;
  rateWindowStartMs = millis();
  rateWindowStartCount = g_state.samplesProduced.load();

  uint32_t lastOled = 0, lastStatus = 0, lastLog = 0;
  char banner[24];

  for (;;) {
    const uint32_t now = millis();

    http.handleClient();
    ws.loop();
    pumpSamples();

    // Temperature lives on this core so OneWire's interrupt-disabled bit
    // slots can never stretch an echo measurement on core 1.
    thermo.service(now);
    g_state.tempValid.store(thermo.valid());
    g_state.tempCentiC.store(static_cast<int32_t>(lroundf(
        (thermo.valid() ? thermo.celsius() : cfg::DEFAULT_TEMP_C) * 100.0f)));

    updateRate(now);

    if (now - lastOled >= cfg::OLED_PERIOD_MS) {
      lastOled = now;
      const char* footer = nullptr;
      if (static_cast<int32_t>(ipBannerUntilMs - now) > 0) {
        snprintf(banner, sizeof banner, "%s%s", apMode ? "AP " : "", ipString.c_str());
        footer = banner;
      }
      oled.render(scan, last.angle, last.distanceCm, sharedTempC(), g_state.tempValid.load(),
                  footer);
    }

    if (now - lastStatus >= cfg::STATUS_PERIOD_MS) {
      lastStatus = now;
      if (ws.connectedClients() > 0) {
        const size_t n = formatStatus();
        ws.broadcastTXT(txBuf, n);
      }
      maxCycleUs = 0;  // report the worst case per status window
    }

    if (now - lastLog >= 5000) {
      lastLog = now;
      Serial.printf("[radar] %.1f Hz, angle %d, dist %.1f cm, temp %.2f C%s, overruns %u, "
                    "clients %u\n",
                    scanRateHz, last.angle, last.distanceCm, sharedTempC(),
                    g_state.tempValid.load() ? "" : " (default)",
                    (unsigned)g_state.overruns.load(), (unsigned)ws.connectedClients());
    }

    vTaskDelay(1);  // yield to the Wi-Fi stack and the idle task
  }
}
