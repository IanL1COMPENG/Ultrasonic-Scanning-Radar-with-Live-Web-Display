#include "oled_view.h"

#include <math.h>

#include "config.h"

namespace {
constexpr int16_t CX = 64;   // sweep origin: bottom centre
constexpr int16_t CY = 63;
constexpr int16_t R = 52;    // leaves the top text row free
constexpr float DEG2RAD = 0.01745329252f;
}  // namespace

bool OledView::begin(uint8_t sda, uint8_t scl, uint8_t addr) {
  Wire.begin(sda, scl);
  // periphBegin=false: Wire is already started on our pins.
  ok_ = d_.begin(SSD1306_SWITCHCAPVCC, addr, true, false);
  if (!ok_) {
    Serial.println(F("[oled] SSD1306 not found, continuing without display"));
    return false;
  }
  d_.cp437(true);  // correct glyph table, char 248 = degree sign
  d_.setTextColor(SSD1306_WHITE);
  d_.setTextSize(1);
  d_.clearDisplay();
  d_.display();
  return true;
}

void OledView::message(const char* line1, const char* line2, const char* line3) {
  if (!ok_) return;
  d_.clearDisplay();
  d_.setCursor(0, 8);
  d_.println(line1);
  if (line2) {
    d_.setCursor(0, 28);
    d_.println(line2);
  }
  if (line3) {
    d_.setCursor(0, 48);
    d_.println(line3);
  }
  d_.display();
}

void OledView::render(const float* dist, int sweepAngle, float distCm, float tempC, bool tempValid,
                      const char* footer) {
  if (!ok_) return;
  d_.clearDisplay();

  // Range rings (top half circles) at 1/3, 2/3 and full range.
  for (int k = 1; k <= 3; ++k) {
    const int16_t r = R * k / 3;
    d_.drawCircleHelper(CX, CY, r, 0x1 | 0x2, SSD1306_WHITE);
  }
  d_.drawFastHLine(CX - R, CY, 2 * R + 1, SSD1306_WHITE);

  // Echo returns.
  for (int a = 0; a <= 180; ++a) {
    const float cm = dist[a];
    if (cm <= 0) continue;
    const float r = cm / cfg::MAX_RANGE_CM * R;
    const float rad = a * DEG2RAD;
    const int16_t x = CX + static_cast<int16_t>(lroundf(r * cosf(rad)));
    const int16_t y = CY - static_cast<int16_t>(lroundf(r * sinf(rad)));
    d_.fillRect(x - 1, y - 1, 2, 2, SSD1306_WHITE);
  }

  // Sweep beam.
  const float srad = sweepAngle * DEG2RAD;
  d_.drawLine(CX, CY, CX + static_cast<int16_t>(R * cosf(srad)),
              CY - static_cast<int16_t>(R * sinf(srad)), SSD1306_WHITE);

  // Text row.
  char line[24];
  d_.setCursor(0, 0);
  if (footer) {
    d_.print(footer);
  } else {
    if (distCm > 0) {
      snprintf(line, sizeof line, "%3d%c %5.1fcm", sweepAngle, (char)248, distCm);
    } else {
      snprintf(line, sizeof line, "%3d%c   ---cm", sweepAngle, (char)248);
    }
    d_.print(line);
    snprintf(line, sizeof line, "%4.1fC%s", tempC, tempValid ? "" : "?");
    d_.setCursor(128 - 6 * static_cast<int16_t>(strlen(line)), 0);
    d_.print(line);
  }
  d_.display();
}
