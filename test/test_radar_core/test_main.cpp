// Host unit tests for lib/RadarCore.   pio test -e native
#include <radar_core.h>
#include <unity.h>

#include <math.h>
#include <stdlib.h>

using namespace radar;

void setUp() {}
void tearDown() {}

// ---- speed of sound ----------------------------------------------------------

void test_speed_of_sound_reference_points() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 331.30f, speedOfSound(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 343.21f, speedOfSound(20.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 349.04f, speedOfSound(30.0f));
  // Close to the common linear rule of thumb near room temperature.
  for (int t = 0; t <= 30; t += 5) {
    const float linear = 331.3f + 0.606f * t;
    TEST_ASSERT_FLOAT_WITHIN(linear * 0.002f, linear, speedOfSound(static_cast<float>(t)));
  }
}

void test_echo_to_cm() {
  // 100 cm at 20 C -> 2 * 1.00 m / 343.21 m/s = 5827.3 us
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, echoToCm(5827, 20.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, echoToCm(0, 20.0f));
}

void test_round_trip_cm_echo() {
  for (float cm = 5.0f; cm <= 150.0f; cm += 5.0f) {
    for (float t = -10.0f; t <= 40.0f; t += 10.0f) {
      TEST_ASSERT_FLOAT_WITHIN(0.02f, cm, echoToCm(cmToEchoUs(cm, t), t));
    }
  }
}

// The point of the temperature sensor: at 35 C an uncompensated sensor that
// assumes 20 C reads a 150 cm target ~4 cm short; compensated, it's exact.
void test_temperature_compensation_matters() {
  const float trueCm = 150.0f;
  const uint32_t echo = cmToEchoUs(trueCm, 35.0f);
  const float uncompensated = echoToCm(echo, 20.0f);
  const float compensated = echoToCm(echo, 35.0f);
  TEST_ASSERT_TRUE(trueCm - uncompensated > 3.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, trueCm, compensated);
}

void test_plausible_temp() {
  TEST_ASSERT_TRUE(isPlausibleTemp(21.5f));
  TEST_ASSERT_FALSE(isPlausibleTemp(85.0f));     // DS18B20 power-on value
  TEST_ASSERT_FALSE(isPlausibleTemp(-127.0f));   // disconnected
}

// ---- median filter -------------------------------------------------------------

void test_median_rejects_outlier() {
  const float s[5] = {100.2f, 99.8f, 100.0f, 37.0f /* multipath */, 100.1f};
  TEST_ASSERT_EQUAL_FLOAT(100.0f, median(s, 5, 3));
}

void test_median_rejects_two_outliers() {
  const float s[5] = {250.0f, 60.1f, 3.0f, 60.3f, 60.2f};
  TEST_ASSERT_EQUAL_FLOAT(60.2f, median(s, 5, 3));
}

void test_median_ignores_misses() {
  const float s[5] = {42.0f, -1.0f, 41.0f, -1.0f, 43.0f};
  TEST_ASSERT_EQUAL_FLOAT(42.0f, median(s, 5, 3));
}

void test_median_requires_majority() {
  const float s[5] = {42.0f, -1.0f, -1.0f, -1.0f, 43.0f};
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, median(s, 5, 3));
  const float none[5] = {-1, -1, -1, -1, -1};
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, median(none, 5, 1));
}

void test_median_even_count_averages() {
  const float s[5] = {10.0f, 12.0f, -1.0f, 14.0f, 16.0f};
  TEST_ASSERT_EQUAL_FLOAT(13.0f, median(s, 5, 3));
}

void test_median_rejects_nan() {
  const float s[5] = {NAN, 20.0f, 21.0f, 22.0f, NAN};
  TEST_ASSERT_EQUAL_FLOAT(21.0f, median(s, 5, 3));
}

void test_median_filter_reduces_error() {
  // Deterministic noisy stream around 80 cm with 20 % gross outliers.
  uint32_t lcg = 12345;
  auto rnd = [&lcg]() {
    lcg = lcg * 1664525u + 1013904223u;
    return (lcg >> 8) / 16777216.0f;  // [0,1)
  };
  float worstRaw = 0, worstFiltered = 0;
  for (int k = 0; k < 500; ++k) {
    float s[5];
    for (float& v : s) {
      v = 80.0f + (rnd() - 0.5f) * 0.8f;          // +/-0.4 cm jitter
      if (rnd() < 0.2f) v = 20.0f + rnd() * 130.f;  // echo outlier
      worstRaw = fmaxf(worstRaw, fabsf(v - 80.0f));
    }
    const float m = median(s, 5, 3);
    // With 2 or fewer outliers (the common case) the median stays on target.
    int outliers = 0;
    for (float v : s) outliers += fabsf(v - 80.0f) > 1.0f;
    if (outliers <= 2) worstFiltered = fmaxf(worstFiltered, fabsf(m - 80.0f));
  }
  TEST_ASSERT_TRUE(worstRaw > 10.0f);
  TEST_ASSERT_TRUE(worstFiltered < 1.0f);
}

// ---- sweep -----------------------------------------------------------------------

void test_sweep_bounces_and_hits_endpoints() {
  Sweep sw(0, 180, 2);
  TEST_ASSERT_EQUAL_INT(0, sw.angle());
  int prev = sw.angle();
  int hits180 = 0, hits0 = 0;
  for (int i = 0; i < 400; ++i) {
    sw.advance();
    TEST_ASSERT_EQUAL_INT(2, abs(sw.angle() - prev));
    TEST_ASSERT_TRUE(sw.angle() >= 0 && sw.angle() <= 180);
    hits180 += sw.angle() == 180;
    hits0 += sw.angle() == 0;
    prev = sw.angle();
  }
  TEST_ASSERT_TRUE(hits180 >= 2);
  TEST_ASSERT_TRUE(hits0 >= 2);
}

void test_sweep_direction_flips_at_ends() {
  Sweep sw(0, 180, 2);
  for (int i = 0; i < 90; ++i) sw.advance();
  TEST_ASSERT_EQUAL_INT(180, sw.angle());
  TEST_ASSERT_EQUAL_INT(-1, sw.direction());
  sw.advance();
  TEST_ASSERT_EQUAL_INT(178, sw.angle());
}

void test_sweep_lands_on_endpoint_with_uneven_step() {
  Sweep sw(0, 180, 7);
  bool saw180 = false;
  for (int i = 0; i < 40; ++i) {
    sw.advance();
    saw180 |= sw.angle() == 180;
  }
  TEST_ASSERT_TRUE(saw180);
}

void test_sweep_move_to_clamps() {
  Sweep sw(0, 180, 2);
  sw.moveTo(250);
  TEST_ASSERT_EQUAL_INT(180, sw.angle());
  sw.moveTo(-5);
  TEST_ASSERT_EQUAL_INT(0, sw.angle());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_speed_of_sound_reference_points);
  RUN_TEST(test_echo_to_cm);
  RUN_TEST(test_round_trip_cm_echo);
  RUN_TEST(test_temperature_compensation_matters);
  RUN_TEST(test_plausible_temp);
  RUN_TEST(test_median_rejects_outlier);
  RUN_TEST(test_median_rejects_two_outliers);
  RUN_TEST(test_median_ignores_misses);
  RUN_TEST(test_median_requires_majority);
  RUN_TEST(test_median_even_count_averages);
  RUN_TEST(test_median_rejects_nan);
  RUN_TEST(test_median_filter_reduces_error);
  RUN_TEST(test_sweep_bounces_and_hits_endpoints);
  RUN_TEST(test_sweep_direction_flips_at_ends);
  RUN_TEST(test_sweep_lands_on_endpoint_with_uneven_step);
  RUN_TEST(test_sweep_move_to_clamps);
  return UNITY_END();
}
