#include <Arduino.h>
#include <esp_arduino_version.h>

constexpr int PWM_PIN = 5;

// Target: 10 Hz pulse train, 10 ms high / 90 ms low
constexpr uint32_t PWM_FREQ_HZ = 10;
constexpr uint8_t PWM_RES_BITS = 12;     // 0..4095
constexpr float DUTY_PERCENT = 50.0f;    // 10%

#if ESP_ARDUINO_VERSION_MAJOR < 3
constexpr int PWM_CH = 0;
#endif

static uint32_t dutyFromPercent(float pct, uint8_t bits) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  const uint32_t maxDuty = (1UL << bits) - 1UL;
  return (uint32_t)((pct / 100.0f) * maxDuty + 0.5f);
}

void setup() {
  const uint32_t duty = dutyFromPercent(DUTY_PERCENT, PWM_RES_BITS);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 v3+
  ledcAttach(PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(PWM_PIN, duty);
#else
  // Arduino-ESP32 v2.x
  ledcSetup(PWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PWM_PIN, PWM_CH);
  ledcWrite(PWM_CH, duty);
#endif
}

void loop() {
  // Hardware PWM runs independently and remains stable.
}