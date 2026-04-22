#include <Arduino.h>

#define OUT_PIN 5

#define CYCLE_US        200000UL  // 200 ms total
#define PULSE_WIDTH_US   10000UL  // 10 ms pulse
#define SECOND_START_US  60000UL  // second pulse at 60 ms

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
}

void loop() {
  static bool outputHigh = false;
  static uint32_t startUs = micros();

  uint32_t elapsedUs = micros() - startUs;
  uint32_t phaseUs = elapsedUs % CYCLE_US;

  bool shouldBeHigh =
      (phaseUs < PULSE_WIDTH_US) ||                                  // 0–10 ms
      (phaseUs >= SECOND_START_US && phaseUs < SECOND_START_US + PULSE_WIDTH_US); // 60–70 ms

  if (shouldBeHigh != outputHigh) {
    digitalWrite(OUT_PIN, shouldBeHigh ? HIGH : LOW);
    outputHigh = shouldBeHigh;
  }
}