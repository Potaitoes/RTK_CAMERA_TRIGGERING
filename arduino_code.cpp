#include <Arduino.h>

#define PPS_PIN 4
#define OUT_PIN 5

#define INTERVAL_US 100000UL   // 100 ms => 10 Hz
#define PULSE_WIDTH_US 100UL // 0.1 ms pulse width

volatile uint32_t syncTimeUs = 0;
volatile bool synced = false;

// ==== PPS ISR ====
void IRAM_ATTR onPPS() {
  syncTimeUs = micros();
  synced = true;
  digitalWrite(OUT_PIN, HIGH);
}

// ==== SETUP ====
void setup() {
  pinMode(PPS_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  attachInterrupt(PPS_PIN, onPPS, RISING);
}

// ==== LOOP ====
void loop() {
  static bool outputHigh = false;

  if (!synced) {
    return;
  }

  uint32_t startUs;
  noInterrupts();
  startUs = syncTimeUs;
  interrupts();

  uint32_t elapsedUs = micros() - startUs;
  uint32_t phaseUs = elapsedUs % INTERVAL_US;
  bool shouldBeHigh = phaseUs < PULSE_WIDTH_US;

  if (shouldBeHigh != outputHigh) {
    digitalWrite(OUT_PIN, shouldBeHigh ? HIGH : LOW);
    outputHigh = shouldBeHigh;
  }
}