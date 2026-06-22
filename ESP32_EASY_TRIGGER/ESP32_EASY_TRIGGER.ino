// Pin and timing configuration (avoid magic numbers)
const int TRIGGER_PIN = 5;
const unsigned long PULSE_MS = 500;
const unsigned long TRIGGER_PULSE_MS = 100;

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
}

void loop() {
  digitalWrite(TRIGGER_PIN, HIGH);
  // delay(TRIGGER_PULSE_MS);
  // digitalWrite(TRIGGER_PIN, LOW);
  // delay(PULSE_MS);
}
