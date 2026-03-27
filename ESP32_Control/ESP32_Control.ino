#include <Arduino.h>

#define PPS_PIN 4
#define OUT_PIN 5

// GNSS UART (u-blox UBX)
#define GNSS_RX_PIN 16
#define GNSS_BAUD 115200

// Spoofed NMEA UART out
#define NMEA_TX_PIN 17
#define NMEA_BAUD 9600

#define INTERVAL_US 100000UL   // 100 ms => 10 Hz
#define PULSE_WIDTH_US 10000UL // 10 ms pulse width

volatile uint32_t syncTimeUs = 0;
volatile bool synced = false;
volatile uint32_t lastPpsUs = 0;
volatile bool rmcPending = false;

struct NavPvtData {
  bool validDateTime = false;
  bool validFix = false;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
};

NavPvtData latestNav;
uint32_t lastNavMicros = 0;

// ===== UBX NAV-PVT parser =====
enum UbxState {
  UBX_SYNC1,
  UBX_SYNC2,
  UBX_CLASS,
  UBX_ID,
  UBX_LEN1,
  UBX_LEN2,
  UBX_PAYLOAD,
  UBX_CKA,
  UBX_CKB
};

UbxState ubxState = UBX_SYNC1;
uint8_t ubxClass = 0;
uint8_t ubxId = 0;
uint16_t ubxLen = 0;
uint16_t ubxIndex = 0;
uint8_t ubxCkA = 0;
uint8_t ubxCkB = 0;
uint8_t ubxRxCkA = 0;
uint8_t ubxRxCkB = 0;
uint8_t ubxPayload[128];

static inline void ubxChecksumUpdate(uint8_t b) {
  ubxCkA = ubxCkA + b;
  ubxCkB = ubxCkB + ubxCkA;
}

static inline uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static bool isLeapYear(uint16_t year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

static uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 2 && isLeapYear(year)) return 29;
  return days[month - 1];
}

static void incrementUtcOneSecond(NavPvtData &nav) {
  nav.second++;
  if (nav.second < 60) return;
  nav.second = 0;

  nav.minute++;
  if (nav.minute < 60) return;
  nav.minute = 0;

  nav.hour++;
  if (nav.hour < 24) return;
  nav.hour = 0;

  nav.day++;
  if (nav.day <= daysInMonth(nav.year, nav.month)) return;
  nav.day = 1;

  nav.month++;
  if (nav.month <= 12) return;
  nav.month = 1;

  nav.year++;
}

static void sendGprmc(const NavPvtData &nav) {
  char status = nav.validFix ? 'A' : 'V';

  char body[128];
  snprintf(
    body,
    sizeof(body),
    "GNRMC,%02u%02u%02u.00,%c,0000.0000,N,00000.0000,E,0.0,0.0,%02u%02u%02u,,",
    nav.hour, nav.minute, nav.second,
    status,
    nav.day, nav.month, (uint8_t)(nav.year % 100)
  );

  uint8_t checksum = 0;
  for (size_t i = 0; body[i] != '\0'; i++) {
    checksum ^= (uint8_t)body[i];
  }

  char sentence[140];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);

  Serial1.print(sentence);
  Serial.print(sentence);
}

void handleNavPvt(const uint8_t *payload, uint16_t len) {
  if (len < 20) {
    return;
  }

  NavPvtData nav;
  uint8_t valid = payload[11];
  uint8_t fixType = payload[20];
  uint8_t flags = payload[21];

  nav.year   = readU16LE(payload + 4);
  nav.month  = payload[6];
  nav.day    = payload[7];
  nav.hour   = payload[8];
  nav.minute = payload[9];
  nav.second = payload[10];

  nav.validDateTime = ((valid & 0x03) == 0x03);
  nav.validFix      = ((flags & 0x01) != 0) && (fixType >= 2);

  latestNav = nav;
  lastNavMicros = micros();

  // Serial.printf("NAV-PVT %04u-%02u-%02u %02u:%02u:%02u validTime=%u fixType=%u\n",
  //   nav.year, nav.month, nav.day,
  //   nav.hour, nav.minute, nav.second,
  //   nav.validDateTime, fixType);
}

void trySendPendingRmc() {
  bool pending;
  uint32_t ppsUs;

  noInterrupts();
  pending = rmcPending;
  ppsUs = lastPpsUs;
  interrupts();

  uint32_t ageUs = micros() - ppsUs;
  if (ageUs >= 10000UL && rmcPending) { // 10 ms after PPS, send RMC based on latest NAV-PVT
  sendGprmc(latestNav);
  noInterrupts();
  rmcPending = false;
  interrupts();
  return;
  }

}

void parseUbxByte(uint8_t b) {
  switch (ubxState) {
    case UBX_SYNC1:
      if (b == 0xB5) ubxState = UBX_SYNC2;
      break;

    case UBX_SYNC2:
      ubxState = (b == 0x62) ? UBX_CLASS : UBX_SYNC1;
      break;

    case UBX_CLASS:
      ubxClass = b;
      ubxCkA = 0;
      ubxCkB = 0;
      ubxChecksumUpdate(b);
      ubxState = UBX_ID;
      break;

    case UBX_ID:
      ubxId = b;
      ubxChecksumUpdate(b);
      ubxState = UBX_LEN1;
      break;

    case UBX_LEN1:
      ubxLen = b;
      ubxChecksumUpdate(b);
      ubxState = UBX_LEN2;
      break;

    case UBX_LEN2:
      ubxLen |= ((uint16_t)b << 8);
      ubxChecksumUpdate(b);
      ubxIndex = 0;
      if (ubxLen > sizeof(ubxPayload)) {
        ubxState = UBX_SYNC1;
      } else if (ubxLen == 0) {
        ubxState = UBX_CKA;
      } else {
        ubxState = UBX_PAYLOAD;
      }
      break;

    case UBX_PAYLOAD:
      ubxPayload[ubxIndex++] = b;
      ubxChecksumUpdate(b);
      if (ubxIndex >= ubxLen) {
        ubxState = UBX_CKA;
      }
      break;

    case UBX_CKA:
      ubxRxCkA = b;
      ubxState = UBX_CKB;
      break;

    case UBX_CKB:
      ubxRxCkB = b;
      if (ubxRxCkA == ubxCkA && ubxRxCkB == ubxCkB) {
        if (ubxClass == 0x01 && ubxId == 0x07) { 
          handleNavPvt(ubxPayload, ubxLen);
        }
      }
      ubxState = UBX_SYNC1;
      break;
  }
}

// ==== PPS ISR ====
void IRAM_ATTR onPPS() {
  syncTimeUs = micros();
  lastPpsUs = syncTimeUs;
  synced = true;
  rmcPending = true;
  digitalWrite(OUT_PIN, HIGH);
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);

  pinMode(PPS_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);

  attachInterrupt(PPS_PIN, onPPS, RISING);

  // GNSS UART input on RX2
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, -1);
  Serial1.begin(NMEA_BAUD, SERIAL_8N1, -1, NMEA_TX_PIN);
}

// ==== LOOP ====
void loop() {
  static bool outputHigh = false;

  while (Serial2.available() > 0) {
    parseUbxByte((uint8_t)Serial2.read());
  }

  trySendPendingRmc();
 
  //Checks to see if we have written the pulse output pin high, and if we have, checks if it's time to set it low again.
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