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
  int32_t lon = 0;
  int32_t lat = 0;
  int32_t gSpeed = 0;
  int32_t headMot = 0;
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

static inline int32_t readI32LE(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] |
                   ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 24));
}

static inline uint32_t readU32LE(const uint8_t *p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
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

static void formatLat(int32_t latE7, char *out, size_t outLen, char &hemi) {
  double absDeg = (latE7 < 0) ? (double)(-latE7) / 1e7 : (double)latE7 / 1e7;
  uint32_t deg = (uint32_t)absDeg;
  double minutes = (absDeg - (double)deg) * 60.0;
  hemi = (latE7 < 0) ? 'S' : 'N';
  snprintf(out, outLen, "%02lu%07.4f", (unsigned long)deg, minutes);
}

static void formatLon(int32_t lonE7, char *out, size_t outLen, char &hemi) {
  double absDeg = (lonE7 < 0) ? (double)(-lonE7) / 1e7 : (double)lonE7 / 1e7;
  uint32_t deg = (uint32_t)absDeg;
  double minutes = (absDeg - (double)deg) * 60.0;
  hemi = (lonE7 < 0) ? 'W' : 'E';
  snprintf(out, outLen, "%03lu%07.4f", (unsigned long)deg, minutes);
}

static void sendGprmc(const NavPvtData &nav) {
  char latBuf[16];
  char lonBuf[16];
  char latHemi;
  char lonHemi;

  formatLat(nav.lat, latBuf, sizeof(latBuf), latHemi);
  formatLon(nav.lon, lonBuf, sizeof(lonBuf), lonHemi);

  double speedKnots = (double)nav.gSpeed / 514.444;
  double courseDeg = (double)nav.headMot / 100000.0;
  char status = nav.validFix ? 'A' : 'V';

  char body[128];
  snprintf(
    body,
    sizeof(body),
    "GNRMC,%02u%02u%02u.00,%c,%s,%c,%s,%c,%.1f,%.1f,%02u%02u%02u,,",
    nav.hour, nav.minute, nav.second,
    status,
    latBuf, latHemi,
    lonBuf, lonHemi,
    speedKnots,
    courseDeg,
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
  if (len < 92) {
    return;
  }

  NavPvtData nav;
  uint8_t valid = payload[11];
  uint8_t fixType = payload[20];
  uint8_t numSV = payload[23];
  uint8_t flags = payload[21];

  nav.year = readU16LE(payload + 4);
  nav.month = payload[6];
  nav.day = payload[7];
  nav.hour = payload[8];
  nav.minute = payload[9];
  nav.second = payload[10];
  nav.lon = readI32LE(payload + 24);
  nav.lat = readI32LE(payload + 28);
  nav.gSpeed = readI32LE(payload + 60);
  nav.headMot = readI32LE(payload + 64);

  nav.validDateTime = ((valid & 0x03) == 0x03);
  nav.validFix = ((flags & 0x01) != 0) && (fixType >= 2);

  latestNav = nav;
  lastNavMicros = micros();

  Serial.print("NAV-PVT ");
  Serial.print(nav.year);
  Serial.print("-");
  Serial.print(nav.month);
  Serial.print("-");
  Serial.print(nav.day);
  Serial.print(" ");
  Serial.print(nav.hour);
  Serial.print(":");
  Serial.print(nav.minute);
  Serial.print(":");
  Serial.print(nav.second);
  Serial.print(" validTime=");
  Serial.print(nav.validDateTime);
  Serial.print(" fixType=");
  Serial.print(fixType);
  Serial.print(" numSV=");
  Serial.print(numSV);
  Serial.print(" validFix=");
  Serial.print(nav.validFix);
  Serial.print(" lat=");
  Serial.print((double)nav.lat / 1e7, 7);
  Serial.print(" lon=");
  Serial.print((double)nav.lon / 1e7, 7);
  Serial.print(" speed(m/s)=");
  Serial.print((double)nav.gSpeed / 1000.0, 3);
  Serial.print(" heading=");
  Serial.println((double)nav.headMot / 100000.0, 5);
}

void trySendPendingRmc() {
  bool pending;
  uint32_t ppsUs;

  noInterrupts();
  pending = rmcPending;
  ppsUs = lastPpsUs;
  interrupts();

  if (!pending || !latestNav.validDateTime) {
    return;
  }

  uint32_t ageUs = micros() - ppsUs;

  if (lastNavMicros >= ppsUs) {
    sendGprmc(latestNav);
    noInterrupts();
    rmcPending = false;
    interrupts();
    return;
  }

  if (ageUs >= 350000UL) {
    NavPvtData fallback = latestNav;
    incrementUtcOneSecond(fallback);
    sendGprmc(fallback);

    noInterrupts();
    rmcPending = false;
    interrupts();
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