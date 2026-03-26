import struct

# UBX-CFG-TP5 message
# Class: 0x06, ID: 0x31
# Configure TIMEPULSE as a phase-locked 10 Hz signal with 10 ms high time.

FREQ_PERIOD_US = 100000   # 10 Hz => 100 ms period
PULSE_LEN_US = 10000      # 10 ms high time

# Payload (32 bytes)
payload = struct.pack(
    '<BBHhhIIIIiI',
    0,               # tpIdx
    1,               # version
    0,
    50,
    0,
    FREQ_PERIOD_US,  # freqPeriod
    FREQ_PERIOD_US,  # freqPeriodLock
    PULSE_LEN_US,    # pulseLenRatio
    PULSE_LEN_US,    # pulseLenRatioLock
    0,
    0x17             # active, locked to GNSS time, use period/length in us
)


# UBX header
msg = b'\xb5\x62' + b'\x06\x31' + struct.pack('<H', len(payload)) + payload

# checksum
ck_a = 0
ck_b = 0
for b in msg[2:]:
    ck_a = (ck_a + b) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF

msg += bytes([ck_a, ck_b])

# send to device
with open('/dev/ttyACM0', 'wb') as f:
    f.write(msg)

print("CFG-TP5 sent")
