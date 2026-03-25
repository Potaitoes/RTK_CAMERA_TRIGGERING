import struct

# UBX-CFG-TP5 message
# Class: 0x06, ID: 0x31

# Payload (32 bytes)
payload = struct.pack(
    '<BBHhhIIIIiI',
    0,          # tpIdx
    1,          # version
    0,
    50,
    0,
    1000000,
    1000000,
    100000,
    100000,
    0,
    0x17        # <-- FIXED FLAGS
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
