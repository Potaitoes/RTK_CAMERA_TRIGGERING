#include "ubx_parser.h"

UBXParser::UBXParser()
    : state(SYNC1), msg_class(0), msg_id(0), msg_len(0), msg_idx(0),
      ck_a(0), ck_b(0), rx_ck_a(0), rx_ck_b(0) {
    payload.reserve(256);
}

void UBXParser::checksum_update(uint8_t b) {
    ck_a = (ck_a + b) & 0xFF;
    ck_b = (ck_b + ck_a) & 0xFF;
}

uint16_t UBXParser::read_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t UBXParser::read_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

bool UBXParser::feed_byte(uint8_t b) {
    switch (state) {
        case SYNC1:
            if (b == 0xB5) state = SYNC2;
            break;

        case SYNC2:
            state = (b == 0x62) ? CLASS : SYNC1;
            break;

        case CLASS:
            msg_class = b;
            ck_a = 0;
            ck_b = 0;
            checksum_update(b);
            state = ID;
            break;

        case ID:
            msg_id = b;
            checksum_update(b);
            state = LEN1;
            break;

        case LEN1:
            msg_len = b;
            checksum_update(b);
            state = LEN2;
            break;

        case LEN2:
            msg_len |= ((uint16_t)b << 8);
            checksum_update(b);
            msg_idx = 0;
            payload.clear();
            if (msg_len > 256) {
                state = SYNC1;
            } else if (msg_len == 0) {
                state = CKA;
            } else {
                state = PAYLOAD;
            }
            break;

        case PAYLOAD:
            payload.push_back(b);
            checksum_update(b);
            msg_idx++;
            if (msg_idx >= msg_len) {
                state = CKA;
            }
            break;

        case CKA:
            rx_ck_a = b;
            state = CKB;
            break;

        case CKB:
            rx_ck_b = b;
            if (rx_ck_a == ck_a && rx_ck_b == ck_b) {
                if (msg_class == 0x0D && msg_id == 0x03) {  // TIM-TM2
                    state = SYNC1;
                    return true;  // Valid message received
                }
            }
            state = SYNC1;
            break;
    }
    return false;
}

bool UBXParser::parse_message(const std::vector<uint8_t>& pay, TIM_TM2& tm2) {
    if (pay.size() < 28) {
        return false;
    }

    tm2.ch         = pay[0];
    tm2.flags      = pay[1];
    tm2.mode       = (pay[1] & 0x01) != 0;
    tm2.run        = (pay[1] & 0x02) != 0;
    tm2.newFallingEdge = (pay[1] & 0x04) != 0;
    tm2.timeBase   = (pay[1] >> 3) & 0x03;
    tm2.utc        = (pay[1] & 0x20) != 0;
    tm2.time       = (pay[1] & 0x40) != 0;
    tm2.newRisingEdge = (pay[1] & 0x80) != 0;
    tm2.count      = read_u16_le(&pay[2]);
    tm2.wnR        = read_u16_le(&pay[4]);
    tm2.wnF        = read_u16_le(&pay[6]);
    tm2.towMsR     = read_u32_le(&pay[8]);
    tm2.towSubMsR  = read_u32_le(&pay[12]);
    tm2.towMsF     = read_u32_le(&pay[16]);
    tm2.towSubMsF  = read_u32_le(&pay[20]);
    tm2.accEst     = read_u32_le(&pay[24]);

    return true;
}
