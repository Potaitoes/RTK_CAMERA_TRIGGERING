#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

struct TIM_TM2 {
    uint8_t ch;
    uint8_t flags;
    bool mode;
    bool run;
    bool newFallingEdge;
    uint8_t timeBase;
    bool utc;
    bool time;
    bool newRisingEdge;
    uint16_t count;
    uint16_t wnR;
    uint16_t wnF;
    uint32_t towMsR;
    uint32_t towSubMsR;
    uint32_t towMsF;
    uint32_t towSubMsF;
    uint32_t accEst;
};

class UBXParser {
public:
    UBXParser();
    bool feed_byte(uint8_t b);
    bool parse_message(const std::vector<uint8_t>& payload, TIM_TM2& tm2);

    std::vector<uint8_t> payload;

private:
    enum State {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CKA, CKB
    };

    State state;
    uint8_t msg_class, msg_id;
    uint16_t msg_len, msg_idx;
    uint8_t ck_a, ck_b, rx_ck_a, rx_ck_b;

    void checksum_update(uint8_t b);
    uint16_t read_u16_le(const uint8_t* p);
    uint32_t read_u32_le(const uint8_t* p);
};
