#include "runcam_crc.h"

uint8_t runcam_crc8_dvb_s2(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
            else crc <<= 1;
        }
    }
    return crc;
}


