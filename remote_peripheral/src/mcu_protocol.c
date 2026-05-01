#include <string.h>
#include "mcu_protocol.h"

uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int escape_bytes(const uint8_t *in, int in_len,
                        uint8_t *out, int out_size) {
    int n = 0;
    for (int i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if (b == 0xA5 || b == 0x5A || b == 0xD0) {
            if (n + 2 > out_size) return -1;
            out[n++] = 0xD0;
            out[n++] = (uint8_t)(b ^ 0xE0);
        } else {
            if (n + 1 > out_size) return -1;
            out[n++] = b;
        }
    }
    return n;
}

static int unescape_bytes(const uint8_t *in, int in_len,
                          uint8_t *out, int out_size) {
    int n = 0;
    for (int i = 0; i < in_len; i++) {
        if (in[i] == 0xD0) {
            if (i + 1 >= in_len) return -1;
            if (n + 1 > out_size) return -1;
            out[n++] = (uint8_t)(in[++i] ^ 0xE0);
        } else {
            if (n + 1 > out_size) return -1;
            out[n++] = in[i];
        }
    }
    return n;
}
