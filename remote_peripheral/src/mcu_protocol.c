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

int frame_build(uint16_t type, uint32_t tag,
                const uint8_t *param, uint8_t param_len,
                uint8_t *out_buf, int buf_size) {
    if (!param || param_len == 0 || param_len > 128) return -1;
    if (!out_buf || buf_size < 1) return -1;

    /* A frame */
    uint8_t a[2 + 4 + 128];
    a[0] = (uint8_t)(type >> 8);
    a[1] = (uint8_t)(type & 0xFF);
    a[2] = (uint8_t)(tag >> 24);
    a[3] = (uint8_t)(tag >> 16);
    a[4] = (uint8_t)(tag >> 8);
    a[5] = (uint8_t)(tag & 0xFF);
    memcpy(a + 6, param, param_len);
    int a_len = 6 + param_len;

    /* B frame: A + CRC8(A) */
    uint8_t b[2 + 4 + 128 + 1];
    memcpy(b, a, a_len);
    b[a_len] = crc8(a, a_len);
    int b_len = a_len + 1;

    /* C frame: escape B */
    uint8_t c[2 * (2 + 4 + 128 + 1)];
    int c_len = escape_bytes(b, b_len, c, (int)sizeof(c));
    if (c_len < 0) return -1;

    /* D frame: 0xA5 + C + 0x5A */
    if (1 + c_len + 1 > buf_size) return -1;
    out_buf[0] = 0xA5;
    memcpy(out_buf + 1, c, c_len);
    out_buf[1 + c_len] = 0x5A;
    return 1 + c_len + 1;
}

int frame_parse(const uint8_t *in_buf, int in_len,
                uint16_t *out_type, uint32_t *out_tag,
                uint8_t *out_param, int param_buf_size) {
    if (!in_buf || in_len < 4) return -1;
    if (in_buf[0] != 0xA5 || in_buf[in_len - 1] != 0x5A) return -1;

    /* Unescape C frame -> B frame */
    const uint8_t *c = in_buf + 1;
    int c_len = in_len - 2;
    uint8_t b[2 + 4 + 128 + 1];
    int b_len = unescape_bytes(c, c_len, b, (int)sizeof(b));
    if (b_len < 0) return -1;

    /* B = A + CRC; need at least Type(2)+Tag(4)+Param(1)+CRC(1) = 8 bytes */
    if (b_len < 8) return -1;
    int a_len = b_len - 1;

    /* Verify CRC */
    if (crc8(b, a_len) != b[b_len - 1]) return -1;

    /* Parse A frame */
    *out_type = ((uint16_t)b[0] << 8) | b[1];
    *out_tag  = ((uint32_t)b[2] << 24) | ((uint32_t)b[3] << 16)
              | ((uint32_t)b[4] << 8)  |  (uint32_t)b[5];
    int param_len = a_len - 6;
    if (param_len > param_buf_size) return -1;
    if (out_param) memcpy(out_param, b + 6, param_len);
    return param_len;
}
