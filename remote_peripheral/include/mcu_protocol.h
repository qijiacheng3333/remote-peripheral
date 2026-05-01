#ifndef MCU_PROTOCOL_H
#define MCU_PROTOCOL_H
#include <stdint.h>

uint8_t crc8(const uint8_t *data, int len);

int frame_build(uint16_t type, uint32_t tag,
                const uint8_t *param, uint8_t param_len,
                uint8_t *out_buf, int buf_size);

int frame_parse(const uint8_t *in_buf, int in_len,
                uint16_t *out_type, uint32_t *out_tag,
                uint8_t *out_param, int param_buf_size);

#endif
