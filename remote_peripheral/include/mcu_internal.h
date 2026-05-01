#ifndef MCU_INTERNAL_H
#define MCU_INTERNAL_H
#include "mcu_transport.h"

int mcu_request(mcu_handle_t *mcu, uint16_t type,
                const uint8_t *param, uint8_t param_len,
                uint8_t *resp_param, int resp_buf_size);

#endif
