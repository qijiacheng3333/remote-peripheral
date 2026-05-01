#ifndef REMOTE_I2C_H
#define REMOTE_I2C_H
#include <stdint.h>
#include "mcu_transport.h"

int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len);

int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len);

#endif
