#include <string.h>
#include "remote_i2c.h"
#include "mcu_internal.h"

#define TYPE_I2C 0x0002

int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len) {
    if (!data || len == 0) return -1;
    uint8_t param[128];
    param[0] = 0x01;
    param[1] = bus;
    param[2] = dev_addr;
    param[3] = reg_addr;
    param[4] = len;
    memcpy(param + 5, data, len);
    return mcu_request(mcu, TYPE_I2C, param, 5 + len, NULL, 0) < 0 ? -1 : 0;
}

int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len) {
    uint8_t param[5] = {0x00, bus, dev_addr, reg_addr, len};
    return mcu_request(mcu, TYPE_I2C, param, 5, out_buf, len);
}
