#include <string.h>
#include "remote_spi.h"
#include "mcu_internal.h"

#define TYPE_SPI 0x0003

int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len) {
    uint8_t param[128];
    param[0] = 0x01;
    param[1] = bus;
    param[2] = cs;
    param[3] = mode;
    param[4] = len;
    memcpy(param + 5, data, len);
    return mcu_request(mcu, TYPE_SPI, param, 5 + len, NULL, 0) < 0 ? -1 : 0;
}

int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len) {
    uint8_t param[5] = {0x00, bus, cs, mode, len};
    return mcu_request(mcu, TYPE_SPI, param, 5, out_buf, len);
}

int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len) {
    uint8_t param[128];
    param[0] = 0x02;
    param[1] = bus;
    param[2] = cs;
    param[3] = mode;
    param[4] = len;
    memcpy(param + 5, tx_buf, len);
    return mcu_request(mcu, TYPE_SPI, param, 5 + len, rx_buf, len);
}
