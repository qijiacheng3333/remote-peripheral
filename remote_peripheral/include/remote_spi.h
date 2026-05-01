#ifndef REMOTE_SPI_H
#define REMOTE_SPI_H
#include <stdint.h>
#include "mcu_transport.h"

int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len);

int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len);

int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len);

#endif
