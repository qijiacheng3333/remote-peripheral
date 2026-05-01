#ifndef REMOTE_GPIO_H
#define REMOTE_GPIO_H
#include <stdint.h>
#include "mcu_transport.h"

#define GPIO_LOW    0
#define GPIO_HIGH   1
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

int gpio_set_direction(mcu_handle_t *mcu, uint8_t pin, uint8_t direction);
int gpio_write(mcu_handle_t *mcu, uint8_t pin, uint8_t value);
int gpio_read(mcu_handle_t *mcu, uint8_t pin);

#endif
