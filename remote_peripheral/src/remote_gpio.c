#include <stddef.h>
#include "remote_gpio.h"
#include "mcu_internal.h"

#define TYPE_GPIO 0x0001

int gpio_set_direction(mcu_handle_t *mcu, uint8_t pin, uint8_t direction) {
    uint8_t param[3] = {0x02, pin, direction};
    return mcu_request(mcu, TYPE_GPIO, param, 3, NULL, 0) < 0 ? -1 : 0;
}

int gpio_write(mcu_handle_t *mcu, uint8_t pin, uint8_t value) {
    uint8_t param[3] = {0x01, pin, value};
    return mcu_request(mcu, TYPE_GPIO, param, 3, NULL, 0) < 0 ? -1 : 0;
}

int gpio_read(mcu_handle_t *mcu, uint8_t pin) {
    uint8_t param[2] = {0x00, pin};
    uint8_t resp[1];
    int r = mcu_request(mcu, TYPE_GPIO, param, 2, resp, sizeof(resp));
    if (r < 1) return -1;
    return resp[0];
}
