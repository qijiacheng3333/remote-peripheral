#ifndef MCU_TRANSPORT_H
#define MCU_TRANSPORT_H
#include <stdint.h>

typedef struct {
    int      fd;
    uint32_t tag;
    int      timeout_ms;
    int      retry;
    char     dev[64];
} mcu_handle_t;

mcu_handle_t *mcu_open(const char *dev, int baudrate);
void          mcu_close(mcu_handle_t *mcu);
void          mcu_set_timeout(mcu_handle_t *mcu, int timeout_ms, int retry);

#endif
