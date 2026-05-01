#ifndef REMOTE_PWM_H
#define REMOTE_PWM_H
#include <stdint.h>
#include "mcu_transport.h"

int pwm_start(mcu_handle_t *mcu, uint8_t channel,
              uint32_t freq_hz, uint16_t duty_ppt);

int pwm_stop(mcu_handle_t *mcu, uint8_t channel);

int pwm_get(mcu_handle_t *mcu, uint8_t channel,
            uint32_t *out_freq_hz, uint16_t *out_duty_ppt);

#endif
