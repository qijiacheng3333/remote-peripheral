#include <stddef.h>
#include "remote_pwm.h"
#include "mcu_internal.h"

#define TYPE_PWM 0x0004

int pwm_start(mcu_handle_t *mcu, uint8_t channel, uint32_t freq_hz, uint16_t duty_ppt) {
    uint8_t param[8];
    param[0] = 0x01;
    param[1] = channel;
    param[2] = (freq_hz >> 24) & 0xFF;
    param[3] = (freq_hz >> 16) & 0xFF;
    param[4] = (freq_hz >> 8)  & 0xFF;
    param[5] =  freq_hz        & 0xFF;
    param[6] = (duty_ppt >> 8) & 0xFF;
    param[7] =  duty_ppt       & 0xFF;
    return mcu_request(mcu, TYPE_PWM, param, 8, NULL, 0) < 0 ? -1 : 0;
}

int pwm_stop(mcu_handle_t *mcu, uint8_t channel) {
    uint8_t param[8] = {0x00, channel, 0, 0, 0, 0, 0, 0};
    return mcu_request(mcu, TYPE_PWM, param, 8, NULL, 0) < 0 ? -1 : 0;
}

int pwm_get(mcu_handle_t *mcu, uint8_t channel, uint32_t *out_freq_hz, uint16_t *out_duty_ppt) {
    if (!mcu) return -1;
    uint8_t param[8] = {0x02, channel, 0, 0, 0, 0, 0, 0};
    uint8_t resp[6];
    int r = mcu_request(mcu, TYPE_PWM, param, 8, resp, sizeof(resp));
    if (r < 6) return -1;
    *out_freq_hz  = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
                    ((uint32_t)resp[2] << 8)  |  (uint32_t)resp[3];
    *out_duty_ppt = ((uint16_t)resp[4] << 8) | resp[5];
    return 0;
}
