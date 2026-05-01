#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include "mcu_transport.h"
#include "mcu_protocol.h"
#include "mcu_internal.h"

static speed_t baudrate_to_speed(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B0;
    }
}

mcu_handle_t *mcu_open(const char *dev, int baudrate) {
    speed_t speed = baudrate_to_speed(baudrate);
    if (speed == B0) return NULL;

    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return NULL;

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { close(fd); return NULL; }

    cfmakeraw(&tio);
    tio.c_cflag &= (tcflag_t)~CRTSCTS;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return NULL; }
    tcflush(fd, TCIOFLUSH);

    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    if (!mcu) { close(fd); return NULL; }
    mcu->fd = fd;
    mcu->timeout_ms = 500;
    mcu->retry = 2;
    strncpy(mcu->dev, dev, sizeof(mcu->dev) - 1);
    return mcu;
}

void mcu_close(mcu_handle_t *mcu) {
    if (!mcu) return;
    if (mcu->fd >= 0) close(mcu->fd);
    free(mcu);
}

void mcu_set_timeout(mcu_handle_t *mcu, int timeout_ms, int retry) {
    if (!mcu) return;
    mcu->timeout_ms = timeout_ms;
    mcu->retry = retry;
}

static void drain_fd(int fd) {
    uint8_t tmp[64];
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    while (read(fd, tmp, sizeof(tmp)) > 0) {}
    fcntl(fd, F_SETFL, flags);
}

static int recv_frame(int fd, int timeout_ms, uint8_t *buf, int buf_size) {
    struct timeval deadline, now, remaining;
    gettimeofday(&deadline, NULL);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_usec += (timeout_ms % 1000) * 1000;
    if (deadline.tv_usec >= 1000000) {
        deadline.tv_sec++;
        deadline.tv_usec -= 1000000;
    }

    int len = 0;
    int in_frame = 0;

    while (1) {
        gettimeofday(&now, NULL);
        remaining.tv_sec  = deadline.tv_sec  - now.tv_sec;
        remaining.tv_usec = deadline.tv_usec - now.tv_usec;
        if (remaining.tv_usec < 0) {
            remaining.tv_sec--;
            remaining.tv_usec += 1000000;
        }
        if (remaining.tv_sec < 0 ||
            (remaining.tv_sec == 0 && remaining.tv_usec <= 0))
            return -1;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int ret = select(fd + 1, &rfds, NULL, NULL, &remaining);
        if (ret <= 0) return -1;

        uint8_t byte;
        if (read(fd, &byte, 1) != 1) return -1;

        if (!in_frame) {
            if (byte == 0xA5) {
                len = 0;
                buf[len++] = byte;
                in_frame = 1;
            }
        } else {
            if (len >= buf_size) { in_frame = 0; len = 0; continue; }
            buf[len++] = byte;
            if (byte == 0x5A) return len;
        }
    }
}

int mcu_request(mcu_handle_t *mcu, uint16_t type,
                const uint8_t *param, uint8_t param_len,
                uint8_t *resp_param, int resp_buf_size) {
    if (!mcu || mcu->fd < 0) return -1;

    uint8_t frame_buf[300];
    int frame_len = frame_build(type, mcu->tag, param, param_len,
                                frame_buf, (int)sizeof(frame_buf));
    if (frame_len < 0) return -1;

    uint8_t resp_buf[300];
    uint16_t resp_type;
    uint32_t resp_tag;
    uint8_t tmp_param[128];

    for (int attempt = 0; attempt <= mcu->retry; attempt++) {
        if (attempt > 0) drain_fd(mcu->fd);
        if (write(mcu->fd, frame_buf, frame_len) != frame_len) continue;

        int resp_len = recv_frame(mcu->fd, mcu->timeout_ms,
                                  resp_buf, (int)sizeof(resp_buf));
        if (resp_len < 0) continue;

        int p_len = frame_parse(resp_buf, resp_len,
                                &resp_type, &resp_tag,
                                tmp_param, (int)sizeof(tmp_param));
        if (p_len < 0) continue;
        if (resp_tag != mcu->tag || resp_type != type) continue;

        mcu->tag++;

        if (p_len < 1 || tmp_param[0] == 0xFF) return -1;

        int data_len = p_len - 1;
        if (data_len > resp_buf_size) return -1;
        if (data_len > 0 && resp_param)
            memcpy(resp_param, tmp_param + 1, data_len);
        return data_len;
    }

    mcu->tag++;
    return -1;
}
