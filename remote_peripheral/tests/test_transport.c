#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include "test_harness.h"
#include "mcu_transport.h"
#include "mcu_protocol.h"
#include "mcu_internal.h"

/* Build a valid response frame and write it into mock_fd */
static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *param, uint8_t len) {
    uint8_t frame[300];
    int flen = frame_build(type, tag, param, len, frame, sizeof(frame));
    write(mock_fd, frame, flen);
}

static void test_mcu_close_null_safe(void) {
    mcu_close(NULL);
    _pass++;
}

static void test_mcu_set_timeout(void) {
    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    mcu->fd = -1;
    mcu_set_timeout(mcu, 1000, 3);
    ASSERT_EQ(mcu->timeout_ms, 1000);
    ASSERT_EQ(mcu->retry, 3);
    free(mcu);
}

static void test_mcu_set_timeout_null_safe(void) {
    mcu_set_timeout(NULL, 500, 2);
    _pass++;
}

static void test_mcu_open_bad_device(void) {
    mcu_handle_t *h = mcu_open("/dev/nonexistent_uart_xyz", 115200);
    ASSERT_EQ(h == NULL ? 1 : 0, 1);
    mcu_close(h);
}

static void test_mcu_request_write_success(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t mcu = {0};
    mcu.fd = sv[0]; mcu.timeout_ms = 200; mcu.retry = 0; mcu.tag = 0;

    const uint8_t resp[] = {0x00};
    inject_response(sv[1], 0x0001, 0, resp, 1);

    const uint8_t param[] = {0x01, 0x03, 0x01};
    int result = mcu_request(&mcu, 0x0001, param, 3, NULL, 0);
    ASSERT_EQ(result, 0);
    ASSERT_EQ(mcu.tag, 1);

    close(sv[0]); close(sv[1]);
}

static void test_mcu_request_read_returns_data(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t mcu = {0};
    mcu.fd = sv[0]; mcu.timeout_ms = 200; mcu.retry = 0; mcu.tag = 5;

    const uint8_t resp[] = {0x00, 0x01};
    inject_response(sv[1], 0x0001, 5, resp, 2);

    const uint8_t param[] = {0x00, 0x07};
    uint8_t out[4];
    int result = mcu_request(&mcu, 0x0001, param, 2, out, sizeof(out));
    ASSERT_EQ(result, 1);
    ASSERT_EQ(out[0], 0x01);
    ASSERT_EQ(mcu.tag, 6);

    close(sv[0]); close(sv[1]);
}

static void test_mcu_request_mcu_failure(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t mcu = {0};
    mcu.fd = sv[0]; mcu.timeout_ms = 200; mcu.retry = 0; mcu.tag = 0;

    const uint8_t resp[] = {0xFF};
    inject_response(sv[1], 0x0001, 0, resp, 1);

    const uint8_t param[] = {0x01, 0x00, 0x00};
    int result = mcu_request(&mcu, 0x0001, param, 3, NULL, 0);
    ASSERT_EQ(result, -1);

    close(sv[0]); close(sv[1]);
}

static void test_mcu_request_timeout(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t mcu = {0};
    mcu.fd = sv[0]; mcu.timeout_ms = 50; mcu.retry = 0; mcu.tag = 0;

    /* No response injected */
    const uint8_t param[] = {0x01, 0x00, 0x01};
    int result = mcu_request(&mcu, 0x0001, param, 3, NULL, 0);
    ASSERT_EQ(result, -1);

    close(sv[0]); close(sv[1]);
}

static void test_mcu_request_null_handle(void) {
    const uint8_t param[] = {0x00};
    ASSERT_EQ(mcu_request(NULL, 0x0001, param, 1, NULL, 0), -1);
}

int main(void) {
    printf("=== test_transport ===\n");
    RUN_TEST(test_mcu_close_null_safe);
    RUN_TEST(test_mcu_set_timeout);
    RUN_TEST(test_mcu_set_timeout_null_safe);
    RUN_TEST(test_mcu_open_bad_device);
    RUN_TEST(test_mcu_request_write_success);
    RUN_TEST(test_mcu_request_read_returns_data);
    RUN_TEST(test_mcu_request_mcu_failure);
    RUN_TEST(test_mcu_request_timeout);
    RUN_TEST(test_mcu_request_null_handle);
    TEST_SUMMARY();
}
