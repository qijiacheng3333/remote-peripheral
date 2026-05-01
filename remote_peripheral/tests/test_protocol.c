#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "test_harness.h"
#include "mcu_protocol.h"

static void test_crc8_standard_vector(void) {
    const uint8_t data[] = "123456789";
    ASSERT_EQ(crc8(data, 9), 0xF4);
}

static void test_crc8_empty(void) {
    const uint8_t data[] = {0x00};
    ASSERT_EQ(crc8(data, 0), 0x00);
}

static void test_crc8_single_byte(void) {
    const uint8_t data[] = {0x01};
    ASSERT_EQ(crc8(data, 1), 0x07);
}

static void test_escape_xor_math(void) {
    ASSERT_EQ((uint8_t)(0xA5 ^ 0xE0), 0x45);
    ASSERT_EQ((uint8_t)(0x5A ^ 0xE0), 0xBA);
    ASSERT_EQ((uint8_t)(0xD0 ^ 0xE0), 0x30);
    ASSERT_EQ((uint8_t)((uint8_t)(0xA5 ^ 0xE0) ^ 0xE0), 0xA5);
    ASSERT_EQ((uint8_t)((uint8_t)(0x5A ^ 0xE0) ^ 0xE0), 0x5A);
    ASSERT_EQ((uint8_t)((uint8_t)(0xD0 ^ 0xE0) ^ 0xE0), 0xD0);
}

static void test_frame_build_basic(void) {
    const uint8_t param[] = {0x01, 0x03, 0x01};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 3, buf, sizeof(buf));
    ASSERT_EQ(len > 0, 1);
    ASSERT_EQ(buf[0], 0xA5);
    ASSERT_EQ(buf[len-1], 0x5A);
}

static void test_frame_build_parse_roundtrip(void) {
    const uint8_t param[] = {0x00, 0x05};
    uint8_t buf[64];
    int len = frame_build(0x0001, 42, param, 2, buf, sizeof(buf));
    ASSERT_EQ(len > 0, 1);

    uint16_t rtype; uint32_t rtag;
    uint8_t rparam[32];
    int plen = frame_parse(buf, len, &rtype, &rtag, rparam, sizeof(rparam));
    ASSERT_EQ(plen, 2);
    ASSERT_EQ(rtype, 0x0001);
    ASSERT_EQ(rtag, 42);
    ASSERT_EQ(rparam[0], 0x00);
    ASSERT_EQ(rparam[1], 0x05);
}

static void test_frame_build_escapes_0xA5(void) {
    const uint8_t param[] = {0xA5};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 1, buf, sizeof(buf));
    ASSERT_EQ(len > 0, 1);
    /* No raw 0xA5 in body (between delimiters) */
    for (int i = 1; i < len - 1; i++) {
        ASSERT_EQ(buf[i] == 0xA5 ? 1 : 0, 0);
    }
    /* 0xD0 escape marker must appear in body */
    int found = 0;
    for (int i = 1; i < len - 1; i++) { if (buf[i] == 0xD0) found = 1; }
    ASSERT_EQ(found, 1);
}

static void test_frame_build_rejects_null_param(void) {
    uint8_t buf[64];
    ASSERT_EQ(frame_build(0x0001, 0, NULL, 0, buf, sizeof(buf)), -1);
}

static void test_frame_build_rejects_oversized_param(void) {
    uint8_t big[129];
    uint8_t buf[512];
    ASSERT_EQ(frame_build(0x0001, 0, big, 129, buf, sizeof(buf)), -1);
}

static void test_frame_parse_bad_start_delimiter(void) {
    uint8_t bad[] = {0xFF, 0x01, 0x5A};
    uint16_t t; uint32_t g; uint8_t p[32];
    ASSERT_EQ(frame_parse(bad, 3, &t, &g, p, 32), -1);
}

static void test_frame_parse_crc_mismatch(void) {
    const uint8_t param[] = {0x01};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 1, buf, sizeof(buf));
    buf[2] ^= 0xFF;
    uint16_t t; uint32_t g; uint8_t p[32];
    ASSERT_EQ(frame_parse(buf, len, &t, &g, p, 32), -1);
}

static void test_frame_build_parse_all_types(void) {
    const uint16_t types[] = {0x0001, 0x0002, 0x0003, 0x0004};
    const uint8_t param[] = {0x00};
    for (int i = 0; i < 4; i++) {
        uint8_t buf[64];
        int len = frame_build(types[i], (uint32_t)i, param, 1, buf, sizeof(buf));
        ASSERT_EQ(len > 0, 1);
        uint16_t rtype; uint32_t rtag; uint8_t rp[32];
        int plen = frame_parse(buf, len, &rtype, &rtag, rp, sizeof(rp));
        ASSERT_EQ(plen, 1);
        ASSERT_EQ(rtype, types[i]);
        ASSERT_EQ(rtag, (uint32_t)i);
    }
}

int main(void) {
    printf("=== test_protocol ===\n");
    RUN_TEST(test_crc8_standard_vector);
    RUN_TEST(test_crc8_empty);
    RUN_TEST(test_crc8_single_byte);
    RUN_TEST(test_escape_xor_math);
    RUN_TEST(test_frame_build_basic);
    RUN_TEST(test_frame_build_parse_roundtrip);
    RUN_TEST(test_frame_build_escapes_0xA5);
    RUN_TEST(test_frame_build_rejects_null_param);
    RUN_TEST(test_frame_build_rejects_oversized_param);
    RUN_TEST(test_frame_parse_bad_start_delimiter);
    RUN_TEST(test_frame_parse_crc_mismatch);
    RUN_TEST(test_frame_build_parse_all_types);
    TEST_SUMMARY();
}
