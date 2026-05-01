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

int main(void) {
    printf("=== test_protocol ===\n");
    RUN_TEST(test_crc8_standard_vector);
    RUN_TEST(test_crc8_empty);
    RUN_TEST(test_crc8_single_byte);
    RUN_TEST(test_escape_xor_math);
    TEST_SUMMARY();
}
