#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>
#include <string.h>

static int _pass = 0, _fail = 0;

#define ASSERT_EQ(actual, expected) do { \
    if ((long long)(actual) != (long long)(expected)) { \
        fprintf(stderr, "  FAIL %s:%d  expected=%lld got=%lld\n", \
                __FILE__, __LINE__, (long long)(expected), (long long)(actual)); \
        _fail++; \
    } else { _pass++; } \
} while(0)

#define ASSERT_MEM_EQ(a, b, n) do { \
    if (memcmp((a),(b),(n)) != 0) { \
        fprintf(stderr, "  FAIL %s:%d  buffer mismatch\n", __FILE__, __LINE__); \
        _fail++; \
    } else { _pass++; } \
} while(0)

#define RUN_TEST(fn) do { printf("  " #fn " ... "); fn(); printf("ok\n"); } while(0)

#define TEST_SUMMARY() do { \
    printf("\n%d passed, %d failed\n", _pass, _fail); \
    return _fail > 0 ? 1 : 0; \
} while(0)

#endif
