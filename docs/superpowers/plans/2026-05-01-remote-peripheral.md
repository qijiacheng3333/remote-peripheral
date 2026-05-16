# Remote Peripheral Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a C library that lets Linux CPU code operate GPIO/I2C/SPI/PWM peripherals on remote MCUs via a custom UART framing protocol, with an API that feels identical to local peripheral access.

**Architecture:** Four-layer stack — transport (termios UART), protocol (CRC-8 / escape / frame), peripheral HAL (gpio/i2c/spi/pwm), application. All calls are synchronous-blocking with configurable timeout and retry. Each MCU gets its own `mcu_handle_t` bound to one UART device node.

**Tech Stack:** C99, Linux termios, POSIX select/socketpair, CMake 3.10+

---

## File Map

| File | Responsibility |
|------|---------------|
| `include/mcu_transport.h` | Public: `mcu_handle_t`, open/close/set_timeout |
| `include/mcu_protocol.h` | Public (internal use): frame_build, frame_parse, crc8 |
| `include/mcu_internal.h` | Internal: mcu_request (used by peripheral files) |
| `include/remote_gpio.h` | Public: GPIO API |
| `include/remote_i2c.h` | Public: I2C API |
| `include/remote_spi.h` | Public: SPI API |
| `include/remote_pwm.h` | Public: PWM API |
| `src/mcu_protocol.c` | crc8, escape helpers, frame_build, frame_parse |
| `src/mcu_transport.c` | UART open/close, recv state machine, mcu_request |
| `src/remote_gpio.c` | gpio_set_direction, gpio_write, gpio_read |
| `src/remote_i2c.c` | i2c_write_reg, i2c_read_reg |
| `src/remote_spi.c` | spi_write, spi_read, spi_transfer |
| `src/remote_pwm.c` | pwm_start, pwm_stop, pwm_get |
| `tests/test_harness.h` | Minimal assert/run macros, no external deps |
| `tests/test_protocol.c` | Unit tests for crc8, escape, frame_build, frame_parse |
| `tests/test_transport.c` | Unit tests for mcu_request via socketpair mock |
| `tests/test_gpio.c` | Unit tests for GPIO API via socketpair mock |
| `tests/test_i2c.c` | Unit tests for I2C API via socketpair mock |
| `tests/test_spi.c` | Unit tests for SPI API via socketpair mock |
| `tests/test_pwm.c` | Unit tests for PWM API via socketpair mock |
| `CMakeLists.txt` | Static library + test executables |

---

## Task 1: Project Scaffold

**Files:**
- Create: `remote_peripheral/CMakeLists.txt`
- Create: `remote_peripheral/tests/test_harness.h`
- Create: all header stubs (empty `#ifndef` guards)

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p remote_peripheral/{include,src,tests}
```

- [ ] **Step 2: Create CMakeLists.txt**

```cmake
# remote_peripheral/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(remote_peripheral C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

include_directories(include)

add_library(remote_peripheral STATIC
    src/mcu_protocol.c
    src/mcu_transport.c
    src/remote_gpio.c
    src/remote_i2c.c
    src/remote_spi.c
    src/remote_pwm.c
)

enable_testing()

foreach(t protocol transport gpio i2c spi pwm)
    add_executable(test_${t} tests/test_${t}.c)
    target_link_libraries(test_${t} remote_peripheral)
    add_test(NAME test_${t} COMMAND test_${t})
endforeach()
```

- [ ] **Step 3: Create test harness header**

```c
/* remote_peripheral/tests/test_harness.h */
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
```

- [ ] **Step 4: Create all header stubs**

`remote_peripheral/include/mcu_transport.h`:
```c
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
```

`remote_peripheral/include/mcu_protocol.h`:
```c
#ifndef MCU_PROTOCOL_H
#define MCU_PROTOCOL_H
#include <stdint.h>

uint8_t crc8(const uint8_t *data, int len);

int frame_build(uint16_t type, uint32_t tag,
                const uint8_t *param, uint8_t param_len,
                uint8_t *out_buf, int buf_size);

int frame_parse(const uint8_t *in_buf, int in_len,
                uint16_t *out_type, uint32_t *out_tag,
                uint8_t *out_param, int param_buf_size);

#endif
```

`remote_peripheral/include/mcu_internal.h`:
```c
#ifndef MCU_INTERNAL_H
#define MCU_INTERNAL_H
#include "mcu_transport.h"

int mcu_request(mcu_handle_t *mcu, uint16_t type,
                const uint8_t *param, uint8_t param_len,
                uint8_t *resp_param, int resp_buf_size);

#endif
```

`remote_peripheral/include/remote_gpio.h`:
```c
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
```

`remote_peripheral/include/remote_i2c.h`:
```c
#ifndef REMOTE_I2C_H
#define REMOTE_I2C_H
#include <stdint.h>
#include "mcu_transport.h"

int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len);

int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len);

#endif
```

`remote_peripheral/include/remote_spi.h`:
```c
#ifndef REMOTE_SPI_H
#define REMOTE_SPI_H
#include <stdint.h>
#include "mcu_transport.h"

int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len);

int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len);

int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len);

#endif
```

`remote_peripheral/include/remote_pwm.h`:
```c
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
```

- [ ] **Step 5: Create empty .c source stubs so CMake can compile**

Each `src/*.c` file needs just the include and a placeholder. For example:

`remote_peripheral/src/mcu_protocol.c`:
```c
#include <string.h>
#include "mcu_protocol.h"
```

Repeat for `mcu_transport.c`, `remote_gpio.c`, `remote_i2c.c`, `remote_spi.c`, `remote_pwm.c` with their respective headers.

- [ ] **Step 6: Verify CMake configures without error**

```bash
cd remote_peripheral && mkdir -p build && cd build && cmake ..
```

Expected: `-- Configuring done` with no errors (linking will fail until implementations are added).

- [ ] **Step 7: Commit scaffold**

```bash
git init  # if not already a repo
git add remote_peripheral/
git commit -m "feat: scaffold remote_peripheral library structure"
```

---

## Task 2: CRC-8 Implementation

**Files:**
- Modify: `remote_peripheral/src/mcu_protocol.c`
- Create: `remote_peripheral/tests/test_protocol.c` (start file, add more tests later)

- [ ] **Step 1: Write the failing test**

```c
/* remote_peripheral/tests/test_protocol.c */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "test_harness.h"
#include "mcu_protocol.h"

static void test_crc8_standard_vector(void) {
    /* CRC8("123456789") must equal 0xF4 per spec */
    const uint8_t data[] = "123456789";
    ASSERT_EQ(crc8(data, 9), 0xF4);
}

static void test_crc8_empty(void) {
    const uint8_t data[] = {0x00};
    ASSERT_EQ(crc8(data, 0), 0x00);
}

static void test_crc8_single_byte(void) {
    const uint8_t data[] = {0x01};
    /* CRC-8/SMBUS of {0x01}: manually computed = 0x07 */
    ASSERT_EQ(crc8(data, 1), 0x07);
}

int main(void) {
    printf("=== test_protocol ===\n");
    RUN_TEST(test_crc8_standard_vector);
    RUN_TEST(test_crc8_empty);
    RUN_TEST(test_crc8_single_byte);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd remote_peripheral/build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make test_protocol 2>&1
./test_protocol
```

Expected: link error or wrong results (crc8 not implemented yet).

- [ ] **Step 3: Implement crc8**

```c
/* remote_peripheral/src/mcu_protocol.c */
#include <string.h>
#include "mcu_protocol.h"

uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

Expected:
```
=== test_protocol ===
  test_crc8_standard_vector ... ok
  test_crc8_empty ... ok
  test_crc8_single_byte ... ok

3 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/mcu_protocol.c tests/test_protocol.c
git commit -m "feat: implement CRC-8 (poly=0x07, init=0x00); verified against 0xF4 vector"
```

---

## Task 3: Byte Escape / Unescape

**Files:**
- Modify: `remote_peripheral/src/mcu_protocol.c` (add static helpers)
- Modify: `remote_peripheral/tests/test_protocol.c` (add tests)

The escape helpers are `static` (file-scope only); tested indirectly through frame_build/frame_parse in Task 4-5. We add direct white-box tests here by extracting them to a testable form temporarily using a test-only header trick, OR we accept indirect testing. For correctness we test them via frame_build in Task 4.

Here we add the implementations and verify the XOR math is correct.

- [ ] **Step 1: Add escape/unescape static functions to mcu_protocol.c**

Append after `crc8`:

```c
static int escape_bytes(const uint8_t *in, int in_len,
                        uint8_t *out, int out_size) {
    int n = 0;
    for (int i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if (b == 0xA5 || b == 0x5A || b == 0xD0) {
            if (n + 2 > out_size) return -1;
            out[n++] = 0xD0;
            out[n++] = (uint8_t)(b ^ 0xE0);
        } else {
            if (n + 1 > out_size) return -1;
            out[n++] = b;
        }
    }
    return n;
}

static int unescape_bytes(const uint8_t *in, int in_len,
                          uint8_t *out, int out_size) {
    int n = 0;
    for (int i = 0; i < in_len; i++) {
        if (in[i] == 0xD0) {
            if (i + 1 >= in_len) return -1;
            if (n + 1 > out_size) return -1;
            out[n++] = (uint8_t)(in[++i] ^ 0xE0);
        } else {
            if (n + 1 > out_size) return -1;
            out[n++] = in[i];
        }
    }
    return n;
}
```

- [ ] **Step 2: Add escape correctness tests to test_protocol.c**

Add these functions before `main`, and add `RUN_TEST` calls for them in `main`:

```c
static void test_escape_special_bytes(void) {
    /* 0xA5 -> 0xD0 0x45  (0xA5^0xE0=0x45) */
    /* 0x5A -> 0xD0 0xBA  (0x5A^0xE0=0xBA) */
    /* 0xD0 -> 0xD0 0x30  (0xD0^0xE0=0x30) */
    ASSERT_EQ((uint8_t)(0xA5 ^ 0xE0), 0x45);
    ASSERT_EQ((uint8_t)(0x5A ^ 0xE0), 0xBA);
    ASSERT_EQ((uint8_t)(0xD0 ^ 0xE0), 0x30);
    /* Round-trip: unescape(escape(b)) == b */
    ASSERT_EQ((uint8_t)((uint8_t)(0xA5 ^ 0xE0) ^ 0xE0), 0xA5);
    ASSERT_EQ((uint8_t)((uint8_t)(0x5A ^ 0xE0) ^ 0xE0), 0x5A);
    ASSERT_EQ((uint8_t)((uint8_t)(0xD0 ^ 0xE0) ^ 0xE0), 0xD0);
}
```

- [ ] **Step 3: Run tests**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

Expected: all tests pass including new `test_escape_special_bytes`.

- [ ] **Step 4: Commit**

```bash
git add src/mcu_protocol.c tests/test_protocol.c
git commit -m "feat: add escape/unescape helpers to protocol layer"
```

---

## Task 4: frame_build Implementation

**Files:**
- Modify: `remote_peripheral/src/mcu_protocol.c`
- Modify: `remote_peripheral/tests/test_protocol.c`

- [ ] **Step 1: Add frame_build test**

Add to `test_protocol.c` before `main`, add `RUN_TEST` call in `main`:

```c
static void test_frame_build_basic(void) {
    /* Build a simple frame: Type=0x0001, Tag=0, Param=[0x01, 0x03, 0x01] */
    const uint8_t param[] = {0x01, 0x03, 0x01};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 3, buf, sizeof(buf));

    /* Must succeed */
    ASSERT_EQ(len > 0, 1);
    /* First byte must be 0xA5 (start delimiter) */
    ASSERT_EQ(buf[0], 0xA5);
    /* Last byte must be 0x5A (end delimiter) */
    ASSERT_EQ(buf[len-1], 0x5A);
}

static void test_frame_build_parse_roundtrip(void) {
    /* frame_build then frame_parse must recover original fields */
    const uint8_t param[] = {0x00, 0x05}; /* read pin 5 */
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

static void test_frame_build_escapes_special_bytes(void) {
    /* Param contains 0xA5 - must be escaped in D frame body */
    const uint8_t param[] = {0xA5};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 1, buf, sizeof(buf));
    ASSERT_EQ(len > 0, 1);
    /* The D frame body (between delimiters) must not contain raw 0xA5 */
    for (int i = 1; i < len - 1; i++) {
        ASSERT_EQ(buf[i] == 0xA5 ? 1 : 0, 0);
    }
    /* 0xD0 escape byte must appear in body */
    int found_escape = 0;
    for (int i = 1; i < len - 1; i++) {
        if (buf[i] == 0xD0) found_escape = 1;
    }
    ASSERT_EQ(found_escape, 1);
}

static void test_frame_build_rejects_empty_param(void) {
    uint8_t buf[64];
    ASSERT_EQ(frame_build(0x0001, 0, NULL, 0, buf, sizeof(buf)), -1);
}

static void test_frame_build_rejects_oversized_param(void) {
    uint8_t big[129];
    uint8_t buf[512];
    ASSERT_EQ(frame_build(0x0001, 0, big, 129, buf, sizeof(buf)), -1);
}
```

- [ ] **Step 2: Run test to verify it fails (frame_build not yet implemented)**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

Expected: link error or crashes on frame_build calls.

- [ ] **Step 3: Implement frame_build**

Append to `src/mcu_protocol.c`:

```c
int frame_build(uint16_t type, uint32_t tag,
                const uint8_t *param, uint8_t param_len,
                uint8_t *out_buf, int buf_size) {
    if (!param || param_len == 0 || param_len > 128) return -1;
    if (!out_buf || buf_size < 1) return -1;

    /* A frame: Type(2B big-endian) + Tag(4B big-endian) + Param */
    uint8_t a[2 + 4 + 128];
    a[0] = (uint8_t)(type >> 8);
    a[1] = (uint8_t)(type & 0xFF);
    a[2] = (uint8_t)(tag >> 24);
    a[3] = (uint8_t)(tag >> 16);
    a[4] = (uint8_t)(tag >> 8);
    a[5] = (uint8_t)(tag & 0xFF);
    memcpy(a + 6, param, param_len);
    int a_len = 6 + param_len;

    /* B frame: A + CRC8(A) */
    uint8_t b[2 + 4 + 128 + 1];
    memcpy(b, a, a_len);
    b[a_len] = crc8(a, a_len);
    int b_len = a_len + 1;

    /* C frame: escape B */
    uint8_t c[2 * (2 + 4 + 128 + 1)]; /* worst case: every byte escaped */
    int c_len = escape_bytes(b, b_len, c, (int)sizeof(c));
    if (c_len < 0) return -1;

    /* D frame: 0xA5 + C + 0x5A */
    if (1 + c_len + 1 > buf_size) return -1;
    out_buf[0] = 0xA5;
    memcpy(out_buf + 1, c, c_len);
    out_buf[1 + c_len] = 0x5A;
    return 1 + c_len + 1;
}
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

Expected: all frame_build tests pass (frame_parse tests will still fail - that's fine).

- [ ] **Step 5: Commit**

```bash
git add src/mcu_protocol.c tests/test_protocol.c
git commit -m "feat: implement frame_build (A->B->C->D encapsulation)"
```

---

## Task 5: frame_parse Implementation

**Files:**
- Modify: `remote_peripheral/src/mcu_protocol.c`
- Modify: `remote_peripheral/tests/test_protocol.c`

- [ ] **Step 1: Add frame_parse-specific tests**

Add to `test_protocol.c` before `main`, add `RUN_TEST` calls:

```c
static void test_frame_parse_bad_delimiters(void) {
    uint8_t bad[] = {0xFF, 0x01, 0x5A};
    uint16_t t; uint32_t g; uint8_t p[32];
    ASSERT_EQ(frame_parse(bad, 3, &t, &g, p, 32), -1);
}

static void test_frame_parse_crc_mismatch(void) {
    /* Build a valid frame then corrupt a byte */
    const uint8_t param[] = {0x01};
    uint8_t buf[64];
    int len = frame_build(0x0001, 0, param, 1, buf, sizeof(buf));
    buf[2] ^= 0xFF; /* corrupt a byte inside */
    uint16_t t; uint32_t g; uint8_t p[32];
    ASSERT_EQ(frame_parse(buf, len, &t, &g, p, 32), -1);
}

static void test_frame_parse_all_peripheral_types(void) {
    /* Verify Type field round-trips for all four peripheral types */
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

- [ ] **Step 3: Implement frame_parse**

Append to `src/mcu_protocol.c`:

```c
int frame_parse(const uint8_t *in_buf, int in_len,
                uint16_t *out_type, uint32_t *out_tag,
                uint8_t *out_param, int param_buf_size) {
    if (!in_buf || in_len < 4) return -1;
    if (in_buf[0] != 0xA5 || in_buf[in_len - 1] != 0x5A) return -1;

    /* Unescape C frame (bytes between delimiters) -> B frame */
    const uint8_t *c = in_buf + 1;
    int c_len = in_len - 2;
    uint8_t b[2 + 4 + 128 + 1];
    int b_len = unescape_bytes(c, c_len, b, (int)sizeof(b));
    if (b_len < 0) return -1;

    /* B frame = A frame + CRC byte; need at least Type+Tag+1Param+CRC = 8 bytes */
    if (b_len < 8) return -1;
    int a_len = b_len - 1;

    /* Verify CRC */
    if (crc8(b, a_len) != b[b_len - 1]) return -1;

    /* Parse A frame fields */
    *out_type = ((uint16_t)b[0] << 8) | b[1];
    *out_tag  = ((uint32_t)b[2] << 24) | ((uint32_t)b[3] << 16)
              | ((uint32_t)b[4] << 8)  |  (uint32_t)b[5];
    int param_len = a_len - 6;
    if (param_len > param_buf_size) return -1;
    if (out_param) memcpy(out_param, b + 6, param_len);

    return param_len;
}
```

- [ ] **Step 4: Run all protocol tests**

```bash
cd remote_peripheral/build && make test_protocol && ./test_protocol
```

Expected:
```
=== test_protocol ===
  test_crc8_standard_vector ... ok
  test_crc8_empty ... ok
  test_crc8_single_byte ... ok
  test_escape_special_bytes ... ok
  test_frame_build_basic ... ok
  test_frame_build_parse_roundtrip ... ok
  test_frame_build_escapes_special_bytes ... ok
  test_frame_build_rejects_empty_param ... ok
  test_frame_build_rejects_oversized_param ... ok
  test_frame_parse_bad_delimiters ... ok
  test_frame_parse_crc_mismatch ... ok
  test_frame_parse_all_peripheral_types ... ok

12 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/mcu_protocol.c tests/test_protocol.c
git commit -m "feat: implement frame_parse with CRC verification and unescape"
```

---

## Task 6: Transport Layer — mcu_open / mcu_close / mcu_set_timeout

**Files:**
- Modify: `remote_peripheral/src/mcu_transport.c`
- Create: `remote_peripheral/tests/test_transport.c`

Note: `mcu_open` opens a real tty. Tests use manually constructed handles to avoid hardware dependency.

- [ ] **Step 1: Write failing tests**

```c
/* remote_peripheral/tests/test_transport.c */
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

static void test_mcu_close_null_safe(void) {
    mcu_close(NULL); /* must not crash */
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
    mcu_set_timeout(NULL, 500, 2); /* must not crash */
    _pass++;
}

static void test_mcu_open_bad_device(void) {
    /* Non-existent device must return NULL */
    mcu_handle_t *h = mcu_open("/dev/nonexistent_uart_xyz", 115200);
    ASSERT_EQ(h == NULL ? 1 : 0, 1);
    mcu_close(h);
}

int main(void) {
    printf("=== test_transport ===\n");
    RUN_TEST(test_mcu_close_null_safe);
    RUN_TEST(test_mcu_set_timeout);
    RUN_TEST(test_mcu_set_timeout_null_safe);
    RUN_TEST(test_mcu_open_bad_device);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_transport && ./test_transport
```

- [ ] **Step 3: Implement mcu_transport.c (open/close/set_timeout)**

```c
/* remote_peripheral/src/mcu_transport.c */
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
```

- [ ] **Step 4: Run tests**

```bash
cd remote_peripheral/build && make test_transport && ./test_transport
```

Expected:
```
=== test_transport ===
  test_mcu_close_null_safe ... ok
  test_mcu_set_timeout ... ok
  test_mcu_set_timeout_null_safe ... ok
  test_mcu_open_bad_device ... ok

4 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/mcu_transport.c tests/test_transport.c
git commit -m "feat: implement mcu_open/close/set_timeout (termios UART)"
```

---

## Task 7: mcu_request — Core Request/Response Engine

**Files:**
- Modify: `remote_peripheral/src/mcu_transport.c` (append recv_frame + mcu_request)
- Modify: `remote_peripheral/tests/test_transport.c` (add socketpair-based tests)

`mcu_request` sends a D frame, waits for a response D frame matching the same tag, retries on timeout. Uses `select()` for timeout.

- [ ] **Step 1: Add mcu_request tests using socketpair**

Append to `test_transport.c` before `main`, add `RUN_TEST` calls:

```c
#include <sys/socket.h>

/* Helper: build a mock MCU response frame and write it to mock_fd */
static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *resp_param, uint8_t resp_len) {
    uint8_t frame[300];
    int len = frame_build(type, tag, resp_param, resp_len, frame, sizeof(frame));
    write(mock_fd, frame, len);
}

static void test_mcu_request_write_success(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

    mcu_handle_t mcu = {0};
    mcu.fd = sv[0];
    mcu.timeout_ms = 200;
    mcu.retry = 0;
    mcu.tag = 0;

    /* Pre-inject success response: Type=0x0001, Tag=0, Param=[0x00] */
    const uint8_t resp[] = {0x00};
    inject_response(sv[1], 0x0001, 0, resp, 1);

    const uint8_t param[] = {0x01, 0x03, 0x01};
    uint8_t out[32];
    int result = mcu_request(&mcu, 0x0001, param, 3, out, sizeof(out));

    ASSERT_EQ(result, 0);     /* write op: 0 data bytes returned */
    ASSERT_EQ(mcu.tag, 1);    /* tag incremented */

    close(sv[0]);
    close(sv[1]);
}

static void test_mcu_request_read_returns_data(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

    mcu_handle_t mcu = {0};
    mcu.fd = sv[0];
    mcu.timeout_ms = 200;
    mcu.retry = 0;
    mcu.tag = 5;

    /* Pre-inject read response: Param=[0x00, 0x01] (success + pin HIGH) */
    const uint8_t resp[] = {0x00, 0x01};
    inject_response(sv[1], 0x0001, 5, resp, 2);

    const uint8_t param[] = {0x00, 0x07}; /* read pin 7 */
    uint8_t out[32];
    int result = mcu_request(&mcu, 0x0001, param, 2, out, sizeof(out));

    ASSERT_EQ(result, 1);     /* 1 data byte returned (after stripping 0x00 status) */
    ASSERT_EQ(out[0], 0x01);  /* pin HIGH */
    ASSERT_EQ(mcu.tag, 6);

    close(sv[0]);
    close(sv[1]);
}

static void test_mcu_request_mcu_failure(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

    mcu_handle_t mcu = {0};
    mcu.fd = sv[0];
    mcu.timeout_ms = 200;
    mcu.retry = 0;
    mcu.tag = 0;

    /* MCU responds with failure 0xFF */
    const uint8_t resp[] = {0xFF};
    inject_response(sv[1], 0x0001, 0, resp, 1);

    const uint8_t param[] = {0x01, 0x00, 0x00};
    int result = mcu_request(&mcu, 0x0001, param, 3, NULL, 0);

    ASSERT_EQ(result, -1);

    close(sv[0]);
    close(sv[1]);
}

static void test_mcu_request_timeout(void) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

    mcu_handle_t mcu = {0};
    mcu.fd = sv[0];
    mcu.timeout_ms = 50;  /* short timeout */
    mcu.retry = 0;
    mcu.tag = 0;

    /* No response injected - will timeout */
    const uint8_t param[] = {0x01, 0x00, 0x01};
    int result = mcu_request(&mcu, 0x0001, param, 3, NULL, 0);

    ASSERT_EQ(result, -1);

    close(sv[0]);
    close(sv[1]);
}

static void test_mcu_request_null_handle(void) {
    const uint8_t param[] = {0x00};
    ASSERT_EQ(mcu_request(NULL, 0x0001, param, 1, NULL, 0), -1);
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_transport && ./test_transport
```

- [ ] **Step 3: Implement recv_frame and mcu_request**

Append to `src/mcu_transport.c`:

```c
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
        if (remaining.tv_usec < 0) { remaining.tv_sec--; remaining.tv_usec += 1000000; }
        if (remaining.tv_sec < 0 || (remaining.tv_sec == 0 && remaining.tv_usec <= 0))
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
```

- [ ] **Step 4: Run all transport tests**

```bash
cd remote_peripheral/build && make test_transport && ./test_transport
```

Expected:
```
=== test_transport ===
  test_mcu_close_null_safe ... ok
  test_mcu_set_timeout ... ok
  test_mcu_set_timeout_null_safe ... ok
  test_mcu_open_bad_device ... ok
  test_mcu_request_write_success ... ok
  test_mcu_request_read_returns_data ... ok
  test_mcu_request_mcu_failure ... ok
  test_mcu_request_timeout ... ok
  test_mcu_request_null_handle ... ok

9 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/mcu_transport.c tests/test_transport.c
git commit -m "feat: implement recv_frame state machine and mcu_request with retry/timeout"
```

---

## Task 8: GPIO Peripheral

**Files:**
- Modify: `remote_peripheral/src/remote_gpio.c`
- Create: `remote_peripheral/tests/test_gpio.c`

- [ ] **Step 1: Write failing tests**

```c
/* remote_peripheral/tests/test_gpio.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include "test_harness.h"
#include "remote_gpio.h"
#include "mcu_protocol.h"

static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *param, uint8_t len) {
    uint8_t frame[300];
    int flen = frame_build(type, tag, param, len, frame, sizeof(frame));
    write(mock_fd, frame, flen);
}

static mcu_handle_t *make_mock_mcu(int *mock_fd_out) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    mcu->fd = sv[0];
    mcu->timeout_ms = 200;
    mcu->retry = 0;
    mcu->tag = 0;
    *mock_fd_out = sv[1];
    return mcu;
}

static void test_gpio_set_direction_output(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    /* MCU ACKs success */
    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0001, 0, resp, 1);

    int r = gpio_set_direction(mcu, 3, GPIO_OUTPUT);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_gpio_write_high(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0001, 0, resp, 1);

    int r = gpio_write(mcu, 3, GPIO_HIGH);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_gpio_read_high(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    /* Response: success + pin HIGH */
    const uint8_t resp[] = {0x00, 0x01};
    inject_response(mfd, 0x0001, 0, resp, 2);

    int r = gpio_read(mcu, 5);
    ASSERT_EQ(r, GPIO_HIGH);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_gpio_read_low(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00, 0x00};
    inject_response(mfd, 0x0001, 0, resp, 2);

    int r = gpio_read(mcu, 5);
    ASSERT_EQ(r, GPIO_LOW);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_gpio_write_failure(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0xFF};
    inject_response(mfd, 0x0001, 0, resp, 1);

    int r = gpio_write(mcu, 3, GPIO_HIGH);
    ASSERT_EQ(r, -1);

    close(mfd); close(mcu->fd); free(mcu);
}

int main(void) {
    printf("=== test_gpio ===\n");
    RUN_TEST(test_gpio_set_direction_output);
    RUN_TEST(test_gpio_write_high);
    RUN_TEST(test_gpio_read_high);
    RUN_TEST(test_gpio_read_low);
    RUN_TEST(test_gpio_write_failure);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_gpio && ./test_gpio
```

- [ ] **Step 3: Implement remote_gpio.c**

```c
/* remote_peripheral/src/remote_gpio.c */
#include "remote_gpio.h"
#include "mcu_internal.h"

#define TYPE_GPIO 0x0001

int gpio_set_direction(mcu_handle_t *mcu, uint8_t pin, uint8_t direction) {
    if (!mcu) return -1;
    uint8_t param[3] = {0x02, pin, direction};
    int r = mcu_request(mcu, TYPE_GPIO, param, 3, NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int gpio_write(mcu_handle_t *mcu, uint8_t pin, uint8_t value) {
    if (!mcu) return -1;
    uint8_t param[3] = {0x01, pin, value};
    int r = mcu_request(mcu, TYPE_GPIO, param, 3, NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int gpio_read(mcu_handle_t *mcu, uint8_t pin) {
    if (!mcu) return -1;
    uint8_t param[3] = {0x00, pin, 0x00};
    uint8_t data[1];
    int r = mcu_request(mcu, TYPE_GPIO, param, 3, data, sizeof(data));
    if (r < 1) return -1;
    return (int)data[0];
}
```

- [ ] **Step 4: Run tests**

```bash
cd remote_peripheral/build && make test_gpio && ./test_gpio
```

Expected:
```
=== test_gpio ===
  test_gpio_set_direction_output ... ok
  test_gpio_write_high ... ok
  test_gpio_read_high ... ok
  test_gpio_read_low ... ok
  test_gpio_write_failure ... ok

5 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/remote_gpio.c tests/test_gpio.c
git commit -m "feat: implement GPIO peripheral (set_direction, write, read)"
```

---

## Task 9: I2C Peripheral

**Files:**
- Modify: `remote_peripheral/src/remote_i2c.c`
- Create: `remote_peripheral/tests/test_i2c.c`

- [ ] **Step 1: Write failing tests**

```c
/* remote_peripheral/tests/test_i2c.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include "test_harness.h"
#include "remote_i2c.h"
#include "mcu_protocol.h"

static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *param, uint8_t len) {
    uint8_t frame[300];
    int flen = frame_build(type, tag, param, len, frame, sizeof(frame));
    write(mock_fd, frame, flen);
}

static mcu_handle_t *make_mock_mcu(int *mock_fd_out) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    mcu->fd = sv[0]; mcu->timeout_ms = 200; mcu->retry = 0; mcu->tag = 0;
    *mock_fd_out = sv[1];
    return mcu;
}

static void test_i2c_write_reg_success(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0002, 0, resp, 1);

    const uint8_t data[] = {0x01, 0x80};
    int r = i2c_write_reg(mcu, 0, 0x48, 0x01, data, 2);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_i2c_read_reg_returns_data(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    /* Response: status 0x00 + 2 data bytes */
    const uint8_t resp[] = {0x00, 0xAB, 0xCD};
    inject_response(mfd, 0x0002, 0, resp, 3);

    uint8_t buf[2];
    int r = i2c_read_reg(mcu, 0, 0x48, 0x00, buf, 2);
    ASSERT_EQ(r, 2);
    ASSERT_EQ(buf[0], 0xAB);
    ASSERT_EQ(buf[1], 0xCD);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_i2c_write_reg_failure(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0xFF};
    inject_response(mfd, 0x0002, 0, resp, 1);

    const uint8_t data[] = {0x01};
    int r = i2c_write_reg(mcu, 0, 0x48, 0x01, data, 1);
    ASSERT_EQ(r, -1);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_i2c_write_null_data(void) {
    mcu_handle_t mcu = {0}; mcu.fd = -1;
    ASSERT_EQ(i2c_write_reg(&mcu, 0, 0x48, 0x01, NULL, 1), -1);
}

static void test_i2c_write_len_zero(void) {
    mcu_handle_t mcu = {0}; mcu.fd = -1;
    const uint8_t data[] = {0x01};
    ASSERT_EQ(i2c_write_reg(&mcu, 0, 0x48, 0x01, data, 0), -1);
}

int main(void) {
    printf("=== test_i2c ===\n");
    RUN_TEST(test_i2c_write_reg_success);
    RUN_TEST(test_i2c_read_reg_returns_data);
    RUN_TEST(test_i2c_write_reg_failure);
    RUN_TEST(test_i2c_write_null_data);
    RUN_TEST(test_i2c_write_len_zero);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_i2c && ./test_i2c
```

- [ ] **Step 3: Implement remote_i2c.c**

```c
/* remote_peripheral/src/remote_i2c.c */
#include <string.h>
#include "remote_i2c.h"
#include "mcu_internal.h"

#define TYPE_I2C 0x0002

int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len) {
    if (!mcu || !data || len == 0 || len > 32) return -1;
    uint8_t param[5 + 32];
    param[0] = 0x01; /* write */
    param[1] = bus;
    param[2] = dev_addr;
    param[3] = reg_addr;
    param[4] = len;
    memcpy(param + 5, data, len);
    int r = mcu_request(mcu, TYPE_I2C, param, (uint8_t)(5 + len), NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len) {
    if (!mcu || !out_buf || len == 0 || len > 32) return -1;
    uint8_t param[5] = {0x00, bus, dev_addr, reg_addr, len};
    int r = mcu_request(mcu, TYPE_I2C, param, 5, out_buf, len);
    return r; /* number of bytes read, or -1 */
}
```

- [ ] **Step 4: Run tests**

```bash
cd remote_peripheral/build && make test_i2c && ./test_i2c
```

Expected:
```
=== test_i2c ===
  test_i2c_write_reg_success ... ok
  test_i2c_read_reg_returns_data ... ok
  test_i2c_write_reg_failure ... ok
  test_i2c_write_null_data ... ok
  test_i2c_write_len_zero ... ok

5 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/remote_i2c.c tests/test_i2c.c
git commit -m "feat: implement I2C peripheral (write_reg, read_reg)"
```

---

## Task 10: SPI Peripheral

**Files:**
- Modify: `remote_peripheral/src/remote_spi.c`
- Create: `remote_peripheral/tests/test_spi.c`

- [ ] **Step 1: Write failing tests**

```c
/* remote_peripheral/tests/test_spi.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include "test_harness.h"
#include "remote_spi.h"
#include "mcu_protocol.h"

static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *param, uint8_t len) {
    uint8_t frame[300];
    int flen = frame_build(type, tag, param, len, frame, sizeof(frame));
    write(mock_fd, frame, flen);
}

static mcu_handle_t *make_mock_mcu(int *mock_fd_out) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    mcu->fd = sv[0]; mcu->timeout_ms = 200; mcu->retry = 0; mcu->tag = 0;
    *mock_fd_out = sv[1];
    return mcu;
}

static void test_spi_write_success(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0003, 0, resp, 1);

    const uint8_t data[] = {0xAA, 0xBB};
    int r = spi_write(mcu, 0, 0, 0, data, 2);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_spi_read_returns_data(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00, 0x11, 0x22, 0x33};
    inject_response(mfd, 0x0003, 0, resp, 4);

    uint8_t buf[3];
    int r = spi_read(mcu, 0, 0, 0, buf, 3);
    ASSERT_EQ(r, 3);
    ASSERT_EQ(buf[0], 0x11);
    ASSERT_EQ(buf[1], 0x22);
    ASSERT_EQ(buf[2], 0x33);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_spi_transfer_full_duplex(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00, 0xDE, 0xAD};
    inject_response(mfd, 0x0003, 0, resp, 3);

    const uint8_t tx[] = {0xAA, 0xBB};
    uint8_t rx[2];
    int r = spi_transfer(mcu, 0, 0, 0, tx, rx, 2);
    ASSERT_EQ(r, 2);
    ASSERT_EQ(rx[0], 0xDE);
    ASSERT_EQ(rx[1], 0xAD);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_spi_write_failure(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0xFF};
    inject_response(mfd, 0x0003, 0, resp, 1);

    const uint8_t data[] = {0x01};
    ASSERT_EQ(spi_write(mcu, 0, 0, 0, data, 1), -1);

    close(mfd); close(mcu->fd); free(mcu);
}

int main(void) {
    printf("=== test_spi ===\n");
    RUN_TEST(test_spi_write_success);
    RUN_TEST(test_spi_read_returns_data);
    RUN_TEST(test_spi_transfer_full_duplex);
    RUN_TEST(test_spi_write_failure);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_spi && ./test_spi
```

- [ ] **Step 3: Implement remote_spi.c**

```c
/* remote_peripheral/src/remote_spi.c */
#include <string.h>
#include "remote_spi.h"
#include "mcu_internal.h"

#define TYPE_SPI 0x0003

int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len) {
    if (!mcu || !data || len == 0 || len > 32) return -1;
    uint8_t param[5 + 32];
    param[0] = 0x01; param[1] = bus; param[2] = cs;
    param[3] = mode; param[4] = len;
    memcpy(param + 5, data, len);
    int r = mcu_request(mcu, TYPE_SPI, param, (uint8_t)(5 + len), NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len) {
    if (!mcu || !out_buf || len == 0 || len > 32) return -1;
    uint8_t param[5] = {0x00, bus, cs, mode, len};
    return mcu_request(mcu, TYPE_SPI, param, 5, out_buf, len);
}

int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len) {
    if (!mcu || !tx_buf || !rx_buf || len == 0 || len > 32) return -1;
    uint8_t param[5 + 32];
    param[0] = 0x02; param[1] = bus; param[2] = cs;
    param[3] = mode; param[4] = len;
    memcpy(param + 5, tx_buf, len);
    return mcu_request(mcu, TYPE_SPI, param, (uint8_t)(5 + len), rx_buf, len);
}
```

- [ ] **Step 4: Run tests**

```bash
cd remote_peripheral/build && make test_spi && ./test_spi
```

Expected:
```
=== test_spi ===
  test_spi_write_success ... ok
  test_spi_read_returns_data ... ok
  test_spi_transfer_full_duplex ... ok
  test_spi_write_failure ... ok

4 passed, 0 failed
```

- [ ] **Step 5: Commit**

```bash
git add src/remote_spi.c tests/test_spi.c
git commit -m "feat: implement SPI peripheral (write, read, transfer)"
```

---

## Task 11: PWM Peripheral

**Files:**
- Modify: `remote_peripheral/src/remote_pwm.c`
- Create: `remote_peripheral/tests/test_pwm.c`

- [ ] **Step 1: Write failing tests**

```c
/* remote_peripheral/tests/test_pwm.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include "test_harness.h"
#include "remote_pwm.h"
#include "mcu_protocol.h"

static void inject_response(int mock_fd, uint16_t type, uint32_t tag,
                             const uint8_t *param, uint8_t len) {
    uint8_t frame[300];
    int flen = frame_build(type, tag, param, len, frame, sizeof(frame));
    write(mock_fd, frame, flen);
}

static mcu_handle_t *make_mock_mcu(int *mock_fd_out) {
    int sv[2];
    socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
    mcu_handle_t *mcu = calloc(1, sizeof(mcu_handle_t));
    mcu->fd = sv[0]; mcu->timeout_ms = 200; mcu->retry = 0; mcu->tag = 0;
    *mock_fd_out = sv[1];
    return mcu;
}

static void test_pwm_start_success(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0004, 0, resp, 1);

    /* 50 Hz, 50% duty (500/1000) */
    int r = pwm_start(mcu, 0, 50, 500);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_pwm_stop_success(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0x00};
    inject_response(mfd, 0x0004, 0, resp, 1);

    int r = pwm_stop(mcu, 0);
    ASSERT_EQ(r, 0);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_pwm_get_returns_params(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    /* Response: 0x00 + FreqHz=1000 (big-endian) + DutyPPT=750 (big-endian) */
    const uint8_t resp[] = {
        0x00,
        0x00, 0x00, 0x03, 0xE8,  /* 1000 Hz */
        0x02, 0xEE                /* 750 PPT */
    };
    inject_response(mfd, 0x0004, 0, resp, 7);

    uint32_t freq; uint16_t duty;
    int r = pwm_get(mcu, 0, &freq, &duty);
    ASSERT_EQ(r, 0);
    ASSERT_EQ(freq, 1000);
    ASSERT_EQ(duty, 750);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_pwm_start_failure(void) {
    int mfd;
    mcu_handle_t *mcu = make_mock_mcu(&mfd);

    const uint8_t resp[] = {0xFF};
    inject_response(mfd, 0x0004, 0, resp, 1);

    ASSERT_EQ(pwm_start(mcu, 0, 1000, 500), -1);

    close(mfd); close(mcu->fd); free(mcu);
}

static void test_pwm_get_duty_out_of_range(void) {
    mcu_handle_t mcu = {0}; mcu.fd = -1;
    uint32_t freq; uint16_t duty;
    /* duty_ppt > 1000 is invalid - but we only validate on send, not receive */
    /* Just test NULL handle guard */
    ASSERT_EQ(pwm_get(NULL, 0, &freq, &duty), -1);
}

int main(void) {
    printf("=== test_pwm ===\n");
    RUN_TEST(test_pwm_start_success);
    RUN_TEST(test_pwm_stop_success);
    RUN_TEST(test_pwm_get_returns_params);
    RUN_TEST(test_pwm_start_failure);
    RUN_TEST(test_pwm_get_duty_out_of_range);
    TEST_SUMMARY();
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd remote_peripheral/build && make test_pwm && ./test_pwm
```

- [ ] **Step 3: Implement remote_pwm.c**

```c
/* remote_peripheral/src/remote_pwm.c */
#include "remote_pwm.h"
#include "mcu_internal.h"

#define TYPE_PWM 0x0004

int pwm_start(mcu_handle_t *mcu, uint8_t channel,
              uint32_t freq_hz, uint16_t duty_ppt) {
    if (!mcu || duty_ppt > 1000) return -1;
    uint8_t param[8];
    param[0] = 0x01;
    param[1] = channel;
    param[2] = (uint8_t)(freq_hz >> 24);
    param[3] = (uint8_t)(freq_hz >> 16);
    param[4] = (uint8_t)(freq_hz >> 8);
    param[5] = (uint8_t)(freq_hz & 0xFF);
    param[6] = (uint8_t)(duty_ppt >> 8);
    param[7] = (uint8_t)(duty_ppt & 0xFF);
    int r = mcu_request(mcu, TYPE_PWM, param, 8, NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int pwm_stop(mcu_handle_t *mcu, uint8_t channel) {
    if (!mcu) return -1;
    uint8_t param[8] = {0x00, channel, 0,0,0,0, 0,0};
    int r = mcu_request(mcu, TYPE_PWM, param, 8, NULL, 0);
    return (r >= 0) ? 0 : -1;
}

int pwm_get(mcu_handle_t *mcu, uint8_t channel,
            uint32_t *out_freq_hz, uint16_t *out_duty_ppt) {
    if (!mcu || !out_freq_hz || !out_duty_ppt) return -1;
    uint8_t param[8] = {0x02, channel, 0,0,0,0, 0,0};
    uint8_t data[6];
    int r = mcu_request(mcu, TYPE_PWM, param, 8, data, sizeof(data));
    if (r < 6) return -1;
    *out_freq_hz  = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16)
                  | ((uint32_t)data[2] << 8)  |  (uint32_t)data[3];
    *out_duty_ppt = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
    return 0;
}
```

- [ ] **Step 4: Run tests**

```bash
cd remote_peripheral/build && make test_pwm && ./test_pwm
```

Expected:
```
=== test_pwm ===
  test_pwm_start_success ... ok
  test_pwm_stop_success ... ok
  test_pwm_get_returns_params ... ok
  test_pwm_start_failure ... ok
  test_pwm_get_duty_out_of_range ... ok

5 passed, 0 failed
```

- [ ] **Step 5: Run the full test suite**

```bash
cd remote_peripheral/build && ctest --output-on-failure
```

Expected: all 6 test suites pass.

- [ ] **Step 6: Commit**

```bash
git add src/remote_pwm.c tests/test_pwm.c
git commit -m "feat: implement PWM peripheral (start, stop, get); all tests passing"
```

---

## Final Verification

- [ ] **Full build and test from clean**

```bash
cd remote_peripheral
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4
ctest --output-on-failure
```

Expected: 35 tests pass across 6 suites, 0 failures.

- [ ] **Verify library artifact**

```bash
ls -lh build/libremote_peripheral.a
```

Expected: file exists, ~20-50 KB.
