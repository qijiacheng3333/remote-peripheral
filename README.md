# remote_peripheral

CPU 侧 C 静态库，通过 UART 串口远程控制 MCU 扩展的 GPIO / I2C / SPI / PWM 外设。

## 概述

CPU（Linux）通过独立 UART 与多个 MCU 通信，每个 MCU 独占一路串口。本库封装了完整的串口帧协议，使上层代码能像操作本地外设一样调用远端 MCU 的外设资源。

```
App code
   │
   ▼
remote_gpio / remote_i2c / remote_spi / remote_pwm
   │
   ▼
mcu_request  (transport layer)
   │
   ▼
frame_build / frame_parse / crc8  (protocol layer)
   │
   ▼
UART (RS232, 8N1, 115200)
   │
   ▼
MCU
```

## 通信协议

采用自定义四步串行帧封装：

| 步骤 | 描述 |
|------|------|
| A 帧 | Type(2B) + Tag(4B) + Param(1~128B) |
| B 帧 | A + CRC8(A)，poly=0x07，init=0x00 |
| C 帧 | 对 B 帧进行字节转义（0xA5/0x5A/0xD0） |
| D 帧 | 0xA5 + C + 0x5A，最终串口发送内容 |

完整协议规范见 [`docs/superpowers/specs/2026-05-01-remote-peripheral-design.md`](docs/superpowers/specs/2026-05-01-remote-peripheral-design.md)。

## API

### 连接管理

```c
#include "mcu_transport.h"

mcu_handle_t *mcu_open(const char *dev, int baudrate);
void          mcu_close(mcu_handle_t *mcu);
void          mcu_set_timeout(mcu_handle_t *mcu, int timeout_ms, int retry);
```

### GPIO

```c
#include "remote_gpio.h"

int gpio_set_direction(mcu_handle_t *mcu, uint8_t pin, uint8_t direction);
int gpio_write(mcu_handle_t *mcu, uint8_t pin, uint8_t value);
int gpio_read(mcu_handle_t *mcu, uint8_t pin);
```

### I2C

```c
#include "remote_i2c.h"

int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len);
int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len);
```

### SPI

```c
#include "remote_spi.h"

int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len);
int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len);
int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len);
```

### PWM

```c
#include "remote_pwm.h"

int pwm_start(mcu_handle_t *mcu, uint8_t channel,
              uint32_t freq_hz, uint16_t duty_ppt);
int pwm_stop(mcu_handle_t *mcu, uint8_t channel);
int pwm_get(mcu_handle_t *mcu, uint8_t channel,
            uint32_t *out_freq_hz, uint16_t *out_duty_ppt);
```

返回值：成功返回 0（或读取的字节数），失败返回 -1。

## 使用示例

```c
#include "mcu_transport.h"
#include "remote_gpio.h"
#include "remote_i2c.h"

int main(void) {
    mcu_handle_t *mcu = mcu_open("/dev/ttyS0", 115200);
    if (!mcu) return -1;

    /* 设置 pin3 为输出，拉高 */
    gpio_set_direction(mcu, 3, GPIO_OUTPUT);
    gpio_write(mcu, 3, GPIO_HIGH);

    /* 读取 I2C 传感器 2 字节 */
    uint8_t buf[2];
    i2c_read_reg(mcu, 0, 0x48, 0x00, buf, 2);

    mcu_close(mcu);
    return 0;
}
```

## 构建

```bash
cd remote_peripheral
mkdir build && cd build
cmake ..
make
ctest --output-on-failure
```

要求：CMake >= 3.10，GCC（C99）。

## 测试

| 测试套件 | 用例数 | 覆盖内容 |
|----------|--------|----------|
| test_protocol | 12 | crc8、escape/unescape、frame_build、frame_parse |
| test_transport | 9 | mcu_open/close、mcu_request、timeout/retry |
| test_gpio | 5 | set_direction、write、read、failure |
| test_i2c | 5 | write_reg、read_reg、failure、参数校验 |
| test_spi | 4 | write、read、transfer、failure |
| test_pwm | 5 | start、stop、get、failure、null handle |

测试使用 `socketpair()` 模拟 MCU，无需真实硬件即可运行。

## 文档

- [设计规范](docs/superpowers/specs/2026-05-01-remote-peripheral-design.md) — 完整协议定义，MCU 开发者参考此文档实现 MCU 侧
- [实施计划](docs/superpowers/plans/2026-05-01-remote-peripheral.md) — 开发计划与任务分解

## MCU 侧开发

MCU 开发者只需参考设计规范文档实现：
1. 相同的帧接收/解析逻辑（D→C→B→A）
2. 相同的帧构建/发送逻辑（A→B→C→D）
3. 按 Type 字段分发到对应外设驱动

## 许可

MIT
