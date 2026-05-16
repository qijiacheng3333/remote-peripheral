# 远程外设扩展系统设计文档

**日期：** 2026-05-01  
**版本：** 1.0  
**范围：** CPU 侧 C 函数库 + 完整通信协议规范

---

## 1. 背景与目标

CPU（运行 Linux）通过多路独立 UART（RS232，8N1，无流控）与多个 MCU 通信，每个 MCU 独占一个串口。CPU 侧提供 C 函数库，使调用方能像操作本地外设一样操作 MCU 扩展的 GPIO / I2C / SPI / PWM 外设。MCU 侧按本文协议规范独立开发，不在本文档实现范围内。

---

## 2. 物理层

| 参数 | 值 |
|------|----|
| 接口标准 | RS232 |
| 帧格式 | 8N1（8数据位，无校验，1停止位） |
| 流控 | 无 |
| 波特率 | 可配置，建议 115200 |
| 拓扑 | CPU 每路 UART 独占连接一个 MCU |

---

## 3. 通信协议

### 3.1 发送帧封装（四步）

#### 第一步：构造 A 帧（应用数据）

```
| Type (2B, 大端) | Tag (4B, 大端) | Param (1~128B) |
```

- **Type**：外设操作类型编号
- **Tag**：帧序号，从 0 起每帧 +1
- **Param**：操作参数，格式按外设类型定义

#### 第二步：构造 B 帧（加 CRC）

```
B = A + CRC8(A)
```

CRC-8 参数：poly=0x07，init=0x00，不反转，不异或。  
校验向量：`CRC8("123456789") = 0xF4`

#### 第三步：构造 C 帧（字节转义）

对 B 帧逐字节扫描：

| 原字节 | 替换为 |
|--------|--------|
| `0xA5` | `0xD0, 0x45` |
| `0x5A` | `0xD0, 0xBA` |
| `0xD0` | `0xD0, 0x30` |
| 其他   | 不变 |

转义规则：遇到 `0xA5`、`0x5A`、`0xD0` 任意一个，替换为 `0xD0, (原字节 XOR 0xE0)`。

#### 第四步：构造 D 帧（加定界符）

```
D = 0xA5 + C + 0x5A
```

D 帧为最终串口发送内容。

---

### 3.2 响应帧格式

MCU 响应帧同样按 A→B→C→D 四步封装：

- **Type**：与请求帧相同
- **Tag**：与请求帧相同
- **Param**：
  - 执行失败：`[0xFF]`
  - 写操作成功：`[0x00]`
  - 读操作成功：`[0x00, data0, data1, ...]`

---

### 3.3 Type 编号表

| Type | 外设 |
|------|------|
| `0x0001` | GPIO |
| `0x0002` | I2C |
| `0x0003` | SPI |
| `0x0004` | PWM |

---

### 3.4 Param 格式定义

#### GPIO（Type=0x0001）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | Op | `0x00`=读，`0x01`=写，`0x02`=配置方向 |
| 1 | 1 | Pin | 引脚编号（0~127） |
| 2 | 1 | Value | 写时：`0x00`=LOW，`0x01`=HIGH；配置时：`0x00`=输入，`0x01`=输出；读时忽略 |

响应 Param：
- 成功（写/配置）：`[0x00]`
- 成功（读）：`[0x00, 0x00/0x01]`（第2字节为引脚电平）
- 失败：`[0xFF]`

#### I2C（Type=0x0002）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | Op | `0x00`=读，`0x01`=写 |
| 1 | 1 | Bus | I2C 总线编号 |
| 2 | 1 | DevAddr | 7位设备地址 |
| 3 | 1 | RegAddr | 寄存器地址 |
| 4 | 1 | Len | 读/写字节数（1~32） |
| 5 | Len | Data | 写时为数据；读时此字段不存在 |

响应 Param：
- 成功（写）：`[0x00]`
- 成功（读）：`[0x00, data0, ..., dataN]`
- 失败：`[0xFF]`

#### SPI（Type=0x0003）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | Op | `0x00`=读，`0x01`=写，`0x02`=全双工收发 |
| 1 | 1 | Bus | SPI 总线编号 |
| 2 | 1 | CS | 片选编号 |
| 3 | 1 | Mode | SPI 模式（0~3） |
| 4 | 1 | Len | 数据字节数（1~32） |
| 5 | Len | Data | 写/收发时为发送数据；读时此字段不存在 |

响应 Param：
- 成功（写）：`[0x00]`
- 成功（读/收发）：`[0x00, data0, ..., dataN]`
- 失败：`[0xFF]`

#### PWM（Type=0x0004）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | Op | `0x00`=停止，`0x01`=启动/更新，`0x02`=读取当前参数 |
| 1 | 1 | Channel | PWM 通道编号 |
| 2 | 4 | FreqHz | 频率（Hz），大端 uint32；Op=0x00/0x02 时填 0 忽略 |
| 6 | 2 | DutyPPT | 占空比千分比（0~1000），大端 uint16；Op=0x00/0x02 时填 0 忽略 |

响应 Param：
- 成功（停止/启动）：`[0x00]`
- 成功（读取）：`[0x00, FreqHz(4B大端), DutyPPT(2B大端)]`
- 失败：`[0xFF]`

---

## 4. CPU 侧软件架构

### 4.1 分层结构

```
┌─────────────────────────────────────────────────┐
│              应用层（用户代码）                    │
│  gpio_write()  i2c_read()  spi_transfer()  ...   │
├─────────────────────────────────────────────────┤
│              外设抽象层（Peripheral HAL）          │
│  remote_gpio.c  remote_i2c.c  remote_spi.c      │
│  remote_pwm.c                                    │
├─────────────────────────────────────────────────┤
│              协议层（Protocol Layer）             │
│  frame_build()  frame_parse()  crc8()  escape()  │
├─────────────────────────────────────────────────┤
│              传输层（Transport Layer）            │
│  mcu_open()  mcu_send()  mcu_recv()  mcu_close() │
├─────────────────────────────────────────────────┤
│              UART 驱动（Linux termios）           │
│              /dev/ttyS0  /dev/ttyUSB0  ...       │
└─────────────────────────────────────────────────┘
```

### 4.2 核心数据结构

```c
typedef struct {
    int      fd;            /* 串口文件描述符 */
    uint32_t tag;           /* 帧序号，每发一帧自增 */
    int      timeout_ms;    /* 等待响应超时，默认 500ms */
    int      retry;         /* 超时重试次数，默认 2 */
    char     dev[64];       /* 串口路径，如 /dev/ttyS1 */
} mcu_handle_t;
```

### 4.3 文件组织

```
remote_peripheral/
├── include/
│   ├── mcu_transport.h
│   ├── mcu_protocol.h
│   ├── remote_gpio.h
│   ├── remote_i2c.h
│   ├── remote_spi.h
│   └── remote_pwm.h
├── src/
│   ├── mcu_transport.c
│   ├── mcu_protocol.c
│   ├── remote_gpio.c
│   ├── remote_i2c.c
│   ├── remote_spi.c
│   └── remote_pwm.c
└── CMakeLists.txt
```

### 4.4 核心调用流程

```
gpio_write(mcu, pin, HIGH)
    │
    ├─ 1. 构造 Param: [0x01, pin, 0x01]
    ├─ 2. 协议层：A帧 → B帧（加CRC）→ C帧（转义）→ D帧（加定界符）
    ├─ 3. 传输层：write(fd, D帧)
    ├─ 4. 传输层：阻塞读，等待 0xA5...0x5A 响应帧，超时则重试
    ├─ 5. 协议层：解析响应，验证 Tag 匹配 + CRC 正确
    └─ 6. 返回 0（成功）或 -1（失败/超时）
```

---

## 5. API 接口定义

### 5.1 传输层 `mcu_transport.h`

```c
mcu_handle_t *mcu_open(const char *dev, int baudrate);
void          mcu_close(mcu_handle_t *mcu);
void          mcu_set_timeout(mcu_handle_t *mcu, int timeout_ms, int retry);
```

### 5.2 协议层 `mcu_protocol.h`（内部使用）

```c
int     frame_build(uint16_t type, uint32_t tag,
                    const uint8_t *param, uint8_t param_len,
                    uint8_t *out_buf, int buf_size);

int     frame_parse(const uint8_t *in_buf, int in_len,
                    uint16_t *out_type, uint32_t *out_tag,
                    uint8_t *out_param, int param_buf_size);

uint8_t crc8(const uint8_t *data, int len);
```

### 5.3 GPIO `remote_gpio.h`

```c
#define GPIO_LOW    0
#define GPIO_HIGH   1
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

int gpio_set_direction(mcu_handle_t *mcu, uint8_t pin, uint8_t direction);
int gpio_write(mcu_handle_t *mcu, uint8_t pin, uint8_t value);
int gpio_read(mcu_handle_t *mcu, uint8_t pin);
```

### 5.4 I2C `remote_i2c.h`

```c
int i2c_write_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                  uint8_t reg_addr, const uint8_t *data, uint8_t len);

int i2c_read_reg(mcu_handle_t *mcu, uint8_t bus, uint8_t dev_addr,
                 uint8_t reg_addr, uint8_t *out_buf, uint8_t len);
```

### 5.5 SPI `remote_spi.h`

```c
int spi_write(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
              const uint8_t *data, uint8_t len);

int spi_read(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
             uint8_t *out_buf, uint8_t len);

int spi_transfer(mcu_handle_t *mcu, uint8_t bus, uint8_t cs, uint8_t mode,
                 const uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len);
```

### 5.6 PWM `remote_pwm.h`

```c
int pwm_start(mcu_handle_t *mcu, uint8_t channel,
              uint32_t freq_hz, uint16_t duty_ppt);

int pwm_stop(mcu_handle_t *mcu, uint8_t channel);

int pwm_get(mcu_handle_t *mcu, uint8_t channel,
            uint32_t *out_freq_hz, uint16_t *out_duty_ppt);
```

---

## 6. 错误处理

### 6.1 CPU 侧策略

| 场景 | 处理 | 返回值 |
|------|------|--------|
| 超时未收到响应 | 重发原帧（最多 retry 次），仍超时放弃 | `-1` |
| 响应 Tag 不匹配 | 丢弃，继续等待直到超时 | `-1` |
| CRC 校验失败 | 丢弃，由超时机制兜底 | `-1` |
| MCU 回复 `0xFF` | 直接返回失败，不重试 | `-1` |
| Param 超出范围 | 构建阶段拦截，不发送 | `-1` |
| 串口 fd 无效 | 所有 API 首行检查，立即返回 | `-1` |

### 6.2 接收状态机

```
IDLE
  └─ 收到 0xA5 ──→ IN_FRAME
                      ├─ 收到 0xD0 ──→ 读下一字节 XOR 0xE0，追加到缓冲区
                      ├─ 收到 0x5A ──→ 帧完整，交给 frame_parse
                      └─ 超时       ──→ 重置到 IDLE，返回错误
```

---

## 7. MCU 侧协议规范（供对方开发参考）

### 7.1 帧接收与解析

1. 以 `0xA5` 为帧起始，`0x5A` 为帧结束
2. 收到 `0xD0` 时，取下一字节 XOR `0xE0` 还原原始字节
3. 解析 A 帧：前2字节 Type（大端）、后4字节 Tag（大端）、剩余为 Param
4. 对 A 帧计算 CRC-8，与 B 帧最后1字节比对，不符则丢弃，不回复

### 7.2 响应帧封装

同发送方，按 A→B→C→D 四步封装后发送：
- Type 和 Tag 与请求帧相同
- Param 格式见第 3.2 节

### 7.3 时序要求

- 收到完整帧后 **50ms 内** 必须回复响应帧
- 不支持主动上报，MCU 只响应 CPU 请求
- CPU 同一时刻只发一帧，等响应后再发下一帧

### 7.4 外设分发

```
Type=0x0001 → GPIO handler
Type=0x0002 → I2C handler
Type=0x0003 → SPI handler
Type=0x0004 → PWM handler
未知 Type   → 回复 [0xFF]
```

---

## 8. 典型使用示例

```c
#include "remote_gpio.h"
#include "remote_i2c.h"
#include "remote_spi.h"
#include "remote_pwm.h"

int main(void) {
    mcu_handle_t *mcu1 = mcu_open("/dev/ttyS1", 115200);
    mcu_handle_t *mcu2 = mcu_open("/dev/ttyS2", 115200);

    /* GPIO：配置方向并读写 */
    gpio_set_direction(mcu1, 3, GPIO_OUTPUT);
    gpio_write(mcu1, 3, GPIO_HIGH);
    int val = gpio_read(mcu1, 5);

    /* I2C：读写寄存器 */
    uint8_t wbuf[2] = {0x01, 0x80};
    i2c_write_reg(mcu2, 0, 0x48, 0x01, wbuf, 2);
    uint8_t rbuf[2];
    i2c_read_reg(mcu2, 0, 0x48, 0x00, rbuf, 2);

    /* SPI：全双工收发 */
    uint8_t tx[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t rx[4];
    spi_transfer(mcu1, 0, 0, 0, tx, rx, 4);

    /* PWM：50Hz，50% 占空比 */
    pwm_start(mcu1, 0, 50, 500);

    mcu_close(mcu1);
    mcu_close(mcu2);
    return 0;
}
```
