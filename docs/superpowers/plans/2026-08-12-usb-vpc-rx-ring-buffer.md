# USB VPC RX Ring Buffer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `usb_fs_vpc` 的公开接收数组替换为可完整使用 512 字节容量的单生产者/单消费者环形缓冲区，并提供整包写入与按可用量读取接口。

**Architecture:** `User/usb_fs_vpc.c` 私有保存字节数组和单调递增的 32 位读写位置，使用 GCC 原子 acquire/release 操作发布位置。USB 接收回调作为唯一生产者调用写入函数，主循环作为唯一消费者调用读取函数；跨数组末尾的操作拆为最多两次 `memcpy`。

**Tech Stack:** C11、GCC `__atomic` 内建函数、STM32H7/USB CDC、主机 GCC `assert` 测试、CMake/Ninja。

---

## 文件映射

- Modify: `User/usb_fs_vpc.h`，保留容量常量，移除裸数组声明，公开读写接口并添加中文 Doxygen。
- Modify: `User/usb_fs_vpc.c`，私有化缓冲区，实现单生产者/单消费者读写和回绕复制。
- Create: `tests/stubs/usbd_cdc_if.h`，提供主机测试链接 `usb_fs_vpc.c` 所需的最小 USB/HAL 类型与声明。
- Create: `tests/usb_fs_vpc_test.c`，覆盖普通操作、回绕、满缓冲区、整包拒绝和无效参数。
- Preserve: `USB_DEVICE/App/usbd_cdc_if.c`，本次不修改 CubeMX 生成的接收回调。

### Task 1: 用失败测试定义公开接口和缓冲区行为

**Files:**
- Create: `tests/stubs/usbd_cdc_if.h`
- Create: `tests/usb_fs_vpc_test.c`
- Test: `tests/usb_fs_vpc_test.c`

- [ ] **Step 1: 创建 USB 主机测试桩**

创建 `tests/stubs/usbd_cdc_if.h`：

```c
#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

#define USBD_OK 0U
#define USBD_BUSY 1U

uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length);

#endif /* TEST_USBD_CDC_IF_H */
```

- [ ] **Step 2: 写入完整的失败测试**

创建 `tests/usb_fs_vpc_test.c`：

```c
#include "usb_fs_vpc.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
    return USBD_OK;
}

/**
 * @brief 清空环形缓冲区，隔离各测试用例。
 * @return 无。
 */
static void clear_rx_buffer(void)
{
    uint8_t scratch[USB_RX_BUFFER_MAX_SIZE];

    while (USB_VPC_RxRead(scratch, sizeof(scratch)) != 0U) {
    }
}

/**
 * @brief 验证普通写入以及请求长度超过可用量时的读取。
 * @return 无。
 */
static void test_write_and_read_available_data(void)
{
    const uint8_t input[] = {1U, 2U, 3U};
    uint8_t output[5] = {0U};

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(input));
    assert(memcmp(output, input, sizeof(input)) == 0);
    assert(USB_VPC_RxRead(output, sizeof(output)) == 0U);
}

/**
 * @brief 验证 512 字节容量全部可用且空间不足时整包拒绝。
 * @return 无。
 */
static void test_full_buffer_rejects_entire_write(void)
{
    uint8_t input[USB_RX_BUFFER_MAX_SIZE];
    uint8_t output[USB_RX_BUFFER_MAX_SIZE];
    const uint8_t extra[] = {0xA5U};
    size_t i;

    clear_rx_buffer();
    for (i = 0U; i < sizeof(input); i++) {
        input[i] = (uint8_t)i;
    }

    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxWrite(extra, sizeof(extra)) == 0U);
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 验证读写位置跨越数组末尾后仍保持字节顺序。
 * @return 无。
 */
static void test_wraparound_preserves_order(void)
{
    uint8_t prefix[400];
    uint8_t input[200];
    uint8_t output[200];
    size_t i;

    clear_rx_buffer();
    memset(prefix, 0xCC, sizeof(prefix));
    assert(USB_VPC_RxWrite(prefix, sizeof(prefix)) == sizeof(prefix));
    assert(USB_VPC_RxRead(prefix, sizeof(prefix)) == sizeof(prefix));

    for (i = 0U; i < sizeof(input); i++) {
        input[i] = (uint8_t)(i + 17U);
    }
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 验证部分读取释放空间后可继续写入并保持 FIFO 顺序。
 * @return 无。
 */
static void test_interleaved_read_and_write(void)
{
    const uint8_t first[] = {10U, 11U, 12U, 13U};
    const uint8_t second[] = {20U, 21U, 22U};
    const uint8_t expected[] = {12U, 13U, 20U, 21U, 22U};
    uint8_t output[sizeof(expected)];

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(first, sizeof(first)) == sizeof(first));
    assert(USB_VPC_RxRead(output, 2U) == 2U);
    assert(USB_VPC_RxWrite(second, sizeof(second)) == sizeof(second));
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, expected, sizeof(expected)) == 0);
}

/**
 * @brief 验证空指针与零长度不会改变缓冲区状态。
 * @return 无。
 */
static void test_invalid_arguments_do_not_change_state(void)
{
    const uint8_t input[] = {7U, 8U};
    uint8_t output[sizeof(input)] = {0U};

    clear_rx_buffer();
    assert(USB_VPC_RxWrite(NULL, sizeof(input)) == 0U);
    assert(USB_VPC_RxWrite(input, 0U) == 0U);
    assert(USB_VPC_RxWrite(input, sizeof(input)) == sizeof(input));
    assert(USB_VPC_RxRead(NULL, sizeof(output)) == 0U);
    assert(USB_VPC_RxRead(output, 0U) == 0U);
    assert(USB_VPC_RxRead(output, sizeof(output)) == sizeof(output));
    assert(memcmp(output, input, sizeof(input)) == 0);
}

/**
 * @brief 运行 USB VPC 接收环形缓冲区主机测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    test_write_and_read_available_data();
    test_full_buffer_rejects_entire_write();
    test_wraparound_preserves_order();
    test_interleaved_read_and_write();
    test_invalid_arguments_do_not_change_state();
    return 0;
}
```

- [ ] **Step 3: 编译测试并确认 RED**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/usb_fs_vpc_test.c User/usb_fs_vpc.c -o build/usb_fs_vpc_test.exe
```

预期：编译或链接失败，报告 `USB_VPC_RxWrite`、`USB_VPC_RxRead` 尚未声明或定义。失败应来自待实现接口，而不是测试语法或测试桩错误。

- [ ] **Step 4: 提交失败测试**

```powershell
git add tests/stubs/usbd_cdc_if.h tests/usb_fs_vpc_test.c
git commit -m "test: 定义 USB 接收环形缓冲区行为"
```

### Task 2: 实现环形缓冲区读写接口

**Files:**
- Modify: `User/usb_fs_vpc.h`
- Modify: `User/usb_fs_vpc.c`
- Test: `tests/usb_fs_vpc_test.c`

- [ ] **Step 1: 修改公开头文件**

在 `User/usb_fs_vpc.h` 中删除：

```c
extern uint8_t usb_rx_buffer[USB_RX_BUFFER_MAX_SIZE];
HAL_StatusTypeDef USB_VPC_Receive(uint8_t *data, uint32_t len);
```

加入以下中文 Doxygen 接口声明：

```c
/**
 * @brief 将一包数据写入 USB 接收环形缓冲区。
 * @param[in] data 待写入数据。
 * @param[in] len 待写入字节数。
 * @return 写入成功时返回 len；空间不足或参数无效时返回 0。
 */
uint32_t USB_VPC_RxWrite(const uint8_t *data, uint32_t len);

/**
 * @brief 从 USB 接收环形缓冲区读取数据。
 * @param[out] data 接收数据的目标缓冲区。
 * @param[in] len 期望读取的最大字节数。
 * @return 实际读取字节数；无数据或参数无效时返回 0。
 */
uint32_t USB_VPC_RxRead(uint8_t *data, uint32_t len);
```

- [ ] **Step 2: 私有化状态并实现整包写入**

在 `User/usb_fs_vpc.c` 引入 `<string.h>`，将公开数组替换为：

```c
static uint8_t usb_rx_buffer[USB_RX_BUFFER_MAX_SIZE];
static uint32_t usb_rx_read_pos;
static uint32_t usb_rx_write_pos;
```

加入写入实现：

```c
uint32_t USB_VPC_RxWrite(const uint8_t *data, uint32_t len)
{
    uint32_t read_pos;
    uint32_t write_pos;
    uint32_t used;
    uint32_t offset;
    uint32_t first_len;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    write_pos = __atomic_load_n(&usb_rx_write_pos, __ATOMIC_RELAXED);
    read_pos = __atomic_load_n(&usb_rx_read_pos, __ATOMIC_ACQUIRE);
    used = write_pos - read_pos;
    if (len > (USB_RX_BUFFER_MAX_SIZE - used)) {
        return 0U;
    }

    offset = write_pos % USB_RX_BUFFER_MAX_SIZE;
    first_len = USB_RX_BUFFER_MAX_SIZE - offset;
    if (first_len > len) {
        first_len = len;
    }
    memcpy(&usb_rx_buffer[offset], data, first_len);
    if (len > first_len) {
        memcpy(usb_rx_buffer, &data[first_len], len - first_len);
    }
    __atomic_store_n(&usb_rx_write_pos, write_pos + len, __ATOMIC_RELEASE);
    return len;
}
```

- [ ] **Step 3: 实现按可用量读取**

加入读取实现，并删除未完成的 `USB_VPC_Receive()`：

```c
uint32_t USB_VPC_RxRead(uint8_t *data, uint32_t len)
{
    uint32_t read_pos;
    uint32_t write_pos;
    uint32_t available;
    uint32_t read_len;
    uint32_t offset;
    uint32_t first_len;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    read_pos = __atomic_load_n(&usb_rx_read_pos, __ATOMIC_RELAXED);
    write_pos = __atomic_load_n(&usb_rx_write_pos, __ATOMIC_ACQUIRE);
    available = write_pos - read_pos;
    read_len = len < available ? len : available;
    if (read_len == 0U) {
        return 0U;
    }

    offset = read_pos % USB_RX_BUFFER_MAX_SIZE;
    first_len = USB_RX_BUFFER_MAX_SIZE - offset;
    if (first_len > read_len) {
        first_len = read_len;
    }
    memcpy(data, &usb_rx_buffer[offset], first_len);
    if (read_len > first_len) {
        memcpy(&data[first_len], usb_rx_buffer, read_len - first_len);
    }
    __atomic_store_n(&usb_rx_read_pos, read_pos + read_len, __ATOMIC_RELEASE);
    return read_len;
}
```

- [ ] **Step 4: 编译并运行测试确认 GREEN**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/usb_fs_vpc_test.c User/usb_fs_vpc.c -o build/usb_fs_vpc_test.exe
build\usb_fs_vpc_test.exe
```

预期：编译无警告，测试进程退出码为 0，标准输出为空。

- [ ] **Step 5: 提交实现**

```powershell
git add User/usb_fs_vpc.h User/usb_fs_vpc.c
git commit -m "feat: 添加 USB 接收环形缓冲区"
```

### Task 3: 回归、固件构建和范围验证

**Files:**
- Verify: `User/usb_fs_vpc.h`
- Verify: `User/usb_fs_vpc.c`
- Verify: `tests/stubs/usbd_cdc_if.h`
- Verify: `tests/usb_fs_vpc_test.c`

- [ ] **Step 1: 重新运行 USB 环形缓冲区测试**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/usb_fs_vpc_test.c User/usb_fs_vpc.c -o build/usb_fs_vpc_test.exe
build\usb_fs_vpc_test.exe
```

预期：编译与执行退出码均为 0。

- [ ] **Step 2: 运行现有主机回归测试**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/servo_position_test.c User/servo_position.c -o build/servo_position_test.exe
build\servo_position_test.exe
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
build\state_machine_test.exe
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser tests/time_us_test.c User/time_us.c -o build/time_us_test.exe
build\time_us_test.exe
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build\wheel_pid_test.exe
```

预期：五个现有测试程序都以退出码 0 结束。

- [ ] **Step 3: 构建 STM32 Debug 固件**

```powershell
cmake --build --preset Debug
```

预期：Ninja 返回退出码 0，并生成 `build/Debug/26rcf.elf`。`usb_fs_vpc.c` 中 32 位 `__atomic` 操作应由 Cortex-M7 指令内联完成，不产生缺失 `libatomic` 符号。

- [ ] **Step 4: 检查编码和中文 Doxygen**

```powershell
@('User/usb_fs_vpc.h','User/usb_fs_vpc.c','tests/stubs/usbd_cdc_if.h','tests/usb_fs_vpc_test.c') |
  ForEach-Object {
    $bytes = [IO.File]::ReadAllBytes($_)
    $utf8 = New-Object Text.UTF8Encoding($false,$true)
    [void]$utf8.GetString($bytes)
  }
rg -n "^/\*\*|@brief|@param|@return" User/usb_fs_vpc.h User/usb_fs_vpc.c tests/usb_fs_vpc_test.c
```

预期：严格 UTF-8 解码无异常；新增公开接口和测试辅助函数均包含中文 Doxygen 字段。

- [ ] **Step 5: 检查改动范围与空白错误**

```powershell
git diff --check
git status --short
git diff -- User/usb_fs_vpc.h User/usb_fs_vpc.c tests/stubs/usbd_cdc_if.h tests/usb_fs_vpc_test.c
```

预期：没有空白错误；功能改动只涉及计划列出的四个文件，不还原或暂存用户现有的 CubeMX、Core、Middleware 和 USB_DEVICE 改动。

- [ ] **Step 6: 记录实际验证结果**

最终交付中报告 USB 新测试、五个现有主机测试和 STM32 固件构建的真实结果。任何未运行或失败的命令都必须明确说明，不得描述为已通过。
