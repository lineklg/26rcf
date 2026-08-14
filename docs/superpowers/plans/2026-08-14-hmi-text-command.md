# HMI 文本命令补全 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 补全 HMI 阻塞式串口发送与文本控件命令生成，使示例输入准确发送 `main.t0.txt="A: aigavh123"` 和三个 `0xFF` 帧尾。

**Architecture:** `HMI_UART_Send` 负责发送任意长度明确的命令主体并在成功后追加帧尾；`HMI_UART_Send_ModifyTxt` 使用 128 字节静态缓冲区和 `snprintf` 生成文本控件命令。主机测试通过专用 HAL 头文件和 UART 发送桩捕获每次传输，不修改共享测试桩。

**Tech Stack:** C11、STM32 HAL UART、GCC 主机测试、CMake/Ninja ARM 固件构建

---

## 文件结构

- Modify: `User/hmi.c`，实现参数校验、阻塞式 UART 发送、帧尾追加和文本命令格式化。
- Modify: `User/hmi.h`，为两个公开接口补充中文 Doxygen 契约。
- Create: `tests/stubs/hmi_test_hal.h`，为 HMI 主机测试声明最小 HAL UART 接口。
- Create: `tests/hmi_test.c`，捕获 UART 调用并验证命令格式、边界和错误路径。

### Task 1: 用主机测试定义 HMI 发送行为

**Files:**
- Create: `tests/stubs/hmi_test_hal.h`
- Create: `tests/hmi_test.c`
- Test: `tests/hmi_test.c`

- [ ] **Step 1: 创建 HMI 专用 HAL 测试声明**

创建 `tests/stubs/hmi_test_hal.h`：

```c
#ifndef HMI_TEST_HAL_H
#define HMI_TEST_HAL_H

#include "main.h"

#define HAL_MAX_DELAY 0xFFFFFFFFU

/**
 * @brief 主机测试使用的阻塞式 UART 发送桩。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送数据。
 * @param[in] size 数据长度。
 * @param[in] timeout 超时时间。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout
);

#endif
```

- [ ] **Step 2: 创建失败的行为测试**

创建 `tests/hmi_test.c`：

```c
#include "hmi.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#define UART_CALL_CAPACITY 4U
#define UART_DATA_CAPACITY 128U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t data[UART_DATA_CAPACITY];
    uint16_t size;
    uint32_t timeout;
} UartCall;

static UART_HandleTypeDef test_uart;
static UartCall uart_calls[UART_CALL_CAPACITY];
static uint32_t uart_call_count;
static HAL_StatusTypeDef next_uart_status;

/**
 * @brief 捕获阻塞式 UART 发送调用。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送数据。
 * @param[in] size 数据长度。
 * @param[in] timeout 超时时间。
 * @return 当前测试设置的 HAL 状态。
 */
HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout
)
{
    UartCall *call;

    assert(uart_call_count < UART_CALL_CAPACITY);
    assert(size <= UART_DATA_CAPACITY);
    call = &uart_calls[uart_call_count++];
    call->huart = huart;
    call->size = size;
    call->timeout = timeout;
    memcpy(call->data, data, size);
    return next_uart_status;
}

/**
 * @brief 重置 UART 发送测试夹具。
 * @return 无。
 */
static void Reset_Fixture(void)
{
    memset(uart_calls, 0, sizeof(uart_calls));
    uart_call_count = 0U;
    next_uart_status = HAL_OK;
}

/**
 * @brief 验证文本控件命令及帧尾格式。
 * @return 无。
 */
static void Test_ModifyTxt_Sends_Formatted_Command_And_Tail(void)
{
    static const char expected[] = "main.t0.txt=\"A: aigavh123\"";
    static const uint8_t expected_tail[] = {0xFFU, 0xFFU, 0xFFU};

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "main.t0", "A: aigavh123");

    assert(uart_call_count == 2U);
    assert(uart_calls[0].huart == &test_uart);
    assert(uart_calls[0].size == sizeof(expected) - 1U);
    assert(memcmp(uart_calls[0].data, expected, sizeof(expected) - 1U) == 0);
    assert(uart_calls[0].timeout == HAL_MAX_DELAY);
    assert(uart_calls[1].size == sizeof(expected_tail));
    assert(memcmp(uart_calls[1].data, expected_tail, sizeof(expected_tail)) == 0);
}

/**
 * @brief 验证基础接口按明确长度发送二进制数据。
 * @return 无。
 */
static void Test_Send_Uses_Explicit_Binary_Size(void)
{
    static const uint8_t data[] = {0x41U, 0x00U, 0x42U};

    Reset_Fixture();
    HMI_UART_Send(&test_uart, data, sizeof(data));

    assert(uart_call_count == 2U);
    assert(uart_calls[0].size == sizeof(data));
    assert(memcmp(uart_calls[0].data, data, sizeof(data)) == 0);
}

/**
 * @brief 验证空指针和零长度输入不会发送数据。
 * @return 无。
 */
static void Test_Invalid_Input_Does_Not_Send(void)
{
    static const uint8_t data[] = {0x41U};

    Reset_Fixture();
    HMI_UART_Send(NULL, data, sizeof(data));
    HMI_UART_Send(&test_uart, NULL, sizeof(data));
    HMI_UART_Send(&test_uart, data, 0U);
    HMI_UART_Send_ModifyTxt(NULL, "main.t0", "text");
    HMI_UART_Send_ModifyTxt(&test_uart, NULL, "text");
    HMI_UART_Send_ModifyTxt(&test_uart, "main.t0", NULL);
    assert(uart_call_count == 0U);
}

/**
 * @brief 验证恰好 127 字节的命令可发送，128 字节命令被拒绝。
 * @return 无。
 */
static void Test_ModifyTxt_Rejects_Overflow(void)
{
    char fitting_text[120];
    char overflow_text[121];

    memset(fitting_text, 'x', sizeof(fitting_text) - 1U);
    fitting_text[sizeof(fitting_text) - 1U] = '\0';
    memset(overflow_text, 'x', sizeof(overflow_text) - 1U);
    overflow_text[sizeof(overflow_text) - 1U] = '\0';

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "w", fitting_text);
    assert(uart_call_count == 2U);
    assert(uart_calls[0].size == 127U);

    Reset_Fixture();
    HMI_UART_Send_ModifyTxt(&test_uart, "w", overflow_text);
    assert(uart_call_count == 0U);
}

/**
 * @brief 验证命令主体发送失败时不追加帧尾。
 * @return 无。
 */
static void Test_Send_Failure_Does_Not_Send_Tail(void)
{
    static const uint8_t data[] = {0x41U};

    Reset_Fixture();
    next_uart_status = HAL_ERROR;
    HMI_UART_Send(&test_uart, data, sizeof(data));
    assert(uart_call_count == 1U);
}

/**
 * @brief 运行 HMI 串口发送测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    Test_ModifyTxt_Sends_Formatted_Command_And_Tail();
    Test_Send_Uses_Explicit_Binary_Size();
    Test_Invalid_Input_Does_Not_Send();
    Test_ModifyTxt_Rejects_Overflow();
    Test_Send_Failure_Does_Not_Send_Tail();
    return 0;
}
```

- [ ] **Step 3: 编译并运行测试，确认 RED**

```powershell
gcc -std=c11 -Wall -Wextra -Itests/stubs -IUser -include tests/stubs/hmi_test_hal.h tests/hmi_test.c User/hmi.c -o build/hmi_test.exe
build\hmi_test.exe
```

Expected: 编译现有未完成实现时可出现未初始化变量警告；测试在
`uart_call_count == 2U` 处断言失败，因为 `HMI_UART_Send` 尚未发送数据。

### Task 2: 实现 HMI 命令发送

**Files:**
- Modify: `User/hmi.c`
- Modify: `User/hmi.h`
- Test: `tests/hmi_test.c`

- [ ] **Step 1: 补全公开接口的中文 Doxygen 契约**

将 `User/hmi.h` 更新为：

```c
#ifndef _HMI_H_
#define _HMI_H_

#include "main.h"

/**
 * @brief 通过 UART 发送 HMI 命令主体并追加三个 0xFF 帧尾。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送的命令主体。
 * @param[in] size 命令主体字节数。
 * @return 无。
 */
void HMI_UART_Send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);

/**
 * @brief 修改 HMI 文本控件内容。
 * @param[in,out] huart UART 句柄。
 * @param[in] widget 控件名称，例如 `main.t0`。
 * @param[in] txt 待显示文本。
 * @return 无。
 * @note 本函数使用静态缓冲区，不可并发调用；超长命令将被拒绝发送。
 */
void HMI_UART_Send_ModifyTxt(UART_HandleTypeDef *huart, char *widget, char *txt);

#endif // !_HMI_H_
```

- [ ] **Step 2: 编写通过测试的最小实现**

将 `User/hmi.c` 更新为：

```c
#include "hmi.h"

#include <stddef.h>
#include <stdio.h>

#define HMI_COMMAND_BUFFER_SIZE 128U

/**
 * @brief HMI 指令帧尾。
 */
static const uint8_t HMI_send_tail[] = {0xFFU, 0xFFU, 0xFFU};

void HMI_UART_Send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size)
{
    if (huart == NULL || data == NULL || size == 0U)
    {
        return;
    }

    if (HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY) != HAL_OK)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        huart,
        HMI_send_tail,
        (uint16_t)sizeof(HMI_send_tail),
        HAL_MAX_DELAY
    );
}

void HMI_UART_Send_ModifyTxt(UART_HandleTypeDef *huart, char *widget, char *txt)
{
    static char data[HMI_COMMAND_BUFFER_SIZE];
    int written_size;

    if (huart == NULL || widget == NULL || txt == NULL)
    {
        return;
    }

    written_size = snprintf(data, sizeof(data), "%s.txt=\"%s\"", widget, txt);
    if (written_size < 0 || (size_t)written_size >= sizeof(data))
    {
        return;
    }

    HMI_UART_Send(huart, (const uint8_t *)data, (uint16_t)written_size);
}
```

- [ ] **Step 3: 使用严格警告重新编译并确认 GREEN**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -include tests/stubs/hmi_test_hal.h tests/hmi_test.c User/hmi.c -o build/hmi_test.exe
build\hmi_test.exe
```

Expected: 编译无警告，测试退出码为 0。

- [ ] **Step 4: 提交 HMI 实现与主机测试**

```powershell
git add User/hmi.c User/hmi.h tests/hmi_test.c tests/stubs/hmi_test_hal.h
git commit -m "feat: 补全HMI文本命令发送"
```

### Task 3: 执行固件与文本完整性验证

**Files:**
- Verify: `User/hmi.c`
- Verify: `User/hmi.h`
- Verify: `tests/hmi_test.c`
- Verify: `tests/stubs/hmi_test_hal.h`

- [ ] **Step 1: 构建 STM32 Debug 固件**

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Expected: `26rcf.elf` 构建成功，`User/hmi.c` 无编译错误或警告。

- [ ] **Step 2: 检查 UTF-8 文本和补丁格式**

```powershell
@('User/hmi.c','User/hmi.h','tests/hmi_test.c','tests/stubs/hmi_test_hal.h') | ForEach-Object {
    $bytes = [System.IO.File]::ReadAllBytes($_)
    $utf8 = [System.Text.UTF8Encoding]::new($false, $true)
    $null = $utf8.GetString($bytes)
}
git diff --check HEAD~1 -- User/hmi.c User/hmi.h tests/hmi_test.c tests/stubs/hmi_test_hal.h
```

Expected: UTF-8 解码无异常，`git diff --check` 无输出并返回 0。

- [ ] **Step 3: 检查最终提交范围**

```powershell
git status --short
git show --stat --oneline HEAD
```

Expected: 最新提交仅包含两个 HMI 源文件和两个 HMI 测试文件；用户原有的其他工作区改动保持不变。
