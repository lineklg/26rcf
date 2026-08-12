# VOFA 小端字节转浮点数 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 VOFA 模块中新增安全、明确按小端解释 `uint8_t[4]` 的 `float` 转换函数。

**Architecture:** 公共头文件声明 `VOFA_BytesToFloatLE()`，实现文件显式组装 `uint32_t` 位模式并通过 `memcpy` 复制到 `float`。独立主机测试使用真实 `vofa.c`，仅提供 UART HAL 的最小测试桩。

**Tech Stack:** C11、STM32 HAL 接口、GCC 主机测试、CMake/Ninja 固件构建。

---

## 文件结构

- Create: `tests/vofa_test.c`，验证小端字节到浮点数的转换行为，并提供 UART 发送测试桩。
- Modify: `tests/stubs/main.h`，补齐 `vofa.c` 主机编译所需的 UART HAL 最小声明。
- Modify: `User/vofa.h`，声明新公共函数并添加中文 Doxygen 注释。
- Modify: `User/vofa.c`，实现与处理器字节序无关的转换逻辑。

### Task 1: 编写转换函数的失败测试

**Files:**
- Create: `tests/vofa_test.c`
- Modify: `tests/stubs/main.h`
- Test: `tests/vofa_test.c`

- [ ] **Step 1: 补齐 UART HAL 测试桩声明**

在 `tests/stubs/main.h` 的 UART 句柄定义后加入：

```c
#define HAL_MAX_DELAY UINT32_MAX

/**
 * @brief 主机测试使用的 UART 阻塞发送桩。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送数据。
 * @param[in] size 数据长度。
 * @param[in] timeout 超时时间。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout);
```

- [ ] **Step 2: 创建转换行为测试**

创建 `tests/vofa_test.c`：

```c
#include "vofa.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t size,
                                    uint32_t timeout)
{
    (void)huart;
    (void)data;
    (void)size;
    (void)timeout;
    return HAL_OK;
}

static void test_little_endian_bytes_to_float(void)
{
    const uint8_t zero[4] = {0x00U, 0x00U, 0x00U, 0x00U};
    const uint8_t one[4] = {0x00U, 0x00U, 0x80U, 0x3FU};
    const uint8_t negative[4] = {0x00U, 0x00U, 0x20U, 0xC0U};

    assert(VOFA_BytesToFloatLE(zero) == 0.0f);
    assert(VOFA_BytesToFloatLE(one) == 1.0f);
    assert(VOFA_BytesToFloatLE(negative) == -2.5f);
}

static void test_null_bytes_return_zero(void)
{
    assert(VOFA_BytesToFloatLE(NULL) == 0.0f);
}

int main(void)
{
    test_little_endian_bytes_to_float();
    test_null_bytes_return_zero();
    return 0;
}
```

- [ ] **Step 3: 编译测试并确认因接口缺失而失败**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/vofa_test.c User/vofa.c -o build/vofa_test.exe
```

Expected: FAIL，错误指出 `VOFA_BytesToFloatLE` 未声明或未定义。

### Task 2: 实现小端字节转换接口

**Files:**
- Modify: `User/vofa.h`
- Modify: `User/vofa.c`
- Test: `tests/vofa_test.c`

- [ ] **Step 1: 在头文件中声明接口**

在 `User/vofa.h` 的发送函数声明前加入：

```c
/**
 * @brief 将四个小端字节转换为单精度浮点数。
 *
 * 输入采用小端顺序：data[0] 为最低有效字节，data[3] 为最高有效字节。
 *
 * @param data 包含 IEEE 754 单精度位模式的四字节数组。
 * @return 转换后的浮点数；data 为 NULL 时返回 0.0f。
 */
float VOFA_BytesToFloatLE(const uint8_t data[4]);
```

- [ ] **Step 2: 在实现文件中加入编译期检查和最小实现**

在 `User/vofa.c` 中加入：

```c
_Static_assert(sizeof(float) == sizeof(uint32_t),
               "VOFA float conversion requires 32-bit float");

float VOFA_BytesToFloatLE(const uint8_t data[4])
{
    uint32_t bits;
    float value;

    if (data == NULL)
    {
        return 0.0f;
    }

    bits = (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
    memcpy(&value, &bits, sizeof(value));
    return value;
}
```

- [ ] **Step 3: 编译并运行测试确认通过**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/vofa_test.c User/vofa.c -o build/vofa_test.exe
.\build\vofa_test.exe
```

Expected: 编译退出码为 0，测试程序退出码为 0 且无输出。

### Task 3: 完整验证与提交

**Files:**
- Verify: `User/vofa.h`
- Verify: `User/vofa.c`
- Verify: `tests/stubs/main.h`
- Verify: `tests/vofa_test.c`

- [ ] **Step 1: 检查 UTF-8 文本和中文 Doxygen 字段**

Run:

```powershell
rg -n "^/\*\*|@brief|@param|@return" User/vofa.h User/vofa.c tests/stubs/main.h tests/vofa_test.c
```

Expected: 新增公共接口和测试桩声明包含中文 `@brief`、`@param`、`@return`。

- [ ] **Step 2: 运行 VOFA 主机测试**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/vofa_test.c User/vofa.c -o build/vofa_test.exe
.\build\vofa_test.exe
```

Expected: 两条命令退出码均为 0。

- [ ] **Step 3: 构建 STM32 固件**

Run:

```powershell
cmake --build --preset Debug
```

Expected: 构建成功，新增接口没有编译或链接错误。

- [ ] **Step 4: 检查变更范围**

Run:

```powershell
git diff --check -- User/vofa.h User/vofa.c tests/stubs/main.h tests/vofa_test.c
git diff -- User/vofa.h User/vofa.c tests/stubs/main.h tests/vofa_test.c
```

Expected: 无空白错误，差异仅包含本计划所述接口、实现和测试支持。

- [ ] **Step 5: 提交实现**

```powershell
git add User/vofa.h User/vofa.c tests/stubs/main.h tests/vofa_test.c docs/superpowers/plans/2026-08-12-vofa-bytes-to-float.md
git commit -m "feat: 添加 VOFA 小端字节转浮点函数"
```
