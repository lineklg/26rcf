# 四轮 PID VOFA 串口上报 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在每次四轮速度 PID 周期完成 PWM 写入后，通过 `huart1` 向 VOFA+ 发送一帧固定顺序的 12 通道 JustFloat 数据。

**Architecture:** `PID_Task()` 继续负责四轮反馈采样、PID 计算和驱动输出，并用一个局部 `float[12]` 同步收集目标、反馈和输出。循环结束后只调用一次现有 `VOFA_JustFloat_UART_Send()`；UART 错误被显式忽略，不进入控制状态。

**Tech Stack:** STM32 HAL、C11、现有 PID/Task/VOFA 模块、GCC 主机断言测试、CMake ARM 交叉构建。

---

### Task 1: 建立 VOFA 主机测试边界

**Files:**
- Modify: `tests/stubs/main.h`
- Modify: `tests/wheel_pid_test.c`
- Test: `tests/wheel_pid_test.c`

- [ ] **Step 1: 为主机测试增加最小 UART 句柄类型**

在 `tests/stubs/main.h` 的 HAL 类型区加入：

```c
/** @brief 主机测试使用的最小 UART 句柄。 */
typedef struct {
    uint32_t instance;
} UART_HandleTypeDef;
```

- [ ] **Step 2: 写入 VOFA 调用捕获桩**

在 `tests/wheel_pid_test.c` 引入 `vofa.h`，定义 `huart1`、发送次数、通道数、12 元素数据副本、发送返回状态和发送瞬间的四路 PWM 写入次数，并实现：

```c
HAL_StatusTypeDef VOFA_JustFloat_UART_Send(
    UART_HandleTypeDef *huart,
    const float *data,
    uint8_t channel_num
)
{
    size_t i;

    vofa_uart = huart;
    vofa_channel_num = channel_num;
    vofa_send_count++;
    memcpy(vofa_data, data, sizeof(vofa_data));
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        output_count_at_send[i] = output_count[i];
    }
    return vofa_status;
}
```

在 `reset_fixture()` 中将上述状态清零，并将 `vofa_status` 设为 `HAL_OK`。

- [ ] **Step 3: 写入 12 通道顺序和单次发送测试**

增加 `test_vofa_sends_one_ordered_frame_after_pwm()`：设置四路不同反馈，直行目标为 `0.8f`，执行一个周期后断言：

```c
const float expected[12] = {
    0.8f, 0.0f, 0.8f,
    0.8f, 0.1f, 0.7f,
    0.8f, 0.2f, 0.6f,
    0.8f, 0.3f, 0.5f
};

assert(vofa_send_count == 1U);
assert(vofa_uart == &huart1);
assert(vofa_channel_num == 12U);
```

逐项使用 `assert_float_close()` 比较 `vofa_data`，并断言 `output_count_at_send[0..3]` 均为 `1U`，证明发送发生在四路 PWM 写入之后。

- [ ] **Step 4: 写入停止与 UART 错误隔离测试**

增加 `test_stop_prevents_vofa_send()`：先运行一周期，调用 `WheelPID_Stop()`，再运行一周期，断言发送次数保持不变。增加 `test_vofa_error_does_not_change_pwm_outputs()`：将 `vofa_status` 设为 `HAL_ERROR`，运行一周期，断言发送仍调用一次且 `assert_outputs()` 通过。

- [ ] **Step 5: 编译并运行测试，确认 RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User -ICore/Inc tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build/wheel_pid_test.exe
```

Expected: 链接或断言失败，原因是 `PID_Task()` 尚未调用 `VOFA_JustFloat_UART_Send()`；不能因测试桩类型、声明或数组越界而失败。

### Task 2: 实现每周期一帧 VOFA 上报

**Files:**
- Modify: `Core/User/wheel_pid.c`
- Test: `tests/wheel_pid_test.c`

- [ ] **Step 1: 接入现有 VOFA 与 UART 声明**

在 `wheel_pid.c` 增加：

```c
#include "usart.h"
#include "vofa.h"
```

- [ ] **Step 2: 在 PID 循环中收集 12 通道数据**

在 `PID_Task()` 通过零初始化保证缺失 PID 的反馈和输出为零，并始终写入目标：

```c
float vofa_data[WHEEL_PID_COUNT * 3U] = {0.0f};

for (i = 0U; i < WHEEL_PID_COUNT; i++) {
    float feedback;
    float output;
    uint8_t channel = i * 3U;

    vofa_data[channel] = wheel_target[i];
    if (wheel_pid[i] == NULL) {
        continue;
    }
    feedback = Motor_CalcSpeed_Smooth(&wheel_motor[i]);
    output = PID_Calc(wheel_pid[i], wheel_target[i], feedback);
    vofa_data[channel + 1U] = feedback;
    vofa_data[channel + 2U] = output;
    DRV8870_SetDutyPercent(&wheel_driver[i], output);
}
```

- [ ] **Step 3: 在四路 PWM 写入后发送一次**

循环结束后增加：

```c
(void)VOFA_JustFloat_UART_Send(
    &huart1,
    vofa_data,
    (uint8_t)(WHEEL_PID_COUNT * 3U)
);
```

不修改 PID 状态、不重试、不修改 `vofa.c/.h`、`behavior.c/.h` 或 `user.c`。

- [ ] **Step 4: 编译并运行轮速测试，确认 GREEN**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User -ICore/Inc tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build/wheel_pid_test.exe
```

Expected: 编译无警告，进程返回码为 `0`。

### Task 3: 回归与交付检查

**Files:**
- Verify: `Core/User/wheel_pid.c`
- Verify: `tests/wheel_pid_test.c`
- Verify: `tests/stubs/main.h`

- [ ] **Step 1: 运行所有现有主机测试**

分别按各测试既有源码依赖使用 `gcc -std=c11 -Wall -Wextra -Werror` 编译并运行 `encoder_motor_test.c`、`servo_position_test.c`、`state_machine_test.c`、`time_us_test.c` 和 `wheel_pid_test.c`。

Expected: 五个测试程序均返回 `0`，编译无新增警告。

- [ ] **Step 2: 运行 STM32 ARM Debug 构建**

Run:

```powershell
cmake -S . -B build/Debug-Make2 -G "MinGW Makefiles" "-DCMAKE_MAKE_PROGRAM=C:/Program Files/msys2/ucrt64/bin/mingw32-make.exe" "-DCMAKE_TOOLCHAIN_FILE=D:/Workspace/microcontroller/STM32/competitions/26rcf/cmake/gcc-arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug-Make2
```

Expected: 配置和构建均返回 `0`，最终生成 `26rcf.elf`。

- [ ] **Step 3: 检查范围、编码和用户并行改动**

Run:

```powershell
git diff -- Core/User/wheel_pid.c tests/wheel_pid_test.c tests/stubs/main.h
git status --short
```

Expected: 功能改动只出现在计划文件中列明的三个代码文件；`Core/User/user.c` 仍保持用户原有 `MM` 状态且未被本任务修改。新增中文 Doxygen 注释以 UTF-8 保存，`behavior.c/.h` 与 `vofa.c/.h` 无变化。

- [ ] **Step 4: 提交实现文件**

Run:

```powershell
git add -- Core/User/wheel_pid.c tests/wheel_pid_test.c tests/stubs/main.h docs/superpowers/plans/2026-08-11-wheel-pid-vofa.md
git commit --only Core/User/wheel_pid.c tests/wheel_pid_test.c tests/stubs/main.h docs/superpowers/plans/2026-08-11-wheel-pid-vofa.md -m "feat: 添加轮速 PID VOFA 上报"
```

Expected: 提交仅包含上述四个文件，`user.c` 的暂存和未暂存内容继续保留。
