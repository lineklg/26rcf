# Independent Wheel Speed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为现有四轮 PID 速度环增加单轮和四轮目标速度设置接口，并在设置有效目标时自动启动速度环。

**Architecture:** 继续使用 `wheel_pid.c` 内部的四元素目标数组和现有启动逻辑；新接口只负责按索引或按 A-D 顺序更新目标。主机测试使用真实 `pid.c` 与 `task.c`，在 VOFA 发送边界捕获每轮目标，并在电机驱动边界记录任务是否执行。

**Tech Stack:** C11、STM32H7、现有 PID/Task/WheelPID 模块、主机 GCC `assert` 测试、CMake/Ninja。

---

## 文件映射

- Create: `tests/wheel_pid_test.c`，验证独立目标设置、自动启动、运行中历史保留和无效索引。
- Modify: `Core/User/wheel_pid.h`，声明两个公开接口并提供中文 Doxygen 注释。
- Modify: `Core/User/wheel_pid.c`，实现目标更新并复用现有 `WheelPID_Start()`。
- Preserve: `Core/User/user.c`、`Core/User/user.h`、`User/vofa.c`、`User/vofa.h` 中已有的未提交修改。

### Task 1: 用失败测试定义四轮和单轮设置行为

**Files:**
- Create: `tests/wheel_pid_test.c`
- Test: `tests/wheel_pid_test.c`

- [ ] **Step 1: 编写目标设置测试与硬件边界桩**

创建 `tests/wheel_pid_test.c`。测试桩记录每轮驱动写入次数，并从 `VOFA_JustFloat_UART_Send()` 的通道 `0`、`3`、`6`、`9` 捕获 PID 任务实际使用的四个目标：

```c
#include "wheel_pid.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t fake_tick;
static Motor test_motors[WHEEL_PID_COUNT];
static DRV8870_Motor test_drivers[WHEEL_PID_COUNT];
static uint32_t duty_write_count[WHEEL_PID_COUNT];
static float sent_target[WHEEL_PID_COUNT];
static uint32_t send_count;

UART_HandleTypeDef huart1;

static uint32_t Fake_GetTick(void)
{
    return fake_tick;
}

float Motor_CalcSpeed_Smooth(Motor *motor)
{
    assert(motor >= test_motors);
    assert(motor < test_motors + WHEEL_PID_COUNT);
    return 0.0f;
}

void DRV8870_SetDutyPercent(DRV8870_Motor *driver, float percent)
{
    ptrdiff_t index = driver - test_drivers;

    (void)percent;
    assert(index >= 0);
    assert(index < (ptrdiff_t)WHEEL_PID_COUNT);
    duty_write_count[index]++;
}

HAL_StatusTypeDef VOFA_JustFloat_UART_Send(
    UART_HandleTypeDef *huart,
    const float *data,
    uint8_t channel_num
)
{
    uint8_t i;

    assert(huart == &huart1);
    assert(channel_num == WHEEL_PID_COUNT * 3U);
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        sent_target[i] = data[i * 3U];
    }
    send_count++;
    return HAL_OK;
}

static void Reset_Fixture(void)
{
    uint8_t i;

    fake_tick = 0U;
    send_count = 0U;
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        test_motors[i] = (Motor){0};
        test_drivers[i] = (DRV8870_Motor){0};
        duty_write_count[i] = 0U;
        sent_target[i] = 0.0f;
    }
    Task_Init(Fake_GetTick);
    WheelPID_Init(test_motors, test_drivers);
}

static void Run_Current_Tick(void)
{
    Task_Update();
}

static void Run_Next_Period(void)
{
    fake_tick += WHEEL_PID_PERIOD_MS;
    Task_Update();
}

static void Assert_Targets(float a, float b, float c, float d)
{
    assert(sent_target[0] == a);
    assert(sent_target[1] == b);
    assert(sent_target[2] == c);
    assert(sent_target[3] == d);
}

static void Test_SetSpeeds_Updates_All_Wheels_And_Starts(void)
{
    Reset_Fixture();
    WheelPID_SetSpeeds(0.1f, 0.2f, -0.3f, -0.4f);
    Run_Current_Tick();
    assert(send_count == 1U);
    Assert_Targets(0.1f, 0.2f, -0.3f, -0.4f);
}

static void Test_SetSpeed_Only_Updates_Selected_Wheel(void)
{
    Reset_Fixture();
    WheelPID_SetSpeeds(0.1f, 0.2f, 0.3f, 0.4f);
    Run_Current_Tick();
    WheelPID_SetSpeed(2U, -0.6f);
    Run_Next_Period();
    assert(send_count == 2U);
    Assert_Targets(0.1f, 0.2f, -0.6f, 0.4f);
}

static void Test_Invalid_Index_Does_Not_Start_Or_Modify(void)
{
    Reset_Fixture();
    WheelPID_SetSpeed(WHEEL_PID_COUNT, 0.8f);
    Run_Current_Tick();
    assert(send_count == 0U);

    WheelPID_SetSpeeds(0.1f, 0.2f, 0.3f, 0.4f);
    Run_Current_Tick();
    WheelPID_SetSpeed(WHEEL_PID_COUNT, 0.8f);
    Run_Next_Period();
    Assert_Targets(0.1f, 0.2f, 0.3f, 0.4f);
}

static void Test_Running_Update_Keeps_PID_History(void)
{
    Reset_Fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_SetSpeeds(0.1f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    assert(duty_write_count[0] == 1U);
    WheelPID_SetSpeed(0U, 0.2f);
    Run_Next_Period();
    assert(duty_write_count[0] == 2U);
    Assert_Targets(0.2f, 0.0f, 0.0f, 0.0f);
}

int main(void)
{
    Test_SetSpeeds_Updates_All_Wheels_And_Starts();
    Test_SetSpeed_Only_Updates_Selected_Wheel();
    Test_Invalid_Index_Does_Not_Start_Or_Modify();
    Test_Running_Update_Keeps_PID_History();
    return 0;
}
```

- [ ] **Step 2: 编译测试并确认 RED**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
```

预期：编译或链接失败，明确报告 `WheelPID_SetSpeed` 与 `WheelPID_SetSpeeds` 尚未声明或定义。失败原因必须来自缺少新功能，而不是测试语法或桩函数错误。

- [ ] **Step 3: 提交失败测试**

```powershell
git add tests/wheel_pid_test.c
git commit -m "test: 定义四轮独立速度设置行为"
```

### Task 2: 实现独立目标速度接口

**Files:**
- Modify: `Core/User/wheel_pid.h`
- Modify: `Core/User/wheel_pid.c`
- Test: `tests/wheel_pid_test.c`

- [ ] **Step 1: 在头文件声明公开接口**

在 `WheelPID_Turn()` 声明后加入中文 Doxygen 声明：

```c
/**
 * @brief 设置指定车轮的目标线速度并启动速度环。
 * @param[in] index 车轮索引，0 至 3 分别对应 A 至 D 轮。
 * @param[in] speed 目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeed(uint8_t index, float speed);

/**
 * @brief 分别设置 A、B、C、D 四轮目标线速度并启动速度环。
 * @param[in] speed_a A 轮目标线速度，单位为 m/s。
 * @param[in] speed_b B 轮目标线速度，单位为 m/s。
 * @param[in] speed_c C 轮目标线速度，单位为 m/s。
 * @param[in] speed_d D 轮目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeeds(
    float speed_a,
    float speed_b,
    float speed_c,
    float speed_d
);
```

- [ ] **Step 2: 在实现文件更新目标并复用启动逻辑**

在 `WheelPID_Turn()` 后加入：

```c
/**
 * @brief 设置指定车轮的目标线速度并启动速度环。
 * @param[in] index 车轮索引，0 至 3 分别对应 A 至 D 轮。
 * @param[in] speed 目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeed(uint8_t index, float speed)
{
    if (index >= WHEEL_PID_COUNT) {
        return;
    }

    wheel_target[index] = speed;
    WheelPID_Start();
}

/**
 * @brief 分别设置 A、B、C、D 四轮目标线速度并启动速度环。
 * @param[in] speed_a A 轮目标线速度，单位为 m/s。
 * @param[in] speed_b B 轮目标线速度，单位为 m/s。
 * @param[in] speed_c C 轮目标线速度，单位为 m/s。
 * @param[in] speed_d D 轮目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeeds(
    float speed_a,
    float speed_b,
    float speed_c,
    float speed_d
)
{
    wheel_target[0] = speed_a;
    wheel_target[1] = speed_b;
    wheel_target[2] = speed_c;
    wheel_target[3] = speed_d;
    WheelPID_Start();
}
```

- [ ] **Step 3: 编译并运行测试确认 GREEN**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build\wheel_pid_test.exe
```

预期：编译无警告，测试进程退出码为 `0` 且没有断言失败。

- [ ] **Step 4: 提交最小实现**

```powershell
git add Core/User/wheel_pid.h Core/User/wheel_pid.c
git commit -m "feat: 支持四轮独立目标速度"
```

### Task 3: 完整回归与编码检查

**Files:**
- Verify: `Core/User/wheel_pid.h`
- Verify: `Core/User/wheel_pid.c`
- Verify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 重新运行新增主机测试**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build\wheel_pid_test.exe
```

预期：退出码为 `0`。

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
```

预期：四个测试进程均以退出码 `0` 结束。

- [ ] **Step 3: 构建 STM32 固件**

```powershell
cmake --build --preset Debug
```

预期：构建成功并生成 `build/Debug/26rcf.elf`，没有因新接口产生编译或链接错误。

- [ ] **Step 4: 检查 UTF-8、Doxygen 和差异范围**

```powershell
@('Core/User/wheel_pid.h','Core/User/wheel_pid.c','tests/wheel_pid_test.c') | ForEach-Object {
    $bytes = [IO.File]::ReadAllBytes($_)
    $utf8 = New-Object Text.UTF8Encoding($false, $true)
    [void]$utf8.GetString($bytes)
}
Select-String -Path Core/User/wheel_pid.h,Core/User/wheel_pid.c -Pattern '@brief|@param|@return' -Encoding UTF8
git diff --check
git status --short
```

预期：UTF-8 严格解码成功；两个接口的中文 Doxygen 字段完整；无空白错误；用户已有的 `user.c`、`user.h`、`vofa.c`、`vofa.h` 修改保持原样。
