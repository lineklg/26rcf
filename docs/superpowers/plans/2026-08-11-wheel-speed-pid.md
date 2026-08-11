# Four-Wheel Speed PID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 STM32 四轮底盘中加入 20 ms 编码器反馈 PID 速度环，使 `Wheel_Forward()`、`Wheel_Turn()` 和 `Wheel_Stop()` 具备已确认的闭环控制语义。

**Architecture:** `Core/User/wheel_pid.c` 通过现有 `PID_Manager` 创建四个独立 PID，并以强定义 `PID_Task()` 覆盖通用模块的弱任务。行为层只把前进、转向和停止命令委托给速度环；停止速度环后仍由 `behavior.c` 调用四路 DRV8870 制动。

**Tech Stack:** C11、STM32H7 HAL、现有 Task/PID/Motor/DRV8870 模块、主机 GCC `assert` 测试、CMake/Ninja。

---

## 文件映射

- Create: `Core/User/wheel_pid.h`，四轮速度环常量与公开接口。
- Create: `Core/User/wheel_pid.c`，四路目标、PID 对象、任务回调和运行状态。
- Create: `tests/wheel_pid_test.c`，使用真实 `pid.c`、`task.c` 的主机行为测试。
- Modify: `Core/User/behavior.c`，将轮子动作接入速度环；不新增注释。
- Modify: `Core/User/user.h`，引入 `wheel_pid.h`；不新增注释。
- Modify: `Core/User/user.c`，在任务管理器初始化后初始化轮子；不新增注释。
- Modify: `CMakeLists.txt`，显式加入 `Core/User/wheel_pid.c`。
- Preserve: `Core/User/behavior.h` 中用户已改好的 `Wheel_*()` 命名。
- Preserve: `User/pid.h` 中用户已改好的 `PID_MAX_COUNT 9`。
- Ignore: 与本任务无关的 `User/vofa.c`、`User/vofa.h` 及其他工作区改动。

### Task 1: 以失败测试定义四轮 PID 基础接口

**Files:**
- Create: `tests/wheel_pid_test.c`
- Create: `Core/User/wheel_pid.h`
- Create: `Core/User/wheel_pid.c`
- Test: `tests/wheel_pid_test.c`

- [ ] **Step 1: 写前进、转向、独立反馈和限幅测试**

先创建 `tests/wheel_pid_test.c`，测试使用真实 PID 和任务实现，只替换硬件测速与
PWM 写入边界。初始完整内容如下：

```c
#include "wheel_pid.h"
#include "task.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t fake_tick;
static Motor test_motors[WHEEL_PID_COUNT];
static DRV8870_Motor test_drivers[WHEEL_PID_COUNT];
static float feedback[WHEEL_PID_COUNT];
static float output[WHEEL_PID_COUNT];
static uint32_t output_count[WHEEL_PID_COUNT];

static uint32_t fake_get_tick(void)
{
    return fake_tick;
}

float Motor_CalcSpeed_Smooth(Motor *motor)
{
    ptrdiff_t index = motor - test_motors;

    assert(index >= 0 && index < (ptrdiff_t)WHEEL_PID_COUNT);
    return feedback[index];
}

void DRV8870_SetDutyPercent(DRV8870_Motor *driver, float percent)
{
    ptrdiff_t index = driver - test_drivers;

    assert(index >= 0 && index < (ptrdiff_t)WHEEL_PID_COUNT);
    output[index] = percent;
    output_count[index]++;
}

static void assert_close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static void reset_fixture(void)
{
    uint8_t i;

    fake_tick = 0U;
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        test_motors[i] = (Motor){0};
        test_drivers[i] = (DRV8870_Motor){0};
        feedback[i] = 0.0f;
        output[i] = 0.0f;
        output_count[i] = 0U;
    }
    Task_Init(fake_get_tick);
    WheelPID_Init(test_motors, test_drivers);
}

static void run_next_period(void)
{
    Task_Update();
    fake_tick += WHEEL_PID_PERIOD_MS;
}

static void test_forward_uses_same_target_for_all_wheels(void)
{
    uint8_t i;

    reset_fixture();
    WheelPID_Forward(0.5f);
    run_next_period();
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        assert_close(output[i], 0.5f);
        assert(output_count[i] == 1U);
    }
}

static void test_turn_uses_opposite_targets(void)
{
    reset_fixture();
    WheelPID_Turn(0.4f);
    run_next_period();
    assert_close(output[0], -0.4f);
    assert_close(output[1], 0.4f);
    assert_close(output[2], -0.4f);
    assert_close(output[3], 0.4f);
}

static void test_each_wheel_uses_its_own_feedback(void)
{
    reset_fixture();
    feedback[0] = 0.0f;
    feedback[1] = 0.1f;
    feedback[2] = 0.2f;
    feedback[3] = 0.3f;
    WheelPID_Forward(0.8f);
    run_next_period();
    assert_close(output[0], 0.8f);
    assert_close(output[1], 0.7f);
    assert_close(output[2], 0.6f);
    assert_close(output[3], 0.5f);
}

static void test_output_is_limited_to_duty_range(void)
{
    reset_fixture();
    WheelPID_Forward(2.0f);
    run_next_period();
    assert_close(output[0], 1.0f);
    assert_close(output[1], 1.0f);
    assert_close(output[2], 1.0f);
    assert_close(output[3], 1.0f);
}

int main(void)
{
    test_forward_uses_same_target_for_all_wheels();
    test_turn_uses_opposite_targets();
    test_each_wheel_uses_its_own_feedback();
    test_output_is_limited_to_duty_range();
    return 0;
}
```

- [ ] **Step 2: 运行测试并确认 RED**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
```

预期：编译失败，首个原因是 `wheel_pid.h` 或 `wheel_pid.c` 尚不存在；失败来自
待实现功能，而不是测试语法。

- [ ] **Step 3: 创建速度环头文件**

`Core/User/wheel_pid.h` 使用 UTF-8 和中文 Doxygen，公开接口固定为：

```c
#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include "DRV8870.h"
#include "motor.h"
#include <stdint.h>

/** @brief 四轮速度环管理的轮子数量。 */
#define WHEEL_PID_COUNT 4U
/** @brief 速度环执行周期，单位为毫秒。 */
#define WHEEL_PID_PERIOD_MS 20U

/**
 * @brief 初始化四轮 PID 速度环。
 * @param[in,out] motors 四个带编码器反馈的电机对象。
 * @param[in,out] drivers 四个 DRV8870 驱动对象。
 * @return 无。
 */
void WheelPID_Init(
    Motor motors[WHEEL_PID_COUNT],
    DRV8870_Motor drivers[WHEEL_PID_COUNT]
);

/**
 * @brief 设置四轮同向目标线速度并启动速度环。
 * @param speed 目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_Forward(float speed);

/**
 * @brief 设置原地转向目标线速度并启动速度环。
 * @param speed 转向速度幅值，单位为 m/s。
 * @return 无。
 */
void WheelPID_Turn(float speed);

/**
 * @brief 停止速度环并清除四轮目标及 PID 历史。
 * @return 无。
 */
void WheelPID_Stop(void);

/**
 * @brief 设置指定轮子的 PID 参数。
 * @param wheel_index 轮子索引，有效范围为 0 到 3。
 * @param kp 比例系数。
 * @param ki 积分系数。
 * @param kd 微分系数。
 * @return 无。
 */
void WheelPID_SetK(
    uint8_t wheel_index,
    float kp,
    float ki,
    float kd
);

#endif /* WHEEL_PID_H */
```

- [ ] **Step 4: 写使基础测试通过的最小实现**

`Core/User/wheel_pid.c` 先实现初始化、启动、前进、转向和周期输出：

```c
#include "wheel_pid.h"
#include "pid.h"

#include <stddef.h>

static PID *wheel_pid[WHEEL_PID_COUNT];
static Motor *wheel_motor;
static DRV8870_Motor *wheel_driver;
static float wheel_target[WHEEL_PID_COUNT];
static uint8_t wheel_pid_running;

static void WheelPID_Start(void)
{
    uint8_t i;

    if (wheel_pid_running != 0U) {
        return;
    }
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        if (wheel_pid[i] != NULL) {
            PID_ResetHistory(wheel_pid[i]);
        }
    }
    wheel_pid_running = 1U;
    PID_Start();
}

void WheelPID_Init(
    Motor motors[WHEEL_PID_COUNT],
    DRV8870_Motor drivers[WHEEL_PID_COUNT]
)
{
    uint8_t i;

    wheel_motor = motors;
    wheel_driver = drivers;
    wheel_pid_running = 0U;
    PID_Init();
    PID_SetPeriodMs(WHEEL_PID_PERIOD_MS);
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_target[i] = 0.0f;
        wheel_pid[i] = NULL;
        if (PID_Create(&wheel_pid[i], 1.0f, 0.0f, 0.0f) != UINT16_MAX) {
            PID_SetOutputMax(wheel_pid[i], 1.0f);
        }
    }
}

void WheelPID_Forward(float speed)
{
    uint8_t i;

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_target[i] = speed;
    }
    WheelPID_Start();
}

void WheelPID_Turn(float speed)
{
    wheel_target[0] = -speed;
    wheel_target[1] = speed;
    wheel_target[2] = -speed;
    wheel_target[3] = speed;
    WheelPID_Start();
}

void PID_Task(void)
{
    uint8_t i;

    if (wheel_pid_running == 0U || wheel_motor == NULL ||
        wheel_driver == NULL) {
        return;
    }
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        if (wheel_pid[i] != NULL) {
            float feedback = Motor_CalcSpeed_Smooth(&wheel_motor[i]);
            float duty = PID_Calc(wheel_pid[i], wheel_target[i], feedback);
            DRV8870_SetDutyPercent(&wheel_driver[i], duty);
        }
    }
}
```

本任务的基础测试不调用 `WheelPID_Stop()` 和 `WheelPID_SetK()`；Task 2 将先用
失败测试定义这两个接口的生命周期行为。

- [ ] **Step 5: 运行基础测试确认 GREEN**

重复 Step 2 的编译命令并运行：

```powershell
build\wheel_pid_test.exe
```

预期：编译无警告，进程退出码为 0，标准输出为空。

- [ ] **Step 6: 提交基础速度环**

```powershell
git add Core/User/wheel_pid.h Core/User/wheel_pid.c tests/wheel_pid_test.c
git commit -m "feat: 添加四轮 PID 速度调节"
```

### Task 2: 以失败测试定义参数修改和停止生命周期

**Files:**
- Modify: `tests/wheel_pid_test.c`
- Modify: `Core/User/wheel_pid.c`

- [ ] **Step 1: 添加运行中保留历史、停止休眠、重启清历史和无效轮号测试**

在测试文件中加入：

```c
static void test_running_command_keeps_integral_history(void)
{
    reset_fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_Forward(1.0f);
    run_next_period();
    assert_close(output[0], 0.02f);
    WheelPID_Forward(1.0f);
    run_next_period();
    assert_close(output[0], 0.04f);
}

static void test_stop_sleeps_task_and_restart_clears_history(void)
{
    uint32_t writes_after_stop;

    reset_fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_Forward(1.0f);
    run_next_period();
    WheelPID_Stop();
    writes_after_stop = output_count[0];
    run_next_period();
    assert(output_count[0] == writes_after_stop);
    WheelPID_Forward(0.0f);
    run_next_period();
    assert_close(output[0], 0.0f);
}

static void test_invalid_wheel_index_does_not_change_pid(void)
{
    reset_fixture();
    WheelPID_SetK(WHEEL_PID_COUNT, 0.0f, 0.0f, 0.0f);
    WheelPID_Forward(0.5f);
    run_next_period();
    assert_close(output[0], 0.5f);
    assert_close(output[1], 0.5f);
    assert_close(output[2], 0.5f);
    assert_close(output[3], 0.5f);
}
```

在 `main()` 返回前调用这三个测试。

- [ ] **Step 2: 运行测试并确认 RED**

重复 Task 1 的编译和执行命令。

预期：链接失败，报告 `WheelPID_SetK` 和 `WheelPID_Stop` 未定义。该失败证明新增
断言确实覆盖尚未实现的生命周期接口。

- [ ] **Step 3: 实现参数设置和停止**

在 `wheel_pid.c` 中加入：

```c
void WheelPID_SetK(
    uint8_t wheel_index,
    float kp,
    float ki,
    float kd
)
{
    if (wheel_index < WHEEL_PID_COUNT && wheel_pid[wheel_index] != NULL) {
        PID_SetK(wheel_pid[wheel_index], kp, ki, kd);
    }
}

void WheelPID_Stop(void)
{
    uint8_t i;

    PID_Stop();
    wheel_pid_running = 0U;
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_target[i] = 0.0f;
        if (wheel_pid[i] != NULL) {
            PID_ResetHistory(wheel_pid[i]);
        }
    }
}
```

- [ ] **Step 4: 运行测试确认 GREEN**

重复 Task 1 的编译和执行命令。

预期：所有断言通过，退出码为 0；尤其是停止期间写入计数保持不变，重启后的
积分输出为零。

- [ ] **Step 5: 提交生命周期实现**

```powershell
git add Core/User/wheel_pid.c tests/wheel_pid_test.c
git commit -m "feat: 完善轮速 PID 启停控制"
```

### Task 3: 接入行为层与系统初始化

**Files:**
- Modify: `Core/User/behavior.c:1-52`
- Modify: `Core/User/user.h:8-18`
- Modify: `Core/User/user.c:35-43`
- Modify: `CMakeLists.txt:53-58`

- [ ] **Step 1: 先确认新增模块尚未进入固件构建**

运行：

```powershell
cmake --build --preset Debug
```

预期：现有固件仍可构建，但 `build/Debug/compile_commands.json` 中搜索不到
`Core/User/wheel_pid.c`。这说明接入测试能区分“模块存在”和“模块已进入固件”。

- [ ] **Step 2: 将模块加入头文件聚合和 CMake**

在 `Core/User/user.h` 的用户模块 include 区加入：

```c
#include "wheel_pid.h"
```

在 `CMakeLists.txt` 的 `target_sources()` 中加入：

```cmake
    \${CMAKE_SOURCE_DIR}/Core/User/wheel_pid.c
```

不修改 `User/*.c` 的递归收集逻辑。

- [ ] **Step 3: 接入 Wheel 行为且不新增注释**

`Wheel_Init()` 完成四个 `Motor_Init()` 后加入：

```c
    WheelPID_Init(motor, motor_ic);
    Wheel_Stop();
```

用以下实现替换现有开环前进和转向函数体：

```c
void Wheel_Forward(float speed)
{
    WheelPID_Forward(speed);
}

void Wheel_Turn(float speed)
{
    WheelPID_Turn(speed);
}
```

`Wheel_Stop()` 必须先停止速度环，再保留现有四轮制动循环：

```c
void Wheel_Stop(void)
{
    WheelPID_Stop();
    for (uint8_t i = 0; i < 4; i++)
    {
        DRV8870_Brake(&motor_ic[i]);
    }
}
```

不要在 `behavior.c` 或 `behavior.h` 新增注释。

- [ ] **Step 4: 在正确初始化顺序中启用轮子**

在 `User_Init()` 中紧跟 `Task_Init(HAL_GetTick);` 添加：

```c
    Wheel_Init();
```

保持 `TimeUs_Init()` 在它之前，使 `Motor_Init()` 使用的微秒时基已经可用；
保持 `Task_Init()` 在它之前，使 `PID_Init()` 能创建有效周期任务。

- [ ] **Step 5: 构建固件并确认接入 GREEN**

运行：

```powershell
cmake --build --preset Debug
```

预期：Ninja 返回退出码 0，生成 `build/Debug/26rcf.elf`，且
`build/Debug/compile_commands.json` 中能搜索到 `Core/User/wheel_pid.c`。

- [ ] **Step 6: 提交行为层接入**

```powershell
git add Core/User/behavior.c Core/User/user.h Core/User/user.c CMakeLists.txt
git commit -m "feat: 接入四轮闭环速度控制"
```

不要暂存用户已修改的 `Core/User/behavior.h`、`User/pid.h` 或无关 VOFA 文件。

### Task 4: 完整回归、编码和范围验证

**Files:**
- Verify: `Core/User/wheel_pid.h`
- Verify: `Core/User/wheel_pid.c`
- Verify: `Core/User/behavior.c`
- Verify: `Core/User/behavior.h`
- Verify: `Core/User/user.h`
- Verify: `Core/User/user.c`
- Verify: `CMakeLists.txt`
- Verify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 运行新增轮速 PID 测试**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_pid_test.exe
build\wheel_pid_test.exe
```

预期：编译无警告，测试退出码为 0。

- [ ] **Step 2: 运行全部现有主机测试**

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

预期：四个测试程序都以退出码 0 结束，标准输出无错误。

- [ ] **Step 3: 运行 STM32 Debug 构建**

```powershell
cmake --build --preset Debug
```

预期：构建退出码为 0 并生成 `build/Debug/26rcf.elf`。

- [ ] **Step 4: 检查 UTF-8、Doxygen 和 behavior 注释约束**

```powershell
@('Core/User/wheel_pid.h','Core/User/wheel_pid.c','tests/wheel_pid_test.c') |
  ForEach-Object {
    $bytes = [IO.File]::ReadAllBytes($_)
    $utf8 = New-Object Text.UTF8Encoding($false,$true)
    [void]$utf8.GetString($bytes)
  }
rg -n "^/\\*\\*|@brief|@param|@return" Core/User/wheel_pid.h Core/User/wheel_pid.c
git diff --word-diff=plain -- Core/User/behavior.c Core/User/behavior.h
```

预期：UTF-8 解码无异常；新模块公开接口均有中文 Doxygen；`behavior.c/.h` 的
差异中没有新增注释。

- [ ] **Step 5: 检查工作区范围**

```powershell
git status --short
git diff --check
git diff -- Core/User/wheel_pid.h Core/User/wheel_pid.c Core/User/behavior.c Core/User/user.h Core/User/user.c CMakeLists.txt tests/wheel_pid_test.c
```

预期：无空白错误；不还原、不覆盖、不暂存用户的 `behavior.h`、`pid.h`、
`vofa.c/.h` 或其他无关修改。

- [ ] **Step 6: 记录实际验证结果**

最终交付中报告新增测试、现有测试和 STM32 构建的实际结果，并列出用户仍保留的
未提交改动。不得把未运行或失败的命令描述为通过。
