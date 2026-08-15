# 单轴位置保持 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在四轮差速底盘转向期间，用一个雷达位置 PID 保持运行时选定的世界坐标 X 轴或 Y 轴位置。

**Architecture:** `wheel_pid.c` 创建唯一的 `radar_pos_error_pid`，由公开接口保存有效目标轴和显式目标位置。`PID_Task()` 仅在 `enable_fix_pos == 1U` 且雷达反馈有效时计算位置误差，将 X/Y 修正分别按 `cosf(yaw)` 或 `sinf(yaw)` 投影，并同号叠加到四轮输出；使能原始值或目标轴变化时清除该 PID 历史。

**Tech Stack:** C11、现有 `User/pid.c` 与 `User/task.c`、GCC 主机测试、STM32 CMake/Ninja 固件工程。

---

### Task 1: 先定义并验证位置保持行为

**Files:**
- Modify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 扩展雷达位置测试夹具**

在 `radar_get_angle` 旁新增固件侧同名位置反馈：

```c
volatile float radar_get_axis[2];
```

在 `Reset_Fixture()` 的 `radar_get_angle = 0.0f;` 后加入：

```c
radar_get_axis[0] = 0.0f;
radar_get_axis[1] = 0.0f;
enable_fix_pos = 0U;
```

复用现有 `__wrap_PID_ResetHistory()` 统计真实 PID 历史清零次数；把计数器重命名为 `pid_reset_count`，避免它只表示角度 PID 的误导。

- [ ] **Step 2: 写禁用和 X/Y 投影失败测试**

新增并在 `main()` 中调用以下三个独立测试：

```c
static void Test_Position_PID_Disabled_Does_Not_Change_Output(void)
{
    Reset_Fixture();
    WheelPID_SetTargetPosition(WHEEL_PID_POSITION_AXIS_X, 0.04f);
    enable_fix_pos = 0U;
    WheelPID_SetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Duties(0.0f, 0.0f, 0.0f, 0.0f);
}

static void Test_Position_PID_Projects_X_Correction(void)
{
    Reset_Fixture();
    WheelPID_SetTargetPosition(WHEEL_PID_POSITION_AXIS_X, 0.04f);
    radar_get_axis[0] = 0.0f;
    radar_get_angle = 0.0f;
    enable_fix_pos = 1U;
    WheelPID_SetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Duties(0.04f, 0.04f, 0.04f, 0.04f);
}

static void Test_Position_PID_Projects_Y_Correction(void)
{
    const float pi = 3.14159265358979323846f;

    Reset_Fixture();
    WheelPID_SetTargetPosition(WHEEL_PID_POSITION_AXIS_Y, 0.04f);
    radar_get_axis[1] = 0.0f;
    radar_get_angle = pi / 2.0f;
    enable_fix_pos = 1U;
    WheelPID_SetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Duties(0.04f, 0.04f, 0.04f, 0.04f);
}
```

- [ ] **Step 3: 写投影退化、限幅和输入校验失败测试**

新增测试分别验证：

```c
static void Test_Position_PID_Has_No_X_Authority_At_Right_Angle(void)
{
    const float pi = 3.14159265358979323846f;

    Reset_Fixture();
    WheelPID_SetTargetPosition(WHEEL_PID_POSITION_AXIS_X, 0.04f);
    radar_get_angle = pi / 2.0f;
    enable_fix_pos = 1U;
    WheelPID_SetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Duties(0.0f, 0.0f, 0.0f, 0.0f);
}

static void Test_Position_PID_Limits_Output(void)
{
    Reset_Fixture();
    WheelPID_SetTargetPosition(WHEEL_PID_POSITION_AXIS_X, 1.0f);
    enable_fix_pos = 1U;
    WheelPID_SetSpeeds(0.0f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Duties(0.1f, 0.1f, 0.1f, 0.1f);
}
```

输入校验测试先设置有效 X 目标 `0.04f`，再分别传入非法轴、`NAN` 目标；每次运行都仍应得到四轮 `0.04f`。另将当前 X 雷达位置或雷达角度设为 `NAN`，断言本周期四轮位置修正均为零。

- [ ] **Step 4: 写使能变化、轴切换和停车复位失败测试**

启动零轮速目标并运行一次后将 `pid_reset_count` 清零。依次把 `enable_fix_pos` 改为 `1U`、`0U`、`2U`，每次推进一个周期并断言计数恰好增加一次；保持 `2U` 再运行时计数不变且四轮无位置修正。

单独测试从有效 X 目标切换到 Y 目标时清零一次共享位置 PID；重复设置 Y 轴新目标时不因轴未变化而再次清零。调用 `WheelPID_Stop()` 后断言 `enable_fix_pos == 0U`，且位置 PID 历史已被清除。

- [ ] **Step 5: 编译并运行测试，确认 RED**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Wl,--wrap=PID_ResetHistory -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_position_pid_test.exe
```

预期：编译失败，报告 `enable_fix_pos`、`WheelPID_PositionAxis` 或 `WheelPID_SetTargetPosition()` 尚未定义。这证明测试确实要求新增公开行为。

### Task 2: 创建唯一位置 PID 和目标接口

**Files:**
- Modify: `Core/User/wheel_pid.h`
- Modify: `Core/User/wheel_pid.c`

- [ ] **Step 1: 在头文件声明轴类型、使能变量和目标接口**

在 `wheel_pid.h` 中加入中文 Doxygen：

```c
/** @brief 雷达位置保持使用的世界坐标轴。 */
typedef enum {
    WHEEL_PID_POSITION_AXIS_X = 0,
    WHEEL_PID_POSITION_AXIS_Y = 1
} WheelPID_PositionAxis;

/** @brief 位置保持使能，只有值为 1 时计算位置 PID。 */
extern uint8_t enable_fix_pos;

/**
 * @brief 设置单轴位置保持目标。
 * @param[in] axis 要保持的雷达世界坐标轴。
 * @param[in] target_position 目标位置，单位为米。
 * @return 无。
 */
void WheelPID_SetTargetPosition(
    WheelPID_PositionAxis axis,
    float target_position
);
```

- [ ] **Step 2: 定义单 PID 和位置保持状态**

在 `wheel_pid.c` 中加入 `<math.h>` 和雷达位置反馈声明，将当前未使用的位置 PID 数组收敛成单指针：

```c
#include <math.h>

extern volatile float radar_get_axis[2];

static PID *radar_pos_error_pid;
static float radar_target_position;
static WheelPID_PositionAxis radar_target_axis;
uint8_t enable_fix_pos;
static uint8_t enable_fix_pos_previous;
```

`WheelPID_Init()` 只创建一次位置 PID：

```c
radar_pos_error_pid = NULL;
PID_Create(&radar_pos_error_pid, 1.0f, 0.0f, 0.0f);
if (radar_pos_error_pid != NULL) {
    PID_SetOutputMax(radar_pos_error_pid, 0.1f);
}
radar_target_position = 0.0f;
radar_target_axis = WHEEL_PID_POSITION_AXIS_X;
enable_fix_pos = 0U;
enable_fix_pos_previous = 0U;
```

`ki` 为零，因此不调用除以 `ki` 的积分限幅设置。

- [ ] **Step 3: 实现带校验的显式目标接口**

```c
void WheelPID_SetTargetPosition(
    WheelPID_PositionAxis axis,
    float target_position
)
{
    if (((axis != WHEEL_PID_POSITION_AXIS_X) &&
         (axis != WHEEL_PID_POSITION_AXIS_Y)) ||
        !isfinite(target_position)) {
        return;
    }

    if ((axis != radar_target_axis) && (radar_pos_error_pid != NULL)) {
        PID_ResetHistory(radar_pos_error_pid);
    }
    radar_target_axis = axis;
    radar_target_position = target_position;
}
```

- [ ] **Step 4: 重新编译，确认接口错误消失但行为仍为 RED**

重复 Task 1 的 GCC 命令并运行：

```powershell
build\wheel_position_pid_test.exe
```

预期：编译成功，X/Y 位置修正输出相关断言失败。

### Task 3: 按严格使能语义计算并叠加位置修正

**Files:**
- Modify: `Core/User/wheel_pid.c`

- [ ] **Step 1: 处理使能原始值变化**

在 `PID_Task()` 的指针检查通过后、计算任何位置输出前加入：

```c
if (enable_fix_pos != enable_fix_pos_previous) {
    if (radar_pos_error_pid != NULL) {
        PID_ResetHistory(radar_pos_error_pid);
    }
    enable_fix_pos_previous = enable_fix_pos;
}
```

这里比较原始整数值，因此 `0U`、`1U`、`2U` 之间任何变化都会清除历史。

- [ ] **Step 2: 每周期最多计算一次位置 PID**

在四轮循环前初始化 `position_output`，并严格检查使能和反馈：

```c
float position_output = 0.0f;

if ((enable_fix_pos == 1U) &&
    (radar_pos_error_pid != NULL) &&
    isfinite(radar_target_position) &&
    isfinite(radar_get_axis[radar_target_axis]) &&
    isfinite(radar_get_angle)) {
    float axis_output = PID_Calc(
        radar_pos_error_pid,
        radar_target_position,
        radar_get_axis[radar_target_axis]
    );
    float projection = (radar_target_axis == WHEEL_PID_POSITION_AXIS_X)
        ? cosf(radar_get_angle)
        : sinf(radar_get_angle);
    position_output = axis_output * projection;
}
```

- [ ] **Step 3: 对四轮同号叠加位置修正**

在现有速度 PID、前馈和角度修正之后加入：

```c
if (enable_fix_pos == 1U) {
    result += position_output;
}
```

`position_output` 在无效反馈时为零，因此不会影响其他控制环。

- [ ] **Step 4: 停车时关闭位置保持并清历史**

在 `WheelPID_Stop()` 中加入：

```c
enable_fix_pos = 0U;
enable_fix_pos_previous = 0U;
if (radar_pos_error_pid != NULL) {
    PID_ResetHistory(radar_pos_error_pid);
}
```

这保证 PID 任务停止后仍满足“使能值修改后清除记录”的要求。

- [ ] **Step 5: 运行位置 PID 测试，确认 GREEN**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Wl,--wrap=PID_ResetHistory -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_position_pid_test.exe
build\wheel_position_pid_test.exe
```

预期：进程退出码为 `0`，编译器无警告。

### Task 4: 完整验证和交付检查

**Files:**
- Verify: `Core/User/wheel_pid.h`
- Verify: `Core/User/wheel_pid.c`
- Verify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 运行相关主机回归测试**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Wl,--wrap=PID_ResetHistory -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_position_pid_test.exe
build\wheel_position_pid_test.exe
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/servo_position_test.c User/servo_position.c -o build/servo_position_test.exe
build\servo_position_test.exe
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
build\state_machine_test.exe
```

预期：所有编译和测试进程退出码为 `0`，GCC 无警告。若发现与本功能无关的工作区既有失败，记录具体命令和失败断言，不修改无关源码。

- [ ] **Step 2: 构建 STM32 固件**

```powershell
cmake --build --preset Debug
```

预期：固件构建成功，不出现新增接口的编译或链接错误。

- [ ] **Step 3: 检查 UTF-8、Doxygen 和差异**

```powershell
@('Core/User/wheel_pid.h','Core/User/wheel_pid.c','tests/wheel_pid_test.c') | ForEach-Object {
    $bytes = [IO.File]::ReadAllBytes($_)
    $utf8 = New-Object Text.UTF8Encoding($false, $true)
    [void]$utf8.GetString($bytes)
}
Select-String -Path Core/User/wheel_pid.h,Core/User/wheel_pid.c,tests/wheel_pid_test.c -Pattern '@brief|@param|@return' -Encoding UTF8
git diff --check
```

预期：严格 UTF-8 解码成功，新增 C 接口和测试辅助函数使用中文 Doxygen，差异无空白错误。

- [ ] **Step 4: 复核只创建一个位置 PID**

```powershell
rg -n "radar_pos_error_pid|PID_Create" Core/User/wheel_pid.c
```

预期：`radar_pos_error_pid` 是单个 `PID *`，且初始化代码只对它调用一次 `PID_Create()`。
