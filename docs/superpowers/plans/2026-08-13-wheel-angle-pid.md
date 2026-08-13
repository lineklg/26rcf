# 车轮角度保持 PID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在四轮速度 PID 的 `result` 上按需叠加一个可限幅的角度保持 PID，使底盘稳定在预设雷达角度。

**Architecture:** `wheel_pid.c` 创建独立角度 PID，读取 `radar_get_angle` 和公开的 `enable_fix_angle`，将归一化的最短角误差转换为 `[-0.05, 0.05]` 的修正量。`PID_Task()` 先检测使能原始值变化并清除角度历史，再一次性计算角度修正；A/C 轮加修正，B/D 轮减修正。

**Tech Stack:** C11 主机测试、现有 `User/pid.c` 与 `User/task.c`、STM32 CMake/Ninja 固件工程。

---

### Task 1: 扩展测试夹具并写角度 PID 失败测试

**Files:**
- Modify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 增加雷达角度测试状态和历史清零观察器**

在测试文件中定义固件侧同名反馈：

```c
volatile float radar_get_angle;
static uint32_t angle_reset_count;

void __real_PID_ResetHistory(PID *pid);

void __wrap_PID_ResetHistory(PID *pid)
{
    angle_reset_count++;
    __real_PID_ResetHistory(pid);
}
```

测试继续使用真实 `pid.c`；链接器包装函数只记录清零调用并转发给真实实现。`Reset_Fixture()` 将 `radar_get_angle`、`enable_fix_angle` 和计数器清零。

- [ ] **Step 2: 写禁用、轮组方向和限幅测试**

新增三个独立测试：

1. 设置四轮零目标、目标角 `0.5f`、反馈 `0.0f`，保持 `enable_fix_angle = 0U`，断言四轮输出仍为零。
2. 启用后以目标角 `0.1f` 运行，断言 A/C 输出为 `+0.02f`，B/D 为 `-0.02f`；改为负误差后四轮符号反转。
3. 目标角 `1.0f` 时断言 A/C 为 `+0.05f`，B/D 为 `-0.05f`。

使用现有 `Assert_Close()` 和 `duty_output[]` 观察真实驱动边界。

- [ ] **Step 3: 写最短角误差测试**

目标角设为 `3.13f`，反馈设为 `-3.13f`，启用并运行。断言 A/C 修正为约 `-0.004637f`，B/D 为相反数，证明使用约 `-0.023185f` 的短误差而非接近整圈的误差。

- [ ] **Step 4: 写使能任意数值变化清历史测试**

在初始化完成后清零 `angle_reset_count`，依次令 `enable_fix_angle` 为 `1U`、`0U`、`2U`，每次推进一个 PID 周期。每次变化后断言计数恰好增加一次；值保持 `2U` 再运行一周期时计数不变，并断言 `2U` 不产生角度修正。

- [ ] **Step 5: 运行测试确认 RED**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Wl,--wrap=PID_ResetHistory -Itests/stubs -IUser -ICore/User tests/wheel_pid_test.c Core/User/wheel_pid.c User/pid.c User/task.c -lm -o build/wheel_angle_pid_test.exe
build\wheel_angle_pid_test.exe
```

预期：因 `WheelPID_SetTargetAngle()`、`enable_fix_angle` 和角度计算尚未实现而编译或断言失败。

### Task 2: 创建角度 PID 并提供目标接口

**Files:**
- Modify: `Core/User/wheel_pid.h`
- Modify: `Core/User/wheel_pid.c`

- [ ] **Step 1: 声明公开使能变量和目标接口**

在 `wheel_pid.h` 加入：

```c
/** @brief 角度保持使能，只有值为 1 时计算角度 PID。 */
extern uint8_t enable_fix_angle;

/**
 * @brief 设置角度保持目标值。
 * @param[in] target_angle 目标角度，单位为弧度，将归一化到 [-pi, pi]。
 * @return 无。
 */
void WheelPID_SetTargetAngle(float target_angle);
```

- [ ] **Step 2: 正式接入现有预留状态**

在 `wheel_pid.c` 中保留已有 `radar_error_pid` 和 `radar_target_angle`，删除未使用的 `radar_error_pid_enable`，新增：

```c
#include <math.h>

extern volatile float radar_get_angle;
uint8_t enable_fix_angle;
static uint8_t enable_fix_angle_previous;
```

`WheelPID_Init()` 中创建 `PID_Create(&radar_error_pid, 0.2f, 0.0f, 0.0f)`，成功后设置输出上限 `0.05f`；同时把目标角、当前使能和上次使能清零。

- [ ] **Step 3: 实现角度归一化和目标接口**

新增带中文 Doxygen 的私有函数：

```c
static float WheelPID_NormalizeAngle(float angle)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;

    while (angle > pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }
    return angle;
}

void WheelPID_SetTargetAngle(float target_angle)
{
    radar_target_angle = WheelPID_NormalizeAngle(target_angle);
}
```

- [ ] **Step 4: 编译确认接口错误消失但行为测试仍为 RED**

重复 Task 1 的编译运行命令。预期：编译成功，角度输出相关断言失败。

### Task 3: 计算并叠加角度修正

**Files:**
- Modify: `Core/User/wheel_pid.c`

- [ ] **Step 1: 在任务开头处理使能值变化**

在运行状态和指针检查通过后、车轮循环前加入：

```c
if (enable_fix_angle != enable_fix_angle_previous) {
    if (radar_error_pid != NULL) {
        PID_ResetHistory(radar_error_pid);
    }
    enable_fix_angle_previous = enable_fix_angle;
}
```

这使用原始值比较，保证 `0 -> 1`、`1 -> 0`、`0 -> 2` 等变化均清历史。

- [ ] **Step 2: 每个任务周期只计算一次角度修正**

在循环前初始化 `float angle_output = 0.0f;`。仅当 `enable_fix_angle == 1U` 且 PID 存在时：

```c
float angle_error = WheelPID_NormalizeAngle(
    radar_target_angle - radar_get_angle
);
angle_output = PID_Calc(radar_error_pid, angle_error, 0.0f);
```

把已归一化误差作为通用 PID 的设定值、零作为反馈值，以保留 PID 自身历史。

- [ ] **Step 3: 按轮组符号叠加到 `result`**

```c
float result = output;
result += WheelPID_FeedForward(wheel_target[i]);
if (enable_fix_angle == 1U) {
    result += ((i == 0U) || (i == 2U)) ? angle_output : -angle_output;
}
DRV8870_SetDutyPercent(&wheel_driver[i], result);
```

不改变现有 VOFA 通道协议，也不增加最终占空比限幅。

- [ ] **Step 4: 运行角度测试确认 GREEN**

重复 Task 1 的编译与运行命令。预期：所有 wheel PID 测试退出码为 `0` 且无警告。

### Task 4: 完整验证

**Files:**
- Verify: `Core/User/wheel_pid.h`
- Verify: `Core/User/wheel_pid.c`
- Verify: `tests/wheel_pid_test.c`

- [ ] **Step 1: 运行其余主机测试**

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

预期：所有测试进程退出码为 `0`。若旧轮速数值断言因当前工作区已有参数变化失败，单独记录，不修改无关生产参数。

- [ ] **Step 2: 构建 STM32 固件**

```powershell
cmake --build --preset Debug
```

预期：构建成功，无新接口的编译或链接错误。

- [ ] **Step 3: 检查编码、注释和差异**

```powershell
@('Core/User/wheel_pid.h','Core/User/wheel_pid.c','tests/wheel_pid_test.c') | ForEach-Object {
    $bytes = [IO.File]::ReadAllBytes($_)
    $utf8 = New-Object Text.UTF8Encoding($false, $true)
    [void]$utf8.GetString($bytes)
}
Select-String -Path Core/User/wheel_pid.h,Core/User/wheel_pid.c -Pattern '@brief|@param|@return' -Encoding UTF8
git diff --check
```

预期：严格 UTF-8 解码成功，新增 C 接口使用中文 Doxygen，无空白错误。

- [ ] **Step 4: 提交实现和测试**

```powershell
git add Core/User/wheel_pid.h Core/User/wheel_pid.c tests/wheel_pid_test.c
git commit -m "feat: 增加车轮角度保持PID"
```

