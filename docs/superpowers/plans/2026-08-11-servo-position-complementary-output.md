# ServoPosition Complementary Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 `ServoPosition` 增加显式普通/互补 PWM 输出选择，并把三路机械臂舵机范围修正为 0.5–2.5 ms。

**Architecture:** 在 `ServoPosition` 公共接口中增加输出类型枚举并保存到实例中，初始化和反初始化时分别分派到 HAL 普通 PWM 或互补 PWM API。位置换算继续共享现有 CCR 插值逻辑；调用方只负责声明各路输出类型和校准后的比较值。

**Tech Stack:** C11、STM32H7 HAL、GCC 主机测试、CMake/Ninja ARM 固件构建。

---

### Task 1: 添加 ServoPosition 主机回归测试

**Files:**
- Modify: `tests/stubs/main.h`
- Create: `tests/servo_position_test.c`
- Test: `tests/servo_position_test.c`

- [ ] **Step 1: 扩充主机 HAL 桩声明**

在 `tests/stubs/main.h` 中保留现有 GPIO/DWT 定义，并补充以下最小定时器接口：

```c
#include <stddef.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

typedef struct {
    volatile uint32_t ARR;
    volatile uint32_t CCR[4];
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U

void Test_HAL_TIM_SetCompare(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t compare
);

#define __HAL_TIM_GET_AUTORELOAD(__HANDLE__) ((__HANDLE__)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    Test_HAL_TIM_SetCompare((__HANDLE__), (__CHANNEL__), (__COMPARE__))

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t channel);
```

- [ ] **Step 2: 编写要求新 API 的失败测试**

创建 `tests/servo_position_test.c`，以真实 `ServoPosition_Init()`、`ServoPosition_DeInit()` 和 `ServoPosition_GetCompare()` 为测试对象。测试提供 HAL 函数实现和调用计数，核心断言如下：

```c
static void test_main_output_uses_main_hal_api(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_2,
               SERVO_POSITION_OUTPUT_MAIN,
               1375U,
               4125U,
               6875U
           ) == HAL_OK);
    assert(main_start_calls == 1U);
    assert(complementary_start_calls == 0U);
    assert(ServoPosition_DeInit(&servo) == HAL_OK);
    assert(main_stop_calls == 1U);
    assert(complementary_stop_calls == 0U);
}

static void test_complementary_output_uses_complementary_hal_api(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_1,
               SERVO_POSITION_OUTPUT_COMPLEMENTARY,
               1375U,
               4125U,
               6875U
           ) == HAL_OK);
    assert(main_start_calls == 0U);
    assert(complementary_start_calls == 1U);
    assert(ServoPosition_DeInit(&servo) == HAL_OK);
    assert(main_stop_calls == 0U);
    assert(complementary_stop_calls == 1U);
}

static void test_rejects_invalid_output_type(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_1,
               (ServoPositionOutput)99,
               1375U,
               4125U,
               6875U
           ) == HAL_ERROR);
    assert(main_start_calls == 0U);
    assert(complementary_start_calls == 0U);
}

static void test_half_to_two_and_half_millisecond_range(void)
{
    assert(ServoPosition_GetCompare(54999U, -1.0f, 1375U, 4125U, 6875U) == 1375U);
    assert(ServoPosition_GetCompare(54999U, 0.0f, 1375U, 4125U, 6875U) == 4125U);
    assert(ServoPosition_GetCompare(54999U, 1.0f, 1375U, 4125U, 6875U) == 6875U);
}
```

`main()` 依次调用上述四个测试。

- [ ] **Step 3: 编译测试并确认按预期失败**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/servo_position_test.c User/servo_position.c -o build/servo_position_test.exe
```

预期：编译失败，错误指出 `ServoPositionOutput`、`SERVO_POSITION_OUTPUT_MAIN` 或新 `ServoPosition_Init()` 参数尚不存在；失败原因必须是待实现功能缺失。

### Task 2: 实现普通与互补输出分派

**Files:**
- Modify: `User/servo_position.h`
- Modify: `User/servo_position.c`
- Test: `tests/servo_position_test.c`

- [ ] **Step 1: 添加公共输出类型和实例字段**

在 `User/servo_position.h` 中增加中文 Doxygen 注释的枚举，并在实例中保存类型：

```c
/**
 * @brief 位置舵机使用的 PWM 输出类型。
 */
typedef enum
{
    SERVO_POSITION_OUTPUT_MAIN = 0U,          /**< 普通 PWM 输出。 */
    SERVO_POSITION_OUTPUT_COMPLEMENTARY = 1U /**< 互补 PWM 输出。 */
} ServoPositionOutput;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    ServoPositionOutput output;
    uint32_t pwmPeriod;
    uint32_t minCompare;
    uint32_t centerCompare;
    uint32_t maxCompare;
    uint8_t initialized;
} ServoPosition;
```

将 `ServoPosition_Init()` 签名改为：

```c
HAL_StatusTypeDef ServoPosition_Init(
    ServoPosition *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    ServoPositionOutput output,
    uint32_t minCompare,
    uint32_t centerCompare,
    uint32_t maxCompare
);
```

同步补充 `@param[in] output` 中文 Doxygen 说明。

- [ ] **Step 2: 实现最小 HAL API 分派**

在 `User/servo_position.c` 中增加输出类型校验，并让清理函数将 `output` 恢复为普通输出：

```c
static uint8_t ServoPosition_IsValidOutput(ServoPositionOutput output)
{
    return (output == SERVO_POSITION_OUTPUT_MAIN) ||
           (output == SERVO_POSITION_OUTPUT_COMPLEMENTARY);
}
```

`ServoPosition_Init()` 在参数校验中拒绝非法类型，保存 `servo->output = output`，并按类型启动：

```c
const HAL_StatusTypeDef status =
    (output == SERVO_POSITION_OUTPUT_COMPLEMENTARY)
        ? HAL_TIMEx_PWMN_Start(htim, channel)
        : HAL_TIM_PWM_Start(htim, channel);
```

`ServoPosition_DeInit()` 在清理实例之前按已保存类型停止：

```c
const HAL_StatusTypeDef status =
    (servo->output == SERVO_POSITION_OUTPUT_COMPLEMENTARY)
        ? HAL_TIMEx_PWMN_Stop(servo->htim, servo->channel)
        : HAL_TIM_PWM_Stop(servo->htim, servo->channel);
```

- [ ] **Step 3: 重新编译并运行 ServoPosition 测试**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/servo_position_test.c User/servo_position.c -o build/servo_position_test.exe
build\servo_position_test.exe
```

预期：编译成功且测试进程退出码为 0。

- [ ] **Step 4: 提交接口和测试**

```powershell
git add tests/stubs/main.h tests/servo_position_test.c User/servo_position.h User/servo_position.c
git commit -m "fix: 支持舵机互补 PWM 输出"
```

### Task 3: 更新机械臂输出类型和脉宽参数

**Files:**
- Modify: `Core/User/behavior.c:81`
- Verify: `Core/Src/tim.c:172`

- [ ] **Step 1: 更新三路舵机初始化参数**

将 `Arm_Init()` 改为：

```c
void Arm_Init(void)
{
    ServoPosition_Init(
        &arm_servo[0],
        &htim8,
        TIM_CHANNEL_1,
        SERVO_POSITION_OUTPUT_COMPLEMENTARY,
        1375U,
        4125U,
        6875U
    );
    ServoPosition_Init(
        &arm_servo[1],
        &htim8,
        TIM_CHANNEL_2,
        SERVO_POSITION_OUTPUT_MAIN,
        1375U,
        4125U,
        6875U
    );
    ServoPosition_Init(
        &arm_servo[2],
        &htim8,
        TIM_CHANNEL_3,
        SERVO_POSITION_OUTPUT_MAIN,
        1375U,
        4125U,
        6875U
    );
}
```

- [ ] **Step 2: 运行所有主机测试**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/servo_position_test.c User/servo_position.c -o build/servo_position_test.exe
build\servo_position_test.exe
build\time_us_test.exe
build\state_machine_test.exe
build\encoder_motor_test.exe
```

预期：四个测试进程退出码均为 0。

- [ ] **Step 3: 构建 Debug 固件**

运行：

```powershell
cmake --build --preset Debug
```

预期：Ninja 构建成功，`26rcf.elf` 链接完成且命令退出码为 0。

- [ ] **Step 4: 检查差异和编码**

运行：

```powershell
git diff --check
git diff -- User/servo_position.h User/servo_position.c Core/User/behavior.c tests/stubs/main.h tests/servo_position_test.c
```

预期：`git diff --check` 无输出；代码注释为中文 Doxygen，文件保持 UTF-8。

- [ ] **Step 5: 提交调用方修改**

```powershell
git add Core/User/behavior.c
git commit -m "fix: 修正机械臂舵机脉宽范围"
```
