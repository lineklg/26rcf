# Encoder Motor Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 STM32H723 HAL 工程中新增与 MSPM0 版本主要接口和核心算法兼容的编码器、电机运动学模块，不修改外部源文件或当前工程的具体硬件绑定。

**Architecture:** `encoder` 模块提供不依赖具体引脚的 GPIO 端口+引脚抽象，内部静态管理中断项，并同时暴露 EXTI 转发入口和轮询式软件正交解码。`motor` 模块只依赖 `Encoder` 和调用方提供的微秒时基，保留原速度、路程、加速度和三点中值计算；HAL 依赖隔离在 `encoder.c` 中，主机测试用最小 HAL 桩替换。

**Tech Stack:** C11、STM32H7 HAL、CMake/Ninja、主机 GCC `assert` 测试、PowerShell UTF-8/哈希检查。

---

## 文件映射

- Create: `User/encoder.h`，编码器公开类型、宏、Doxygen 注释和兼容函数声明。
- Create: `User/encoder.c`，STM32 GPIO 读取、静态中断表、双相判向和轮询状态机。
- Create: `User/motor.h`，电机运动学类型、回调类型和兼容函数声明。
- Create: `User/motor.c`，速度、路程、加速度、中值滤波和标定表实现。
- Create: `tests/stubs/main.h`，主机测试所需的 `GPIO_TypeDef`、GPIO 状态和 HAL 函数声明。
- Create: `tests/encoder_motor_test.c`，编码器和电机的主机端行为测试。
- Verify only: `CMakeLists.txt`，确认已有 `User/*.c` 递归收集，无需修改。

不要修改：`D:\Workspace\microcontroller\TI\MSPM0G3507\06_competition_h\user\encoder.c`、`encoder.h`、`motor.c`、`motor.h`，以及当前工程的 `Core`、`fixedioc` 和 `User/DRV8870.*`。

### Task 1: 建立编码器主机测试并观察失败

**Files:**
- Create: `tests/stubs/main.h`
- Create: `tests/encoder_motor_test.c`

- [x] **Step 1: 写最小 HAL 桩和失败测试**

`tests/stubs/main.h` 提供不依赖 ARM 的测试接口：

```c
#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef struct {
    uint16_t levels;
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
uint32_t HAL_GetTick(void);

#endif
```

测试文件先声明 `HAL_GetTick()` 和 `HAL_GPIO_ReadPin()` 的可控实现，构造两个
端口对象，并写出以下行为断言：

```c
static uint32_t fake_tick;
static GPIO_TypeDef port_a;
static GPIO_TypeDef port_b;

uint32_t HAL_GetTick(void) { return fake_tick; }

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->levels & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
```

测试必须先覆盖 `Encoder_USI_Create()` 后的正向状态序列
`00 -> 10 -> 11 -> 01 -> 00`、反向序列、`00 -> 11` 的跨两状态补偿，以及两个
轮询实例在同一时间点都能独立更新。再覆盖 `Encoder_GetChange()` 的预分频余数。
编码器中断测试创建四个实例，调用 `Encoder_GPIO_EXTI_Callback()` 模拟 A/B 相变化，
并确认第五个实例不会越过 `ENCODER_MAX_COUNT`。

电机测试用公开的 `Encoder.counter` 注入增量，使用 `fake_time_us` 回调验证
速度、反向速度、路程、加速度、三点中值和六项标定表；同时写出空对象、`k == 0`、
零时间差及越界标定下标的断言。

- [x] **Step 2: 运行测试确认是预期的 RED**

运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
```

预期：编译失败，因为四个生产文件尚未创建，且测试引用的公开类型和函数不存在。
此时不要先写生产实现。

### Task 2: 实现编码器兼容层

**Files:**
- Create: `User/encoder.h`
- Create: `User/encoder.c`
- Test: `tests/encoder_motor_test.c`

- [x] **Step 1: 定义公开类型和函数签名**

在 `encoder.h` 中使用无 BOM UTF-8，并为每个公开宏、结构体字段组和函数写中文
Doxygen。核心定义如下，字段名保持原调用方式：

```c
#define ENCODER_MAX_COUNT 4U
#define ENCODER_USI_UPDATE_PERIOD 3U

typedef void *GPIO_Port;
typedef uint32_t GPIO_Pin_IID;
typedef struct { GPIO_Port port; uint16_t pin; } GPIO_Pin;

typedef struct gpio_isr_item GPIO_ISR_Item;
typedef void (*GPIO_Interrupt_Callback)(GPIO_ISR_Item *pin);
struct gpio_isr_item {
    GPIO_Pin pin;
    GPIO_Pin_IID iid;
    GPIO_Interrupt_Callback callback;
};

typedef struct {
    GPIO_ISR_Item *pin_A;
    GPIO_ISR_Item *pin_B;
    volatile int32_t counter;
    int32_t last_get_counter;
    uint16_t prescaler;
    int32_t prescaler_counter;
} Encoder;

typedef struct {
    Encoder base_enc;
    GPIO_Pin pin_A;
    GPIO_Pin pin_B;
    uint8_t last_AB;
    int8_t last_direction;
    uint32_t last_update_timestamp;
} Encoder_USI;
```

声明原有函数，并新增：

```c
void Encoder_GPIO_EXTI_Callback(uint16_t gpio_pin);
```

该入口只按 HAL 回调传入的引脚掩码分派，原有 `GPIO_Pin_IID` 参数只保存不使用。

- [x] **Step 2: 实现失败测试所需的 GPIO 读取和轮询解码**

在 `encoder.c` 仅包含 `main.h`、`encoder.h` 和标准整数头；使用
`HAL_GPIO_ReadPin((GPIO_TypeDef *)pin.port, pin.pin)` 读取电平。保留原四状态表和
跨两状态使用 `last_direction` 补偿的算法，但把更新时间戳放进每个
`Encoder_USI` 实例。`Encoder_USI_Create()` 初始化 `last_AB`、方向、计数器、
读取游标、预分频和时间戳；`Encoder_USI_Update()` 对空指针安全，并使用无符号
HAL tick 差值判断 3 ms 周期。

- [x] **Step 3: 实现静态中断表和增量读取**

使用最多 `ENCODER_MAX_COUNT * 2` 个静态 `GPIO_ISR_Item` 和
`ENCODER_MAX_COUNT` 个静态编码器指针。`Encoder_Create_GPIO_ISR_Item()` 在
目标指针为空、端口为空、引脚为零或静态表已满时返回并将目标置空；
`Encoder_Create_UsePin()` 预先检查两个表项和一个编码器槽位都可用，避免只创建一相。
`Encoder_Create()` 初始化对象并给两相绑定 `Encoder_Callback()`。

`Encoder_GPIO_EXTI_Callback()` 遍历已注册的表项，对匹配的引脚调用其回调。
`Encoder_Callback()` 遍历编码器表，找到 A 或 B 相后按原判向公式更新计数并立即返回。
`Encoder_GetChange()` 使用带符号快照、预分频余数和 `prescaler + 1`，对空指针安全。

- [x] **Step 4: 运行编码器测试确认 GREEN**

运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DENCODER_TEST_ONLY -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
```

预期：编译无警告，进程退出码为 0；失败时只修改编码器实现或测试桩，不修改断言来掩盖行为差异。

### Task 3: 实现电机运动学模块

**Files:**
- Create: `User/motor.h`
- Create: `User/motor.c`
- Test: `tests/encoder_motor_test.c`

- [x] **Step 1: 补充电机公开声明和中文 Doxygen**

`motor.h` 只包含 `stdint.h` 和 `encoder.h`，保留原结构字段和回调类型：

```c
typedef void (*SetSpeedCallback)(void *, float);
typedef uint64_t (*GetTimeCallback)(void);

typedef struct {
    Encoder *enc;
    uint64_t old_time;
    uint16_t k;
    float l;
    float speed_history[3];
    float route;
    float acceleration;
    GetTimeCallback time_callback;
    SetSpeedCallback speed_callback;
    void *device;
    uint8_t reverse;
    float speed_to_pulse_table[6];
} Motor;

typedef struct { Motor *left; Motor *right; } Motor_Group;
```

声明 `Motor_Init()`、`Motor_CalcSpeed()`、`Motor_CalcSpeed_Smooth()` 和
`Motor_RecordCurrentPulse()`，并写明时间单位、符号和无效输入行为。

- [x] **Step 2: 写最小实现并保持原计算顺序**

`Motor_Init()` 对空时基回调使用零初始时间；正常回调只调用一次。速度计算先
检查对象、编码器、回调和 `k`，再计算正的无符号时间差；零时间差返回 `0.0f`
且不移动编码器读取游标。有效计算时按以下顺序更新历史、速度、反向符号、
路程和加速度：

```c
delta = Encoder_GetChange(this->enc);
speed = ((float)delta * 1000000.0f / (float)delta_time)
      / (float)this->k * this->l;
if (this->reverse != 0U) speed = -speed;
this->route += ((float)delta / (float)this->k)
             * (this->reverse != 0U ? -1.0f : 1.0f) * this->l;
this->acceleration = (speed - previous_speed) * 1000000.0f
                   / (float)delta_time;
```

保留三点中值算法；`Motor_RecordCurrentPulse()` 对空对象和 `index >= 6U`
直接返回。

- [x] **Step 3: 运行联合主机测试确认 GREEN**

运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
```

预期：所有编码器与电机断言通过，退出码为 0，标准输出为空。

### Task 4: 进行 STM32 构建和编码审计

**Files:**
- Verify: `CMakeLists.txt`
- Verify: `User/encoder.h`
- Verify: `User/encoder.c`
- Verify: `User/motor.h`
- Verify: `User/motor.c`
- Verify: external MSPM0 source files

- [x] **Step 1: 确认 CMake 自动纳入新增源文件**

运行：

```powershell
rg -n "GLOB_RECURSE USER_SOURCES|User/\\*.c" CMakeLists.txt
```

预期能看到 `file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
"${CMAKE_SOURCE_DIR}/User/*.c")`，不修改 CMake。

- [x] **Step 2: 构建 STM32 Debug 目标**

运行：

```powershell
cmake --build --preset Debug
```

预期 Ninja 返回退出码 0 并生成 `build/Debug/26rcf.elf`；若失败，只处理新增
模块的编译兼容问题，不改动 CubeMX 生成代码来绕过错误。

- [x] **Step 3: 检查四个新增文件为严格无 BOM UTF-8**

运行：

```powershell
@('User/encoder.h','User/encoder.c','User/motor.h','User/motor.c') |
  ForEach-Object {
    $b=[IO.File]::ReadAllBytes($_)
    $u=New-Object Text.UTF8Encoding($false,$true)
    [void]$u.GetString($b)
    if ($b.Length -ge 3 -and $b[0] -eq 239 -and $b[1] -eq 187 -and $b[2] -eq 191) { throw "BOM: $_" }
  }
```

预期无输出、退出码为 0；同时用 `rg -n "^/\*\*" User/encoder.* User/motor.*`
抽查所有公开声明均有 Doxygen 块。

- [x] **Step 4: 确认 MSPM0 源文件未改变**

分别在移植前后运行：

```powershell
$source_files = @(
  'D:\Workspace\microcontroller\TI\MSPM0G3507\06_competition_h\user\encoder.c',
  'D:\Workspace\microcontroller\TI\MSPM0G3507\06_competition_h\user\encoder.h',
  'D:\Workspace\microcontroller\TI\MSPM0G3507\06_competition_h\user\motor.c',
  'D:\Workspace\microcontroller\TI\MSPM0G3507\06_competition_h\user\motor.h'
)
Get-FileHash -Algorithm SHA256 -LiteralPath $source_files
```

预期四个路径的哈希逐项相同，且当前工程只出现新增模块、测试、桩和计划文档。

- [x] **Step 5: 汇总验证结果**

记录主机测试、STM32 构建、UTF-8 审计和源文件哈希的实际退出码与关键输出；不
使用“应该通过”或未运行的验证作为完成依据。当前无 Git 元数据，不执行提交命令。
