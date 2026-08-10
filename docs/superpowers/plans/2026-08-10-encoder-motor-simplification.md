# Encoder Motor Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除编码器模块中未使用的 MSPM0 中断兼容层，并在不改变已测试行为的前提下缩短编码器和电机模块。

**Architecture:** `Encoder` 直接持有两个 `GPIO_Pin`，静态表只保存最多四个 `Encoder *`；HAL EXTI 转发直接匹配 A/B 引脚并调用同一个判向函数。电机模块保持现有数据结构和公开函数，仅压缩初始化、局部变量和重复表达式，所有安全检查保留。

**Tech Stack:** C11、STM32H7 HAL、主机 GCC `assert` 测试、CMake/Ninja、PowerShell UTF-8 检查。

---

### Task 1: 用新接口建立失败测试

**Files:**
- Modify: `tests/encoder_motor_test.c`
- Test: `tests/encoder_motor_test.c`

- [x] **Step 1: 将中断编码器创建改为三参数 API**

把测试中的创建调用：

```c
Encoder_Create_UsePin(&encoders[i], pins_a[i], 0U, pins_b[i], 0U);
```

改为：

```c
Encoder_Create_UsePin(&encoders[i], pins_a[i], pins_b[i]);
```

第五个超容量实例也做相同修改。容量、计数和未修改对象断言保持原样。

- [x] **Step 2: 运行测试并确认 RED**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
```

预期：编译器报告 `Encoder_Create_UsePin` 参数数量不足，证明测试正在约束新 API。

### Task 2: 精简编码器公开接口和实现

**Files:**
- Modify: `User/encoder.h`
- Modify: `User/encoder.c`
- Test: `tests/encoder_motor_test.c`

- [x] **Step 1: 缩减 `encoder.h` 的类型和声明**

删除 `GPIO_Pin_IID`、`GPIO_ISR_Item`、`GPIO_Interrupt_Callback` 以及
`Encoder_Callback()`、`Encoder_Create_GPIO_ISR_Item()`、`Encoder_Create()` 声明。
把 `Encoder` 改为：

```c
typedef struct {
    GPIO_Pin pin_A;              /**< A 相 GPIO。 */
    GPIO_Pin pin_B;              /**< B 相 GPIO。 */
    volatile int32_t counter;    /**< 累计计数。 */
    int32_t last_get_counter;    /**< 上次读取计数。 */
    uint16_t prescaler;          /**< 预分频值。 */
    int32_t prescaler_counter;   /**< 尚未输出的余数。 */
} Encoder;
```

保留简洁中文 Doxygen，并把创建声明改为：

```c
void Encoder_Create_UsePin(Encoder *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B);
```

- [x] **Step 2: 删除中断项表并直接登记编码器**

`encoder.c` 只保留：

```c
static Encoder *encoder_table[ENCODER_MAX_COUNT];
static uint16_t encoder_count;
```

创建函数验证对象、两个引脚和容量后，统一初始化 `Encoder`，再把对象指针写入
`encoder_table`。不再分配两个中断项，也不保存 IID。

- [x] **Step 3: 直接完成 EXTI 匹配和判向**

把判向函数简化为接收编码器和触发相：

```c
static int8_t Encoder_GetDirection(const Encoder *encoder, uint8_t channel)
{
    uint8_t score = channel + Encoder_ReadPin(encoder->pin_A)
                            + Encoder_ReadPin(encoder->pin_B);
    return (score == 0U || score == 2U) ? -1 : 1;
}
```

EXTI 转发遍历已登记对象，匹配 `pin_A.pin` 时以通道 1 计数，匹配
`pin_B.pin` 时以通道 0 计数，并在匹配后返回。轮询状态表、每实例时间戳、
跨两状态补偿和预分频实现保持不变，只压缩重复初始化和局部变量。

- [x] **Step 4: 运行联合测试确认 GREEN**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
```

预期：无编译警告，测试退出码为 0。

### Task 3: 精简电机模块而不改变行为

**Files:**
- Modify: `User/motor.h`
- Modify: `User/motor.c`
- Test: `tests/encoder_motor_test.c`

- [x] **Step 1: 缩短头文件注释**

保留 `MOTOR_SPEED_TO_PULSE_COUNT`、两个回调类型、`Motor`、`Motor_Group` 和四个
公开函数。每个公开函数保留一段中文 Doxygen，参数含义在签名附近只说明一次；
结构字段保留单行 Doxygen，不删除字段。

- [x] **Step 2: 合并速度计算中的重复表达式**

保留现有空依赖、`k == 0` 和零时间差提前返回。有效计算先缓存：

```c
const float direction = motor->reverse ? -1.0f : 1.0f;
const float distance = (float)delta / (float)motor->k * motor->l * direction;
const float speed = distance * 1000000.0f / (float)delta_time;
```

再用 `distance` 更新路程、用 `speed` 更新历史和加速度。`Motor_Init()` 使用紧凑的
指定初始化器，三点中值和标定边界行为不变。

- [x] **Step 3: 重新运行主机测试**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -Itests/stubs -IUser tests/encoder_motor_test.c User/encoder.c User/motor.c -o build/encoder_motor_test.exe
build\encoder_motor_test.exe
```

预期：退出码为 0，速度、路程、加速度、中值和异常输入断言全部保持通过。

### Task 4: 工程级验证和精简审计

**Files:**
- Verify: `User/encoder.h`
- Verify: `User/encoder.c`
- Verify: `User/motor.h`
- Verify: `User/motor.c`
- Verify: `tests/encoder_motor_test.c`

- [x] **Step 1: 运行原有回归测试和 STM32 构建**

运行：

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
build\state_machine_test.exe
cmake --build --preset Debug
```

预期：两个主机测试退出码均为 0，Ninja 成功生成或确认最新的
`build/Debug/26rcf.elf`。

- [x] **Step 2: 检查删除的兼容符号不再存在**

运行：

```powershell
rg -n "GPIO_ISR_Item|GPIO_Pin_IID|GPIO_Interrupt_Callback|Encoder_Callback|Encoder_Create_GPIO_ISR_Item" User\encoder.h User\encoder.c
```

预期：`rg` 退出码为 1 且无匹配输出。

- [x] **Step 3: 检查 UTF-8 和文件体量**

对四个模块文件使用严格 `UTF8Encoding(false, true)` 解码并拒绝 BOM；随后运行：

```powershell
Get-Content User\encoder.h,User\encoder.c,User\motor.h,User\motor.c |
  Measure-Object -Line -Word -Character
```

预期：编码检查通过；总行数低于精简前的 467 行。行数只作为辅助指标，不能通过
删除测试、安全检查或必要 Doxygen 来达成。

- [x] **Step 4: 核对外部源文件并记录结果**

重新计算外部 MSPM0 四个源文件的 SHA-256，预期仍分别为：

```text
encoder.c 7A235F51E302D0C7B2AE0CBF9D9140E47EE043CDC5EEEF9FC220DE5750B0BAC8
encoder.h FEF920ACCB8A1150A1BD326CA2F98112D934CF916098C4DB02CC453C58B3B405
motor.c   6ECF04F5FA1EE10AD998574C2A6FF73A023CBF03C10791E635C43910044CB3FB
motor.h   8727B3D118AC34FEE142140E8D368FE3DFFBC9DDA2537ED36D590EA847B5488E
```

当前目录不是 Git 仓库，不执行提交、合并或分支清理。
