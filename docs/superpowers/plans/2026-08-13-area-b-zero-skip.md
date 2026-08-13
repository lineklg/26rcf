# B 区零干旱位置跳过 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** B 区车辆到达干旱程度为 `0` 的位置时，跳过机械臂转向、语音和喷水，同时正确推进到下一位置。

**Architecture:** 新增一个无外设依赖的 B 区位置决策模块，根据当前位置和干旱程度返回是否灌溉、推进后的位置和下一状态。`user.c` 使用该结果组织现有状态机动作，并统一正常灌溉与零值跳过的位置推进规则。

**Tech Stack:** C11、STM32 HAL、GCC 主机测试、CMake/Ninja 固件构建。

---

## 文件结构

- Create: `Core/User/area_b_logic.h`，定义 B 区位置决策结果和接口，不添加注释。
- Create: `Core/User/area_b_logic.c`，实现无外设依赖的位置决策，不添加注释。
- Create: `tests/area_b_logic_test.c`，覆盖零值跳过、连续零值和非零灌溉决策，不添加注释。
- Modify: `CMakeLists.txt`，将新模块加入固件目标。
- Modify: `Core/User/user.c`，接入决策结果并修正 B 区索引误用，不添加注释。

### Task 1: 建立 B 区位置决策的失败测试

**Files:**
- Create: `tests/area_b_logic_test.c`
- Test: `tests/area_b_logic_test.c`

- [ ] **Step 1: 创建失败测试**

创建 `tests/area_b_logic_test.c`：

```c
#include "area_b_logic.h"

#include <assert.h>

static void TestZeroEvenPositionSkipsToRight(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(0U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 1U);
    assert(decision.next_state == 3U);
}

static void TestZeroOddPositionSkipsToCenter(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(1U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 2U);
    assert(decision.next_state == 6U);
}

static void TestNonzeroPositionRequiresIrrigation(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(2U, 3U);

    assert(decision.should_irrigate == 1U);
    assert(decision.next_position == 3U);
    assert(decision.next_state == 3U);
}

static void TestLastPositionAdvancesPastArea(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(5U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 6U);
    assert(decision.next_state == 6U);
}

int main(void)
{
    TestZeroEvenPositionSkipsToRight();
    TestZeroOddPositionSkipsToCenter();
    TestNonzeroPositionRequiresIrrigation();
    TestLastPositionAdvancesPastArea();
    return 0;
}
```

- [ ] **Step 2: 编译测试并确认因接口缺失而失败**

Run:

```powershell
New-Item -ItemType Directory -Force build | Out-Null
gcc -std=c11 -pedantic -Wall -Wextra -Werror -ICore/User tests/area_b_logic_test.c -o build/area_b_logic_test.exe
```

Expected: FAIL，错误指出 `area_b_logic.h` 不存在。

### Task 2: 实现最小位置决策模块

**Files:**
- Create: `Core/User/area_b_logic.h`
- Create: `Core/User/area_b_logic.c`
- Modify: `CMakeLists.txt`
- Test: `tests/area_b_logic_test.c`

- [ ] **Step 1: 创建无注释公共定义**

创建 `Core/User/area_b_logic.h`：

```c
#ifndef AREA_B_LOGIC_H
#define AREA_B_LOGIC_H

#include <stdint.h>

typedef struct
{
    uint8_t should_irrigate;
    uint8_t next_position;
    uint16_t next_state;
} AreaBPositionDecision;

AreaBPositionDecision AreaB_DecidePosition(uint8_t current_position,
                                           uint8_t situation);

#endif
```

- [ ] **Step 2: 创建最小实现**

创建 `Core/User/area_b_logic.c`：

```c
#include "area_b_logic.h"

AreaBPositionDecision AreaB_DecidePosition(uint8_t current_position,
                                           uint8_t situation)
{
    AreaBPositionDecision decision;

    decision.should_irrigate = situation != 0U;
    decision.next_position = current_position + 1U;
    decision.next_state = current_position % 2U == 0U ? 3U : 6U;
    return decision;
}
```

- [ ] **Step 3: 将模块加入固件构建**

在 `CMakeLists.txt` 的 `target_sources` 中加入：

```cmake
    ${CMAKE_SOURCE_DIR}/Core/User/area_b_logic.c
```

- [ ] **Step 4: 编译并运行测试确认通过**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -ICore/User tests/area_b_logic_test.c Core/User/area_b_logic.c -o build/area_b_logic_test.exe
./build/area_b_logic_test.exe
```

Expected: 两条命令退出码均为 0，测试程序无输出。

### Task 3: 将零值决策接入 B 区状态机

**Files:**
- Modify: `Core/User/user.c:1-218`
- Test: `tests/area_b_logic_test.c`

- [ ] **Step 1: 引入决策模块并添加无注释内部推进函数**

在 `user.c` 引入头文件：

```c
#include "area_b_logic.h"
```

在 `Area_B_State_Change()` 之前加入：

```c
static void Area_B_Advance_Position(const AreaBPositionDecision *decision,
                                    uint32_t delay)
{
    area_B_current_position = decision->next_position;
    if (delay == 0U)
    {
        StateMachine_Change(&area_B_state_machine, decision->next_state);
    }
    else
    {
        State_Change_WithDelay(&area_B_state_machine,
                               decision->next_state,
                               delay);
    }
}
```

- [ ] **Step 2: 在状态 2 和状态 3 跳过零值位置**

将 B 区状态 `2` 和 `3` 改为：

```c
        case 2:
        case 3:
        {
            AreaBPositionDecision decision = AreaB_DecidePosition(
                area_B_current_position,
                area_B_situation[area_B_current_position]
            );
            if (!decision.should_irrigate)
            {
                Area_B_Advance_Position(&decision, 0U);
                break;
            }
            Arm_RoughAdjustment(state_id == 2U ? 1U : 2U);
            State_Change_WithDelay(&area_B_state_machine, 4, 200);
            break;
        }
```

- [ ] **Step 3: 在喷水后复用相同推进规则**

将 B 区状态 `5` 改为：

```c
        case 5:
        {
            uint8_t situation = area_B_situation[area_B_current_position];
            AreaBPositionDecision decision = AreaB_DecidePosition(
                area_B_current_position,
                situation
            );
            pump_spray_time = situation;
            Task_SetRunTick_Current(task_pump_spray);
            Task_Awake(task_pump_spray);
            Area_B_Advance_Position(&decision, 1000U + situation * 600U);
            break;
        }
```

该实现同时消除对 `area_A_current_position` 的错误引用。

- [ ] **Step 4: 运行主机测试**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -ICore/User tests/area_b_logic_test.c Core/User/area_b_logic.c -o build/area_b_logic_test.exe
./build/area_b_logic_test.exe
```

Expected: 两条命令退出码均为 0，测试程序无输出。

### Task 4: 完整验证

**Files:**
- Verify: `Core/User/area_b_logic.h`
- Verify: `Core/User/area_b_logic.c`
- Verify: `Core/User/user.c`
- Verify: `CMakeLists.txt`
- Verify: `tests/area_b_logic_test.c`

- [ ] **Step 1: 运行现有主机测试和新增测试**

Run:

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
./build/state_machine_test.exe
gcc -std=c11 -pedantic -Wall -Wextra -Werror -ICore/User tests/area_b_logic_test.c Core/User/area_b_logic.c -o build/area_b_logic_test.exe
./build/area_b_logic_test.exe
```

Expected: 所有命令退出码均为 0，两个测试程序无输出。

- [ ] **Step 2: 构建 STM32 固件**

Run:

```powershell
cmake --build --preset Debug
```

Expected: 构建退出码为 0。

- [ ] **Step 3: 检查无新增源码注释和差异范围**

Run:

```powershell
git diff --check -- CMakeLists.txt Core/User/user.c Core/User/area_b_logic.h Core/User/area_b_logic.c tests/area_b_logic_test.c
git diff -- CMakeLists.txt Core/User/user.c Core/User/area_b_logic.h Core/User/area_b_logic.c tests/area_b_logic_test.c
```

Expected: 无空白错误；差异仅包含决策模块、测试、构建接入和 B 区状态机修改，新增源码不含注释。
