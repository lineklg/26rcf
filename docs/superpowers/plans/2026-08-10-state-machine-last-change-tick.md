# State Machine Last-Change Tick Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Store and expose the HAL tick at which the state machine was initialized or most recently changed state.

**Architecture:** Extend `StateMachine` with a `uint32_t last_state_change_tick` field. Add `StateMachine_GetTick(void)` as the only wrapper that calls the firmware-provided `HAL_GetTick()`, and use it on initialization and real transitions. Add `StateMachine_GetLastChangeTick()` for null-safe read access while preserving current state IDs, callback ordering, same-state no-op behavior, and null-machine return values.

**Tech Stack:** C11, host GCC assertion test, STM32 HAL, CMake/Ninja Debug build.

---

### Task 1: Add failing host coverage for timestamp behavior

**Files:**
- Modify: `tests/state_machine_test.c`
- Test: `tests/state_machine_test.c`

- [x] **Step 1: Add a deterministic HAL tick source and timestamp assertions**

Add a test-controlled tick variable and this exact symbol before `main`:

```c
static uint32_t fake_tick;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}
```

Extend `main` so it verifies the requested behavior:

```c
    _Static_assert(
        sizeof(machine.last_state_change_tick) == sizeof(uint32_t),
        "last change tick must be stored as uint32_t"
    );

    fake_tick = 100U;
    StateMachine_Init(&machine, 1U, RecordStateChange);
    assert(StateMachine_GetLastChangeTick(&machine) == 100U);

    fake_tick = 200U;
    assert(StateMachine_GetTick() == 200U);
    assert(StateMachine_Change(&machine, 1U) == 1U);
    assert(StateMachine_GetLastChangeTick(&machine) == 100U);

    fake_tick = 250U;
    assert(StateMachine_Change(&machine, 2U) == 1U);
    assert(StateMachine_GetLastChangeTick(&machine) == 250U);

    fake_tick = 400U;
    assert(StateMachine_Change(&machine, STATE_MACHINE_NO_STATE) == 1U);
    assert(StateMachine_GetLastChangeTick(&machine) == 400U);
    assert(StateMachine_GetLastChangeTick(NULL) == 0U);
```

Keep the existing callback and current-state assertions; adjust the current
initialization block to use the single `StateMachine_Init()` call shown above.

- [x] **Step 2: Run the focused host test and verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
```

Expected result: compilation fails because `StateMachine` has no
`last_state_change_tick` member and the new timestamp functions are not yet
declared or defined. Do not change production code before observing this
failure.

### Task 2: Implement the timestamp API and transition updates

**Files:**
- Modify: `User/state_machine.h`
- Modify: `User/state_machine.c`
- Test: `tests/state_machine_test.c`

- [x] **Step 1: Declare the field and functions in the public header**

Include `<stdint.h>` as already done, add the field to `StateMachine`, and add
these declarations and concise Doxygen comments:

```c
uint32_t StateMachine_GetTick(void);
uint32_t StateMachine_GetLastChangeTick(const StateMachine *machine);
```

Document that the first function returns the current HAL tick and the second
returns the tick stored for the last initialization or real state transition,
with `0U` for a null machine.

- [x] **Step 2: Implement the HAL wrapper and null-safe getter**

At the top of `User/state_machine.c`, keep the public module hardware-neutral
and declare the firmware symbol locally:

```c
extern uint32_t HAL_GetTick(void);

uint32_t StateMachine_GetTick(void)
{
    return HAL_GetTick();
}
```

Implement `StateMachine_GetLastChangeTick()` to return `0U` for `NULL` and the
stored field otherwise.

- [x] **Step 3: Record ticks at initialization and real transitions**

In `StateMachine_Init()`, after assigning `current_state_id` and `callback`,
assign `last_state_change_tick = StateMachine_GetTick()` before the optional
initial enter callback.

In `StateMachine_Change()`, preserve the existing null and same-state early
returns. After the old-state exit callback and assignment of
`current_state_id = next_state_id`, assign
`last_state_change_tick = StateMachine_GetTick()` before the optional new-state
enter callback.

- [x] **Step 4: Run the focused host test and verify GREEN**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
build\state_machine_test.exe
```

Expected result: compilation succeeds with no warnings and the executable exits
with code 0.

### Task 3: Verify the firmware target and regression behavior

**Files:**
- Verify: `User/state_machine.h`
- Verify: `User/state_machine.c`
- Verify: `Core/User/user.c`
- Verify: `tests/state_machine_test.c`

- [x] **Step 1: Build the STM32 Debug target**

Run:

```powershell
cmake --build --preset Debug
```

Expected result: Ninja completes successfully and links
`build/Debug/26rcf.elf`; the existing `HAL_GetTick()` implementation satisfies
the state-machine wrapper's external symbol.

- [x] **Step 2: Re-run the host regression test**

Run:

```powershell
build\state_machine_test.exe
```

Expected result: exit code 0 with no output or assertion failures.

- [x] **Step 3: Review the final changed files**

Read `User/state_machine.h`, `User/state_machine.c`, and
`tests/state_machine_test.c` and confirm the public signatures, field name, test
stub, and transition ordering match this plan. This workspace currently has no
Git repository, so record the modified file paths and verification results in
the handoff instead of attempting a commit.
