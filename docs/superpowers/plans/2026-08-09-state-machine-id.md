# State Machine ID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace pointer-based `StateNode` state storage with a consistent `uint16_t` state ID API.

**Architecture:** The state machine owns only a numeric current-state value and a callback. `UINT16_MAX` represents no active state, so initialization, transitions, callbacks, and queries never require node storage or pointer lifetime management.

**Tech Stack:** C11, host GCC assertions, STM32 CMake/Ninja build

---

### Task 1: Specify ID-Based Behavior With a Host Test

**Files:**
- Create: `tests/state_machine_test.c`
- Test: `tests/state_machine_test.c`

- [x] **Step 1: Write a failing compile-and-runtime test**

Create a C11 test that statically requires `StateMachine.current_state_id`, records callback ID/event pairs, and asserts initialization, same-ID no-op, normal transition, transition to `STATE_MACHINE_NO_STATE`, null-machine change failure, and null-machine query behavior.

- [x] **Step 2: Run the test to verify RED**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
```

Expected: compilation fails because the old API exposes `StateNode *current_state` and has no ID-based members or sentinel.

### Task 2: Convert the State Machine Public API and Implementation

**Files:**
- Modify: `User/state_machine.h`
- Modify: `User/state_machine.c`
- Test: `tests/state_machine_test.c`

- [x] **Step 1: Replace node pointers with IDs in the public header**

Remove `StateNode`; define `STATE_MACHINE_NO_STATE` as `UINT16_MAX`; change the callback parameter to `uint16_t state_id`; store `uint16_t current_state_id`; and change init, transition, and query declarations to use `uint16_t`. Rewrite public documentation as UTF-8 Chinese Doxygen comments.

- [x] **Step 2: Implement sentinel-aware ID transitions**

Initialization enters only a non-sentinel initial ID. A transition exits a non-sentinel old ID, stores the new ID, and enters a non-sentinel new ID. Repeating the current ID returns success without callbacks. A null machine keeps the established failure/no-state behavior.

- [x] **Step 3: Run the host test to verify GREEN**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -IUser tests/state_machine_test.c User/state_machine.c -o build/state_machine_test.exe
build\state_machine_test.exe
```

Expected: compilation and execution both return exit code 0 with no warnings.

### Task 3: Update the Application Caller and Verify Firmware

**Files:**
- Modify: `Core/User/user.c`
- Verify: `User/state_machine.h`, `User/state_machine.c`, `Core/User/user.c`

- [x] **Step 1: Convert application state declarations and callback**

Replace `main_state_node` with a UTF-8 documented `uint16_t main_state_id[4]` initialized to IDs 0 through 3, and change `State_Change` to receive `uint16_t state_id`.

- [x] **Step 2: Confirm obsolete node references are gone**

Run:

```powershell
rg -n "StateNode|current_state\b" User Core/User tests
```

Expected: no matches.

- [x] **Step 3: Build the STM32 Debug target**

Run:

```powershell
cmake --build --preset Debug
```

Expected: Ninja completes successfully and links `build/Debug/26rcf.elf`.

- [x] **Step 4: Re-run the host regression test**

Run:

```powershell
build\state_machine_test.exe
```

Expected: exit code 0.

No commit step is included because this directory is not a Git repository.
