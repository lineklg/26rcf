# A 区避障状态机 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the A 区 obstacle-avoidance state machine with the two requested paths, fixed transition delays, radar-based actions, and distance-derived completion timing.

**Architecture:** Add a small hardware-independent logic module for path progression and distance-to-delay conversion, plus a focused callback module for motor, radar, task, and state-machine integration. Keep `Core/User/user.c` responsible only for state-machine storage, initialization, and emergency shutdown. Add hosted C tests that verify the real path logic and callback while recording stubbed actions and scheduling calls.

**Tech Stack:** C11, STM32 HAL/CMake firmware, hosted GCC test harness, existing `StateMachine`, `Task`, and `behavior` APIs.

---

### Task 1: Add the failing hosted callback test

**Files:**
- Create: `tests/a_obstacle_avoidance_state_machine_test.c`
- Modify: `Core/User/user.h`

- [x] **Step 1: Add test doubles and assertions for the callback contract**

The test defines `HAL_GetTick`, task globals, `machine_delay`, `next_state_id_delay`, and stubs for wheel and task functions referenced by `user.c`. It records the last wheel action, signed route or angle, next state, and scheduled delay. Call `A_Obstacle_Avoidance_State_Change(state, STATE_ENTER)` directly and assert this sequence:

```c
Enter(1U, ACTION_TURN, -40.0f, 3U, 1200U);
Enter(3U, ACTION_CURRENT_ANGLE, 0.7f, 0U, 2500U);
Enter(0U, ACTION_TURN, 40.0f, 5U, 1200U);
Enter(5U, ACTION_AXIS_Y, -radar_get_axis[1], 6U, 3000U);
Enter(6U, ACTION_TURN, -radar_get_angle, 7U, 1200U);
Enter(7U, ACTION_AXIS_X, oa_current_set_target_x - radar_get_axis[0],
      STATE_MACHINE_NO_STATE, 2000U);
```

Repeat from state 0 with a fresh fixture and assert `0 -> 2 -> 1 -> 5 -> 6 -> 7`. Use a state 7 distance of `0.6 m`, which must produce `2000 ms` at `0.3 m/s`. Assert exit events cause no wheel or scheduling calls.

- [x] **Step 2: Expose the callback declaration**

Add this declaration beside the existing state callbacks in `Core/User/user.h`:

```c
void A_Obstacle_Avoidance_State_Change(uint16_t state_id, uint8_t enter_or_exit);
```

- [x] **Step 3: Run the focused test and verify the expected failure**

Run:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -IUser -ICore/User tests/a_obstacle_avoidance_state_machine_test.c Core/User/a_obstacle_avoidance.c -lm -o build/a_obstacle_avoidance_state_machine_test.exe
```

Expected: compilation or link failure because `A_Obstacle_Avoidance_State_Change` is declared but not implemented.

### Task 2: Implement hardware-independent path and delay logic

**Files:**
- Create: `Core/User/a_obstacle_avoidance_logic.h`
- Create: `Core/User/a_obstacle_avoidance_logic.c`
- Create: `tests/a_obstacle_avoidance_logic_test.c`

- [x] **Step 1: Add a failing pure logic test**

Assert the exact paths `1,3,0,5,6,7,NO_STATE` and `0,2,1,5,6,7,NO_STATE`, null flag handling, invalid states, `0.6 m / 0.3 m/s == 2000`, zero and negative distance, and invalid speed or distance.

- [x] **Step 2: Verify the logic test fails**

```powershell
gcc -std=c11 -pedantic -Wall -Wextra -Werror -IUser -ICore/User tests/a_obstacle_avoidance_logic_test.c Core/User/a_obstacle_avoidance_logic.c -lm -o build/a_obstacle_avoidance_logic_test.exe
```

Expected: compilation fails because the logic header and source do not exist.

- [x] **Step 3: Add the minimal logic API**

Declare and implement:

```c
uint16_t AObstacleAvoidance_NextState(uint16_t state_id,
                                      uint8_t *detour_started);
uint32_t AObstacleAvoidance_DistanceDelayMs(float distance_m,
                                            float speed_mps);
```

`detour_started` is initially zero. The first state 0 or 1 selects forward state 2 or 3 and sets it to one; the second turn selects state 5. Fixed transitions are `2 -> 1`, `3 -> 0`, `5 -> 6`, `6 -> 7`, and `7 -> STATE_MACHINE_NO_STATE`. Invalid state IDs return `STATE_MACHINE_NO_STATE` without dereferencing a null flag.

For finite positive speed and finite distance, calculate `ceilf(fabsf(distance_m) / speed_mps * 1000.0f)`, convert to `uint32_t`, and saturate at the scheduler limit `UINT32_MAX / 2`. Return zero for non-finite inputs or non-positive speed.

- [x] **Step 4: Run the logic test and verify it passes**

Run the Step 2 command and then:

```powershell
build/a_obstacle_avoidance_logic_test.exe
```

Expected: exit code 0.

### Task 3: Implement the state callback and initialization integration

**Files:**
- Create: `Core/User/a_obstacle_avoidance.c`
- Modify: `Core/User/user.c`
- Modify: `Core/User/user.h`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Add obstacle state-machine storage and initialization**

Initialize the state machine in `User_Init` with `STATE_MACHINE_NO_STATE` and `A_Obstacle_Avoidance_State_Change`, and clear it in `User_EmergencyStop`.

- [x] **Step 2: Implement the enter-state actions**

Create `Core/User/a_obstacle_avoidance.c`, include `user.h` and `a_obstacle_avoidance_logic.h`, add a file-scope `uint8_t detour_started`, and add the callback with no comments. On `STATE_ENTER`, implement:

```c
case 0:
    Wheel_Turn_WithRadar_Angle(0.3f, 40.0f * (float)M_PI / 180.0f);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine,
                           AObstacleAvoidance_NextState(0U, &detour_started), 1200U);
    break;
case 1:
    Wheel_Turn_WithRadar_Angle(0.3f, -40.0f * (float)M_PI / 180.0f);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine,
                           AObstacleAvoidance_NextState(1U, &detour_started), 1200U);
    break;
case 2:
case 3:
    Wheel_Forward_WithRadar_CurrentAngle(0.3f, 0.7f);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine,
                           AObstacleAvoidance_NextState(state_id, &detour_started), 2500U);
    break;
case 5:
    Wheel_Forward_WithRadar_AxisY(0.3f, -radar_get_axis[1]);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine, 6U, 3000U);
    break;
case 6:
    Wheel_Turn_WithRadar_Angle(0.3f, -radar_get_angle);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine, 7U, 1200U);
    break;
case 7:
{
    float x_delta = oa_current_set_target_x - radar_get_axis[0];
    Wheel_Forward_WithRadar_AxisX(0.3f, x_delta);
    State_Change_WithDelay(&A_obstacle_avoidance_state_machine,
                           STATE_MACHINE_NO_STATE,
                           AObstacleAvoidance_DistanceDelayMs(x_delta, 0.3f));
    detour_started = 0U;
    break;
}
```

Ignore exit events. State 0 and state 1 use the logic module so initial entry selects the detour and the second turn goes to state 5.

- [x] **Step 3: Add the logic source to the firmware target**

Add `${CMAKE_SOURCE_DIR}/Core/User/a_obstacle_avoidance_logic.c` and `${CMAKE_SOURCE_DIR}/Core/User/a_obstacle_avoidance.c` to `target_sources` in `CMakeLists.txt`.

- [x] **Step 4: Run the callback test and verify it passes**

Re-run the Task 1 GCC command with `Core/User/a_obstacle_avoidance_logic.c` in the source list, then execute:

```powershell
build/a_obstacle_avoidance_state_machine_test.exe
```

Expected: exit code 0 and both recorded action sequences match the specification.

### Task 4: Verify firmware build and repository hygiene

**Files:**
- Verify: `Core/User/user.c`
- Verify: `Core/User/user.h`
- Verify: `Core/User/a_obstacle_avoidance.c`
- Verify: `Core/User/a_obstacle_avoidance_logic.h`
- Verify: `Core/User/a_obstacle_avoidance_logic.c`
- Verify: `CMakeLists.txt`
- Verify: `tests/a_obstacle_avoidance_state_machine_test.c`

- [x] **Step 1: Build the existing firmware target**

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Expected: build succeeds without new warnings.

- [x] **Step 2: Run the focused and related tests**

Run the two new test executables plus the existing standalone `state_machine_test`, `area_b_logic_test`, and `behavior_axisx_test` commands documented under `docs/superpowers/plans`.

- [x] **Step 3: Inspect the diff**

```powershell
git diff --check
git status --short
```

Confirm only the requested state-machine files, focused tests, and CMake source registration changed; preserve all pre-existing user edits.

- [x] **Step 4: Commit the implementation**

```powershell
git add Core/User/user.c Core/User/user.h Core/User/a_obstacle_avoidance.c Core/User/a_obstacle_avoidance_logic.h Core/User/a_obstacle_avoidance_logic.c tests/a_obstacle_avoidance_logic_test.c tests/a_obstacle_avoidance_state_machine_test.c CMakeLists.txt
git commit -m "feat: add A obstacle avoidance state machine"
```
