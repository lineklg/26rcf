# Wheel Forward Current Angle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a radar-aware straight-drive function that travels a signed distance along the heading captured at command time, including angle hold and projected-position stopping.

**Architecture:** Keep the public behavior API in `Core/User/behavior.h` and implementation in `Core/User/behavior.c`. Store the command's initial position and heading in behavior-module state; a task condition computes displacement projected onto the captured heading and stops on tolerance or target overshoot. Reuse `Wheel_Forward`, `Wheel_Stop`, and the existing angle-hold mechanism.

**Tech Stack:** C11-compatible embedded C, STM32 HAL, existing task scheduler, hosted C test harness compiled by CMake/PowerShell.

---

### Task 1: Add failing behavior tests

**Files:**
- Modify: `tests/behavior_axisx_test.c`

- [ ] **Step 1: Add the test fixture observation and behavior cases**

Declare the new function through `behavior.h` and add tests that set `radar_get_angle`, call `Wheel_Forward_WithRadar_CurrentAngle`, advance `Task_Update`, then update radar coordinates. Cover angle `0`, angle `pi / 2`, angle `pi / 4` with lateral-only movement, and invalid angle input.

- [ ] **Step 2: Run the focused test to verify it fails**

Run: `cmake --preset test; cmake --build --preset test --target behavior_axisx_test; .\build\test\behavior_axisx_test.exe`

Expected: compilation fails because `Wheel_Forward_WithRadar_CurrentAngle` is not declared/defined.

### Task 2: Implement projected current-angle movement

**Files:**
- Modify: `Core/User/behavior.h`
- Modify: `Core/User/behavior.c`

- [ ] **Step 1: Add command state and condition callback**

Store initial X/Y, captured angle, projected target, and a mode flag. The callback returns stop when either radar coordinate is non-finite, projected error is within `WHEEL_TARGET_AXIS_ERROR`, or error has crossed the signed route direction.

- [ ] **Step 2: Add the public function**

Validate finite speed, route, coordinates, angle, nonzero speed, nontrivial route, and task availability. Capture state, call `Wheel_Forward(speed)`, set `WheelPID_SetTargetAngle(captured_angle)`, enable angle hold, configure the condition task, and wake it. Invalid commands call `Wheel_Stop`.

- [ ] **Step 3: Add Chinese Doxygen comments**

Document the function in `behavior.h` with `@brief`, `@param[in]`, and `@return`, using UTF-8 source encoding.

### Task 3: Verify and refactor

**Files:**
- Modify: `tests/behavior_axisx_test.c` only if assertions need correction.

- [ ] **Step 1: Run the focused behavior test**

Run: `cmake --build --preset test --target behavior_axisx_test; .\build\test\behavior_axisx_test.exe`

Expected: exit code 0.

- [ ] **Step 2: Run the complete test suite**

Run: `ctest --preset test --output-on-failure`

Expected: all registered tests pass.

- [ ] **Step 3: Inspect the final diff**

Run: `git diff --check; git status --short`

Expected: no whitespace errors; unrelated pre-existing user changes remain untouched.
