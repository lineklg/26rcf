# Wheel Forward Radar AxisX Improvement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `Wheel_Forward_WithRadar_AxisX` stop correctly when the signed radar displacement and wheel speed have different directions.

**Architecture:** Treat `route_m` as a signed displacement along radar X. Capture the target coordinate and displacement direction once, then stop when the coordinate enters the tolerance window or crosses the target. Reject non-finite inputs, invalid radar readings, zero displacement, zero speed, and unavailable stop tasks before starting motion.

**Tech Stack:** C11, existing `Task` scheduler, STM32 behavior layer, host-side GCC regression tests.

---

### Task 1: Add regression coverage

**Files:**
- Create: `tests/behavior_axisx_test.c`

- [ ] Write a host test harness with stubs for motor, radar, task, HAL, and behavior dependencies.
- [ ] Add a failing test showing `speed < 0` with `route_m > 0` stops after radar X reaches the positive target.
- [ ] Add tests for negative displacement, zero displacement, non-finite arguments, and non-finite radar input.
- [ ] Run the test and confirm it fails against the current implementation.

### Task 2: Implement signed-displacement behavior

**Files:**
- Modify: `Core/User/behavior.c`
- Modify: `Core/User/behavior.h`

- [ ] Store direction from `route_m`, not `speed`.
- [ ] Use an inclusive tolerance/crossing condition based on signed target error.
- [ ] Validate all inputs before starting wheels and safely handle a missing condition task.
- [ ] Add/update Chinese Doxygen documenting signed `route_m` semantics.

### Task 3: Verify

- [ ] Compile and run the focused host regression test.
- [ ] Run the existing host test executables.
- [ ] Build the STM32 CMake target.
- [ ] Check modified source/header files decode as UTF-8 and preserve unrelated worktree changes.
