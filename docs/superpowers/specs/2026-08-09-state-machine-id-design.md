# State Machine ID Representation

## Goal

Represent the active state in the state machine by a numeric ID instead of a pointer to the `StateNode` structure.

## API Design

- Remove `StateNode`, which currently contains only an `id` field.
- Define a `STATE_MACHINE_NO_STATE` sentinel with value `UINT16_MAX` for the uninitialized/no-active-state case.
- Change `StateChangeCallback` to receive a `uint16_t state_id` and the existing enter/exit event value.
- Replace `StateMachine.current_state` with `uint16_t current_state_id`.
- Change `StateMachine_Init`, `StateMachine_Change`, and `StateMachine_GetCurrent` to accept or return `uint16_t` state IDs.
- Preserve the existing transition order: exit the old state, update the current ID, then enter the new state. Changing to the same ID remains a no-op.
- Preserve the existing invalid-machine behavior: initialization returns without action, change returns `0`, and get-current returns the no-state sentinel.

## Caller Updates

Update `Core/User/user.c` to store the four state IDs directly and adapt `State_Change` to the numeric callback signature. No state-node array or pointer lifetime is needed.

## Verification

Build the project with the existing Debug CMake configuration. Confirm there are no remaining `StateNode` references in the application state-machine code and that the public API compiles for all current callers.

## Scope

This change does not add transition tables, dynamic allocation, validation of state ID ranges, or changes to enter/exit event semantics.

## Documentation

Use UTF-8 encoded Chinese Doxygen comments for the public state-machine sentinel, callback, structure fields, and API declarations. Keep comments focused on parameter meaning, no-state semantics, return values, and transition order.
