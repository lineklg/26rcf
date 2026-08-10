# State Machine Last-Change Tick

## Goal

Record the HAL tick at which a state machine was initialized or most recently
changed to a different state.

## API Design

- Add `uint32_t last_state_change_tick` to `StateMachine`.
- Add `uint32_t StateMachine_GetTick(void)`, implemented as a thin wrapper
  around `HAL_GetTick()`. The state-machine implementation uses this function
  whenever it records a timestamp.
- Add `uint32_t StateMachine_GetLastChangeTick(const StateMachine *machine)` to
  query the stored timestamp. A null machine returns `0U`.

## Transition Semantics

- `StateMachine_Init()` stores `StateMachine_GetTick()` after validating the
  machine pointer, including when the initial state is `STATE_MACHINE_NO_STATE`.
- A transition to the same state remains a successful no-op and does not update
  the timestamp.
- A transition to a different state, including `STATE_MACHINE_NO_STATE`, stores
  `StateMachine_GetTick()` after the state ID is updated and before the new
  state's enter callback.
- Existing callback ordering and null-machine behavior remain unchanged.
- The timestamp is an unsigned 32-bit HAL tick and follows the normal HAL tick
  wraparound behavior.

## HAL Dependency and Host Tests

`User/state_machine.c` declares the existing HAL symbol locally instead of
pulling MCU-specific headers into the public state-machine interface. The host
test supplies a deterministic `HAL_GetTick()` implementation so tests can
assert initialization, real transitions, same-state no-ops, no-state
transitions, and null queries without depending on MCU hardware.

## Scope

This change does not alter state IDs, callback event values, transition
validation, or application state-machine callers beyond the added timestamp
field/API.
