#include "state_machine.h"

#include <stddef.h>

extern uint32_t HAL_GetTick(void);

uint32_t StateMachine_GetTick(void)
{
    return HAL_GetTick();
}

void StateMachine_Init(
    StateMachine *machine,
    uint16_t initial_state_id,
    StateChangeCallback callback
)
{
    if (machine == NULL)
    {
        return;
    }

    machine->current_state_id = initial_state_id;
    machine->callback = callback;
    machine->last_state_change_tick = StateMachine_GetTick();

    if ((initial_state_id != STATE_MACHINE_NO_STATE) && (callback != NULL))
    {
        callback(initial_state_id, (uint8_t)STATE_ENTER);
    }
}

uint8_t StateMachine_Change(StateMachine *machine, uint16_t next_state_id)
{
    if (machine == NULL)
    {
        return 0U;
    }

    if (machine->current_state_id == next_state_id)
    {
        return 1U;
    }

    if ((machine->current_state_id != STATE_MACHINE_NO_STATE) &&
        (machine->callback != NULL))
    {
        machine->callback(machine->current_state_id, (uint8_t)STATE_EXIT);
    }

    machine->current_state_id = next_state_id;
    machine->last_state_change_tick = StateMachine_GetTick();

    if ((next_state_id != STATE_MACHINE_NO_STATE) && (machine->callback != NULL))
    {
        machine->callback(next_state_id, (uint8_t)STATE_ENTER);
    }

    return 1U;
}

uint16_t StateMachine_GetCurrent(const StateMachine *machine)
{
    if (machine == NULL)
    {
        return STATE_MACHINE_NO_STATE;
    }

    return machine->current_state_id;
}

uint32_t StateMachine_GetLastChangeTick(const StateMachine *machine)
{
    if (machine == NULL)
    {
        return 0U;
    }

    return machine->last_state_change_tick;
}
