#include "state_machine.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    uint16_t state_id;
    uint8_t event;
} StateEventRecord;

static StateEventRecord event_records[8];
static size_t event_count;
static uint32_t fake_tick;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

static void RecordStateChange(uint16_t state_id, uint8_t event)
{
    assert(event_count < (sizeof(event_records) / sizeof(event_records[0])));
    event_records[event_count].state_id = state_id;
    event_records[event_count].event = event;
    event_count++;
}

int main(void)
{
    StateMachine machine;

    _Static_assert(
        sizeof(machine.current_state_id) == sizeof(uint16_t),
        "current state must be stored as a uint16_t ID"
    );
    _Static_assert(
        sizeof(machine.last_state_change_tick) == sizeof(uint32_t),
        "last change tick must be stored as uint32_t"
    );

    fake_tick = 100U;
    StateMachine_Init(&machine, 1U, RecordStateChange);
    assert(StateMachine_GetCurrent(&machine) == 1U);
    assert(StateMachine_GetLastChangeTick(&machine) == 100U);
    assert(event_count == 1U);
    assert(event_records[0].state_id == 1U);
    assert(event_records[0].event == (uint8_t)STATE_ENTER);

    fake_tick = 200U;
    assert(StateMachine_GetTick() == 200U);
    assert(StateMachine_Change(&machine, 1U) == 1U);
    assert(StateMachine_GetLastChangeTick(&machine) == 100U);
    assert(event_count == 1U);

    fake_tick = 250U;
    assert(StateMachine_Change(&machine, 2U) == 1U);
    assert(StateMachine_GetCurrent(&machine) == 2U);
    assert(StateMachine_GetLastChangeTick(&machine) == 250U);
    assert(event_count == 3U);
    assert(event_records[1].state_id == 1U);
    assert(event_records[1].event == (uint8_t)STATE_EXIT);
    assert(event_records[2].state_id == 2U);
    assert(event_records[2].event == (uint8_t)STATE_ENTER);

    fake_tick = 400U;
    assert(StateMachine_Change(&machine, STATE_MACHINE_NO_STATE) == 1U);
    assert(StateMachine_GetCurrent(&machine) == STATE_MACHINE_NO_STATE);
    assert(StateMachine_GetLastChangeTick(&machine) == 400U);
    assert(event_count == 4U);
    assert(event_records[3].state_id == 2U);
    assert(event_records[3].event == (uint8_t)STATE_EXIT);

    assert(StateMachine_Change(NULL, 3U) == 0U);
    assert(StateMachine_GetCurrent(NULL) == STATE_MACHINE_NO_STATE);
    assert(StateMachine_GetLastChangeTick(NULL) == 0U);

    return 0;
}
