#include "user_task.h"
#include "state_machine.h"
#include "task.h"

Task *task_pump_spray;
Task *task_pump_stop;
Task *task_change_state_delay;

uint8_t pump_spray_time;
StateMachine *machine_delay;
uint16_t next_state_id_delay;

static void Pump_Spray_Task()
{
    Pump_Start();
    Task_SetRunTick_Delay(task_pump_stop, 150);
    Task_Awake(task_pump_stop);
}

static void Pump_Stop_Task()
{
    Pump_Stop();
    pump_spray_time--;
    if (pump_spray_time > 0 && pump_spray_time <= 3)
    {
        Task_SetRunTick_Delay(task_pump_spray, 500);
        Task_Awake(task_pump_spray);
    }
}

static void Change_State_Delay()
{
    StateMachine_Change(machine_delay, next_state_id_delay);
}

void User_Task_Init(void)
{
    Task_Init(HAL_GetTick);
    Task_Create(&task_pump_spray, Pump_Spray_Task, TASK_COMMON);
    Task_Create(&task_pump_stop, Pump_Stop_Task, TASK_COMMON);
    Task_Create(&task_change_state_delay, Change_State_Delay, TASK_COMMON);
}