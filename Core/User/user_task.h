#ifndef USER_TASK_H
#define USER_TASK_H

#include "user.h"
#include "behavior.h"

extern Task *task_pump_spray;
extern Task *task_pump_stop;
extern Task *task_change_state_delay;
extern Task *task_wheel_stop_delay;
extern Task *task_wheel_stop_condition1;

extern uint8_t pump_spray_time;
extern StateMachine *machine_delay;
extern uint16_t next_state_id_delay;

void User_Task_Init(void);

#endif // !USER_TASK_H