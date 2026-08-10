#ifndef _TASK_H_
#define _TASK_H_

#include <stdint.h>
#include <stddef.h>

#define TASK_QUEUE_SIZE             20
#define TASK_PERIOD_MAX             (UINT32_MAX / 2)

typedef struct sTask Task;

typedef void (*FuncCallback)(void);
typedef void (*EndProcessCallback)(Task*);
typedef uint8_t (*ConditionCallback)(void);
typedef uint32_t (*GetTickCallback)(void);

typedef enum
{
    TASK_COMMON,
    TASK_PERIOD,
    TASK_CONDITION
} Task_Type;

typedef enum
{
    TASK_AWAKE,
    TASK_SLEEP
} Task_State;

typedef union
{
    uint32_t period;
    ConditionCallback condition;
} Task_ExtraData;

typedef struct sTask
{
    FuncCallback func;

    uint32_t run_tick;
    Task_Type type;
    EndProcessCallback end_process;
    Task_ExtraData data;
    Task_State state;
} Task;

typedef struct 
{
    Task queue[TASK_QUEUE_SIZE];
    GetTickCallback get_tick;
    uint16_t count;
    uint32_t last_tick;
} Task_Manager;

uint16_t Task_Create(Task **dest, FuncCallback func, Task_Type type);
uint16_t Task_CreateAndStart(Task **dest, FuncCallback func, Task_Type type, Task_ExtraData data);
void Task_SetExtraData(Task *this, Task_ExtraData data);
void Task_SetRunTick(Task *this, uint32_t run_tick);
void Task_SetRunTick_Current(Task *this);
void Task_SetRunTick_Delay(Task *this, uint32_t delay);
void Task_Awake(Task *this);
void Task_Sleep(Task *this);

void Task_Init(GetTickCallback get_tick);
void Task_Get(Task **dest, uint16_t index);
void Task_Update();

#endif 