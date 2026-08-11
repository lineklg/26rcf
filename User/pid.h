#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>
#include "task.h"

#define PID_MAX_COUNT           9

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_feedback;

    float integral_max;                         // 设置为 0 即视作不限制
    float output_max;
} PID;

typedef struct 
{
    PID pid[PID_MAX_COUNT];
    uint16_t count;
    uint32_t period;                            // ms
} PID_Manager;

void PID_Task(void);

uint16_t PID_Create_Copy(PID **dest, PID *src);
uint16_t PID_Create(PID **dest, float kp, float ki, float kd);
void PID_Copy(PID *this, PID *src);
void PID_SetK(PID *this, float kp, float ki, float kd);
void PID_SetIntegralMax(PID *this, float integral_max);
void PID_SetOutputMax(PID *this, float output_max);
void PID_ResetHistory(PID *this);
void PID_ResetHistoryAll();
float PID_Calc(PID *this, float set, float feedback);
float PID_Calc_Withdt(PID *this, float set, float feedback, float dt);

void PID_Init();
void PID_Start();
void PID_Stop();
void PID_SetPeriodMs(uint32_t period);

void PID_Exportk(float *dest, PID *this);

#endif
