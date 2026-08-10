#include "pid.h"
#include <math.h>

static PID_Manager pid_manager;

Task *pid_task;

__attribute__((weak)) void PID_Task(void) {}

uint16_t PID_Create_Copy(PID **dest, PID *src)
{
    if (src == NULL || pid_manager.count >= PID_MAX_COUNT)
    {
        dest = NULL;
        return UINT16_MAX;
    }
    uint16_t index = PID_Create(dest, 0, 0, 0);
    PID_Copy(*dest, src);
    return index;
}

/**
 * @brief 创建一个PID配置
 * @param dest pid配置的目标指针
 * @return pid配置对应的索引
 */
uint16_t PID_Create(PID **dest, float kp, float ki, float kd)
{
    if (pid_manager.count >= PID_MAX_COUNT)
    {
        *dest = NULL;
        return UINT16_MAX;
    }
    PID *this = &(pid_manager.pid[pid_manager.count]);
    PID_SetK(this, kp, ki, kd);
    PID_SetIntegralMax(this, 0);
    PID_SetOutputMax(this, 0);
    PID_ResetHistory(this);
    *dest = this;
    pid_manager.count++;
    return pid_manager.count - 1;
}

void PID_Copy(PID *this, PID *src)
{
    PID_SetK(this, src->kp, src->ki, src->kd);
    PID_SetIntegralMax(this, src->integral_max);
    PID_SetOutputMax(this, src->output_max);
}

void PID_SetK(PID *this, float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID_SetIntegralMax(PID *this, float integral_max)
{
    this->integral_max = fabsf(integral_max);
}

void PID_SetOutputMax(PID *this, float output_max)
{
    this->output_max = fabsf(output_max);
}

void PID_ResetHistory(PID *this)
{
    this->prev_feedback = 0;
    this->integral = 0;
}

void PID_ResetHistoryAll()
{
    for (uint16_t i = 0; i < pid_manager.count; i++)
    {
        PID_ResetHistory(&(pid_manager.pid[i]));
    }
}

/**
 * @brief 根据当前设定值和反馈值做pid计算
 * @param this 目标pid的配置设定
 * @param set 设定值
 * @param feedback 反馈值
 * @return 返回计算得出的输出，并更新自身的积分和误差记录
 * @note 该函数计算微分时是根据对反馈求导进行的
 */
float PID_Calc(PID *this, float set, float feedback)
{
    return PID_Calc_Withdt(this, set, feedback, (float)pid_manager.period / 1000);
}

/**
 * @brief 根据当前设定值和反馈值以及周期时间做pid计算
 * @param this 目标pid的配置设定
 * @param set 设定值
 * @param feedback 反馈值
 * @param dt 距上次pid计算经过了多长时间，单位 s
 * @return 返回计算得出的输出，并更新自身的积分和误差记录
 * @note 该函数计算微分时是根据对反馈求导进行的
 */
float PID_Calc_Withdt(PID *this, float set, float feedback, float dt)
{
    float error = set - feedback;

    float proportional = this->kp * error;

    this->integral += error * dt;
    if (this->integral_max != 0)
    {
        if (this->integral > this->integral_max)
        {
            this->integral = this->integral_max;
        }
        else if (this->integral < -(this->integral_max))
        {
            this->integral = -(this->integral_max);
        }
    }
    float integral = this->ki * this->integral;

    float derivative = this->kd * (this->prev_feedback - feedback) / dt;
    this->prev_feedback = feedback;

    float output = proportional + integral + derivative;
    if (this->output_max != 0)
    {
        if (output > this->output_max)
        {
            output = this->output_max;
        }
        else if (output < -(this->output_max))
        {
            output = -(this->output_max);
        }
    }
    return output;
}

void PID_Init()
{
    pid_manager.count = 0;

    Task_Create(&pid_task, PID_Task, TASK_PERIOD);
    PID_SetPeriodMs(20);
}

void PID_Start()
{
    Task_SetRunTick_Current(pid_task);
    Task_Awake(pid_task);
}

void PID_Stop()
{
    Task_Sleep(pid_task);
}

void PID_SetPeriodMs(uint32_t period)
{
    if (period > 0)
    {
        pid_manager.period = period;
        Task_SetExtraData(pid_task, (Task_ExtraData){.period = pid_manager.period});
    }
}

void PID_Exportk(float *dest, PID *this)
{
    dest[0] = this->kp;
    dest[1] = this->ki;
    dest[2] = this->kd;
}
