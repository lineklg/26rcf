#include "wheel_pid.h"

#include "pid.h"
#include "usart.h"
#include "vofa.h"

#include <stddef.h>

static PID *wheel_pid[WHEEL_PID_COUNT];
static Motor *wheel_motor;
static DRV8870_Motor *wheel_driver;
static float wheel_target[WHEEL_PID_COUNT];
static uint8_t wheel_pid_running;

/**
 * @brief 在停止状态下重置历史并启动四轮速度环。
 * @return 无。
 */
static void WheelPID_Start(void)
{
    uint8_t i;

    if (wheel_pid_running == 0U) {
        for (i = 0U; i < WHEEL_PID_COUNT; i++) {
            if (wheel_pid[i] != NULL) {
                PID_ResetHistory(wheel_pid[i]);
            }
        }
        wheel_pid_running = 1U;
        PID_Start();
    }
}

static float WheelPID_FeedForward(float target_speed)
{
    float result = 0;
    if (target_speed > 0)
    {
        result = 0.51f + 0.805f * target_speed;
    }
    if (target_speed < 0)
    {
        result = -0.51f + 0.805f * target_speed;
    }
    return result;
}

/**
 * @brief 初始化四轮速度 PID 控制器。
 * @param[in,out] motors 四个车轮电机对象。
 * @param[in,out] drivers 四个车轮驱动对象。
 * @return 无。
 */
void WheelPID_Init(
    Motor motors[WHEEL_PID_COUNT],
    DRV8870_Motor drivers[WHEEL_PID_COUNT]
)
{
    uint8_t i;

    wheel_motor = motors;
    wheel_driver = drivers;
    wheel_pid_running = 0U;

    PID_Init();
    PID_SetPeriodMs(WHEEL_PID_PERIOD_MS);
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_pid[i] = NULL;
        wheel_target[i] = 0.0f;
        PID_Create(&wheel_pid[i], 0.2f, 0.1f, 0.0f);
        if (wheel_pid[i] != NULL) {
            PID_SetOutputMax(wheel_pid[i], 0.4f);
            PID_SetIntegralMax(wheel_pid[i], 3);
        }
    }
}

/**
 * @brief 设置四轮同向直行目标速度并启动速度环。
 * @param[in] speed 四个车轮的目标速度。
 * @return 无。
 */
void WheelPID_Forward(float speed)
{
    uint8_t i;

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_target[i] = speed;
        // if (i == 0 || i == 2)
        // {
        //     wheel_target[i] *= 0.985;
        // }
    }
    WheelPID_Start();
}

/**
 * @brief 设置原地转向目标速度并启动速度环。
 * @param[in] speed B、D 轮目标速度，A、C 轮使用其相反数。
 * @return 无。
 */
void WheelPID_Turn(float speed)
{
    wheel_target[0] = speed;
    wheel_target[1] = -speed;
    wheel_target[2] = speed;
    wheel_target[3] = -speed;
    WheelPID_Start();
}

/**
 * @brief 设置指定车轮的目标线速度并启动速度环。
 * @param[in] index 车轮索引，0 至 3 分别对应 A 至 D 轮。
 * @param[in] speed 目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeed(uint8_t index, float speed)
{
    if (index >= WHEEL_PID_COUNT) {
        return;
    }

    wheel_target[index] = speed;
    WheelPID_Start();
}

/**
 * @brief 分别设置 A、B、C、D 四轮目标线速度并启动速度环。
 * @param[in] speed_a A 轮目标线速度，单位为 m/s。
 * @param[in] speed_b B 轮目标线速度，单位为 m/s。
 * @param[in] speed_c C 轮目标线速度，单位为 m/s。
 * @param[in] speed_d D 轮目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeeds(
    float speed_a,
    float speed_b,
    float speed_c,
    float speed_d
)
{
    wheel_target[0] = speed_a;
    wheel_target[1] = speed_b;
    wheel_target[2] = speed_c;
    wheel_target[3] = speed_d;
    WheelPID_Start();
}

/**
 * @brief 设置指定车轮的 PID 参数。
 * @param[in] index 车轮索引。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] kd 微分系数。
 * @return 无。
 */
void WheelPID_SetK(uint8_t index, float kp, float ki, float kd)
{
    if ((index < WHEEL_PID_COUNT) && (wheel_pid[index] != NULL)) {
        PID_SetK(wheel_pid[index], kp, ki, kd);
    }
}

/**
 * @brief 停止四轮速度环并清除目标与 PID 历史。
 * @return 无。
 */
void WheelPID_Stop(void)
{
    uint8_t i;

    PID_Stop();
    wheel_pid_running = 0U;
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_target[i] = 0.0f;
        if (wheel_pid[i] != NULL) {
            PID_ResetHistory(wheel_pid[i]);
            DRV8870_SetDutyPercent(&wheel_driver[i], 0);
        }
    }
}

/**
 * @brief 执行一次四轮速度 PID 计算并写入驱动占空比。
 * @return 无。
 */
void PID_Task(void)
{
    uint8_t i;
    float vofa_data[WHEEL_PID_COUNT * 3U] = {0.0f};

    if ((wheel_pid_running == 0U) ||
        (wheel_motor == NULL) ||
        (wheel_driver == NULL)) {
        return;
    }

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        float feedback;
        float output;
        uint8_t channel = i * 3U;

        vofa_data[channel] = wheel_target[i];
        if (wheel_pid[i] == NULL) {
            continue;
        }
        feedback = Motor_CalcSpeed_Smooth(&wheel_motor[i]);
        output = PID_Calc(wheel_pid[i], wheel_target[i], feedback);
        vofa_data[channel + 1U] = feedback;
        vofa_data[channel + 2U] = output;
        // DRV8870_SetDutyPercent(&wheel_driver[i], output + WheelPID_FeedForward(wheel_target[i]));
        DRV8870_SetDutyPercent(&wheel_driver[i], output + WheelPID_FeedForward(wheel_target[i]));
    }

    VOFA_JustFloat_UART_Send(
        &huart1,
        vofa_data,
        (uint8_t)(WHEEL_PID_COUNT * 3U)
    );
}
