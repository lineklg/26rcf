#include "wheel_pid.h"

#include "pid.h"
#include "usart.h"
#include "vofa.h"

#include <math.h>
#include <stddef.h>

extern volatile float radar_get_axis[2];
extern volatile float radar_get_angle;

static PID *wheel_pid[WHEEL_PID_COUNT];
static PID *radar_angle_error_pid;
static PID *radar_pos_error_pid;

static Motor *wheel_motor;
static DRV8870_Motor *wheel_driver;
static float wheel_target[WHEEL_PID_COUNT];
static uint8_t wheel_pid_running;

static float radar_target_angle;
uint8_t enable_fix_angle;
static uint8_t enable_fix_angle_previous;

uint8_t enable_fix_pos;
static uint8_t enable_fix_pos_previous;
static float radar_target_position;
static WheelPID_PositionAxis radar_target_axis;

/**
 * @brief 将角度归一化到 [-pi, pi]。
 * @param[in] angle 待归一化的角度，单位为弧度。
 * @return 归一化后的角度。
 */
static float WheelPID_NormalizeAngle(float angle)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;

    while (angle > pi) {
        angle -= two_pi;
    }
    while (angle < -pi) {
        angle += two_pi;
    }
    return angle;
}

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
        result = 0.04f + 2.0f * target_speed;
    }
    if (target_speed < 0)
    {
        result = -0.04f + 2.0f * target_speed;
    }
    return result;
    // return 0.805f * target_speed;
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
    radar_target_angle = 0.0f;
    enable_fix_angle = 0U;
    enable_fix_angle_previous = 0U;
    radar_target_position = 0.0f;
    radar_target_axis = WHEEL_PID_POSITION_AXIS_X;
    enable_fix_pos = 0U;
    enable_fix_pos_previous = 0U;

    PID_Init();
    PID_SetPeriodMs(WHEEL_PID_PERIOD_MS);
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        wheel_pid[i] = NULL;
        wheel_target[i] = 0.0f;
        PID_Create(&wheel_pid[i], 1.0f, 0.3f, 0.0f);
        if (wheel_pid[i] != NULL) {
            PID_SetOutputMax(wheel_pid[i], 1.0f);
            PID_SetIntegralMax(wheel_pid[i], (wheel_pid[i]->output_max / wheel_pid[i]->ki));
        }
    }
    radar_angle_error_pid = NULL;
    PID_Create(&radar_angle_error_pid, 2.5f, 0.05f, 0.0f);
    if (radar_angle_error_pid != NULL) {
        PID_SetOutputMax(radar_angle_error_pid, 0.1f);
        PID_SetIntegralMax(radar_angle_error_pid, (radar_angle_error_pid->output_max / radar_angle_error_pid->ki));
    }
    radar_pos_error_pid = NULL;
    PID_Create(&radar_pos_error_pid, 2.0f, 0.0f, 0.0f);
    if (radar_pos_error_pid != NULL) {
        PID_SetOutputMax(radar_pos_error_pid, 0.1f);
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
 * @brief 设置角度保持目标值。
 * @param[in] target_angle 目标角度，单位为弧度。
 * @return 无。
 */
void WheelPID_SetTargetAngle(float target_angle)
{
    radar_target_angle = WheelPID_NormalizeAngle(target_angle);
}

/**
 * @brief 设置单轴位置保持目标。
 * @param[in] axis 要保持的雷达世界坐标轴。
 * @param[in] target_position 目标位置，单位为米。
 * @return 无。
 */
void WheelPID_SetTargetPosition(
    WheelPID_PositionAxis axis,
    float target_position
)
{
    if (((axis != WHEEL_PID_POSITION_AXIS_X) &&
         (axis != WHEEL_PID_POSITION_AXIS_Y)) ||
        !isfinite(target_position)) {
        return;
    }
    if ((axis != radar_target_axis) && (radar_pos_error_pid != NULL)) {
        PID_ResetHistory(radar_pos_error_pid);
    }
    radar_target_axis = axis;
    radar_target_position = target_position;
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
    enable_fix_angle = 0;
    enable_fix_pos = 0U;
    enable_fix_pos_previous = 0U;
    if (radar_pos_error_pid != NULL) {
        PID_ResetHistory(radar_pos_error_pid);
    }
}

/**
 * @brief 执行一次四轮速度 PID 计算并写入驱动占空比。
 * @return 无。
 */
void PID_Task(void)
{
    uint8_t i;
    float angle_output = 0.0f;
    float position_output = 0.0f;
    float vofa_data[WHEEL_PID_COUNT * 3U] = {0.0f};

    if ((wheel_pid_running == 0U) ||
        (wheel_motor == NULL) ||
        (wheel_driver == NULL)) {
        return;
    }

    if (enable_fix_angle != enable_fix_angle_previous) {
        if (radar_angle_error_pid != NULL) {
            PID_ResetHistory(radar_angle_error_pid);
        }
        enable_fix_angle_previous = enable_fix_angle;
    }

    if (enable_fix_pos != enable_fix_pos_previous) {
        if (radar_pos_error_pid != NULL) {
            PID_ResetHistory(radar_pos_error_pid);
        }
        enable_fix_pos_previous = enable_fix_pos;
    }

    if ((enable_fix_pos == 1U) &&
        (radar_pos_error_pid != NULL) &&
        isfinite(radar_get_axis[radar_target_axis]) &&
        isfinite(radar_get_angle)) {
        float axis_output = PID_Calc(
            radar_pos_error_pid,
            radar_target_position,
            radar_get_axis[radar_target_axis]
        );
        position_output = axis_output;
    }

    if ((enable_fix_angle == 1U) && (radar_angle_error_pid != NULL)) {
        float angle_error = WheelPID_NormalizeAngle(
            radar_target_angle - radar_get_angle
        );
        angle_output = PID_Calc(radar_angle_error_pid, 0.0f, angle_error);
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
        float result = output;
        result += WheelPID_FeedForward(wheel_target[i]);
        if (enable_fix_angle == 1U) {
            result += ((i == 0U) || (i == 2U))
                ? angle_output
                : -angle_output;
        }
        // if (enable_fix_pos == 1U) {
        //     result += position_output;
        // }
        DRV8870_SetDutyPercent(&wheel_driver[i], result);
    }

    VOFA_JustFloat_UART_Send(
        &huart1,
        vofa_data,
        (uint8_t)(WHEEL_PID_COUNT * 3U)
    );
}
