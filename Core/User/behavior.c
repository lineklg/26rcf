#include "behavior.h"
#include "DRV8870.h"
#include "SYN6288.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"

#include <math.h>

DRV8870_Motor motor_ic[4];
Encoder motor_enc[4];
Motor motor[4];
ServoPosition arm_servo[3];

static float wheel_target_axis;
static float wheel_target_angle;
static float wheel_target_start_position[2];
static float wheel_target_direction_cos;
static float wheel_target_direction_sin;
static int8_t wheel_target_direction;

/**
 * @brief 将角度归一化到 [-pi, pi]。
 * @param[in] angle 待归一化的角度，单位为弧度。
 * @return 归一化后的角度，单位为弧度。
 */
static float Wheel_NormalizeAngle(float angle)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 2.0f * pi;

    return remainderf(angle, two_pi);
}

/**
 * @brief 取消当前雷达角度保持和未完成的车轮停止条件任务。
 * @return 无。
 */
static void Wheel_CancelRadarAngleControl(void)
{
    enable_fix_angle = 0U;
    if (task_wheel_stop_condition1 != NULL)
    {
        Task_Sleep(task_wheel_stop_condition1);
    }
}

static uint8_t Wheel_Stop_AxisX_Condition_Task()
{
    float axis_error;

    if (!isfinite(radar_get_axis[0]))
    {
        return 1;
    }

    axis_error = wheel_target_axis - radar_get_axis[0];
    if ((fabsf(axis_error) <= WHEEL_TARGET_AXIS_ERROR) ||
        (axis_error * wheel_target_direction <= 0.0f))
    {
        return 1;
    }
    return 0;
}

static uint8_t Wheel_Stop_AxisY_Condition_Task()
{
    float axis_error;

    if (!isfinite(radar_get_axis[1]))
    {
        return 1;
    }

    axis_error = wheel_target_axis - radar_get_axis[1];
    if ((fabsf(axis_error) <= WHEEL_TARGET_AXIS_ERROR) ||
        (axis_error * wheel_target_direction <= 0.0f))
    {
        return 1;
    }
    return 0;
}

static uint8_t Wheel_Stop_Angle_Condition_Task()
{
    float angle_error = Wheel_NormalizeAngle(wheel_target_angle - radar_get_angle);

    if ((fabsf(angle_error) <= WHEEL_TARGET_ANGLE_MAX_ERROR) ||
        (angle_error * wheel_target_direction <= 0.0f))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 判断沿调用瞬间朝向的投影位移是否已经到达目标。
 * @return 到达目标、越过目标或雷达数据无效时返回 1，否则返回 0。
 */
static uint8_t Wheel_Stop_CurrentAngle_Condition_Task()
{
    float progress;
    float projection_error;

    if (!isfinite(radar_get_axis[0]) || !isfinite(radar_get_axis[1]) ||
        !isfinite(radar_get_angle))
    {
        return 1;
    }

    progress =
        (radar_get_axis[0] - wheel_target_start_position[0]) * wheel_target_direction_cos +
        (radar_get_axis[1] - wheel_target_start_position[1]) * wheel_target_direction_sin;
    projection_error = wheel_target_axis - progress;
    if ((fabsf(projection_error) <= WHEEL_TARGET_AXIS_ERROR) ||
        (projection_error * wheel_target_direction <= 0.0f))
    {
        return 1;
    }
    return 0;
}

void Wheel_Init(void)
{
    DRV8870_Motor_Init(&motor_ic[0], &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, 0);
    DRV8870_Motor_Init(&motor_ic[1], &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, 1);
    DRV8870_Motor_Init(&motor_ic[2], &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 1);
    DRV8870_Motor_Init(&motor_ic[3], &htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);

    Encoder_Create_UsePin(&motor_enc[0], (GPIO_Pin){BM_A_EA_GPIO_Port, BM_A_EA_Pin}, (GPIO_Pin){BM_A_EB_GPIO_Port, BM_A_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[1], (GPIO_Pin){BM_B_EA_GPIO_Port, BM_B_EA_Pin}, (GPIO_Pin){BM_B_EB_GPIO_Port, BM_B_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[2], (GPIO_Pin){BM_C_EA_GPIO_Port, BM_C_EA_Pin}, (GPIO_Pin){BM_C_EB_GPIO_Port, BM_C_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[3], (GPIO_Pin){BM_D_EA_GPIO_Port, BM_D_EA_Pin}, (GPIO_Pin){BM_D_EB_GPIO_Port, BM_D_EB_Pin});
    
    motor[0] = Motor_Init(&motor_enc[0], 3960, 0.267, HAL_GetTick, NULL, NULL, 1);
    motor[1] = Motor_Init(&motor_enc[1], 3960, 0.267, HAL_GetTick, NULL, NULL, 0);
    motor[2] = Motor_Init(&motor_enc[2], 3960, 0.267, HAL_GetTick, NULL, NULL, 1);
    motor[3] = Motor_Init(&motor_enc[3], 3960, 0.267, HAL_GetTick, NULL, NULL, 0);

    WheelPID_Init(motor, motor_ic);
    Wheel_Stop();
}

void Wheel_Forward(float speed)
{
    Wheel_CancelRadarAngleControl();
    WheelPID_Forward(speed);
}

void Wheel_Turn(float speed)
{
    Wheel_CancelRadarAngleControl();
    WheelPID_Turn(speed);
}

void Wheel_Stop(void)
{
    Wheel_CancelRadarAngleControl();
    WheelPID_Stop();
    for (uint8_t i = 0; i < 4; i++)
    {
        DRV8870_Brake(&motor_ic[i]);
    }
}

void Wheel_Forward_WithTime(float speed, uint32_t time_ms)
{
    Wheel_Forward(speed);
    Wheel_Stop_WithDelay(time_ms);
}

void Wheel_Forward_WithRadar_AxisX(float speed, float route_m)
{
    if (!isfinite(speed) || !isfinite(route_m) ||
        !isfinite(radar_get_axis[0]) ||
        (fabsf(speed) <= 0.0001f) ||
        (fabsf(route_m) <= WHEEL_TARGET_AXIS_ERROR) ||
        (task_wheel_stop_condition1 == NULL))
    {
        Wheel_Stop();
        return;
    }

    wheel_target_axis = radar_get_axis[0] + route_m;
    wheel_target_direction = route_m > 0.0f ? 1 : -1;
    Wheel_Forward(speed);
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_AxisX_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Wheel_Forward_WithRadar_AxisY(float speed, float route_m)
{
    if (!isfinite(speed) || !isfinite(route_m) ||
        !isfinite(radar_get_axis[1]) ||
        (fabsf(speed) <= 0.0001f) ||
        (fabsf(route_m) <= WHEEL_TARGET_AXIS_ERROR) ||
        (task_wheel_stop_condition1 == NULL))
    {
        Wheel_Stop();
        return;
    }

    wheel_target_axis = radar_get_axis[1] + route_m;
    wheel_target_direction = route_m > 0.0f ? 1 : -1;
    Wheel_Forward(speed);
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_AxisY_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Wheel_Forward_WithRadar_CurrentAngle(float speed, float route_m)
{
    if (!isfinite(speed) || !isfinite(route_m) ||
        !isfinite(radar_get_axis[0]) || !isfinite(radar_get_axis[1]) ||
        !isfinite(radar_get_angle) ||
        (fabsf(speed) <= 0.0001f) ||
        (fabsf(route_m) <= WHEEL_TARGET_AXIS_ERROR) ||
        (task_wheel_stop_condition1 == NULL))
    {
        Wheel_Stop();
        return;
    }

    wheel_target_axis = route_m;
    wheel_target_direction = route_m > 0.0f ? 1 : -1;
    wheel_target_start_position[0] = radar_get_axis[0];
    wheel_target_start_position[1] = radar_get_axis[1];
    wheel_target_direction_cos = cosf(radar_get_angle);
    wheel_target_direction_sin = sinf(radar_get_angle);
    wheel_target_angle = Wheel_NormalizeAngle(radar_get_angle);

    Wheel_Forward(speed);
    WheelPID_SetTargetAngle(wheel_target_angle);
    enable_fix_angle = 1U;
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_CurrentAngle_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Wheel_Stop_WithDelay(uint32_t time_ms)
{
    Task_SetRunTick_Delay(task_wheel_stop_delay, time_ms);
    Task_Awake(task_wheel_stop_delay);
}

void Wheel_Turn_WithRadar_Angle(float speed, float angle_rad)
{
    float turn_angle;

    if (!isfinite(speed) || !isfinite(angle_rad) || !isfinite(radar_get_angle))
    {
        Wheel_Stop();
        return;
    }

    turn_angle = Wheel_NormalizeAngle(angle_rad);
    if ((fabsf(speed) <= 0.0001f) || (fabsf(turn_angle) <= 0.0001f))
    {
        Wheel_Stop();
        return;
    }

    wheel_target_direction = turn_angle > 0.0f ? 1 : -1;
    Wheel_Turn(-copysignf(fabsf(speed), turn_angle));
    wheel_target_angle = Wheel_NormalizeAngle(radar_get_angle + turn_angle);
    WheelPID_SetTargetAngle(wheel_target_angle);
    enable_fix_angle = 1U;
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_Angle_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Voice_BroadCast(uint16_t situation)
{
    switch (situation)
    {
    case 1:
        SYN_FrameInfo(&huart2, 0, "[v16][t5]轻微干旱");
        break;
    case 2:
        SYN_FrameInfo(&huart2, 0, "[v16][t5]一般干旱");
        break;
    case 3:
        SYN_FrameInfo(&huart2, 0, "[v16][t5]严重干旱");
        break;
    default:
        break;
    }
}

void Pump_Start(void)
{
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
}

void Pump_Stop(void)
{
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
}

void Arm_Init(void)
{
    ServoPosition_Init(
        &arm_servo[0],
        &htim8,
        TIM_CHANNEL_1,
        SERVO_POSITION_OUTPUT_MAIN,
        1375U,
        4125U,
        6875U
    );
    ServoPosition_Init(
        &arm_servo[1],
        &htim8,
        TIM_CHANNEL_2,
        SERVO_POSITION_OUTPUT_MAIN,
        1375U,
        4125U,
        6875U
    );
    ServoPosition_Init(
        &arm_servo[2],
        &htim8,
        TIM_CHANNEL_3,
        SERVO_POSITION_OUTPUT_MAIN,
        1375U,
        4125U,
        6875U
    );
}

void Arm_RoughAdjustment(uint8_t direction)
{
    switch (direction)
    {
    case 0:
        ServoPosition_SetPosition(&arm_servo[0], 0);
        ServoPosition_SetPosition(&arm_servo[1], -0.58);
        ServoPosition_SetPosition(&arm_servo[2], -0.58);
        break;
    case 1:                         // A
        ServoPosition_SetPosition(&arm_servo[0], 0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.54);
        ServoPosition_SetPosition(&arm_servo[2], -0.07);
        break;
    case 2:
        ServoPosition_SetPosition(&arm_servo[0], -0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.54);
        ServoPosition_SetPosition(&arm_servo[2], -0.07);
        break;
    case 3:                         // B
        ServoPosition_SetPosition(&arm_servo[0], 0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.24);
        ServoPosition_SetPosition(&arm_servo[2], -0.27);
        break;
    case 4:
        ServoPosition_SetPosition(&arm_servo[0], -0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.24);
        ServoPosition_SetPosition(&arm_servo[2], -0.27);
        break;
    default:
        break;
    }
}
