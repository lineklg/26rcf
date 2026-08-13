#include "behavior.h"
#include "DRV8870.h"
#include "SYN6288.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"

DRV8870_Motor motor_ic[4];
Encoder motor_enc[4];
Motor motor[4];
ServoPosition arm_servo[3];

static float wheel_target_axis;
static float wheel_target_angle;
static int8_t wheel_target_direction;

static uint8_t Wheel_Stop_AxisX_Condition_Task()
{
    // if (fabsf(radar_get_axis[0] - wheel_target_axisx) <= WHEEL_TARGET_AXIS_MAX_ERROR)
    if ((radar_get_axis[0] - wheel_target_axis) * wheel_target_direction >= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t Wheel_Stop_AxisY_Condition_Task()
{
    // if (fabsf(radar_get_axis[0] - wheel_target_axisx) <= WHEEL_TARGET_AXIS_MAX_ERROR)
    if ((radar_get_axis[1] - wheel_target_axis) * wheel_target_direction >= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t Wheel_Stop_Angle_Condition_Task()
{
    if (fabsf(radar_get_angle - wheel_target_angle) <= WHEEL_TARGET_ANGLE_MAX_ERROR)
    {
        return 1;
    }
    else
    {
        return 0;
    }
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
    WheelPID_Forward(speed);
}

void Wheel_Turn(float speed)
{
    WheelPID_Turn(speed);
}

void Wheel_Stop(void)
{
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
    if (fabsf(speed) <= 0.0001)
    {
        return;
    }
    Wheel_Forward(speed);
    wheel_target_axis = radar_get_axis[0] + route_m;
    wheel_target_axis += speed > 0 ? -WHEEL_TARGET_AXIS_ERROR : WHEEL_TARGET_AXIS_ERROR;
    wheel_target_direction = speed > 0 ? 1 : -1;
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_AxisX_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Wheel_Stop_WithDelay(uint32_t time_ms)
{
    Task_SetRunTick_Delay(task_wheel_stop_delay, time_ms);
    Task_Awake(task_wheel_stop_delay);
}

void Wheel_Turn_WithRadar_Angle(float speed, float angle_rad)
{
    Wheel_Turn(speed);
    wheel_target_angle = radar_get_angle + angle_rad;
    while (wheel_target_angle > M_PI)
    {
        wheel_target_angle -= M_PI * 2;
    }
    while (wheel_target_angle < -M_PI)
    {
        wheel_target_angle += M_PI * 2;
    }
    wheel_target_direction = speed > 0 ? 1 : -1;
    Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Wheel_Stop_Angle_Condition_Task});
    Task_Awake(task_wheel_stop_condition1);
}

void Voice_BroadCast(uint16_t situation)
{
    switch (situation)
    {
    case 1:
        SYN_FrameInfo(&huart2, 0, "[v16]轻微干旱");
        break;
    case 2:
        SYN_FrameInfo(&huart2, 0, "[v16]一般干旱");
        break;
    case 3:
        SYN_FrameInfo(&huart2, 0, "[v16]严重干旱");
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
        ServoPosition_SetPosition(&arm_servo[1], -0.62);
        ServoPosition_SetPosition(&arm_servo[2], -0.62);
        break;
    case 1:
        ServoPosition_SetPosition(&arm_servo[0], 0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.54);
        ServoPosition_SetPosition(&arm_servo[2], -0.07);
        break;
    case 2:
        ServoPosition_SetPosition(&arm_servo[0], -0.5);
        ServoPosition_SetPosition(&arm_servo[1], 0.53);
        ServoPosition_SetPosition(&arm_servo[2], -0.06);
        break;
    default:
        break;
    }
}
