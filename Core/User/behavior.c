#include "behavior.h"
#include "DRV8870.h"

static DRV8870_Motor motor_ic[4];  // 0-4:A-D
static Encoder motor_enc[4];
static Motor motor[4];

void BaseMotor_Init(void)
{
    DRV8870_Motor_Init(&motor_ic[0], &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, 0);
    DRV8870_Motor_Init(&motor_ic[1], &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, 1);
    DRV8870_Motor_Init(&motor_ic[2], &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 1);
    DRV8870_Motor_Init(&motor_ic[3], &htim12, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);

    Encoder_Create_UsePin(&motor_enc[0], (GPIO_Pin){BM_A_EA_GPIO_Port, BM_A_EA_Pin}, (GPIO_Pin){BM_A_EB_GPIO_Port, BM_A_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[1], (GPIO_Pin){BM_B_EA_GPIO_Port, BM_B_EA_Pin}, (GPIO_Pin){BM_B_EB_GPIO_Port, BM_B_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[2], (GPIO_Pin){BM_C_EA_GPIO_Port, BM_C_EA_Pin}, (GPIO_Pin){BM_C_EB_GPIO_Port, BM_C_EB_Pin});
    Encoder_Create_UsePin(&motor_enc[3], (GPIO_Pin){BM_D_EA_GPIO_Port, BM_D_EA_Pin}, (GPIO_Pin){BM_D_EB_GPIO_Port, BM_D_EB_Pin});
    
    // motor[0] = Motor_Init(&motor_enc, 990, 0.267, HAL_GetTick)
}

void BaseMotor_Forward(uint16_t speed)
{

}

void BaseMotor_Turn(uint8_t direction, uint16_t time, uint16_t speed)
{

}

void Voice_BroadCast(uint16_t situation)
{

}

void Pump_Start(void)
{

}

void Pump_Stop(void)
{

}

void Arm_Init(void)
{

}

void Arm_RoughAdjustment(uint8_t direction)
{
    switch (direction)
    {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    default:
        break;
    }
}
