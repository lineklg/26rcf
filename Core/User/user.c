#include "user.h"

// 0：空闲/设置 1：A区灌溉 2：移动到B区 3：B区灌溉 6:移动到终点区
StateMachine main_state_machine; 

// 0: 准备 1：移动到下一个灌溉点 2：转向左边 3：转向右边 4：语音播报 5：浇灌
StateMachine area_A_state_machine;
StateMachine area_B_state_machine;

/* A区使用的变量 */
// 干旱情况; 1：轻微干旱 2：一般干旱 3：严重干旱 对应6个位置
uint8_t area_A_situation[6];
uint8_t area_A_current_position;

/* B区使用的变量 */
// 灌溉顺序;
uint8_t area_B_sequence[4];
// 干旱情况; 1：轻微干旱 2：一般干旱 3：严重干旱 对应4个需要灌溉的位置
uint8_t area_B_situation[4];
uint8_t area_B_current_position;

/* Debug */
// 0:正常数据 1:检测到帧尾 2:异常状态
StateMachine debug_uart_rx_state_machine;

uint8_t debug_uart_rx_buffer[40];
uint8_t debug_uart_rx_buffer_count;

void User_Init(void)
{
    TimeUs_Init();
    Task_Init(HAL_GetTick);
    Wheel_Init();
    StateMachine_Init(&main_state_machine, 0, Main_State_Change);
    StateMachine_Init(&area_A_state_machine, STATE_MACHINE_NO_STATE, Main_State_Change);
    StateMachine_Init(&area_B_state_machine, STATE_MACHINE_NO_STATE, Main_State_Change);
    Arm_Init();
    Arm_RoughAdjustment(0);

    debug_uart_rx_buffer_count = 0;
    HAL_UART_Receive_IT(&huart1, &debug_uart_rx_buffer[0], 1);
}

void User_Update(void)
{
    Task_Update();
    
}

void Main_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    switch (state_id)
    {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;
    }
}

void Area_A_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    switch (state_id)
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

void Area_B_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    switch (state_id)
    {
    case 0:
        break;
    case 1:
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    default:
        break;
    }
}

static void Debug_UARTRx_Operation(void)
{
    switch (debug_uart_rx_buffer[0])
    {
    case 0x10:                              // 轮子直行 后面数据为速度 后3个字节为0xF0
        float speed
        break;
    case 0x11:                              // 轮子转弯 后面数据为速度 后3个字节为0xF0
        break;
    case 0x12:                              // 停车 后3个字节为0xF0
        break;                               
    default:
        break;
    }
}

void Debug_UARTRx_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    switch (state_id)
    {
    case 0:
        break;
    case 1:
        Debug_UARTRx_Operation();
        debug_uart_rx_buffer_count = 0;
        break;
    case 2:
        debug_uart_rx_buffer_count = 0;
        break;
    default:
        StateMachine_Change(&debug_uart_rx_state_machine, 2);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        HAL_UART_Receive_IT(&huart1, &debug_uart_rx_buffer[debug_uart_rx_buffer_count], 1);
        if (debug_uart_rx_buffer[debug_uart_rx_buffer_count] == 0xF0 &&
            debug_uart_rx_buffer[debug_uart_rx_buffer_count - 1] == 0xF0 &&
            debug_uart_rx_buffer[debug_uart_rx_buffer_count - 2] == 0xF0)
        {
            StateMachine_Change(&debug_uart_rx_state_machine, 1);
        }   
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_GPIO_EXTI_Callback(GPIO_Pin);
}

