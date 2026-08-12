#include "user.h"
#include "behavior.h"
#include "stm32h7xx_hal_uart.h"
#include <stdio.h>

// 0：空闲/设置 1：A区灌溉 2：移动到B区 3：B区灌溉 6:移动到终点区
StateMachine main_state_machine; 

// 0: 准备 1：移动到下一个灌溉点 2：转向左边 3：转向右边 4：语音播报 5：浇灌 6：回正
StateMachine area_A_state_machine;
StateMachine area_B_state_machine;

/* A区使用的变量 */
// 干旱情况; 1：轻微干旱 2：一般干旱 3：严重干旱 对应6个位置
uint8_t area_A_situation[6];
uint8_t area_A_current_position;

/* B区使用的变量 */
// 灌溉顺序;
uint8_t area_B_sequence[4];
// 干旱情况; 0：不干旱 1：轻微干旱 2：一般干旱 3：严重干旱 对应6个位置
uint8_t area_B_situation[6];
uint8_t area_B_current_position;

/* 树莓派传输变量 */
uint8_t usb_rx_buffer[256];
float radar_linear_speed;
float radar_angle_error;

/* Debug */
uint8_t debug_uart_rx_buffer[40];
uint8_t enable_radar_control;

void User_Init(void)
{
    TimeUs_Init();
    User_Task_Init();
    Wheel_Init();
    StateMachine_Init(&main_state_machine, 0, Main_State_Change);
    StateMachine_Init(&area_A_state_machine, STATE_MACHINE_NO_STATE, Main_State_Change);
    StateMachine_Init(&area_B_state_machine, STATE_MACHINE_NO_STATE, Main_State_Change);
    
    Arm_Init();
    Arm_RoughAdjustment(0);

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, debug_uart_rx_buffer, 40);
    enable_radar_control = 0;
}

void User_Update(void)
{
    Task_Update();
    
}

static void State_Change_WithDelay(StateMachine *machine, uint16_t state_id, uint32_t delay)
{
    machine_delay = machine;
    next_state_id_delay = state_id;
    Task_SetRunTick_Delay(task_change_state_delay, delay);
    Task_Awake(task_change_state_delay);
}

void Main_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    switch (state_id)
    {
    case 0:
        break;
    case 1: 
        if (enter_or_exit == STATE_ENTER)
        {
            StateMachine_Change(&area_A_state_machine, 0);
            area_A_current_position = 0;
        }
        break;
    case 2:
        break;
    case 3:
        if (enter_or_exit == STATE_ENTER)
        {
            StateMachine_Change(&area_B_state_machine, 0);
            area_B_current_position = 0;
        }
        break;
    case 6:
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
        // TODO 移动到目标位置的任务
        // State_Change_WithDelay(&area_A_state_machine, 2, 2000);
        break;
    case 2:
        Arm_RoughAdjustment(1);
        State_Change_WithDelay(&area_A_state_machine, 4, 1000);
        break;
    case 3:
        Arm_RoughAdjustment(2);
        State_Change_WithDelay(&area_A_state_machine, 4, 1000);
        break;
    case 4:
        Voice_BroadCast(area_A_situation[area_A_current_position]);
        State_Change_WithDelay(&area_A_state_machine, 5, 1000);
        break;
    case 5:
        pump_spray_time = area_A_situation[area_A_current_position];
        Task_SetRunTick_Current(task_pump_spray);
        Task_Awake(task_pump_spray);
        if (area_A_current_position % 2 == 0)
        {
            State_Change_WithDelay(&area_A_state_machine, 3, 1000 + area_A_situation[area_A_current_position] * 600);
        }
        else
        {
            State_Change_WithDelay(&area_A_state_machine, 6, 1000 + area_A_situation[area_A_current_position] * 600);
        }
        area_A_current_position++;
        break;
    case 6:
        Arm_RoughAdjustment(0);
        if (area_A_current_position < 6)
        {
            State_Change_WithDelay(&area_A_state_machine, 1, 500);
        }
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
        // TODO 移动到目标位置的任务
        // State_Change_WithDelay(&area_B_state_machine, 2, 2000);
        break;
    case 2:
        Arm_RoughAdjustment(1);
        State_Change_WithDelay(&area_B_state_machine, 4, 200);
        break;
    case 3:
        Arm_RoughAdjustment(2);
        State_Change_WithDelay(&area_B_state_machine, 4, 200);
        break;
    case 4:
        Voice_BroadCast(area_B_situation[area_B_current_position]);
        State_Change_WithDelay(&area_B_state_machine, 5, 1000);
        break;
    case 5:
        pump_spray_time = area_B_situation[area_B_current_position];
        Task_SetRunTick_Current(task_pump_spray);
        Task_Awake(task_pump_spray);
        if (area_B_current_position % 2 == 0)
        {
            State_Change_WithDelay(&area_B_state_machine, 3, 1000 + area_B_situation[area_A_current_position] * 600);
        }
        else
        {
            State_Change_WithDelay(&area_B_state_machine, 6, 1000 + area_B_situation[area_A_current_position] * 600);
        }
        area_B_current_position++;
        break;
    case 6:
        Arm_RoughAdjustment(0);
        if (area_B_current_position < 6)
        {
            State_Change_WithDelay(&area_B_state_machine, 1, 500);
        }
        break;
    default:
        break;
    }
}

static void Debug_UARTRx_Operation(void)
{
    switch (debug_uart_rx_buffer[0])
    {
    case 0x10:                              // 轮子直行 后面数据为速度
        float forward_speed = VOFA_BytesToFloatLE(&debug_uart_rx_buffer[1]);
        Wheel_Forward(forward_speed);
        break;
    case 0x11:                              // 轮子转弯 后面数据为速度
        float turn_speed = VOFA_BytesToFloatLE(&debug_uart_rx_buffer[1]);
        Wheel_Turn(turn_speed);
        break;
    case 0x12:                              // 停车
        Wheel_Stop();
        break;         
    case 0x20:
        Pump_Start();
        break;
    case 0x21:
        Pump_Stop();
        break;
    case 0x22:
        pump_spray_time = debug_uart_rx_buffer[1] < 3 ? debug_uart_rx_buffer[1] : 3;
        Task_SetRunTick_Current(task_pump_spray);
        Task_Awake(task_pump_spray);
        break;
    case 0x50:
        if (debug_uart_rx_buffer[1] <= 3 && debug_uart_rx_buffer[1])
        {
            Voice_BroadCast(debug_uart_rx_buffer[1]);
        }
    case 0xA0:
        enable_radar_control = 0;  
        break;        
    case 0xA1:
        enable_radar_control = 1;   
        break;          
    default:
        break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart1)
    {
        Debug_UARTRx_Operation();
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, debug_uart_rx_buffer, 40);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_GPIO_EXTI_Callback(GPIO_Pin);
}

void USB_Receive(void)
{
    if (!enable_radar_control)
    {
        return;
    }
    sscanf((char *)usb_rx_buffer, "L:%f,A:%f", &radar_linear_speed, &radar_angle_error);
    WheelPID_SetSpeeds(
        (radar_linear_speed - radar_angle_error * 3),
        (radar_linear_speed + radar_angle_error * 3),
        (radar_linear_speed - radar_angle_error * 3),
        (radar_linear_speed + radar_angle_error * 3)
    );
}
