#include "user.h"
#include "area_b_logic.h"
#include "behavior.h"
#include "hmi.h"
#include "state_machine.h"
#include "stm32h7xx_hal_uart.h"
#include "task.h"
#include <stdio.h>

// 0：空闲/设置 1：A区灌溉 2：移动到B区 3：B区灌溉 6:移动到终点区
StateMachine main_state_machine; 

// 0: 准备 1：移动到下一个灌溉点 2：转向左边 3：转向右边 4：语音播报 5：浇灌 6：回正 7：结束    10: 避障路径 11：避障路径
StateMachine area_A_state_machine;
StateMachine area_B_state_machine;

// 0: 转向 1：移动 2：转向
StateMachine A_to_B_state_machine;

// 0：向左拐弯到40° 1：向右拐弯到40° 2：向前移动0.7m 3：向前移动0.7m 5:向前移动直到回到y=0 6：角度回到0° 7:x轴移动到oa_current_set_target_x
StateMachine A_obstacle_avoidance_state_machine;

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
volatile float radar_get_axis[2];
volatile float radar_get_angle;              // 弧度制
uint8_t rpi_ready;
uint8_t get_radar_data;

float oa_current_set_target_x;

/* 屏幕传输变量 */
uint8_t screen_uart_rx_buffer[40];
char screen_uart_txt_buf[35];
uint8_t screen_set_situation[2];

/* Debug */
uint8_t debug_uart_rx_buffer[40];
// uint8_t enable_radar_control;

float stop_turn_radar_angle;

/**
 * @brief 执行串口急停并永久停机。
 *
 * 停止轮胎、轮速 PID、喷水和用户任务，退出所有状态机后关闭全局中断，
 * 使程序停留在死循环中，不再响应后续控制命令。
 * @return 无返回值。
 */
static void User_EmergencyStop(void)
{
    Wheel_Stop();
    Pump_Stop();
    pump_spray_time = 0U;

    Task_Sleep(task_pump_spray);
    Task_Sleep(task_pump_stop);
    Task_Sleep(task_change_state_delay);
    Task_Sleep(task_wheel_stop_delay);
    Task_Sleep(task_wheel_stop_condition1);

    StateMachine_Change(&main_state_machine, STATE_MACHINE_NO_STATE);
    StateMachine_Change(&area_A_state_machine, STATE_MACHINE_NO_STATE);
    StateMachine_Change(&area_B_state_machine, STATE_MACHINE_NO_STATE);
    StateMachine_Change(&A_to_B_state_machine, STATE_MACHINE_NO_STATE);

    __disable_irq();
    while (1)
    {
        __BKPT();
    }
}

void User_Init(void)
{
    TimeUs_Init();
    User_Task_Init();
    Wheel_Init();
    StateMachine_Init(&main_state_machine, 0, Main_State_Change);
    StateMachine_Init(&area_A_state_machine, STATE_MACHINE_NO_STATE, Area_A_State_Change);
    StateMachine_Init(&area_B_state_machine, STATE_MACHINE_NO_STATE, Area_B_State_Change);
    StateMachine_Init(&A_to_B_state_machine, STATE_MACHINE_NO_STATE, A_to_B_State_Change);
    
    Arm_Init();
    Arm_RoughAdjustment(0);

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, debug_uart_rx_buffer, 40);
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, screen_uart_rx_buffer, 40);

    // enable_radar_control = 1;
    get_radar_data = 0;

    screen_set_situation[0] = 0;
    screen_set_situation[1] = 0;
    rpi_ready = 0;
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
            sprintf(screen_uart_txt_buf, "A: %d%d%d%d%d%d", area_A_situation[0], area_A_situation[1], area_A_situation[2], area_A_situation[3], area_A_situation[4], area_A_situation[5]);
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t0", screen_uart_txt_buf);
            sprintf(screen_uart_txt_buf, "B: %d %d %d %d", area_B_sequence[0], area_B_sequence[1], area_B_sequence[2], area_B_sequence[3]);
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t1", screen_uart_txt_buf);
            sprintf(screen_uart_txt_buf, "   %d %d %d %d", area_B_situation[area_B_sequence[0] - 1], area_B_situation[area_B_sequence[1] - 1], area_B_situation[area_B_sequence[2] - 1], area_B_situation[area_B_sequence[3] - 1]);
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", screen_uart_txt_buf);
            StateMachine_Change(&area_A_state_machine, 0);
            area_A_current_position = 0;
        }
        break;
    case 2:
        if (enter_or_exit == STATE_ENTER)
        {
            StateMachine_Change(&A_to_B_state_machine, 0);
        }
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
    if (enter_or_exit == STATE_ENTER)
    {
        switch (state_id)
        {
        case 0:
            Wheel_Forward_WithRadar_AxisX(0.3, 0.6);
            WheelPID_SetTargetAngle(0.0f);
            enable_fix_angle = 1;
            State_Change_WithDelay(&area_A_state_machine, 2, 2400);
            break;
        case 1:
            Wheel_Forward_WithRadar_AxisX(0.3, 1);
            WheelPID_SetTargetAngle(0.0f);
            enable_fix_angle = 1;
            State_Change_WithDelay(&area_A_state_machine, 2, 3700);
            break;
        case 2:
        case 3:
            Arm_RoughAdjustment(state_id == 2 ? 1 : 2);
            State_Change_WithDelay(&area_A_state_machine, 4, 600);
            break;
        case 4:
            Voice_BroadCast(area_A_situation[area_A_current_position]);
            State_Change_WithDelay(&area_A_state_machine, 5, 1100);
            break;
        case 5:
            pump_spray_time = area_A_situation[area_A_current_position];
            Task_SetRunTick_Current(task_pump_spray);
            Task_Awake(task_pump_spray);
            if (area_A_current_position % 2 == 0)
            {
                State_Change_WithDelay(&area_A_state_machine, 3, 900 + area_A_situation[area_A_current_position] * 750);
            }
            else
            {
                State_Change_WithDelay(&area_A_state_machine, 6, 900 + area_A_situation[area_A_current_position] * 750);
            }
            area_A_current_position++;
            break;
        case 6:
            Arm_RoughAdjustment(0);
            if (area_A_current_position < 6)
            {
                State_Change_WithDelay(&area_A_state_machine, 1, 700);
            }
            else
            {
                State_Change_WithDelay(&area_A_state_machine, 7, 700);
            }
            break;
        case 7:
            Wheel_Forward_WithRadar_AxisX(0.3, 0.6);
            WheelPID_SetTargetAngle(0.0f);
            enable_fix_angle = 1;
            State_Change_WithDelay(&main_state_machine, 2, 2600);
            StateMachine_Change(&area_A_state_machine, STATE_MACHINE_NO_STATE);
            break;
        default:
            break;
        }
    }
}

static void Area_B_Advance_Position(const AreaBPositionDecision *decision,
                                    uint32_t delay)
{
    area_B_current_position = decision->next_position;
    if (delay == 0U)
    {
        StateMachine_Change(&area_B_state_machine, decision->next_state);
    }
    else
    {
        State_Change_WithDelay(&area_B_state_machine,
                               decision->next_state,
                               delay);
    }
}

void Area_B_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    if (enter_or_exit == STATE_ENTER)
    {
        switch (state_id)
        {
        case 0:
            Wheel_Forward_WithRadar_AxisX(0.3, -0.65);
            WheelPID_SetTargetAngle(M_PI);
            enable_fix_angle = 1;
            State_Change_WithDelay(&area_B_state_machine, 2, 2600);
            break;
        case 1:
            Wheel_Forward_WithRadar_AxisX(0.45, -0.8);
            WheelPID_SetTargetAngle(M_PI);
            enable_fix_angle = 1;
            State_Change_WithDelay(&area_B_state_machine, 2, 2100);
            break;
        case 2:
        case 3:
        {
            AreaBPositionDecision decision = AreaB_DecidePosition(
                area_B_current_position,
                area_B_situation[area_B_current_position]
            );
            if (!decision.should_irrigate)
            {
                Area_B_Advance_Position(&decision, 0U);
                break;
            }
            Arm_RoughAdjustment(state_id == 2U ? 3U : 4U);
            State_Change_WithDelay(&area_B_state_machine, 4, 1000);
            break;
        }
        case 4:
            Voice_BroadCast(area_B_situation[area_B_current_position]);
            State_Change_WithDelay(&area_B_state_machine, 5, 1100);
            break;
        case 5:
        {
            uint8_t situation = area_B_situation[area_B_current_position];
            AreaBPositionDecision decision = AreaB_DecidePosition(
                area_B_current_position,
                situation
            );
            pump_spray_time = situation;
            Task_SetRunTick_Current(task_pump_spray);
            Task_Awake(task_pump_spray);
            Area_B_Advance_Position(&decision, 900U + situation * 750U);
            break;
        }
        case 6:
            Arm_RoughAdjustment(0);
            if (area_B_current_position < 6)
            {
                State_Change_WithDelay(&area_B_state_machine, 1, 500);
            }
            else
            {
                State_Change_WithDelay(&area_B_state_machine, 7, 500);
            }
            break;
        case 7:
            Wheel_Forward_WithRadar_AxisX(0.45, -0.8);
            WheelPID_SetTargetAngle(M_PI);
            enable_fix_angle = 1;
            State_Change_WithDelay(&main_state_machine, 6, 2100);
            StateMachine_Change(&area_B_state_machine, STATE_MACHINE_NO_STATE);
            break;
        default:
            break;
        }
    }
    
}

void A_to_B_State_Change(uint16_t state_id, uint8_t enter_or_exit)
{
    if (enter_or_exit == STATE_ENTER)
    {
        switch (state_id)
        {
        case 0:
            Wheel_Turn_WithRadar_Angle(0.3, -M_PI_2);
            State_Change_WithDelay(&A_to_B_state_machine, 1, 1500);
            break;
        case 1:
            Wheel_Forward_WithRadar_AxisY(0.35, -1.34);
            WheelPID_SetTargetAngle(-M_PI_2);
            enable_fix_angle = 1;
            State_Change_WithDelay(&A_to_B_state_machine, 2, 4500);
            break;
        case 2:
            Wheel_Turn_WithRadar_Angle(0.3, -M_PI_2);
            State_Change_WithDelay(&main_state_machine, 3, 1500);
            StateMachine_Change(&A_to_B_state_machine, STATE_MACHINE_NO_STATE);
            break;
        default:
            break;
        }
    }
}

// static uint8_t Task_Wheel_Stop_Condition()
// {
//     if (fabsf(stop_turn_radar_angle - radar_get_angle) * 8.0f < M_PI)
//     {
//         return 1;
//     }
//     else
//     {
//         return 0;
//     }
// }

static void Screen_UARTRx_Operation(void)
{
    if (screen_uart_rx_buffer[0] != '#')
    {
        return;
    }
    switch (screen_uart_rx_buffer[1])
    {
    case 'a':                               // A区的数据
        for (uint16_t i = 0; i < 6; i++)
        {
            area_A_situation[i] = screen_uart_rx_buffer[2 + i] - '0';
            
        }
        screen_set_situation[0] = 1;
        break;
    case 'b':                               // B区的数据
        for (uint16_t i = 0; i < 4; i++)
        {
            area_B_sequence[i] = screen_uart_rx_buffer[2 + i] - '0';
        }
        for (uint16_t i = 0; i < 6; i++)
        {
            area_B_situation[i] = 0;
        }
        for (uint16_t i = 0; i < 4; i++)
        {
            area_B_situation[area_B_sequence[i] - 1] = screen_uart_rx_buffer[6 + i] - '0';
        }
        screen_set_situation[1] = 1;
        break;
    case 's':
        if (screen_set_situation[0] && 
            screen_set_situation[1] && 
            get_radar_data &&
            rpi_ready)
        {
            StateMachine_Change(&main_state_machine, 1);
            screen_set_situation[0] = 0;
            screen_set_situation[1] = 0;
        }
        else
        {
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t0", "");
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t1", "");
            HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", "");
            if (!screen_set_situation[0])
            {
                HMI_UART_Send_ModifyTxt(&huart3, "page5.t0", "A Not Set!");
            }
            if (!screen_set_situation[1])
            {
                HMI_UART_Send_ModifyTxt(&huart3, "page5.t1", "B Not Set!");
            }
            if (!get_radar_data)
            {
                HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", "Radar Not Ready");
            }
            if (!rpi_ready)
            {
                HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", "RPI Not Ready");
            }
        }
        break;
    case 'e':
        break;
    case 'r':
        CDC_Transmit_HS((uint8_t *)"START", 6);
        break;
    default:
        break;
    }
}

static void Debug_UARTRx_Operation(void)
{
    switch (debug_uart_rx_buffer[0])
    {
    /* 轮子 */
    case 0x10:                              // 直行 后面数据为速度
        float forward_speed = VOFA_BytesToFloatLE(&debug_uart_rx_buffer[1]);
        Wheel_Forward(forward_speed);
        break;
    case 0x11:                              // 转弯 后面数据为速度
        float turn_speed = VOFA_BytesToFloatLE(&debug_uart_rx_buffer[1]);
        Wheel_Turn(turn_speed);
        break;
    case 0x12:                              // 停车
        Wheel_Stop();
        break;
    case 0x13:                              // 转弯开环 后面数据依次为速度和时间
        Wheel_Turn(((float)debug_uart_rx_buffer[1] / 256.0 - 0.5) * 0.8);
        Wheel_Stop_WithDelay(((uint16_t)debug_uart_rx_buffer[2] << 8) + ((uint16_t)debug_uart_rx_buffer[3])); 
        break;
    case 0x14:                              // 转弯 基于激光雷达给出的角度 后面数据为速度 目前只能达到最终右转90°的效果
        // Wheel_Turn(((float)debug_uart_rx_buffer[1] / 256.0 - 0.5) * 0.8);
        // stop_turn_radar_angle = radar_get_angle - M_PI_2 + 0.065;
        // if (stop_turn_radar_angle < - M_PI)
        // {
        //     stop_turn_radar_angle += 2 * M_PI;
        // }
        // Task_SetExtraData(task_wheel_stop_condition1, (Task_ExtraData){.condition = Task_Wheel_Stop_Condition});
        // Task_Awake(task_wheel_stop_condition1);
        break;
    case 0x16:                              // 基于激光雷达给出的距离移动 后面数据依次为 速度(/ 256.0 * 0.4) 距离(/64.0)
        Wheel_Forward_WithRadar_AxisX(((float)debug_uart_rx_buffer[1] / 256.0 - 0.5) * 0.8, debug_uart_rx_buffer[2] / 64.0);
        break;
    case 0x17:                              // 基于激光雷达给出的距离移动 后面数据依次为 速度 距离 并启用角度校准 角度
        Wheel_Forward_WithRadar_AxisX(((float)debug_uart_rx_buffer[1] / 256.0 - 0.5) * 1.6, debug_uart_rx_buffer[2] / 64.0);
        WheelPID_SetTargetAngle(((float)debug_uart_rx_buffer[3] / 256.0 - 0.5) * M_PI * 2);
        enable_fix_angle = 1;
        break;
    /* 水泵 */
    case 0x20:                              // 开启
        Pump_Start();
        break;
    case 0x21:                              // 关闭
        Pump_Stop();
        break;
    case 0x22:                              // 根据次数喷水 最多3次
        pump_spray_time = debug_uart_rx_buffer[1] < 3 ? debug_uart_rx_buffer[1] : 3;
        Task_SetRunTick_Current(task_pump_spray);
        Task_Awake(task_pump_spray);
        break;
    /* 语音播报 */
    case 0x50:
        if (debug_uart_rx_buffer[1] <= 3 && debug_uart_rx_buffer[1])
        {
            Voice_BroadCast(debug_uart_rx_buffer[1]);
        }
        break;
    /* 机械臂 */
    case 0x60:
        Arm_RoughAdjustment(0);
        break;
    case 0x61:
        Arm_RoughAdjustment(1);
        break;
    case 0x62:
        Arm_RoughAdjustment(2);
        break;   
    case 0x63:
        Arm_RoughAdjustment(3);
        break;
    case 0x64:
        Arm_RoughAdjustment(4);
        break;   
    case 0x65:                              // 后面数据第一位为控制的舵机，第二位为位置（0-FF,80为中位）
        if (debug_uart_rx_buffer[1] < 3)
        {
            ServoPosition_SetPosition(&arm_servo[debug_uart_rx_buffer[1]], ((float)debug_uart_rx_buffer[2] / 256.0 - 0.5) * 2.0);
        }
        break;
    /* 屏幕模拟 */ 
    case 0x80:
        sprintf((char *)screen_uart_rx_buffer, "#a123123");
        Screen_UARTRx_Operation();
        break;
    case 0x81:
        sprintf((char *)screen_uart_rx_buffer, "#b12463212");
        Screen_UARTRx_Operation();
        break;
    case 0x82:
        sprintf((char *)screen_uart_rx_buffer, "#s");
        Screen_UARTRx_Operation();
        break;
    /* Deperated 树莓派控制 */
    // case 0xA0:
    //     enable_radar_control = 0;  
    //     break;        
    // case 0xA1:
    //     enable_radar_control = 1;   
    //     break;   
    /* 串口控制急停 */
    case 0xF0:
        User_EmergencyStop();
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
    if (huart == &huart3)
    {
        Screen_UARTRx_Operation();
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, screen_uart_rx_buffer, 40);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_GPIO_EXTI_Callback(GPIO_Pin);
}

void USB_Receive(void)
{
    // if (!enable_radar_control)
    // {
    //     return;
    // }
    // if (!strcmp((char *)usb_rx_buffer, "READY"))
    // {
    //     rpi_ready = 1;
    //     HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", "RPI Ready");
    //     return;
    // }
    // if (!get_radar_data)
    // {
    //     get_radar_data = 1;
    //     HMI_UART_Send_ModifyTxt(&huart3, "page5.t2", "Radar Ready");
    // }
    // sscanf((char *)usb_rx_buffer, "X:%f,Y:%f,A:%f", &radar_get_axis[0], &radar_get_axis[1], &radar_get_angle);
    sscanf((char *)usb_rx_buffer, "L:%f,A:%f", &radar_linear_speed, &radar_angle_error);
    WheelPID_SetSpeeds(
        (radar_linear_speed - radar_angle_error * 1),
        (radar_linear_speed + radar_angle_error * 1),
        (radar_linear_speed - radar_angle_error * 1),
        (radar_linear_speed + radar_angle_error * 1)
    );
}
