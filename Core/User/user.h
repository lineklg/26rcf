#ifndef USER_H
#define USER_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "SYN6288.h"
#include "task.h"
#include "DRV8870.h"
#include "pid.h"
#include "state_machine.h"
#include "servo_position.h"
#include "encoder.h"
#include "motor.h"
#include "wheel_pid.h"
#include "behavior.h"
#include "time_us.h"
#include "usb_fs_vpc.h"

void User_Init(void);
void User_Update(void);

void Main_State_Change(uint16_t state_id, uint8_t enter_or_exit);
void Area_A_State_Change(uint16_t state_id, uint8_t enter_or_exit);
void Area_B_State_Change(uint16_t state_id, uint8_t enter_or_exit);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif // !USER_H
