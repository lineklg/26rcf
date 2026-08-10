#ifndef DRV8870_H_
#define DRV8870_H_

#include "main.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t in1Channel;
    uint32_t in2Channel;
    uint32_t pwmPeriod;
    uint8_t reverse;
    uint8_t initialized;
} DRV8870_Motor;

/**
 * @brief 初始化一个 DRV8870 实例，并启动两个 STM32 PWM 通道。
 *
 * 定时器和 GPIO 复用功能必须已通过 STM32CubeMX 配置完成。两个通道必须属于
 * 传入的同一个定时器，并采用高电平有效的 PWM1 模式。占空比范围取自定时器 ARR。
 *
 * @param motor       待初始化的电机实例。
 * @param htim        IN1 和 IN2 共用的 STM32 HAL 定时器句柄。
 * @param in1Channel  与 DRV8870 IN1 相连的 TIM_CHANNEL_x。
 * @param in2Channel  与 DRV8870 IN2 相连的 TIM_CHANNEL_x。
 * @param reverse     非零时反转正转方向映射。
 * @return 成功时返回 HAL_OK，否则返回 HAL_ERROR 或 PWM 启动错误。
 */
HAL_StatusTypeDef DRV8870_Motor_Init(
    DRV8870_Motor *motor,
    TIM_HandleTypeDef *htim,
    uint32_t in1Channel,
    uint32_t in2Channel,
    uint8_t reverse
);

/**
 * @brief 停止电机实例使用的两个 PWM 通道。
 * @return 两个通道均成功停止时返回 HAL_OK。
 */
HAL_StatusTypeDef DRV8870_Motor_DeInit(DRV8870_Motor *motor);

/**
 * @brief 以定时器计数值设置带符号的电机占空比。
 *
 * 正值驱动电机正转，负值驱动电机反转，零值选择滑行模式。输入值将被限制在
 * [-pwmPeriod, pwmPeriod] 范围内。
 */
void DRV8870_SetDuty(DRV8870_Motor *motor, int32_t duty);

/**
 * @brief 以 [-1.0, 1.0] 范围内的比例值设置带符号的电机占空比。
 */
void DRV8870_SetDutyPercent(DRV8870_Motor *motor, float percent);

/**
 * @brief 将 IN1 和 IN2 置为低电平，以选择滑行模式。
 */
void DRV8870_Coast(DRV8870_Motor *motor);

/**
 * @brief 将 IN1 和 IN2 置为高电平，以选择制动模式。
 *
 * 在 PWM1 模式下，两个通道均使用可用的最大比较值。
 */
void DRV8870_Brake(DRV8870_Motor *motor);

#endif /* DRV8870_H_ */
