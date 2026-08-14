#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t levels;
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;

extern CoreDebug_Type *CoreDebug;
extern DWT_Type *DWT;
extern uint32_t SystemCoreClock;

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24U)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0U)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
uint32_t HAL_GetTick(void);

/**
 * @brief HAL 函数状态。
 */
typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U
} HAL_StatusTypeDef;

#define TEST_HAL_STATUS_DEFINED 1

/**
 * @brief 主机测试使用的最小 UART 句柄。
 */
typedef struct {
    uint32_t instance;
} UART_HandleTypeDef;

/**
 * @brief 主机测试使用的最小定时器寄存器集合。
 */
typedef struct {
    volatile uint32_t ARR;
    volatile uint32_t CCR[4];
} TIM_TypeDef;

/**
 * @brief 主机测试使用的最小定时器句柄。
 */
typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U

/**
 * @brief 在主机测试定时器中写入比较值。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @param[in] compare 待写入的比较值。
 */
void Test_HAL_TIM_SetCompare(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t compare
);

#define __HAL_TIM_GET_AUTORELOAD(__HANDLE__) ((__HANDLE__)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    Test_HAL_TIM_SetCompare((__HANDLE__), (__CHANNEL__), (__COMPARE__))

/**
 * @brief 启动普通 PWM 输出的测试桩。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 停止普通 PWM 输出的测试桩。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 启动互补 PWM 输出的测试桩。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 停止互补 PWM 输出的测试桩。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t channel);

#endif
