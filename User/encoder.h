#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/** @brief 最多管理的中断式编码器数量。 */
#define ENCODER_MAX_COUNT 4U
/** @brief 软件轮询最小周期，单位为毫秒。 */
#define ENCODER_USI_UPDATE_PERIOD 3U

/** @brief GPIO 端口的不透明指针。 */
typedef void *GPIO_Port;

/** @brief GPIO 端口与引脚组合。 */
typedef struct {
    GPIO_Port port; /**< GPIOx。 */
    uint16_t pin;   /**< GPIO_PIN_x。 */
} GPIO_Pin;

/** @brief 双相增量编码器。 */
typedef struct {
    GPIO_Pin pin_A;              /**< A 相 GPIO。 */
    GPIO_Pin pin_B;              /**< B 相 GPIO。 */
    volatile int32_t counter;    /**< 累计计数。 */
    int32_t last_get_counter;    /**< 上次读取计数。 */
    uint16_t prescaler;          /**< 预分频值。 */
    int32_t prescaler_counter;   /**< 尚未输出的余数。 */
} Encoder;

/** @brief 软件轮询式编码器。 */
typedef struct {
    Encoder base_enc;              /**< 通用计数数据。 */
    GPIO_Pin pin_A;                 /**< A 相 GPIO。 */
    GPIO_Pin pin_B;                 /**< B 相 GPIO。 */
    uint8_t last_AB;                /**< 上一次 AB 状态。 */
    int8_t last_direction;          /**< 上一次有效方向。 */
    uint32_t last_update_timestamp; /**< 上一次更新时间。 */
} Encoder_USI;

/**
 * @brief 创建中断式编码器。
 * @param encoder 编码器对象。
 * @param pin_A A 相 GPIO。
 * @param pin_B B 相 GPIO。
 */
void Encoder_Create_UsePin(Encoder *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B);

/**
 * @brief 获取自上次读取以来经过预分频的计数变化。
 * @return 有符号计数变化；对象为空时返回 0。
 */
int32_t Encoder_GetChange(Encoder *encoder);

/** @brief 设置预分频值，实际除数为 prescaler + 1。 */
void Encoder_SetPrescaler(Encoder *encoder, uint16_t prescaler);

/** @brief 创建软件轮询式编码器。 */
void Encoder_USI_Create(Encoder_USI *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B);

/** @brief 按四状态转移表更新软件轮询式编码器。 */
void Encoder_USI_Update(Encoder_USI *encoder);

/**
 * @brief 转发 STM32 的 GPIO EXTI 回调。
 * @param gpio_pin HAL 回调传入的 GPIO_PIN_x。
 */
void Encoder_GPIO_EXTI_Callback(uint16_t gpio_pin);

#endif /* ENCODER_H */
