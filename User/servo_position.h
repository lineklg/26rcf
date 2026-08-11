#ifndef SERVO_POSITION_H_
#define SERVO_POSITION_H_

#include "main.h"

/**
 * @brief 位置舵机使用的 PWM 输出类型。
 */
typedef enum
{
    SERVO_POSITION_OUTPUT_MAIN = 0U,          /**< 普通 PWM 输出。 */
    SERVO_POSITION_OUTPUT_COMPLEMENTARY = 1U /**< 互补 PWM 输出。 */
} ServoPositionOutput;

/**
 * @brief 位置舵机控制器。
 *
 * 位置舵机通过 PWM 脉宽表示目标角度。position 使用 -1.0f 到 1.0f 的归一化
 * 表示：-1.0f 对应最小位置，0.0f 对应中位，1.0f 对应最大位置。
 * 比较值必须使用与定时器计数器相同的单位。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;    /**< PWM 定时器句柄。 */
    uint32_t channel;           /**< PWM 通道。 */
    ServoPositionOutput output; /**< PWM 输出类型。 */
    uint32_t pwmPeriod;         /**< 定时器自动重装载值（ARR）。 */
    uint32_t minCompare;        /**< 最小位置对应的比较值。 */
    uint32_t centerCompare;     /**< 中位位置对应的比较值。 */
    uint32_t maxCompare;        /**< 最大位置对应的比较值。 */
    uint8_t initialized;        /**< 非零表示已经成功初始化并启动 PWM。 */
} ServoPosition;

/**
 * @brief 初始化位置舵机并启动指定的 PWM 通道。
 *
 * 定时器和 GPIO 必须已经通过 STM32CubeMX 完成配置。初始化时先输出中位
 * 脉宽，再启动 PWM，避免启动瞬间产生非预期动作。
 *
 * @param[out] servo 待初始化的位置舵机实例，不能为 NULL。
 * @param[in] htim 已完成 PWM 配置的定时器句柄，不能为 NULL。
 * @param[in] channel 要使用的 PWM 通道（TIM_CHANNEL_1 到 TIM_CHANNEL_4）。
 * @param[in] output PWM 输出类型。
 * @param[in] minCompare 最小位置对应的比较值。
 * @param[in] centerCompare 中位位置对应的比较值。
 * @param[in] maxCompare 最大位置对应的比较值。
 * @return HAL_OK 初始化成功；HAL_ERROR 参数无效；其他值表示 PWM 启动失败。
 */
HAL_StatusTypeDef ServoPosition_Init(
    ServoPosition *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    ServoPositionOutput output,
    uint32_t minCompare,
    uint32_t centerCompare,
    uint32_t maxCompare
);

/**
 * @brief 设置位置舵机的目标位置。
 *
 * position 会被限制在 -1.0f 到 1.0f；NaN 输入按中位处理。函数只修改
 * 比较值，不会启动或停止 PWM 通道。
 *
 * @param[in,out] servo 已成功初始化的位置舵机实例。
 * @param[in] position 归一化目标位置，-1.0f 为最小位置，1.0f 为最大位置。
 */
void ServoPosition_SetPosition(ServoPosition *servo, float position);

/**
 * @brief 让位置舵机回到中位，但保持 PWM 通道运行。
 *
 * @param[in,out] servo 已成功初始化的位置舵机实例。
 */
void ServoPosition_Stop(ServoPosition *servo);

/**
 * @brief 停止位置舵机 PWM 并清理实例状态。
 *
 * 函数会先输出中位脉宽，再停止 PWM 通道。反初始化后如需再次使用，必须
 * 重新调用 ServoPosition_Init()。
 *
 * @param[in,out] servo 已成功初始化的位置舵机实例。
 * @return HAL_OK 关闭成功；HAL_ERROR 表示实例未初始化；其他值表示 PWM 停止失败。
 */
HAL_StatusTypeDef ServoPosition_DeInit(ServoPosition *servo);

/**
 * @brief 将归一化位置换算为定时器比较值。
 *
 * 这是无硬件副作用的换算函数。参数不满足 minCompare <= centerCompare <=
 * maxCompare 或比较值超出定时器周期时返回 0；NaN 位置返回中位比较值。
 *
 * @param[in] period 定时器自动重装载值（ARR）。
 * @param[in] position 归一化目标位置，范围为 -1.0f 到 1.0f。
 * @param[in] minCompare 最小位置对应的比较值。
 * @param[in] centerCompare 中位位置对应的比较值。
 * @param[in] maxCompare 最大位置对应的比较值。
 * @return 对应的定时器比较值；参数无效时返回 0。
 */
uint32_t ServoPosition_GetCompare(
    uint32_t period,
    float position,
    uint32_t minCompare,
    uint32_t centerCompare,
    uint32_t maxCompare
);

#endif /* SERVO_POSITION_H_ */
