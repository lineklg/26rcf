#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include <stdint.h>

/** @brief 速度到脉冲标定表长度。 */
#define MOTOR_SPEED_TO_PULSE_COUNT 6U

/** @brief 电机输出回调。 */
typedef void (*SetSpeedCallback)(void *device, float speed);
/** @brief 返回微秒时间戳的回调。 */
typedef uint64_t (*GetTimeCallback)(void);

/** @brief 带编码器反馈的电机运动学对象。 */
typedef struct {
    Encoder *enc;                    /**< 编码器。 */
    uint64_t old_time;               /**< 上次计算时间。 */
    uint16_t k;                      /**< 每圈编码器计数。 */
    float l;                         /**< 车轮周长。 */
    float speed_history[3];          /**< 最近三次速度。 */
    float route;                     /**< 累计路程。 */
    float acceleration;              /**< 加速度。 */
    GetTimeCallback time_callback;   /**< 微秒时基。 */
    SetSpeedCallback speed_callback; /**< 输出回调。 */
    void *device;                    /**< 驱动对象。 */
    uint8_t reverse;                 /**< 反向标志。 */
    float speed_to_pulse_table[MOTOR_SPEED_TO_PULSE_COUNT]; /**< 标定表。 */
} Motor;

/** @brief 左右电机组合。 */
typedef struct {
    Motor *left;
    Motor *right;
} Motor_Group;

/**
 * @brief 初始化电机运动学对象。
 * @param time_callback 返回微秒时间戳的回调。
 */
Motor Motor_Init(Encoder *enc, uint16_t k, float l,
                 GetTimeCallback time_callback,
                 SetSpeedCallback speed_callback,
                 void *device, uint8_t reverse);

/** @brief 计算速度，并更新路程、速度历史和加速度。 */
float Motor_CalcSpeed(Motor *motor);

/** @brief 计算速度并返回最近三次速度的中值。 */
float Motor_CalcSpeed_Smooth(Motor *motor);

/** @brief 写入速度到脉冲标定值，index 有效范围为 0 到 5。 */
void Motor_RecordCurrentPulse(Motor *motor, uint16_t index, float pulse);

#endif /* MOTOR_H */
