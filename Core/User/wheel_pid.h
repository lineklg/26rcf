#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include "DRV8870.h"
#include "motor.h"

#include <stdint.h>

/** @brief 四轮速度环管理的轮子数量。 */
#define WHEEL_PID_COUNT 4U
/** @brief 速度环执行周期，单位为毫秒。 */
#define WHEEL_PID_PERIOD_MS 20U

/**
 * @brief 初始化四轮速度 PID 控制器。
 * @param[in,out] motors 四个车轮电机对象。
 * @param[in,out] drivers 四个车轮驱动对象。
 * @return 无。
 */
void WheelPID_Init(
    Motor motors[WHEEL_PID_COUNT],
    DRV8870_Motor drivers[WHEEL_PID_COUNT]
);

/**
 * @brief 设置四轮同向直行目标速度并启动速度环。
 * @param[in] speed 四个车轮的目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_Forward(float speed);

/**
 * @brief 设置原地转向目标速度并启动速度环。
 * @param[in] speed B、D 轮目标线速度，单位为 m/s，A、C 轮使用其相反数。
 * @return 无。
 */
void WheelPID_Turn(float speed);

/**
 * @brief 停止四轮速度 PID 控制器。
 * @return 无。
 */
void WheelPID_Stop(void);

/**
 * @brief 设置指定车轮的 PID 参数。
 * @param[in] index 车轮索引。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] kd 微分系数。
 * @return 无。
 */
void WheelPID_SetK(uint8_t index, float kp, float ki, float kd);

#endif /* WHEEL_PID_H */
