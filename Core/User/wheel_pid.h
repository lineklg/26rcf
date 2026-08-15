#ifndef WHEEL_PID_H
#define WHEEL_PID_H

#include "DRV8870.h"
#include "motor.h"

#include <stdint.h>

/** @brief 四轮速度环管理的轮子数量。 */
#define WHEEL_PID_COUNT 4U
/** @brief 速度环执行周期，单位为毫秒。 */
#define WHEEL_PID_PERIOD_MS 20U

/** @brief 角度保持使能，只有值为 1 时计算角度 PID。 */
extern uint8_t enable_fix_angle;

/** @brief 雷达位置保持使用的世界坐标轴。 */
typedef enum {
    WHEEL_PID_POSITION_AXIS_X = 0,
    WHEEL_PID_POSITION_AXIS_Y = 1
} WheelPID_PositionAxis;

/** @brief 位置保持使能，只有值为 1 时计算位置 PID。 */
extern uint8_t enable_fix_pos;

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
 * @brief 设置指定车轮的目标线速度并启动速度环。
 * @param[in] index 车轮索引，0 至 3 分别对应 A 至 D 轮。
 * @param[in] speed 目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeed(uint8_t index, float speed);

/**
 * @brief 分别设置 A、B、C、D 四轮目标线速度并启动速度环。
 * @param[in] speed_a A 轮目标线速度，单位为 m/s。
 * @param[in] speed_b B 轮目标线速度，单位为 m/s。
 * @param[in] speed_c C 轮目标线速度，单位为 m/s。
 * @param[in] speed_d D 轮目标线速度，单位为 m/s。
 * @return 无。
 */
void WheelPID_SetSpeeds(
    float speed_a,
    float speed_b,
    float speed_c,
    float speed_d
);

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

/**
 * @brief 设置角度保持目标值。
 * @param[in] target_angle 目标角度，单位为弧度，将归一化到 [-pi, pi]。
 * @return 无。
 */
void WheelPID_SetTargetAngle(float target_angle);

/**
 * @brief 设置单轴位置保持目标。
 * @param[in] axis 要保持的雷达世界坐标轴。
 * @param[in] target_position 目标位置，单位为米。
 * @return 无。
 */
void WheelPID_SetTargetPosition(
    WheelPID_PositionAxis axis,
    float target_position
);

#endif /* WHEEL_PID_H */
