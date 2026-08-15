#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "user.h"
#include <stdint.h>

#define WHEEL_TARGET_AXIS_ERROR                     0.025            // m
#define WHEEL_TARGET_ANGLE_MAX_ERROR                0.160            // rad

extern DRV8870_Motor motor_ic[4];  // 0-3:A-D
extern Encoder motor_enc[4];
extern Motor motor[4];
extern ServoPosition arm_servo[3]; // 0-2:yaw first second

void Wheel_Init(void);
void Wheel_Forward(float speed);
void Wheel_Turn(float speed);
void Wheel_Stop(void);
void Wheel_Forward_WithTime(float speed, uint32_t time_ms);

/**
 * @brief 按雷达 X 轴有符号位移执行直行，并在到位后自动停车。
 * @param[in] speed 车轮目标速度，符号仅表示实际运动速度。
 * @param[in] route_m 沿雷达 X 轴的有符号位移，正值向 X 正方向移动，负值向 X 负方向移动。
 * @return 无。
 */
void Wheel_Forward_WithRadar_AxisX(float speed, float route_m);
void Wheel_Forward_WithRadar_AxisX_ABS(float speed, float pos);

void Wheel_Forward_WithRadar_AxisY(float speed, float route_m);
void Wheel_Forward_WithRadar_AxisY_ABS(float speed, float pos);

/**
 * @brief 按调用瞬间的雷达角度执行斜向直行，并在到位后自动停车。
 * @param[in] speed 车轮目标速度，符号仅表示实际运动速度。
 * @param[in] route_m 沿调用瞬间朝向的有符号位移，单位为米。
 * @return 无。
 */
void Wheel_Forward_WithRadar_CurrentAngle(float speed, float route_m);

void Wheel_Stop_WithDelay(uint32_t time_ms);

/**
 * @brief 按雷达当前角度执行指定相对角度的原地转向，并在到位后自动停车。
 * @param[in] speed 原地转向速度幅值，符号不会影响最终转向方向。
 * @param[in] angle_rad 相对转向角度，单位为弧度。
 * @return 无。
 */
void Wheel_Turn_WithRadar_Angle(float speed, float angle_rad);
void Wheel_Turn_WithRadar_Angle_ABS(float speed, float angle_rad);

void Voice_BroadCast(uint16_t situation);

void Pump_Start(void);
void Pump_Stop(void);

void Arm_Init(void);
void Arm_RoughAdjustment(uint8_t direction);

#endif // !BEHAVIOR_H
