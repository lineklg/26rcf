#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "user.h"
#include <stdint.h>

#define WHEEL_TARGET_AXIS_ERROR                     0.025            // m
#define WHEEL_TARGET_ANGLE_MAX_ERROR                0.20            // rad

extern DRV8870_Motor motor_ic[4];  // 0-3:A-D
extern Encoder motor_enc[4];
extern Motor motor[4];
extern ServoPosition arm_servo[3]; // 0-2:yaw first second

void Wheel_Init(void);
void Wheel_Forward(float speed);
void Wheel_Turn(float speed);
void Wheel_Stop(void);
void Wheel_Forward_WithTime(float speed, uint32_t time_ms);
void Wheel_Forward_WithRadar_AxisX(float speed, float route_m);
void Wheel_Stop_WithDelay(uint32_t time_ms);
void Wheel_Turn_WithRadar_Angle(float speed, float angle_rad);

void Voice_BroadCast(uint16_t situation);

void Pump_Start(void);
void Pump_Stop(void);

void Arm_Init(void);
void Arm_RoughAdjustment(uint8_t direction);

#endif // !BEHAVIOR_H