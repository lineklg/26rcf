#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "user.h"
#include <stdint.h>

extern DRV8870_Motor motor_ic[4];  // 0-3:A-D
extern Encoder motor_enc[4];
extern Motor motor[4];
extern ServoPosition arm_servo[3]; // 0-2:yaw first second

void BaseMotor_Init(void);
void BaseMotor_Forward(float speed);
void BaseMotor_Turn(float speed);
void BaseMotor_Stop(void);

void Voice_BroadCast(uint16_t situation);

void Pump_Start(void);
void Pump_Stop(void);

void Arm_Init(void);
void Arm_RoughAdjustment(uint8_t direction);

#endif // !BEHAVIOR_H