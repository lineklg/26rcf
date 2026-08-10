#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "user.h"
#include <stdint.h>

void BaseMotor_Init(void);
void BaseMotor_Forward(uint16_t speed);
void BaseMotor_Turn(uint8_t direction, uint16_t time, uint16_t speed);

void Voice_BroadCast(uint16_t situation);

void Pump_Start(void);
void Pump_Stop(void);

void Arm_Init(void);
void Arm_RoughAdjustment(uint8_t direction);

#endif // !BEHAVIOR_H