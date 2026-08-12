#include "motor.h"
#include <stddef.h>

static float Motor_MedianOfThree(float a, float b, float c)
{
    float temp;

    if (a > b) { temp = a; a = b; b = temp; }
    if (b > c) { temp = b; b = c; c = temp; }
    if (a > b) { temp = a; a = b; b = temp; }
    return b;
}

Motor Motor_Init(Encoder *enc, uint16_t k, float l,
                 GetTimeCallback time_callback,
                 SetSpeedCallback speed_callback,
                 void *device, uint8_t reverse)
{
    Motor motor = {
        .enc = enc, .k = k, .l = l,
        .time_callback = time_callback,
        .speed_callback = speed_callback,
        .device = device,
        .reverse = reverse != 0U
    };

    motor.old_time = time_callback != NULL ? time_callback() : 0U;
    return motor;
}

float Motor_CalcSpeed(Motor *motor)
{
    uint64_t now;
    uint64_t delta_time;
    int32_t delta;
    float previous_speed;
    float distance;
    float speed;

    if (motor == NULL || motor->enc == NULL ||
        motor->time_callback == NULL || motor->k == 0U) {
        return 0.0f;
    }

    now = motor->time_callback();
    delta_time = now - motor->old_time;
    if (delta_time == 0U) {
        return 0.0f;
    }

    motor->old_time = now;
    delta = Encoder_GetChange(motor->enc);
    previous_speed = motor->speed_history[0];
    distance = (float)delta / (float)motor->k * motor->l *
               (motor->reverse ? -1.0f : 1.0f);
    speed = distance * 1000.0f / (float)delta_time;

    motor->speed_history[2] = motor->speed_history[1];
    motor->speed_history[1] = previous_speed;
    motor->speed_history[0] = speed;
    motor->route += distance;
    motor->acceleration = (speed - previous_speed) * 1000.0f /
                          (float)delta_time;
    return speed;
}

float Motor_CalcSpeed_Smooth(Motor *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }
    (void)Motor_CalcSpeed(motor);
    return Motor_MedianOfThree(motor->speed_history[0],
                               motor->speed_history[1],
                               motor->speed_history[2]);
}

void Motor_RecordCurrentPulse(Motor *motor, uint16_t index, float pulse)
{
    if (motor != NULL && index < MOTOR_SPEED_TO_PULSE_COUNT) {
        motor->speed_to_pulse_table[index] = pulse;
    }
}
