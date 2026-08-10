#include "DRV8870.h"

#include <limits.h>
#include <stddef.h>

static uint8_t DRV8870_IsValidChannel(uint32_t channel)
{
    return (channel == TIM_CHANNEL_1) ||
           (channel == TIM_CHANNEL_2) ||
           (channel == TIM_CHANNEL_3) ||
           (channel == TIM_CHANNEL_4);
}

static uint8_t DRV8870_IsReady(const DRV8870_Motor *motor)
{
    return (motor != NULL) &&
           (motor->htim != NULL) &&
           (motor->htim->Instance != NULL) &&
           (motor->initialized != 0U);
}

static int32_t DRV8870_GetMaxDuty(const DRV8870_Motor *motor)
{
    return (motor->pwmPeriod > (uint32_t)INT32_MAX)
               ? INT32_MAX
               : (int32_t)motor->pwmPeriod;
}

static void DRV8870_SetChannelDuty(
    DRV8870_Motor *motor,
    uint32_t channel,
    uint32_t duty
)
{
    if (!DRV8870_IsReady(motor)) {
        return;
    }

    if (duty > motor->pwmPeriod) {
        duty = motor->pwmPeriod;
    }

    __HAL_TIM_SET_COMPARE(motor->htim, channel, duty);
}

HAL_StatusTypeDef DRV8870_Motor_Init(
    DRV8870_Motor *motor,
    TIM_HandleTypeDef *htim,
    uint32_t in1Channel,
    uint32_t in2Channel,
    uint8_t reverse
)
{
    if ((motor == NULL) ||
        (htim == NULL) ||
        (htim->Instance == NULL) ||
        !DRV8870_IsValidChannel(in1Channel) ||
        !DRV8870_IsValidChannel(in2Channel) ||
        (in1Channel == in2Channel)) {
        return HAL_ERROR;
    }

    const uint32_t pwmPeriod = __HAL_TIM_GET_AUTORELOAD(htim);
    if (pwmPeriod == 0U) {
        return HAL_ERROR;
    }

    motor->htim = htim;
    motor->in1Channel = in1Channel;
    motor->in2Channel = in2Channel;
    motor->pwmPeriod = pwmPeriod;
    motor->reverse = (reverse != 0U) ? 1U : 0U;
    motor->initialized = 0U;

    __HAL_TIM_SET_COMPARE(htim, in1Channel, 0U);
    __HAL_TIM_SET_COMPARE(htim, in2Channel, 0U);

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim, in1Channel);
    if (status != HAL_OK) {
        motor->htim = NULL;
        return status;
    }

    status = HAL_TIM_PWM_Start(htim, in2Channel);
    if (status != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(htim, in1Channel);
        motor->htim = NULL;
        return status;
    }

    motor->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef DRV8870_Motor_DeInit(DRV8870_Motor *motor)
{
    if (!DRV8870_IsReady(motor)) {
        return HAL_ERROR;
    }

    DRV8870_Coast(motor);

    const HAL_StatusTypeDef in1Status =
        HAL_TIM_PWM_Stop(motor->htim, motor->in1Channel);
    const HAL_StatusTypeDef in2Status =
        HAL_TIM_PWM_Stop(motor->htim, motor->in2Channel);

    motor->initialized = 0U;
    motor->htim = NULL;

    return ((in1Status == HAL_OK) && (in2Status == HAL_OK))
               ? HAL_OK
               : HAL_ERROR;
}

void DRV8870_SetDuty(DRV8870_Motor *motor, int32_t duty)
{
    if (!DRV8870_IsReady(motor)) {
        return;
    }

    const int32_t maxDuty = DRV8870_GetMaxDuty(motor);

    if (duty > maxDuty) {
        duty = maxDuty;
    } else if (duty < -maxDuty) {
        duty = -maxDuty;
    }

    if (motor->reverse != 0U) {
        duty = -duty;
    }

    if (duty > 0) {
        DRV8870_SetChannelDuty(motor, motor->in2Channel, 0U);
        DRV8870_SetChannelDuty(motor, motor->in1Channel, (uint32_t)duty);
    } else if (duty < 0) {
        DRV8870_SetChannelDuty(motor, motor->in1Channel, 0U);
        DRV8870_SetChannelDuty(motor, motor->in2Channel, (uint32_t)(-duty));
    } else {
        DRV8870_Coast(motor);
    }
}

void DRV8870_SetDutyPercent(DRV8870_Motor *motor, float percent)
{
    if (!DRV8870_IsReady(motor)) {
        return;
    }

    const int32_t maxDuty = DRV8870_GetMaxDuty(motor);

    if (percent >= 1.0f) {
        DRV8870_SetDuty(motor, maxDuty);
        return;
    } else if (percent <= -1.0f) {
        DRV8870_SetDuty(motor, -maxDuty);
        return;
    } else if (percent != percent) {
        DRV8870_Coast(motor);
        return;
    }

    const int32_t duty = (int32_t)((float)maxDuty * percent);
    DRV8870_SetDuty(motor, duty);
}

void DRV8870_Coast(DRV8870_Motor *motor)
{
    if (!DRV8870_IsReady(motor)) {
        return;
    }

    DRV8870_SetChannelDuty(motor, motor->in1Channel, 0U);
    DRV8870_SetChannelDuty(motor, motor->in2Channel, 0U);
}

void DRV8870_Brake(DRV8870_Motor *motor)
{
    if (!DRV8870_IsReady(motor)) {
        return;
    }

    DRV8870_SetChannelDuty(motor, motor->in1Channel, motor->pwmPeriod);
    DRV8870_SetChannelDuty(motor, motor->in2Channel, motor->pwmPeriod);
}
