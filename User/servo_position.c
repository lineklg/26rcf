#include "servo_position.h"

static uint8_t ServoPosition_IsValidChannel(uint32_t channel)
{
    return (channel == TIM_CHANNEL_1) ||
           (channel == TIM_CHANNEL_2) ||
           (channel == TIM_CHANNEL_3) ||
           (channel == TIM_CHANNEL_4);
}

/**
 * @brief 检查 PWM 输出类型是否有效。
 * @param[in] output PWM 输出类型。
 * @return 有效时返回非零值，否则返回零。
 */
static uint8_t ServoPosition_IsValidOutput(ServoPositionOutput output)
{
    return (output == SERVO_POSITION_OUTPUT_MAIN) ||
           (output == SERVO_POSITION_OUTPUT_COMPLEMENTARY);
}

static uint8_t ServoPosition_IsReady(const ServoPosition *servo)
{
    return (servo != NULL) &&
           (servo->htim != NULL) &&
           (servo->htim->Instance != NULL) &&
           (servo->initialized != 0U);
}

static void ServoPosition_Clear(ServoPosition *servo)
{
    servo->htim = NULL;
    servo->channel = 0U;
    servo->output = SERVO_POSITION_OUTPUT_MAIN;
    servo->pwmPeriod = 0U;
    servo->minCompare = 0U;
    servo->centerCompare = 0U;
    servo->maxCompare = 0U;
    servo->initialized = 0U;
}

uint32_t ServoPosition_GetCompare(
    uint32_t period,
    float position,
    uint32_t minCompare,
    uint32_t centerCompare,
    uint32_t maxCompare
)
{
    if ((period == 0U) ||
        (minCompare > centerCompare) ||
        (centerCompare > maxCompare) ||
        (maxCompare > period))
    {
        return 0U;
    }

    if (position != position)
    {
        return centerCompare;
    }

    if (position <= -1.0f)
    {
        return minCompare;
    }

    if (position >= 1.0f)
    {
        return maxCompare;
    }

    if (position < 0.0f)
    {
        return centerCompare -
               (uint32_t)((float)(centerCompare - minCompare) * (-position));
    }

    return centerCompare +
           (uint32_t)((float)(maxCompare - centerCompare) * position);
}

HAL_StatusTypeDef ServoPosition_Init(
    ServoPosition *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    ServoPositionOutput output,
    uint32_t minCompare,
    uint32_t centerCompare,
    uint32_t maxCompare
)
{
    if ((servo == NULL) ||
        (htim == NULL) ||
        (htim->Instance == NULL) ||
        !ServoPosition_IsValidChannel(channel) ||
        !ServoPosition_IsValidOutput(output))
    {
        return HAL_ERROR;
    }

    const uint32_t pwmPeriod = __HAL_TIM_GET_AUTORELOAD(htim);
    if ((pwmPeriod == 0U) ||
        (minCompare > centerCompare) ||
        (centerCompare > maxCompare) ||
        (maxCompare > pwmPeriod))
    {
        return HAL_ERROR;
    }

    servo->htim = htim;
    servo->channel = channel;
    servo->output = output;
    servo->pwmPeriod = pwmPeriod;
    servo->minCompare = minCompare;
    servo->centerCompare = centerCompare;
    servo->maxCompare = maxCompare;
    servo->initialized = 0U;

    __HAL_TIM_SET_COMPARE(htim, channel, centerCompare);

    const HAL_StatusTypeDef status =
        (output == SERVO_POSITION_OUTPUT_COMPLEMENTARY)
            ? HAL_TIMEx_PWMN_Start(htim, channel)
            : HAL_TIM_PWM_Start(htim, channel);
    if (status != HAL_OK)
    {
        ServoPosition_Clear(servo);
        return status;
    }

    servo->initialized = 1U;
    return HAL_OK;
}

void ServoPosition_SetPosition(ServoPosition *servo, float position)
{
    if (!ServoPosition_IsReady(servo))
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(
        servo->htim,
        servo->channel,
        ServoPosition_GetCompare(
            servo->pwmPeriod,
            position,
            servo->minCompare,
            servo->centerCompare,
            servo->maxCompare
        )
    );
}

void ServoPosition_Stop(ServoPosition *servo)
{
    if (!ServoPosition_IsReady(servo))
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(
        servo->htim,
        servo->channel,
        servo->centerCompare
    );
}

HAL_StatusTypeDef ServoPosition_DeInit(ServoPosition *servo)
{
    if (!ServoPosition_IsReady(servo))
    {
        return HAL_ERROR;
    }

    ServoPosition_Stop(servo);
    const HAL_StatusTypeDef status =
        (servo->output == SERVO_POSITION_OUTPUT_COMPLEMENTARY)
            ? HAL_TIMEx_PWMN_Stop(servo->htim, servo->channel)
            : HAL_TIM_PWM_Stop(servo->htim, servo->channel);
    ServoPosition_Clear(servo);
    return status;
}
