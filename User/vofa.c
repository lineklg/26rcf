#include "vofa.h"
#include <string.h>

#define VOFA_MAX_CHANNELS 255U

/**
 * @brief VOFA+ JustFloat 帧尾：四字节 little-endian 浮点数 0x7F800000。
 */
static const uint8_t vofa_just_float_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

HAL_StatusTypeDef VOFA_JustFloat_UART_Send(UART_HandleTypeDef *huart,
                                           const float *data,
                                           uint8_t channel_num)
{
    uint16_t data_size = (uint16_t)channel_num * (uint16_t)sizeof(float);
    uint8_t frame[VOFA_MAX_CHANNELS * sizeof(float) + sizeof(vofa_just_float_tail)];

    if (huart == NULL || (data == NULL && channel_num != 0U))
    {
        return HAL_ERROR;
    }

    if (data_size > 0U)
    {
        memcpy(frame, data, data_size);
    }
    memcpy(&frame[data_size], vofa_just_float_tail, sizeof(vofa_just_float_tail));

    return HAL_UART_Transmit(huart,
                             frame,
                             (uint16_t)(data_size + sizeof(vofa_just_float_tail)),
                             HAL_MAX_DELAY);
}


float VOFA_BytesToFloatLE(const uint8_t *data)
{
    uint32_t bits;
    float value;

    if (data == NULL)
    {
        return 0.0f;
    }

    bits = (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
    memcpy(&value, &bits, sizeof(value));
    return value;
}