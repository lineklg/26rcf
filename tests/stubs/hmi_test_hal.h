#ifndef HMI_TEST_HAL_H
#define HMI_TEST_HAL_H

#include "main.h"

#define HAL_MAX_DELAY 0xFFFFFFFFU

/**
 * @brief 主机测试使用的阻塞式 UART 发送桩。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送数据。
 * @param[in] size 数据长度。
 * @param[in] timeout 超时时间。
 * @return HAL 状态。
 */
HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout
);

#endif
