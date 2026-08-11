#ifndef VOFA_H
#define VOFA_H

#include "main.h"

/**
 * @brief 通过 UART 以 VOFA+ JustFloat 格式发送浮点数据。
 *
 * 数据按 little-endian 浮点数组发送，并追加 JustFloat 规定的四字节帧尾
 * ``00 00 80 7F``。发送过程为阻塞式调用。
 *
 * @param huart UART 外设句柄。
 * @param data 待发送的浮点数据数组。
 * @param channel_num 浮点数据通道数量。
 * @return HAL_OK 表示发送成功；否则返回 HAL 串口发送状态。
 */
HAL_StatusTypeDef VOFA_JustFloat_UART_Send(UART_HandleTypeDef *huart,
                                           const float *data,
                                           uint8_t channel_num);

#endif /* VOFA_H */
