#ifndef _HMI_H_
#define _HMI_H_

#include "main.h"

/**
 * @brief 通过 UART 发送 HMI 命令主体并追加三个 0xFF 帧尾。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送的命令主体。
 * @param[in] size 命令主体字节数。
 * @return 无。
 */
void HMI_UART_Send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);

/**
 * @brief 修改 HMI 文本控件内容。
 * @param[in,out] huart UART 句柄。
 * @param[in] widget 控件名称，例如 `main.t0`。
 * @param[in] txt 待显示文本。
 * @return 无。
 * @note 本函数使用静态缓冲区，不可并发调用；超长命令将被拒绝发送。
 */
void HMI_UART_Send_ModifyTxt(UART_HandleTypeDef *huart, char *widget, char *txt);

#endif // !_HMI_H_
