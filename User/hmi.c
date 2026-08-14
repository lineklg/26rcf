#include "hmi.h"

#include <stddef.h>
#include <stdio.h>

#define HMI_COMMAND_BUFFER_SIZE 128U

/**
 * @brief HMI 指令帧尾。
 */
static const uint8_t HMI_send_tail[] = {0xFFU, 0xFFU, 0xFFU};

/**
 * @brief 通过 UART 发送 HMI 命令主体并追加三个 0xFF 帧尾。
 * @param[in,out] huart UART 句柄。
 * @param[in] data 待发送的命令主体。
 * @param[in] size 命令主体字节数。
 * @return 无。
 */
void HMI_UART_Send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size)
{
    if (huart == NULL || data == NULL || size == 0U)
    {
        return;
    }

    if (HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY) != HAL_OK)
    {
        return;
    }

    (void)HAL_UART_Transmit(
        huart,
        HMI_send_tail,
        (uint16_t)sizeof(HMI_send_tail),
        HAL_MAX_DELAY
    );
}

/**
 * @brief 修改 HMI 文本控件内容。
 * @param[in,out] huart UART 句柄。
 * @param[in] widget 控件名称，例如 `main.t0`。
 * @param[in] txt 待显示文本。
 * @return 无。
 * @note 本函数使用静态缓冲区，不可并发调用；超长命令将被拒绝发送。
 */
void HMI_UART_Send_ModifyTxt(UART_HandleTypeDef *huart, char *widget, char *txt)
{
    static char data[HMI_COMMAND_BUFFER_SIZE];
    int written_size;

    if (huart == NULL || widget == NULL || txt == NULL)
    {
        return;
    }

    written_size = snprintf(data, sizeof(data), "%s.txt=\"%s\"", widget, txt);
    if (written_size < 0 || (size_t)written_size >= sizeof(data))
    {
        return;
    }

    HMI_UART_Send(huart, (const uint8_t *)data, (uint16_t)written_size);
}
