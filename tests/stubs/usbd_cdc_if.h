#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U
} HAL_StatusTypeDef;

#define USBD_OK 0U
#define USBD_BUSY 1U

/**
 * @brief 提供主机测试使用的 USB CDC 发送接口。
 * @param[in] buffer 待发送数据。
 * @param[in] length 待发送字节数。
 * @return 模拟的 USB 设备状态。
 */
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t length);

#endif /* TEST_USBD_CDC_IF_H */
