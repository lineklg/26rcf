#ifndef _USB_FS_VPC_H
#define _USB_FS_VPC_H

#include "usbd_cdc_if.h"

/** @brief USB 接收环形缓冲区容量，单位为字节。 */
#define USB_RX_BUFFER_MAX_SIZE 512U

/**
 * @brief 通过 USB 虚拟串口发送数据。
 * @param[in] data 待发送数据。
 * @param[in] len 待发送字节数。
 * @return 发送请求对应的 HAL 状态。
 */
HAL_StatusTypeDef USB_VPC_Send(const uint8_t *data, uint32_t len);

/**
 * @brief 将一包数据写入 USB 接收环形缓冲区。
 * @param[in] data 待写入数据。
 * @param[in] len 待写入字节数。
 * @return 写入成功时返回 len；空间不足或参数无效时返回 0。
 */
uint32_t USB_VPC_RxWrite(const uint8_t *data, uint32_t len);

/**
 * @brief 从 USB 接收环形缓冲区读取数据。
 * @param[out] data 接收数据的目标缓冲区。
 * @param[in] len 期望读取的最大字节数。
 * @return 实际读取字节数；无数据或参数无效时返回 0。
 */
uint32_t USB_VPC_RxRead(uint8_t *data, uint32_t len);

#endif /* _USB_FS_VPC_H */
