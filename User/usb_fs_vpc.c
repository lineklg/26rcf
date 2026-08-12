#include "usb_fs_vpc.h"

#include <stddef.h>
#include <string.h>

static uint8_t usb_rx_buffer[USB_RX_BUFFER_MAX_SIZE];
static uint32_t usb_rx_read_pos;
static uint32_t usb_rx_write_pos;

/**
 * @brief 通过 USB 虚拟串口发送数据。
 * @param[in] data 待发送数据。
 * @param[in] len 待发送字节数。
 * @return 发送请求对应的 HAL 状态。
 */
HAL_StatusTypeDef USB_VPC_Send(const uint8_t *data, uint32_t len)
{
    uint8_t result;

    result = CDC_Transmit_HS((uint8_t *)data, len);

    if (result == USBD_BUSY)
    {
        return HAL_BUSY;
    }
    return HAL_OK;
}

/**
 * @brief 将一包数据写入 USB 接收环形缓冲区。
 * @param[in] data 待写入数据。
 * @param[in] len 待写入字节数。
 * @return 写入成功时返回 len；空间不足或参数无效时返回 0。
 */
uint32_t USB_VPC_RxWrite(const uint8_t *data, uint32_t len)
{
    uint32_t read_pos;
    uint32_t write_pos;
    uint32_t used;
    uint32_t offset;
    uint32_t first_len;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    write_pos = __atomic_load_n(&usb_rx_write_pos, __ATOMIC_RELAXED);
    read_pos = __atomic_load_n(&usb_rx_read_pos, __ATOMIC_ACQUIRE);
    used = write_pos - read_pos;
    if (len > (USB_RX_BUFFER_MAX_SIZE - used)) {
        return 0U;
    }

    offset = write_pos % USB_RX_BUFFER_MAX_SIZE;
    first_len = USB_RX_BUFFER_MAX_SIZE - offset;
    if (first_len > len) {
        first_len = len;
    }
    memcpy(&usb_rx_buffer[offset], data, first_len);
    if (len > first_len) {
        memcpy(usb_rx_buffer, &data[first_len], len - first_len);
    }
    __atomic_store_n(&usb_rx_write_pos, write_pos + len, __ATOMIC_RELEASE);
    return len;
}

/**
 * @brief 从 USB 接收环形缓冲区读取数据。
 * @param[out] data 接收数据的目标缓冲区。
 * @param[in] len 期望读取的最大字节数。
 * @return 实际读取字节数；无数据或参数无效时返回 0。
 */
uint32_t USB_VPC_RxRead(uint8_t *data, uint32_t len)
{
    uint32_t read_pos;
    uint32_t write_pos;
    uint32_t available;
    uint32_t read_len;
    uint32_t offset;
    uint32_t first_len;

    if (data == NULL || len == 0U) {
        return 0U;
    }

    read_pos = __atomic_load_n(&usb_rx_read_pos, __ATOMIC_RELAXED);
    write_pos = __atomic_load_n(&usb_rx_write_pos, __ATOMIC_ACQUIRE);
    available = write_pos - read_pos;
    read_len = len < available ? len : available;
    if (read_len == 0U) {
        return 0U;
    }

    offset = read_pos % USB_RX_BUFFER_MAX_SIZE;
    first_len = USB_RX_BUFFER_MAX_SIZE - offset;
    if (first_len > read_len) {
        first_len = read_len;
    }
    memcpy(data, &usb_rx_buffer[offset], first_len);
    if (read_len > first_len) {
        memcpy(&data[first_len], usb_rx_buffer, read_len - first_len);
    }
    __atomic_store_n(&usb_rx_read_pos, read_pos + read_len, __ATOMIC_RELEASE);
    return read_len;
}
