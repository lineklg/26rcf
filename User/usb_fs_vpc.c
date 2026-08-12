#include "usb_fs_vpc.h"

uint8_t usb_rx_buffer[USB_RX_BUFFER_MAX_SIZE];

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