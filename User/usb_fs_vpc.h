#ifndef _USB_FS_VPC_H
#define _USB_FS_VPC_H

#include "usbd_cdc_if.h"

#define USB_RX_BUFFER_MAX_SIZE              512

extern uint8_t usb_rx_buffer[USB_RX_BUFFER_MAX_SIZE];

HAL_StatusTypeDef USB_VPC_Send(const uint8_t *data, uint32_t len);

#endif // !_USB_FS_H
