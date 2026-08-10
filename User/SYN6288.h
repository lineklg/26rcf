#ifndef _SYN6288_H_
#define _SYN6288_H_

#include "main.h"

/* HZdata is a UTF-8, NUL-terminated string. */
HAL_StatusTypeDef SYN_FrameInfo(UART_HandleTypeDef *huart,
                                uint8_t Music,
                                const char *HZdata);

#endif // !_SYN6288_H_
