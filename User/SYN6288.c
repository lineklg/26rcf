#include "SYN6288.h"
#include <string.h>

#define SYN6288_MAX_UNICODE_BYTES 256U

/* Convert UTF-8 to the UTF-16 big-endian byte stream required by SYN6288. */
static HAL_StatusTypeDef SYN_Utf8ToUnicodeBE(const char *src,
                                              uint8_t *dst,
                                              uint16_t dst_size,
                                              uint16_t *dst_len)
{
  size_t i = 0U;
  uint16_t out = 0U;

  while (src[i] != '\0')
  {
    uint32_t codepoint;
    uint8_t c = (uint8_t)src[i++];

    if (c < 0x80U)
    {
      codepoint = c;
    }
    else if ((c & 0xE0U) == 0xC0U &&
             src[i] != '\0' &&
             ((uint8_t)src[i] & 0xC0U) == 0x80U)
    {
      codepoint = ((uint32_t)(c & 0x1FU) << 6) |
                  (uint32_t)((uint8_t)src[i++] & 0x3FU);
      if (codepoint < 0x80U)
      {
        return HAL_ERROR;
      }
    }
    else if ((c & 0xF0U) == 0xE0U &&
             src[i] != '\0' && src[i + 1U] != '\0' &&
             ((uint8_t)src[i] & 0xC0U) == 0x80U &&
             ((uint8_t)src[i + 1U] & 0xC0U) == 0x80U)
    {
      uint8_t c1 = (uint8_t)src[i++];
      uint8_t c2 = (uint8_t)src[i++];
      codepoint = ((uint32_t)(c & 0x0FU) << 12) |
                  ((uint32_t)(c1 & 0x3FU) << 6) |
                  (uint32_t)(c2 & 0x3FU);
      if (codepoint < 0x800U || (codepoint >= 0xD800U && codepoint <= 0xDFFFU))
      {
        return HAL_ERROR;
      }
    }
    else if ((c & 0xF8U) == 0xF0U &&
             src[i] != '\0' && src[i + 1U] != '\0' && src[i + 2U] != '\0' &&
             ((uint8_t)src[i] & 0xC0U) == 0x80U &&
             ((uint8_t)src[i + 1U] & 0xC0U) == 0x80U &&
             ((uint8_t)src[i + 2U] & 0xC0U) == 0x80U)
    {
      uint8_t c1 = (uint8_t)src[i++];
      uint8_t c2 = (uint8_t)src[i++];
      uint8_t c3 = (uint8_t)src[i++];
      codepoint = ((uint32_t)(c & 0x07U) << 18) |
                  ((uint32_t)(c1 & 0x3FU) << 12) |
                  ((uint32_t)(c2 & 0x3FU) << 6) |
                  (uint32_t)(c3 & 0x3FU);
      if (codepoint < 0x10000U || codepoint > 0x10FFFFU)
      {
        return HAL_ERROR;
      }
    }
    else
    {
      return HAL_ERROR;
    }

    if (codepoint <= 0xFFFFU)
    {
      if ((uint32_t)out + 2U > dst_size)
      {
        return HAL_ERROR;
      }
      dst[out++] = (uint8_t)(codepoint >> 8);
      dst[out++] = (uint8_t)codepoint;
    }
    else
    {
      uint32_t surrogate = codepoint - 0x10000U;
      uint16_t high = (uint16_t)(0xD800U | (surrogate >> 10));
      uint16_t low = (uint16_t)(0xDC00U | (surrogate & 0x3FFU));

      if ((uint32_t)out + 4U > dst_size)
      {
        return HAL_ERROR;
      }
      dst[out++] = (uint8_t)(high >> 8);
      dst[out++] = (uint8_t)high;
      dst[out++] = (uint8_t)(low >> 8);
      dst[out++] = (uint8_t)low;
    }
  }

  *dst_len = out;
  return HAL_OK;
}

HAL_StatusTypeDef SYN_FrameInfo(UART_HandleTypeDef *huart,
                                uint8_t Music,
                                const char *HZdata)
{
  uint8_t unicode_text[SYN6288_MAX_UNICODE_BYTES];
  uint8_t frame[5U + SYN6288_MAX_UNICODE_BYTES + 1U];
  uint16_t text_len;
  uint16_t frame_len;
  uint16_t i;
  uint8_t ecc = 0U;

  if (huart == NULL || HZdata == NULL)
  {
    return HAL_ERROR;
  }

  if (SYN_Utf8ToUnicodeBE(HZdata,
                          unicode_text,
                          sizeof(unicode_text),
                          &text_len) != HAL_OK)
  {
    return HAL_ERROR;
  }

  frame[0] = 0xFD;
  frame[1] = (uint8_t)((text_len + 3U) >> 8);
  frame[2] = (uint8_t)(text_len + 3U);
  frame[3] = 0x01;
  frame[4] = (uint8_t)(0x03U | ((Music & 0x0FU) << 4));
  memcpy(&frame[5], unicode_text, text_len);

  frame_len = (uint16_t)(5U + text_len);
  for (i = 0U; i < frame_len; i++)
  {
    ecc ^= frame[i];
  }
  frame[frame_len] = ecc;

  return HAL_UART_Transmit(huart, frame, (uint16_t)(frame_len + 1U), 10000U);
}
