#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <stdint.h>

typedef struct {
    uint16_t levels;
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;

extern CoreDebug_Type *CoreDebug;
extern DWT_Type *DWT;
extern uint32_t SystemCoreClock;

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24U)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL << 0U)

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
uint32_t HAL_GetTick(void);

#endif
