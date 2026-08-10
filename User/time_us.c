#include "time_us.h"
#include "main.h"

static uint32_t time_last_cycle;
static uint32_t time_remainder;
static uint64_t time_elapsed_us;

void TimeUs_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    time_last_cycle = 0U;
    time_remainder = 0U;
    time_elapsed_us = 0U;
}

uint64_t TimeUs_Get(void)
{
    uint32_t current_cycle;
    uint32_t delta_cycle;
    uint64_t scaled_cycle;

    if (SystemCoreClock == 0U) {
        return time_elapsed_us;
    }

    current_cycle = DWT->CYCCNT;
    delta_cycle = current_cycle - time_last_cycle;
    time_last_cycle = current_cycle;

    scaled_cycle = (uint64_t)delta_cycle * 1000000ULL + time_remainder;
    time_elapsed_us += scaled_cycle / SystemCoreClock;
    time_remainder = (uint32_t)(scaled_cycle % SystemCoreClock);
    return time_elapsed_us;
}
