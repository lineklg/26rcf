#include "main.h"
#include "time_us.h"

#include <assert.h>
#include <stdint.h>

static CoreDebug_Type core_debug_instance;
static DWT_Type dwt_instance;

CoreDebug_Type *CoreDebug = &core_debug_instance;
DWT_Type *DWT = &dwt_instance;
uint32_t SystemCoreClock = 550000000U;

static void test_init_and_fractional_cycles(void)
{
    CoreDebug->DEMCR = 0U;
    DWT->CTRL = 0U;
    DWT->CYCCNT = 123U;

    TimeUs_Init();

    assert((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U);
    assert((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    assert(DWT->CYCCNT == 0U);
    assert(TimeUs_Get() == 0U);

    DWT->CYCCNT = 275U;
    assert(TimeUs_Get() == 0U);
    DWT->CYCCNT = 550U;
    assert(TimeUs_Get() == 1U);
    DWT->CYCCNT = 825U;
    assert(TimeUs_Get() == 1U);
    DWT->CYCCNT = 1100U;
    assert(TimeUs_Get() == 2U);
}

static void test_cycle_counter_wrap(void)
{
    const uint64_t before_wrap_cycles = (uint64_t)UINT32_MAX - 100ULL;
    uint64_t expected_us;

    TimeUs_Init();
    DWT->CYCCNT = (uint32_t)before_wrap_cycles;
    expected_us = before_wrap_cycles * 1000000ULL / SystemCoreClock;
    assert(TimeUs_Get() == expected_us);

    DWT->CYCCNT = 200U;
    expected_us = (before_wrap_cycles + 301ULL) * 1000000ULL /
                  SystemCoreClock;
    assert(TimeUs_Get() == expected_us);
}

int main(void)
{
    test_init_and_fractional_cycles();
    test_cycle_counter_wrap();
    return 0;
}
