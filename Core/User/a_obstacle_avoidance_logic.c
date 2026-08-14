#include "a_obstacle_avoidance_logic.h"
#include "state_machine.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

uint16_t AObstacleAvoidance_NextState(uint16_t state_id,
                                      uint8_t *detour_started)
{
    switch (state_id)
    {
    case 0U:
        if (detour_started == NULL)
        {
            return STATE_MACHINE_NO_STATE;
        }
        if (*detour_started != 0U)
        {
            return 5U;
        }
        *detour_started = 1U;
        return 2U;
    case 1U:
        if (detour_started == NULL)
        {
            return STATE_MACHINE_NO_STATE;
        }
        if (*detour_started != 0U)
        {
            return 5U;
        }
        *detour_started = 1U;
        return 3U;
    case 2U:
        return 1U;
    case 3U:
        return 0U;
    case 5U:
        return 6U;
    case 6U:
        return 7U;
    case 7U:
        if (detour_started != NULL)
        {
            *detour_started = 0U;
        }
        return STATE_MACHINE_NO_STATE;
    default:
        return STATE_MACHINE_NO_STATE;
    }
}

uint32_t AObstacleAvoidance_DistanceDelayMs(float distance_m,
                                            float speed_mps)
{
    float delay_ms;

    if (!isfinite(distance_m) || !isfinite(speed_mps) || speed_mps <= 0.0f)
    {
        return 0U;
    }

    delay_ms = ceilf(fabsf(distance_m) / speed_mps * 1000.0f);
    if (!isfinite(delay_ms) || delay_ms >= (float)(UINT32_MAX / 2U))
    {
        return UINT32_MAX / 2U;
    }
    return (uint32_t)delay_ms;
}
