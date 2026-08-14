#include "a_obstacle_avoidance_logic.h"
#include "state_machine.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static void TestPathStartingFromOne(void)
{
    uint8_t detour_started = 0U;

    assert(AObstacleAvoidance_NextState(1U, &detour_started) == 3U);
    assert(detour_started == 1U);
    assert(AObstacleAvoidance_NextState(3U, &detour_started) == 0U);
    assert(AObstacleAvoidance_NextState(0U, &detour_started) == 5U);
    assert(AObstacleAvoidance_NextState(5U, &detour_started) == 6U);
    assert(AObstacleAvoidance_NextState(6U, &detour_started) == 7U);
    assert(AObstacleAvoidance_NextState(7U, &detour_started) ==
           STATE_MACHINE_NO_STATE);
    assert(detour_started == 0U);
}

static void TestPathStartingFromZero(void)
{
    uint8_t detour_started = 0U;

    assert(AObstacleAvoidance_NextState(0U, &detour_started) == 2U);
    assert(detour_started == 1U);
    assert(AObstacleAvoidance_NextState(2U, &detour_started) == 1U);
    assert(AObstacleAvoidance_NextState(1U, &detour_started) == 5U);
    assert(AObstacleAvoidance_NextState(5U, &detour_started) == 6U);
    assert(AObstacleAvoidance_NextState(6U, &detour_started) == 7U);
    assert(AObstacleAvoidance_NextState(7U, &detour_started) ==
           STATE_MACHINE_NO_STATE);
    assert(detour_started == 0U);
}

static void TestInvalidStateAndFlag(void)
{
    uint8_t detour_started = 0U;

    assert(AObstacleAvoidance_NextState(0U, NULL) ==
           STATE_MACHINE_NO_STATE);
    assert(AObstacleAvoidance_NextState(1U, NULL) ==
           STATE_MACHINE_NO_STATE);
    assert(AObstacleAvoidance_NextState(4U, &detour_started) ==
           STATE_MACHINE_NO_STATE);
    assert(detour_started == 0U);
}

static void TestDistanceDelay(void)
{
    assert(AObstacleAvoidance_DistanceDelayMs(0.6f, 0.3f) == 2000U);
    assert(AObstacleAvoidance_DistanceDelayMs(-0.6f, 0.3f) == 2000U);
    assert(AObstacleAvoidance_DistanceDelayMs(0.0f, 0.3f) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(1.0f, 0.0f) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(1.0f, -0.3f) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(NAN, 0.3f) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(1.0f, INFINITY) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(INFINITY, 0.3f) == 0U);
    assert(AObstacleAvoidance_DistanceDelayMs(1.0e30f, 0.3f) ==
           UINT32_MAX / 2U);
}

int main(void)
{
    TestPathStartingFromOne();
    TestPathStartingFromZero();
    TestInvalidStateAndFlag();
    TestDistanceDelay();
    return 0;
}
