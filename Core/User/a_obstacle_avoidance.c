#include "a_obstacle_avoidance_logic.h"
#include "user.h"

extern StateMachine A_obstacle_avoidance_state_machine;
extern float oa_current_set_target_x;

static uint8_t detour_started;

static void AObstacleAvoidance_ChangeWithDelay(uint16_t state_id,
                                               uint32_t delay)
{
    machine_delay = &A_obstacle_avoidance_state_machine;
    next_state_id_delay = state_id;
    Task_SetRunTick_Delay(task_change_state_delay, delay);
    Task_Awake(task_change_state_delay);
}

void A_Obstacle_Avoidance_State_Change(uint16_t state_id,
                                       uint8_t enter_or_exit)
{
    if (enter_or_exit != STATE_ENTER)
    {
        return;
    }

    switch (state_id)
    {
    case 0U:
        Wheel_Turn_WithRadar_Angle(0.3f,
                                   40.0f * 3.14159265358979323846f / 180.0f);
        AObstacleAvoidance_ChangeWithDelay(
            AObstacleAvoidance_NextState(state_id, &detour_started),
            1200U);
        break;
    case 1U:
        Wheel_Turn_WithRadar_Angle(0.3f,
                                   -40.0f * 3.14159265358979323846f / 180.0f);
        AObstacleAvoidance_ChangeWithDelay(
            AObstacleAvoidance_NextState(state_id, &detour_started),
            1200U);
        break;
    case 2U:
    case 3U:
        Wheel_Forward_WithRadar_CurrentAngle(0.3f, 0.7f);
        AObstacleAvoidance_ChangeWithDelay(
            AObstacleAvoidance_NextState(state_id, &detour_started),
            2500U);
        break;
    case 5U:
        Wheel_Forward_WithRadar_AxisY(0.3f, -radar_get_axis[1]);
        AObstacleAvoidance_ChangeWithDelay(6U, 3000U);
        break;
    case 6U:
        Wheel_Turn_WithRadar_Angle(0.3f, -radar_get_angle);
        AObstacleAvoidance_ChangeWithDelay(7U, 1200U);
        break;
    case 7U:
    {
        float x_delta = oa_current_set_target_x - radar_get_axis[0];
        uint16_t next_state = AObstacleAvoidance_NextState(
            state_id,
            &detour_started);

        Wheel_Forward_WithRadar_AxisX(0.3f, x_delta);
        AObstacleAvoidance_ChangeWithDelay(
            next_state,
            AObstacleAvoidance_DistanceDelayMs(x_delta, 0.3f));
        break;
    }
    default:
        break;
    }
}
