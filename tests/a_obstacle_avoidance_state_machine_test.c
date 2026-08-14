#include "user.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

StateMachine A_obstacle_avoidance_state_machine;
float oa_current_set_target_x;
volatile float radar_get_axis[2];
volatile float radar_get_angle;

typedef enum
{
    ACTION_NONE,
    ACTION_TURN,
    ACTION_CURRENT_ANGLE,
    ACTION_AXIS_Y,
    ACTION_AXIS_X
} RecordedAction;

Task *task_pump_spray;
Task *task_pump_stop;
Task *task_change_state_delay;
Task *task_wheel_stop_delay;
Task *task_wheel_stop_condition1;
StateMachine *machine_delay;
uint16_t next_state_id_delay;
uint8_t pump_spray_time;
uint8_t enable_fix_angle;

static Task task_storage;
static RecordedAction recorded_action;
static float recorded_speed;
static float recorded_value;
static uint32_t recorded_delay;
static uint32_t schedule_count;
static const float test_pi = 3.14159265358979323846f;

static void ResetRecord(void)
{
    recorded_action = ACTION_NONE;
    recorded_speed = 0.0f;
    recorded_value = 0.0f;
    recorded_delay = 0U;
    schedule_count = 0U;
    machine_delay = NULL;
    next_state_id_delay = STATE_MACHINE_NO_STATE;
    task_change_state_delay = &task_storage;
}

static void AssertAction(RecordedAction action,
                         float speed,
                         float value,
                         uint16_t next_state,
                         uint32_t delay)
{
    assert(recorded_action == action);
    assert(fabsf(recorded_speed - speed) < 0.0001f);
    assert(fabsf(recorded_value - value) < 0.0001f);
    assert(machine_delay == &A_obstacle_avoidance_state_machine);
    assert(next_state_id_delay == next_state);
    if (recorded_delay != delay)
    {
        fprintf(stderr,
                "next=%u actual_delay=%lu expected_delay=%lu value=%.9f\n",
                (unsigned int)next_state,
                (unsigned long)recorded_delay,
                (unsigned long)delay,
                (double)recorded_value);
    }
    assert(recorded_delay == delay);
    assert(schedule_count == 1U);
}

static void Enter(uint16_t state_id)
{
    ResetRecord();
    A_Obstacle_Avoidance_State_Change(state_id, STATE_ENTER);
}

void Wheel_Turn_WithRadar_Angle(float speed, float angle_rad)
{
    recorded_action = ACTION_TURN;
    recorded_speed = speed;
    recorded_value = angle_rad;
}

void Wheel_Forward_WithRadar_CurrentAngle(float speed, float route_m)
{
    recorded_action = ACTION_CURRENT_ANGLE;
    recorded_speed = speed;
    recorded_value = route_m;
}

void Wheel_Forward_WithRadar_AxisY(float speed, float route_m)
{
    recorded_action = ACTION_AXIS_Y;
    recorded_speed = speed;
    recorded_value = route_m;
}

void Wheel_Forward_WithRadar_AxisX(float speed, float route_m)
{
    recorded_action = ACTION_AXIS_X;
    recorded_speed = speed;
    recorded_value = route_m;
}

void Task_SetRunTick_Delay(Task *task, uint32_t delay)
{
    assert(task == &task_storage);
    recorded_delay = delay;
}

void Task_Awake(Task *task)
{
    assert(task == &task_storage);
    schedule_count++;
}

static void TestPathStartingFromOne(void)
{
    const float turn_angle = 40.0f * test_pi / 180.0f;

    Enter(1U);
    AssertAction(ACTION_TURN, 0.3f, -turn_angle, 3U, 1200U);

    Enter(3U);
    AssertAction(ACTION_CURRENT_ANGLE, 0.3f, 0.7f, 0U, 2500U);

    Enter(0U);
    AssertAction(ACTION_TURN, 0.3f, turn_angle, 5U, 1200U);

    radar_get_axis[1] = 0.45f;
    Enter(5U);
    AssertAction(ACTION_AXIS_Y, 0.3f, -0.45f, 6U, 3000U);

    radar_get_angle = -0.25f;
    Enter(6U);
    AssertAction(ACTION_TURN, 0.3f, 0.25f, 7U, 1200U);

    radar_get_axis[0] = 1.2f;
    oa_current_set_target_x = 1.8f;
    Enter(7U);
    AssertAction(ACTION_AXIS_X, 0.3f, 0.6f,
                 STATE_MACHINE_NO_STATE, 2000U);
}

static void TestPathStartingFromZero(void)
{
    const float turn_angle = 40.0f * test_pi / 180.0f;

    Enter(0U);
    AssertAction(ACTION_TURN, 0.3f, turn_angle, 2U, 1200U);

    Enter(2U);
    AssertAction(ACTION_CURRENT_ANGLE, 0.3f, 0.7f, 1U, 2500U);

    Enter(1U);
    AssertAction(ACTION_TURN, 0.3f, -turn_angle, 5U, 1200U);

    radar_get_axis[1] = -0.3f;
    Enter(5U);
    AssertAction(ACTION_AXIS_Y, 0.3f, 0.3f, 6U, 3000U);

    radar_get_angle = 0.4f;
    Enter(6U);
    AssertAction(ACTION_TURN, 0.3f, -0.4f, 7U, 1200U);

    radar_get_axis[0] = 2.0f;
    oa_current_set_target_x = 1.4f;
    Enter(7U);
    AssertAction(ACTION_AXIS_X, 0.3f, -0.6f,
                 STATE_MACHINE_NO_STATE, 2000U);
}

static void TestExitDoesNothing(void)
{
    ResetRecord();
    A_Obstacle_Avoidance_State_Change(1U, STATE_EXIT);
    assert(recorded_action == ACTION_NONE);
    assert(schedule_count == 0U);
}

int main(void)
{
    TestPathStartingFromOne();
    TestPathStartingFromZero();
    TestExitDoesNothing();
    return 0;
}
