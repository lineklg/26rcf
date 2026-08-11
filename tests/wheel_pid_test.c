#include "wheel_pid.h"
#include "task.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TEST_FLOAT_TOLERANCE 0.0001f

static uint32_t fake_tick;
static Motor motors[WHEEL_PID_COUNT];
static DRV8870_Motor drivers[WHEEL_PID_COUNT];
static float feedback[WHEEL_PID_COUNT];
static float output[WHEEL_PID_COUNT];
static uint32_t output_count[WHEEL_PID_COUNT];

/**
 * @brief 返回主机测试使用的毫秒时基。
 * @return 当前模拟 tick。
 */
static uint32_t fake_get_tick(void)
{
    return fake_tick;
}

/**
 * @brief 返回指定电机对应的模拟平滑速度反馈。
 * @param[in,out] motor 待读取反馈的电机。
 * @return 对应轮位的模拟速度反馈。
 */
float Motor_CalcSpeed_Smooth(Motor *motor)
{
    size_t i;

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        if (motor == &motors[i]) {
            return feedback[i];
        }
    }

    assert(0);
    return 0.0f;
}

/**
 * @brief 记录指定驱动器的占空比输出。
 * @param[in,out] motor 接收输出的驱动器。
 * @param[in] percent 带符号占空比。
 * @return 无。
 */
void DRV8870_SetDutyPercent(DRV8870_Motor *motor, float percent)
{
    size_t i;

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        if (motor == &drivers[i]) {
            output[i] = percent;
            output_count[i]++;
            return;
        }
    }

    assert(0);
}

/**
 * @brief 重置四轮 PID 测试夹具。
 * @return 无。
 */
static void reset_fixture(void)
{
    fake_tick = 0U;
    memset(motors, 0, sizeof(motors));
    memset(drivers, 0, sizeof(drivers));
    memset(feedback, 0, sizeof(feedback));
    memset(output, 0, sizeof(output));
    memset(output_count, 0, sizeof(output_count));
    Task_Init(fake_get_tick);
    WheelPID_Init(motors, drivers);
}

/**
 * @brief 运行一次到期的 PID 周期并推进模拟时钟。
 * @return 无。
 */
static void run_next_period(void)
{
    Task_Update();
    fake_tick += WHEEL_PID_PERIOD_MS;
}

/**
 * @brief 断言两个浮点数在测试容差内相等。
 * @param[in] actual 实际值。
 * @param[in] expected 期望值。
 * @return 无。
 */
static void assert_float_close(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_FLOAT_TOLERANCE);
}

/**
 * @brief 断言四路输出均被写入一次且与期望一致。
 * @param[in] expected 四路期望输出。
 * @return 无。
 */
static void assert_outputs(const float expected[WHEEL_PID_COUNT])
{
    size_t i;

    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        assert(output_count[i] == 1U);
        assert_float_close(output[i], expected[i]);
    }
}

/**
 * @brief 验证直行目标会等量写入四轮。
 * @return 无。
 */
static void test_forward_sets_equal_targets(void)
{
    const float expected[WHEEL_PID_COUNT] = {0.5f, 0.5f, 0.5f, 0.5f};

    reset_fixture();
    WheelPID_Forward(0.5f);
    run_next_period();

    assert_outputs(expected);
}

/**
 * @brief 验证转向目标令 A/C 反转且 B/D 正转。
 * @return 无。
 */
static void test_turn_sets_opposite_targets(void)
{
    const float expected[WHEEL_PID_COUNT] = {-0.4f, 0.4f, -0.4f, 0.4f};

    reset_fixture();
    WheelPID_Turn(0.4f);
    run_next_period();

    assert_outputs(expected);
}

/**
 * @brief 验证各轮输出使用各自的速度反馈。
 * @return 无。
 */
static void test_each_wheel_uses_its_feedback(void)
{
    const float expected[WHEEL_PID_COUNT] = {0.8f, 0.7f, 0.6f, 0.5f};

    reset_fixture();
    feedback[0] = 0.0f;
    feedback[1] = 0.1f;
    feedback[2] = 0.2f;
    feedback[3] = 0.3f;
    WheelPID_Forward(0.8f);
    run_next_period();

    assert_outputs(expected);
}

/**
 * @brief 验证四轮 PID 输出限制在 -1.0 到 1.0。
 * @return 无。
 */
static void test_output_is_limited(void)
{
    const float positive[WHEEL_PID_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    const float negative[WHEEL_PID_COUNT] = {-1.0f, -1.0f, -1.0f, -1.0f};

    reset_fixture();
    WheelPID_Forward(2.0f);
    run_next_period();
    assert_outputs(positive);

    reset_fixture();
    WheelPID_Forward(-2.0f);
    run_next_period();
    assert_outputs(negative);
}

/**
 * @brief 验证运行中重复设置目标不会清除积分历史。
 * @return 无。
 */
static void test_running_command_keeps_integral_history(void)
{
    reset_fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_Forward(1.0f);
    run_next_period();
    assert_float_close(output[0], 0.02f);

    WheelPID_Forward(1.0f);
    run_next_period();
    assert_float_close(output[0], 0.04f);
}

/**
 * @brief 验证停止后任务休眠且重新启动会清除积分历史。
 * @return 无。
 */
static void test_stop_sleeps_task_and_restart_clears_history(void)
{
    uint32_t writes_after_stop;

    reset_fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_Forward(1.0f);
    run_next_period();

    WheelPID_Stop();
    writes_after_stop = output_count[0];
    run_next_period();
    assert(output_count[0] == writes_after_stop);

    WheelPID_Forward(0.0f);
    run_next_period();
    assert_float_close(output[0], 0.0f);
}

/**
 * @brief 验证无效轮号不会改变任一有效轮子的 PID 参数。
 * @return 无。
 */
static void test_invalid_wheel_index_does_not_change_pid(void)
{
    const float expected[WHEEL_PID_COUNT] = {0.5f, 0.5f, 0.5f, 0.5f};

    reset_fixture();
    WheelPID_SetK(WHEEL_PID_COUNT, 0.0f, 0.0f, 0.0f);
    WheelPID_Forward(0.5f);
    run_next_period();
    assert_outputs(expected);
}

/**
 * @brief 运行四轮 PID 主机测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    test_forward_sets_equal_targets();
    test_turn_sets_opposite_targets();
    test_each_wheel_uses_its_feedback();
    test_output_is_limited();
    test_running_command_keeps_integral_history();
    test_stop_sleeps_task_and_restart_clears_history();
    test_invalid_wheel_index_does_not_change_pid();
    return 0;
}
