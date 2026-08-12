#include "wheel_pid.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t fake_tick;
static Motor test_motors[WHEEL_PID_COUNT];
static DRV8870_Motor test_drivers[WHEEL_PID_COUNT];
static float duty_output[WHEEL_PID_COUNT];
static uint32_t duty_write_count[WHEEL_PID_COUNT];
static float sent_target[WHEEL_PID_COUNT];
static uint32_t send_count;

UART_HandleTypeDef huart1;

/**
 * @brief 返回主机测试使用的毫秒时间戳。
 * @return 当前模拟时间戳。
 */
static uint32_t Fake_GetTick(void)
{
    return fake_tick;
}

/**
 * @brief 返回测试电机的固定零速度反馈。
 * @param[in,out] motor 待测电机对象。
 * @return 固定返回 0 m/s。
 */
float Motor_CalcSpeed_Smooth(Motor *motor)
{
    assert(motor >= test_motors);
    assert(motor < test_motors + WHEEL_PID_COUNT);
    return 0.0f;
}

/**
 * @brief 记录指定测试驱动的占空比输出。
 * @param[in,out] driver 待记录的驱动对象。
 * @param[in] percent 输出占空比。
 * @return 无。
 */
void DRV8870_SetDutyPercent(DRV8870_Motor *driver, float percent)
{
    ptrdiff_t index = driver - test_drivers;

    assert(index >= 0);
    assert(index < (ptrdiff_t)WHEEL_PID_COUNT);
    duty_output[index] = percent;
    duty_write_count[index]++;
}

/**
 * @brief 捕获速度环通过 VOFA 发送的四轮目标值。
 * @param[in] huart 测试 UART 句柄。
 * @param[in] data 待发送的 VOFA 通道数据。
 * @param[in] channel_num 通道数量。
 * @return 固定返回 HAL_OK。
 */
HAL_StatusTypeDef VOFA_JustFloat_UART_Send(
    UART_HandleTypeDef *huart,
    const float *data,
    uint8_t channel_num
)
{
    uint8_t i;

    assert(huart == &huart1);
    assert(channel_num == WHEEL_PID_COUNT * 3U);
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        sent_target[i] = data[i * 3U];
    }
    send_count++;
    return HAL_OK;
}

/**
 * @brief 断言两个浮点数在测试精度内相等。
 * @param[in] actual 实际值。
 * @param[in] expected 期望值。
 * @return 无。
 */
static void Assert_Close(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

/**
 * @brief 重置四轮速度环测试夹具。
 * @return 无。
 */
static void Reset_Fixture(void)
{
    uint8_t i;

    fake_tick = 0U;
    send_count = 0U;
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        test_motors[i] = (Motor){0};
        test_drivers[i] = (DRV8870_Motor){0};
        duty_output[i] = 0.0f;
        duty_write_count[i] = 0U;
        sent_target[i] = 0.0f;
    }
    Task_Init(Fake_GetTick);
    WheelPID_Init(test_motors, test_drivers);
}

/**
 * @brief 在当前模拟时刻运行一次任务调度。
 * @return 无。
 */
static void Run_Current_Tick(void)
{
    Task_Update();
}

/**
 * @brief 推进一个 PID 周期并运行一次任务调度。
 * @return 无。
 */
static void Run_Next_Period(void)
{
    fake_tick += WHEEL_PID_PERIOD_MS;
    Task_Update();
}

/**
 * @brief 断言最近发送的 A、B、C、D 四轮目标值。
 * @param[in] a A 轮期望目标。
 * @param[in] b B 轮期望目标。
 * @param[in] c C 轮期望目标。
 * @param[in] d D 轮期望目标。
 * @return 无。
 */
static void Assert_Targets(float a, float b, float c, float d)
{
    Assert_Close(sent_target[0], a);
    Assert_Close(sent_target[1], b);
    Assert_Close(sent_target[2], c);
    Assert_Close(sent_target[3], d);
}

/** @brief 验证四轮接口分别更新四个目标并自动启动速度环。 */
static void Test_SetSpeeds_Updates_All_Wheels_And_Starts(void)
{
    Reset_Fixture();
    WheelPID_SetSpeeds(0.1f, 0.2f, -0.3f, -0.4f);
    Run_Current_Tick();
    assert(send_count == 1U);
    Assert_Targets(0.1f, 0.2f, -0.3f, -0.4f);
}

/** @brief 验证单轮接口只更新指定轮子的目标。 */
static void Test_SetSpeed_Only_Updates_Selected_Wheel(void)
{
    Reset_Fixture();
    WheelPID_SetSpeeds(0.1f, 0.2f, 0.3f, 0.4f);
    Run_Current_Tick();
    WheelPID_SetSpeed(2U, -0.6f);
    Run_Next_Period();
    assert(send_count == 2U);
    Assert_Targets(0.1f, 0.2f, -0.6f, 0.4f);
}

/** @brief 验证无效轮号不修改目标且不会启动速度环。 */
static void Test_Invalid_Index_Does_Not_Start_Or_Modify(void)
{
    Reset_Fixture();
    WheelPID_SetSpeed(WHEEL_PID_COUNT, 0.8f);
    Run_Current_Tick();
    assert(send_count == 0U);

    WheelPID_SetSpeeds(0.1f, 0.2f, 0.3f, 0.4f);
    Run_Current_Tick();
    WheelPID_SetSpeed(WHEEL_PID_COUNT, 0.8f);
    Run_Next_Period();
    Assert_Targets(0.1f, 0.2f, 0.3f, 0.4f);
}

/** @brief 验证运行中修改单轮目标不会清除 PID 积分历史。 */
static void Test_Running_Update_Keeps_PID_History(void)
{
    Reset_Fixture();
    WheelPID_SetK(0U, 0.0f, 1.0f, 0.0f);
    WheelPID_SetSpeeds(0.1f, 0.0f, 0.0f, 0.0f);
    Run_Current_Tick();
    Assert_Close(duty_output[0], 0.58325f);

    WheelPID_SetSpeed(0U, 0.2f);
    Run_Next_Period();
    Assert_Close(duty_output[0], 0.6685f);
    Assert_Targets(0.2f, 0.0f, 0.0f, 0.0f);
}

/** @brief 验证现有直行、转向和停止接口行为保持不变。 */
static void Test_Existing_Commands_Do_Not_Regress(void)
{
    uint8_t i;
    uint32_t writes_after_stop[WHEEL_PID_COUNT];

    Reset_Fixture();
    WheelPID_Forward(0.5f);
    Run_Current_Tick();
    Assert_Targets(0.4925f, 0.5f, 0.4925f, 0.5f);

    WheelPID_Turn(0.4f);
    Run_Next_Period();
    Assert_Targets(-0.4f, 0.4f, -0.4f, 0.4f);

    WheelPID_Stop();
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        writes_after_stop[i] = duty_write_count[i];
    }
    Run_Next_Period();
    for (i = 0U; i < WHEEL_PID_COUNT; i++) {
        assert(duty_write_count[i] == writes_after_stop[i]);
    }
}

/**
 * @brief 运行四轮独立目标速度测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    Test_SetSpeeds_Updates_All_Wheels_And_Starts();
    Test_SetSpeed_Only_Updates_Selected_Wheel();
    Test_Invalid_Index_Does_Not_Start_Or_Modify();
    Test_Running_Update_Keeps_PID_History();
    Test_Existing_Commands_Do_Not_Regress();
    return 0;
}
