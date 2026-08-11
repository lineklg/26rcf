#include "servo_position.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t main_start_calls;
static uint32_t main_stop_calls;
static uint32_t complementary_start_calls;
static uint32_t complementary_stop_calls;

/**
 * @brief 清空 HAL 测试桩调用次数。
 */
static void reset_hal_calls(void)
{
    main_start_calls = 0U;
    main_stop_calls = 0U;
    complementary_start_calls = 0U;
    complementary_stop_calls = 0U;
}

/**
 * @brief 在测试定时器中写入指定通道的比较值。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @param[in] compare 待写入的比较值。
 */
void Test_HAL_TIM_SetCompare(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t compare
)
{
    assert(htim != NULL);
    assert(htim->Instance != NULL);
    assert(channel <= TIM_CHANNEL_4);
    htim->Instance->CCR[channel / 4U] = compare;
}

/**
 * @brief 记录普通 PWM 启动调用。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return 始终返回 HAL_OK。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    main_start_calls++;
    return HAL_OK;
}

/**
 * @brief 记录普通 PWM 停止调用。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return 始终返回 HAL_OK。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    main_stop_calls++;
    return HAL_OK;
}

/**
 * @brief 记录互补 PWM 启动调用。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return 始终返回 HAL_OK。
 */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    complementary_start_calls++;
    return HAL_OK;
}

/**
 * @brief 记录互补 PWM 停止调用。
 * @param[in,out] htim 定时器句柄。
 * @param[in] channel 定时器通道。
 * @return 始终返回 HAL_OK。
 */
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    complementary_stop_calls++;
    return HAL_OK;
}

/**
 * @brief 验证普通输出使用普通 HAL PWM 接口。
 */
static void test_main_output_uses_main_hal_api(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_2,
               SERVO_POSITION_OUTPUT_MAIN,
               1375U,
               4125U,
               6875U
           ) == HAL_OK);
    assert(main_start_calls == 1U);
    assert(complementary_start_calls == 0U);
    assert(ServoPosition_DeInit(&servo) == HAL_OK);
    assert(main_stop_calls == 1U);
    assert(complementary_stop_calls == 0U);
}

/**
 * @brief 验证互补输出使用 HAL 互补 PWM 接口。
 */
static void test_complementary_output_uses_complementary_hal_api(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_1,
               SERVO_POSITION_OUTPUT_COMPLEMENTARY,
               1375U,
               4125U,
               6875U
           ) == HAL_OK);
    assert(main_start_calls == 0U);
    assert(complementary_start_calls == 1U);
    assert(ServoPosition_DeInit(&servo) == HAL_OK);
    assert(main_stop_calls == 0U);
    assert(complementary_stop_calls == 1U);
}

/**
 * @brief 验证初始化拒绝未知输出类型。
 */
static void test_rejects_invalid_output_type(void)
{
    TIM_TypeDef timer = {.ARR = 54999U};
    TIM_HandleTypeDef htim = {.Instance = &timer};
    ServoPosition servo = {0};

    reset_hal_calls();
    assert(ServoPosition_Init(
               &servo,
               &htim,
               TIM_CHANNEL_1,
               (ServoPositionOutput)99,
               1375U,
               4125U,
               6875U
           ) == HAL_ERROR);
    assert(main_start_calls == 0U);
    assert(complementary_start_calls == 0U);
}

/**
 * @brief 验证 0.5 ms 至 2.5 ms 的端点和中位比较值。
 */
static void test_half_to_two_and_half_millisecond_range(void)
{
    assert(ServoPosition_GetCompare(54999U, -1.0f, 1375U, 4125U, 6875U) == 1375U);
    assert(ServoPosition_GetCompare(54999U, 0.0f, 1375U, 4125U, 6875U) == 4125U);
    assert(ServoPosition_GetCompare(54999U, 1.0f, 1375U, 4125U, 6875U) == 6875U);
}

/**
 * @brief 运行 ServoPosition 主机测试。
 * @return 全部断言通过时返回 0。
 */
int main(void)
{
    test_main_output_uses_main_hal_api();
    test_complementary_output_uses_complementary_hal_api();
    test_rejects_invalid_output_type();
    test_half_to_two_and_half_millisecond_range();
    return 0;
}
