#include "behavior.h"
#include "task.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
UART_HandleTypeDef huart2;
GPIO_TypeDef *PUMP_GPIO_Port;
uint16_t PUMP_Pin;
GPIO_TypeDef *BM_A_EA_GPIO_Port;
uint16_t BM_A_EA_Pin;
GPIO_TypeDef *BM_A_EB_GPIO_Port;
uint16_t BM_A_EB_Pin;
GPIO_TypeDef *BM_B_EA_GPIO_Port;
uint16_t BM_B_EA_Pin;
GPIO_TypeDef *BM_B_EB_GPIO_Port;
uint16_t BM_B_EB_Pin;
GPIO_TypeDef *BM_C_EA_GPIO_Port;
uint16_t BM_C_EA_Pin;
GPIO_TypeDef *BM_C_EB_GPIO_Port;
uint16_t BM_C_EB_Pin;
GPIO_TypeDef *BM_D_EA_GPIO_Port;
uint16_t BM_D_EA_Pin;
GPIO_TypeDef *BM_D_EB_GPIO_Port;
uint16_t BM_D_EB_Pin;

volatile float radar_get_axis[2];
volatile float radar_get_angle;
uint8_t enable_fix_angle;
Task *task_wheel_stop_condition1;
Task *task_wheel_stop_delay;

static uint32_t fake_tick;
static uint32_t stop_count;
static float commanded_speed;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

static uint32_t Fake_GetTick(void)
{
    return fake_tick;
}

static void Record_Stop(void)
{
    stop_count++;
}

void WheelPID_Forward(float speed)
{
    commanded_speed = speed;
}

void WheelPID_Turn(float speed)
{
    (void)speed;
}

void WheelPID_Stop(void)
{
    commanded_speed = 0.0f;
}

void WheelPID_Init(Motor motors[WHEEL_PID_COUNT], DRV8870_Motor drivers[WHEEL_PID_COUNT])
{
    (void)motors;
    (void)drivers;
}

HAL_StatusTypeDef ServoPosition_Init(
    ServoPosition *servo,
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    ServoPositionOutput output,
    uint32_t min_compare,
    uint32_t center_compare,
    uint32_t max_compare)
{
    (void)servo;
    (void)htim;
    (void)channel;
    (void)output;
    (void)min_compare;
    (void)center_compare;
    (void)max_compare;
    return HAL_OK;
}

void ServoPosition_SetPosition(ServoPosition *servo, float position)
{
    (void)servo;
    (void)position;
}

void WheelPID_SetTargetAngle(float target_angle)
{
    (void)target_angle;
}

void DRV8870_Brake(DRV8870_Motor *motor)
{
    (void)motor;
}

HAL_StatusTypeDef DRV8870_Motor_Init(
    DRV8870_Motor *motor,
    TIM_HandleTypeDef *htim,
    uint32_t in1_channel,
    uint32_t in2_channel,
    uint8_t reverse)
{
    (void)motor;
    (void)htim;
    (void)in1_channel;
    (void)in2_channel;
    (void)reverse;
    return HAL_OK;
}

void Encoder_Create_UsePin(Encoder *encoder, GPIO_Pin pin_a, GPIO_Pin pin_b)
{
    (void)encoder;
    (void)pin_a;
    (void)pin_b;
}

Motor Motor_Init(
    Encoder *encoder,
    uint16_t cpr,
    float wheel_diameter,
    GetTimeCallback get_tick,
    SetSpeedCallback unused_a,
    void *unused_b,
    uint8_t reverse)
{
    Motor motor = {0};
    (void)encoder;
    (void)cpr;
    (void)wheel_diameter;
    (void)get_tick;
    (void)unused_a;
    (void)unused_b;
    (void)reverse;
    return motor;
}

void PID_Init(void) {}
void PID_SetPeriodMs(uint32_t period_ms) { (void)period_ms; }
uint16_t PID_Create(PID **dest, float kp, float ki, float kd)
{
    (void)kp;
    (void)ki;
    (void)kd;
    *dest = NULL;
    return 0U;
}
void PID_Start(void) {}
void PID_Stop(void) {}
void PID_ResetHistory(PID *pid) { (void)pid; }

HAL_StatusTypeDef SYN_FrameInfo(UART_HandleTypeDef *huart, uint8_t music, const char *data)
{
    (void)huart;
    (void)music;
    (void)data;
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    (void)pin;
    (void)state;
}

static void Reset_Fixture(void)
{
    fake_tick = 0U;
    stop_count = 0U;
    commanded_speed = 0.0f;
    radar_get_axis[0] = 0.0f;
    radar_get_axis[1] = 0.0f;
    radar_get_angle = 0.0f;
    enable_fix_angle = 0U;
    Task_Init(Fake_GetTick);
    Task_Create(&task_wheel_stop_condition1, Record_Stop, TASK_CONDITION);
}

static void Run_Task_Update(void)
{
    fake_tick++;
    Task_Update();
}

static void Test_Speed_Direction_Does_Not_Override_Positive_Route(void)
{
    Reset_Fixture();
    Wheel_Forward_WithRadar_AxisX(-0.3f, 1.0f);
    assert(commanded_speed == -0.3f);

    Run_Task_Update();
    assert(stop_count == 0U);

    radar_get_axis[0] = 1.0f;
    Run_Task_Update();
    assert(stop_count == 1U);
}

static void Test_Negative_Route_Stops_When_X_Decreases(void)
{
    Reset_Fixture();
    radar_get_axis[0] = 2.0f;
    Wheel_Forward_WithRadar_AxisX(0.3f, -0.5f);

    Run_Task_Update();
    assert(stop_count == 0U);

    radar_get_axis[0] = 1.5f;
    Run_Task_Update();
    assert(stop_count == 1U);
}

static void Test_Invalid_Command_Stops_without_Starting(void)
{
    Reset_Fixture();
    Wheel_Forward_WithRadar_AxisX(0.0f, 1.0f);
    assert(commanded_speed == 0.0f);
    assert(stop_count == 0U);

    Wheel_Forward_WithRadar_AxisX(0.3f, NAN);
    assert(commanded_speed == 0.0f);
    assert(stop_count == 0U);

    radar_get_axis[0] = INFINITY;
    Wheel_Forward_WithRadar_AxisX(0.3f, 1.0f);
    assert(commanded_speed == 0.0f);
    assert(stop_count == 0U);
}

int main(void)
{
    Test_Speed_Direction_Does_Not_Override_Positive_Route();
    Test_Negative_Route_Stops_When_X_Decreases();
    Test_Invalid_Command_Stops_without_Starting();
    return 0;
}
