#include "main.h"
#include "encoder.h"
#ifndef ENCODER_TEST_ONLY
#include "motor.h"
#endif

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static uint32_t fake_tick;
#ifndef ENCODER_TEST_ONLY
static uint64_t fake_time_us;
#endif
static GPIO_TypeDef port_a;
static GPIO_TypeDef port_b;
static GPIO_TypeDef interrupt_ports[10];

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return ((port != NULL) && ((port->levels & pin) != 0U))
               ? GPIO_PIN_SET
               : GPIO_PIN_RESET;
}

#ifndef ENCODER_TEST_ONLY
static uint64_t fake_time_callback(void)
{
    return fake_time_us;
}
#endif

static GPIO_Pin make_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return (GPIO_Pin){(GPIO_Port)port, pin};
}

static void test_usi_decoding_and_independent_timestamps(void)
{
    Encoder_USI forward;
    Encoder_USI second;
    const GPIO_Pin forward_a = make_pin(&port_a, 0x0001U);
    const GPIO_Pin forward_b = make_pin(&port_a, 0x0002U);
    const GPIO_Pin second_a = make_pin(&port_b, 0x0001U);
    const GPIO_Pin second_b = make_pin(&port_b, 0x0002U);

    port_a.levels = 0U;
    port_b.levels = 0U;
    fake_tick = 0U;
    Encoder_USI_Create(&forward, forward_a, forward_b);
    Encoder_USI_Create(&second, second_a, second_b);

    fake_tick = 3U;
    port_a.levels = 0x0001U;
    port_b.levels = 0x0001U;
    Encoder_USI_Update(&forward);
    Encoder_USI_Update(&second);
    fake_tick = 6U;
    port_a.levels = 0x0003U;
    port_b.levels = 0x0003U;
    Encoder_USI_Update(&forward);
    Encoder_USI_Update(&second);
    fake_tick = 9U;
    port_a.levels = 0x0002U;
    port_b.levels = 0x0002U;
    Encoder_USI_Update(&forward);
    Encoder_USI_Update(&second);
    fake_tick = 12U;
    port_a.levels = 0U;
    port_b.levels = 0U;
    Encoder_USI_Update(&forward);
    Encoder_USI_Update(&second);

    assert(Encoder_GetChange(&forward.base_enc) == 4);
    assert(Encoder_GetChange(&second.base_enc) == 4);

    fake_tick = 15U;
    port_a.levels = 0x0002U;
    Encoder_USI_Update(&forward);
    fake_tick = 18U;
    port_a.levels = 0x0003U;
    Encoder_USI_Update(&forward);
    fake_tick = 21U;
    port_a.levels = 0x0001U;
    Encoder_USI_Update(&forward);
    fake_tick = 24U;
    port_a.levels = 0U;
    Encoder_USI_Update(&forward);
    assert(Encoder_GetChange(&forward.base_enc) == -4);

    fake_tick = 27U;
    port_b.levels = 0x0001U;
    Encoder_USI_Update(&second);
    assert(Encoder_GetChange(&second.base_enc) == 1);
}

static void test_encoder_prescaler(void)
{
    Encoder encoder = {0};

    encoder.counter = 5;
    Encoder_SetPrescaler(&encoder, 1U);
    assert(Encoder_GetChange(&encoder) == 2);
    encoder.counter = 6;
    assert(Encoder_GetChange(&encoder) == 1);
}

static void test_interrupt_dispatch_and_capacity(void)
{
    Encoder encoders[5] = {0};
    GPIO_Pin pins_a[5];
    GPIO_Pin pins_b[5];
    size_t i;

    for (i = 0; i < 5U; i++) {
        pins_a[i] = make_pin(&interrupt_ports[i], (uint16_t)(1U << (i * 2U)));
        pins_b[i] = make_pin(&interrupt_ports[i], (uint16_t)(1U << (i * 2U + 1U)));
    }

    for (i = 0; i < 4U; i++) {
        Encoder_Create_UsePin(&encoders[i], pins_a[i], pins_b[i]);
        interrupt_ports[i].levels = (uint16_t)(pins_a[i].pin | pins_b[i].pin);
        Encoder_GPIO_EXTI_Callback(pins_a[i].pin);
        assert(Encoder_GetChange(&encoders[i]) == 1);
    }

    encoders[4].pin_A = make_pin(&interrupt_ports[8], 0x4000U);
    encoders[4].pin_B = make_pin(&interrupt_ports[9], 0x8000U);
    encoders[4].counter = 123;
    Encoder_Create_UsePin(&encoders[4], pins_a[4], pins_b[4]);
    assert(encoders[4].pin_A.port == (GPIO_Port)&interrupt_ports[8]);
    assert(encoders[4].pin_A.pin == 0x4000U);
    assert(encoders[4].pin_B.port == (GPIO_Port)&interrupt_ports[9]);
    assert(encoders[4].pin_B.pin == 0x8000U);
    assert(encoders[4].counter == 123);
}

#ifndef ENCODER_TEST_ONLY
static void test_motor_calculation(void)
{
    Encoder encoder = {0};
    Motor motor;
    Motor reverse_motor;
    Motor invalid_motor;

    fake_time_us = 0U;
    motor = Motor_Init(&encoder, 2U, 1.0f, fake_time_callback, NULL, NULL, 0U);
    encoder.counter = 2;
    fake_time_us = 1000000U;
    assert(Motor_CalcSpeed(&motor) == 1.0f);
    encoder.counter = 6;
    fake_time_us = 2000000U;
    assert(Motor_CalcSpeed(&motor) == 2.0f);
    assert(motor.route == 3.0f);
    assert(motor.acceleration == 1.0f);
    encoder.counter = 26;
    fake_time_us = 3000000U;
    assert(Motor_CalcSpeed_Smooth(&motor) == 2.0f);
    assert(Motor_CalcSpeed(&motor) == 0.0f);

    reverse_motor = Motor_Init(&encoder, 2U, 1.0f, fake_time_callback, NULL, NULL, 1U);
    encoder.counter = 28;
    fake_time_us = 4000000U;
    assert(Motor_CalcSpeed(&reverse_motor) == -1.0f);
    assert(reverse_motor.route == -1.0f);

    invalid_motor = Motor_Init(NULL, 0U, 1.0f, NULL, NULL, NULL, 0U);
    assert(Motor_CalcSpeed(&invalid_motor) == 0.0f);

    fake_time_us = 5000000U;
    assert(Motor_CalcSpeed(&motor) == 0.0f);
    fake_time_us = 6000000U;
    motor.speed_to_pulse_table[3] = 0.25f;
    Motor_RecordCurrentPulse(&motor, 3U, 0.5f);
    assert(motor.speed_to_pulse_table[3] == 0.5f);
    Motor_RecordCurrentPulse(&motor, 6U, 0.75f);
    assert(motor.speed_to_pulse_table[3] == 0.5f);
}
#endif

int main(void)
{
    test_usi_decoding_and_independent_timestamps();
    test_encoder_prescaler();
    test_interrupt_dispatch_and_capacity();
#ifndef ENCODER_TEST_ONLY
    test_motor_calculation();
#endif
    return 0;
}
