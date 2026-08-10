#include "encoder.h"
#include "main.h"
#include <stddef.h>

static const int8_t usi_rot_table[4][4] = {
    {0, -1, 1, 2}, {1, 0, 2, -1},
    {-1, 2, 0, 1}, {2, 1, -1, 0}
};

static Encoder *encoder_table[ENCODER_MAX_COUNT];
static uint16_t encoder_count;

static uint8_t Encoder_IsValidPin(GPIO_Pin pin)
{
    return pin.port != NULL && pin.pin != 0U;
}

static uint8_t Encoder_ReadPin(GPIO_Pin pin)
{
    return HAL_GPIO_ReadPin((GPIO_TypeDef *)pin.port, pin.pin) == GPIO_PIN_SET;
}

static int8_t Encoder_GetDirection(const Encoder *encoder, uint8_t channel)
{
    uint8_t score = channel;
    score += Encoder_ReadPin(encoder->pin_A) + Encoder_ReadPin(encoder->pin_B);
    return (score == 0U || score == 2U) ? -1 : 1;
}

void Encoder_Create_UsePin(Encoder *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B)
{
    if (encoder == NULL || !Encoder_IsValidPin(pin_A) ||
        !Encoder_IsValidPin(pin_B) || encoder_count >= ENCODER_MAX_COUNT) {
        return;
    }

    *encoder = (Encoder){.pin_A = pin_A, .pin_B = pin_B};
    encoder_table[encoder_count++] = encoder;
}

int32_t Encoder_GetChange(Encoder *encoder)
{
    int32_t divisor;
    int32_t delta;

    if (encoder == NULL) {
        return 0;
    }

    encoder->prescaler_counter += encoder->counter - encoder->last_get_counter;
    divisor = (int32_t)encoder->prescaler + 1;
    delta = encoder->prescaler_counter / divisor;
    encoder->prescaler_counter %= divisor;
    encoder->last_get_counter = encoder->counter;
    return delta;
}

void Encoder_SetPrescaler(Encoder *encoder, uint16_t prescaler)
{
    if (encoder != NULL) {
        encoder->prescaler = prescaler;
    }
}

void Encoder_USI_Create(Encoder_USI *encoder, GPIO_Pin pin_A, GPIO_Pin pin_B)
{
    uint8_t a;
    uint8_t b;

    if (encoder == NULL || !Encoder_IsValidPin(pin_A) ||
        !Encoder_IsValidPin(pin_B)) {
        return;
    }

    a = Encoder_ReadPin(pin_A);
    b = Encoder_ReadPin(pin_B);
    *encoder = (Encoder_USI){
        .base_enc = {.pin_A = pin_A, .pin_B = pin_B},
        .pin_A = pin_A,
        .pin_B = pin_B,
        .last_AB = (uint8_t)((a << 1U) | b),
        .last_direction = 1,
        .last_update_timestamp = HAL_GetTick()
    };
}

void Encoder_USI_Update(Encoder_USI *encoder)
{
    uint32_t now;
    uint8_t current_ab;
    int8_t delta;

    if (encoder == NULL || !Encoder_IsValidPin(encoder->pin_A) ||
        !Encoder_IsValidPin(encoder->pin_B)) {
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - encoder->last_update_timestamp) <
        ENCODER_USI_UPDATE_PERIOD) {
        return;
    }
    encoder->last_update_timestamp = now;
    current_ab = (uint8_t)((Encoder_ReadPin(encoder->pin_A) << 1U) |
                           Encoder_ReadPin(encoder->pin_B));
    if (current_ab == encoder->last_AB) {
        return;
    }

    delta = usi_rot_table[encoder->last_AB][current_ab];
    if (delta == 1 || delta == -1) {
        encoder->last_direction = delta;
    } else if (delta == 2) {
        delta = (int8_t)(delta * encoder->last_direction);
    }
    encoder->base_enc.counter += delta;
    encoder->last_AB = current_ab;
}

void Encoder_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    uint16_t i;

    for (i = 0U; i < encoder_count; i++) {
        Encoder *encoder = encoder_table[i];

        if (encoder->pin_A.pin == gpio_pin) {
            encoder->counter += Encoder_GetDirection(encoder, 1U);
            return;
        }
        if (encoder->pin_B.pin == gpio_pin) {
            encoder->counter += Encoder_GetDirection(encoder, 0U);
            return;
        }
    }
}
