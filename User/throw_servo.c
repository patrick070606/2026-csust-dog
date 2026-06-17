#include "throw_servo.h"

#include "tim.h"

#define THROW_SERVO_STOP_PULSE_US 1500U
#define THROW_SERVO_CW_PULSE_US   2000U
#define THROW_SERVO_CCW_PULSE_US  1000U
#define THROW_SERVO_ROTATE_MS     2000U

typedef enum
{
    THROW_SERVO_STATE_IDLE = 0,
    THROW_SERVO_STATE_ROTATING,
} ThrowServoState_t;

static ThrowServoState_t s_state = THROW_SERVO_STATE_IDLE;
static uint32_t s_rotate_start_ms;

void ThrowServo_SetPulse(uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void ThrowServo_Stop(void)
{
    ThrowServo_SetPulse(THROW_SERVO_STOP_PULSE_US);
}

void ThrowServo_Init(void)
{
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    s_state = THROW_SERVO_STATE_IDLE;
    s_rotate_start_ms = HAL_GetTick();
    ThrowServo_Stop();
}

void ThrowServo_Start(ThrowServoDirection_t direction)
{
    if (direction == THROW_SERVO_DIRECTION_CCW)
    {
        ThrowServo_SetPulse(THROW_SERVO_CCW_PULSE_US);
    }
    else
    {
        ThrowServo_SetPulse(THROW_SERVO_CW_PULSE_US);
    }

    s_state = THROW_SERVO_STATE_ROTATING;
    s_rotate_start_ms = HAL_GetTick();
}

void ThrowServo_Update(void)
{
    if (s_state == THROW_SERVO_STATE_ROTATING)
    {
        uint32_t now_ms = HAL_GetTick();

        if ((uint32_t)(now_ms - s_rotate_start_ms) >= THROW_SERVO_ROTATE_MS)
        {
            ThrowServo_Stop();
            s_state = THROW_SERVO_STATE_IDLE;
        }
    }
}

uint8_t ThrowServo_IsBusy(void)
{
    return (uint8_t)(s_state != THROW_SERVO_STATE_IDLE);
}
