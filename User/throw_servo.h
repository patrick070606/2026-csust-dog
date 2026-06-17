#ifndef __THROW_SERVO_H
#define __THROW_SERVO_H

#include <stdint.h>

typedef enum
{
    THROW_SERVO_DIRECTION_CW = 0,
    THROW_SERVO_DIRECTION_CCW,
} ThrowServoDirection_t;

void ThrowServo_Init(void);
void ThrowServo_Stop(void);
void ThrowServo_SetPulse(uint16_t pulse);
void ThrowServo_Start(ThrowServoDirection_t direction);
void ThrowServo_Update(void);
uint8_t ThrowServo_IsBusy(void);

#endif
