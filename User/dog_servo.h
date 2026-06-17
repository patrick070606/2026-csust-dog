#ifndef __DOG_SERVO_H
#define __DOG_SERVO_H

#include "dog_servo_config.h"
#include <stdint.h>

uint16_t DogServo_AngleToPosition(DogServoId_t servo, float angle_deg);
void DogServo_SetAngle(DogServoId_t servo, float angle_deg, uint16_t time_ms);
void DogServo_SetAngles(const float angle_deg[DOG_SERVO_COUNT], uint16_t time_ms);
void DogServo_AllCenter(uint16_t time_ms);
void DogServo_TestAllCenterAndSmallMove(void);
void DogServo_TestOneByOneSmallMove(void);

#endif
