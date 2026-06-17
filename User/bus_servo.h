#ifndef __BUS_SERVO_H
#define __BUS_SERVO_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint8_t id;
    uint16_t position;
} BusServo_t;

void BusServo_SetPosition(uint8_t id, uint16_t position, uint16_t time_ms);
void BusServo_SetPositions(BusServo_t *servos, uint8_t num, uint16_t time_ms);
void BusServo_TestID7(void);

#endif
