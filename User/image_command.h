#ifndef __IMAGE_COMMAND_H
#define __IMAGE_COMMAND_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    IMAGE_COMMAND_NONE = 0,
    IMAGE_COMMAND_FORWARD,
    IMAGE_COMMAND_TURN_LEFT,
    IMAGE_COMMAND_TURN_RIGHT,
    IMAGE_COMMAND_STOP,
    IMAGE_COMMAND_PLATFORM,
    IMAGE_COMMAND_PURPLE,
    IMAGE_COMMAND_BROWN,
} ImageCommand_t;

typedef struct
{
    int16_t error;
    uint8_t valid;
} ImageTrack_t;

void ImageCommand_Init(void);
ImageCommand_t ImageCommand_TakeLatest(void);
ImageTrack_t ImageCommand_TakeLatestTrack(void);
uint8_t ImageCommand_DecodeByte(uint8_t data, ImageCommand_t *command);

#endif
