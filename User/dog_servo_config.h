#ifndef __DOG_SERVO_CONFIG_H
#define __DOG_SERVO_CONFIG_H

#include <stdint.h>

typedef enum
{
    DOG_SERVO_LF_KNEE = 0,
    DOG_SERVO_LF_HIP,
    DOG_SERVO_RF_KNEE,
    DOG_SERVO_RF_HIP,
    DOG_SERVO_LB_KNEE,
    DOG_SERVO_LB_HIP,
    DOG_SERVO_RB_KNEE,
    DOG_SERVO_RB_HIP,
    DOG_SERVO_COUNT  
} DogServoId_t;

typedef struct
{
    uint8_t id; // 舵机 ID，对应总线舵机的 ID，例如 1~8
    int8_t reverse; // 方向修正，取 +1 或 -1
    int16_t center_pos; // 中心位置，对应机械装配后舵机在自然站立姿势下的位置，单位是总线舵机的位置单位，例如 500
    int16_t offset_pos; // 偏移位置，用于微调舵机的初始位置，单位是总线舵机的位置单位，例如 0
    uint16_t min_pos; // 最小位置，单位是总线舵机的位置单位，例如 100
    uint16_t max_pos; // 最大位置，单位是总线舵机的位置单位，例如 900
} DogServoConfig_t;

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo);

#endif
