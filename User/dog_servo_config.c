#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    [DOG_SERVO_LF_KNEE] = {1, -1, 500, -20, 280, 800}, //500 - 200 = 300
    [DOG_SERVO_LF_HIP]  = {2, -1, 500, -15, 200, 820}, // 435+260 = 695
    [DOG_SERVO_RF_KNEE] = {3, 1, 500, -5, 200, 690},
    [DOG_SERVO_RF_HIP]  = {4, 1, 500, 27, 200, 800},
    [DOG_SERVO_LB_KNEE] = {5, -1, 500, -15, 285, 1000},
    [DOG_SERVO_LB_HIP]  = {6, -1, 500, -10, 200, 940},
    [DOG_SERVO_RB_KNEE] = {7, 1, 500, -1, 0, 700},
    [DOG_SERVO_RB_HIP]  = {8, 1, 500, 6, 80, 800},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}
