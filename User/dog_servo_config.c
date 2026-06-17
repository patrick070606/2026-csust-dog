#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    [DOG_SERVO_LF_KNEE] = {1, -1, 500, -5, 200, 800},
    [DOG_SERVO_LF_HIP]  = {2, -1, 500, 40, 200, 800},
    [DOG_SERVO_RF_KNEE] = {3, 1, 500, 5, 200, 800},
    [DOG_SERVO_RF_HIP]  = {4, 1, 500, 20, 200, 800},
    [DOG_SERVO_LB_KNEE] = {5, -1, 500, 35, 200, 800},
    [DOG_SERVO_LB_HIP]  = {6, -1, 500, 10, 200, 800},
    [DOG_SERVO_RB_KNEE] = {7, 1, 500, 5, 200, 800},
    [DOG_SERVO_RB_HIP]  = {8, 1, 500, 40, 200, 800},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}
