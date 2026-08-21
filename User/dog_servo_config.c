#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    // 髋关节：480-220 = 260
    // 小腿关节：720-520=200
    [DOG_SERVO_LF_KNEE] = {4, -1, 500, 22, 320, 900},
    [DOG_SERVO_LF_HIP]  = {3, -1, 500, 30, 100, 860}, 
    [DOG_SERVO_RF_KNEE] = {2, 1, 500, 25, 100, 715},
    [DOG_SERVO_RF_HIP]  = {1, 1, 500, 12, 180, 900},
    [DOG_SERVO_LB_KNEE] = {8, -1, 500, -35, 290, 900},
    [DOG_SERVO_LB_HIP]  = {7, -1, 500, -1, 100, 895},
    [DOG_SERVO_RB_KNEE] = {6, 1, 500, 35, 100, 715},
    [DOG_SERVO_RB_HIP]  = {5, 1, 500, 10, 115, 900},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}