#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    // 髋关节：480-220 = 260
    // 小腿关节：720-520=200
    [DOG_SERVO_LF_KNEE] = {1, -1, 500, 10, 300, 800}, //500 - 200 = 300
    [DOG_SERVO_LF_HIP]  = {2, -1, 500, -65, 200, 700}, // 435+260 = 695
    [DOG_SERVO_RF_KNEE] = {3, 1, 500, 15, 200, 715},
    [DOG_SERVO_RF_HIP]  = {4, 1, 500, -15, 215, 800},
    [DOG_SERVO_LB_KNEE] = {5, -1, 500, 40, 300, 800},
    [DOG_SERVO_LB_HIP]  = {6, -1, 500, 60, 200, 940},
    [DOG_SERVO_RB_KNEE] = {7, 1, 500, 15, 200, 715},
    [DOG_SERVO_RB_HIP]  = {8, 1, 500, 30, 130, 800},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}
