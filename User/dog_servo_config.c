#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    // 髋关节：480-220 = 260
    // 小腿关节：720-520=200
    [DOG_SERVO_LF_KNEE] = {1, -1, 500, -10, 310, 800}, //500 - 200 = 300
    [DOG_SERVO_LF_HIP]  = {2, -1, 500, -65, 200, 745}, // 435+260 = 695
    [DOG_SERVO_RF_KNEE] = {3, 1, 500, -20, 200, 630},
    [DOG_SERVO_RF_HIP]  = {4, 1, 500, -21, 189, 800},
    [DOG_SERVO_LB_KNEE] = {5, -1, 500, 0, 320, 800},
    [DOG_SERVO_LB_HIP]  = {6, -1, 500, 18, 200, 933},
    [DOG_SERVO_RB_KNEE] = {7, 1, 500, 35, 200, 730},
    [DOG_SERVO_RB_HIP]  = {8, 1, 500, -40, 40, 800},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}
