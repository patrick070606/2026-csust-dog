#include "dog_servo_config.h"

/*
 * 默认映射：每条腿按 KNEE -> HIP 顺序排列。
 * 机械装配后，根据实际腿部位置、转向和限位修改本表。
 */
static const DogServoConfig_t s_dog_servo_configs[DOG_SERVO_COUNT] = {
    [DOG_SERVO_LF_KNEE] = {1, -1, 500, -18, 295, 800}, //500 - 200 = 300
    [DOG_SERVO_LF_HIP]  = {2, -1, 500, -5, 115, 815}, // 435+260 = 695
    [DOG_SERVO_RF_KNEE] = {3, 1, 500, -4, 100, 685},
    [DOG_SERVO_RF_HIP]  = {4, 1, 500, -27, 150, 850},
    [DOG_SERVO_LB_KNEE] = {5, -1, 500, 25, 355, 1000},
    [DOG_SERVO_LB_HIP]  = {6, -1, 500, -22, 330, 860},
    [DOG_SERVO_RB_KNEE] = {7, 1, 500, -13, 30, 675},
    [DOG_SERVO_RB_HIP]  = {8, 1, 500, -11, 70, 625},
};

const DogServoConfig_t *DogServoConfig_Get(DogServoId_t servo)
{
    if ((uint8_t)servo >= DOG_SERVO_COUNT)
    {
        return 0;
    }

    return &s_dog_servo_configs[servo];
}
