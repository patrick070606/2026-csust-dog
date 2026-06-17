#include "dog_servo.h"
#include "bus_servo.h"

#define DOG_SERVO_POS_PER_DEG      (1000.0f / 240.0f)

static uint16_t DogServo_ClampPosition(int32_t position, uint16_t min_pos, uint16_t max_pos)
{
    if (position < (int32_t)min_pos)
    {
        return min_pos;
    }

    if (position > (int32_t)max_pos)
    {
        return max_pos;
    }

    return (uint16_t)position;
}

static int32_t DogServo_RoundFloat(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

uint16_t DogServo_AngleToPosition(DogServoId_t servo, float angle_deg)
{
    const DogServoConfig_t *config = DogServoConfig_Get(servo);
    float delta_pos;
    int32_t target_pos;

    if (config == 0)
    {
        return 500;
    }

    delta_pos = (float)config->reverse * angle_deg * DOG_SERVO_POS_PER_DEG;
    target_pos = (int32_t)config->center_pos + (int32_t)config->offset_pos + DogServo_RoundFloat(delta_pos);

    return DogServo_ClampPosition(target_pos, config->min_pos, config->max_pos);
}

void DogServo_SetAngle(DogServoId_t servo, float angle_deg, uint16_t time_ms)
{
    const DogServoConfig_t *config = DogServoConfig_Get(servo);

    if (config == 0)
    {
        return;
    }

    BusServo_SetPosition(config->id, DogServo_AngleToPosition(servo, angle_deg), time_ms);
}

void DogServo_SetAngles(const float angle_deg[DOG_SERVO_COUNT], uint16_t time_ms)
{
    BusServo_t targets[DOG_SERVO_COUNT];

    if (angle_deg == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < DOG_SERVO_COUNT; i++)
    {
        const DogServoConfig_t *config = DogServoConfig_Get((DogServoId_t)i);

        if (config == 0)
        {
            return;
        }

        targets[i].id = config->id;
        targets[i].position = DogServo_AngleToPosition((DogServoId_t)i, angle_deg[i]);
    }

    BusServo_SetPositions(targets, DOG_SERVO_COUNT, time_ms);
}

void DogServo_AllCenter(uint16_t time_ms)
{
    static const float center_angles[DOG_SERVO_COUNT] = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };

    DogServo_SetAngles(center_angles, time_ms);
}

void DogServo_TestAllCenterAndSmallMove(void)
{
    float angles[DOG_SERVO_COUNT] = {
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
    };

    DogServo_AllCenter(1000);
    HAL_Delay(2000);

    angles[DOG_SERVO_LF_HIP] = 5.0f;
    angles[DOG_SERVO_LF_KNEE] = -5.0f;
    angles[DOG_SERVO_RF_HIP] = 5.0f;
    angles[DOG_SERVO_RF_KNEE] = -5.0f;
    angles[DOG_SERVO_LB_HIP] = 5.0f;
    angles[DOG_SERVO_LB_KNEE] = -5.0f;
    angles[DOG_SERVO_RB_HIP] = 5.0f;
    angles[DOG_SERVO_RB_KNEE] = -5.0f;
    DogServo_SetAngles(angles, 1000);
    HAL_Delay(2000);

    DogServo_AllCenter(1000);
    HAL_Delay(2000);

    angles[DOG_SERVO_LF_HIP] = -5.0f;
    angles[DOG_SERVO_LF_KNEE] = 5.0f;
    angles[DOG_SERVO_RF_HIP] = -5.0f;
    angles[DOG_SERVO_RF_KNEE] = 5.0f;
    angles[DOG_SERVO_LB_HIP] = -5.0f;
    angles[DOG_SERVO_LB_KNEE] = 5.0f;
    angles[DOG_SERVO_RB_HIP] = -5.0f;
    angles[DOG_SERVO_RB_KNEE] = 5.0f;
    DogServo_SetAngles(angles, 1000);
    HAL_Delay(2000);

    DogServo_AllCenter(1000);
    HAL_Delay(2000);
}

void DogServo_TestOneByOneSmallMove(void)
{
    static const DogServoId_t test_order[DOG_SERVO_COUNT] = {
        DOG_SERVO_LF_KNEE,
        DOG_SERVO_LF_HIP,
        DOG_SERVO_RF_KNEE,
        DOG_SERVO_RF_HIP,
        DOG_SERVO_LB_KNEE,
        DOG_SERVO_LB_HIP,
        DOG_SERVO_RB_KNEE,
        DOG_SERVO_RB_HIP,
    };

    DogServo_AllCenter(1000);
    HAL_Delay(2000);

    for (uint8_t i = 0; i < DOG_SERVO_COUNT; i++)
    {
        DogServoId_t servo = test_order[i];

        DogServo_SetAngle(servo, 5.0f, 800);
        HAL_Delay(1500);

        DogServo_SetAngle(servo, 0.0f, 800);
        HAL_Delay(1500);

        DogServo_SetAngle(servo, -5.0f, 800);
        HAL_Delay(1500);

        DogServo_SetAngle(servo, 0.0f, 800);
        HAL_Delay(1500);

        DogServo_AllCenter(800);
        HAL_Delay(1500);
    }
}
