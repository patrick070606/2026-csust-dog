#include "track_blue_boost_test_program.h"

#include "dog_gait.h"
#include "dog_servo.h"
#include "image_command.h"
#include "main.h"

#define TRACK_BLUE_TEST_GAIT_PERIOD_MS        150U
#define TRACK_BLUE_TEST_GAIT_MOVE_MS          100U
#define TRACK_BLUE_TEST_SERVO_READY_MS        2000U
#define TRACK_BLUE_TEST_CENTER_MOVE_MS        5000U
#define TRACK_BLUE_TEST_CENTER_WAIT_MS        6500U
#define TRACK_BLUE_TEST_STAND_MOVE_MS         2000U
#define TRACK_BLUE_TEST_STAND_WAIT_MS         2500U
#define TRACK_BLUE_TEST_USE_PAYLOAD_GAIT      1U

#define TRACK_BLUE_TEST_DEADBAND              35
#define TRACK_BLUE_TEST_RECOVER_MS            500U
#define TRACK_BLUE_TEST_STEP_H_MM             20.0f
#define TRACK_BLUE_TEST_LEFT_FORWARD_R_MM     20.0f
#define TRACK_BLUE_TEST_RIGHT_FORWARD_R_MM    15.0f
#define TRACK_BLUE_TEST_MAX_STEER_MM          8.0f
#define TRACK_BLUE_TEST_STEER_GAIN            0.08f
#define TRACK_BLUE_TEST_SPEED_FREQ            0.25f

#define TRACK_BLUE_TEST_BOOST_STEP_H_MM             40.0f
#define TRACK_BLUE_TEST_BOOST_LEFT_FORWARD_R_MM     40.0f
#define TRACK_BLUE_TEST_BOOST_RIGHT_FORWARD_R_MM    40.0f

typedef enum
{
    TRACK_BLUE_TEST_MOTION_STOP = 0,
    TRACK_BLUE_TEST_MOTION_FORWARD,
    TRACK_BLUE_TEST_MOTION_TURN_LEFT,
    TRACK_BLUE_TEST_MOTION_TURN_RIGHT,
} TrackBlueTestMotion_t;

static TrackBlueTestMotion_t s_motion = TRACK_BLUE_TEST_MOTION_STOP;
static TrackBlueTestMotion_t s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
static uint32_t s_last_gait_ms;
static uint32_t s_last_track_ms;
static uint8_t s_has_seen_track;
static uint8_t s_track_boost;

static void TrackBlueBoostTest_ApplyMotion(TrackBlueTestMotion_t motion)
{
    if (motion == s_motion)
    {
        return;
    }

    if (motion == TRACK_BLUE_TEST_MOTION_STOP)
    {
        DogGait_AllStand(TRACK_BLUE_TEST_GAIT_MOVE_MS);
    }

    s_motion = motion;
}

static void TrackBlueBoostTest_ApplyTrackError(int16_t error)
{
    float steer = 0.0f;
    float step_h = TRACK_BLUE_TEST_STEP_H_MM;
    float left_forward = TRACK_BLUE_TEST_LEFT_FORWARD_R_MM;
    float right_forward = TRACK_BLUE_TEST_RIGHT_FORWARD_R_MM;

    if (s_track_boost != 0U)
    {
        step_h = TRACK_BLUE_TEST_BOOST_STEP_H_MM;
        left_forward = TRACK_BLUE_TEST_BOOST_LEFT_FORWARD_R_MM;
        right_forward = TRACK_BLUE_TEST_BOOST_RIGHT_FORWARD_R_MM;
    }

    if (error > TRACK_BLUE_TEST_DEADBAND)
    {
        steer = (float)error * TRACK_BLUE_TEST_STEER_GAIN;
        if (steer > TRACK_BLUE_TEST_MAX_STEER_MM)
        {
            steer = TRACK_BLUE_TEST_MAX_STEER_MM;
        }

        s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_TURN_RIGHT;
        DogGait_SetTrackParams(step_h,
                               left_forward,
                               right_forward,
                               steer,
                               TRACK_BLUE_TEST_SPEED_FREQ);
        s_motion = TRACK_BLUE_TEST_MOTION_TURN_RIGHT;
    }
    else if (error < -TRACK_BLUE_TEST_DEADBAND)
    {
        steer = (float)(-error) * TRACK_BLUE_TEST_STEER_GAIN;
        if (steer > TRACK_BLUE_TEST_MAX_STEER_MM)
        {
            steer = TRACK_BLUE_TEST_MAX_STEER_MM;
        }

        s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_TURN_LEFT;
        DogGait_SetTrackParams(step_h,
                               left_forward,
                               right_forward,
                               -steer,
                               TRACK_BLUE_TEST_SPEED_FREQ);
        s_motion = TRACK_BLUE_TEST_MOTION_TURN_LEFT;
    }
    else
    {
        s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
        DogGait_SetTrackParams(step_h,
                               left_forward,
                               right_forward,
                               0.0f,
                               TRACK_BLUE_TEST_SPEED_FREQ);
        s_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
    }
}

void TrackBlueBoostTest_Init(void)
{
    HAL_Delay(TRACK_BLUE_TEST_SERVO_READY_MS);

    DogServo_AllCenter(TRACK_BLUE_TEST_CENTER_MOVE_MS);
    HAL_Delay(TRACK_BLUE_TEST_CENTER_WAIT_MS);

    DogGait_SetLoadMode((TRACK_BLUE_TEST_USE_PAYLOAD_GAIT != 0U) ? DOG_GAIT_LOAD_WITH_PAYLOAD : DOG_GAIT_LOAD_NONE);
    DogGait_Init();
    DogGait_GotoStandPose(TRACK_BLUE_TEST_STAND_MOVE_MS);
    HAL_Delay(TRACK_BLUE_TEST_STAND_WAIT_MS);

    ImageCommand_Init();

    s_last_gait_ms = HAL_GetTick();
    s_last_track_ms = s_last_gait_ms;
    s_has_seen_track = 0U;
    s_track_boost = 0U;
    s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
    s_motion = TRACK_BLUE_TEST_MOTION_STOP;

    DogGait_SetTrackParams(TRACK_BLUE_TEST_STEP_H_MM,
                           TRACK_BLUE_TEST_LEFT_FORWARD_R_MM,
                           TRACK_BLUE_TEST_RIGHT_FORWARD_R_MM,
                           0.0f,
                           TRACK_BLUE_TEST_SPEED_FREQ);
    s_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
}

void TrackBlueBoostTest_Run(void)
{
    uint32_t now_ms = HAL_GetTick();
    ImageCommand_t command = ImageCommand_TakeLatest();
    ImageTrack_t track = ImageCommand_TakeLatestTrack();
    uint32_t track_lost_ms = (uint32_t)(now_ms - s_last_track_ms);

    if (command == IMAGE_COMMAND_PLATFORM)
    {
        s_has_seen_track = 0U;
        s_last_track_ms = now_ms;
        s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
        TrackBlueBoostTest_ApplyMotion(TRACK_BLUE_TEST_MOTION_STOP);
        s_track_boost = 1U;
    }
    else if (command == IMAGE_COMMAND_STOP)
    {
        s_has_seen_track = 0U;
        s_last_track_ms = now_ms;
        s_last_track_recover_motion = TRACK_BLUE_TEST_MOTION_FORWARD;
        TrackBlueBoostTest_ApplyMotion(TRACK_BLUE_TEST_MOTION_STOP);
    }

    if (track.valid != 0U)
    {
        s_has_seen_track = 1U;
        s_last_track_ms = now_ms;
        TrackBlueBoostTest_ApplyTrackError(track.error);
    }
    else if ((s_has_seen_track != 0U) &&
             (track_lost_ms < TRACK_BLUE_TEST_RECOVER_MS))
    {
        s_motion = s_last_track_recover_motion;
    }
    else if ((s_has_seen_track != 0U) &&
             (track_lost_ms >= TRACK_BLUE_TEST_RECOVER_MS))
    {
        TrackBlueBoostTest_ApplyMotion(TRACK_BLUE_TEST_MOTION_STOP);
    }

    if ((s_motion != TRACK_BLUE_TEST_MOTION_STOP) &&
        ((uint32_t)(now_ms - s_last_gait_ms) >= TRACK_BLUE_TEST_GAIT_PERIOD_MS))
    {
        s_last_gait_ms = now_ms;
        DogGait_UpdateTrot(TRACK_BLUE_TEST_GAIT_MOVE_MS);
    }
}
