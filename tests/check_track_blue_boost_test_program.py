from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "User" / "track_blue_boost_test_program.c"
HEADER = ROOT / "User" / "track_blue_boost_test_program.h"
MAIN = ROOT / "Core" / "Src" / "main_track_blue_boost_test.c"
DOG_GAIT = ROOT / "User" / "dog_gait.c"
CMAKELISTS = ROOT / "CMakeLists.txt"


def main() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    main_source = MAIN.read_text(encoding="utf-8")
    dog_gait = DOG_GAIT.read_text(encoding="utf-8")
    cmake = CMAKELISTS.read_text(encoding="utf-8")

    required_source = [
        "#define TRACK_BLUE_TEST_CENTER_MOVE_MS        5000U",
        "#define TRACK_BLUE_TEST_CENTER_WAIT_MS        6500U",
        "#define TRACK_BLUE_TEST_STAND_MOVE_MS         2000U",
        "#define TRACK_BLUE_TEST_STAND_WAIT_MS         2500U",
        "#define TRACK_BLUE_TEST_STEP_H_MM             20.0f",
        "#define TRACK_BLUE_TEST_LEFT_FORWARD_R_MM     20.0f",
        "#define TRACK_BLUE_TEST_RIGHT_FORWARD_R_MM    15.0f",
        "#define TRACK_BLUE_TEST_MAX_STEER_MM          8.0f",
        "#define TRACK_BLUE_TEST_STEER_GAIN            0.08f",
        "#define TRACK_BLUE_TEST_BOOST_STEP_H_MM             40.0f",
        "#define TRACK_BLUE_TEST_BOOST_LEFT_FORWARD_R_MM     40.0f",
        "#define TRACK_BLUE_TEST_BOOST_RIGHT_FORWARD_R_MM    40.0f",
        "DogServo_AllCenter(TRACK_BLUE_TEST_CENTER_MOVE_MS);",
        "DogGait_GotoStandPose(TRACK_BLUE_TEST_STAND_MOVE_MS);",
        "ImageCommand_Init();",
        "if (command == IMAGE_COMMAND_PLATFORM)",
        "TrackBlueBoostTest_ApplyMotion(TRACK_BLUE_TEST_MOTION_STOP);",
        "s_track_boost = 1U;",
        "DogGait_SetTrackParams(step_h,",
        "DogGait_UpdateTrot(TRACK_BLUE_TEST_GAIT_MOVE_MS);",
    ]

    missing = [fragment for fragment in required_source if fragment not in source]
    if missing:
        raise AssertionError("track blue boost test source check failed: " + repr(missing))

    required_other = [
        "void TrackBlueBoostTest_Init(void);",
        "void TrackBlueBoostTest_Run(void);",
        '#include "track_blue_boost_test_program.h"',
        "TrackBlueBoostTest_Init();",
        "TrackBlueBoostTest_Run();",
        "add_executable(DogRobotTrackBlueBoostTest)",
        "Core/Src/main_track_blue_boost_test.c",
        "User/image_command.c",
        "User/track_blue_boost_test_program.c",
    ]
    combined = "\n".join([header, main_source, cmake])
    missing_other = [fragment for fragment in required_other if fragment not in combined]
    if missing_other:
        raise AssertionError("track blue boost test target check failed: " + repr(missing_other))

    track_params_start = dog_gait.find("void DogGait_SetTrackParams")
    track_params_end = dog_gait.find("void DogGait_SetTurnLeftParams", track_params_start)
    track_params = dog_gait[track_params_start:track_params_end]
    if "DogGait_ClampFloat(step_height_mm, 0.0f, 40.0f)" not in track_params:
        raise AssertionError("track step height should allow 40 mm")

    if "stair_platform_test_program.c" in cmake[
        cmake.find("add_executable(DogRobotTrackBlueBoostTest)") :
    ]:
        raise AssertionError("track blue boost test target should not include stair test program")


if __name__ == "__main__":
    main()
