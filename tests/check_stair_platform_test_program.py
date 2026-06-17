from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEST_C = ROOT / "User" / "stair_platform_test_program.c"
TEST_H = ROOT / "User" / "stair_platform_test_program.h"
TEST_MAIN = ROOT / "Core" / "Src" / "main_stair_platform_test.c"
CMAKE = ROOT / "CMakeLists.txt"


def main() -> None:
    if not TEST_C.exists():
        raise AssertionError("missing User/stair_platform_test_program.c")
    if not TEST_H.exists():
        raise AssertionError("missing User/stair_platform_test_program.h")
    if not TEST_MAIN.exists():
        raise AssertionError("missing Core/Src/main_stair_platform_test.c")

    source = TEST_C.read_text(encoding="utf-8")
    header = TEST_H.read_text(encoding="utf-8")
    test_main = TEST_MAIN.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    required_source = [
        "#define STAIR_TEST_FRONT_CLEARANCE_HEIGHT_MM    55.0f",
        "#define STAIR_TEST_REAR_LIFT_FORWARD_MM         70.0f",
        "#define STAIR_TEST_PLATFORM_HEIGHT_MM           30.0f",
        "#define STAIR_TEST_REAR_PLACE_X_MM              45.0f",
        "#define STAIR_TEST_FRONT_WALK_UPDATES           32U",
        "#define STAIR_TEST_FRONT_TROT_PREPARE_MS        2000U",
        "STAIR_TEST_FRONT_TROT_PREPARE,",
        "STAIR_TEST_FRONT_WALK_8_STEPS,",
        "static uint8_t s_front_walk_updates;",
        "s_front_walk_updates = 0U;",
        "if (s_front_walk_updates >= STAIR_TEST_FRONT_WALK_UPDATES)",
        "void StairPlatformTest_Init(void)",
        "void StairPlatformTest_Run(void)",
        "DogServo_AllCenter(STAIR_TEST_CENTER_MOVE_MS);",
        "DogGait_GotoStandPose(STAIR_TEST_STAND_MOVE_MS);",
        "StairPlatformTest_SetState(STAIR_TEST_ASCEND_SETTLE, now_ms);",
        "DogGait_SetStairPose(s_stair_targets, move_ms);",
        "DogGait_SetStairPoseWithBase(s_stair_targets, base, move_ms);",
        "DogGait_SetTrotParams(STAIR_TEST_PLATFORM_STEP_H_MM,",
        "DogGait_UpdateTrot(STAIR_TEST_GAIT_MOVE_MS);",
    ]
    required_header = [
        "void StairPlatformTest_Init(void);",
        "void StairPlatformTest_Run(void);",
    ]
    required_test_main = [
        "#include \"stair_platform_test_program.h\"",
        "StairPlatformTest_Init();",
        "StairPlatformTest_Run();",
    ]
    required_cmake = [
        "add_executable(DogRobotStairTest)",
        "Core/Src/main_stair_platform_test.c",
        "User/stair_platform_test_program.c",
    ]
    required_gait = [
        "#define DOG_GAIT_STAND_FOOT_X_OFFSET_NO_LOAD_MM 20.0f",
        "#define DOG_GAIT_STAND_FOOT_X_OFFSET_LOAD_MM    20.0f",
        "DOG_GAIT_STAIR_BASE_WALK",
        "DogGait_SetStairPoseWithBase",
    ]

    missing = [fragment for fragment in required_source if fragment not in source]
    missing += [fragment for fragment in required_header if fragment not in header]
    missing += [fragment for fragment in required_test_main if fragment not in test_main]
    missing += [fragment for fragment in required_cmake if fragment not in cmake]
    gait_source = (ROOT / "User" / "dog_gait.c").read_text(encoding="utf-8")
    missing += [fragment for fragment in required_gait if fragment not in gait_source]
    if "Core/Src/main_stair_platform_test.c\n  User/dog_task.c" in cmake:
        missing.append("test target must not use DogTask main logic")
    if "DogTask_" in source or "#include \"dog_task.h\"" in source:
        missing.append("test program must not call DogTask main logic")
    front_walk_setup = source[
        source.find("else if (state == STAIR_TEST_FRONT_WALK_8_STEPS)"):
        source.find("else if (state == STAIR_TEST_ASCEND_BODY_ADVANCE)")
    ]
    front_walk_run = source[
        source.find("if (s_state == STAIR_TEST_FRONT_WALK_8_STEPS)"):
        source.find("if (s_state == STAIR_TEST_PLATFORM_FORWARD)")
    ]
    enum_block = source[
        source.find("typedef enum"):
        source.find("} StairPlatformTestState_t;")
    ]
    if "DogGait_SetTrotParams(STAIR_TEST_PLATFORM_STEP_H_MM," not in front_walk_setup:
        missing.append("front walk must use the same trot setup as platform forward")
    if "DogGait_UpdateTrot(STAIR_TEST_FRONT_TROT_PREPARE_MS);" not in source:
        missing.append("front trot prepare must last 2000ms")
    if "DogGait_UpdateTrot(STAIR_TEST_GAIT_MOVE_MS);" not in front_walk_run:
        missing.append("front walk must use the same trot update as platform forward")
    rear_ascend_block = source[
        source.find("else if (state == STAIR_TEST_ASCEND_BODY_ADVANCE)"):
        source.find("else if (state == STAIR_TEST_DESCEND_SETTLE)")
    ]
    for state in [
        "STAIR_TEST_ASCEND_LB_LIFT",
        "STAIR_TEST_ASCEND_RB_LIFT",
    ]:
        state_pos = rear_ascend_block.find(f"state == {state}")
        next_pos = rear_ascend_block.find("else if", state_pos + 1)
        state_block = rear_ascend_block[state_pos: next_pos if next_pos > 0 else len(rear_ascend_block)]
        if "STAIR_TEST_REAR_LIFT_FORWARD_MM" not in state_block:
            missing.append(f"{state} must use rear lift forward length")
    for state in [
        "STAIR_TEST_ASCEND_BODY_ADVANCE",
        "STAIR_TEST_ASCEND_LB_LIFT",
        "STAIR_TEST_ASCEND_LB_PLACE",
        "STAIR_TEST_ASCEND_RB_LIFT",
        "STAIR_TEST_ASCEND_RB_PLACE",
        "STAIR_TEST_PLATFORM_SETTLE",
    ]:
        state_pos = rear_ascend_block.find(f"state == {state}")
        next_pos = rear_ascend_block.find("else if", state_pos + 1)
        state_block = rear_ascend_block[state_pos: next_pos if next_pos > 0 else len(rear_ascend_block)]
        if "StairPlatformTest_ApplyTargetsWithBase(DOG_GAIT_STAIR_BASE_WALK," not in state_block:
            missing.append(f"{state} must use walk stair base")
    for state in [
        "STAIR_TEST_ASCEND_SETTLE",
        "STAIR_TEST_ASCEND_LF_LIFT",
        "STAIR_TEST_ASCEND_LF_PLACE",
        "STAIR_TEST_ASCEND_RF_LIFT",
        "STAIR_TEST_ASCEND_RF_PLACE",
        "STAIR_TEST_FRONT_TROT_PREPARE",
        "STAIR_TEST_FRONT_WALK_8_STEPS",
        "STAIR_TEST_ASCEND_BODY_ADVANCE",
        "STAIR_TEST_ASCEND_LB_LIFT",
        "STAIR_TEST_ASCEND_LB_PLACE",
        "STAIR_TEST_ASCEND_RB_LIFT",
        "STAIR_TEST_ASCEND_RB_PLACE",
        "STAIR_TEST_PLATFORM_SETTLE",
        "STAIR_TEST_PLATFORM_FORWARD",
        "STAIR_TEST_DESCEND_SETTLE",
        "STAIR_TEST_DESCEND_LF_LIFT",
        "STAIR_TEST_DESCEND_LF_PLACE",
        "STAIR_TEST_DESCEND_RF_LIFT",
        "STAIR_TEST_DESCEND_RF_PLACE",
        "STAIR_TEST_DESCEND_BODY_ADVANCE",
        "STAIR_TEST_DESCEND_LB_LIFT",
        "STAIR_TEST_DESCEND_LB_PLACE",
        "STAIR_TEST_DESCEND_RB_LIFT",
        "STAIR_TEST_DESCEND_RB_PLACE",
        "STAIR_TEST_FINAL_SETTLE",
        "STAIR_TEST_DONE",
    ]:
        line_start = enum_block.find(state)
        line_end = enum_block.find("\n", line_start)
        line = enum_block[line_start:line_end]
        if line_start < 0 or "//" not in line:
            missing.append(f"{state} must have an inline state comment")

    if missing:
        raise AssertionError("stair platform test program check failed: " + repr(missing))


if __name__ == "__main__":
    main()
