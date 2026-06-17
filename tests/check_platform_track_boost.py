from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"
DOG_GAIT_C = ROOT / "User" / "dog_gait.c"
CMAKELISTS = ROOT / "CMakeLists.txt"


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")
    gait_source = DOG_GAIT_C.read_text(encoding="utf-8")
    cmake = CMAKELISTS.read_text(encoding="utf-8")

    required = [
        "#define DOG_TASK_PLATFORM_TRACK_STEP_H_MM          30.0f",
        "#define DOG_TASK_PLATFORM_TRACK_LEFT_FORWARD_R_MM  30.0f",
        "#define DOG_TASK_PLATFORM_TRACK_RIGHT_FORWARD_R_MM 25.0f",
        "static uint8_t s_platform_track_boost;",
        "s_platform_track_boost = 0U;",
        "s_platform_track_boost = 1U;",
        "DogTask_BeginPlatformTrackBoost();",
        "/* DogTask_BeginStairSequence(now_ms); */",
        "DogGait_SetTrackParams(track_step_h,",
    ]

    missing = [fragment for fragment in required if fragment not in source]
    if missing:
        raise AssertionError("platform track boost check failed: " + repr(missing))

    platform_branch_start = source.find("else if (command == IMAGE_COMMAND_PLATFORM)")
    platform_branch_end = source.find("else", platform_branch_start + 1)
    platform_branch = source[platform_branch_start:platform_branch_end]
    if "DogTask_BeginStairSequence(now_ms);" in platform_branch.replace(
        "/* DogTask_BeginStairSequence(now_ms); */", ""
    ):
        raise AssertionError("platform command still enters stair sequence")

    gait_required = [
        "#define DOG_GAIT_STAIR_POSE_ENABLE          0",
        "#if DOG_GAIT_STAIR_POSE_ENABLE",
        "void DogGait_SetStairPoseWithBase",
        "(void)targets;",
    ]
    missing_gait = [fragment for fragment in gait_required if fragment not in gait_source]
    if missing_gait:
        raise AssertionError("dog_gait stair isolation check failed: " + repr(missing_gait))

    if "DOG_GAIT_STAIR_POSE_ENABLE=1" not in cmake:
        raise AssertionError("stair test target does not enable dog_gait stair pose implementation")


if __name__ == "__main__":
    main()
