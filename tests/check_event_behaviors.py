from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"
DESKTOP_MAIN = Path("C:/Users/lei/Desktop/main.py")


def main() -> None:
    dog_task = DOG_TASK_C.read_text(encoding="utf-8")
    desktop_main = DESKTOP_MAIN.read_text(encoding="utf-8")

    required_dog_task = [
        "#define DOG_TASK_VISION_COLOR_STOP_TEST_ENABLE 0U",
        "#define DOG_TASK_THROW_TRACK_DELAY_MS  10000U",
        "DOG_TASK_EVENT_THROW_TRACK_DELAY",
        "DogTask_BeginThrowTrackDelay(command, now_ms);",
        "DogTask_BeginThrowForward(command, now_ms);",
        "DogTask_BeginPlatformTrackBoost();",
        "/* DogTask_BeginStairSequence(now_ms); */",
        "if (command == IMAGE_COMMAND_BROWN)",
        "direction = THROW_SERVO_DIRECTION_CCW;",
        "s_purple_throw_delay_used = 1U;",
        "s_brown_throw_delay_used = 1U;",
    ]
    required_desktop_main = [
        "if target_house_color == \"purple\":",
        "return \"R\"",
        "return \"L\"",
        "return \"4\", \"P\"",
        "return \"5\", \"B\"",
        "return \"3\", \"U\"",
    ]

    missing = [fragment for fragment in required_dog_task if fragment not in dog_task]
    missing += [fragment for fragment in required_desktop_main if fragment not in desktop_main]
    if missing:
        raise AssertionError("event behavior check failed: " + repr(missing))


if __name__ == "__main__":
    main()
