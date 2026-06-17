from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
THROW_SERVO_C = ROOT / "User" / "throw_servo.c"
DOG_TASK_C = ROOT / "User" / "dog_task.c"


def main() -> None:
    throw_servo = THROW_SERVO_C.read_text(encoding="utf-8")
    dog_task = DOG_TASK_C.read_text(encoding="utf-8")

    required_throw = [
        "#define THROW_SERVO_ROTATE_MS     950U",
        "ThrowServo_SetPulse(THROW_SERVO_CW_PULSE_US);",
        "ThrowServo_SetPulse(THROW_SERVO_CCW_PULSE_US);",
    ]
    required_task = [
        "if (command == IMAGE_COMMAND_BROWN)",
        "direction = THROW_SERVO_DIRECTION_CCW;",
    ]

    missing = [fragment for fragment in required_throw if fragment not in throw_servo]
    missing += [fragment for fragment in required_task if fragment not in dog_task]
    if missing:
        raise AssertionError("throw servo timing check failed: " + repr(missing))


if __name__ == "__main__":
    main()
