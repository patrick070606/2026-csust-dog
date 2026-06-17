from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")

    required_fragments = [
        "#define DOG_TASK_VISION_COLOR_STOP_TEST_ENABLE 0U",
        "#define DOG_TASK_VISION_COLOR_STOP_MS          10000U",
        "return (uint8_t)((command == IMAGE_COMMAND_TURN_LEFT) ||",
        "/* Keep lost-line 9999 non-latching so tracking can recover on the next valid frame. */",
    ]

    missing = [fragment for fragment in required_fragments if fragment not in source]
    if missing:
        raise AssertionError("missing expected vision color-stop fragments: " + repr(missing))


if __name__ == "__main__":
    main()
