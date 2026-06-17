from pathlib import Path


DOG_TASK_C = Path("User/dog_task.c")


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")

    required_fragments = [
        "#define DOG_TASK_TRACK_LEFT_FORWARD_R_MM   20.0f",
        "#define DOG_TASK_TRACK_RIGHT_FORWARD_R_MM  20.0f",
        "DOG_TASK_TRACK_LEFT_FORWARD_R_MM",
        "DOG_TASK_TRACK_RIGHT_FORWARD_R_MM",
    ]

    missing = [fragment for fragment in required_fragments if fragment not in source]
    if missing:
        raise AssertionError("track right base check failed: " + repr(missing))


if __name__ == "__main__":
    main()
