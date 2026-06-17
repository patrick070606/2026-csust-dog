from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")

    required_fragments = [
        "/* Positive camera error means the line is to the right; steer right. */",
        "                               steer,",
        "/* Negative camera error means the line is to the left; steer left. */",
        "                               -steer,",
    ]

    missing = [fragment for fragment in required_fragments if fragment not in source]
    if missing:
        raise AssertionError("missing expected track steering fragments: " + repr(missing))


if __name__ == "__main__":
    main()
