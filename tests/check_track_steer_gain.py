from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")
    expected = "#define DOG_TASK_TRACK_STEER_GAIN      0.08f"

    if expected not in source:
        raise AssertionError(f"missing expected track steer gain: {expected}")


if __name__ == "__main__":
    main()
