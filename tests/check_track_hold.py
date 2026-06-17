from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOG_TASK_C = ROOT / "User" / "dog_task.c"


def main() -> None:
    source = DOG_TASK_C.read_text(encoding="utf-8")

    required_fragments = [
        "/* Keep the last track gait for short frame gaps so differential steering is not overwritten. */",
        "s_is_track_correcting = (uint8_t)(s_last_track_recover_motion != DOG_TASK_MOTION_FORWARD);",
    ]

    forbidden_fragments = [
        "track_lost_ms < DOG_TASK_TRACK_RECOVER_MS))\n    {\n        s_is_track_correcting = 0U;\n        DogTask_ApplyMotion(DOG_TASK_MOTION_FORWARD);",
    ]

    missing = [fragment for fragment in required_fragments if fragment not in source]
    present_forbidden = [fragment for fragment in forbidden_fragments if fragment in source]

    if missing or present_forbidden:
        raise AssertionError(
            "track hold check failed: "
            + repr({"missing": missing, "forbidden": present_forbidden})
        )


if __name__ == "__main__":
    main()
