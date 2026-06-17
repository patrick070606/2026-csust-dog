from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    checks = {
        ROOT / "vision" / "main.py": [
            "COLOR_STOP_TEST_ENABLE = False",
            "if mission_stage == STAGE_WAIT_FORK_GREEN:",
            "return name == \"green\"",
            "if mission_stage == STAGE_WAIT_HOUSE:",
            "return name == target_house_color",
            "if mission_stage == STAGE_WAIT_BLUE:",
            "return name == \"blue\"",
        ],
        Path("D:/STM32/tmp/main.py"): [
            "COLOR_STOP_TEST_ENABLE = False",
            "if mission_stage == STAGE_WAIT_FORK_GREEN:",
            "return name == \"green\"",
            "if mission_stage == STAGE_WAIT_HOUSE:",
            "return name == target_house_color",
            "if mission_stage == STAGE_WAIT_BLUE:",
            "return name == \"blue\"",
        ],
    }

    missing = []
    for path, fragments in checks.items():
        text = path.read_text(encoding="utf-8")
        for fragment in fragments:
            if fragment not in text:
                missing.append(f"{path}: {fragment}")

    if missing:
        raise AssertionError("ordered color recognition check failed: " + repr(missing))


if __name__ == "__main__":
    main()
