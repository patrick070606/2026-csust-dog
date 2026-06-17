from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    checks = {
        ROOT / "User" / "dog_task.c": [
            "#define DOG_TASK_STATUS_INTERVAL_MS    200U",
            "DogTask_SendVisionStatus(\"OK\");",
            "DogTask_SendVisionStatus(\"ST\");",
            "E:%s M:%s",
        ],
        ROOT / "vision" / "main.py": [
            "def process_stm32_rx(now):",
            "print(\"[STM32] %s\" % line)",
        ],
        Path("D:/STM32/tmp/main.py"): [
            "def process_stm32_rx(now):",
            "print(\"[STM32] %s\" % line)",
        ],
    }

    missing = []
    for path, fragments in checks.items():
        text = path.read_text(encoding="utf-8")
        for fragment in fragments:
            if fragment not in text:
                missing.append(f"{path}: {fragment}")

    if missing:
        raise AssertionError("missing expected status feedback fragments: " + repr(missing))


if __name__ == "__main__":
    main()
