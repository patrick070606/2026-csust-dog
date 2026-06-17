from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    checks = {
        ROOT / "vision" / "main.py": "UART_BAUD = 115200",
        Path("D:/STM32/tmp/main.py"): "UART_BAUD = 115200",
        ROOT / "Core" / "Src" / "usart.c": "huart2.Init.BaudRate = 115200;",
    }

    missing = []
    for path, fragment in checks.items():
        text = path.read_text(encoding="utf-8")
        if fragment not in text:
            missing.append(f"{path}: {fragment}")

    if missing:
        raise AssertionError("missing expected baud-rate settings: " + repr(missing))


if __name__ == "__main__":
    main()
