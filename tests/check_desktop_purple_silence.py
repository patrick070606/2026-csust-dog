from pathlib import Path


DESKTOP_MAIN = Path("C:/Users/lei/Desktop/main.py")


def main() -> None:
    source = DESKTOP_MAIN.read_text(encoding="utf-8")

    required_fragments = [
        "PURPLE_SILENCE_MS = 30000",
        "color_silence_until = 0",
        "if color_silence_until != 0:",
        "detected_color = None",
        "color_silence_until = now + PURPLE_SILENCE_MS",
    ]

    missing = [fragment for fragment in required_fragments if fragment not in source]
    if missing:
        raise AssertionError("purple silence check failed: " + repr(missing))


if __name__ == "__main__":
    main()
